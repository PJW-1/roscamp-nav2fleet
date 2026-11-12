"""명령의 결과: 이동, 후진, 도킹의 성공과 실패, 거절, 못 보냄, 잃음."""
from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL
from fleet_core.traffic.mission import (
    AT_STATION,
    DOCK,
    HEARTBEAT_LOST,
    LOCALIZING,
    MOVE,
    NOT_AT_TARGET,
    PATH_NOT_CONNECTED,
    RETREAT,
)


class Results:
    def _active(self, robot, goal_id, *kinds):
        """그 로봇의 지금 goal이 이 id이고 이 종류이면 로봇 기록, 아니면 None."""
        m = self._robots.get(robot)
        if m is None or m.goal is None or m.goal.id != goal_id or m.goal.kind not in kinds:
            return None
        return m

    def on_move_result(self, robot, goal_id, success, reason, last_done_seq=0) -> None:
        m = self._robots.get(robot)
        if m is not None and m.move_order is not None and self._old_goal_result(m, goal_id, reason):
            return
        m = self._active(robot, goal_id, MOVE)
        if m is None:
            return
        if m.move_order is not None and self._order_result(m, success, reason, last_done_seq):
            return
        if success:
            self._succeeded(m)
        else:
            self._move_failed(m, reason)

    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        if self._backoff_result(robot, goal_id, "" if success else reason or "RETREAT_FAILED"):
            return
        m = self._active(robot, goal_id, RETREAT)
        if m is None:
            return
        if success:
            m.behind_since = None
            self._succeeded(m)
        else:
            self._move_failed(m, reason or "RETREAT_FAILED")

    def on_dock_result(self, robot, goal_id, success, error_x_cm=0.0, error_y_cm=0.0,
                       error_yaw_deg=0.0) -> None:
        m = self._active(robot, goal_id, DOCK)
        if m is None:
            return
        m.goal = None
        if not success:
            self._dock_failed(m)
            return
        m.attempts = m.dock_tries = 0
        m.phase = AT_STATION
        m.dock_error_m = math.hypot(error_x_cm, error_y_cm) / 100.0
        self._event("INFO", "DOCKED", f"at {m.dest}: x {error_x_cm:.1f} cm, y {error_y_cm:.1f} cm,"
                    f" yaw {error_yaw_deg:.1f} deg", m.name)
        self._finish_dispatch(m, error_x_cm, error_y_cm, error_yaw_deg)

    def on_goal_rejected(self, robot, goal_id) -> None:
        if self._backoff_result(robot, goal_id, "REJECTED"):
            return
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is None or self._update_unclear(m, "the goal was rejected or lost its result"):
            return
        if m.goal.kind == DOCK:
            m.goal = None
            self._dock_failed(m, "REJECTED")
        else:
            self._move_failed(m, "REJECTED")

    def on_goal_not_sent(self, robot, goal_id) -> None:
        if self._backoff_result(robot, goal_id, "NOT_SENT"):
            return
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is None or self._update_lost(m, "the goal was not sent"):
            return
        self._forget_goal(m)
        m.retry_at = self._clock() + self.params.move_retry_wait_s
        if not m.not_sent_warned:
            m.not_sent_warned = True
            self._event("WARN", "GOAL_NOT_SENT", "the action server is not ready, sends it again",
                        m.name)

    def on_goal_lost(self, robot, goal_id) -> None:
        if self._backoff_result(robot, goal_id, "LOST"):
            return
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is not None and not self._update_unclear(m, "the goal was lost"):
            self._forget_goal(m)

    # ---- 끝난 조각 ----

    def _succeeded(self, m) -> None:
        """성공 결과. 보고 위치가 목표에 닿았을 때만 조각을 끝낸다."""
        if self._gap(m, m.goal.end) <= self.params.arrival_tolerance_m:
            self._segment_done(m)
        else:  # 성공이 자세보다 먼저 왔다. 새 보고를 기다린다
            m.arrival_by = self._clock() + self.params.arrival_settle_s
            m.heard_since_success = False

    def _judge_arrival(self, m, now: float) -> None:
        """성공 뒤에 온 보고로 정말 닿았는지 본다."""
        if m.arrival_by is None or m.goal is None:
            return
        if m.heard_since_success and self._gap(m, m.goal.end) <= self.params.arrival_tolerance_m:
            self._segment_done(m)
        elif now >= m.arrival_by:
            self._move_failed(m, NOT_AT_TARGET)

    def _segment_done(self, m) -> None:
        goal = m.goal
        m.goal = None
        m.move_order = None
        m.arrival_by = None
        if m.off_node is None or goal.via != NAV_GOAL:
            m.attempts = 0  # 고정 경로에 붙으려고 Nav2 goal로 지점에 돌아온 것이면 그대로 둔다
        m.not_sent_warned = False
        m.node, m.off_node = goal.end, None
        if m.trip is not None:
            m.trip.done = m.trip.sent = goal.links
        if goal.kind == RETREAT:
            m.retreat_to = ""
            m.behind_since = None  # 실패로 끝났지만 끝에 선 후진(_arrived_anyway)도 기다림을 끝낸다
            self._event("INFO", "RETREAT_DONE", f"stands at {goal.end}", m.name)

    def _forget_goal(self, m) -> None:
        m.goal = None
        m.move_order = None
        m.arrival_by = None
        if m.trip is not None:
            m.trip.sent = m.trip.done

    def _stands_at_end(self, m) -> bool:
        """이동이나 후진 goal의 끝 지점에 로봇이 서 있는가. 방금 들은 위치로만 본다.

        보고가 끊겼거나 로봇이 위치를 잃었으면(LOCALIZING) 남아 있는 위치는 옛것이다.
        막혀 선 로봇(BLOCKED)은 가리지 않는다: 위치가 맞으면 그 지점에 있는 것이다.

        Nav2 goal로 가던 이동은 도착 허용 거리의 절반 안이어야 한다. 로봇이 닿았다고 한 것이
        아니라 보고 하나로 관제가 정하는 것이고, 도착 허용 거리는 로봇이 받은 경로에 붙었다고
        보는 거리와 같다(0.15 m). 그 언저리의 보고를 믿으면 이어 보낸 고정 경로를 로봇이
        거절한다(PATH_NOT_CONNECTED). 못 미쳐 선 로봇에게는 같은 goal을 다시 보내면 된다.
        고정 경로와 후진은 도착 허용 거리 그대로다: 선을 따라 끝까지 온 로봇에게 다시 보내면
        남은 길이 없어 바로 실패한다(pinky_fsm의 _too_short). 끝 점의 옆으로 벗어나 있어도 그렇다.
        """
        goal = m.goal
        heard = (m.available and m.robot_state != LOCALIZING
                 and self._clock() - m.last_seen <= self.params.arrival_settle_s)
        if not (heard and goal is not None and goal.kind in (MOVE, RETREAT)):
            return False
        limit = self.params.arrival_tolerance_m
        # order goal이면 지금 조각의 방식으로 본다(_via)
        return self._gap(m, goal.end) <= (limit / 2 if self._via(m) == NAV_GOAL else limit)

    def _arrived_anyway(self, m, reason: str) -> None:
        """goal은 실패했거나 열린 채 멈췄지만 로봇은 끝 지점에 서 있다. 닿은 것으로 본다.

        실패한 시도로 세면 같은 goal을 다시 보내게 되는데, 끝에 선 로봇은 남은 길이 없어 바로
        실패로 답하고(pinky_fsm의 후진), 세 번 뒤에는 격리된다.
        """
        goal = m.goal
        self._event("WARN", "GOAL_ENDED_AT_TARGET",
                    f"{goal.kind} goal {goal.id} ended {reason}, but the robot stands at "
                    f"{goal.end}", m.name)
        self._segment_done(m)

    def _move_failed(self, m, reason: str) -> None:
        if reason not in (HEARTBEAT_LOST, NOT_AT_TARGET) and self._stands_at_end(m):
            self._arrived_anyway(m, reason)
            return
        self._forget_goal(m)
        if reason == "OBSTACLE_BEHIND" and self._waits_behind(m):
            return  # 뒤에 선 사람이 비키기를 기다린다. 시도로 세지 않는다(block_marks.py)
        if reason == HEARTBEAT_LOST:  # 로봇이 관제 heartbeat를 잃고 스스로 섰다. 시도로 세지 않는다
            m.retry_at = self._clock() + self.params.move_retry_wait_s
            return
        if reason == PATH_NOT_CONNECTED:
            # 관제의 "지점 위"(0.15 m)와 로봇의 "경로에 붙음"(0.15 m)이 가장자리에서 어긋났다.
            # 넓은 구역이면 지점 밖에 선 로봇으로 다뤄, Nav2 goal로 그 지점부터 보낸다
            if self._in_yard(m):
                m.off_node = (m.x, m.y)
                self._drop_trip(m)  # 같은 경로를 다시 보내지 않고 선 자리에서 길을 다시 고른다
        self._goal_failed(m, reason)
