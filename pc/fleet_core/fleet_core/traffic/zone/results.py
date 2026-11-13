# ---- goal 결과 처리 믹스인 ----
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


# ---- 지금 goal과 맞는 로봇 찾기와 이동 결과 ----
class Results:
    def _active(self, robot, goal_id, *kinds):
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

    # ---- 후퇴 결과와 도킹 결과 ----
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

    # ---- goal 거절, 미전송, 잃음 처리 ----
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

    # ---- 도착 확인과 구간 완료 ----
    def _succeeded(self, m) -> None:
        if self._gap(m, m.goal.end) <= self.params.arrival_tolerance_m:
            self._segment_done(m)
        else:
            m.arrival_by = self._clock() + self.params.arrival_settle_s
            m.heard_since_success = False

    def _judge_arrival(self, m, now: float) -> None:
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
            m.attempts = 0
        m.not_sent_warned = False
        m.node, m.off_node = goal.end, None
        if m.trip is not None:
            m.trip.done = m.trip.sent = goal.links
        if goal.kind == RETREAT:
            m.retreat_to = ""
            m.behind_since = None
            self._event("INFO", "RETREAT_DONE", f"stands at {goal.end}", m.name)

    # ---- goal 잊기와 끝 지점에 서 있는지 판정 ----
    def _forget_goal(self, m) -> None:
        m.goal = None
        m.move_order = None
        m.arrival_by = None
        if m.trip is not None:
            m.trip.sent = m.trip.done

    def _stands_at_end(self, m) -> bool:
        goal = m.goal
        heard = (m.available and m.robot_state != LOCALIZING
                 and self._clock() - m.last_seen <= self.params.arrival_settle_s)
        if not (heard and goal is not None and goal.kind in (MOVE, RETREAT)):
            return False
        limit = self.params.arrival_tolerance_m
        return self._gap(m, goal.end) <= (limit / 2 if self._via(m) == NAV_GOAL else limit)

    def _arrived_anyway(self, m, reason: str) -> None:
        goal = m.goal
        self._event("WARN", "GOAL_ENDED_AT_TARGET",
                    f"{goal.kind} goal {goal.id} ended {reason}, but the robot stands at "
                    f"{goal.end}", m.name)
        self._segment_done(m)

    # ---- 이동 실패 사유별 처리 ----
    def _move_failed(self, m, reason: str) -> None:
        if reason not in (HEARTBEAT_LOST, NOT_AT_TARGET) and self._stands_at_end(m):
            self._arrived_anyway(m, reason)
            return
        self._forget_goal(m)
        if reason == "OBSTACLE_BEHIND" and self._waits_behind(m):
            return
        if reason == HEARTBEAT_LOST:
            m.retry_at = self._clock() + self.params.move_retry_wait_s
            return
        if reason == PATH_NOT_CONNECTED:
            if self._in_yard(m):
                m.off_node = (m.x, m.y)
                self._drop_trip(m)
        self._goal_failed(m, reason)
