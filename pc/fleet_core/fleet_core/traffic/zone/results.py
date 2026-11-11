"""명령의 결과: 이동, 후진, 도킹의 성공과 실패, 거절, 못 보냄, 잃음."""
from __future__ import annotations

from fleet_core.traffic.mission import (
    AT_STATION,
    DOCK,
    HEARTBEAT_LOST,
    MOVE,
    NOT_AT_TARGET,
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
        m = self._active(robot, goal_id, RETREAT)
        if m is None:
            return
        if success:
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
            self._goal_failed(m, "DOCK_FAILED")
            return
        m.attempts = 0
        m.phase = AT_STATION
        self._event("INFO", "DOCKED", f"at {m.dest}: x {error_x_cm:.1f} cm, y {error_y_cm:.1f} cm,"
                    f" yaw {error_yaw_deg:.1f} deg", m.name)
        self._finish_dispatch(m, error_x_cm, error_y_cm, error_yaw_deg)

    def on_goal_rejected(self, robot, goal_id) -> None:
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is None or self._update_lost(m, "the goal was rejected"):
            return
        if m.goal.kind == DOCK:
            m.goal = None
            self._goal_failed(m, "REJECTED")
        else:
            self._move_failed(m, "REJECTED")

    def on_goal_not_sent(self, robot, goal_id) -> None:
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
        m = self._active(robot, goal_id, MOVE, RETREAT, DOCK)
        if m is not None and not self._update_lost(m, "the goal was lost"):
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
        m.attempts = 0
        m.not_sent_warned = False
        m.node = goal.end
        if m.trip is not None:
            m.trip.done = m.trip.sent = goal.links
        if goal.kind == RETREAT:
            m.retreat_to = ""
            self._event("INFO", "RETREAT_DONE", f"stands at {goal.end}", m.name)

    def _forget_goal(self, m) -> None:
        m.goal = None
        m.move_order = None
        m.arrival_by = None
        if m.trip is not None:
            m.trip.sent = m.trip.done

    def _move_failed(self, m, reason: str) -> None:
        self._forget_goal(m)
        if reason == HEARTBEAT_LOST:  # 로봇이 관제 heartbeat를 잃고 스스로 섰다. 시도로 세지 않는다
            m.retry_at = self._clock() + self.params.move_retry_wait_s
        else:
            self._goal_failed(m, reason)
