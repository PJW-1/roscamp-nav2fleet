# ---- Nav2 goal(nav_goal 모드) 이동 믹스인 ----
from __future__ import annotations

import math
from collections.abc import Hashable, Sequence

from pinky_fsm.geometry import Pose
from pinky_fsm.nav_goal import NavGoal
from pinky_fsm.results import CANCELED, IDLE, NAV_ABORTED, PATH_NOT_CONNECTED, TIMEOUT, Finish


# ---- NavGoal 시작과 주행 중 여부 판단 ----
class NavGoalMoves:
    def _move_nav(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
                  pose: Sequence[float]) -> list:
        old = self._active
        replace = (isinstance(old, NavGoal) and old.seq > 0 and not self._nav_running
                   and not self._nav_busy(now) and not self._backup_busy(now))
        wait = not replace and self._driving(now)
        cmds = self._end_active(CANCELED, stop=not replace)
        g = NavGoal(goal_id, target_id, tuple(float(v) for v in target), now)
        self._active = g
        self._stopped_since = None
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        return cmds if wait else cmds + self._goals.send(g, now, pose)

    def _driving(self, now: float) -> bool:
        return (self._nav_running or self._nav_busy(now) or self._backup_busy(now)
                or self._goals.busy(now))

    def _motion_ended(self, now: float, speed: tuple[float, float]) -> bool:
        lin, ang = speed
        stopped = abs(lin) < self.p.stopped_speed_mps and abs(ang) < self.p.stopped_turn_rps
        return stopped and not self._driving(now)

    # ---- 앞 goal이 멈출 때까지 기다렸다 넘겨받기 ----
    @staticmethod
    def _handing_over(g) -> bool:
        if isinstance(g, NavGoal):
            return g.seq == 0
        return getattr(g, "handover_since", None) is not None

    def _hand_over(self, now: float, g, pose, speed) -> list:
        since = g.since if isinstance(g, NavGoal) else g.handover_since
        if self._motion_ended(now, speed):
            if isinstance(g, NavGoal):
                return self._goals.send(g, now, pose)
            g.handover_since = None
            if (pose is not None and g.route.project((float(pose[0]), float(pose[1])))[1]
                    > self.p.connect_tolerance_m):
                return self._end_active(PATH_NOT_CONNECTED, stop=True)
            return self._send_rest(g, now, pose)
        if now - since >= self.p.retreat_stop_wait_s:
            return self._end_active(TIMEOUT, stop=True)
        return []

    # ---- NavGoal 피드백, 취소 수락, 결과 처리 ----
    def nav_goal_feedback(self, seq: int, distance_remaining: float) -> None:
        g = self._active
        if isinstance(g, NavGoal) and g.seq == seq:
            g.distance = max(0.0, float(distance_remaining))

    def nav_goal_cancel_accepted(self, seq: int, now: float) -> list:
        self._goals.cancel_accepted(seq, now)
        return []

    def nav_goal_result(self, seq: int, outcome: str, now: float,
                        pose: Sequence[float] | None, lost: bool = False) -> list:
        if not lost:
            self._goals.forget(seq)
        g = self._active
        if not isinstance(g, NavGoal) or seq != g.seq:
            return []
        r = self._recovery
        if r is not None and r.local and r.backup_seq is not None:
            return []
        if outcome == "succeeded":
            if pose is not None and math.dist(pose[:2], g.target[:2]) <= self.p.arrival_tolerance_m:
                self._active = None
                self._rest = IDLE
                self._attempts, self._attempt_origin = 0, None
                return [Finish(g.goal_id, True, "")] + self._end_active("", stop=True)
            g.far_successes += 1
            if g.far_successes <= self.p.far_success_limit:
                return self._goals.send(g, now, pose)
        return self._end_active(NAV_ABORTED, stop=True)
