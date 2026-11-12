"""PinkyFsm의 mode nav_goal 부분과 Nav2 서버 사이 바꾸기(계약 0.11-draft 제안). ROS를 모른다.

core.PinkyFsm이 이 클래스를 물려받는다(core.py가 길어 책임별로 나눴다). 여기 메서드는 core의
상태(_active, _goals, FollowPath·BackUp 기록)를 그대로 쓴다.

바꾸기 규칙: NavigateToPose(bt_navigator), FollowPath(controller), BackUp(behavior_server)은 모두
cmd_vel을 낸다. 다른 서버의 움직임으로 바꿀 때는 앞 움직임을 취소하고, 그 결과가 오거나(취소가
받아들여지면 nav_result_grace_s 뒤) 바퀴가 선 뒤에 새 움직임을 보낸다. retreat_stop_wait_s 안에
안 되면 TIMEOUT이다. 물러나기(Retreat)가 쓰던 규칙과 같다. 같은 서버 안의 교체(FollowPath끼리,
NavigateToPose끼리)는 서버가 새 goal로 바꾸므로 서지 않는다(안내 2.1).
"""
from __future__ import annotations

import math
from collections.abc import Hashable, Sequence

from pinky_fsm.geometry import Pose
from pinky_fsm.nav_goal import NavGoal
from pinky_fsm.results import CANCELED, IDLE, NAV_ABORTED, TIMEOUT, Finish


class NavGoalMoves:
    def _move_nav(self, goal_id: Hashable, now: float, target_id: str, target: Pose,
                  pose: Sequence[float]) -> list:
        """mode nav_goal. 도는 것이 NavigateToPose뿐이면 바로 새 goal을 보낸다. FollowPath나
        BackUp이 돌 수 있으면 멈추고 끝나기를 기다린다(tick의 _hand_over)."""
        old = self._active
        replace = (isinstance(old, NavGoal) and old.seq > 0 and not self._nav_running
                   and not self._nav_busy(now) and not self._backup_busy(now))
        wait = not replace and self._driving(now)
        cmds = self._end_active(CANCELED, stop=not replace)
        g = NavGoal(goal_id, target_id, tuple(float(v) for v in target), now)
        self._active = g
        self._stopped_since = None
        # follow_path의 BLOCKED(앞 경로의 물체)는 이 goal의 것이 아니다
        self._blocked, self._clear_since, self._blocked_points = False, None, None
        return cmds if wait else cmds + self._goals.send(g, now, pose)

    def _driving(self, now: float) -> bool:
        """아직 cmd_vel을 낼 수 있는 Nav2 goal(FollowPath, BackUp, NavigateToPose)이 있다."""
        return (self._nav_running or self._nav_busy(now) or self._backup_busy(now)
                or self._goals.busy(now))

    def _motion_ended(self, now: float, speed: tuple[float, float]) -> bool:
        """앞 움직임의 결과가 모두 왔고(또는 취소 답 뒤 여유가 지났고) 바퀴가 섰다."""
        lin, ang = speed
        stopped = abs(lin) < self.p.stopped_speed_mps and abs(ang) < self.p.stopped_turn_rps
        return stopped and not self._driving(now)

    @staticmethod
    def _handing_over(g) -> bool:
        """앞 움직임을 기다리는 이동이다: 아직 안 보낸 NavGoal, handover_since가 있는 이동."""
        if isinstance(g, NavGoal):
            return g.seq == 0
        return getattr(g, "handover_since", None) is not None

    def _hand_over(self, now: float, g, pose, speed) -> list:
        since = g.since if isinstance(g, NavGoal) else g.handover_since
        if self._motion_ended(now, speed):
            if isinstance(g, NavGoal):
                return self._goals.send(g, now, pose)
            g.handover_since = None
            return self._send_rest(g, now, pose)
        if now - since >= self.p.retreat_stop_wait_s:
            return self._end_active(TIMEOUT, stop=True)  # 움직이지 않고 끝낸다
        return []

    def nav_goal_feedback(self, seq: int, distance_remaining: float) -> None:
        g = self._active
        if isinstance(g, NavGoal) and g.seq == seq:
            g.distance = max(0.0, float(distance_remaining))

    def nav_goal_cancel_accepted(self, seq: int, now: float) -> list:
        """NavigateToPose 취소가 받아들여졌다. nav_cancel_accepted와 같은 규칙이다."""
        self._goals.cancel_accepted(seq, now)
        return []

    def nav_goal_result(self, seq: int, outcome: str, now: float,
                        pose: Sequence[float] | None, lost: bool = False) -> list:
        """NavigateToPose 결과(succeeded, aborted, canceled, rejected). lost면 결과 future를
        잃었다: 이동은 실패로 끝내되 goal은 아직 돌 수 있으므로 취소가 받아들여질 때까지 잊지
        않는다(nav_lost와 같다)."""
        if not lost:
            self._goals.forget(seq)
        g = self._active
        if not isinstance(g, NavGoal) or seq != g.seq:
            return []  # 바뀐 goal의 결과(bt_navigator는 새 goal을 받으면 옛 것을 끝낸다)
        if outcome == "succeeded":
            if pose is not None and math.dist(pose[:2], g.target[:2]) <= self.p.arrival_tolerance_m:
                self._active = None
                self._rest = IDLE
                self._attempts, self._attempt_origin = 0, None
                return [Finish(g.goal_id, True, "")] + self._end_active("", stop=True)
            g.far_successes += 1  # 목표에서 먼 성공은 다시 보낸다(follow_path와 같은 한계)
            if g.far_successes <= self.p.far_success_limit:
                return self._goals.send(g, now, pose)
        return self._end_active(NAV_ABORTED, stop=True)
