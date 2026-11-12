"""PinkyFsm의 로봇단 교착 처리 중 mode nav_goal 쪽(추가 지시서 3의 7-4, 기준(사용자)). ROS를 모른다.

core.PinkyFsm이 이 클래스를 물려받는다. 고정 경로 쪽과 순서와 시간은 같고(yield_moves.py), 다른
것은 아래다.
- 섰다: 진전(nav_goal.py) 없이 block_report_s. 앞을 보는 경로가 없으니 로봇이 보는 방향으로
  block_lookahead_m을 본다. 물체가 비켰다는 것도 진전으로 안다.
- 기다리는 동안 NavigateToPose는 그대로 돈다. BT의 복구는 지도 지우기와 기다리기뿐이라 로봇을
  움직이지 않는다(Spin, BackUp을 뺐다. pinky_nav_config의 BT).
- 물러날 때는 NavigateToPose를 멈추고 바퀴가 선 뒤 BackUp을 낸다(nav_moves.py의 바꾸기 규칙).
  거리는 이 goal에서 지나온 길(trail)을 곧게 거꾸로 따를 수 있는 만큼이다. 끝나면 같은 goal을
  다시 보낸다(retry).
- 막힌 곳을 지났다: 목표까지 곧은 거리가 선 때보다 recovery_pass_m 줄었다.
"""
from __future__ import annotations

import math
from collections.abc import Sequence

from pinky_fsm.backup import BackUp, CancelBackUp
from pinky_fsm.local_yield import extend_trail, straight_back_m
from pinky_fsm.nav_goal import NavGoal
from pinky_fsm.records import _Recovery

TRAIL_STEP_M = 0.02  # 지나온 길의 점 간격
TRAIL_KEEP_M = 0.5  # 지나온 길은 이만큼만 둔다(local_backup_m보다 넉넉히)
PENDING = 0  # _Recovery.backup_seq: NavigateToPose가 서기를 기다린다


class NavYield:
    def _watch_nav_yield(self, now: float, g: NavGoal, pose, speed) -> list:
        if pose is None or g.seq == 0:
            return []
        r = self._recovery
        if r is None or r.backup_seq is None:
            g.driven += extend_trail(g.trail, pose, TRAIL_STEP_M, TRAIL_KEEP_M)
        if r is None:
            stalled = now - g.anchor_at >= self.p.block_report_s
            if stalled and not g.blocked and self._can_recover(pose):
                self._attempts += 1
                if self._attempt_origin is None:
                    self._attempt_origin = (float(pose[0]), float(pose[1]))
                self._recovery = _Recovery(
                    stuck_s=math.dist(pose[:2], g.target[:2]), local=True,
                    wait_until=now + self.p.local_wait_s, stood_since=g.anchor_at)
            return []
        if not r.local:
            return []
        if now - r.stood_since >= self.p.blocked_after_s:
            return self._nav_yield_blocked(g, now, pose)
        if r.wait_until is not None:
            if g.anchor_at > r.stood_since:
                self._recovery = None  # 움직였다. 비켰거나 Nav2가 돌아간다
                return []
            if now < r.wait_until:
                return []
            r.wait_until = None
            d = self._nav_backup_m(g, pose) if self._backs_up(self._ray(pose), now) else 0.0
            if d < self.p.backup_min_m:
                return self._nav_yield_blocked(g, now, pose)
            r.backup_seq, r.backup_since, r.backup_m = PENDING, now, d
            return self._goals.stop()
        if r.backup_seq == PENDING:
            if self._motion_ended(now, speed):
                return self._nav_backup(r, now)
            if now - r.backup_since >= self.p.retreat_stop_wait_s:
                return self._nav_yield_blocked(g, now, pose)
            return []
        if r.backup_seq is not None:
            limit = self.p.backup_time_allowance_s + self.p.backup_result_grace_s
            if now - r.backup_since < limit:
                return []
            self._backup_running = False
            return [CancelBackUp()] + self._nav_follow_again(g, r, now, pose)
        if math.dist(pose[:2], g.target[:2]) <= r.stuck_s - self.p.recovery_pass_m:
            self._recovery = None  # 막힌 곳을 지났다
            return []
        if (now - g.anchor_at >= self.p.block_report_s
                and now - r.follow_since >= self.p.block_report_s):
            return self._nav_yield_blocked(g, now, pose)
        return []

    def _ray(self, pose: Sequence[float]) -> list:
        """로봇이 보는 방향으로 block_lookahead_m까지의 점들."""
        n = max(1, math.ceil(self.p.block_lookahead_m / 0.05))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return [(pose[0] + c * self.p.block_lookahead_m * i / n,
                 pose[1] + s * self.p.block_lookahead_m * i / n) for i in range(n + 1)]

    def _nav_backup_m(self, g: NavGoal, pose: Sequence[float]) -> float:
        """이 goal에서 지나온 길 안에서만. goal을 받은 자리는 꼭짓점으로 쳐서 release + 여유를
        남긴다(자가 복구의 backup_distance와 같은 규칙)."""
        keep = self.p.release_distance_m + self.p.backup_margin_m
        straight = straight_back_m(g.trail, pose, self.p.local_backup_m,
                                   self.p.backup_lateral_limit_m)
        return max(0.0, min(straight, g.driven - keep))

    def _nav_backup(self, r: _Recovery, now: float) -> list:
        self._bseq += 1
        r.backup_seq, r.backup_since = self._bseq, now
        self._backup_running = True
        self._backup_unanswered[self._bseq] = (now + self.p.backup_time_allowance_s
                                               + self.p.backup_result_grace_s)
        return [BackUp(self._bseq, r.backup_m, self.p.backup_speed_mps,
                       self.p.backup_time_allowance_s)]

    def _nav_follow_again(self, g: NavGoal, r: _Recovery, now: float, pose) -> list:
        r.backup_seq, r.follow_since = None, now
        return self._goals.send(g, now, pose)

    def _nav_backup_result(self, g: NavGoal, now: float, pose) -> list:
        self._backup_running = False
        return self._nav_follow_again(g, self._recovery, now, pose)

    def _nav_yield_blocked(self, g: NavGoal, now: float, pose) -> list:
        """BLOCKED. 물러나던 중이면 멈추고, 멈춘 NavigateToPose는 다시 보낸다(goal은 그대로)."""
        r, cmds = self._recovery, []
        if r.backup_seq is not None:
            if self._backup_running:
                self._backup_running = False
                cmds.append(CancelBackUp())
            cmds += self._goals.send(g, now, pose)
        self._recovery = None
        g.blocked = True
        return cmds
