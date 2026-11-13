from __future__ import annotations

import math
from collections.abc import Sequence

from pinky_fsm.backup import BackUp, CancelBackUp
from pinky_fsm.local_yield import extend_trail, straight_back_m
from pinky_fsm.nav_goal import NavGoal
from pinky_fsm.records import _Recovery

TRAIL_STEP_M = 0.02
TRAIL_KEEP_M = 0.5
PENDING = 0


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
                self._recovery = None
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
            self._recovery = None
            return []
        if (now - g.anchor_at >= self.p.block_report_s
                and now - r.follow_since >= self.p.block_report_s):
            return self._nav_yield_blocked(g, now, pose)
        return []

    def _ray(self, pose: Sequence[float]) -> list:
        n = max(1, math.ceil(self.p.block_lookahead_m / 0.05))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return [(pose[0] + c * self.p.block_lookahead_m * i / n,
                 pose[1] + s * self.p.block_lookahead_m * i / n) for i in range(n + 1)]

    def _nav_backup_m(self, g: NavGoal, pose: Sequence[float]) -> float:
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
        r, cmds = self._recovery, []
        if r.backup_seq is not None:
            if self._backup_running:
                self._backup_running = False
                cmds.append(CancelBackUp())
            cmds += self._goals.send(g, now, pose)
        self._recovery = None
        g.blocked = True
        return cmds
