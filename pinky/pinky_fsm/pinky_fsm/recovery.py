"""PinkyFsm의 자가 복구(핑키_계약 3절 7번). ROS를 모른다.

core.PinkyFsm이 이 클래스를 물려받는다(core.py가 길어 책임별로 나눴다). 여기 메서드는 core의
상태(_active, _recovery, _attempts, BackUp 기록)를 그대로 쓴다.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence

from pinky_fsm.backup import BackUp, CancelBackUp, backup_distance, keep_lateral
from pinky_fsm.commands import CancelNav, ClearCostmap
from pinky_fsm.geometry import Point
from pinky_fsm.records import _Move, _Recovery
from pinky_fsm.results import NAV_ABORTED

Obstacle = Callable[[list[Point]], bool]


class SelfRecovery:
    def _can_recover(self, pose) -> bool:
        return pose is not None and self._attempts < self.p.recovery_attempts

    def _start_attempt(self, g: _Move, now: float, pose: Sequence[float]) -> list:
        """FollowPath를 멈추고(BackUp과 둘 다 cmd_vel을 낸다) 물러난다. 물러날 자리가 없으면
        바로 지도를 비우고 다시 보낸다."""
        self._attempts += 1
        if self._attempt_origin is None:
            self._attempt_origin = (float(pose[0]), float(pose[1]))
        g.s = self._track(g, pose)
        r = self._recovery = _Recovery(stuck_s=g.s)
        self._blocked, self._clear_since = False, None
        g.waiting_since = g.blocked_since = None
        cmds: list = []
        if self._nav_running:
            self._nav_running = g.nav_running = False
            cmds.append(CancelNav())
        d = self._backup_distance(g, pose)
        if d < self.p.backup_min_m:
            return cmds + self._follow_again(g, r, now, pose)
        self._bseq += 1
        r.backup_seq, r.backup_since = self._bseq, now
        self._backup_running = True
        self._backup_unanswered[self._bseq] = (now + self.p.backup_time_allowance_s
                                               + self.p.backup_result_grace_s)
        return cmds + [BackUp(self._bseq, d, self.p.backup_speed_mps,
                              self.p.backup_time_allowance_s)]

    def _backup_distance(self, g: _Move, pose: Sequence[float]) -> float:
        """마지막으로 지난 경로 꼭짓점(경로 시작 포함)에서 온 거리로 정하고(backup_distance),
        레인 옆 거리가 커지지 않게 줄인다."""
        d = backup_distance(g.s - g.route.vertex_before(g.s), self.p.backup_max_m,
                            self.p.release_distance_m + self.p.backup_margin_m)
        qx, qy = g.route.point_at(g.s)
        phi = g.route.heading_at(g.s)
        lateral = -(pose[0] - qx) * math.sin(phi) + (pose[1] - qy) * math.cos(phi)
        err = math.atan2(math.sin(pose[2] - phi), math.cos(pose[2] - phi))
        return keep_lateral(d, lateral, err, self.p.backup_lateral_limit_m)

    def _follow_again(self, g: _Move, r: _Recovery, now: float, pose) -> list:
        r.backup_seq = None
        r.follow_since = now
        self._stopped_since = None
        return [ClearCostmap()] + self._send_rest(g, now, pose)

    def _watch_recovery(self, now: float, g, pose, obstacle: Obstacle) -> list:
        """복구가 끝났는지, 시간이 다 됐는지 본다. 복구 밖에서는 시도 수를 지울지 본다."""
        r = self._recovery
        if r is None:
            if (self._attempt_origin is not None and pose is not None
                    and math.dist(pose[:2], self._attempt_origin) >= self.p.recovery_reset_m):
                self._attempts, self._attempt_origin = 0, None
            return []
        if not isinstance(g, _Move) or pose is None:
            return []
        if r.backup_seq is not None:
            limit = self.p.backup_time_allowance_s + self.p.backup_result_grace_s
            if now - r.backup_since < limit:
                return []
            self._backup_running = False
            return [CancelBackUp()] + self._follow_again(g, r, now, pose)
        if g.s >= r.stuck_s + self.p.recovery_pass_m:
            self._recovery = None  # 막힌 곳을 지났다. 시도 수는 recovery_reset_m 뒤에 지운다
            return []
        if now - r.follow_since < self.p.recovery_timeout_s:
            return []
        if self._can_recover(pose):
            return self._start_attempt(g, now, pose)
        self._recovery = None
        if obstacle(self._ahead(g, pose)):
            self._blocked = True
            g.blocked_since = now
            return []
        return self._end_active(NAV_ABORTED, stop=True)
