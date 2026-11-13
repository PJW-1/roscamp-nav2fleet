# ---- 막혔을 때 후진 후 다시 주행하는 자기 복구 믹스인 ----
from __future__ import annotations

import math
from collections.abc import Callable, Sequence

from pinky_fsm.backup import BackUp, CancelBackUp, backup_distance, keep_lateral
from pinky_fsm.commands import CancelNav, ClearCostmap
from pinky_fsm.geometry import Point
from pinky_fsm.records import _Move, _Recovery
from pinky_fsm.results import NAV_ABORTED

Obstacle = Callable[[list[Point]], bool]


# ---- 복구 시도 가능 여부와 후진 시작 ----
class SelfRecovery:
    def _can_recover(self, pose) -> bool:
        limit = 1 if self.p.local_yield else self.p.recovery_attempts
        return pose is not None and self._attempts < limit

    def _start_attempt(self, g: _Move, now: float, pose: Sequence[float]) -> list:
        if self.p.local_yield:
            return self._start_yield(g, now, pose)
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

    # ---- 옆 이탈을 넘지 않는 후진 거리 계산과 다시 주행 ----
    def _backup_distance(self, g: _Move, pose: Sequence[float]) -> float:
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

    # ---- 복구 진행 감시: 후진 시간 초과, 통과, 재시도, 포기 ----
    def _watch_recovery(self, now: float, g, pose, obstacle: Obstacle) -> list:
        r = self._recovery
        if r is None:
            if (self._attempt_origin is not None and pose is not None
                    and math.dist(pose[:2], self._attempt_origin) >= self.p.recovery_reset_m):
                self._attempts, self._attempt_origin = 0, None
            return []
        if not isinstance(g, _Move) or pose is None:
            return []
        if r.local:
            cmds = self._watch_yield(now, g, pose, obstacle)
            if cmds is not None:
                return cmds
        if r.backup_seq is not None:
            limit = self.p.backup_time_allowance_s + self.p.backup_result_grace_s
            if now - r.backup_since < limit:
                return []
            self._backup_running = False
            return [CancelBackUp()] + self._follow_again(g, r, now, pose)
        if g.s >= r.stuck_s + self.p.recovery_pass_m:
            self._recovery = None
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
