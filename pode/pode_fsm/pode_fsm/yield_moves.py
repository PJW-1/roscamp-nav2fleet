# ---- 경로 주행 중 로봇단 교착 처리(대기, 후진, 재시도) 믹스인 ----
from __future__ import annotations

from collections.abc import Sequence

from pode_fsm.backup import CancelBackUp
from pode_fsm.commands import CancelNav
from pode_fsm.geometry import Point
from pode_fsm.local_yield import BACKUP, RETRY, WAIT, Peer, blocking_peer, yields_to
from pode_fsm.records import Move, Recovery


# ---- 상대 로봇 받기와 현재 양보 단계 조회 ----
class LocalYield:
    def see_peers(self, now: float, peers: Sequence[Peer]) -> None:
        self._peers, self._peers_at = list(peers), now

    def local_yield_step(self) -> str:
        r = self._recovery
        if r is None or not r.local:
            return ""
        if r.wait_until is not None:
            return WAIT
        if r.backup_seq is not None:
            return BACKUP
        return RETRY

    # ---- 양보 시작: 주행 취소 후 대기 상태로 들어가기 ----
    def _start_yield(self, g: Move, now: float, pose: Sequence[float]) -> list:
        self._count_attempt(pose)
        g.s = self._track(g, pose)
        stood = self._stopped_since if self._stopped_since is not None else now
        self._recovery = Recovery(stuck_s=g.s, local=True, wait_until=now + self.p.local_wait_s,
                                   stood_since=stood)
        self._blocked, self._clear_since = False, None
        self._drop_hold()
        g.waiting_since = g.blocked_since = None
        if not self._nav_running:
            return []
        self._nav_running = g.nav_running = False
        return [CancelNav()]

    # ---- 양보 진행 감시: 길이 트이면 재주행, 아니면 후진 ----
    def _watch_yield(self, now: float, g: Move, pose, obstacle) -> list | None:
        r = self._recovery
        ahead = self._ahead(g, pose)
        if now - r.stood_since >= self.p.blocked_after_s and obstacle(ahead):
            return self._yield_blocked(g, now)
        if r.wait_until is None:
            return None
        if not obstacle(ahead):
            if self._clear_since is None:
                self._clear_since = now
            if now - self._clear_since < self.p.block_clear_s:
                return []
            self._clear_since, self._recovery = None, None
            self._stopped_since = None
            return self._send_rest(g, now, pose)
        self._clear_since = None
        if now < r.wait_until:
            return []
        r.wait_until = None
        d = self._yield_backup_m(g, pose) if self._backs_up(ahead, now) else 0.0
        if d < self.p.backup_min_m:
            return self._yield_blocked(g, now)
        return self._send_backup(r, now, d)

    # ---- 양보 후진 거리와 후진 여부 판정 ----
    def _yield_backup_m(self, g: Move, pose: Sequence[float]) -> float:
        return max(0.0, min(self.p.local_backup_m, self._backup_distance(g, pose),
                            g.s - g.s_start))

    def _backs_up(self, ahead: list[Point], now: float) -> bool:
        if now - self._peers_at > self.p.peers_timeout_s:
            return False
        peer = blocking_peer(ahead, self._peers, self.robot_id, self.p.local_peer_near_m)
        if peer is None:
            return False
        mine = next((p.priority for p in self._peers if p.name == self.robot_id), self._priority)
        return yields_to(self.robot_id, mine, peer)

    # ---- 양보 포기하고 막힘 표시 ----
    def _yield_blocked(self, g, now: float) -> list:
        cmds: list = []
        if self._backup_running:
            self._backup_running = False
            cmds.append(CancelBackUp())
        self._recovery = None
        self._mark_blocked(self._pose)
        if isinstance(g, Move):
            if g.blocked_since is None:
                g.blocked_since = now
            if not g.nav_running:
                g.waiting_since = now
        return cmds
