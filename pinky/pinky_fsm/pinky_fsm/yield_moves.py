"""PinkyFsm의 로봇단 교착 처리(추가 지시서 3의 7-4, 기준(사용자)). ROS를 모른다.

core.PinkyFsm이 이 클래스를 물려받는다(core.py가 길어 책임별로 나눴다). 여기 메서드는 core의
상태(_active, _recovery, _attempts, BackUp 기록)를 그대로 쓴다. 판단은 local_yield.py에 있다.

순서(고정 경로): 물체 앞에 block_report_s 서면 FollowPath를 멈추고 local_wait_s 기다린다(wait).
물체가 block_clear_s 동안 안 보이면 남은 선을 다시 보낸다. 기다린 뒤에도 있으면, 앞을 막은 것이
관제가 아는 다른 로봇이고 내가 낮은 쪽일 때만 물러나고(backup) 지도를 비운 뒤 다시 간다(retry).
물러날 길이 없거나(목표 처음) 낮은 쪽이 아니면 바로 BLOCKED다. 다시 가서 또 서면 BLOCKED다.
선 때부터 blocked_after_s가 지나도 물체가 있으면 어느 단계에서든 BLOCKED다(후진은 멈춘다).
한 자리에서 한 번만 한다(자가 복구의 시도 수와 recovery_reset_m을 같이 쓴다). 그동안 상태는
RECOVERING이고 단계는 local_yield_step()으로 보고한다(RobotState.local_yield, 계약 0.13-draft).
"""
from __future__ import annotations

from collections.abc import Sequence

from pinky_fsm.backup import BackUp, CancelBackUp
from pinky_fsm.commands import CancelNav
from pinky_fsm.geometry import Point
from pinky_fsm.local_yield import BACKUP, RETRY, WAIT, Peer, blocking_peer, yields_to
from pinky_fsm.records import _Move, _Recovery


class LocalYield:
    # -- 입력과 보고 ---------------------------------------------------------------------------

    def see_peers(self, now: float, peers: Sequence[Peer]) -> None:
        """관제 /fleet/robots의 다른 로봇들(위치와 priority). 받은 시각과 함께 둔다."""
        self._peers, self._peers_at = list(peers), now

    def local_yield_step(self) -> str:
        """로봇단 교착 처리의 단계(wait, backup, retry). 그 밖에는 빈 값이다."""
        r = self._recovery
        if r is None or not r.local:
            return ""
        if r.wait_until is not None:
            return WAIT
        if r.backup_seq is not None:
            return BACKUP
        return RETRY

    # -- 고정 경로 ------------------------------------------------------------------------------

    def _start_yield(self, g: _Move, now: float, pose: Sequence[float]) -> list:
        """서 있는 채로 기다리기 시작한다. FollowPath는 멈춘다(물러날 때도 먼저 멈춰야 한다)."""
        self._attempts += 1
        if self._attempt_origin is None:
            self._attempt_origin = (float(pose[0]), float(pose[1]))
        g.s = self._track(g, pose)
        stood = self._stopped_since if self._stopped_since is not None else now
        self._recovery = _Recovery(stuck_s=g.s, local=True, wait_until=now + self.p.local_wait_s,
                                   stood_since=stood)
        self._blocked, self._clear_since = False, None
        g.waiting_since = g.blocked_since = None
        if not self._nav_running:
            return []
        self._nav_running = g.nav_running = False
        return [CancelNav()]

    def _watch_yield(self, now: float, g: _Move, pose, obstacle) -> list | None:
        """wait 단계와 blocked_after_s를 본다. None이면 자가 복구와 같은 감시(backup, retry)로
        넘긴다."""
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
            self._clear_since, self._recovery = None, None  # 비켰다. 시도 수는 남는다
            self._stopped_since = None
            return self._send_rest(g, now, pose)
        self._clear_since = None
        if now < r.wait_until:
            return []
        r.wait_until = None
        d = self._yield_backup_m(g, pose) if self._backs_up(ahead, now) else 0.0
        if d < self.p.backup_min_m:
            return self._yield_blocked(g, now)
        self._bseq += 1
        r.backup_seq, r.backup_since = self._bseq, now
        self._backup_running = True
        self._backup_unanswered[self._bseq] = (now + self.p.backup_time_allowance_s
                                               + self.p.backup_result_grace_s)
        return [BackUp(self._bseq, d, self.p.backup_speed_mps, self.p.backup_time_allowance_s)]

    def _yield_backup_m(self, g: _Move, pose: Sequence[float]) -> float:
        """지금 목표에서 지나온 선(s_start부터) 안에서, local_backup_m까지. 자가 복구의 규칙(지난
        꼭짓점에서 release + 여유를 남기고, 레인 옆으로 벗어나지 않게)도 지킨다."""
        return max(0.0, min(self.p.local_backup_m, self._backup_distance(g, pose),
                            g.s - g.s_start))

    def _backs_up(self, ahead: list[Point], now: float) -> bool:
        """앞을 막은 것이 관제가 아는 다른 로봇이고 내가 낮은 쪽이다. 내 priority도 같은
        /fleet/robots의 내 줄에서 읽는다(두 로봇이 같은 두 값을 비교하게). 그 줄이 없으면 MoveTo의
        것이다."""
        if now - self._peers_at > self.p.peers_timeout_s:
            return False
        peer = blocking_peer(ahead, self._peers, self.robot_id, self.p.local_peer_near_m)
        if peer is None:
            return False
        mine = next((p.priority for p in self._peers if p.name == self.robot_id), self._priority)
        return yields_to(self.robot_id, mine, peer)

    def _yield_blocked(self, g, now: float) -> list:
        """로봇단 처리를 끝내고 BLOCKED. 물러나던 중이면 멈춘다. 물체가 비키면 남은 선을 다시
        보낸다(_watch_obstacle의 waiting)."""
        cmds: list = []
        if self._backup_running:
            self._backup_running = False
            cmds.append(CancelBackUp())
        self._recovery = None
        self._blocked = True
        if isinstance(g, _Move):
            if g.blocked_since is None:
                g.blocked_since = now
            if not g.nav_running:
                g.waiting_since = now
        return cmds
