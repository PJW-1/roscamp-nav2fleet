# ---- 운영 모드 맞추기: /system/mode를 로봇에 알리고, 모드가 다른 Pode는 새 배정에서 뺀다 (IR-73) ----
from __future__ import annotations

import uuid
from collections.abc import Callable, Iterable
from dataclasses import dataclass

RECONNECT_GAP_S = 1.5  # 상태가 이만큼 끊겼다 다시 오면 다시 연결로 본다 (IR-34)
RETRY_S = 2.0          # 같은 로봇에 다시 부르는 간격
ANSWER_S = 3.0         # 답을 기다리는 한도. 넘기면 다음 차례에 다시 부른다
TRIES = 2              # 모드가 다른 채 이만큼 부르고 RETRY_S 뒤에도 다르면 배정에서 뺀다
NO_SERVER_S = 5.0      # set_mode 서버가 이만큼 없으면 부르지 못해도 뺀다 (IR-32)


@dataclass
class _Robot:
    blockable: bool
    seen_at: float | None = None
    boot_id: str = ""
    mode: str = ""
    notify: bool = False
    next_at: float = 0.0
    tries: int = 0
    block_at: float | None = None  # TRIES번째 호출 + RETRY_S: 이때도 다르면 뺀다
    asked_at: float | None = None
    no_server_since: float | None = None
    request_id: str = ""
    id_mode: str | None = None  # request_id를 만든 시스템 모드
    answered: bool = True       # request_id에 답이 왔음 (다음 호출은 새 번호)
    blocked: bool = False
    told: bool = False


class ModeSync:
    def __init__(self, pods: Iterable[str], arms: Iterable[str] = (),
                 gap_s: float = RECONNECT_GAP_S, retry_s: float = RETRY_S,
                 answer_s: float = ANSWER_S, tries: int = TRIES,
                 no_server_s: float = NO_SERVER_S):
        self.system_mode: str | None = None
        self.no_server: set[str] = set()  # 마지막 시도에 서버가 없던 로봇
        self.expired: list[str] = []      # 이번 차례에 답 기한이 지난 로봇 (옛 요청 정리용)
        self._gap_s, self._retry_s, self._answer_s, self._tries = gap_s, retry_s, answer_s, tries
        self._no_server_s = no_server_s
        self._robots = {name: _Robot(True) for name in pods}
        self._robots.update({name: _Robot(False) for name in arms})

    # ---- 입력: 시스템 모드, 로봇 상태, set_mode 답 ----
    def on_system_mode(self, mode: str) -> bool:
        """바뀌었으면 True. 모든 로봇에 다시 알린다."""
        if not mode or mode == self.system_mode:
            return False
        self.system_mode = mode
        for r in self._robots.values():
            r.notify, r.next_at, r.tries, r.block_at = True, 0.0, 0, None
        return True

    def on_state(self, robot: str, mode: str, boot_id: str, now: float) -> None:
        r = self._robots.get(robot)
        if r is None:
            return
        rebooted = bool(r.boot_id) and boot_id != r.boot_id
        back = r.seen_at is None or now - r.seen_at > self._gap_s or rebooted
        r.seen_at, r.boot_id, r.mode = now, boot_id, mode
        if back:
            r.notify, r.next_at = True, 0.0
        if rebooted:  # 새 프로그램이면 호출 횟수와 서버 없음을 처음부터 센다
            r.tries, r.block_at, r.no_server_since = 0, None, None
        if self.system_mode is not None and mode == self.system_mode:
            r.tries, r.block_at, r.blocked, r.no_server_since = 0, None, False, None

    def on_answer(self, robot: str, request_id: str) -> None:
        r = self._robots.get(robot)
        if r is not None and r.request_id == request_id:
            r.asked_at, r.answered = None, True

    # ---- 출력: 지금 부를 로봇, 배정 제외가 바뀐 Pode ----
    def due(self, now: float, ready: Callable[[str], bool]) -> list[tuple[str, str]]:
        """(로봇, request_id). 서버가 없으면 부르지 않고, NO_SERVER_S 넘게 없으면 뺀다."""
        self.expired = []
        if self.system_mode is None:
            return []
        calls = []
        for name, r in self._robots.items():
            pending = r.asked_at is not None and now - r.asked_at <= self._answer_s
            if r.asked_at is not None and not pending:
                r.asked_at = None
                self.expired.append(name)
            if r.seen_at is None or now - r.seen_at > self._gap_s:
                continue
            differs = r.mode != self.system_mode
            # 답을 기다리는 중이어도 TRIES번 부르고 RETRY_S가 지나면 뺀다
            if differs and r.blockable and r.block_at is not None and now >= r.block_at:
                r.blocked = True
            if pending or not (r.notify or differs) or now < r.next_at:
                continue
            r.notify, r.next_at = False, now + self._retry_s
            if ready(name):
                r.no_server_since = None
                if differs:
                    r.tries += 1
                    if r.tries >= self._tries and r.block_at is None:
                        r.block_at = now + self._retry_s
                # IR-20: 답이 없던 요청은 같은 번호로, 답을 받았거나 시스템 모드가 바뀌면 새 UUID
                if r.answered or r.id_mode != self.system_mode or not r.request_id:
                    r.request_id, r.id_mode = str(uuid.uuid4()), self.system_mode
                r.asked_at, r.answered = now, False
                calls.append((name, r.request_id))
                self.no_server.discard(name)
            else:
                if r.no_server_since is None:
                    r.no_server_since = now
                if differs and r.blockable and now - r.no_server_since >= self._no_server_s:
                    r.blocked = True
                self.no_server.add(name)
        return calls

    def changes(self) -> list[tuple[str, bool, str]]:
        """(Pode, 배정 제외 여부, 까닭). 바뀐 것만 한 번."""
        out = []
        for name, r in self._robots.items():
            if r.blockable and r.blocked != r.told:
                r.told = r.blocked
                why = f"reports {r.mode or 'no mode'}, system {self.system_mode}" if r.blocked else ""
                out.append((name, r.blocked, why))
        return out
