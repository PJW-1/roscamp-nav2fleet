# ---- 로봇까지 가는 관리 명령 부르기: 한도 · 간격 · 횟수, 결과 모름과 확정 (W-31 · IR-40 · IR-62) ----
from __future__ import annotations

import random
from collections.abc import Callable
from dataclasses import dataclass, field

from fleet_core.common.params import RETRY_JITTER

# ---- ReportCommand 결과 값 (ReportCommand.srv와 같은 값) ----
ACCEPTED = "ACCEPTED"
REJECTED = "REJECTED"
UNCONFIRMED = "UNCONFIRMED"
TIMEOUT = "TIMEOUT"
LATE_COMMAND_RESULT = "LATE_COMMAND_RESULT"

_SENDING = "SENDING"
_UNSURE = "UNSURE"
_STATE_WAIT = "STATE_WAIT"  # 받았다는 답 뒤 상태 확정을 기다림


@dataclass(frozen=True)
class CallParams:
    # Q-09: 잠정 값
    try_s: float = 2.0       # 한 번 부를 때 답 기다리는 한도
    gap_s: float = 1.0       # 다음 부르기까지 간격 (±20 %)
    tries: int = 5           # 처음 포함 부르는 횟수
    confirm_s: float = 30.0  # 첫 요청부터 이 안에 확정 못 하면 TIMEOUT
    max_in_flight: int = 8   # 노드 전체에서 답을 기다리는 요청 상한 (IR-41)


@dataclass
class _Call:
    request_id: str
    command: str
    target: str
    client: object
    request: object
    observe: Callable | None
    then: Callable | None
    first_at: float
    next_at: float
    key: str = ""
    report: Callable | None = None  # 없으면 노드의 ReportCommand
    confirm_accepted: bool = False  # 받았다는 답도 상태 관측으로 확정
    seen_ok: bool = False  # 보내는 중에 확정할 상태를 이미 봄 (confirm_accepted만)
    sent: bool = False  # 실제로 한 번이라도 보냄
    tries: int = 0
    asked_at: float | None = None
    future: object = None
    phase: str = _SENDING


@dataclass
class Step:
    send: list = field(default_factory=list)  # (key, client, request)
    drop: list = field(default_factory=list)  # (client, future): remove_pending_request 할 것


class RobotCalls:
    """같은 request_id로 다시 부르고, 끝 상태는 한 번만 ReportCommand로 알린다."""

    def __init__(self, params: CallParams, codes, report_command: Callable, event: Callable,
                 rand=random.random):
        self.params = params
        self._codes = codes
        self._report = report_command
        self._event = event
        self._rand = rand
        self._calls: dict[str, _Call] = {}

    def __contains__(self, key) -> bool:
        return key in self._calls

    def add(self, request_id, command, target, client, request, now, observe=None,
            then=None, key=None, report=None, confirm_accepted=False) -> None:
        """key: 한 request_id로 여러 로봇을 부를 때 부르기마다 다른 이름 (없으면 request_id).
        request_id 문자열과 겹치지 않게 튜플로 준다.
        confirm_accepted: observe가 있으면 받았다는 답도 상태가 맞을 때 확정한다."""
        key = request_id if key is None else key
        if key in self._calls:
            return
        self._calls[key] = _Call(request_id, command, target, client, request, observe, then,
                                 now, now, key, report, confirm_accepted)

    def target(self, key) -> str | None:
        c = self._calls.get(key)
        return None if c is None else c.target

    def pending(self, command, target) -> str | None:
        """target에 아직 끝나지 않은 command가 있으면 그 request_id."""
        for c in self._calls.values():
            if c.command == command and c.target == target:
                return c.request_id
        return None

    def _factor(self) -> float:
        return 1.0 - RETRY_JITTER + 2.0 * RETRY_JITTER * self._rand()

    # ---- 주기 처리: 보낼 것, 버릴 future, 결과 모름 · TIMEOUT ----
    def step(self, now: float, ready: Callable) -> Step:
        p, out = self.params, Step()
        # 쥐고 있는 마지막 요청도 답을 기다리는 요청이다 (IR-41)
        busy = sum(1 for c in self._calls.values() if c.future is not None)
        for c in list(self._calls.values()):
            if self._late(c, now):
                # 상한에 밀려 보내지 못한 명령도 30초에서 끝낸다
                if c.future is not None:
                    busy -= 1
                self._drop(c, out)
                why = ("accepted but the robot state did not confirm"
                       if c.phase == _STATE_WAIT else "no result")
                self._finish(c, TIMEOUT, self._codes.ROBOT_TIMEOUT,
                             f"{why} within {p.confirm_s:.0f} s", None)
                self._event("WARN", LATE_COMMAND_RESULT,
                            f"{c.command} {c.request_id} has no result after "
                            f"{p.confirm_s:.0f} s", c.target)
                continue
            if c.phase in (_UNSURE, _STATE_WAIT):
                continue
            if c.asked_at is not None:
                if now - c.asked_at < p.try_s:
                    continue
                c.asked_at = None
                if c.tries >= p.tries:
                    if c.seen_ok:
                        # 답은 없지만 보낸 뒤 상태로 이미 확인됐다
                        if c.future is not None:
                            busy -= 1
                        self._drop(c, out)
                        self._finish(c, ACCEPTED, "", "confirmed by the robot state", True)
                        continue
                    # 마지막 요청은 늦은 답을 받으려고 쥔다
                    c.phase = _UNSURE
                    (c.report or self._report)(c.request_id, c.command, c.target, UNCONFIRMED,
                                               self._codes.ROBOT_TIMEOUT,
                                               f"no answer after {c.tries} tries")
                    continue
                if c.future is not None:
                    busy -= 1
                    self._drop(c, out)
                c.next_at = now + p.gap_s * self._factor()
            elif now >= c.next_at:
                if ready(c.client):
                    if busy >= p.max_in_flight:
                        continue
                    busy += 1
                    c.sent = True
                    out.send.append((c.key, c.client, c.request))
                c.tries += 1
                c.asked_at = now
        return out

    def track(self, key, future) -> None:
        c = self._calls.get(key)
        if c is not None and c.future is None:
            c.future = future

    # ---- 답과 관측 ----
    def _late(self, c, now) -> bool:
        return now is not None and now - c.first_at >= self.params.confirm_s

    def on_answer(self, key, future, accepted: bool, reason="", detail="",
                  now=None, view=None) -> list:
        """늦은 답도 기한 안이면 끝 상태를 한 번 정한다. 버릴 (client, future)를 돌려준다.
        view: 답을 받을 때 그 로봇의 마지막 상태 (confirm_accepted 확정용)."""
        c = self._calls.get(key)
        if c is None or self._late(c, now) or c.phase == _STATE_WAIT:
            return []  # 기한이 지났으면 다음 step이 TIMEOUT으로 끝낸다
        out = Step()
        if c.future is not future:
            self._drop(c, out)
        c.future = None
        if accepted and c.confirm_accepted and c.observe is not None:
            if c.seen_ok or (view is not None and c.observe(view)):
                self._finish(c, ACCEPTED, "", "confirmed by the robot state", True)
            else:
                c.phase = _STATE_WAIT  # 다시 부르지 않고 상태 관측이나 30초를 기다린다
        elif accepted:
            self._finish(c, ACCEPTED, "", detail, True)
        else:
            self._finish(c, REJECTED, self._codes.ROBOT_REJECTED, f"{reason} {detail}".strip(),
                         False)
        return out.drop

    def on_failed(self, key, future) -> None:
        """답 없이 끝난 요청은 비우고, 한도가 지나면 다음 부르기로 간다."""
        c = self._calls.get(key)
        if c is not None and c.future is future:
            c.future = None

    def observe(self, target, view, now=None) -> list:
        """결과 모름인 명령을 기한 안의 상태 관측으로 확정한다 (IR-62)."""
        out = Step()
        for c in list(self._calls.values()):
            if (c.phase == _SENDING and c.confirm_accepted and c.sent and c.target == target
                    and c.observe is not None and not self._late(c, now) and c.observe(view)):
                c.seen_ok = True  # 답이 오면 확정에 쓴다
                continue
            if (c.phase in (_UNSURE, _STATE_WAIT) and c.target == target
                    and c.observe is not None
                    and not self._late(c, now) and c.observe(view)):
                self._drop(c, out)
                self._finish(c, ACCEPTED, "", "confirmed by the robot state", True)
        return out.drop

    def _drop(self, c, out: Step) -> None:
        if c.future is not None:
            out.drop.append((c.client, c.future))
            c.future = None

    def _finish(self, c, result, reason, detail, accepted) -> None:
        self._calls.pop(c.key, None)
        (c.report or self._report)(c.request_id, c.command, c.target, result, reason, detail)
        if c.then is not None:
            c.then(accepted)
