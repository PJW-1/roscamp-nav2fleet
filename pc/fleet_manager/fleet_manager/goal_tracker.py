# ---- 액션 goal 추적과 보고 재전송 대기열(central·브리지) ----
from __future__ import annotations

import random
from dataclasses import dataclass, field

from fleet_core.common.params import RETRY_JITTER


def _factor(rand) -> float:
    # IR-41: 재전송 간격에 곱하는 배수, 0.8~1.2 균등
    return 1.0 - RETRY_JITTER + 2.0 * RETRY_JITTER * rand()


# ---- goal 전송·수락·완료·유실과 취소 상태 추적 ----
@dataclass
class _Cancel:
    tried_at: float
    factor: float
    tries: int = 1
    warned: bool = False


class GoalTracker:
    def __init__(self, rand=random.random):
        self._rand = rand
        self._pending: set = set()
        self._handles: dict = {}
        self._cancels: dict = {}
        self._lost: set = set()
        self._result_lost: set = set()  # 결과 future를 잃었지만 handle은 쥔 goal(IR-62)
        self._lost_seen: dict = {}  # 그 goal을 처음 본 시각. 오래되면 놓는다
        self._dropped: list = []

    def sent(self, key) -> None:
        self._pending.add(key)

    def accepted(self, key, handle) -> None:
        self._pending.discard(key)
        self._handles[key] = handle
        if key in self._cancels:
            handle.cancel_goal_async()

    def finished(self, key) -> None:
        self._pending.discard(key)
        self._handles.pop(key, None)
        self._cancels.pop(key, None)
        self._lost.discard(key)
        self._result_lost.discard(key)
        self._lost_seen.pop(key, None)

    def lost(self, key) -> None:
        if key in self._cancels:
            self._lost.add(key)

    def result_lost(self, key) -> None:
        self._pending.discard(key)
        self._result_lost.add(key)

    def drop_stale_lost(self, now: float, keep_s: float) -> None:
        for key in list(self._result_lost):
            if now - self._lost_seen.setdefault(key, now) >= keep_s:
                self.finished(key)

    def cancel(self, key, now: float) -> None:
        if key in self._cancels:
            return
        handle = self._handles.get(key)
        if handle is not None:
            handle.cancel_goal_async()
        elif key not in self._pending:
            return
        if key in self._result_lost:  # 결과가 올 수 없으니 한 번 보내고 끝낸다
            self.finished(key)
            return
        self._cancels[key] = _Cancel(now, _factor(self._rand))

    def canceling(self, key) -> bool:
        return key in self._cancels

    def take_dropped(self) -> list:
        dropped, self._dropped = self._dropped, []
        return dropped

    def retry_cancels(self, now: float, retry_s: float, warn_after: int) -> list:
        unconfirmed = []
        for key, cancel in list(self._cancels.items()):
            if now - cancel.tried_at < retry_s * cancel.factor:
                continue
            if cancel.warned and key in self._lost:
                self.finished(key)
                self._dropped.append(key)
                continue
            if cancel.tries >= warn_after and not cancel.warned:
                cancel.warned = True
                unconfirmed.append(key)
            cancel.tried_at = now
            cancel.factor = _factor(self._rand)
            cancel.tries += 1
            handle = self._handles.get(key)
            if handle is not None:
                handle.cancel_goal_async()
        return unconfirmed


# ---- 주문 보고 재전송 대기열 ----
@dataclass
class _Report:
    item: object
    sent_at: float | None = None
    factor: float = 1.0
    requests: list = field(default_factory=list)  # 답을 기다리는 [보낸 시각, future]


class ReportOutbox:
    def __init__(self, rand=random.random):
        self._rand = rand
        self._items: dict = {}
        self._next_id = 0

    def add(self, item) -> None:
        self._items[self._next_id] = _Report(item)
        self._next_id += 1

    def due(self, now: float, ready: bool, retry_s: float, limit: int) -> list:
        if not ready:
            return []
        # IR-41: 답을 기다리는 요청 수가 상한이다. 다시 보내기도 요청 하나다
        in_flight = sum(len(report.requests) for report in self._items.values())
        out = []
        for report_id, report in list(self._items.items()):
            if report.sent_at is not None and now - report.sent_at < retry_s * report.factor:
                continue
            if in_flight >= limit:
                continue
            in_flight += 1
            report.sent_at = now
            report.factor = _factor(self._rand)
            report.requests.append([now, None])
            out.append((report_id, report.item))
        return out

    def track(self, report_id, future) -> None:
        # due가 연 마지막 요청에 future를 붙인다
        report = self._items.get(report_id)
        if report is not None and report.requests and report.requests[-1][1] is None:
            report.requests[-1][1] = future

    def expire(self, now: float, reply_limit_s: float) -> list:
        # IR-41: 응답 한도를 넘긴 요청을 빼고 그 future를 돌려준다(클라이언트에서 버림)
        expired = []
        for report in self._items.values():
            keep = []
            for sent, future in report.requests:
                if now - sent < reply_limit_s:
                    keep.append([sent, future])
                elif future is not None:
                    expired.append(future)
            report.requests = keep
        return expired

    def forget(self, report_id, future) -> None:
        # 실패로 끝난 요청은 자리를 비운다. 보고는 남아 다시 보낸다
        report = self._items.get(report_id)
        if report is not None:
            report.requests = [r for r in report.requests if r[1] is not future]

    def finish(self, report_id, future):
        # 보고를 끝내고 아직 답을 기다리는 다른 요청을 돌려준다. 이미 끝났으면 None
        report = self._items.pop(report_id, None)
        if report is None:
            return None
        return [f for _, f in report.requests if f is not None and f is not future]


# ---- 브리지 보고(Report*) 대기열: epoch 전 보류, 건마다 하나씩, 최신으로 바꿔 끼움 ----
def backoff_s(attempt: int, first_s: float, max_s: float, factor: float) -> float:
    # IR-40: 1 → 2 → 4 s … 상한 max_s, 지터 배수를 곱한다
    # 지수를 묶어 오래 실패해도 넘치지 않게 한다
    return min(first_s * 2.0 ** min(attempt - 1, 60), max_s) * factor


@dataclass
class _Slot:
    queued: tuple | None = None  # 보낼 (n, payload). 새 보고가 오면 바꿔 끼운다
    flight: tuple | None = None  # 답을 기다리는 (token, n, payload, 보낸 시각)
    attempt: int = 0
    next_at: float = float("-inf")


class BridgeOutbox:
    def __init__(self, first_s: float, max_s: float, reply_limit_s: float, max_in_flight: int,
                 rand=random.random):
        self._first_s, self._max_s = first_s, max_s
        self._reply_limit_s, self._cap = reply_limit_s, max_in_flight
        self._rand = rand
        self._slots: dict = {}  # 키 -> _Slot, 넣은 순서
        self._tokens: dict = {}  # token -> 키
        self._n = 0
        self._next_token = 0
        self.epoch = None

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def put(self, key, payload) -> int:
        # IR-23: 보내지 않은 같은 건 보고는 최신 것으로 바꾸고 n을 새로 준다
        self._n += 1
        self._slots.setdefault(key, _Slot()).queued = (self._n, payload)
        return self._n

    def waiting(self) -> int:
        return sum(s.queued is not None or s.flight is not None for s in self._slots.values())

    def in_flight(self) -> list:
        return [s.flight[0] for s in self._slots.values() if s.flight is not None]

    def due(self, now: float) -> list:
        # IR-22: epoch 전에는 보내지 않는다
        if self.epoch is None:
            return []
        busy = len(self.in_flight())
        out = []
        for key, slot in self._slots.items():
            if slot.flight is not None or slot.queued is None or now < slot.next_at:
                continue
            if busy >= self._cap:
                break
            busy += 1
            n, payload = slot.queued
            slot.queued = None
            token = self._next_token
            self._next_token += 1
            slot.flight = (token, n, payload, now)
            self._tokens[token] = key
            out.append((token, payload, self.epoch, n))
        return out

    def answered(self, token, delivered: bool, now: float) -> None:
        key = self._tokens.pop(token, None)
        slot = self._slots.get(key)
        if slot is None or slot.flight is None or slot.flight[0] != token:
            return  # 이미 버린 요청의 늦은 답
        _, n, payload, _ = slot.flight
        slot.flight = None
        if delivered:
            slot.attempt, slot.next_at = 0, float("-inf")
            if slot.queued is None:
                del self._slots[key]
            return
        # 같은 n으로 다시 보낸다. 그 사이 새 보고가 왔으면 그것을 보낸다
        if slot.queued is None:
            slot.queued = (n, payload)
        slot.attempt += 1
        slot.next_at = now + backoff_s(slot.attempt, self._first_s, self._max_s,
                                       _factor(self._rand))

    def expire(self, now: float) -> list:
        # IR-41: 응답 한도를 넘긴 요청은 실패로 치고 token을 돌려준다(클라이언트에서 버림)
        expired = [s.flight[0] for s in self._slots.values()
                   if s.flight is not None and now - s.flight[3] >= self._reply_limit_s]
        for token in expired:
            self.answered(token, False, now)
        return expired


# ---- Hello → ListOpenTrips: 한 번에 하나, 실패하면 같은 간격으로 계속 ----
class Handshake:
    def __init__(self, first_s: float, max_s: float, reply_limit_s: float, rand=random.random):
        self._first_s, self._max_s, self._reply_limit_s = first_s, max_s, reply_limit_s
        self._rand = rand
        self._phase = "hello"
        self._sent_at = None
        self._attempt = 0
        self._next_at = float("-inf")
        self.epoch = None

    @property
    def done(self) -> bool:
        return self._phase == "done"

    def due(self, now: float):
        if self.done or self._sent_at is not None or now < self._next_at:
            return None
        return self._phase

    def sent(self, now: float) -> None:
        self._sent_at = now

    def failed(self, now: float) -> None:
        self._sent_at = None
        self._attempt += 1
        self._next_at = now + backoff_s(self._attempt, self._first_s, self._max_s,
                                        _factor(self._rand))

    def expired(self, now: float) -> bool:
        if self._sent_at is None or now - self._sent_at < self._reply_limit_s:
            return False
        self.failed(now)
        return True

    def hello_ok(self, epoch: int) -> None:
        self.epoch = epoch
        self._advance("list")

    def list_ok(self) -> None:
        self._advance("done")

    def _advance(self, phase: str) -> None:
        self._phase, self._sent_at = phase, None
        self._attempt, self._next_at = 0, float("-inf")
