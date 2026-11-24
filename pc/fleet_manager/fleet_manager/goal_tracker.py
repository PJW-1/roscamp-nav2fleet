# ---- 액션 goal 추적과 보고 재전송 대기열 ----
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
