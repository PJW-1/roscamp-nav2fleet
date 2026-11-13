# ---- 액션 goal 추적과 보고 재전송 대기열 ----
from __future__ import annotations

from dataclasses import dataclass


# ---- goal 전송·수락·완료·유실과 취소 상태 추적 ----
@dataclass
class _Cancel:
    tried_at: float
    tries: int = 1
    warned: bool = False


class GoalTracker:
    def __init__(self):
        self._pending: set = set()
        self._handles: dict = {}
        self._cancels: dict = {}
        self._lost: set = set()
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

    def lost(self, key) -> None:
        if key in self._cancels:
            self._lost.add(key)

    def cancel(self, key, now: float) -> None:
        if key in self._cancels:
            return
        handle = self._handles.get(key)
        if handle is not None:
            handle.cancel_goal_async()
        elif key not in self._pending:
            return
        self._cancels[key] = _Cancel(now)

    def canceling(self, key) -> bool:
        return key in self._cancels

    def take_dropped(self) -> list:
        dropped, self._dropped = self._dropped, []
        return dropped

    def retry_cancels(self, now: float, retry_s: float, warn_after: int) -> list:
        unconfirmed = []
        for key, cancel in list(self._cancels.items()):
            if now - cancel.tried_at < retry_s:
                continue
            if cancel.warned and key in self._lost:
                self.finished(key)
                self._dropped.append(key)
                continue
            if cancel.tries >= warn_after and not cancel.warned:
                cancel.warned = True
                unconfirmed.append(key)
            cancel.tried_at = now
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


class ReportOutbox:
    def __init__(self):
        self._items: dict = {}
        self._next_id = 0

    def add(self, item) -> None:
        self._items[self._next_id] = _Report(item)
        self._next_id += 1

    def due(self, now: float, ready: bool, retry_s: float) -> list:
        if not ready:
            return []
        out = []
        for report_id, report in list(self._items.items()):
            if report.sent_at is None or now - report.sent_at >= retry_s:
                report.sent_at = now
                out.append((report_id, report.item))
        return out

    def done(self, report_id) -> bool:
        return self._items.pop(report_id, None) is not None
