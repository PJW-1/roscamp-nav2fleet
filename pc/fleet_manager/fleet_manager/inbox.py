# ---- 받은 편지함: 받은 편지함 그룹 콜백이 넣고 코어 그룹이 꺼낸다 (IR-30) ----
from __future__ import annotations

import threading
from collections import deque


class Inbox:
    """스레드 안전한 사건 줄. 넣는 쪽은 처리 함수와 인자만 넣고, 꺼낸 쪽이 넣은 순서대로 부른다."""

    def __init__(self):
        self._items: deque = deque()
        self._lock = threading.Lock()
        self.peak = 0  # 가장 길었던 줄 길이

    def put(self, handler, args: tuple) -> None:
        with self._lock:
            self._items.append((handler, args))
            self.peak = max(self.peak, len(self._items))

    def qsize(self) -> int:
        with self._lock:
            return len(self._items)

    def take(self) -> list:
        with self._lock:
            items = list(self._items)
            self._items.clear()
        return items
