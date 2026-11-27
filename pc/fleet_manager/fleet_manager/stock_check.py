# ---- 담기 전 결품 확인 (W-40 · PI-28 · IR-40): CheckStock 3초, 다시 보내지 않는다 ----
# 답이 없거나 실패하면 담기를 시작하지 않고 잡아 둔다. 끝은 코어의 취소 · 포기로 난다.
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

LIMIT_S = 3.0  # IR-40 CheckStock 응답 한도


@dataclass
class _Check:
    command: object
    asked_at: float
    future: object = None


def shortfall(items: dict[str, int], available: dict[str, int]) -> dict[str, int]:
    """상품별 모자란 수. 답에 없는 상품은 가용 0으로 본다."""
    return {kind: count - available.get(kind, 0) for kind, count in items.items()
            if available.get(kind, 0) < count}


class StockChecks:
    """답을 기다리는 확인과 답 없이 잡아 둔 담기를 goal_id로 쥔다."""

    def __init__(self):
        self._asking: dict[int, _Check] = {}
        self._held: dict[int, object] = {}

    def __contains__(self, goal_id) -> bool:
        return goal_id in self._asking or goal_id in self._held

    def ask(self, command, future, now: float) -> None:
        self._asking[command.goal_id] = _Check(command, now, future)

    def answered(self, goal_id, future) -> _Check | None:
        """지금 기다리는 답이면 그 확인을 꺼낸다. 버린 확인 · 다른 요청의 답이면 None."""
        c = self._asking.get(goal_id)
        if c is None or c.future is not future:
            return None
        del self._asking[goal_id]
        return c

    def hold(self, command) -> None:
        self._held[command.goal_id] = command

    def expired(self, now: float) -> list[_Check]:
        """한도를 넘긴 확인을 꺼낸다. 부른 쪽이 future를 버리고 담기를 잡아 둔다.

        답이 이미 와서 처리를 기다리는 확인은 남긴다. 도착 시각으로 따로 판정한다.
        """
        late = [c for c in self._asking.values()
                if now - c.asked_at > LIMIT_S and not (c.future is not None and c.future.done())]
        for c in late:
            del self._asking[c.command.goal_id]
        return late

    def command(self, goal_id):
        c = self._asking.get(goal_id)
        return c.command if c is not None else self._held.get(goal_id)

    def take(self, goal_id):
        """확인을 버린다. (명령, 버릴 future)를 돌려준다."""
        c = self._asking.pop(goal_id, None)
        if c is not None:
            return c.command, c.future
        command = self._held.pop(goal_id, None)
        return (command, None) if command is not None else None

    def given_up(self, holds: Callable[[int], bool]) -> list[tuple[object, object]]:
        """코어가 이미 포기한 담기의 확인을 버린다. (명령, 버릴 future) 목록."""
        gone = [g for g in [*self._asking, *self._held] if not holds(g)]
        return [self.take(g) for g in gone]
