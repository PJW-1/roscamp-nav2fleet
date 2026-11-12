"""order goal(MoveOrderCmd로 보낸 MoveTo goal)을 다루는 순수 파이썬 도우미."""
from __future__ import annotations

from dataclasses import replace


class FeedbackFilter:
    """피드백은 자주 온다. goal마다 마지막 값을 기억해 바뀐 것만 코어로 넘긴다."""

    def __init__(self):
        self._last: dict = {}

    def changed(self, key, values: tuple) -> bool:
        if self._last.get(key) == values:
            return False
        self._last[key] = values
        return True

    def forget(self, key) -> None:
        self._last.pop(key, None)


def apply_overrides(params, use_order_model, lookahead_s):
    """ROS 파라미터로 준 값만 파일 값을 덮는다. None은 건드리지 않는다."""
    changes = {}
    if use_order_model is not None:
        changes["use_order_model"] = bool(use_order_model)
    if lookahead_s is not None:
        changes["lookahead_s"] = float(lookahead_s)
    return replace(params, **changes)
