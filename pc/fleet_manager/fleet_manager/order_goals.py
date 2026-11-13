from __future__ import annotations

from dataclasses import replace


class FeedbackFilter:
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
    changes = {}
    if use_order_model is not None:
        changes["use_order_model"] = bool(use_order_model)
    if lookahead_s is not None:
        changes["lookahead_s"] = float(lookahead_s)
    return replace(params, **changes)
