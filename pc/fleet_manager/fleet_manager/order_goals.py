# ---- 피드백 중복 걸러내기와 파라미터 덮어쓰기 ----
from __future__ import annotations

from dataclasses import replace


# ---- 값이 바뀔 때만 통과시키는 피드백 필터 ----
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


# ---- use_order_model, lookahead_s 값 덮어쓰기 ----
def apply_overrides(params, use_order_model, lookahead_s):
    changes = {}
    if use_order_model is not None:
        changes["use_order_model"] = bool(use_order_model)
    if lookahead_s is not None:
        changes["lookahead_s"] = float(lookahead_s)
    return replace(params, **changes)
