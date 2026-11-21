# ---- 파라미터 덮어쓰기 ----
from __future__ import annotations

from dataclasses import replace


# ---- use_order_model, lookahead_s 값 덮어쓰기 ----
def apply_overrides(params, use_order_model, lookahead_s):
    changes = {}
    if use_order_model is not None:
        changes["use_order_model"] = bool(use_order_model)
    if lookahead_s is not None:
        changes["lookahead_s"] = float(lookahead_s)
    return replace(params, **changes)
