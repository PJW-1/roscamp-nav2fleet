# ---- 담기 실패의 다음 처리 (팔 관제 합의 ①, PI-21 · 29) ----
"""관제가 다시 보내는 것은 NOT_READY뿐이다. 결과로 끝난 실패는 담기 일시 중지,
팔 고장 · 쥔 상품 모름 · 시간 초과(결과 모름) · 모르는 상품 코드 · 정하지 않은 사유는 보류."""
from __future__ import annotations

RETRY = "RETRY"
PAUSE = "FILLING_PAUSE"
HOLD = "HELD"

NOT_READY = "NOT_READY"
# 관리자가 [담기 재개] · [보충 불가]를 고른다. 자동 일부 출고는 없다
PAUSE_REASONS = frozenset({"PICK_FAILED", "RECOGNITION_ERROR", "OUT_OF_STOCK"})


def next_after_fill(reason: str, uncertain: dict[str, int], timed_out: bool) -> str:
    # 같은 상품을 두 번 집지 않도록 결과 모름 · 쥔 상품 모름은 사유보다 먼저 본다
    if timed_out or any(count > 0 for count in uncertain.values()):
        return HOLD
    if reason == NOT_READY:
        return RETRY
    if reason in PAUSE_REASONS:
        return PAUSE
    return HOLD
