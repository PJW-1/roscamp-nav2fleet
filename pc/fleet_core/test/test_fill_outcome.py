"""담기 실패의 다음 처리 (팔 관제 합의 ①, PI-21 · 29): 재시도는 NOT_READY만.
PICK_FAILED는 자리만, Phody가 당분간 보내지 않음."""
import pytest
from fleet_core.arm.fill_outcome import HOLD, PAUSE, RETRY, next_after_fill


@pytest.mark.parametrize(("reason", "then"), [
    ("NOT_READY", RETRY),
    ("PICK_FAILED", PAUSE),
    ("RECOGNITION_ERROR", PAUSE),
    ("OUT_OF_STOCK", PAUSE),
    ("ARM_FAULT", HOLD),
    ("UNKNOWN_TARGET", HOLD),  # 다시 보내지 않음
    ("MODE_MISMATCH", HOLD),  # 정하지 않은 사유는 보류
    ("", HOLD),
])
def test_each_reason_has_one_next_step(reason, then):
    assert next_after_fill(reason, {}, timed_out=False) == then


@pytest.mark.parametrize("reason", ["NOT_READY", "PICK_FAILED", "OUT_OF_STOCK"])
def test_an_item_held_in_the_gripper_is_always_held(reason):
    assert next_after_fill(reason, {"A": 1}, timed_out=False) == HOLD


def test_a_zero_count_is_no_uncertain_item():
    assert next_after_fill("PICK_FAILED", {"A": 0}, timed_out=False) == PAUSE


@pytest.mark.parametrize("reason", ["CANCELED", "NOT_READY", "PICK_FAILED"])
def test_a_result_after_a_timeout_is_held_without_a_retry(reason):
    assert next_after_fill(reason, {}, timed_out=True) == HOLD
