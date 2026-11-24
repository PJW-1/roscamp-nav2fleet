"""stall_s가 로봇의 BLOCKED 보고보다 짧으면 경고 문장을 낸다(오류가 아니다). 로봇 값은 계약에 없어 기본값(ROBOT_BLOCKED_AFTER_S)과 비교한다."""
from dataclasses import replace

from fleet_core.common.params import (
    BLOCKED_REPORT_MARGIN_S,
    ROBOT_BLOCKED_AFTER_S,
    load_params,
    timing_warnings,
)

P = load_params()


def test_the_constants():
    assert (ROBOT_BLOCKED_AFTER_S, BLOCKED_REPORT_MARGIN_S) == (10.0, 2.0)


def test_the_shipped_params_give_no_warning():
    assert timing_warnings(P) == []


def test_a_6_s_stall_gives_one_warning_with_the_numbers():
    warnings = timing_warnings(replace(P, stall_s=6.0))
    assert len(warnings) == 1
    assert "stall_s" in warnings[0] and "6" in warnings[0] and "12" in warnings[0]


def test_the_limit_itself_is_enough():
    assert timing_warnings(replace(P, stall_s=12.0)) == []
    assert len(timing_warnings(replace(P, stall_s=11.9))) == 1


def test_dock_stall_s_is_not_compared():
    """DOCK goal은 BLOCKED여도 멈춤 시계가 돈다(watch.py:27)."""
    assert timing_warnings(replace(P, dock_stall_s=1.0)) == []
