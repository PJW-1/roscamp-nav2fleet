"""order goal의 피드백 거르기와 파라미터 덮어쓰기. 순수 파이썬이라 ROS 없이도 돈다."""
from fleet_core.common.params import load_params
from fleet_manager.order_goals import FeedbackFilter, apply_overrides


def test_the_first_feedback_passes_and_an_unchanged_one_does_not():
    gate = FeedbackFilter()
    assert gate.changed(("pinky1", 7), (0, False, ""))
    assert not gate.changed(("pinky1", 7), (0, False, ""))


def test_a_change_in_any_of_the_three_values_passes():
    gate = FeedbackFilter()
    key = ("pinky1", 7)
    gate.changed(key, (0, False, ""))
    assert gate.changed(key, (1, False, ""))
    assert gate.changed(key, (1, True, ""))
    assert gate.changed(key, (1, True, "HORIZON_INVALID: x"))
    assert not gate.changed(key, (1, True, "HORIZON_INVALID: x"))


def test_goals_are_gated_apart_and_forget_clears_one():
    gate = FeedbackFilter()
    gate.changed(("pinky1", 7), (1, False, ""))
    assert gate.changed(("pinky1", 8), (1, False, ""))
    gate.forget(("pinky1", 7))
    assert gate.changed(("pinky1", 7), (1, False, ""))


def test_unset_overrides_keep_the_file_values_and_set_ones_win():
    base = load_params()
    assert apply_overrides(base, None, None) == base
    out = apply_overrides(base, True, 3.5)
    assert out.use_order_model is True and out.lookahead_s == 3.5
    assert out.heartbeat_timeout_s == base.heartbeat_timeout_s
    assert apply_overrides(base, False, None).use_order_model is False
