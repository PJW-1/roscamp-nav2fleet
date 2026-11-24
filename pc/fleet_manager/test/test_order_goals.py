"""order goal의 파라미터 덮어쓰기 시험(ROS 없이 돈다)."""
from fleet_core.common.params import load_params
from fleet_manager.order_goals import apply_overrides


def test_unset_overrides_keep_the_file_values_and_set_ones_win():
    base = load_params()
    assert apply_overrides(base, None, None) == base
    out = apply_overrides(base, True, 3.5)
    assert out.use_order_model is True and out.lookahead_s == 3.5
    assert out.heartbeat_timeout_s == base.heartbeat_timeout_s
    assert apply_overrides(base, False, None).use_order_model is False
