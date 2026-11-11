"""상황 시험(지시서 6-2): 앞 로봇 고장, 통신 끊김과 재투입, 위치가 크게 튐."""
import pytest
from situation_helpers import ALL_LAYOUTS, SEEDS, check


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_a_stuck_robot_is_isolated_and_the_others_keep_working(layout, seed):
    outcome = check("front_robot_stuck", layout, seed)
    assert outcome.world.central.finished >= 1


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_comm_loss_then_removal_and_reinstate(layout, seed):
    check("comm_loss_reinstate", layout, seed)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_jumping_reports_are_logged_outside_the_zones_and_nothing_stops(layout, seed):
    check("pose_jumps", layout, seed)
