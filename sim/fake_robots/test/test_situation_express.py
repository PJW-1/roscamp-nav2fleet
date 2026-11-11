"""상황 시험(지시서 6-2): 급행 양 끝에서 동시에, 급행 안 장애물(오래)."""
import pytest
from situation_helpers import ALL_LAYOUTS, SEEDS, check


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_two_robots_at_both_ends_never_meet_in_the_express(layout, seed):
    outcome = check("express_both_ends", layout, seed)
    assert not [v for v in outcome.world.violations if v.code == "I2_LANE_DIRECTION"]


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_a_long_obstacle_in_the_express_is_left_by_retreat_without_deadlock(layout, seed):
    check("express_obstacle", layout, seed)
