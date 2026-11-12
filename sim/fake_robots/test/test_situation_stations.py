"""상황 시험(지시서 6-2): 출고 도크 오래 씀, 주문 몰림, 세 대가 갈림목에 동시에."""
import pytest
from situation_helpers import ALL_LAYOUTS, SEEDS, check


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_a_slow_outbound_arm_keeps_the_others_waiting_off_the_express(layout, seed):
    outcome = check("outbound_slow", layout, seed)
    assert not [v for v in outcome.world.violations if v.code == "I5_STANDING"]


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_ten_orders_in_a_burst_all_finish(layout, seed):
    outcome = check("order_burst", layout, seed)
    assert outcome.world.central.finished == 10


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_three_robots_at_the_junction_keep_its_capacity_of_one(layout, seed):
    outcome = check("three_at_junction", layout, seed)
    assert not [v for v in outcome.world.violations if v.code == "I1_CAPACITY"]
