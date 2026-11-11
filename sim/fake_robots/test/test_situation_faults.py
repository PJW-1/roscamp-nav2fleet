"""상황 시험(지시서 6-2): 앞 로봇 고장, 통신 끊김과 재투입, 위치가 크게 튐."""
import pytest
from situation_helpers import ALL_LAYOUTS, SEEDS, check


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_a_stuck_robot_is_isolated_and_the_others_keep_working(layout, seed):
    outcome = check("front_robot_stuck", layout, seed)
    world = outcome.world
    if layout != "c":  # c: 작업 도크로 가는 갈림목이 하나라, 거기서 멈춘 로봇이 막으면 끝이다
        assert world.central.finished == world.central.total
        assert world.metrics.count("ZONES_RETURNED") >= 1


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_comm_loss_then_removal_and_reinstate(layout, seed):
    check("comm_loss_reinstate", layout, seed)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_jumping_reports_leave_every_invariant_but_standing_intact(layout, seed):
    check("pose_jumps", layout, seed)
