"""상황 시험: 앞 로봇 고장, 통신 끊김과 재투입, 위치가 크게 튐."""
import pytest
from situation_helpers import (
    ALL_LAYOUTS,
    POSE_JUMPS_OPEN,
    SEEDS,
    PoseJumpsOpen,
    check,
    check_pose_jumps,
)


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
    """v2.4: 치울 때 바구니에 담긴 것이 있으면 그 주문은 HELD ROBOT_FAULT로 한 번 보고하고
    되돌리지 않는다(central이 닫는다). 비었거나 이미 출고했으면 예전처럼 다 끝난다."""
    central = check("comm_loss_reinstate", layout, seed).world.central
    held = central.hold_reports
    assert len({r.order_id for r in held}) == len(held)  # 보류 보고는 주문마다 한 번
    for r in held:
        assert (r.robot, r.hold_reason) == ("pode2", "ROBOT_FAULT") and r.placed, r
    assert central.closed_by_hand == [r.order_id for r in held]
    finals = [r.order_id for r in central.reports]
    assert len(finals) == len(set(finals)) and not set(finals) & set(central.closed_by_hand)
    assert central.finished + len(held) == central.total


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_jumping_reports_leave_every_invariant_but_standing_intact(layout, seed):
    """끈 길을 지킨다(order 모델 끔). 켠 길은 아래 xfail이다."""
    check("pose_jumps", layout, seed, overrides={"use_order_model": False})


@pytest.mark.xfail(strict=True, raises=PoseJumpsOpen, reason=POSE_JUMPS_OPEN)
@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_jumping_reports_with_the_order_model_on(layout, seed):
    check_pose_jumps(layout, seed)
