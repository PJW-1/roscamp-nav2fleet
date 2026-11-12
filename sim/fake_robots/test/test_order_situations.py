"""order 추가지시서 6-2의 상황: order 모델을 끄고 켜서 D2에서 씨드 셋으로 돌린다."""
import pytest
from fake_robots.world import CommLoss, OrderFault, ZoneWorld
from fake_robots.world.order_run import OrderRun
from fake_robots.world.order_situations import ORDER_SITUATIONS, run_order_situation
from fleet_core.common.commands import MoveOrderCmd

SEEDS = (1, 2, 3)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("order_model", (False, True))
@pytest.mark.parametrize("name", ORDER_SITUATIONS)
def test_every_order_situation_finishes_its_orders_with_no_violation(name, order_model, seed):
    outcome = run_order_situation(name, order_model, seed)
    assert not outcome.failures, "\n".join(outcome.failures)
    if order_model:
        assert outcome.world.order_metrics.outside_base == 0
        assert outcome.world.metrics.count("SEGMENT_BEYOND_BASE") == 0
    else:
        assert not outcome.world.commands_of(MoveOrderCmd)


def test_a_robot_that_loses_contact_with_a_horizon_drives_to_its_base_end_and_keeps_its_zones():
    world = ZoneWorld("d2", seed=1, overrides={"use_order_model": True},
                      injections=[CommLoss("pinky1", 2.0, 17.0)])
    world.run(2.0)
    run = world.robots["pinky1"].run
    assert isinstance(run, OrderRun) and run.horizon, "pinky1 has a horizon when it goes silent"
    end = run.last.end_node
    world.run(14.0)
    r = world.robots["pinky1"]
    assert r.run is run and run.waiting  # 끊긴 채 base 끝에 서서 기다린다
    assert world.site.nearest_node(r.x, r.y) == end
    assert set(world.traffic.robot_view("pinky1")["held"]) >= {world.site.nodes[end].zone}
    assert not world.violations
    assert world.order_metrics.outside_base == 0


def test_a_wrong_stitch_is_refused_and_the_fleet_sends_a_new_order_from_the_base_end():
    world = ZoneWorld("d2", seed=1, overrides={"use_order_model": True},
                      injections=[OrderFault(1.0, "stitch")])
    world.run_until(lambda: "ORDER_UPDATE_REJECTED" in world.event_codes(), timeout_s=300)
    t0, refused = next((t, e) for t, e in world.events if e.code == "ORDER_UPDATE_REJECTED")
    robot = refused.robot
    assert world.run_until(lambda: any(t > t0 and e.code == "ORDER_SENT" and e.robot == robot
                                       for t, e in world.events), timeout_s=120)
    assert not world.violations
