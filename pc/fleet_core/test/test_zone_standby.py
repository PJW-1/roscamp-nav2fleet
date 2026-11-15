"""작업대로 불린 로봇이 대기 자리가 없어 주차하러 가는 길(standby 트립).
- 가는 동안 대기 자리가 비면 주차 칸에 들어가지 않고 그 자리로 간다
  (2026-11-15 G5: 12건 가운데 9번, 자리가 빈 뒤에도 합계 81초를 주차하러 더 갔다).
  고정 경로 위에서는 그 구간 끝에서, 마당 goal로 가는 중이면 goal을 취소하고 돌린다.
- 자리가 끝내 비지 않으면 지금처럼 주차한다.
- 양보하려고 주차하러 가는 로봇(DEADLOCK_SEEN, HALL_YIELD)은 돌리지 않는다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, MoveOrderCmd, RetreatCmd
from fleet_core.common.params import load_params
from zone_harness import Harness

ORDER_MODEL = pytest.mark.parametrize("order_model", [False, True])


def world_with_a_busy_line(order_model=False):
    """pinky2는 W에서 일하고, pinky1은 O에서 출고를 끝냈고(아직 쥐고 있다),
    pinky3은 불려서 Q2로 간다."""
    world = Harness(layout="v7", starts={"pinky1": "O", "pinky2": "W", "pinky3": "P-3"},
                    params=replace(load_params(), use_order_model=order_model))
    world.report()
    at_work, at_out = world.call("WORK", "pinky2"), world.call("OUTBOUND", "pinky1")
    assert world.run_until(lambda: world.result_of(at_work) and world.result_of(at_out), 30.0)
    for _ in range(3):
        world.call("WORK")
    assert world.run_until(lambda: world.traffic.grants.node_holder("Q2") == "pinky3", 30.0)
    return world


def backed_into_a_bay(world, robot) -> bool:
    orders = [c for c in world.commands if isinstance(c, MoveOrderCmd) and c.robot == robot]
    return bool(world.sent(RetreatCmd, robot)) or any(
        segment.mode == "retreat" for c in orders for segment in (*c.base, *c.horizon))


@ORDER_MODEL
def test_a_spot_that_frees_on_the_way_to_parking_turns_the_robot_to_the_line(order_model):
    world = world_with_a_busy_line(order_model)
    assert world.run_until(lambda: world.bots["pinky3"].x >= 1.3, 60.0)
    world.traffic.on_release("pinky1")
    world.step()
    trip = world.traffic._robots["pinky1"].trip
    assert trip.goal in world.site.parking and trip.standby, "no spot yet: heads for a bay"
    assert world.run_until(lambda: world.view("pinky1")["node"] == "Q2", 120.0), world.log[-30:]
    assert world.sent(CancelCmd, "pinky1"), "the yard goal to the bay is canceled"
    trip = world.traffic._robots["pinky1"].trip
    assert trip is None or not trip.standby
    assert not world.problems


def test_a_spot_that_frees_on_the_fixed_path_out_of_the_dock_is_taken_at_its_end():
    world = world_with_a_busy_line()
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q2", 60.0)
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.view("pinky1")["node"] == "Q2", 120.0), world.log[-30:]
    assert not world.sent(CancelCmd, "pinky1"), "a fixed path is driven to its end"
    assert not backed_into_a_bay(world, "pinky1")
    assert "E-1" not in [c.target_id for c in world.sent(MoveCmd, "pinky1")]
    assert not world.problems


@ORDER_MODEL
def test_with_no_spot_freeing_the_robot_still_parks(order_model):
    world = world_with_a_busy_line(order_model)
    world.silent.add("pinky3")  # Q2를 쥔 채 서 있다: 자리가 비지 않는다
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.view("pinky1")["node"] in world.site.parking, 120.0)
    assert backed_into_a_bay(world, "pinky1")


def test_a_free_spot_it_cannot_be_granted_leaves_no_waiting_record(monkeypatch):
    """자리는 비었지만 가는 길의 구역을 아직 잡지 못하면 주차하러 가던 길을 그대로 간다.
    길에서 자리를 알아본 것은 기다린 것이 아니다(grants.waiting에 남지 않는다)."""
    world = world_with_a_busy_line()
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q2", 60.0)
    world.traffic.on_release("pinky1")
    world.step()
    grants, m = world.traffic.grants, world.traffic._robots["pinky1"]
    assert m.trip.standby and m.goal is not None
    real = grants.take

    def take(robot, zones=(), nodes=(), lanes=None):
        if robot == "pinky1" and "Q2" in nodes:
            zones = [*zones, "WORK_DOCK"]
        return real(robot, zones, nodes, lanes)

    monkeypatch.setattr(grants, "take", take)
    tried, seen = set(), set()
    for _ in range(1200):
        if world.view("pinky1")["node"] in world.site.parking:
            break
        world.step()
        if m.goal is not None:
            seen |= set(grants.waiting().get("pinky1", ()))
        if grants.node_holder("Q2") in ("", "pinky1"):
            tried.add("Q2")
    assert world.view("pinky1")["node"] in world.site.parking
    assert tried == {"Q2"}, "the spot was free on the way"
    assert seen == set()
    assert "STANDBY_LEFT" not in world.codes("pinky1")


def test_a_robot_that_gives_way_to_a_bay_is_not_turned_back():
    world = Harness(layout="v7", starts={"pinky1": "Q2", "pinky2": "W", "pinky3": "Q1"})
    world.report()
    at_work = world.call("WORK", "pinky2")
    for _ in range(3):
        world.call("WORK")
    assert world.run_until(lambda: world.result_of(at_work), 30.0)
    m = world.traffic._robots["pinky1"]
    assert m.phase == "TO_WORK" and m.goal is None and m.node == "Q2"
    assert world.traffic._evade(m)
    assert not m.trip.standby
    assert world.run_until(lambda: world.view("pinky1")["node"] in world.site.parking, 120.0)
    assert backed_into_a_bay(world, "pinky1")
