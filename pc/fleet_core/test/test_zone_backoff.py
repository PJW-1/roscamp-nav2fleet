"""마당 물러나기(yard_backoff, 기본 꺼짐): 넓은 구역에서 비켜 서는 로봇이 BLOCKED를 보고했으면
세우기 전에 선 방향 그대로 뒤로 조금 물린다."""
import copy
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.traffic.zone.backoff import backoff_spot
from fleet_core.traffic.zone.robots import Robot
from zone_harness import Harness

SITE = load_fleet_map("d2")
ON = replace(load_params(), yard_backoff=True)


def stuck_pair(**changes):
    """P-1의 pinky1과 P-2의 pinky2가 0.25 m 떨어져 마당에서 Nav2 goal을 받고 서 있다.

    배차가 없는 pinky2가 비킬 쪽이다. pinky2는 pinky1을 마주 본다: 뒤(+x)는 비어 있다.
    """
    world = Harness(speed=0.0, params=replace(ON, **changes))
    world.step()
    world.call("WORK", "pinky1")
    world.open_call = world.call("WORK")  # 열린 호출: pinky1은 이미 배차가 있어 pinky2가 불린다
    world.step()
    world.step()
    assert world.moves("pinky1") and world.moves("pinky2")
    world.bots["pinky2"].yaw = math.pi
    return world


def reports(world, robot, state):
    """robot은 앞으로 state를 보고한다. 다른 로봇은 그대로다."""
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def yields(world):
    return [(e.robot, e.detail) for e in world.events if e.code == "HALL_YIELD"]


def backing(world):
    """pinky2가 BLOCKED를 보고해 물러나기를 받았다. 그 명령을 돌려준다."""
    reports(world, "pinky2", "BLOCKED")
    world.run(world.params.near_stuck_s)
    [back] = world.sent(RetreatCmd, "pinky2")
    return back


# ---- 내는 때 ----

def test_the_switch_is_off_by_default():
    assert load_params().yard_backoff is False


def test_with_the_switch_off_a_blocked_robot_only_stops():
    world = stuck_pair(yard_backoff=False)
    reports(world, "pinky2", "BLOCKED")
    world.run(world.params.near_stuck_s)
    assert yields(world) == [("pinky2", "stops 5 s for pinky1")]
    assert not world.sent(RetreatCmd)


def test_a_blocked_robot_that_yields_backs_straight_off_first():
    world = stuck_pair()
    p = world.params
    move = world.view("pinky2")["goal_id"]
    back = backing(world)
    assert CancelCmd("pinky2", move) in world.commands
    x, y = SITE.node_pose("P-2")
    assert (back.distance_m, back.speed_mps) == (p.yard_backoff_m, p.retreat_speed_mps)
    assert [v for point in back.path for v in point] == pytest.approx(
        [x, y, math.pi, x + p.yard_backoff_m, y, math.pi])
    assert yields(world) == [("pinky2", "backs 0.12 m, then stops 5 s for pinky1")]
    assert world.view("pinky2")["goal_id"] == back.goal_id


def test_after_backing_off_it_stands_aside_and_then_gets_its_way_again():
    world = stuck_pair()
    p = world.params
    target = world.moves("pinky2")[0].target_id
    backing(world)
    del world._state  # 물러나는 동안은 MOVING을 보고한다
    world.run(p.yard_backoff_m / p.retreat_speed_mps + 0.2)
    assert [e.detail for e in world.events if e.code == "HALL_BACKOFF_DONE"] == ["backed 0.12 m"]
    assert world.bots["pinky2"].x == pytest.approx(SITE.node_pose("P-2")[0] + p.yard_backoff_m)
    world.run(p.near_stuck_s - 0.5)
    assert len(world.moves("pinky2")) == 1, "it stands aside"
    world.run(1.0)
    moves = world.moves("pinky2")
    assert [m.target_id for m in moves] == [target, target]
    assert world.view("pinky2")["attempts"] == 0
    assert not world.problems


def test_a_robot_that_did_not_report_blocked_is_given_a_little_longer_and_then_only_stops():
    world = stuck_pair()
    p = world.params
    world.run(p.near_stuck_s + p.yard_backoff_wait_s - 0.2)
    assert yields(world) == []
    world.run(0.3)
    assert yields(world) == [("pinky2", "stops 5 s for pinky1")]
    assert not world.sent(RetreatCmd)
    assert "GOAL_STALLED" not in world.codes(), "the wait ends before the stall watch"


def test_blocked_reported_during_the_extra_wait_starts_the_back_off():
    world = stuck_pair()
    world.run(world.params.near_stuck_s + 0.2)
    assert yields(world) == []
    reports(world, "pinky2", "BLOCKED")
    world.step()
    assert len(world.sent(RetreatCmd, "pinky2")) == 1


def test_a_robot_near_two_others_is_backed_off_once():
    world = Harness(speed=0.0, params=ON)
    world.step()
    x, y = SITE.node_pose("P-2")
    third = world.bots["pinky3"]
    third.x, third.y = x, y + 0.25  # pinky2의 옆(북쪽) 0.25 m: pinky1과는 멀다
    world.call("WORK", "pinky1")
    world.call("OUTBOUND", "pinky3")
    world.call("WORK")  # pinky2가 불린다: 배차가 없어 두 쌍 모두에서 비킬 쪽이다
    world.run(0.5)
    assert all(world.moves(name) for name in world.bots)
    world.bots["pinky2"].yaw = math.pi
    world.run(world.params.near_stuck_s)  # 두 쌍 모두 pinky2의 BLOCKED를 기다린다
    assert yields(world) == []
    reports(world, "pinky2", "BLOCKED")
    world.step()
    [back] = world.sent(RetreatCmd, "pinky2")
    assert len(yields(world)) == 1
    assert CancelCmd("pinky2", back.goal_id) not in world.commands


def test_a_new_destination_during_the_back_off_lets_it_finish_and_then_picks_a_new_way():
    world = stuck_pair()
    p = world.params
    back = backing(world)
    del world._state
    world.traffic.on_dispatch_cancel(world.open_call)  # 부른 일이 없어졌다: 주차하러 간다
    world.step()
    assert world.view("pinky2")["goal_id"] == back.goal_id, "the back-off goes on"
    assert world.view("pinky2")["route"] == [], "the old way is dropped, not kept for later"
    world.run(p.yard_backoff_m / p.retreat_speed_mps + p.near_stuck_s + 0.5)
    assert CancelCmd("pinky2", back.goal_id) not in world.commands
    # 물러난 자리는 P-2에서 0.12 m다: 닿은 것으로 보는 거리 안이라 그대로 주차한 것이 된다
    assert world.phase("pinky2") == "PARKED"
    assert len(world.moves("pinky2")) == 1, "the old way to W_Q is not sent again"


# ---- 못 내는 때 ----

@pytest.mark.parametrize(("yaw", "why"), [
    (math.pi / 2, "no room behind"),  # 뒤가 남쪽 벽이다
    (0.0, "pinky1 behind"),  # pinky1을 등지고 있다
])
def test_a_robot_with_no_room_behind_only_stops(yaw, why):
    world = stuck_pair()
    world.bots["pinky2"].yaw = yaw
    reports(world, "pinky2", "BLOCKED")
    world.run(world.params.near_stuck_s)
    assert yields(world) == [("pinky2", f"stops 5 s for pinky1 (no back-off: {why})")]
    assert not world.sent(RetreatCmd)
    moves = len(world.moves("pinky2"))
    world.run(world.params.near_stuck_s + 0.5)
    assert len(world.moves("pinky2")) == moves + 1


def robot(name, x, y, yaw_deg=0.0):
    return Robot(name, placed=True, x=x, y=y, yaw=math.radians(yaw_deg))


def spot(m, others=(), held=("HALL",), nodes=()):
    return backoff_spot(SITE, ON, held, m, others, nodes)


def test_the_spot_is_straight_behind_the_robot():
    end, why = spot(robot("pinky1", 0.40, 0.50, -90.0))  # 남쪽을 본다: 뒤는 북쪽이다
    assert end == pytest.approx((0.40, 0.62)) and why == ""


@pytest.mark.parametrize(("x", "y", "yaw_deg"), [
    (0.40, 0.70, -90.0),  # 끝이 갈림목 구역(J_W) 안이다
    (0.40, 0.60, -90.0),  # 끝이 갈림목 구역에서 0.07 m다
    (1.00, 0.58, -90.0),  # 끝이 진열대에서 0.075 m다
    (0.15, 0.40, 0.0),  # 끝이 서쪽 벽에서 0.03 m다
])
def test_a_spot_outside_the_yard_or_too_close_to_its_edge_is_refused(x, y, yaw_deg):
    assert spot(robot("pinky1", x, y, yaw_deg)) == (None, "no room behind")


def test_a_way_that_crosses_a_shelf_is_refused_even_when_the_spot_is_clear_of_it():
    site = copy.copy(SITE)
    site.shelves = ((1.00, 0.30, 1.01, 0.50),)  # 얇은 진열대: 끝 자리는 0.105 m 떨어진다
    here = robot("pinky1", 0.995, 0.40, 180.0)  # 서쪽을 본다: 진열대 너머 (1.115, 0.40)로 물러난다
    assert backoff_spot(site, ON, ("HALL",), here, ()) == (None, "no room behind")
    clear = robot("pinky1", 1.02, 0.40, 180.0)  # 진열대를 등지지 않고 지나서 서 있다
    end, why = backoff_spot(site, ON, ("HALL",), clear, ())
    assert end == pytest.approx((1.14, 0.40)) and why == ""


def test_a_robot_that_does_not_hold_the_yard_it_stands_in_is_not_backed_off():
    here = robot("pinky1", 0.40, 0.50, -90.0)
    assert spot(here, held=()) == (None, "not in a yard it holds")
    on_the_express = robot("pinky1", 1.00, 0.89, 180.0)
    assert spot(on_the_express, held=("EXPRESS",)) == (None, "not in a yard it holds")


def test_a_robot_close_behind_refuses_the_spot_and_one_ahead_or_far_does_not():
    here = robot("pinky1", 1.00, 0.30, 90.0)  # 북쪽을 본다: (1.00, 0.18)로 물러난다
    assert spot(here, [robot("pinky3", 1.05, 0.10)]) == (None, "pinky3 behind")
    for x, y in [(1.00, 0.45), (1.16, 0.30), (1.00, -0.03), (1.30, 0.10)]:
        end, why = spot(here, [robot("pinky3", x, y)])
        assert end == pytest.approx((1.00, 0.18)) and why == "", (x, y)


def test_a_spot_by_a_node_the_robot_does_not_hold_is_refused():
    """물러나 설 자리가 제가 쥐지 않은 지점에서 node_clear_m 안이면 물러나지 않는다: 그 지점을
    지금 쥐었거나 나중에 잡을 로봇의 자리를 막는다."""
    here = robot("pinky1", 1.80, 0.65, 0.0)  # 동쪽을 본다: N_E (1.58, 0.65)에서 0.10 m로 물러난다
    end, why = spot(here)
    assert end == pytest.approx((1.68, 0.65)) and why == ""
    assert spot(here, nodes=("N_E",)) == (None, "too close to N_E")
    assert spot(here, nodes=("N_W",))[0] is not None, "a node far away does not matter"


def test_the_back_off_counts_every_node_the_robot_does_not_hold():
    """빈 지점도 센다: 물러난 뒤에 다른 로봇이 그 지점을 잡을 수 있다."""
    world = Harness(params=ON)
    world.report()
    traffic = world.traffic
    m = traffic._robots["pinky1"]
    m.x, m.y, m.yaw = 1.80, 0.65, 0.0
    assert traffic.grants.node_holder("N_E") == ""
    assert traffic._back_off(m) == "too close to N_E"
    assert not world.sent(RetreatCmd)


# ---- 끝난 뒤 ----

@pytest.mark.parametrize(("answer", "reason"), [
    (lambda traffic, goal: traffic.on_retreat_result("pinky2", goal, False, "OBSTACLE_BEHIND"),
     "OBSTACLE_BEHIND"),
    (lambda traffic, goal: traffic.on_goal_rejected("pinky2", goal), "REJECTED"),
    (lambda traffic, goal: traffic.on_goal_not_sent("pinky2", goal), "NOT_SENT"),
    (lambda traffic, goal: traffic.on_goal_lost("pinky2", goal), "LOST"),
])
def test_a_back_off_that_fails_is_not_an_attempt_and_the_robot_goes_on(answer, reason):
    world = stuck_pair()
    p = world.params
    back = backing(world)
    world.bots["pinky2"].cmd = None  # 로봇은 물러나지 않았다
    answer(world.traffic, back.goal_id)
    world.step()
    failed = [e.detail for e in world.events if e.code == "HALL_BACKOFF_FAILED"]
    assert failed == [f"{reason} after 0.00 m"]
    assert "GOAL_RETRY" not in world.codes("pinky2") and world.view("pinky2")["attempts"] == 0
    assert world.view("pinky2")["goal_id"] is None
    moves = len(world.moves("pinky2"))
    world.run(p.near_stuck_s + 0.5)
    assert len(world.moves("pinky2")) == moves + 1


def test_a_back_off_that_stops_moving_is_cancelled_without_counting_an_attempt():
    world = stuck_pair()
    p = world.params
    back = backing(world)
    del world._state
    x, y = SITE.node_pose("P-2")
    world.obstacles.append((x + p.yard_backoff_m, y, 0.08))  # 뒤에 무엇이 있다: 0.04 m 가서 선다
    blocked = world.codes("pinky2").count("BLOCKED_WAIT")
    world.run(p.stall_s + 1.5)
    assert CancelCmd("pinky2", back.goal_id) in world.commands
    failed = [e.detail for e in world.events if e.code == "HALL_BACKOFF_FAILED"]
    assert failed == ["STALLED after 0.04 m"]
    assert world.codes("pinky2").count("BLOCKED_WAIT") == blocked, "not handled as a blocked move"
    codes = world.codes("pinky2")
    assert "GOAL_RETRY" not in codes and "ZONE_MARKED_BLOCKED" not in codes
    assert world.view("pinky2")["attempts"] == 0
