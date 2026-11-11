"""order 모델(use_order_model): 조각 목록, base와 horizon, update, 미리 늘리기, 거절, 끊긴 로봇."""
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, MoveOrderCmd, RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.zone import move_orders
from zone_harness import Harness

ON = replace(load_params(), use_order_model=True)


def ends(segments):
    return [(s.seq_id, s.mode, s.end_node) for s in segments]


def orders(world, robot):
    return world.sent(MoveOrderCmd, robot)


def queued_for_outbound(release_at=None):
    """pinky2가 출고 도크를 쓰는 동안 pinky1이 작업 도크에서 출고 도크로 불린다."""
    world = Harness(starts={"pinky1": "W", "pinky2": "O", "pinky3": "P-3"}, params=ON)
    world.report()
    world.call("OUTBOUND", "pinky2")  # 도크에 서 있으므로 곧바로 도킹한다
    return world, world.call("OUTBOUND", "pinky1")


def waiting_at_the_queue_spot():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pinky1"), 60.0)
    return world, call


def base_zones_are_held(world, robot):
    """그 로봇에게 준 base 조각의 구역을 모두 쥐고 있는가(마지막 명령 기준)."""
    held = set(world.view(robot)["held"])
    return all(set(segment.zones) <= held for segment in orders(world, robot)[-1].base)


# ---- order 만들기 ----

def test_an_order_carries_the_whole_trip_and_its_base_is_what_was_taken():
    world = Harness(params=ON)
    world.step()
    call = world.call("WORK")
    world.step()
    [order] = orders(world, "pinky2")
    assert (order.update_id, order.stitch_node, order.horizon) == (0, "", ())
    assert ends(order.base) == [(1, "nav_goal", "N_W"), (2, "follow_path", "W")]
    assert [s.controller_id for s in order.base] == ["", "FollowPath"]
    assert [(x, y) for x, y, _ in order.base[1].path] == [(0.40, 0.65), (0.40, 0.89), (0.18, 0.89)]
    assert world.view("pinky2")["held"] == ["HALL", "J_W", "WORK_DOCK"]
    assert base_zones_are_held(world, "pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    codes = world.codes("pinky2")
    assert codes.count("ORDER_SENT") == 1 and codes.count("SEGMENT_DONE") == 2
    assert not world.sent(MoveCmd) and not world.sent(RetreatCmd), "the order replaces them"
    assert not world.problems


def test_a_retreat_out_of_the_dock_is_the_first_segment():
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"}, params=ON)
    world.report()
    call = world.call("OUTBOUND", "pinky1")
    world.step()
    [order] = orders(world, "pinky1")
    assert ends(order.base) == [(1, "retreat", "X_W"), (2, "follow_path", "O")]
    assert order.base[0].controller_id == "FollowPathReverse"
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    codes = world.codes("pinky1")
    assert "RETREAT_START" in codes and "RETREAT_DONE" in codes and not world.problems


def test_a_purpose_changed_inside_a_fixed_stretch_ends_the_order_in_the_yard():
    """작업 도크에서 주차 자리로 나가는 order의 후진 중에 출고 도크로 불린다."""
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"}, params=ON)
    world.report()
    assert world.run_until(lambda: orders(world, "pinky1"), 10.0)
    assert ends(orders(world, "pinky1")[0].base)[0] == (1, "retreat", "X_W")
    world.run(1.0)
    call = world.call("OUTBOUND", "pinky1")
    assert world.run_until(lambda: len(orders(world, "pinky1")) == 2, 30.0)
    bot = world.bots["pinky1"]
    assert math.dist((bot.x, bot.y), world.site.node_pose("N_W")) < 0.05, (
        "the order ends where the fixed stretch ends, not at the parking spot")
    assert world.view("pinky1")["destination"] == "OUTBOUND"
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems


# ---- 도크가 찼을 때: base는 대기 자리까지, 도크까지는 horizon ----

def test_with_the_dock_busy_the_base_ends_at_the_wait_spot_and_the_rest_is_horizon():
    world, call = queued_for_outbound()
    world.step()
    order = orders(world, "pinky1")[-1]
    assert ends(order.base) == [(1, "retreat", "X_W"), (2, "follow_path", "N_W"),
                                (3, "nav_goal", "O_Q")]
    assert ends(order.horizon) == [(4, "nav_goal", "N_E"), (5, "follow_path", "O")]
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pinky1"), 60.0)
    world.run(5.0)
    view = world.view("pinky1")
    assert (view["node"], view["held"]) == ("O_Q", ["HALL"]), "nothing of the next stretch is taken"
    assert view["base"] == [] and view["horizon"] == ["N_E", "X_E", "O"]
    assert "GOAL_STALLED" not in world.codes("pinky1"), "standing at the base end is not a stall"
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    update = orders(world, "pinky1")[-1]
    assert (update.order_id, update.update_id, update.stitch_node) == (order.order_id, 1, "O_Q")
    assert ends(update.base) == [(4, "nav_goal", "N_E"), (5, "follow_path", "O")]
    assert update.horizon == ()
    assert world.codes("pinky1").count("ORDER_UPDATED") == 1
    assert not world.problems and not [e for e in world.events if e.level == "ERROR"]


def test_the_base_is_always_inside_the_zones_the_robot_holds():
    world, call = queued_for_outbound()
    seen = 0
    for _ in range(1200):
        world.step()
        if len(orders(world, "pinky1")) > seen:
            seen = len(orders(world, "pinky1"))
            assert base_zones_are_held(world, "pinky1"), world.log[-5:]
        if world.now > 30.0 and world.view("pinky2")["state"] == "AT_STATION":
            world.traffic.on_release("pinky2")
        if world.result_of(call) is not None:
            break
    assert world.result_of(call).success and seen == 2


def test_a_dock_freed_before_the_base_end_is_taken_whole_and_the_robot_does_not_stop():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: world.view("pinky1")["target"] == "O_Q"
                           and world.view("pinky1")["node"] == "N_W", 60.0)
    world.traffic.on_release("pinky2")  # 아직 대기 자리에 닿기 전이다
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    codes = world.codes("pinky1")
    assert "ORDER_UPDATED" in codes and "BASE_END_WAIT" not in codes
    granted = [e.detail for e in world.events
               if e.robot == "pinky1" and e.code == "ZONE_GRANTED"]
    assert granted[-1] == "HALL+J_E+OUT_DOCK to O", "the whole stretch at once"
    assert not world.problems


# ---- horizon만 바꾸기, 거절 ----

def test_a_changed_route_ahead_replaces_only_the_horizon():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pinky1"), 60.0)
    held = world.view("pinky1")["held"]
    world.traffic._blocked[("N_E", "X_E")] = world.now + 300.0  # 출고 도크 진입로가 막혔다
    world.run(6.0)
    change = orders(world, "pinky1")[-1]
    assert (change.update_id, change.stitch_node, change.base) == (1, "O_Q", ())
    assert ends(change.horizon) == [(4, "nav_goal", "N_W"), (5, "follow_path", "O")]
    assert world.codes("pinky1").count("HORIZON_CHANGED") == 1
    assert world.view("pinky1")["held"] == held, "the base stays as it was"
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert orders(world, "pinky1")[-1].update_id == 2 and not world.problems


def test_a_refused_update_is_followed_by_a_new_order_from_the_base_end():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pinky1"), 60.0)
    world.traffic.order_fault = "stitch"  # 다음 update의 이음 지점을 일부러 틀리게 보낸다
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: "ORDER_UPDATE_REJECTED" in world.codes("pinky1"), 60.0)
    assert world.view("pinky1")["node"] == "O_Q", "the robot stays at the end of its base"
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    codes = world.codes("pinky1")
    assert "GOAL_STALLED" not in codes and "GOAL_RETRY" not in codes, "not a failed attempt"
    first, bad, again = orders(world, "pinky1")[-3:]
    assert (bad.order_id, bad.update_id, bad.stitch_node) == (first.order_id, 1, "?")
    assert again.update_id == 0 and again.order_id != first.order_id
    assert ends(again.base) == [(1, "nav_goal", "N_E"), (2, "follow_path", "O")]
    assert world.result_of(call).success and not world.problems


# ---- update의 답이 오기 전 ----

def test_no_second_update_goes_out_before_the_robot_answers_the_first(monkeypatch):
    monkeypatch.setattr(move_orders, "UPDATE_ANSWER_S", 60.0)
    world, call = waiting_at_the_queue_spot()
    world.traffic.on_release("pinky2")
    world.run(1.0)
    world.hold_answers = True
    world.traffic._blocked[("N_E", "X_E")] = world.now + 300.0  # horizon이 바뀐다
    world.run(15.0)  # 그사이 출고 도크가 비어 다음 구간을 잡을 수 있게 된다
    assert world.view("pinky2")["held"] == ["HALL"]
    assert [o.update_id for o in orders(world, "pinky1")] == [0, 1], "update 1 is unanswered"
    world.hold_answers = False
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert [o.update_id for o in orders(world, "pinky1")] == [0, 1, 2]
    assert world.result_of(call).success and not world.problems


def test_an_update_nobody_answers_ends_both_goals_and_a_new_order_follows():
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pinky1")[-1]
    world.hold_answers = True
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: len(orders(world, "pinky1")) == 2, 30.0)
    update = orders(world, "pinky1")[-1]
    world.run(move_orders.UPDATE_ANSWER_S + 0.5)
    cancelled = {c.goal_id for c in world.sent(CancelCmd, "pinky1")}
    assert {first.goal_id, update.goal_id} <= cancelled, "the old goal may still be the live one"
    assert "ORDER_UPDATE_REJECTED" in world.codes("pinky1")
    world.hold_answers = False
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert orders(world, "pinky1")[-1].update_id == 0
    assert world.result_of(call).success and not world.problems


@pytest.mark.parametrize(("fault", "reached_unknown"), [
    ("on_goal_not_sent", False),  # 서버가 준비되지 않았다. 닿지 않은 것이 확실하다
    ("on_goal_rejected", True),  # 거절이거나 결과를 잃었다. 붙었는지 알 수 없다
    ("on_goal_lost", True),
])
def test_an_update_goal_that_failed_on_the_way_is_followed_by_a_new_order(fault, reached_unknown):
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pinky1")[-1]
    world.goal_fault["pinky1"] = fault
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: "ORDER_UPDATE_REJECTED" in world.codes("pinky1"), 60.0)
    update = orders(world, "pinky1")[-1]
    world.step()
    cancelled = {c.goal_id for c in world.sent(CancelCmd, "pinky1")}
    assert (update.goal_id in cancelled) == reached_unknown, "both goals end when it is unclear"
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    codes = world.codes("pinky1")
    assert "GOAL_RETRY" not in codes and "GOAL_STALLED" not in codes
    again = orders(world, "pinky1")[-1]
    assert again.update_id == 0 and again.order_id != first.order_id
    assert first.goal_id in {c.goal_id for c in world.sent(CancelCmd, "pinky1")}
    assert world.result_of(call).success and not world.problems


def test_the_old_goal_failing_before_the_update_is_answered_counts_as_its_failure():
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pinky1")[-1]
    world.hold_answers = True
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: len(orders(world, "pinky1")) == 2, 30.0)
    world.traffic.on_move_result("pinky1", first.goal_id, False, "NAV_ABORTED", 3)
    world.step()
    assert "GOAL_RETRY" in world.codes("pinky1"), "the running order failed, the fleet saw it"
    assert world.view("pinky1")["goal_id"] is None


# ---- 통신이 끊긴 로봇 ----

def test_a_silent_robot_drives_to_its_base_end_and_its_zones_stay_until_then():
    world = Harness(params=ON)
    world.step()
    call = world.call("WORK")
    world.run(2.0)
    world.silent.add("pinky2")
    assert world.run_until(lambda: world.view("pinky2")["state"] == "COMM_LOST", 10.0)
    held = world.view("pinky2")["held"]
    assert held == ["HALL", "J_W", "WORK_DOCK"]
    lost = next(e for e in world.events if e.code == "COMM_LOST")
    assert "drives on to W" in lost.detail
    assert world.traffic.confirm_clear("pinky2", held, "carried away") is None
    assert "base" in world.traffic.clear_reason
    world.run(30.0)  # base 끝에 닿을 예상 시각과 여유가 지났다
    assert (world.bots["pinky2"].x, world.bots["pinky2"].y) == (0.18, 0.89), "it drove on"
    assert world.view("pinky2")["held"] == held, "nothing is released by itself"
    world.silent.discard("pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    assert world.result_of(call).robot_id == "pinky2" and not world.problems


def test_the_base_time_also_guards_a_robot_that_has_just_started_talking_again():
    world = Harness(params=ON)
    world.step()
    world.call("WORK")
    world.run(2.0)
    world.silent.add("pinky2")
    assert world.run_until(lambda: world.view("pinky2")["state"] == "COMM_LOST", 10.0)
    held = world.view("pinky2")["held"]
    world.silent.discard("pinky2")  # 말이 돌아왔다. 취소가 닿았는지는 아직 모른다
    world.step()
    assert world.view("pinky2")["comm"] == "RECOVERING"
    assert world.traffic.confirm_clear("pinky2", held, "carried away") is None
    assert "base" in world.traffic.clear_reason
    world.run(world.params.recover_check_s + 0.5)
    assert world.view("pinky2")["comm"] == "", "the check is over"
    held = world.view("pinky2")["held"]
    assert world.traffic.confirm_clear("pinky2", held, "carried away") is None, (
        "whether the cancel reached the robot is still unknown")
    assert "base" in world.traffic.clear_reason
