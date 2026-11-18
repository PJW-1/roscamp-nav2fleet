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
    """pode2가 출고 도크를 쓰는 동안 pode1이 작업 도크에서 출고 도크로 불린다."""
    world = Harness(starts={"pode1": "W", "pode2": "O", "pode3": "P-3"}, params=ON)
    world.report()
    world.call("OUTBOUND", "pode2")  # 도크에 서 있으므로 곧바로 도킹한다
    return world, world.call("OUTBOUND", "pode1")


def waiting_at_the_queue_spot():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
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
    [order] = orders(world, "pode2")
    assert (order.update_id, order.stitch_node, order.horizon) == (0, "", ())
    assert ends(order.base) == [(1, "nav_goal", "N_W"), (2, "follow_path", "W")]
    assert [s.controller_id for s in order.base] == ["", "FollowPath"]
    assert [(x, y) for x, y, _ in order.base[1].path] == [(0.35, 0.65), (0.35, 0.89), (0.18, 0.89)]
    assert world.view("pode2")["held"] == ["HALL", "J_W", "WORK_DOCK"]
    assert base_zones_are_held(world, "pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    codes = world.codes("pode2")
    assert codes.count("ORDER_SENT") == 1 and codes.count("SEGMENT_DONE") == 2
    assert not world.sent(MoveCmd) and not world.sent(RetreatCmd), "the order replaces them"
    assert not world.problems


def test_a_retreat_out_of_the_dock_is_the_first_segment():
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, params=ON)
    world.report()
    call = world.call("OUTBOUND", "pode1")
    world.step()
    [order] = orders(world, "pode1")
    assert ends(order.base) == [(1, "retreat", "N_W"), (2, "follow_path", "O")]
    assert order.base[0].controller_id == "FollowPathReverse"
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    codes = world.codes("pode1")
    assert "RETREAT_START" in codes and "RETREAT_DONE" in codes and not world.problems


def test_an_order_segment_that_finishes_forgets_how_long_something_was_behind():
    world = Harness(params=ON)
    world.step()
    world.call("WORK", "pode2")
    world.run(0.5)
    m = world.traffic._robots["pode2"]
    m.avoid.behind_since = 0.0
    assert world.run_until(lambda: world.view("pode2")["node"] != "P-2", 30.0)
    assert m.avoid.behind_since is None


def test_an_order_retreat_that_finishes_forgets_how_long_something_was_behind():
    """뒤가 막혀 기다리기 시작한 시각을 order의 후진 조각이 끝날 때 지우지
    않으면, 다음 후진에서 뒤가 막혔을 때 옛 시각부터 blocked_timeout_s를 세어 바로 실패로 친다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, params=ON)
    world.report()
    world.call("OUTBOUND", "pode1")
    world.step()
    [order] = orders(world, "pode1")
    m = world.traffic._robots["pode1"]
    world.traffic.on_move_result("pode1", order.goal_id, False, "OBSTACLE_BEHIND", 0)
    assert m.avoid.behind_since is not None
    assert world.run_until(lambda: "RETREAT_DONE" in world.codes("pode1"), 30.0)
    assert "RETREAT_WAIT" in world.codes("pode1")
    assert m.avoid.behind_since is None


def test_a_purpose_changed_inside_a_fixed_stretch_ends_the_order_in_the_yard():
    """작업 도크에서 주차 자리로 나가는 order의 후진 중에 출고 도크로 불린다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, params=ON)
    world.report()
    assert world.run_until(lambda: orders(world, "pode1"), 10.0)
    assert ends(orders(world, "pode1")[0].base)[0] == (1, "retreat", "N_W")
    world.run(1.0)
    call = world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: len(orders(world, "pode1")) == 2, 30.0)
    bot = world.bots["pode1"]
    assert math.dist((bot.x, bot.y), world.site.node_pose("N_W")) < 0.05, (
        "the order ends where the fixed stretch ends, not at the parking spot")
    assert world.view("pode1")["destination"] == "OUTBOUND"
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems


# ---- 도크가 찼을 때: base는 대기 자리까지, 도크까지는 horizon ----

def test_with_the_dock_busy_the_base_ends_at_the_wait_spot_and_the_rest_is_horizon():
    world, call = queued_for_outbound()
    world.step()
    order = orders(world, "pode1")[-1]
    assert ends(order.base) == [(1, "retreat", "N_W"), (2, "nav_goal", "O_Q")]
    assert ends(order.horizon) == [(3, "nav_goal", "N_E"), (4, "follow_path", "O")]
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    world.run(5.0)
    view = world.view("pode1")
    assert (view["node"], view["held"]) == ("O_Q", ["HALL"]), "nothing of the next stretch is taken"
    assert view["base"] == [] and view["horizon"] == ["N_E", "X_E", "O"]
    status = world.traffic.status_view("pode1")
    assert (status["base"], status["horizon"]) == ((), ("N_E", "X_E", "O")), "for the fleet screen"
    assert "GOAL_STALLED" not in world.codes("pode1"), "standing at the base end is not a stall"
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    update = orders(world, "pode1")[-1]
    assert (update.order_id, update.update_id, update.stitch_node) == (order.order_id, 1, "O_Q")
    assert ends(update.base) == [(3, "nav_goal", "N_E"), (4, "follow_path", "O")]
    assert update.horizon == ()
    assert world.codes("pode1").count("ORDER_UPDATED") == 1
    assert not world.problems and not [e for e in world.events if e.level == "ERROR"]


def test_a_report_of_a_segment_beyond_the_base_is_not_believed():
    world, _ = queued_for_outbound()
    world.step()
    goal_id = world.view("pode1")["goal_id"]
    world.traffic.on_move_feedback("pode1", goal_id, 4, False)  # 3번과 4번은 horizon이다
    view = world.view("pode1")
    assert view["node"] != "O" and view["horizon"] == ["N_E", "X_E", "O"]
    assert view["base"] == ["N_W", "O_Q"], "nothing of the base counts as done either"
    world.step()
    codes = world.codes("pode1")
    assert codes.count("SEGMENT_BEYOND_BASE") == 1 and "SEGMENT_DONE" not in codes
    world.traffic.on_move_feedback("pode1", goal_id, 5, False)
    world.step()
    assert world.codes("pode1").count("SEGMENT_BEYOND_BASE") == 1, "once for an order"


def test_the_old_goal_cannot_report_what_only_the_unanswered_update_allows(monkeypatch):
    monkeypatch.setattr(move_orders, "UPDATE_ANSWER_S", 60.0)
    world, _ = waiting_at_the_queue_spot()
    old = world.view("pode1")["goal_id"]
    world.hold_answers = True
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: len(orders(world, "pode1")) == 2, 30.0)
    world.traffic.on_move_feedback("pode1", old, 5, True)  # 4번과 5번은 update의 base다
    world.step()
    assert world.view("pode1")["node"] == "O_Q"
    assert "SEGMENT_BEYOND_BASE" in world.codes("pode1")


def test_after_a_refused_update_the_old_goal_is_held_to_its_own_base():
    world, _ = queued_for_outbound()
    assert world.run_until(lambda: world.view("pode1")["target"] == "O_Q"
                           and world.view("pode1")["node"] == "N_W", 60.0)
    old = world.view("pode1")["goal_id"]
    world.traffic.order_fault = "stitch"
    world.traffic.on_release("pode2")  # 달리는 중에 다음 구간이 열려 update가 나가고, 거절된다
    assert world.run_until(lambda: "ORDER_UPDATE_REJECTED" in world.codes("pode1"), 60.0)
    assert world.view("pode1")["goal_id"] == old, "the robot goes on under the old goal"
    world.traffic.on_move_feedback("pode1", old, 5, False)  # 4번과 5번은 붙지 못한 update의 것이다
    world.step()
    assert world.view("pode1")["node"] != "O"
    assert "SEGMENT_BEYOND_BASE" in world.codes("pode1")


def test_the_base_is_always_inside_the_zones_the_robot_holds():
    world, call = queued_for_outbound()
    seen = 0
    for _ in range(1200):
        world.step()
        if len(orders(world, "pode1")) > seen:
            seen = len(orders(world, "pode1"))
            assert base_zones_are_held(world, "pode1"), world.log[-5:]
        if world.now > 30.0 and world.view("pode2")["state"] == "AT_STATION":
            world.traffic.on_release("pode2")
        if world.result_of(call) is not None:
            break
    assert world.result_of(call).success and seen == 2


def test_a_dock_freed_before_the_base_end_is_taken_whole_and_the_robot_does_not_stop():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: world.view("pode1")["target"] == "O_Q"
                           and world.view("pode1")["node"] == "N_W", 60.0)
    world.traffic.on_release("pode2")  # 아직 대기 자리에 닿기 전이다
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    codes = world.codes("pode1")
    assert "ORDER_UPDATED" in codes and "BASE_END_WAIT" not in codes
    granted = [e.detail for e in world.events
               if e.robot == "pode1" and e.code == "ZONE_GRANTED"]
    assert granted[-1] == "HALL+J_E+OUT_DOCK to O", "the whole stretch at once"
    assert not world.problems


def test_the_robot_answers_reach_the_traffic_layer_in_order_feedback_superseded_result():
    """도우미(legs, answer_at)에 가리지 않고 날것의 답 차례를 본다: base 끝 피드백, update를 받으면
    옛 goal의 SUPERSEDED, 새 goal의 조각 피드백, 마지막에 새 goal의 성공 하나."""
    world, call = waiting_at_the_queue_spot()
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    said = [(method, args) for _, robot, method, args in world.answers if robot == "pode1"]
    [first] = [o.goal_id for o in orders(world, "pode1") if o.update_id == 0]
    [update] = [o.goal_id for o in orders(world, "pode1") if o.update_id == 1]
    superseded = said.index(("on_move_result", (first, False, "SUPERSEDED", 2)))
    before, after = said[:superseded], said[superseded + 1:]
    assert before and all(m == "on_move_feedback" and a[0] == first for m, a in before)
    assert before[-1][1][1:3] == (2, True), "it waits at the base end"
    assert all(a[0] == update for _, a in after), "the old goal says nothing more"
    feedback = after[:-1]
    assert feedback and [m for m, _ in feedback] == ["on_move_feedback"] * len(feedback)
    seqs = [a[1] for _, a in feedback]
    # update를 받은 로봇은 새 goal id로 base 끝(seq 2)을 한 번 다시 알리고 이어 달린다
    taken = [seg.seq_id for o in orders(world, "pode1") if o.goal_id == update for seg in o.base]
    assert seqs[0] == 2 and seqs[-1] > 2 and seqs == sorted(set(seqs)), seqs
    assert set(seqs[1:]) <= set(taken), seqs
    last = orders(world, "pode1")[-1].base[-1].seq_id
    assert after[-1] == ("on_move_result", (update, True, "", last))
    assert not world.problems


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_parking_robot_whose_spot_was_taken_while_it_waited_chooses_again(order):
    """v7: pode1은 하나 남은 칸 P-3으로 가다가 Q2(base 끝)에서 칸 입구 E-3을 기다린다. 그동안 P-3을
    남이 잡고 P-1이 빈다. 끈 길은 트립을 버리고 다시 고르는데, order는 base 끝에서 P-3을 계속
    기다렸다. 끊긴 두 로봇의 자리는 손으로 잡고 놓는다."""
    world = Harness(layout="v7", starts={"pode1": "W", "pode2": "P-2", "pode3": "P-1"},
                    params=replace(ON, use_order_model=order))
    world.report()
    world.silent.update({"pode2", "pode3"})
    grants = world.traffic.grants
    assert grants.force("pode2", nodes=("E-3",))
    world.traffic.on_release("pode1")
    world.step()
    assert world.traffic._robots["pode1"].trip.goal == "P-3"
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2"
                           and "ZONE_WAIT" in world.codes("pode1"), 60.0)
    world.run(1.0)
    grants.release_node("pode2", "E-3")
    assert grants.force("pode2", nodes=("P-3",))  # 남이 P-3을 잡았다
    grants.release_node("pode3", "P-1")  # P-1은 비었다
    world.bots["pode3"].x, world.bots["pode3"].y = world.site.node_pose("W")
    assert world.run_until(lambda: world.phase("pode1") == "PARKED", 60.0), world.log[-10:]
    assert world.view("pode1")["node"] == "P-1" and not world.problems


# ---- horizon만 바꾸기, 거절 ----

def test_a_changed_route_ahead_replaces_only_the_horizon():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    held = world.view("pode1")["held"]
    world.traffic._blocked[("N_E", "X_E")] = world.now + 300.0  # 출고 도크 진입로가 막혔다
    world.run(6.0)
    change = orders(world, "pode1")[-1]
    assert (change.update_id, change.stitch_node, change.base) == (1, "O_Q", ())
    assert ends(change.horizon) == [(3, "nav_goal", "N_W"), (4, "follow_path", "O")]
    assert world.codes("pode1").count("HORIZON_CHANGED") == 1
    assert world.view("pode1")["held"] == held, "the base stays as it was"
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert orders(world, "pode1")[-1].update_id == 2 and not world.problems


def test_a_horizon_only_update_at_the_base_end_is_not_a_goal_that_did_not_start():
    """base 끝에서 기다리는 로봇은 horizon만 바뀐 update(새 goal id)를 받아도 움직이지 않는다. 끝
    지점에서 3 cm 떨어져 서 있어도 출발하지 않은 것(NO_START)이 아니다."""
    world, _ = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    world.bots["pode1"].x += 0.03
    world.traffic._blocked[("N_E", "X_E")] = world.now + 300.0  # 출고 도크 진입로가 막혔다
    world.run(12.0)
    assert world.codes("pode1").count("HORIZON_CHANGED") == 1
    assert "NO_START" not in world.codes("pode1")


def test_a_queued_robot_takes_the_dock_turn_when_its_order_is_extended_into_the_dock():
    """줄에 선 로봇의 길은 도크까지 이어져 있어도 차례를 받은 길이 아니다. 도크가 비어 도크 구간을
    잡으면 그때 차례를 받는다."""
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    trip = world.traffic._robots["pode1"].trip
    assert trip.goal == "O" and not trip.turn
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: trip.turn, 30.0)
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success and not world.problems


def test_a_refused_update_is_followed_by_a_new_order_from_the_base_end():
    world, call = queued_for_outbound()
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    world.traffic.order_fault = "stitch"  # 다음 update의 이음 지점을 일부러 틀리게 보낸다
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: "ORDER_UPDATE_REJECTED" in world.codes("pode1"), 60.0)
    assert world.view("pode1")["node"] == "O_Q", "the robot stays at the end of its base"
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    codes = world.codes("pode1")
    assert "GOAL_STALLED" not in codes and "GOAL_RETRY" not in codes, "not a failed attempt"
    first, bad, again = orders(world, "pode1")[-3:]
    assert (bad.order_id, bad.update_id, bad.stitch_node) == (first.order_id, 1, "?")
    assert again.update_id == 0 and again.order_id != first.order_id
    assert ends(again.base) == [(1, "nav_goal", "N_E"), (2, "follow_path", "O")]
    assert world.result_of(call).success and not world.problems


# ---- update의 답이 오기 전 ----

def test_no_second_update_goes_out_before_the_robot_answers_the_first(monkeypatch):
    monkeypatch.setattr(move_orders, "UPDATE_ANSWER_S", 60.0)
    world, call = waiting_at_the_queue_spot()
    world.traffic.on_release("pode2")
    world.run(1.0)
    world.hold_answers = True
    world.traffic._blocked[("N_E", "X_E")] = world.now + 300.0  # horizon이 바뀐다
    world.run(15.0)  # 그사이 출고 도크가 비어 다음 구간을 잡을 수 있게 된다
    assert world.view("pode2")["held"] == ["HALL"]
    assert [o.update_id for o in orders(world, "pode1")] == [0, 1], "update 1 is unanswered"
    world.hold_answers = False
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert [o.update_id for o in orders(world, "pode1")] == [0, 1, 2]
    assert world.result_of(call).success and not world.problems


def test_an_update_nobody_answers_ends_both_goals_and_a_new_order_follows():
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pode1")[-1]
    world.hold_answers = True
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: len(orders(world, "pode1")) == 2, 30.0)
    update = orders(world, "pode1")[-1]
    world.run(move_orders.UPDATE_ANSWER_S + 0.5)
    cancelled = {c.goal_id for c in world.sent(CancelCmd, "pode1")}
    assert {first.goal_id, update.goal_id} <= cancelled, "the old goal may still be the live one"
    assert "ORDER_UPDATE_REJECTED" in world.codes("pode1")
    world.hold_answers = False
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert orders(world, "pode1")[-1].update_id == 0
    assert world.result_of(call).success and not world.problems


@pytest.mark.parametrize(("fault", "reached_unknown"), [
    ("on_goal_not_sent", False),  # 서버가 준비되지 않았다. 닿지 않은 것이 확실하다
    ("on_goal_rejected", True),  # 거절이거나 결과를 잃었다. 붙었는지 알 수 없다
    ("on_goal_lost", True),
])
def test_an_update_goal_that_failed_on_the_way_is_followed_by_a_new_order(fault, reached_unknown):
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pode1")[-1]
    world.goal_fault["pode1"] = fault
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: "ORDER_UPDATE_REJECTED" in world.codes("pode1"), 60.0)
    update = orders(world, "pode1")[-1]
    world.step()
    cancelled = {c.goal_id for c in world.sent(CancelCmd, "pode1")}
    assert (update.goal_id in cancelled) == reached_unknown, "both goals end when it is unclear"
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    codes = world.codes("pode1")
    assert "GOAL_RETRY" not in codes and "GOAL_STALLED" not in codes
    again = orders(world, "pode1")[-1]
    assert again.update_id == 0 and again.order_id != first.order_id
    assert first.goal_id in {c.goal_id for c in world.sent(CancelCmd, "pode1")}
    assert world.result_of(call).success and not world.problems


def test_the_old_goal_failing_before_the_update_is_answered_is_taken_as_that_goals_result():
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pode1")[-1]
    world.hold_answers = True
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: len(orders(world, "pode1")) == 2, 30.0)
    world.traffic.on_move_result("pode1", first.goal_id, False, "NAV_ABORTED", 3)
    assert world.view("pode1")["goal_id"] is None, "the running order ended, the fleet saw it"
    world.step()
    codes = world.codes("pode1")
    # 로봇은 그 goal의 끝(기다리는 자리)에 서 있다. 실패한 시도가 아니라 닿은 것으로 본다
    assert "GOAL_ENDED_AT_TARGET" in codes and "GOAL_RETRY" not in codes
    assert world.view("pode1")["node"] == "O_Q"


# ---- 통신이 끊긴 로봇 ----

def test_a_silent_robot_drives_to_its_base_end_and_its_zones_stay_until_then():
    world = Harness(params=ON)
    world.step()
    call = world.call("WORK")
    world.run(2.0)
    world.silent.add("pode2")
    assert world.run_until(lambda: world.view("pode2")["state"] == "COMM_LOST", 10.0)
    held = world.view("pode2")["held"]
    assert held == ["HALL", "J_W", "WORK_DOCK"]
    lost = next(e for e in world.events if e.code == "COMM_LOST")
    assert "drives on to W" in lost.detail
    assert world.traffic.confirm_clear("pode2", held, "carried away") is None
    assert "base" in world.traffic.clear_reason
    world.run(30.0)  # base 끝에 닿을 예상 시각과 여유가 지났다
    assert (world.bots["pode2"].x, world.bots["pode2"].y) == (0.18, 0.89), "it drove on"
    assert world.view("pode2")["held"] == held, "nothing is released by itself"
    world.silent.discard("pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    assert world.result_of(call).robot_id == "pode2" and not world.problems


def base_until_when_lost(extra_s):
    world = Harness(params=replace(ON, comm_base_hold_extra_s=extra_s))
    world.step()
    world.call("WORK")
    world.run(2.0)
    world.silent.add("pode2")
    assert world.run_until(lambda: world.view("pode2")["state"] == "COMM_LOST", 10.0)
    return world.traffic._robots["pode2"].base_until


def test_the_base_hold_margin_of_a_silent_robot_comes_from_the_parameter():
    default = base_until_when_lost(ON.comm_base_hold_extra_s)
    assert base_until_when_lost(10.0) == pytest.approx(default + 10.0 - ON.comm_base_hold_extra_s)


def test_a_negative_base_hold_margin_does_not_free_the_zones_before_the_base_end():
    assert base_until_when_lost(-5.0) == pytest.approx(base_until_when_lost(0.0))


def test_the_base_time_also_guards_a_robot_that_has_just_started_talking_again():
    world = Harness(params=ON)
    world.step()
    world.call("WORK")
    world.run(2.0)
    world.silent.add("pode2")
    assert world.run_until(lambda: world.view("pode2")["state"] == "COMM_LOST", 10.0)
    held = world.view("pode2")["held"]
    world.silent.discard("pode2")  # 말이 돌아왔다. 취소가 닿았는지는 아직 모른다
    world.step()
    assert world.view("pode2")["comm"] == "RECOVERING"
    assert world.traffic.confirm_clear("pode2", held, "carried away") is None
    assert "base" in world.traffic.clear_reason
    world.run(world.params.recover_check_s + 0.5)
    assert world.view("pode2")["comm"] == "", "the check is over"
    held = world.view("pode2")["held"]
    assert world.traffic.confirm_clear("pode2", held, "carried away") is None, (
        "whether the cancel reached the robot is still unknown")
    assert "base" in world.traffic.clear_reason


def test_an_order_whose_nav_goal_segment_fails_short_of_the_base_end_is_retried():
    """order의 마지막 base 조각이 Nav2 goal이면 조각 하나씩 보낼 때와 같은 규칙이다: 로봇이 닿았다고
    한 것이 아니므로 도착 허용 거리의 절반 안이어야 닿은 것으로 본다(test_zone_at_target.py)."""
    world, _ = queued_for_outbound()
    spot = world.traffic.site.node_pose("O_Q")
    bot = world.bots["pode1"]
    assert world.run_until(lambda: math.dist((bot.x, bot.y), spot) <= 0.10, 60.0)
    order = orders(world, "pode1")[-1]
    assert order.base[-1].end_node == "O_Q" and order.base[-1].mode == "nav_goal", ends(order.base)
    assert math.dist((bot.x, bot.y), spot) > world.params.arrival_tolerance_m / 2
    bot.cmd = None  # 대기 자리에 못 미쳐 섰다. 결과는 아래에서 넣는다
    world.step()
    world.traffic.on_move_result("pode1", order.goal_id, False, "NAV_ABORTED",
                                 order.base[-2].seq_id)
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes
