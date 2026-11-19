"""작업대로 불린 로봇이 대기 자리가 없어 주차하러 가는 길(standby route).
- 가는 동안 대기 자리가 비면 주차 칸에 들어가지 않고 그 자리로 간다
  (2026-11-15 Gazebo: 12건 가운데 9번, 자리가 빈 뒤에도 합계 81초를 주차하러 더 갔다).
  고정 경로 위에서는 그 구간 끝에서, 마당 goal로 가는 중이면 goal을 취소하고 돌린다.
  order 모델이면 첫 고정 경로(out_out)까지만 base로 주고, 그 위에서 자리가 비면 취소 없이
  O_OUT에 이어 붙는 update로 돌린다(2026-11-17). 아니면 O_OUT에 닿기 standby_lookahead_s 전에
  주차 조각을 붙인다. update에 답이 오기 전에는 바꾸지 않는다.
- 자리가 끝내 비지 않으면 지금처럼 주차한다.
- 양보하려고 주차하러 가는 로봇(DEADLOCK_SEEN, HALL_YIELD)은 돌리지 않는다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, MoveOrderCmd, RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.zone.routes import left_s
from zone_harness import Harness

ORDER_MODEL = pytest.mark.parametrize("order_model", [False, True])


def world_with_a_busy_line(order_model=False, **changes):
    """pode2는 W에서 일하고, pode1은 O에서 출고를 끝냈고(아직 쥐고 있다),
    pode3은 불려서 Q2로 간다."""
    world = Harness(layout="v7", starts={"pode1": "O", "pode2": "W", "pode3": "P-3"},
                    params=replace(load_params(), use_order_model=order_model, **changes))
    world.report()
    at_work, at_out = world.call("WORK", "pode2"), world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: world.result_of(at_work) and world.result_of(at_out), 30.0)
    for _ in range(3):
        world.call("WORK")
    assert world.run_until(lambda: world.traffic.grants.node_holder("Q2") == "pode3", 30.0)
    return world


def backed_into_a_bay(world, robot) -> bool:
    orders = [c for c in world.commands if isinstance(c, MoveOrderCmd) and c.robot == robot]
    return bool(world.sent(RetreatCmd, robot)) or any(
        segment.mode == "retreat" for c in orders for segment in (*c.base, *c.horizon))


@ORDER_MODEL
def test_a_spot_that_frees_on_the_way_to_parking_turns_the_robot_to_the_line(order_model):
    world = world_with_a_busy_line(order_model)
    assert world.run_until(lambda: world.bots["pode3"].x >= 1.3, 60.0)
    world.traffic.on_release("pode1")
    world.step()
    route = world.traffic._robots["pode1"].route
    assert route.goal in world.site.parking and route.standby, "no spot yet: heads for a bay"
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2", 120.0), world.log[-30:]
    assert world.sent(CancelCmd, "pode1"), "the yard goal to the bay is canceled"
    route = world.traffic._robots["pode1"].route
    assert route is None or not route.standby
    assert not world.problems


def test_a_spot_that_frees_on_the_fixed_path_out_of_the_dock_is_taken_at_its_end():
    world = world_with_a_busy_line()
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q2", 60.0)
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2", 120.0), world.log[-30:]
    assert not world.sent(CancelCmd, "pode1"), "a fixed path is driven to its end"
    assert not backed_into_a_bay(world, "pode1")
    assert "E-1" not in [c.target_id for c in world.sent(MoveCmd, "pode1")]
    assert not world.problems


@ORDER_MODEL
def test_with_no_spot_freeing_the_robot_still_parks(order_model):
    world = world_with_a_busy_line(order_model)
    world.silent.add("pode3")  # Q2를 쥔 채 서 있다: 자리가 비지 않는다
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.view("pode1")["node"] in world.site.parking, 120.0)
    assert backed_into_a_bay(world, "pode1")


def released_on_the_way_out(order_model, delay_s=1.6, **changes):
    """pode3이 Q2에 닿고 delay_s 뒤에 pode1이 출고를 끝낸다. 그때는 줄 자리가 없고,
    pode1이 O -> O_OUT 고정 경로(out_out, 약 3.1 s)를 가는 도중에 pode3이 Q2를 놓는다.
    Q2는 pode3이 닿고 약 2.35 s 뒤에 빈다: delay_s 1.6이면 out_out 0.75 s쯤, 0.4면 1.95 s쯤."""
    world = world_with_a_busy_line(order_model, **changes)
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q2", 60.0)
    world.run(delay_s)
    world.traffic.on_release("pode1")
    world.step()
    m = world.traffic._robots["pode1"]
    assert m.route.standby and m.route.goal in world.site.parking, "no spot yet: heads for a bay"
    assert m.route.edges[0].b == "O_OUT" and m.route.edges[0].mode == "follow_path"
    return world, m


def stood_at(robot, node, world) -> bool:
    return any(e.code == "BASE_END_WAIT" and e.robot == robot and f"at {node}," in e.detail
               for e in world.events)


def pode1_orders(world, since: int) -> list:
    return [c for c in world.commands[since:] if isinstance(c, MoveOrderCmd)
            and c.robot == "pode1"]


@pytest.mark.parametrize("delay_s", [1.6, 0.4])
def test_a_spot_that_frees_on_the_fixed_path_turns_the_order_before_its_end(delay_s):
    """order 모델: out_out을 가는 도중 자리가 비면 그 조각 끝을 기다리지 않고 바꾼다.
    달리는 고정 경로는 취소하지도 다시 보내지도 않고, O_OUT에 이어 붙는 update로 새 길을 준다.
    delay_s 0.4는 out_out 끝 약 1.1 s 전에 빈다: standby_lookahead_s(0.5) 앞이라 아직 바꾼다."""
    world, m = released_on_the_way_out(True, delay_s)
    since = len(world.commands) - 1
    first = pode1_orders(world, since)
    assert [s.end_node for s in first[0].base] == ["O_OUT"], "the base stops at the fixed path end"
    assert world.run_until(lambda: "STANDBY_LEFT" in world.codes("pode1"), 10.0)
    bot = world.bots["pode1"]
    assert m.route.done == 0 and (bot.x, bot.y) != world.site.node_pose("O_OUT"), \
        "it turns while still on the fixed path"
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2", 120.0), world.log[-30:]
    orders = pode1_orders(world, since)
    assert [c.update_id for c in orders][:2] == [0, 1]
    assert orders[1].stitch_node == "O_OUT" and orders[1].base[0].end_node == "Q2"
    assert [c for c in orders if c.update_id == 0] == orders[:1], "no new order"
    assert not world.sent(CancelCmd, "pode1"), "the fixed path is not canceled"
    assert "ORDER_UPDATE_REJECTED" not in world.codes("pode1")
    assert not stood_at("pode1", "O_OUT", world), "it does not stand at O_OUT"
    assert not world.retreats("pode1"), "the way into the bay stays in the horizon"
    assert not world.problems


def test_with_no_spot_freeing_the_order_drives_on_to_the_bay_without_a_stop():
    """order 모델: 자리가 비지 않으면 O_OUT에 닿기 전에 주차까지의 조각을 update로 붙인다.
    로봇은 O_OUT에서 서지 않는다."""
    world, m = released_on_the_way_out(True)
    world.silent.add("pode3")  # Q2를 쥔 채 서 있다
    since = len(world.commands) - 1
    assert world.run_until(lambda: world.view("pode1")["node"] in world.site.parking, 120.0)
    orders = pode1_orders(world, since)
    assert [c.update_id for c in orders] == [0, 1]
    assert orders[1].stitch_node == "O_OUT"
    assert [s.end_node for s in orders[1].base] == ["E-1", "P-1"]
    assert not world.sent(CancelCmd, "pode1")
    assert not stood_at("pode1", "O_OUT", world)
    assert "STANDBY_LEFT" not in world.codes("pode1")
    assert backed_into_a_bay(world, "pode1")
    assert not world.problems


def left_when_the_bay_is_stitched(standby_lookahead_s) -> float:
    world, m = released_on_the_way_out(True, standby_lookahead_s=standby_lookahead_s)
    world.silent.add("pode3")  # 자리가 비지 않는다
    since = len(world.commands)
    assert world.run_until(lambda: len(pode1_orders(world, since)) == 1, 10.0)
    bot, edge = world.bots["pode1"], m.route.edges[0]
    return left_s(world.params, [edge], bot.x, bot.y)


@pytest.mark.parametrize("standby_lookahead_s", [0.5, 1.5])
def test_the_bay_is_stitched_standby_lookahead_s_before_the_fixed_path_end(standby_lookahead_s):
    """주차 조각을 붙이는 때는 일반 lookahead_s(2.0)가 아니라 standby_lookahead_s를 따른다.
    붙일 때 남은 시간은 그 값 이하이고, 한 걸음(0.1 s)과 틱 하나를 넘게 일찍 붙이지 않는다."""
    left = left_when_the_bay_is_stitched(standby_lookahead_s)
    assert standby_lookahead_s - 0.25 <= left <= standby_lookahead_s


def test_a_spot_freed_while_the_bay_update_is_unanswered_waits_for_the_answer(monkeypatch):
    """주차 조각 update(mo.before)에 답이 오기 전에 옛 goal의 피드백으로 O_OUT을 지났고 그때 자리가
    비었다. 답을 기다리는 동안은 바꾸지 않는다(취소도 새 order도 없다). 답이 온 뒤에 바꾼다."""
    world, m = released_on_the_way_out(True)
    real = world.traffic._station_spots
    closed = [True]
    monkeypatch.setattr(world.traffic, "_station_spots",
                        lambda robot: iter(()) if closed[0] else real(robot))
    since = len(world.commands) - 1
    old = pode1_orders(world, since)[0].goal_id
    assert world.run_until(lambda: len(pode1_orders(world, since)) == 2, 10.0)
    world.hold_answers = True  # 로봇의 답과 피드백이 관제에 닿지 않는다
    assert world.run_until(lambda: world.bots["pode1"].run.done_seq >= 1, 10.0)
    world.inputs.on_move_feedback("pode1", old, 1, False)  # 옛 goal의 피드백만 먼저 닿았다
    assert m.route.done == 1 and m.move_order.before is not None
    closed[0] = False
    world.run(0.5)
    assert not world.sent(CancelCmd, "pode1"), "nothing is canceled while the update is unanswered"
    assert len(pode1_orders(world, since)) == 2
    assert "STANDBY_LEFT" not in world.codes("pode1")
    world.hold_answers = False
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2", 120.0), world.log[-30:]
    assert "STANDBY_LEFT" in world.codes("pode1")
    assert not world.problems
    assert not world.problems


def test_a_spot_whose_zones_cannot_be_taken_does_not_turn_the_order(monkeypatch):
    """order 모델: 자리는 비었지만 새 길의 구역을 잡지 못하면 바꾸지 않고 주차하러 간다.
    달리는 조각의 구역(OUT_DOCK, HALL)은 끝까지 쥐고 있다."""
    world, m = released_on_the_way_out(True)
    grants = world.traffic.grants
    real = grants.take

    def take(robot, zones=(), nodes=(), lanes=None):
        if robot == "pode1" and "Q2" in nodes:
            zones = [*zones, "WORK_DOCK"]
        return real(robot, zones, nodes, lanes)

    monkeypatch.setattr(grants, "take", take)
    since = len(world.commands) - 1
    tried = False
    for _ in range(1200):
        if world.view("pode1")["node"] in world.site.parking:
            break
        world.step()
        if m.route is not None and m.route.done == 0:
            assert {"OUT_DOCK", "HALL"} <= set(grants.held_by("pode1"))
            tried = tried or grants.node_holder("Q2") == ""
    assert world.view("pode1")["node"] in world.site.parking
    assert tried, "the spot was free while on the fixed path"
    assert "STANDBY_LEFT" not in world.codes("pode1")
    assert not world.sent(CancelCmd, "pode1")
    assert [c.update_id for c in pode1_orders(world, since)] == [0, 1]
    assert not world.problems


def test_without_the_order_model_the_fixed_path_still_turns_at_its_end():
    """order를 끈 길은 그대로: 고정 경로 MoveCmd가 끝난 뒤에야 바꾼다."""
    world, m = released_on_the_way_out(False)
    out = world.sent(MoveCmd, "pode1")[-1]
    assert out.target_id == "O_OUT"
    assert world.run_until(lambda: "STANDBY_LEFT" in world.codes("pode1"), 10.0)
    assert world.view("pode1")["node"] == "O_OUT" and m.goal is not None
    assert world.sent(MoveCmd, "pode1")[-1].target_id != "E-1"
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2", 120.0), world.log[-30:]
    assert not world.sent(CancelCmd, "pode1")
    assert not world.sent(MoveOrderCmd, "pode1")
    assert not world.problems


def test_a_free_spot_it_cannot_be_granted_leaves_no_waiting_record(monkeypatch):
    """자리는 비었지만 가는 길의 구역을 아직 잡지 못하면 주차하러 가던 길을 그대로 간다.
    길에서 자리를 알아본 것은 기다린 것이 아니다(grants.waiting에 남지 않는다)."""
    world = world_with_a_busy_line()
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q2", 60.0)
    world.traffic.on_release("pode1")
    world.step()
    grants, m = world.traffic.grants, world.traffic._robots["pode1"]
    assert m.route.standby and m.goal is not None
    real = grants.take

    def take(robot, zones=(), nodes=(), lanes=None):
        if robot == "pode1" and "Q2" in nodes:
            zones = [*zones, "WORK_DOCK"]
        return real(robot, zones, nodes, lanes)

    monkeypatch.setattr(grants, "take", take)
    tried, seen = set(), set()
    for _ in range(1200):
        if world.view("pode1")["node"] in world.site.parking:
            break
        world.step()
        if m.goal is not None:
            seen |= set(grants.waiting().get("pode1", ()))
        if grants.node_holder("Q2") in ("", "pode1"):
            tried.add("Q2")
    assert world.view("pode1")["node"] in world.site.parking
    assert tried == {"Q2"}, "the spot was free on the way"
    assert seen == set()
    assert "STANDBY_LEFT" not in world.codes("pode1")


def test_a_robot_that_gives_way_to_a_bay_is_not_turned_back():
    world = Harness(layout="v7", starts={"pode1": "Q2", "pode2": "W", "pode3": "Q1"})
    world.report()
    at_work = world.call("WORK", "pode2")
    for _ in range(3):
        world.call("WORK")
    assert world.run_until(lambda: world.result_of(at_work), 30.0)
    m = world.traffic._robots["pode1"]
    assert m.phase == "TO_WORK" and m.goal is None and m.node == "Q2"
    assert world.traffic._evade(m)
    assert not m.route.standby
    assert world.run_until(lambda: world.view("pode1")["node"] in world.site.parking, 120.0)
    assert backed_into_a_bay(world, "pode1")
