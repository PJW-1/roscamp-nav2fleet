"""넓은 곳(한 공유 구역) 안에서 이어지는 Nav2 goal 이음은 goal 하나로 합친다(사용자 결정
2026-11-14 "합쳐"). 로봇은 중간 지점에 들르지 않고 Nav2가 그 끝까지 매끈한 길을 짠다.
- 합치는 것: 이음 방식이 모두 Nav2 goal이고 구역이 모두 같은 공유 구역 하나이며, 중간 지점이
  지도에서 `through`로 표시된 이음들(v7은 칸 입구 E-1~E-3), 그리고 두 끝 사이의 가장 짧은 길일 때.
- 안 합치는 것: 구역을 넘는 이음(v7 leave_dock W -> M_E는 WORK_DOCK과 HALL), 설 수 있는 지점(O_IN,
  Q2)이나 우회 지점(M_E)을 거치는 이음, 돌아가는 길. 합친 goal은 Nav2에 끝만 알리므로, 관제가
  무엇을 피해 고른 길을 합치면 Nav2가 피한 곳으로 간다(Codex 검수 2026-11-14).
- 중간 지점은 goal을 보내기 전에 모두 잡는다.
- 중간 지점은 합친 조각이 끝나면 놓는다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import MoveCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import NAV_GOAL, load_fleet_map, route
from fleet_core.traffic.zone import trips
from zone_harness import Harness

V7 = load_fleet_map("v7")


def world_for(order: bool, starts: dict) -> Harness:
    return Harness(layout="v7", params=replace(load_params(), use_order_model=order),
                   starts=starts)


def nav_targets(world, robot) -> list:
    """보낸 Nav2 goal의 끝 지점들(order면 base 조각, 아니면 MoveCmd)."""
    ends = [c.target_id for c in world.sent(MoveCmd, robot) if c.mode == NAV_GOAL]
    for order in world.sent(MoveOrderCmd, robot):
        ends.extend(s.end_node for s in order.base if s.mode == NAV_GOAL)
    return ends


def test_a_run_of_hall_nav_links_is_one_segment():
    way = route(V7, "O_OUT", "Q2")
    assert [link.b for link in way] == ["E-3", "Q2"]
    assert trips.segment_end(V7, way, 0, 2) == 2
    assert trips.segment_end(V7, way, 0, 1) == 1, "never past what is granted"


def test_a_run_merges_only_through_nodes_marked_through():
    assert [name for name, spec in V7.nodes.items() if spec.through] == ["E-1", "E-2", "E-3"]
    way = route(V7, "W", "E-2")
    assert [link.b for link in way] == ["M_E", "O_IN", "E-2"]
    assert way[0].zones == ("WORK_DOCK", "HALL")
    assert trips.segment_end(V7, way, 0, 3) == 1, "a leave_dock link crosses zones"
    assert trips.segment_end(V7, way, 1, 3) == 2, "O_IN is a place to wait"
    detour = [V7.link("O_IN", "M_E"), V7.link("M_E", "Q2")]
    assert trips.segment_end(V7, detour, 0, 2) == 1, "M_E is the detour point"


def test_a_detour_through_bay_entrances_is_not_merged():
    """O_OUT -> E-3을 피해 O_OUT -> E-2 -> E-3 -> Q2로 돌 때 합치면 Nav2는 피한 곧은 길로 간다.
    두 끝 사이의 가장 짧은 길인 데까지만 합친다."""
    straight = [V7.link("O_OUT", "E-3"), V7.link("E-3", "Q2")]
    assert trips.segment_end(V7, straight, 0, 2) == 2
    around = [V7.link("O_OUT", "E-2"), V7.link("E-2", "E-3"), V7.link("E-3", "Q2")]
    assert trips.segment_end(V7, around, 0, 3) == 1
    assert trips.segment_end(V7, around, 1, 3) == 3, "E-2 -> E-3 -> Q2 is the shortest way"


def test_a_merged_nav_goal_faces_the_last_node():
    way = route(V7, "O_OUT", "Q2")
    cmd = trips.move_command(V7, "pinky1", 3, way, 0.15, 0.62)
    assert (cmd.mode, cmd.target_id, cmd.path) == (NAV_GOAL, "Q2", ())
    assert (cmd.x, cmd.y, cmd.yaw) == (pytest.approx(1.66), pytest.approx(0.16), 0.0)


@pytest.mark.parametrize("order", [False, True])
def test_leaving_outbound_goes_to_q2_in_one_nav_goal_holding_e3(order):
    world = world_for(order, {"pinky1": "O", "pinky2": "P-1", "pinky3": "P-2"})
    world.report()
    world.call("WORK", "pinky1")
    seen = set()

    def at_q2():
        if world.view("pinky1")["target"] == "Q2" or nav_targets(world, "pinky1"):
            seen.update(world.view("pinky1")["nodes"])
        return world.view("pinky1")["node"] == "Q2"
    assert world.run_until(at_q2, 120.0), world.log[-10:]
    assert nav_targets(world, "pinky1") == ["Q2"], world.log
    assert "E-3" in seen, "E-3 is held while the robot drives past it"
    assert world.run_until(lambda: "E-3" not in world.view("pinky1")["nodes"], 10.0)
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_a_detour_around_a_held_door_is_sent_leg_by_leg(order):
    """pinky2가 E-3에 서 있으면 출고에서 나온 pinky1은 O_IN -> M_E -> Q2로 돈다. 이 길을 Q2로 가는
    goal 하나로 보내면 Nav2는 남쪽 통로로 E-3 앞을 지난다. M_E에 들르게 이음마다 보낸다."""
    world = world_for(order, {"pinky1": "O", "pinky2": "E-3", "pinky3": "P-1"})
    world.report()
    world.call("WORK", "pinky1")
    assert world.run_until(lambda: world.view("pinky1")["node"] == "Q2", 200.0), world.log[-10:]
    goals = nav_targets(world, "pinky1")
    assert "M_E" in goals and goals[-1] == "Q2", goals
    assert not any("pinky1" in line and "E-3" in line for line in world.log
                   if "GRANTED" in line), "it never takes E-3"
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_a_robot_parked_at_p3_holds_only_its_bay_so_p1_leaves_by_e3(order):
    """주차 0.10에서 칸과 입구는 0.14라 node_clear_m(0.20)으로는 입구를 놓지 못했다. 칸에 선 로봇이
    입구를 쥐면 E-1, E-2에서 Q2로 가는 길(E-3을 거친다)이 막혀 넓은 곳으로 돌았다. 칸에 선 로봇은
    자기 칸만 쥔다."""
    world = world_for(order, {"pinky1": "P-2", "pinky2": "W", "pinky3": "P-1"})
    world.report()
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.phase("pinky2") == "PARKED", 300.0), world.log[-10:]
    assert world.view("pinky2")["node"] == "P-3"
    assert world.run_until(lambda: set(world.view("pinky2")["nodes"]) == {"P-3"}, 10.0), (
        world.view("pinky2")["nodes"])
    world.call("WORK", "pinky3")
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q2", 120.0), world.log[-10:]
    assert nav_targets(world, "pinky3") == ["Q2"], world.log
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_from_work_to_a_bay_the_goals_stop_at_the_waiting_node(order):
    world = world_for(order, {"pinky1": "W", "pinky2": "P-1", "pinky3": "P-3"})
    world.report()
    assert world.run_until(lambda: world.phase("pinky1") == "PARKED", 200.0), world.log[-10:]
    assert world.view("pinky1")["node"] == "P-2"
    assert nav_targets(world, "pinky1") == ["M_E", "O_IN", "E-2"], world.log
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_a_held_node_beyond_a_waiting_node_cuts_the_merged_goal_short(order):
    """W에서 빈 칸 P-2로: M_E -> O_IN -> E-2. 처음 잡을 때 E-2를 다른 로봇이 쥐고 있으면 O_IN까지만
    보낸다(지금처럼, O_IN은 설 수 있는 곳이다). E-2가 풀리면 나머지를 따로 보낸다."""
    world = world_for(order, {"pinky1": "W", "pinky2": "P-1", "pinky3": "P-3"})
    world.report()
    assert world.traffic.grants.take("pinky3", (), ("E-2",))  # 잠깐 쥔다(곧 놓는다)
    world.step()
    assert "E-2" not in world.view("pinky1")["nodes"]
    assert "O_IN" in world.view("pinky1")["nodes"]
    assert world.run_until(lambda: world.phase("pinky1") == "PARKED", 200.0), world.log[-10:]
    assert world.view("pinky1")["node"] == "P-2"
    assert nav_targets(world, "pinky1") == ["M_E", "O_IN", "E-2"], world.log
    assert not world.problems


def test_a_robot_blocked_on_a_later_link_of_a_merged_goal_marks_that_link():
    """합친 goal의 둘째 이음(E-3 -> Q2)에서 막히면 첫 이음(O_OUT -> E-3)이 아니라 막힌 이음을
    표시한다."""
    world = world_for(False, {"pinky1": "O", "pinky2": "P-1", "pinky3": "P-2"})
    world.obstacles.append((1.15, 0.315, 0.05))  # O_OUT -> Q2 곧은 선 위, E-3 동쪽
    world.report()
    world.call("WORK", "pinky1")
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 120.0)
    mark = next(e for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert mark.detail.startswith("E-3->Q2"), mark.detail
