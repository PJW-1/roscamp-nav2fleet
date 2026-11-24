"""넓은 곳(한 공유 구역) 안에서 이어지는 Nav2 goal 이음은 goal 하나로 합친다.
`through` 이음(v7은 칸 입구 E-1~E-3)이 두 끝 사이의 가장 짧은 길일 때만 합치고, 구역을 넘거나 설 수 있는 지점을 거치면 합치지 않는다."""
from dataclasses import replace

import pytest
from fleet_core.common.commands import MoveCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import NAV_GOAL, load_fleet_map, shortest_path
from fleet_core.traffic.zone import routes
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


def test_a_run_of_hall_nav_edges_is_one_segment():
    way = shortest_path(V7, "O_OUT", "Q2")
    assert [edge.b for edge in way] == ["E-3", "Q2"]
    assert routes.segment_end(V7, way, 0, 2) == 2
    assert routes.segment_end(V7, way, 0, 1) == 1, "never past what is granted"


def test_a_run_merges_only_through_nodes_marked_through():
    assert [name for name, spec in V7.nodes.items() if spec.through] == ["E-1", "E-2", "E-3"]
    way = shortest_path(V7, "W", "E-2")
    assert [edge.b for edge in way] == ["M_E", "O_IN", "E-2"]
    assert way[0].zones == ("WORK_DOCK", "HALL")
    assert routes.segment_end(V7, way, 0, 3) == 1, "a leave_dock edge crosses zones"
    assert routes.segment_end(V7, way, 1, 3) == 2, "O_IN is a place to wait"
    detour = [V7.edge("O_IN", "M_E"), V7.edge("M_E", "Q2")]
    assert routes.segment_end(V7, detour, 0, 2) == 1, "M_E is the detour point"


def test_a_detour_through_bay_entrances_is_not_merged():
    """O_OUT -> E-3을 피해 도는 길은 두 끝 사이의 가장 짧은 길인 데까지만 합친다."""
    straight = [V7.edge("O_OUT", "E-3"), V7.edge("E-3", "Q2")]
    assert routes.segment_end(V7, straight, 0, 2) == 2
    around = [V7.edge("O_OUT", "E-2"), V7.edge("E-2", "E-3"), V7.edge("E-3", "Q2")]
    assert routes.segment_end(V7, around, 0, 3) == 1
    assert routes.segment_end(V7, around, 1, 3) == 3, "E-2 -> E-3 -> Q2 is the shortest way"



def test_a_free_edge_from_off_the_graph_is_not_merged():
    """노드 밖에서 다시 계획한 로봇의 첫 이음(free_edge)은 시작 노드가 없어 합치지 않는다."""
    way = [routes.free_edge(V7, 0.45, 0.30, "E-3"), V7.edge("E-3", "Q2")]
    assert routes.segment_end(V7, way, 0, 2) == 1

def test_a_merged_nav_goal_faces_the_last_node():
    way = shortest_path(V7, "O_OUT", "Q2")
    cmd = routes.move_command(V7, "pode1", 3, way, 0.15, 0.62)
    assert (cmd.mode, cmd.target_id, cmd.path) == (NAV_GOAL, "Q2", ())
    # Q2 자리는 지도에서 읽는다
    assert (cmd.x, cmd.y, cmd.yaw) == pytest.approx((*V7.node_pose("Q2"), 0.0))


@pytest.mark.parametrize("order", [False, True])
def test_leaving_outbound_goes_to_q2_in_one_nav_goal_holding_e3(order):
    world = world_for(order, {"pode1": "O", "pode2": "P-1", "pode3": "P-2"})
    world.report()
    world.call("WORK", "pode1")
    seen = set()

    def at_q2():
        if world.view("pode1")["target"] == "Q2" or nav_targets(world, "pode1"):
            seen.update(world.view("pode1")["nodes"])
        return world.view("pode1")["node"] == "Q2"
    assert world.run_until(at_q2, 120.0), world.log[-10:]
    assert nav_targets(world, "pode1") == ["Q2"], world.log
    assert "E-3" in seen, "E-3 is held while the robot drives past it"
    assert world.run_until(lambda: "E-3" not in world.view("pode1")["nodes"], 10.0)
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_a_detour_around_a_held_door_is_sent_leg_by_leg(order):
    """E-3에 pode2가 서 있으면 pode1은 O_IN -> M_E -> Q2로 돌고, M_E에 들르게 이음마다 보낸다."""
    world = world_for(order, {"pode1": "O", "pode2": "E-3", "pode3": "P-1"})
    world.report()
    world.call("WORK", "pode1")
    assert world.run_until(lambda: world.view("pode1")["node"] == "Q2", 200.0), world.log[-10:]
    goals = nav_targets(world, "pode1")
    assert "M_E" in goals and goals[-1] == "Q2", goals
    assert not any("pode1" in line and "E-3" in line for line in world.log
                   if "GRANTED" in line), "it never takes E-3"
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_a_robot_parked_at_p3_holds_only_its_bay_so_p1_leaves_by_e3(order):
    """칸에 선 로봇은 자기 칸만 쥔다(입구를 쥐면 E-3을 거치는 길이 막혀 넓은 곳으로 돈다)."""
    world = world_for(order, {"pode1": "P-2", "pode2": "W", "pode3": "P-1"})
    world.report()
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.phase("pode2") == "PARKED", 300.0), world.log[-10:]
    assert world.view("pode2")["node"] == "P-3"
    assert world.run_until(lambda: set(world.view("pode2")["nodes"]) == {"P-3"}, 10.0), (
        world.view("pode2")["nodes"])
    world.call("WORK", "pode3")
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q2", 120.0), world.log[-10:]
    assert nav_targets(world, "pode3") == ["Q2"], world.log
    assert not world.problems


def test_a_parked_robot_seen_at_its_door_keeps_the_door():
    """칸에 선 로봇은 칸 노드와 입구 사이 절반보다 칸에 가까울 때만 입구를 놓는다."""
    world = world_for(False, {"pode1": "P-2", "pode2": "W", "pode3": "P-1"})
    world.report()
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.phase("pode2") == "PARKED", 300.0), world.log[-10:]
    assert "E-3" in world.view("pode2")["nodes"], "the door is still held at the moment it parks"
    world.bots["pode2"].y = 0.22  # 위치가 입구 쪽으로 보인다
    world.run(5.0)
    assert "E-3" in world.view("pode2")["nodes"], world.view("pode2")["nodes"]


@pytest.mark.parametrize("order", [False, True])
def test_from_work_to_a_bay_the_goals_stop_at_the_waiting_node(order):
    world = world_for(order, {"pode1": "W", "pode2": "P-1", "pode3": "P-3"})
    world.report()
    assert world.run_until(lambda: world.phase("pode1") == "PARKED", 200.0), world.log[-10:]
    assert world.view("pode1")["node"] == "P-2"
    assert nav_targets(world, "pode1") == ["M_E", "O_IN", "E-2"], world.log
    assert not world.problems


@pytest.mark.parametrize("order", [False, True])
def test_a_held_node_beyond_a_waiting_node_cuts_the_merged_goal_short(order):
    """W에서 빈 칸 P-2로 가는 M_E -> O_IN -> E-2: E-2를 다른 로봇이 쥐면 O_IN까지만 보내고, 풀리면 나머지를 따로 보낸다."""
    world = world_for(order, {"pode1": "W", "pode2": "P-1", "pode3": "P-3"})
    world.report()
    assert world.traffic.grants.take("pode3", (), ("E-2",))  # 잠깐 쥔다(곧 놓는다)
    world.step()
    assert "E-2" not in world.view("pode1")["nodes"]
    assert "O_IN" in world.view("pode1")["nodes"]
    assert world.run_until(lambda: world.phase("pode1") == "PARKED", 200.0), world.log[-10:]
    assert world.view("pode1")["node"] == "P-2"
    assert nav_targets(world, "pode1") == ["M_E", "O_IN", "E-2"], world.log
    assert not world.problems


def test_a_robot_blocked_on_a_later_edge_of_a_merged_goal_marks_that_edge():
    """합친 goal의 둘째 이음(E-3 -> Q2)에서 막히면 막힌 이음을 표시한다."""
    world = world_for(False, {"pode1": "O", "pode2": "P-1", "pode3": "P-2"})
    world.obstacles.append((1.15, 0.315, 0.05))  # O_OUT -> Q2 곧은 선 위, E-3 동쪽
    world.report()
    world.call("WORK", "pode1")
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 120.0)
    mark = next(e for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert mark.detail.startswith("E-3->Q2"), mark.detail
