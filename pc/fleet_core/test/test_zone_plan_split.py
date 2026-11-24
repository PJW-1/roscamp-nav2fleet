"""나눈 goal(해결안 B, 2026-11-24): 넓은 곳 HALL의 노드 밖에서 Q2로 가는 Nav2 goal이 두 길(남쪽 진열대
틈, 동쪽으로 도는 길)의 값이 비슷한 자리에서 나오면, 관제가 중간 지점을 거쳐 두 goal로 보낸다.
NavFn이 1 Hz 다시 계획할 때마다 두 길을 번갈아 내는 것("도리도리")을 막는다.

- plan_split(관제 파라미터)이 꺼져 있으면 예전과 같다. 중간 지점 GAP_S는 자유 이음의 목적지가 아니다.
- 켜져 있으면 지도 splits의 출발 사각형 안에서만 나눈다(가운데 통로 동쪽은 GAP_S, 서쪽 끝은 M_E).
- 두 조각은 한 order의 base에 함께 들어 로봇은 중간 지점 nav_pass_m 앞에서 다음 goal로 넘어간다.
- 첫 조각이 막혔다고 표시되었거나 둘째 이음이 막혔으면 나누지 않는다.
"""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.commands import MoveCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import NAV_GOAL
from zone_harness import Harness


def world_at(xy, split=True, order=True, start="P-1"):
    params = replace(load_params(), plan_split=split, use_order_model=order)
    world = Harness(layout="v7", params=params, starts={"pode1": start})
    if xy is not None:
        off_graph(world.site, xy)
        world.bots["pode1"].x, world.bots["pode1"].y = xy
    return world


def off_graph(site, xy):
    """노드 arrival_tolerance_m 안이면 노드 위로 보고 지도 이음으로 짠다(이 시험이 보는 것이 아니다)."""
    near = min(math.dist(xy, site.node_pose(name)) for name in site.nodes)
    assert near > load_params().arrival_tolerance_m, (xy, near)


def first_nav_ends(world, robot="pode1") -> list:
    """처음 보낸 이동의 Nav2 goal 끝 지점들(order면 첫 order의 base, 아니면 첫 MoveCmd)."""
    for cmd in world.commands:
        if isinstance(cmd, MoveOrderCmd) and cmd.robot == robot:
            return [s.end_node for s in cmd.base if s.mode == NAV_GOAL]
        if isinstance(cmd, MoveCmd) and cmd.robot == robot:
            return [cmd.target_id]
    return []


def go_to_work(world, seconds=0.3):
    world.report()
    world.call("WORK", "pode1")
    world.run(seconds)


def test_the_params_file_keeps_the_switch_off():
    assert load_params().plan_split is False


def test_the_b_params_file_differs_from_the_default_only_in_the_switch():
    """Gazebo 비교 판(fleet_params_file)으로 주는 파일. 다른 값이 어긋나면 비교가 틀린다."""
    docs = Path(__file__).resolve().parents[3] / "docs/plan_flip_2026-11-24"
    b = docs / "fleet_params_plan_split.yaml"
    if not b.is_file():
        pytest.skip("docs/ is not next to this package (installed copy)")
    assert replace(load_params(b), plan_split=False) == load_params()
    assert load_params(b).plan_split is True


@pytest.mark.parametrize("xy", [(1.05, 0.55), (0.80, 0.60), (1.15, 0.48)])
def test_off_a_robot_in_the_middle_aisle_gets_one_goal_to_q2(xy):
    world = world_at(xy, split=False)
    go_to_work(world)
    assert first_nav_ends(world) == ["Q2"]


@pytest.mark.parametrize("xy", [(1.05, 0.55), (0.80, 0.60), (1.15, 0.48), (0.34, 0.68)])
def test_on_a_robot_where_the_two_ways_tie_goes_by_the_shelf_gap_first(xy):
    world = world_at(xy)
    go_to_work(world)
    assert first_nav_ends(world) == ["GAP_S", "Q2"]


def test_on_the_west_end_of_the_middle_aisle_goes_by_m_e_first():
    world = world_at((0.68, 0.60))
    go_to_work(world)
    assert first_nav_ends(world) == ["M_E", "Q2"]


@pytest.mark.parametrize("xy", [(1.40, 0.55), (1.30, 0.25), (0.90, 0.25), (0.45, 0.45)])
def test_on_a_robot_where_one_way_is_clearly_cheaper_is_not_split(xy):
    world = world_at(xy)
    go_to_work(world)
    assert first_nav_ends(world) == ["Q2"]


def test_on_both_goals_go_in_one_order_and_the_gap_goal_faces_q2():
    """두 조각이 첫 base에 함께 있어야 로봇이 GAP_S 앞 nav_pass_m에서 서지 않고 넘어간다.
    GAP_S에서 서는 일이 생겨도 다음 goal 쪽(Q2)을 보고 선다."""
    world = world_at((1.05, 0.55))
    go_to_work(world)
    order = next(c for c in world.commands if isinstance(c, MoveOrderCmd))
    gap, q2 = order.base[0], order.base[1]
    assert (gap.end_node, q2.end_node) == ("GAP_S", "Q2")
    site = world.site
    want = site.nodes["GAP_S"].yaw
    assert gap.yaw == pytest.approx(want)
    (gx, gy), (qx, qy) = site.node_pose("GAP_S"), site.node_pose("Q2")
    toward = math.atan2(qy - gy, qx - gx)
    assert want == pytest.approx(toward, abs=0.01)


def test_on_without_the_order_model_the_two_goals_go_one_after_the_other():
    world = world_at((1.05, 0.55), order=False)
    go_to_work(world)
    assert first_nav_ends(world) == ["GAP_S"]
    assert world.run_until(lambda: any(isinstance(c, MoveCmd) and c.target_id == "Q2"
                                       for c in world.commands), 20.0)


@pytest.mark.parametrize("order", [True, False])
def test_on_the_split_robot_still_reaches_work(order):
    world = world_at((1.05, 0.55), order=order)
    go_to_work(world)
    assert world.run_until(lambda: world.phase("pode1") in ("DOCKING", "AT_STATION"), 60.0)
    assert not world.problems


def test_on_a_first_leg_marked_blocked_is_not_split():
    world = world_at((1.05, 0.55))
    world.report()
    world.traffic._blocked[("@pode1", "GAP_S")] = 1e9
    world.call("WORK", "pode1")
    world.run(0.3)
    assert first_nav_ends(world) == ["Q2"]


def test_on_a_second_leg_marked_blocked_is_not_split():
    world = world_at((1.05, 0.55))
    world.report()
    world.traffic._blocked[("GAP_S", "Q2")] = 1e9
    world.call("WORK", "pode1")
    world.run(0.3)
    assert first_nav_ends(world) == ["Q2"]


def test_on_a_via_node_held_by_another_robot_is_not_split():
    """중간 지점을 남이 쥐고 있으면 나눈 길은 그 로봇을 기다린다. 곧장 가는 goal을 보낸다."""
    params = replace(load_params(), plan_split=True)
    world = Harness(layout="v7", params=params, starts={"pode1": "P-1", "pode2": "M_E"})
    off_graph(world.site, (0.68, 0.60))
    world.bots["pode1"].x, world.bots["pode1"].y = (0.68, 0.60)
    go_to_work(world)
    assert world.traffic.grants.node_holder("M_E") == "pode2"
    assert first_nav_ends(world) == ["Q2"]


@pytest.mark.parametrize("split", [True, False])
def test_gap_s_is_never_a_free_edge_target(split):
    """Q2 -> GAP_S 이음이 있어도 GAP_S는 나누는 데만 쓴다. 곧은 길이 막혀도 GAP_S로 돌지 않는다."""
    world = world_at((1.05, 0.55), split=split)
    world.report()
    world.traffic._blocked[("@pode1", "Q2")] = 1e9
    world.call("WORK", "pode1")
    world.run(0.3)
    ends = first_nav_ends(world)
    assert ends and ends[0] != "GAP_S"


def test_a_robot_standing_on_gap_s_plans_from_where_it_stands():
    """GAP_S의 지도 이음은 Q2와만 있다. GAP_S 위에 선 로봇이 지도 이음으로 계획하면 Q2를 거쳐 돈다."""
    world = world_at(None, split=False, start="GAP_S")
    world.report()
    world.run(0.3)
    t = world.traffic
    m = t._robots["pode1"]
    assert m.node == "GAP_S"
    way = t._way(m, "P-3")
    assert way and way[0].a == "" and "Q2" not in [edge.b for edge in way]


def test_a_via_node_is_no_free_edge_target_in_a_zone_without_nodes():
    """노드가 없는 공유 구역에서는 그 구역을 지나는 Nav2 goal 이음의 끝이 후보다. via는 빼야 한다."""
    world = world_at(None)
    t = world.traffic
    t.__dict__["_reach_memo"] = {}
    site = t.site
    gap = site.edge("Q2", "GAP_S")
    fake = [gap.__class__(gap.a, gap.b, gap.mode, gap.path, gap.points, ("NOWHERE",), gap.length)]
    real = site.edges
    site.edges = lambda: fake
    site.nodes_in = lambda zone: []
    try:
        assert "GAP_S" not in t._reach("NOWHERE")
    finally:
        site.edges = real
        del site.nodes_in
