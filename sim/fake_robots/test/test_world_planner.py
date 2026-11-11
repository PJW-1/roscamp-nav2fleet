"""Nav2 goal 흉내의 바닥과 길 찾기: keepout, 키운 벽, 돌아가기, 길 없음."""
import math

import pytest
from fake_robots.world.free_space import FreeSpace
from fake_robots.world.geom import length, path_gap
from fake_robots.world.planner import plan
from fleet_core.fleet_map import load_fleet_map

R = 0.08


@pytest.fixture(scope="module")
def d2():
    site = load_fleet_map("d2")
    return site, FreeSpace(site, R)


def test_keepout_zones_walls_and_shelves_are_not_floor(d2):
    site, space = d2
    cells = space.static
    assert space.free(cells, *site.node_pose("N_W"))
    assert space.free(cells, *site.node_pose("P-1"))
    assert not space.free(cells, *site.node_pose("X_W"))  # 갈림목(dock 구역)
    assert not space.free(cells, 1.0, 0.89)  # 급행(lane)
    assert not space.free(cells, 1.0, 0.72)  # 진열대와 급행에서 반지름 안
    assert not space.free(cells, 0.05, 0.40)  # 서쪽 벽에서 반지름 안
    assert space.free(cells, 1.0, 0.68)


def test_a_clear_straight_line_is_the_plan(d2):
    site, space = d2
    way = plan(space, space.static, site.node_pose("N_W"), site.node_pose("N_E"))
    assert way == [site.node_pose("N_W"), site.node_pose("N_E")]


def test_the_plan_goes_around_a_standing_robot_and_keeps_its_distance(d2):
    site, space = d2
    reach = 2 * R + 0.03
    cells = space.with_discs([(1.0, 0.65, reach)])
    start, goal = site.node_pose("N_W"), site.node_pose("N_E")
    way = plan(space, cells, start, goal)
    assert way[0] == start and way[-1] == goal
    assert path_gap((1.0, 0.65), way) >= 2 * R
    assert length(way) > math.dist(start, goal)


def test_no_way_when_the_goal_is_in_a_keepout_zone_or_walled_off(d2):
    site, space = d2
    assert plan(space, space.static, site.node_pose("N_W"), site.node_pose("X_W")) is None
    wall = space.with_discs([(1.0, y / 100, 0.05) for y in range(0, 80, 2)])
    assert plan(space, wall, site.node_pose("N_W"), site.node_pose("N_E")) is None


def test_layout_c_has_no_floor_way_between_north_and_south():
    site = load_fleet_map("c")
    space = FreeSpace(site, R)
    assert plan(space, space.static, site.node_pose("N_W"), site.node_pose("N_E")) is not None
    assert plan(space, space.static, site.node_pose("N_W"), site.node_pose("S_W")) is None


def test_a_goal_inside_the_grown_wall_ends_at_the_nearest_free_cell(d2):
    _, space = d2
    goal = (0.06, 0.40)
    way = plan(space, space.static, (0.40, 0.40), goal)
    assert way[-1] != goal and math.dist(way[-1], goal) <= 0.05
    assert space.free(space.static, *way[-1])


@pytest.mark.parametrize("layout", ["d2", "d", "c"])
def test_every_node_of_a_shared_zone_is_floor(layout):
    site = load_fleet_map(layout)
    space = FreeSpace(site, R)
    for node in site.nodes.values():
        if site.zones[node.zone].kind == "shared":
            assert space.free(space.static, node.x, node.y), node.id
