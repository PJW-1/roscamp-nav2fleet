"""Nav2 goal 흉내의 바닥과 길 찾기: keepout, 키운 벽, 돌아가기, 길 없음."""
import copy
import math

import pytest
from fake_robots.world.free_space import FreeSpace
from fake_robots.world.geom import length, path_gap
from fake_robots.world.planner import plan
from fleet_core.fleet_map import FleetMap, load_fleet_map

R = 0.08

# pc/fleet_core/test/test_fleet_map.py의 SMALL과 같은 작은 지도
SMALL = {
    "name": "small",
    "bounds": [0.0, 0.0, 2.0, 1.0],
    "shelves": [[0.5, 0.75, 1.5, 0.8]],
    "zones": {
        "HALL": {"kind": "shared", "capacity": 2,
                 "rects": [[0.0, 0.0, 2.0, 0.75], [0.0, 0.75, 0.5, 0.8], [1.5, 0.75, 2.0, 0.8]]},
        "DOCK_W": {"kind": "dock", "rects": [[0.0, 0.8, 0.3, 1.0]]},
        "J_W": {"kind": "dock", "rects": [[0.3, 0.8, 0.5, 1.0]]},
        "EXPRESS": {"kind": "lane", "rects": [[0.5, 0.8, 1.5, 1.0]]},
        "J_E": {"kind": "dock", "rects": [[1.5, 0.8, 1.7, 1.0]]},
        "DOCK_E": {"kind": "dock", "rects": [[1.7, 0.8, 2.0, 1.0]]},
    },
    "nodes": {
        "W": {"x": 0.2, "y": 0.9, "kind": "dock", "yaw_deg": 180},
        "O": {"x": 1.8, "y": 0.9, "kind": "dock", "yaw_deg": 0},
        "X_W": {"x": 0.4, "y": 0.9, "kind": "pass"},
        "X_E": {"x": 1.6, "y": 0.9, "kind": "pass"},
        "N_W": {"x": 0.4, "y": 0.6, "kind": "fork"},
        "N_E": {"x": 1.6, "y": 0.6, "kind": "fork"},
        "W_Q": {"x": 0.2, "y": 0.5, "kind": "wait"},
        "O_Q": {"x": 1.8, "y": 0.5, "kind": "wait"},
        "P-1": {"x": 0.2, "y": 0.15, "kind": "parking", "yaw_deg": 90},
        "P-2": {"x": 1.8, "y": 0.15, "kind": "parking"},
    },
    "paths": {
        "work_in": {"points": ["N_W", "X_W", "W"]},
        "work_out": {"points": ["W", "X_W"], "reverse": True},
        "out_in": {"points": ["N_E", "X_E", "O"]},
        "out_out": {"points": ["O", "X_E"], "reverse": True},
        "express": {"points": ["X_W", "X_E"], "two_way": True},
        "j_w_down": {"points": ["X_W", "N_W"]},
        "j_e_down": {"points": ["X_E", "N_E"]},
    },
    "nav": [{"nodes": ["N_W", "N_E", "W_Q", "O_Q", "P-1", "P-2"]}],
    "stations": {
        "WORK": {"dock": "W", "queue": ["W_Q"], "server": "workcell", "arms": ["arm1", "arm2"],
                 "action": "fill"},
        "OUTBOUND": {"dock": "O", "queue": ["O_Q"], "server": "arm3", "arms": ["arm3"],
                     "action": "swap"},
    },
    "parking": ["P-1", "P-2"],
}


def test_an_opening_only_skips_its_own_docks_keepout():
    """W에서 나가는 열린 상자 안이어도 갈림목(J_W) 구역 곁은 반지름만큼 막혀 있다."""
    data = copy.deepcopy(SMALL)
    data["nav"].append({"nodes": ["W", "N_W"], "leave_dock": True})
    site = FleetMap(data)
    space = FreeSpace(site, R)
    assert space.free(space.static, 0.2, 0.9)
    assert not space.free(space.static, 0.34, 0.80)


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


def test_v7_a_robot_at_work_plans_its_way_out_to_the_hall():
    """v7의 W는 WORK_DOCK 안이다. 그 도크를 Nav2 goal로 떠나는 이음(W -> M_E, 사용자 2026-11-14)의
    곧은 선 둘레는 바닥으로 열고, 도크의 나머지는 그대로 막는다(fleet_core keepout_openings)."""
    site = load_fleet_map("v7")
    space = FreeSpace(site, R)
    assert space.free(space.static, *site.node_pose("W"))
    assert plan(space, space.static, site.node_pose("W"), site.node_pose("M_E")) is not None
    assert not space.free(space.static, 1.88, 0.40), "the south end of the dock stays closed"
    assert not space.free(space.static, 1.80, 0.66), "so does the strip below J_E"
