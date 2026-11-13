"""배치 v7(사용자 확정 경기장, 2026-11-13): 치수, 도크, 대기 L, 구역 여유, 길."""
import math

import pytest
from fleet_core.fleet_map import KNOWN_LAYOUTS, LAYOUTS, SHARED, load_fleet_map, route
from fleet_core.fleet_map.model import FOLLOW_PATH, NAV_GOAL, RETREAT

# 자유바닥 저장소 sites/arena_v7.yaml의 치수(권위 있는 값)
SHELVES = [(0.68, 0.77, 1.56, 0.81), (0.68, 0.365, 0.99, 0.405), (1.25, 0.365, 1.56, 0.405)]
DIVIDERS = [(0.24, 0.0, 0.26, 0.10), (0.50, 0.0, 0.52, 0.10), (0.76, 0.0, 0.78, 0.10)]
RELEASE_GAP_M = 0.135  # footprint_radius_m 0.085 + release_margin_m 0.05
NODE_CLEAR_M = 0.20


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("v7")


def test_v7_is_a_candidate_but_not_the_default():
    assert "v7" in LAYOUTS and "v7" in KNOWN_LAYOUTS
    assert load_fleet_map().name == "arena_d2"


def test_v7_dimensions(site):
    assert site.name == "arena_v7"
    assert site.bounds() == (0.0, 0.0, 1.98, 0.99)
    assert list(site.shelves) == SHELVES + DIVIDERS
    # 칸막이 사이 빈 폭 0.24(잠정), 건넘 틈 0.26, 급행 0.18
    assert [round(b[0] - a[2], 3) for a, b in zip(DIVIDERS, DIVIDERS[1:])] == [0.24, 0.24]
    assert round(DIVIDERS[0][0], 3) == 0.24
    assert round(SHELVES[2][0] - SHELVES[1][2], 3) == 0.26
    assert round(0.99 - SHELVES[0][3], 3) == 0.18


def test_v7_docks_and_parking(site):
    assert site.node_pose("W") == (1.88, 0.495)
    assert site.node_pose("O") == (0.285, 0.90)
    assert math.degrees(site.nodes["W"].yaw) == pytest.approx(90.0)  # 남쪽에서 들어간다
    assert math.degrees(site.nodes["O"].yaw) == pytest.approx(180.0)  # 급행 서쪽 끝
    assert site.stations["WORK"].dock == "W" and site.stations["OUTBOUND"].dock == "O"
    assert site.parking == ("P-1", "P-2", "P-3")
    for name, x in (("P-1", 0.12), ("P-2", 0.38), ("P-3", 0.64)):
        assert site.nodes[name].x == pytest.approx(x)


def test_v7_work_queue_lies_on_the_wall_l(site):
    queue = site.stations["WORK"].queue
    assert len(queue) >= 2
    for spot in queue:
        x, y = site.node_pose(spot)
        on_south_leg = y == pytest.approx(0.10) and x >= 1.56
        on_east_leg = x == pytest.approx(1.88) and y < 0.495
        assert on_south_leg or on_east_leg, spot
    # 대기 첫 자리에서만 WORK로 들어간다(고정 경로)
    into_w = [link for link in site.links() if link.b == "W"]
    assert [(link.a, link.via) for link in into_w] == [(queue[0], FOLLOW_PATH)]


def test_v7_standing_nodes_keep_clear_of_other_zones(site):
    """서는 지점(대기, 주차, 갈림)은 다른 구역에서 몸 반경+여유만큼 떨어진다: 선 로봇이 남의 구역을
    쥐지 않는다. 지점끼리는 node_clear_m 이상 떨어진다."""
    for node in site.nodes.values():
        if site.zones[node.zone].kind != SHARED:
            continue
        for zone in site.zones:
            if zone != node.zone:
                gap = site.distance_to_zone(node.x, node.y, zone)
                assert gap >= RELEASE_GAP_M - 1e-9, (node.id, zone, gap)
    names = list(site.nodes)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            gap = math.dist(site.node_pose(a), site.node_pose(b))
            assert gap >= NODE_CLEAR_M - 1e-9, (a, b, gap)


def test_v7_dock_releases_neighbours_after_docking(site):
    # 도킹을 마친 로봇은 도크에서 0.085 + 오차(시뮬 0.01) 넘게 떨어진 구역을 놓는다
    for dock in ("W", "O"):
        x, y = site.node_pose(dock)
        for zone in site.zones:
            if zone != site.nodes[dock].zone:
                assert site.distance_to_zone(x, y, zone) >= 0.10 - 1e-9, (dock, zone)


def test_v7_routes(site):
    # 급행: WORK에서 출고로 고정 경로
    way = route(site, "W", "O")
    assert [link.b for link in way] == ["X_E", "O"]
    assert all(link.via == FOLLOW_PATH for link in way)
    assert "EXPRESS" in way[-1].zones
    # 급행이 막히면 내려갈 마당 지점이 있다(yard_retreat)
    assert site.link("X_E", "N_E").via == FOLLOW_PATH
    # 출고에서는 후진으로 넓은 곳에 나온다
    assert site.link("O", "N_O").via == RETREAT
    # 가운데 통로와 남쪽 통로는 양방향 Nav2 goal
    for a, b in (("N_O", "N_E"), ("S_W", "Q2")):
        assert site.link(a, b).via == NAV_GOAL and site.link(b, a).via == NAV_GOAL
    assert "MID" in site.link("N_O", "N_E").zones
    assert "SOUTH" in site.link("S_W", "Q2").zones
    # 가운데 통로 끝에서 대기 자리로 바로 내려간다(그림의 빨간 선)
    assert site.link("N_E", "Q1").via == NAV_GOAL
    assert "WORK_DOCK" not in site.link("N_E", "Q1").zones
    for spot in site.parking:
        assert route(site, spot, "W") is not None and route(site, "O", spot) is not None


def test_v7_capacity_covers_three_robots(site):
    assert sum(z.capacity for z in site.zones.values() if z.kind == SHARED) >= 3
    assert site.zones["MID"].capacity == 1 and site.zones["SOUTH"].capacity == 1
