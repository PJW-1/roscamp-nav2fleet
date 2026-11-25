"""배치 v7 경기장: 치수, 도크, 대기 L, 구역 여유, 길."""
import math
from itertools import pairwise

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import (
    KNOWN_LAYOUTS,
    LAYOUTS,
    PASS,
    SHARED,
    load_fleet_map,
    shortest_path,
)
from fleet_core.fleet_map.model import FOLLOW_PATH, NAV_GOAL, RETREAT

# 자유바닥 저장소 sites/arena_v7.yaml의 실측 치수(진열대 A, 진열대 둘)
SHELVES = [(0.65, 0.735, 1.56, 0.775), (0.70, 0.3475, 0.99, 0.3875), (1.25, 0.3475, 1.56, 0.3875)]
# 시뮬 바퀴 충돌 구: 중심에서 바깥 끝 거리(pinky.urdf.xacro)
SIM_WHEEL_OUTER_M = 0.04855 + 0.028
# 칸막이 둘(칸막이 3은 없다)
DIVIDERS = [(0.24, 0.0, 0.26, 0.10), (0.50, 0.0, 0.52, 0.10)]
# 북쪽 벽 안쪽 면: 지도 값과 SLAM 실측 값
NORTH_WALL_Y = 0.98
REAL_NORTH_WALL_Y = 0.975
# AMCL 지도의 동쪽, 북쪽 벽: 벽 쪽 몸 여유는 이 면으로 잰다
AMCL_EAST_WALL_X = 1.965
AMCL_NORTH_WALL_Y = 0.965
# 급행 줄 y: 실측 띠 가운데보다 조금 남쪽
LANE_Y = 0.868
# 급행 들머리 원호(W_X 다음 네 점, 끝점이 줄 위)
EXPRESS_ARC = [(1.78, 0.8466), (1.7547, 0.8584), (1.7278, 0.8656), (1.7, LANE_Y)]
BODY_HALF_M = 0.056          # local footprint 반폭(nav2_params)
RELEASE_GAP_M = 0.135  # footprint_radius_m 0.085 + release_margin_m 0.05
NODE_CLEAR_M = 0.20


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("v7")


def test_v7_is_a_candidate_and_the_default():
    assert "v7" in LAYOUTS and "v7" in KNOWN_LAYOUTS
    assert load_fleet_map().name == "arena_v7"
    assert {"d2", "d", "c", "d2_shelf"} <= set(KNOWN_LAYOUTS)


def test_v7_dimensions(site):
    assert site.name == "arena_v7"
    assert site.bounds() == (0.0, 0.0, 1.98, NORTH_WALL_Y)
    assert list(site.shelves) == SHELVES + DIVIDERS
    # 칸막이 사이 빈 폭, 건넘 틈
    assert [round(b[0] - a[2], 3) for a, b in pairwise(DIVIDERS)] == [0.24]
    assert round(DIVIDERS[0][0], 3) == 0.24
    assert round(SHELVES[2][0] - SHELVES[1][2], 3) == 0.26
    assert round(NORTH_WALL_Y - SHELVES[0][3], 3) == 0.205  # 지도 벽 기준
    assert round(REAL_NORTH_WALL_Y - SHELVES[0][3], 3) == 0.2  # 실측 벽 기준
    # 진열대 A는 폭 0.91, 동쪽 끝 1.56
    assert round(SHELVES[0][2] - SHELVES[0][0], 3) == 0.91 and SHELVES[0][2] == 1.56
    # 세 진열대 모두 두께 0.04이고, 급행을 뺀 나머지를 아래 통로 둘이 같은 폭으로 나눈다
    assert {round(s[3] - s[1], 4) for s in SHELVES} == {0.04}
    south, middle = SHELVES[1][1], SHELVES[0][1] - SHELVES[1][3]
    assert round(south, 4) == round(middle, 4) == 0.3475
    assert SHELVES[1][1:4:2] == SHELVES[2][1:4:2]
    # 진열대 자리는 어느 구역도 아니고 그 서쪽 띠는 HALL이다
    for y in (0.74, 0.755, 0.772):
        assert site.zone_at(0.66, y) is None and site.zone_at(0.59, y) == "HALL", y


def test_v7_sim_shelves_are_the_measured_shelves(site):
    assert list(site.shelves) == SHELVES + DIVIDERS
    assert list(site.sim_shelves) == SHELVES + DIVIDERS


def test_v7_sim_wheel_spheres_clear_shelf_a_on_the_express_lane(site):
    lane_y = site.nodes["O"].y
    assert lane_y == site.nodes["O_X"].y == 0.868
    clearance = lane_y - SIM_WHEEL_OUTER_M - site.sim_shelves[0][3]
    assert clearance >= 0.005, clearance


def test_v7_divider_3_is_gone_and_p3_opens_to_the_east(site):
    """3번 칸막이가 없어 P-3 동쪽에는 진열대도 칸막이도 keepout도 없다."""
    assert site.parking == ("P-1", "P-2", "P-3")
    assert site.node_pose("P-3") == pytest.approx((0.64, 0.10))
    ex0, ey0, ex1, ey1 = 0.66, 0.0, 1.0, 0.30
    for x0, y0, x1, y1 in list(site.shelves) + list(site.keepout_extra):
        assert x1 <= ex0 or x0 >= ex1 or y1 <= ey0 or y0 >= ey1, (x0, y0, x1, y1)


def test_v7_docks_and_parking(site):
    # W는 동쪽 벽에서, O는 북쪽 벽에서 2 cm 안쪽이다
    assert site.node_pose("W") == (1.86, 0.495)
    assert site.node_pose("O") == (0.285, LANE_Y)
    assert math.degrees(site.nodes["W"].yaw) == pytest.approx(90.0)  # 남쪽에서 들어간다
    assert math.degrees(site.nodes["O"].yaw) == pytest.approx(180.0)  # 급행 서쪽 끝
    assert site.stations["WORK"].dock == "W" and site.stations["OUTBOUND"].dock == "O"
    assert site.parking == ("P-1", "P-2", "P-3")
    for name, x in (("P-1", 0.12), ("P-2", 0.38), ("P-3", 0.64)):
        assert site.nodes[name].x == pytest.approx(x)
        # 후진 충돌 검사를 켠 채 닿는 가장 깊은 자리다
        assert site.nodes[name].y == pytest.approx(0.10)
        assert 0.025 < site.nodes[name].y - REAR_M < 0.03
    # 칸 입구는 주차 자리에서 0.14 북쪽이다
    for name, x in (("E-1", 0.12), ("E-2", 0.38), ("E-3", 0.64)):
        assert site.node_pose(name) == pytest.approx((x, 0.24))
        spot = site.node_pose("P-" + name.removeprefix("E-"))
        assert math.dist(site.node_pose(name), spot) == pytest.approx(0.14)


def test_v7_work_queue_is_two_spots_entered_at_q2(site):
    """WORK 줄: 앞자리 Q1은 W 바로 남쪽, 들어오는 자리 Q2는 남동 구석이다. 줄 서기를 켠다."""
    station = site.stations["WORK"]
    assert station.queue == ("Q1", "Q2") and station.line
    assert not site.stations["OUTBOUND"].line
    assert not {"Q3", "G", "F"} & set(site.nodes)
    q1, q2 = site.node_pose("Q1"), site.node_pose("Q2")
    assert q1[0] > 1.80 and q2[0] < q1[0] and q2[1] < 0.20, (q1, q2)  # 구석
    # Nav2 goal로 닿는 자리와 줄 안에서 도는 점은 벽에서 0.13 이상이다
    turns = [q1, q2] + list(site.edge("Q2", "Q1").points[1:-1])
    for x, y in turns:
        assert min(1.98 - x, y) >= WALL_NAV_M - 1e-9, (x, y)
    # 앞자리는 W로 드는 원호 위에서 원호 방향을 보고, 두 자리 사이는 노드 놓는 거리보다 멀다
    assert math.dist(q1, q2) > load_params().node_clear_m + 0.015
    # Q2에 선 로봇과 앞자리 로봇의 몸이 겹치지 않는다
    yaw1 = site.nodes["Q1"].yaw
    c, s = math.cos(yaw1), math.sin(yaw1)
    front_body_west = min(q1[0] + c * u - s * v for u, v in FOOTPRINT)
    assert q2[0] + FRONT_M < front_body_west, (q2, front_body_west)
    # 흐름: Q2 -> 앞자리 -> W, 한 방향 고정 경로
    for a, b in (("Q2", "Q1"), ("Q1", "W")):
        assert site.edge(a, b).mode == FOLLOW_PATH, (a, b)
        assert site.edge(b, a) is None, (b, a)
    into_w = [(edge.a, edge.mode) for edge in site.edges() if edge.b == "W"]
    assert into_w == [("Q1", FOLLOW_PATH)]
    # 줄 밖에서 W로 가는 길은 모두 Q2로 들어와 Q1을 거친다
    for start in site.nodes:
        if start in station.queue or start == "W":
            continue
        way = shortest_path(site, start, "W")
        assert way is not None and [edge.b for edge in way][-3:] == ["Q2", "Q1", "W"], start


def test_v7_standing_nodes_keep_clear_of_other_zones(site):
    """서는 지점은 다른 구역에서 몸 반경 + 여유만큼, 지점끼리는 node_clear_m 이상 떨어진다."""
    for node in site.nodes.values():
        if site.zones[node.zone].kind != SHARED:
            continue
        for zone in site.zones:
            if zone != node.zone:
                gap = site.distance_to_zone(node.x, node.y, zone)
                assert gap >= RELEASE_GAP_M - 1e-9, (node.id, zone, gap)
    # 출고의 두 점은 OUT_DOCK에서 놓기 거리보다 조금 더 떨어진다
    for name in ("O_IN", "O_OUT"):
        node = site.nodes[name]
        gap = site.distance_to_zone(node.x, node.y, "OUT_DOCK")
        assert RELEASE_GAP_M + 0.005 <= gap <= RELEASE_GAP_M + 0.02, (name, gap)
    # 충전 칸과 그 입구는 짝이다
    pairs = {tuple(sorted((spot, _bay_door(spot)))) for spot in site.parking}
    names = list(site.nodes)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if tuple(sorted((a, b))) in pairs:
                continue
            gap = math.dist(site.node_pose(a), site.node_pose(b))
            assert gap >= NODE_CLEAR_M - 1e-9, (a, b, gap)


def _bay_door(spot: str) -> str:
    return "E-" + spot.removeprefix("P-")


@pytest.mark.parametrize("spot", ["P-1", "P-2", "P-3"])
def test_v7_a_robot_parked_in_a_bay_cuts_no_other_way(site, spot):
    """칸에 선 로봇이 E-k를 쥐어도 나머지 지점끼리는 E-k를 거치지 않고 오갈 수 있다."""
    door = _bay_door(spot)
    others = [name for name in site.nodes if name not in (door, spot)]

    def cost(edge):
        return math.inf if door in (edge.a, edge.b) else edge.length
    for a in others:
        for b in others:
            if a != b:
                assert shortest_path(site, a, b, cost) is not None, (spot, a, b)


DIVIDER_TOP_M = 0.10
TURN_MARGIN_M = 0.02


def test_v7_bays_are_entered_backwards_and_left_forwards_on_a_straight_line(site):
    """칸 안에서는 돌지 않고 입구에서 곧게 후진해 들어가 앞으로 곧게 나온다."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import point_rect_distance, point_segment_distance
    for spot in site.parking:
        door = _bay_door(spot)
        p, e = site.nodes[spot], site.nodes[door]
        assert e.x == pytest.approx(p.x) and math.degrees(e.yaw) == pytest.approx(90.0)
        assert math.degrees(p.yaw) == pytest.approx(90.0)
        # 입구에서 도는 몸 원이 칸막이 끝과 진열대에 닿지 않는다
        assert e.y >= DIVIDER_TOP_M + BODY_CIRCLE_M + TURN_MARGIN_M
        for shelf in site.shelves:
            assert point_rect_distance(shelf, e.x, e.y) >= BODY_CIRCLE_M + TURN_MARGIN_M, (door,
                                                                                         shelf)
        into = [(edge.a, edge.mode) for edge in site.edges() if edge.b == spot]
        out = [(edge.b, edge.mode) for edge in site.edges() if edge.a == spot]
        assert into == [(door, RETREAT)] and out == [(door, FOLLOW_PATH)]
        for edge in (site.edge(door, spot), site.edge(spot, door)):
            assert len(edge.points) == 2  # 곧은 선
            for shelf in DIVIDERS:
                gap = min(point_segment_distance(((shelf[0] + shelf[2]) / 2, shelf[3]), a, b)
                          for a, b in pairwise(edge.points))
                assert gap >= HALF_WIDTH_M + TURN_MARGIN_M, (spot, shelf, gap)
    # 마당 Nav2 goal은 칸 안을 지나지 않는다
    for edge in site.edges():
        if edge.mode != NAV_GOAL:
            continue
        assert not {edge.a, edge.b} & set(site.parking), (edge.a, edge.b)
        for shelf in DIVIDERS:
            steps = max(2, int(edge.length / 0.01))
            (ax, ay), (bx, by) = edge.points[0], edge.points[-1]
            gap = min(point_rect_distance(shelf, ax + (bx - ax) * i / steps,
                                          ay + (by - ay) * i / steps) for i in range(steps + 1))
            assert gap >= BODY_CIRCLE_M + TURN_MARGIN_M, (edge.a, edge.b, shelf, round(gap, 3))
    # 진열대 모서리도 곧은 선에서 몸 원보다 멀다
    for edge in site.edges():
        if edge.mode != NAV_GOAL:
            continue
        from fleet_core.fleet_map.geometry import segment_rect_distance
        gap = min(segment_rect_distance(edge.points[0], edge.points[-1], shelf)
                  for shelf in site.shelves)
        assert gap >= BODY_CIRCLE_M, (edge.a, edge.b, round(gap, 3))


def test_v7_dock_releases_neighbours_after_docking(site):
    # 도킹을 마친 로봇은 멀어진 구역을 놓는다. 예외는 W와 서쪽 HALL(0.08)이다
    for dock in ("W", "O"):
        x, y = site.node_pose(dock)
        for zone in site.zones:
            if zone != site.nodes[dock].zone:
                need = 0.08 if (dock, zone) == ("W", "HALL") else 0.10
                assert site.distance_to_zone(x, y, zone) >= need - 1e-9, (dock, zone)


def test_v7_routes(site):
    # 급행: WORK에서 출고로 고정 경로
    way = shortest_path(site, "W", "O")
    assert [edge.b for edge in way] == ["W_X", "O_X", "O"]
    assert all(edge.mode == FOLLOW_PATH for edge in way)
    assert "EXPRESS" in way[1].zones and way[2].zones == ("EXPRESS", "OUT_DOCK")
    # WORK에서 나가는 길은 급행 들머리 W_X(고정 경로)와 넓은 곳 M_E(Nav2 goal) 둘이다
    exit_ = site.nodes["W_X"]
    # W_X는 원호의 50도 점이다(원호 시험)
    assert exit_.kind == PASS and exit_.zone == "J_E" and exit_.x == pytest.approx(1.8028)
    assert site.distance_to_zone(exit_.x, exit_.y, "WORK_DOCK") >= RELEASE_GAP_M
    assert sorted((edge.b, edge.mode) for edge in site.edges() if edge.a == "W") == [
        ("M_E", NAV_GOAL), ("W_X", FOLLOW_PATH)]
    assert site.edge("W", "M_E").zones == ("WORK_DOCK", "HALL")
    assert [edge.a for edge in site.edges() if edge.b == "W"] == ["Q1"]
    # W_X는 W와 급행만 잇는다
    assert [(k.b, k.mode) for k in site.edges() if k.a == "W_X"] == [("O_X", FOLLOW_PATH)]
    assert [k.a for k in site.edges() if k.b == "W_X"] == ["W"]
    assert not {"X_E", "N_E", "N_O", "O_Q"} & set(site.nodes)

    def no_express(edge):
        return math.inf if "EXPRESS" in edge.zones else edge.price_m
    hall = shortest_path(site, "W", "O", no_express)
    assert [edge.b for edge in hall] == ["M_E", "O_IN", "O"]
    # 넓은 곳 쪽이 0.09 m 짧지만 extra_m을 더해 급행이 비면 급행을 고른다
    from fleet_core.traffic.zone.goals import TRANSIT_WAIT_S
    express_m, hall_m = (sum(edge.price_m for edge in w) for w in (way, hall))
    assert sum(edge.length for edge in hall) < sum(edge.length for edge in way)
    assert express_m < hall_m < express_m + TRANSIT_WAIT_S * load_params().nav_speed_mps
    # 출고: 들어오는 점 O_IN과 나가는 점 O_OUT을 나눈다
    assert sorted(edge.a for edge in site.edges() if edge.b == "O") == ["O_IN", "O_X"]
    assert [edge.b for edge in site.edges() if edge.a == "O"] == ["O_OUT"]
    out, into = site.edge("O", "O_OUT"), site.edge("O_IN", "O")
    assert out.mode == FOLLOW_PATH and into.mode == FOLLOW_PATH
    assert "EXPRESS" not in into.zones
    o, o_in, o_out = (site.node_pose(n) for n in ("O", "O_IN", "O_OUT"))
    assert o_out[0] < o[0] < o_in[0]
    # O_IN은 OUT_DOCK 놓기 여유 안에서 가장 위, 동쪽 끝 안쪽이다
    assert o_in[:2] == pytest.approx((0.49, 0.625))
    first = into.points[1]
    heading = math.degrees(math.atan2(first[1] - o_in[1], first[0] - o_in[0]))
    assert math.degrees(site.nodes["O_IN"].yaw) == pytest.approx(heading, abs=2.0)
    assert out.points[1][0] < o[0] - 0.05, "first goes west, the way O faces"
    assert all(b[1] <= a[1] + 1e-9 for a, b in pairwise(out.points[1:])), "then south"
    assert min(x for x, _ in out.points) - BODY_CIRCLE_M >= 0.05, "clear of the west wall"
    # O_OUT은 벽에서 0.18, 마지막 구간이 곧게 남쪽이다
    assert o_out[0] == pytest.approx(0.18) and out.points[-2][0] == pytest.approx(o_out[0])
    assert out.points[-2][1] == pytest.approx(o[1] - 0.028), (
        "꺾는 점은 O보다 0.028 아래(옛 0.888 -> 0.86과 같다)")
    # 나가는 점으로 들어오는 길도, 들어오는 점에서 나가는 길도 없다
    fixed = [edge for edge in site.edges() if edge.mode != NAV_GOAL]
    assert [edge.a for edge in fixed if edge.b == "O_OUT"] == ["O"]
    assert [edge.b for edge in fixed if edge.a == "O_IN"] == ["O"]
    # 출고가 바쁘면 O_IN이나 O_X에서 기다린다
    assert site.stations["OUTBOUND"].queue == ("O_IN", "O_X")
    # 마당 지점끼리는 Nav2 goal로 오가고 HALL만 잡는다
    yard = ["O_IN", "O_OUT", "M_E", "Q2", "E-1", "E-2", "E-3"]
    for a in yard:
        for b in yard:
            edge = site.edge(a, b)
            if edge is not None:
                assert edge.mode == NAV_GOAL and edge.zones == ("HALL",), (a, b, edge.zones)
    # 칸 입구 셋은 서로, 그리고 출고의 두 점과 곧게 이어진다(줄 입구 Q2와는 E-3만)
    doors = ["E-1", "E-2", "E-3"]
    pairs = [(a, b) for a in doors for b in doors if a != b]
    pairs += [(door, "O_OUT") for door in doors] + [("E-3", "Q2")]
    for door in ("E-1", "E-2"):
        assert site.edge(door, "Q2") is None and site.edge("Q2", door) is None, door
    # O_IN은 E-1, E-2와만 잇는다
    pairs += [("O_IN", "E-1"), ("O_IN", "E-2")]
    assert site.edge("O_IN", "E-3") is None and site.edge("E-3", "O_IN") is None
    # 우회로: 가운데 통로 동쪽 끝 M_E는 O_IN과 Q2만 잇는다
    pairs += [("O_OUT", "O_IN"), ("O_IN", "M_E"), ("M_E", "Q2")]
    for a, b in pairs:
        assert site.edge(a, b) is not None and site.edge(b, a) is not None, (a, b)
    assert sorted(edge.b for edge in site.edges() if edge.a == "M_E") == ["O_IN", "Q2"]
    # 평소에는 남쪽 통로가 짧고, 못 쓰면 가운데 통로로 돈다
    for door, way in (("E-1", ["E-3", "Q2"]), ("E-2", ["E-3", "Q2"]), ("E-3", ["Q2"])):
        assert [edge.b for edge in shortest_path(site, door, "Q2")] == way, door
    assert sum(edge.length for edge in shortest_path(site, "E-1", "Q2")) == pytest.approx(
        math.dist(site.node_pose("E-1"), site.node_pose("Q2")), abs=0.002)

    def south_shut(edge):
        return math.inf if "Q2" in (edge.a, edge.b) and "M_E" not in (edge.a, edge.b) \
            else edge.length
    for start in (*doors, "O_OUT"):
        way = [edge.b for edge in shortest_path(site, start, "Q2", south_shut)]
        assert way[-3:] == ["O_IN", "M_E", "Q2"], (start, way)
    # 진열대를 곧게 가로지르는 Nav2 이음이 없다
    from fleet_core.fleet_map.geometry import inside
    for edge in site.edges():
        if edge.mode != NAV_GOAL:
            continue
        (ax, ay), (bx, by) = edge.points[0], edge.points[-1]
        for k in range(101):
            x, y = ax + (bx - ax) * k / 100, ay + (by - ay) * k / 100
            assert not any(inside(shelf, x, y) for shelf in site.shelves), (edge.a, edge.b)
    for spot in site.parking:
        assert (shortest_path(site, spot, "W") is not None
                and shortest_path(site, "O", spot) is not None)


def test_v7_queue_is_entered_at_q2_and_flows_one_way(site):
    """줄에 닿는 이음은 E-3 <-> Q2뿐이고 앞자리 Q1은 줄 안에서만 닿아 W로만 나간다."""
    touching = sorted((edge.a, edge.b, edge.mode) for edge in site.edges()
                      if {"Q1", "Q2"} & {edge.a, edge.b})
    doors = ["E-3"]  # E-1, E-2는 E-3을 거친다(test_v7_routes)
    doors.append("M_E")
    doors.append("GAP_S")  # 해결안 B의 중간 지점
    assert touching == sorted([(door, "Q2", NAV_GOAL) for door in doors]
                              + [("Q2", door, NAV_GOAL) for door in doors]
                              + [("Q1", "W", FOLLOW_PATH), ("Q2", "Q1", FOLLOW_PATH)])
    # Q2는 도착 방향(동쪽)을 본다
    assert math.degrees(site.nodes["Q2"].yaw) == pytest.approx(0.0)
    # 앞자리는 W로 드는 원호 방향을 본다(원호 시험)
    assert math.degrees(site.nodes["Q1"].yaw) == pytest.approx(69.3, abs=0.05)


W_ENTRY_R = 0.17


def test_v7_w_entry_is_one_wide_arc_through_q1(site):
    """Q2에서 W까지 원호 하나로 가고 앞자리 Q1은 그 원호 위에 둔다."""
    q_step, work_in = site.edge("Q2", "Q1"), site.edge("Q1", "W")
    q2, q1, w = site.node_pose("Q2"), site.node_pose("Q1"), site.node_pose("W")
    route_ = list(q_step.points) + list(work_in.points[1:])
    centre = (w[0] - W_ENTRY_R, q2[1] + W_ENTRY_R)
    # Q2에서 원호 시작까지 동쪽으로, 원호 끝에서 W까지 북쪽으로 곧게
    assert route_[1] == pytest.approx((centre[0], q2[1]), abs=1e-4), route_[1]
    assert route_[-2] == pytest.approx((w[0], centre[1]), abs=1e-4), route_[-2]
    arc = route_[1:-1]
    for p in arc:
        assert math.dist(p, centre) == pytest.approx(W_ENTRY_R, abs=1e-3), p
    heads = [math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) for p, q in pairwise(arc)]
    assert all(0.0 < b - a <= 10.5 for a, b in pairwise([0.0, *heads, 90.0])), heads
    # 앞자리는 원호 위, 원호 방향을 본다
    assert math.dist(q1, centre) == pytest.approx(W_ENTRY_R, abs=1e-3)
    tangent = math.degrees(math.atan2(q1[1] - centre[1], q1[0] - centre[0])) + 90.0
    assert math.degrees(site.nodes["Q1"].yaw) == pytest.approx(tangent, abs=0.1)
    assert q1 in q_step.points and q_step.points[-1] == q1
    # 앞자리는 WORK_DOCK을 쥐지 않는다
    assert site.distance_to_zone(*q1, "WORK_DOCK") >= RELEASE_GAP_M - 1e-9
    # 길 끝에서 2 cm 못 미쳐 서도 Q2를 놓는다
    at = math.atan2(q1[1] - centre[1], q1[0] - centre[0]) - 0.02 / W_ENTRY_R
    short = (centre[0] + W_ENTRY_R * math.cos(at), centre[1] + W_ENTRY_R * math.sin(at))
    assert math.dist(short, q2) >= load_params().node_clear_m, short
    # 몸과 AMCL 동쪽 벽의 최소 거리는 원호 80도 근처다
    east, _, _ = _body_gaps(route_, 1.60)
    assert east >= 0.045, east
    assert site.warnings == (), site.warnings


BODY_CIRCLE_M = 0.083  # Pode 뒤 모서리 회전 반지름
WALL_NAV_M = 0.13      # Nav2 goal로 닿는 자리와 벽 사이
FRONT_M, REAR_M = 0.04, 0.0745
HALF_WIDTH_M = 0.056   # 지나가는 로봇의 반 폭(local footprint)


def test_v7_robots_standing_in_line_or_at_a_fork_leave_every_other_way_open(site):
    """오래 서는 자리의 몸 원과 그 자리를 거치지 않는 길 사이에 지나가는 로봇 반 폭이 들어간다."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import point_segment_distance
    # G, F: 갈림에서도 차례를 기다리며 선다
    # 지나가는 로봇은 주차한 로봇 앞을 몸이 닿지 않게 지난다(E-1, E-2 -> Q2는 가까워 잇지 않는다)
    low = {}
    for edge in site.edges():
        if edge.mode != NAV_GOAL:
            continue
        for spot in site.parking:
            gap = point_segment_distance(site.node_pose(spot), edge.points[0], edge.points[-1])
            if gap < 0.175:
                low[frozenset((edge.a, edge.b)), spot] = round(gap, 3)
    assert min(low.values()) >= FRONT_M + HALF_WIDTH_M + 0.03, low
    # 입구 줄과 E-3 -> Q2는 주차한 로봇에서 약 0.14를 지난다
    assert min(low.values()) == pytest.approx(0.139, abs=1e-3)
    standing = ["W", "O", "Q1", "Q2", "O_IN", "O_OUT", "M_E", "O_X"]
    for name in standing:
        here = site.node_pose(name)
        for edge in site.edges():
            if name in (edge.a, edge.b):
                continue
            gap = min(point_segment_distance(here, a, b) for a, b in pairwise(edge.points))
            assert gap >= BODY_CIRCLE_M + HALF_WIDTH_M, (name, edge.a, edge.b, round(gap, 3))
            # 고정 경로는 비켜 가지 못하므로 지나가는 로봇도 몸 원으로 본다
            if edge.mode != NAV_GOAL:
                assert gap >= 2 * BODY_CIRCLE_M + 0.01, (name, edge.a, edge.b, round(gap, 3))


def test_v7_fixed_path_bends_leave_the_body_clear_of_walls_and_shelves(site):
    """고정 경로가 꺾이는 점에서 몸 원이 벽, 진열대, 칸막이에 닿지 않고 곧은 선도 같다."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import segment_rect_distance, wall_gap
    for edge in site.edges():
        if edge.mode == NAV_GOAL:
            continue
        for x, y in edge.points[1:-1]:
            assert wall_gap(site.bounds(), x, y) > BODY_CIRCLE_M, (edge.a, edge.b, (x, y))
        for a, b in pairwise(edge.points):
            # 급행 줄의 곧은 구간은 몸 반폭 밖이면 되고 나머지는 몸 원으로 본다
            on_lane = (edge.path in ("express", "out_in") and b[0] < a[0]
                       and a[1] == pytest.approx(LANE_Y) and b[1] == pytest.approx(LANE_Y))
            need = BODY_HALF_M if on_lane else BODY_CIRCLE_M
            for shelf in site.shelves:
                gap = segment_rect_distance(a, b, shelf)
                assert gap > need, (edge.a, edge.b, shelf, round(gap, 3))


def test_v7_outbound_waits_at_the_west_end_of_the_express_too(site):
    """W에서 오는 로봇은 출고가 바빠도 급행 서쪽 끝 O_X에서 기다리고 OUT_DOCK은 놓는다."""
    from fleet_core.fleet_map.geometry import point_rect_distance
    spot = site.nodes["O_X"]
    assert (spot.kind, spot.zone) == ("wait", "EXPRESS") and site.can_wait("O_X")
    assert math.degrees(spot.yaw) == pytest.approx(180.0)
    # 선 로봇이 OUT_DOCK을 놓도록 동쪽 끝 + 놓기 거리 + 위치 오차 여유 밖이다
    assert site.distance_to_zone(spot.x, spot.y, "OUT_DOCK") >= RELEASE_GAP_M + 0.02
    assert spot.y == pytest.approx(LANE_Y)
    # 진열대 위 면에서 몸 반폭 밖이다(서쪽만 보고 선다)
    assert point_rect_distance(SHELVES[0], spot.x, spot.y) > BODY_HALF_M
    # HALL 띠에서 0.098 떨어져 있다
    assert spot.x == pytest.approx(0.71)
    assert 0.09 < site.distance_to_zone(spot.x, spot.y, "HALL") < RELEASE_GAP_M
    # 선 로봇의 몸은 HALL에 들지 않고 급행으로 오는 길도 HALL을 쥐지 않는다
    assert site.distance_to_zone(spot.x, spot.y, "HALL") > 0.085
    assert "HALL" not in site.edge("W_X", "O_X").zones
    # 나가는 길은 출고 하나, 들어오는 길은 급행 하나
    assert [(k.b, k.mode) for k in site.edges() if k.a == "O_X"] == [("O", FOLLOW_PATH)]
    assert [k.a for k in site.edges() if k.b == "O_X"] == ["W_X"]
    assert site.edge("O_X", "W_X") is None and site.edge("O", "O_X") is None
    # out_in과 out_out은 O_X에 선 로봇을 몸 둘 + 0.01 넘게 비켜 간다
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import point_segment_distance
    gaps = {name: min(point_segment_distance(site.node_pose("O_X"), a, b)
                      for a, b in pairwise(site.edge(*pair).points))
            for name, pair in (("out_in", ("O_IN", "O")), ("out_out", ("O", "O_OUT")))}
    assert gaps["out_in"] == pytest.approx(0.291, abs=1e-3), gaps
    assert gaps["out_out"] > gaps["out_in"] >= 2 * BODY_CIRCLE_M + 0.01, gaps


def test_v7_locks_only_the_express_strip_and_the_docks(site):
    kinds = {name: zone.kind for name, zone in site.zones.items()}
    assert kinds == {"HALL": SHARED, "WORK_DOCK": "dock", "J_E": "dock", "EXPRESS": "lane",
                     "OUT_DOCK": "dock"}
    assert site.zones["HALL"].capacity == 3 and site.zones["EXPRESS"].capacity == 2


def test_v7_global_costmap_keeps_a_way_between_the_south_shelf_end_and_work(site):
    """빨간 선이 남쪽 진열대 동쪽 끝과 WORK_DOCK keepout 사이에 5 cm 전역 costmap으로 한 칸 이상 지난다."""
    from fleet_core.fleet_map.geometry import inside
    from fleet_core.fleet_map.keepout import keepout_rects
    res, inscribed = 0.05, 0.09
    keep = keepout_rects(site)
    shelf = SHELVES[2]
    occupied = [(math.floor((shelf[0] + 0.15) / res + 1e-6) + i) for i in range(
        math.ceil((shelf[2] + 0.15) / res - 1e-6) - math.floor((shelf[0] + 0.15) / res + 1e-6))]
    last_x = (occupied[-1] + 0.5) * res - 0.15  # 진열대가 걸친 가장 동쪽 칸의 중심
    for y in (0.375, 0.425):  # 진열대 끝 높이의 두 줄
        free = []
        for i in range(occupied[-1] + 1, occupied[-1] + 10):
            cx = (i + 0.5) * res - 0.15
            if cx - last_x > inscribed and not any(inside(r, cx, y) for r in keep):
                free.append(cx)
            elif free:
                break
        assert len(free) >= 1, (y, free)


def test_v7_north_wall_is_the_slam_measurement_and_the_express_stays_open(site):
    """북쪽 벽에 붙은 구역은 지도 벽 0.98까지고 급행 줄 경로 점은 모두 띠 안쪽이다."""
    assert site.bounds()[3] == NORTH_WALL_Y
    assert site.node_pose("O") == (0.285, LANE_Y) and site.node_pose("O_X") == (0.71, LANE_Y)
    # 줄은 실측 띠 가운데보다 1 cm 안쪽으로 남쪽이다
    middle = (REAL_NORTH_WALL_Y + SHELVES[0][3]) / 2
    assert middle == pytest.approx(0.875) and 0 < middle - LANE_Y < 0.01
    assert min(REAL_NORTH_WALL_Y - LANE_Y, LANE_Y - SHELVES[0][3]) > BODY_HALF_M
    # AMCL 북쪽 벽에서도 줄 위 몸 여유가 4 cm 남는다
    assert AMCL_NORTH_WALL_Y - LANE_Y - BODY_HALF_M >= 0.04
    for edge in site.edges():
        for x, y in edge.points:
            if SHELVES[0][0] <= x <= SHELVES[0][2] and y > SHELVES[0][3]:
                assert y == pytest.approx(LANE_Y), (edge.a, edge.b, x, y)
    for name in ("EXPRESS", "J_E", "OUT_DOCK"):
        assert max(r[3] for r in site.zones[name].rects) == NORTH_WALL_Y, name
    for _x0, _y0, _x1, y1 in site.keepout_extra:
        assert y1 <= NORTH_WALL_Y
    # 구역은 진열대 면을 따른다
    assert [r[1] for r in site.zones["EXPRESS"].rects] == [SHELVES[0][3]]
    # 띠는 진열대 끝에서 몸 반폭 안까지다
    west = [r for r in site.zones["HALL"].rects if r[1] < SHELVES[0][3] and r[3] > SHELVES[0][1] and r[0] < SHELVES[0][0]]
    assert {r[2] for r in west} == {0.594}
    assert min(r[1] for r in west) <= SHELVES[0][1] and max(r[3] for r in west) == SHELVES[0][3]
    assert round(SHELVES[0][0] - 0.594, 3) == BODY_HALF_M


def _y_at(points, x):
    """서쪽으로 가는 꺾은선에서 x 자리의 y."""
    for (ax, ay), (bx, by) in pairwise(points):
        if bx <= x <= ax and ax > bx:
            return ay + (by - ay) * (ax - x) / (ax - bx)
    raise AssertionError(x)


def test_v7_express_runs_straight_on_the_lane_from_the_arc_to_o(site):
    """O_X와 O의 방향, 급행 두 edge의 점, 원호 끝부터 O까지 곧은 줄을 본다."""
    into, out = site.edge("W_X", "O_X"), site.edge("O_X", "O")
    w_x, o_x, o = site.node_pose("W_X"), site.node_pose("O_X"), site.node_pose("O")
    assert w_x == (1.8028, 0.8306) and o_x == (0.71, LANE_Y) and o == (0.285, LANE_Y)
    assert math.degrees(site.nodes["O_X"].yaw) == pytest.approx(180.0)
    assert math.degrees(site.nodes["O"].yaw) == pytest.approx(180.0)
    assert list(into.points) == [w_x, *EXPRESS_ARC, o_x]
    assert list(out.points) == [o_x, o]
    route_ = list(into.points) + list(out.points[1:])
    for x in (EXPRESS_ARC[-1][0], SHELVES[0][2], 1.40, 1.20, 1.00, 0.80, SHELVES[0][0], o_x[0],
              0.50, 0.40, o[0]):
        assert _y_at(route_, x) == pytest.approx(LANE_Y), x


# local footprint(nav2_params.yaml의 local_costmap footprint)
FOOTPRINT = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049),
             (-0.071, 0.042), (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042),
             (-0.049, -0.049), (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031),
             (0.040, -0.021)]


def _body_gaps(points, x_min):
    """경로를 따라가며 발자국과 AMCL 동쪽, 북쪽 벽, 진열대 A 사이 최소 거리를 잰다."""
    from fleet_core.fleet_map.geometry import point_rect_distance, point_segment_distance
    shelf = SHELVES[0]
    corners = [(shelf[0], shelf[1]), (shelf[2], shelf[1]),
               (shelf[2], shelf[3]), (shelf[0], shelf[3])]
    east = north = near = math.inf
    for (ax, ay), (bx, by) in pairwise(points):
        yaw, length = math.atan2(by - ay, bx - ax), math.dist((ax, ay), (bx, by))
        c, s = math.cos(yaw), math.sin(yaw)
        n = max(1, math.ceil(length / 0.005))
        for i in range(n + 1):
            x, y = ax + (bx - ax) * i / n, ay + (by - ay) * i / n
            if x < x_min:
                continue
            body = [(x + c * u - s * v, y + s * u + c * v) for u, v in FOOTPRINT]
            east = min(east, AMCL_EAST_WALL_X - max(p[0] for p in body))
            north = min(north, AMCL_NORTH_WALL_Y - max(p[1] for p in body))
            near = min(near, min(point_rect_distance(shelf, *p) for p in body),
                       min(point_segment_distance(k, p, q) for k in corners
                           for p, q in pairwise(body + body[:1])))
    return east, north, near


def test_v7_express_turn_is_a_tangent_arc_from_a_lower_w_x(site):
    """급행 들머리 원호는 HALL 안 y 0.698에서 북쪽으로 시작해 반지름 0.16으로 급행 줄에 접해 끝난다."""
    out, express = site.edge("W", "W_X"), site.edge("W_X", "O_X")
    w, exit_ = site.node_pose("W"), site.nodes["W_X"]
    assert exit_.zone == "J_E" and site.zone_at(exit_.x, exit_.y) == "J_E"
    assert out.points[:2] == (w, (w[0], 0.708)), "W에서 원호 시작까지는 곧게 북쪽"
    route_ = list(out.points[1:]) + list(express.points[1:])
    last = next(i for i, p in enumerate(route_) if p[1] == pytest.approx(LANE_Y))
    arc = route_[:last + 1]
    centre = (arc[-1][0], arc[0][1])
    radius = w[0] - centre[0]
    assert LANE_Y - centre[1] == pytest.approx(radius, abs=1e-3)
    # 원호 반지름 0.16: 더 크면 원호 끝이 진열대 끝 검사(0.15)에 걸린다
    assert radius == pytest.approx(0.16, abs=1e-3)
    assert site.warnings == (), site.warnings
    for p in arc:
        assert math.dist(p, centre) == pytest.approx(radius, abs=1e-3), p
    heads = [math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) for p, q in pairwise(arc)]
    assert all(0.0 < b - a <= 10.5 for a, b in pairwise([90.0, *heads, 180.0])), heads
    # 원호가 끝난 뒤 진열대 A 동쪽 끝까지 곧은 줄이 한 lookahead 넘게 남는다
    assert arc[-1][0] - SHELVES[0][2] >= 0.10
    # 구역: work_out은 WORK_DOCK, HALL, J_E만, 급행은 HALL을 지나지 않는다
    assert out.zones == ("WORK_DOCK", "HALL", "J_E")
    assert express.zones[0] == "J_E" and "HALL" not in express.zones
    # 몸 여유: 동쪽 벽, 실측 북쪽 벽, 진열대 A
    points = list(out.points) + list(express.points[1:])
    east, north, shelf = _body_gaps(points, 1.40)
    assert east >= 0.045 and north >= 0.035 and shelf >= 0.005, (east, north, shelf)


def test_v7_pass_node_shortcut_cannot_release_a_dock_early(site):
    """edge를 지난 뒤에는 놓기 거리를 노드 좌표로 재므로, 도크 구역을 나오는 edge 끝의 지나가는 점은 그만큼 멀어야 한다."""
    from fleet_core.traffic.zone.progress import release_gap
    from fleet_core.traffic.zone.routes import ON_NODE_M
    p = load_params()
    gap = release_gap(p)
    checked = []
    for edge in site.edges():
        node = site.nodes[edge.b]
        start = site.nodes[edge.a]
        if node.kind != PASS or site.zones[start.zone].kind != "dock" or start.zone == node.zone:
            continue
        here = (node.x, node.y)
        # 노드에서 edge를 따라 ON_NODE_M 되돌아간 점
        back, left = here, ON_NODE_M
        for a, b in reversed(list(pairwise(edge.points))):
            span = math.dist(a, b)
            if span >= left:
                t = left / span
                back = (b[0] + (a[0] - b[0]) * t, b[1] + (a[1] - b[1]) * t)
                break
            left -= span
        dock = start.zone
        assert site.distance_to_zone(*back, dock) >= gap - 1e-9, (edge.a, edge.b, back)
        assert site.distance_to_zone(*here, dock) - p.release_margin_m >= gap - 1e-9, (
            edge.a, edge.b, round(site.distance_to_zone(*here, dock), 4))
        checked.append(edge.b)
    assert "W_X" in checked
