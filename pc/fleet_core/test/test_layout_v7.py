"""배치 v7(사용자 확정 경기장, 2026-11-13): 치수, 도크, 대기 L, 구역 여유, 길."""
import math

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import KNOWN_LAYOUTS, LAYOUTS, PASS, SHARED, load_fleet_map, route
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
        # 사용자 2026-11-13 밤: 거의 벽에 닿을 만큼 깊숙이. 북쪽을 본 로봇의 뒤 끝이 벽에서 0.0055다
        assert site.nodes[name].y == pytest.approx(0.08)
        assert 0.0 < site.nodes[name].y - REAR_M < 0.01
    # 칸 입구(사용자 2026-11-14 "주차 줄이 너무 길다, 점 사이를 줄여"): y 0.27 -> 0.24,
    # 줄 0.19 -> 0.16. 더 내리면 E-1 -> Q2의 곧은 선이 칸막이 3 위 끝에 몸 원 + 여유(0.103)보다
    # 가까워진다(0.235가 끝, 아래 칸 시험). 입구에서 돌기만 보면 0.203이 끝이다
    for name, x in (("E-1", 0.12), ("E-2", 0.38), ("E-3", 0.64)):
        assert site.node_pose(name) == pytest.approx((x, 0.24))
        spot = site.node_pose("P-" + name.removeprefix("E-"))
        assert math.dist(site.node_pose(name), spot) == pytest.approx(0.16)


def test_v7_work_queue_is_two_spots_entered_at_q2(site):
    """사용자 결정(2026-11-13 밤): WORK 줄은 둘이다. 앞자리 Q1은 W 바로 남쪽, 들어오는 자리 Q2는
    남동 구석이다(사용자 2026-11-13 밤: 칸 입구 높이로 올렸던 것을 다시 내린다). 마당에서는 Q2로만
    들어온다. 꼬리 Q3과 갈림 G는 없다. 줄 서기를 켠다."""
    station = site.stations["WORK"]
    assert station.queue == ("Q1", "Q2") and station.line
    assert not site.stations["OUTBOUND"].line
    assert not {"Q3", "G", "F"} & set(site.nodes)
    q1, q2 = site.node_pose("Q1"), site.node_pose("Q2")
    assert q1[0] > 1.80 and q2[0] < q1[0] and q2[1] < 0.20, (q1, q2)  # 구석
    # Nav2 goal로 닿는 자리(Q2)와 줄 안에서 도는 점은 벽에서 0.13 이상(Gazebo 기준: 덧댐 0.01이
    # 붙은 뒤 모서리 0.098 + 칸 0.025 + 여유). 줄 안 당기기는 고정 경로다
    turns = [q1, q2] + list(site.link("Q2", "Q1").points[1:-1])
    for x, y in turns:
        assert min(1.98 - x, y) >= WALL_NAV_M - 1e-9, (x, y)
    # 앞자리에는 북쪽으로 곧게 닿는다(앞자리에서 돌지 않는다)
    bend = site.link("Q2", "Q1").points[-2]
    assert bend[0] == pytest.approx(q1[0]) and bend[1] < q1[1]
    # 두 자리 사이는 노드 놓는 거리(0.20)보다 멀다: 한 칸 당긴 로봇이 서면 뒷자리를 놓는다
    assert math.dist(q1, q2) > load_params().node_clear_m
    # Q2에 동쪽을 보고 선 로봇이 북쪽을 보고 선 앞자리 로봇 뒤 모서리와 겹치지 않는다
    front_rear_y = q1[1] - REAR_M
    assert q2[0] + FRONT_M < q1[0] - HALF_WIDTH_M or q2[1] + HALF_WIDTH_M < front_rear_y
    # 흐름: Q2 -> 앞자리 -> W, 모두 한 방향 고정 경로
    for a, b in (("Q2", "Q1"), ("Q1", "W")):
        assert site.link(a, b).via == FOLLOW_PATH, (a, b)
        assert site.link(b, a) is None, (b, a)
    into_w = [(link.a, link.via) for link in site.links() if link.b == "W"]
    assert into_w == [("Q1", FOLLOW_PATH)]
    # 줄 밖에서 W로 가는 길은 모두 Q2로 들어와 Q1을 거친다. 줄 서기에서는 줄에 선 로봇이 줄 밖
    # 로봇보다 앞 차례라, 줄을 지나야 하는 로봇이 앞 차례가 되지 않는다
    for start in site.nodes:
        if start in station.queue or start == "W":
            continue
        way = route(site, start, "W")
        assert way is not None and [link.b for link in way][-3:] == ["Q2", "Q1", "W"], start


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
    # 출고의 두 점(사용자 2026-11-13 밤: 도크 입구 바로 아래에 두자는 요청)도 예외가 아니다.
    # 더 올리면 두 규칙에 걸린다: 선 로봇의 몸(0.06)이 Nav2 keepout 띠(OUT_DOCK에서 0.06) 밖이어야
    # 해서 y 0.645가 끝이고(fleet_gz_sim test_arena), 0.135보다 가까우면 거기 선 로봇이 OUT_DOCK을
    # 쥔 채 있어 다음 로봇이 출고에 들어가지 못한다. 두 규칙이 허락하는 끝은 y 0.63이고 0.01의
    # 여유를 둔다
    for name in ("O_IN", "O_OUT"):
        node = site.nodes[name]
        gap = site.distance_to_zone(node.x, node.y, "OUT_DOCK")
        assert RELEASE_GAP_M + 0.005 <= gap <= RELEASE_GAP_M + 0.02, (name, gap)
    # 충전 칸과 그 입구는 짝이다: 칸에 선 로봇이 자기 입구를 쥐고 있어도 그 칸 말고는 쓰지 않는다
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
    """칸에 선 로봇은 칸 입구 E-k를 쥔 채로 있다(P와 E 사이 0.14 < node_clear 0.20). 그래도 E-k와
    그 칸을 뺀 나머지 지점끼리는 E-k를 거치지 않고 오갈 수 있어야 한다."""
    door = _bay_door(spot)
    others = [name for name in site.nodes if name not in (door, spot)]

    def cost(link):
        return math.inf if door in (link.a, link.b) else link.length
    for a in others:
        for b in others:
            if a != b:
                assert route(site, a, b, cost) is not None, (spot, a, b)


DIVIDER_TOP_M = 0.10
TURN_MARGIN_M = 0.02


def test_v7_bays_are_entered_backwards_and_left_forwards_on_a_straight_line(site):
    """칸 안에서는 돌지 않는다: 입구에서 북쪽을 본 채 곧게 후진해 들어가고 앞으로 곧게 나온다
    (Gazebo 1, 2판: Nav2 goal로 칸에서 나가다 셋째 칸막이 위 끝을 잘라 RPP가 멈췄다)."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import point_rect_distance, point_segment_distance
    for spot in site.parking:
        door = _bay_door(spot)
        p, e = site.nodes[spot], site.nodes[door]
        assert e.x == pytest.approx(p.x) and math.degrees(e.yaw) == pytest.approx(90.0)
        assert math.degrees(p.yaw) == pytest.approx(90.0)
        # 입구에서 제자리 회전하는 몸 원이 칸막이 끝과 진열대에 닿지 않는다
        assert e.y >= DIVIDER_TOP_M + BODY_CIRCLE_M + TURN_MARGIN_M
        for shelf in site.shelves:
            assert point_rect_distance(shelf, e.x, e.y) >= BODY_CIRCLE_M + TURN_MARGIN_M, (door,
                                                                                         shelf)
        into = [(link.a, link.via) for link in site.links() if link.b == spot]
        out = [(link.b, link.via) for link in site.links() if link.a == spot]
        assert into == [(door, RETREAT)] and out == [(door, FOLLOW_PATH)]
        for link in (site.link(door, spot), site.link(spot, door)):
            assert len(link.points) == 2  # 곧은 선
            for shelf in DIVIDERS:
                gap = min(point_segment_distance(((shelf[0] + shelf[2]) / 2, shelf[3]), a, b)
                          for a, b in pairwise(link.points))
                assert gap >= HALF_WIDTH_M + TURN_MARGIN_M, (spot, shelf, gap)
    # 마당 Nav2 goal은 칸 안을 지나지 않는다: 곧은 선이 칸막이에서 몸 원 + 여유만큼 떨어진다
    for link in site.links():
        if link.via != NAV_GOAL:
            continue
        assert not {link.a, link.b} & set(site.parking), (link.a, link.b)
        for shelf in DIVIDERS:
            steps = max(2, int(link.length / 0.01))
            (ax, ay), (bx, by) = link.points[0], link.points[-1]
            gap = min(point_rect_distance(shelf, ax + (bx - ax) * i / steps,
                                          ay + (by - ay) * i / steps) for i in range(steps + 1))
            assert gap >= BODY_CIRCLE_M + TURN_MARGIN_M, (link.a, link.b, shelf, round(gap, 3))
    # 진열대 모서리도 곧은 선에서 몸 원보다 멀다(bbb5175 리뷰: G-E-3은 남쪽 진열대 동쪽 끝에서
    # 0.043이라 머리 없는 판에서 로봇이 그 앞에 막혀 섰다)
    for link in site.links():
        if link.via != NAV_GOAL:
            continue
        from fleet_core.fleet_map.geometry import segment_rect_distance
        gap = min(segment_rect_distance(link.points[0], link.points[-1], shelf)
                  for shelf in site.shelves)
        assert gap >= BODY_CIRCLE_M, (link.a, link.b, round(gap, 3))


def test_v7_dock_releases_neighbours_after_docking(site):
    # 도킹을 마친 로봇은 도크에서 0.085 + 오차(시뮬 0.01) 넘게 떨어진 구역을 놓는다
    for dock in ("W", "O"):
        x, y = site.node_pose(dock)
        for zone in site.zones:
            if zone != site.nodes[dock].zone:
                assert site.distance_to_zone(x, y, zone) >= 0.10 - 1e-9, (dock, zone)


def test_v7_routes(site):
    # 급행: WORK에서 출고로 고정 경로. 급행이 비어 있으면 이쪽을 고른다
    way = route(site, "W", "O")
    assert [link.b for link in way] == ["W_X", "O"]
    assert all(link.via == FOLLOW_PATH for link in way)
    assert "EXPRESS" in way[-1].zones
    # WORK에서 나가는 길은 둘이다. 급행 들머리 W_X로 앞으로 곧게(고정 경로), 그리고 넓은 곳의 M_E로
    # Nav2 goal로 풀려 나가기(사용자 결정 2026-11-14 "W에서 HALL로 나가는 건 경로가 아니라 goal
    # 방식으로 풀면 된다": 급행이 막혀도 돌아갈 길이 있다). 들어오는 길은 Q1에서 고정 경로 하나다.
    # 넓은 곳으로 바로 나가던 옛 고정 경로와 그 끝점 N_E는 없다
    exit_ = site.nodes["W_X"]
    assert exit_.kind == PASS and exit_.zone == "J_E" and exit_.x == pytest.approx(1.88)
    assert site.distance_to_zone(exit_.x, exit_.y, "WORK_DOCK") >= RELEASE_GAP_M
    assert sorted((link.b, link.via) for link in site.links() if link.a == "W") == [
        ("M_E", NAV_GOAL), ("W_X", FOLLOW_PATH)]
    assert site.link("W", "M_E").zones == ("WORK_DOCK", "HALL")
    assert [link.a for link in site.links() if link.b == "W"] == ["Q1"]
    # W_X는 W와 급행만 잇는다(W로 물러나는 후진도 없다: W 띠 안에서 제자리 돌기가 된다)
    assert [(l.b, l.via) for l in site.links() if l.a == "W_X"] == [("O", FOLLOW_PATH)]
    assert [l.a for l in site.links() if l.b == "W_X"] == ["W"]
    assert not {"X_E", "N_E", "N_O", "O_Q"} & set(site.nodes)

    def no_express(link):
        return math.inf if "EXPRESS" in link.zones else link.price_m
    hall = route(site, "W", "O", no_express)
    assert [link.b for link in hall] == ["M_E", "O_IN", "O"]
    # 넓은 곳 쪽이 0.09 m 짧다. 그 이음에 extra_m(W에서 M_E 쪽으로 약 100도 제자리 돌기)을 더해
    # 급행이 비면 급행을 고르고, 급행 들머리 J_E를 다른 로봇이 쥐고 있으면(TRANSIT_WAIT_S)
    # 넓은 곳으로 간다
    from fleet_core.traffic.zone.goals import TRANSIT_WAIT_S
    express_m, hall_m = (sum(link.price_m for link in w) for w in (way, hall))
    assert sum(link.length for link in hall) < sum(link.length for link in way)
    assert express_m < hall_m < express_m + TRANSIT_WAIT_S * load_params().nav_speed_mps
    # 출고(사용자 그림 2026-11-13 밤): 들어오는 점과 나가는 점을 나눈다. 급행으로 오지 않는 로봇은
    # 동쪽의 O_IN에서 들어오고(급행 띠를 쥐지 않는다), 나갈 때는 후진하지 않고 앞으로(서쪽) 조금
    # 가서 남쪽으로 꺾어 서쪽의 O_OUT으로 나온다. O_OUT부터는 Nav2 goal로 마당을 자유롭게 간다
    assert sorted(link.a for link in site.links() if link.b == "O") == ["O_IN", "W_X"]
    assert [link.b for link in site.links() if link.a == "O"] == ["O_OUT"]
    out, into = site.link("O", "O_OUT"), site.link("O_IN", "O")
    assert out.via == FOLLOW_PATH and into.via == FOLLOW_PATH
    assert "EXPRESS" not in into.zones
    o, o_in, o_out = (site.node_pose(n) for n in ("O", "O_IN", "O_OUT"))
    assert o_out[0] < o[0] < o_in[0]
    assert out.points[1][0] < o[0] - 0.05, "first goes west, the way O faces"
    assert all(b[1] <= a[1] + 1e-9 for a, b in zip(out.points[1:], out.points[2:])), "then south"
    assert min(x for x, _ in out.points) - BODY_CIRCLE_M >= 0.05, "clear of the west wall"
    # 나가는 점으로 들어오는 고정 경로도, 들어오는 점에서 나가는 고정 경로도 없다
    fixed = [link for link in site.links() if link.via != NAV_GOAL]
    assert [link.a for link in fixed if link.b == "O_OUT"] == ["O"]
    assert [link.b for link in fixed if link.a == "O_IN"] == ["O"]
    # 출고가 바쁘면 들어오는 점에서 기다린다
    assert site.stations["OUTBOUND"].queue == ("O_IN",)
    # 마당 지점끼리는 Nav2 goal로 오가고 HALL만 잡는다(사용자 결정: 잠그는 구역은 급행 띠와 도크만)
    yard = ["O_IN", "O_OUT", "M_E", "Q2", "E-1", "E-2", "E-3"]
    for a in yard:
        for b in yard:
            link = site.link(a, b)
            if link is not None:
                assert link.via == NAV_GOAL and link.zones == ("HALL",), (a, b, link.zones)
    # 칸 입구 셋은 서로, 그리고 줄 입구 Q2, 출고의 두 점과 곧게 이어진다: 칸에 선 로봇이 자기 입구를
    # 쥐고 있어도 나머지 입구로 다닌다. 남쪽 통로로 바로 가므로 가운데 통로를 돌지 않는다
    doors = ["E-1", "E-2", "E-3"]
    pairs = [(a, b) for a in doors for b in doors if a != b]
    pairs += [(door, far) for door in doors for far in ("Q2", "O_OUT")]
    # O_IN은 E-1, E-2와만 잇는다: E-3까지의 곧은 선은 남쪽 진열대 서쪽 끝 아래 모서리에서 0.082다.
    # E-3으로는 다른 입구나 O_OUT을 거쳐 간다
    pairs += [("O_IN", "E-1"), ("O_IN", "E-2")]
    assert site.link("O_IN", "E-3") is None and site.link("E-3", "O_IN") is None
    # 우회로(2026-11-13 밤): 가운데 통로 동쪽 끝의 M_E. O_IN과 Q2만 잇는다
    pairs += [("O_OUT", "O_IN"), ("O_IN", "M_E"), ("M_E", "Q2")]
    for a, b in pairs:
        assert site.link(a, b) is not None and site.link(b, a) is not None, (a, b)
    assert sorted(link.b for link in site.links() if link.a == "M_E") == ["O_IN", "Q2"]
    # 평소에는 남쪽 통로가 짧다. 남쪽 통로(칸 입구 <-> Q2)를 못 쓰면 가운데 통로로 돈다
    for door in doors:
        assert [link.b for link in route(site, door, "Q2")] == ["Q2"], door

    def south_shut(link):
        return math.inf if "Q2" in (link.a, link.b) and "M_E" not in (link.a, link.b) \
            else link.length
    for start in (*doors, "O_OUT"):
        way = [link.b for link in route(site, start, "Q2", south_shut)]
        assert way[-3:] == ["O_IN", "M_E", "Q2"], (start, way)
    # 진열대를 곧게 가로지르는 Nav2 이음이 없다(4079c40 리뷰: 그래프 값이 실제보다 짧다)
    from fleet_core.fleet_map.geometry import inside
    for link in site.links():
        if link.via != NAV_GOAL:
            continue
        (ax, ay), (bx, by) = link.points[0], link.points[-1]
        for k in range(101):
            x, y = ax + (bx - ax) * k / 100, ay + (by - ay) * k / 100
            assert not any(inside(shelf, x, y) for shelf in site.shelves), (link.a, link.b)
    for spot in site.parking:
        assert route(site, spot, "W") is not None and route(site, "O", spot) is not None


def test_v7_queue_is_entered_at_q2_and_flows_one_way(site):
    """줄에 닿는 이음: 칸 입구 셋 <-> Q2(들어오고, 줄을 떠날 때 나간다), 줄 안은 한 방향.
    앞자리 Q1은 줄 안에서만 닿고 W로만 나간다."""
    touching = sorted((link.a, link.b, link.via) for link in site.links()
                      if {"Q1", "Q2"} & {link.a, link.b})
    doors = ["E-1", "E-2", "E-3"]
    doors.append("M_E")  # 우회로의 끝도 줄 입구로 들어온다
    assert touching == sorted([(door, "Q2", NAV_GOAL) for door in doors]
                              + [("Q2", door, NAV_GOAL) for door in doors]
                              + [("Q1", "W", FOLLOW_PATH), ("Q2", "Q1", FOLLOW_PATH)])
    # Q2는 도착 방향(동쪽)을 본다: 북쪽으로 돌면 몸 원이 앞자리 로봇에 닿는다
    assert math.degrees(site.nodes["Q2"].yaw) == pytest.approx(0.0)
    assert math.degrees(site.nodes["Q1"].yaw) == pytest.approx(90.0)


BODY_CIRCLE_M = 0.083  # 핑키 뒤 모서리 회전 반지름(docs/실측_2026-11-10.md)
WALL_NAV_M = 0.13      # Nav2 goal로 닿는 자리와 벽 사이(덧댐 붙은 뒤 모서리 0.098 + 칸 0.025)
FRONT_M, REAR_M = 0.04, 0.0745
HALF_WIDTH_M = 0.056   # 지나가는 로봇의 반 폭(local footprint)


def test_v7_robots_standing_in_line_or_at_a_fork_leave_every_other_way_open(site):
    """오래 서는 자리(도크, 대기, 갈림)의 로봇 몸 원과, 그 자리를 거치지 않는 모든 길(고정 경로의
    선, Nav2 goal의 곧은 선) 사이에 지나가는 로봇의 반 폭이 들어간다."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import point_segment_distance
    # G, F: 갈림에서도 차례를 기다리며 선다(3e84ff6 리뷰: G에 선 로봇이 E-3 -> F 선을 0.076으로 막았다)
    # 지나가는 로봇은 주차한 로봇 앞을 몸 둘이 닿지 않게 지난다(2026-11-13 밤, Gazebo와 머리 없는
    # 세계: 칸 입구 0.25, 주차 0.11이던 때 E-1 -> Q2의 선이 P-3에서 0.11이라 로봇이 서 있었다).
    # 그때 기준은 0.175(머리 없는 세계가 막혔다고 보는 거리 0.17보다 조금 넘게)였다. 칸 입구를
    # 0.24로 내린 뒤(사용자 2026-11-14 "주차 줄을 줄여") 입구 줄과 Q2로 가는 선은 주차한 로봇에서
    # 0.133~0.16을 지난다. 곧은 선으로도 몸끼리는 닿지 않는다(주차한 로봇 앞 끝 0.04 + 지나가는 로봇
    # 반 폭 0.056 + 여유 0.03). Nav2와 머리 없는 planner는 곧은 선이 아니라 그 로봇을 돌아가는 길을
    # 내고, 그래도 막히면 관제가 가운데 통로(M_E)로 돌린다
    low = {}
    for link in site.links():
        if link.via != NAV_GOAL:
            continue
        for spot in site.parking:
            gap = point_segment_distance(site.node_pose(spot), link.points[0], link.points[-1])
            if gap < 0.175:
                low[frozenset((link.a, link.b)), spot] = round(gap, 3)
    assert min(low.values()) >= FRONT_M + HALF_WIDTH_M + 0.03, low
    assert min(low.values()) == pytest.approx(0.133, abs=1e-3)  # E-1 -> Q2가 P-3 앞을 지난다
    standing = ["W", "O", "Q1", "Q2", "O_IN", "O_OUT", "M_E"]
    for name in standing:
        here = site.node_pose(name)
        for link in site.links():
            if name in (link.a, link.b):
                continue
            gap = min(point_segment_distance(here, a, b) for a, b in pairwise(link.points))
            assert gap >= BODY_CIRCLE_M + HALF_WIDTH_M, (name, link.a, link.b, round(gap, 3))
            # 고정 경로는 비켜 가지 못한다: 지나가는 로봇도 몸 원으로 본다(머리 없는 판: 출고 출구가
            # O_Q에 선 로봇에서 0.152라 출고 로봇이 나가지 못하고 O_Q 로봇은 출고를 기다려 교착)
            if link.via != NAV_GOAL:
                assert gap >= 2 * BODY_CIRCLE_M + 0.01, (name, link.a, link.b, round(gap, 3))


def test_v7_fixed_path_bends_leave_the_body_clear_of_walls_and_shelves(site):
    """고정 경로가 꺾이는 점(출고 출구의 약 79도, 64도 등)에서 몸 원(뒤 모서리 회전 반지름 0.083)이
    벽과 진열대·칸막이에 닿지 않는다(6f4e19e 리뷰). 꺾는 점 사이 곧은 선도 같은 몸 원으로 본다."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import segment_rect_distance, wall_gap
    for link in site.links():
        if link.via == NAV_GOAL:
            continue
        for x, y in link.points[1:-1]:
            assert wall_gap(site.bounds(), x, y) > BODY_CIRCLE_M, (link.a, link.b, (x, y))
        for a, b in pairwise(link.points):
            for shelf in site.shelves:
                gap = segment_rect_distance(a, b, shelf)
                assert gap > BODY_CIRCLE_M, (link.a, link.b, shelf, round(gap, 3))


def test_v7_locks_only_the_express_strip_and_the_docks(site):
    kinds = {name: zone.kind for name, zone in site.zones.items()}
    assert kinds == {"HALL": SHARED, "WORK_DOCK": "dock", "J_E": "dock", "EXPRESS": "lane",
                     "OUT_DOCK": "dock"}
    assert site.zones["HALL"].capacity == 3 and site.zones["EXPRESS"].capacity == 2


def test_v7_global_costmap_keeps_a_way_between_the_south_shelf_end_and_work(site):
    """빨간 선(가운데 통로 끝 -> F, Q1)이 남쪽 진열대 동쪽 끝과 WORK_DOCK keepout 사이를 지난다.
    5 cm 전역 costmap에서 진열대 칸 중심과 0.09(내접) 넘게 떨어지고 keepout(몸 반폭 0.06 넓힘) 밖인
    칸이 진열대 끝 높이의 줄마다 한 칸 이상 있다(keepout 마스크는 부풀지 않는다). 한 칸뿐이라 좁다. 도크에 선 로봇(관제가 주는 peer 장애물)은
    따로 빼고 본 값이다."""
    from fleet_core.fleet_map.keepout import keepout_rects
    from fleet_core.fleet_map.geometry import inside
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
