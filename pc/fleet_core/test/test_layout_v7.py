"""배치 v7(사용자 확정 경기장, 2026-11-13): 치수, 도크, 대기 L, 구역 여유, 길."""
import math

import pytest
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


def test_v7_work_queue_lies_on_the_wall_l_off_the_way_into_work(site):
    station = site.stations["WORK"]
    queue = station.queue
    # 사용자 결정(2026-11-13): 대기 자리 둘, 줄 서기(앞자리 먼저, 빈 앞자리로 당기기). 출고는 그대로
    assert queue == ("Q1", "Q2") and station.line and not site.stations["OUTBOUND"].line
    assert site.node_pose("Q1") == pytest.approx((1.62, 0.10))
    assert site.node_pose("Q2") == pytest.approx((1.40, 0.10))
    for spot in queue:
        x, y = site.node_pose(spot)
        # 남쪽 벽 선 위. 예전에는 x >= 1.56(진열대 동쪽)이었으나, 둘째 자리 Q2는 동쪽 진열대 조각
        # 아래다: 그 아래로는 지나가는 교통을 두지 않는다(test_v7_queue_is_entered_at_the_tail_only)
        assert y == pytest.approx(0.10) and x >= 1.25, spot
    # WORK에는 진입점 F에서만 곧게 들어간다(고정 경로)
    into_w = [(link.a, link.via) for link in site.links() if link.b == "W"]
    assert into_w == [("F", FOLLOW_PATH)] and "F" not in queue
    # 대기 자리는 W로 가는 어느 길에도 없다: 대기 자리가 유일한 길 위에 있으면 도크 차례 규칙이 세
    # 대 사이에서 돌았다(첫 v7, 머리 없는 판 주문 8건 0/8, 몰림 0/10). D2의 W_Q와 같은 자리 잡기
    for start in site.nodes:
        if start in queue or start == "W":
            continue
        way = route(site, start, "W")
        if way is not None:
            assert not {link.b for link in way[:-1]} & set(queue), (start, [l.b for l in way])


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
    assert [link.b for link in way] == ["W_X", "O"]
    assert all(link.via == FOLLOW_PATH for link in way)
    assert "EXPRESS" in way[-1].zones
    # WORK에서 나가기(사용자 2026-11-13): W에서 앞으로(북쪽) 곧게 나와 바로 위 갈림 W_X에서 급행을
    # 타거나 넓은 곳(서쪽)으로 간다. 예전의 X_E -> N_E(급행 입구에서 다시 내려오기)는 없다.
    # W_X는 HALL 띠(0.63~0.765)가 아니라 그 위 J_E 안의 지나가는 점이다(063086b 리뷰): HALL 띠에서는
    # WORK_DOCK과 J_E 어느 쪽에서도 0.135 떨어질 수 없어 선 로봇이 WORK_DOCK을 쥐었고, 갈림이 PASS가
    # 아니면 급행이 막혔을 때 넓은 곳으로 물러나지(yard_retreat) 못한다
    exit_ = site.nodes["W_X"]
    assert exit_.kind == PASS and exit_.zone == "J_E" and exit_.x == pytest.approx(1.88)
    assert site.distance_to_zone(exit_.x, exit_.y, "WORK_DOCK") >= RELEASE_GAP_M
    assert [(l.b, l.via) for l in site.links() if l.a == "W"] == [("W_X", FOLLOW_PATH)]
    assert sorted((l.b, l.via) for l in site.links() if l.a == "W_X") == [
        ("N_E", FOLLOW_PATH), ("O", FOLLOW_PATH)]
    assert [l.a for l in site.links() if l.b == "W_X"] == ["W"]
    assert "X_E" not in site.nodes
    from fleet_core.traffic.zone.yard_retreat import yard_retreat
    assert yard_retreat(site, site.link("W_X", "O")) == ("N_E", "W_X")
    # 급행을 못 타면 갈림에서 넓은 곳을 지나 출고로 갈 길이 있다(N_O -> O)
    def no_express(link):
        return math.inf if "EXPRESS" in link.zones else link.length
    way = route(site, "W_X", "O", no_express)
    assert way is not None and way[0].b == "N_E" and way[-1].a == "N_O", [l.b for l in way or []]
    # 출고(사용자 2026-11-13): 급행이나 N_O(out_in)로 동쪽에서 들어와, 나갈 때는 후진하지 않고
    # 앞으로(서쪽) 조금 가서 남쪽으로 꺾어 넓은 곳의 N_O로 나온다(통과형)
    out = site.link("O", "N_O")
    assert out.via == FOLLOW_PATH
    o = site.node_pose("O")
    assert out.points[1][0] < o[0] - 0.05, "first goes west, the way O faces"
    assert all(b[1] <= a[1] + 1e-9 for a, b in zip(out.points[1:], out.points[2:])), "then south"
    assert min(x for x, _ in out.points) - BODY_CIRCLE_M >= 0.05, "clear of the west wall"
    assert site.link("N_O", "O").via == FOLLOW_PATH  # 넓은 곳에서 들어오기는 그대로
    # 마당 지점끼리는 Nav2 goal로 오가고 HALL만 잡는다(사용자 결정: 잠그는 구역은 급행 띠와 도크만)
    yard = ["N_O", "N_E", "G", "F", "Q1", "Q2", "O_Q", "E-1", "E-2", "E-3"]
    for a in yard:
        for b in yard:
            link = site.link(a, b)
            if link is not None and (a, b) != ("Q1", "F"):
                assert link.via == NAV_GOAL and link.zones == ("HALL",), (a, b, link.zones)
    # 가운데 통로 끝에서 WORK 진입점과 줄 꼬리로 간다(그림의 빨간 선)
    for a, b in [("N_O", "N_E"), ("N_E", "G"), ("G", "F"), ("G", "Q2"), ("Q2", "Q1")]:
        assert site.link(a, b) is not None and site.link(b, a) is not None, (a, b)
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


def test_v7_queue_is_entered_at_the_tail_only(site):
    """줄은 꼬리 Q2로만 들어오고(G에서) 앞으로만 흘러 W로 간다: Q2 -> Q1 -> F(고정 경로, 한 방향) -> W.
    Q1, Q2에 닿는 이음은 이것뿐이다(Q2 <-> Q1은 배터리가 낮아 줄을 떠나는 로봇이 G로 되돌아 나가는
    길이기도 하다)."""
    touching = sorted((link.a, link.b, link.via) for link in site.links()
                      if {"Q1", "Q2"} & {link.a, link.b})
    assert touching == [("G", "Q2", NAV_GOAL), ("Q1", "F", FOLLOW_PATH), ("Q1", "Q2", NAV_GOAL),
                        ("Q2", "G", NAV_GOAL), ("Q2", "Q1", NAV_GOAL)]
    # 동쪽 진열대 조각 아래(대기 로봇 위 0.18)로 남쪽 마당을 가로지르는 이음이 없다
    from fleet_core.fleet_map.geometry import point_segment_distance
    for link in site.links():
        if {"Q1", "Q2"} & {link.a, link.b}:
            continue
        a, b = link.points[0], link.points[-1]
        assert point_segment_distance((1.40, 0.20), a, b) > 0.15, (link.a, link.b)


BODY_CIRCLE_M = 0.083  # 핑키 뒤 모서리 회전 반지름(docs/실측_2026-11-10.md)
HALF_WIDTH_M = 0.056   # 지나가는 로봇의 반 폭(local footprint)


def test_v7_robots_standing_in_line_or_at_a_fork_leave_every_other_way_open(site):
    """오래 서는 자리(도크, 대기, 갈림)의 로봇 몸 원과, 그 자리를 거치지 않는 모든 길(고정 경로의
    선, Nav2 goal의 곧은 선) 사이에 지나가는 로봇의 반 폭이 들어간다."""
    from itertools import pairwise

    from fleet_core.fleet_map.geometry import point_segment_distance
    # G, F: 갈림에서도 차례를 기다리며 선다(3e84ff6 리뷰: G에 선 로봇이 E-3 -> F 선을 0.076으로 막았다)
    standing = ["W", "O", "Q1", "Q2", "O_Q", "N_E", "N_O", "G", "F"]
    for name in standing:
        here = site.node_pose(name)
        for link in site.links():
            if name in (link.a, link.b):
                continue
            gap = min(point_segment_distance(here, a, b) for a, b in pairwise(link.points))
            assert gap >= BODY_CIRCLE_M + HALF_WIDTH_M, (name, link.a, link.b, round(gap, 3))


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
