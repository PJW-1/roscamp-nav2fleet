"""배치 v7의 나눈 goal(해결안 B): 중간 지점 GAP_S와 splits 사각형.

근거 숫자는 docs/plan_flip_2026-11-24/two_ways.py(v7 costmap 지도로 잰 NavFn 비용과 길이)에 있다.
"""
import math
from itertools import pairwise

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import NAV_GOAL, load_fleet_map, shortest_path
from fleet_core.fleet_map.geometry import point_rect_distance, segment_rect_distance

SOUTH_WEST, SOUTH_EAST = (0.70, 0.3475, 0.99, 0.3875), (1.25, 0.3475, 1.56, 0.3875)
BODY_HALF_M = 0.06       # 지도 여유 검사(clearance_m)와 keepout 덧댐
NODE_CLEAR_M = 0.20


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("v7")


def test_gap_s_stands_below_the_middle_of_the_south_shelf_gap(site):
    gap = site.nodes["GAP_S"]
    assert (gap.kind, gap.zone, gap.via, gap.through) == ("fork", "HALL", True, False)
    assert gap.x == pytest.approx((SOUTH_WEST[2] + SOUTH_EAST[0]) / 2)
    assert gap.y == pytest.approx(0.22)
    # 로봇은 GAP_S nav_pass_m 앞에서 Q2 goal로 넘어간다. 북쪽에서 오면 그 자리가 진열대 줄
    # (y 0.3475~0.3875) 안이라 이미 틈에 들어선 뒤다
    assert gap.y + load_params().nav_pass_m <= SOUTH_WEST[3]


def test_gap_s_faces_q2(site):
    gap, q2 = site.nodes["GAP_S"], site.nodes["Q2"]
    assert gap.yaw == pytest.approx(math.atan2(q2.y - gap.y, q2.x - gap.x), abs=0.01)


def test_gap_s_keeps_clear_of_walls_shelves_and_other_nodes(site):
    gap = site.node_pose("GAP_S")
    for shelf in site.shelves:
        assert point_rect_distance(shelf, *gap) >= BODY_HALF_M
    # 진열대 줄 아래 면(y 0.3475)에서 0.1275, 가장 가까운 진열대 모서리에서 0.182
    assert SOUTH_WEST[1] - gap[1] == pytest.approx(0.1275)
    assert min(point_rect_distance(s, *gap) for s in (SOUTH_WEST, SOUTH_EAST)) == pytest.approx(
        math.hypot(0.13, 0.1275), abs=1e-3)
    for name in site.nodes:
        if name != "GAP_S":
            assert math.dist(gap, site.node_pose(name)) >= NODE_CLEAR_M, name


def test_gap_s_joins_only_q2_and_that_line_clears_the_east_shelf(site):
    edges = [e for e in site.edges() if "GAP_S" in (e.a, e.b)]
    assert sorted((e.a, e.b, e.mode) for e in edges) == [
        ("GAP_S", "Q2", NAV_GOAL), ("Q2", "GAP_S", NAV_GOAL)]
    line = site.edge("GAP_S", "Q2").points
    gap = min(segment_rect_distance(a, b, SOUTH_EAST) for a, b in pairwise(line))
    assert gap >= BODY_HALF_M + 0.05


def test_gap_s_is_on_no_shortest_way_between_other_nodes(site):
    """GAP_S는 Q2에만 붙은 막다른 지점이다: 지도 길 고르기(plan_split을 끈 기준 판)가 바뀌지 않는다."""
    names = [n for n in site.nodes if n != "GAP_S"]
    for a in names:
        for b in names:
            if a != b:
                way = shortest_path(site, a, b)
                assert way is None or "GAP_S" not in [e.b for e in way], (a, b)


def test_v7_splits_send_q2_goals_by_gap_s_or_m_e_from_the_north_half_only(site):
    assert [(s.to, s.via) for s in site.splits] == [("Q2", "GAP_S"), ("Q2", "M_E")]
    assert not site.nodes["M_E"].via, "M_E stays a free edge target"
    for split in site.splits:
        for x0, y0, x1, y1 in split.rects:
            assert y0 >= SOUTH_WEST[3] and x1 <= 1.20 + 1e-9
    # 기록된 자리, 가운데 통로 서쪽 끝, 서쪽 마당, 동쪽(한 길이 확실히 싸다), 남쪽 통로
    assert site.split_via("Q2", 1.05, 0.55) == "GAP_S"
    assert site.split_via("Q2", 0.68, 0.60) == "M_E"
    assert site.split_via("Q2", 0.34, 0.68) == "GAP_S"
    assert site.split_via("Q2", 0.45, 0.45) == ""  # 서쪽 진열대 서쪽 끝 위: 서쪽 길이 확실히 싸다
    assert site.split_via("Q2", 1.40, 0.55) == ""
    assert site.split_via("Q2", 0.90, 0.25) == ""
    assert site.split_via("E-3", 1.05, 0.55) == ""
