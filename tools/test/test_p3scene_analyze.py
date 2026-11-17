"""P-3 장면 분석(tools/p3scene/analyze.py): 몸 간격, 길 갈래, 서 있던 시간."""
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS / "p3scene"))

import analyze  # noqa: E402

SQ = [(-0.05, -0.05), (0.05, -0.05), (0.05, 0.05), (-0.05, 0.05)]


def test_gap_is_the_distance_between_bodies_and_negative_when_they_overlap():
    assert analyze.gap(SQ, analyze.place(SQ, (0.13, 0.0, 0.0))) == pytest.approx(0.03)
    assert analyze.gap(SQ, analyze.place(SQ, (0.08, 0.02, 0.0))) == pytest.approx(-0.02)
    assert analyze.gap(SQ, analyze.place(SQ, (0.08, 0.0, 0.0))) <= 0.0  # 모서리끼리 겹침


def test_the_footprint_and_world_boxes_come_from_the_repo():
    fp = analyze.footprint(TOOLS.parent)
    assert len(fp) == 14 and (0.040, 0.021) in fp
    world = analyze.boxes(TOOLS.parent)
    xs = [x for x, _ in world["shelf_2"]]
    ys = [y for _, y in world["shelf_2"]]
    assert min(xs) == pytest.approx(0.68) and max(xs) == pytest.approx(0.99)
    assert min(ys) == pytest.approx(0.365) and max(ys) == pytest.approx(0.405)
    assert "wall_south" in world


def test_route_reads_the_y_where_pode2_passes_pode3():
    low = [(0, 0.4, 0.24, 0), (1, 0.9, 0.28, 0)]
    high = [(0, 0.4, 0.24, 0), (1, 0.6, 0.5, 0), (2, 0.9, 0.55, 0)]
    short = [(0, 0.4, 0.24, 0), (1, 0.7, 0.28, 0)]
    assert analyze.route(low)[0] == "gap"
    assert analyze.route(high)[0] == "north"
    assert analyze.route(short)[0] == "none"


def test_only_stops_longer_than_a_second_count_and_turning_is_not_standing():
    track = [(t * 0.1, 0.0, 0.0, 0.0) for t in range(30)]  # 2.9 s 서 있음
    track += [(3.0 + t * 0.1, 0.02 * t, 0.0, 0.0) for t in range(1, 10)]  # 0.2 m/s
    track += [(4.0 + t * 0.1, 0.18, 0.0, 0.2 * t) for t in range(1, 20)]  # 제자리 회전
    [(start, length)] = analyze.stops(track)
    assert start == 0.0 and length == pytest.approx(2.9)



def test_physical_gap_counts_the_wheel_spheres_beyond_the_polygon():
    """시뮬 모델의 바퀴 충돌 구(r 0.028, y ±0.04855)는 발자국(반폭 0.056) 밖 0.0766까지 나온다."""
    fp = analyze.footprint(TOOLS.parent)
    wall = [(-1.0, 0.10), (1.0, 0.10), (1.0, 0.20), (-1.0, 0.20)]  # 아랫면 y 0.10
    pose = (0.0, 0.0, 0.0)
    assert analyze.gap(analyze.place(fp, pose), wall) == pytest.approx(0.10 - 0.056)
    assert analyze.phys_gap_world(fp, pose, wall) == pytest.approx(0.10 - 0.07655)
    # 마주 본 두 로봇: 옆으로 나란히 서면 바퀴끼리 먼저 닿는다
    side = (0.0, 0.14, 0.0)
    assert analyze.phys_gap_robots(fp, pose, side) == pytest.approx(0.14 - 2 * 0.07655)


def test_the_screen_does_not_meet_the_other_robots_wheels():
    """screen_mount(z 0.079~0.122)와 바퀴(z 0~0.056)는 높이가 겹치지 않는다."""
    fp = analyze.footprint(TOOLS.parent)
    # 앞쪽 끝이 0.0421(화면)인데 발자국은 0.040: 벽 쪽으로는 화면이 먼저 닿는다
    wall = [(0.10, -1.0), (0.20, -1.0), (0.20, 1.0), (0.10, 1.0)]
    assert analyze.phys_gap_world(fp, (0.0, 0.0, 0.0), wall) == pytest.approx(0.10 - 0.0421)
