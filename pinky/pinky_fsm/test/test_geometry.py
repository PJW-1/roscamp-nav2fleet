"""경로 기하: 펼치기, 투영, 남은 선. ROS 없이 돈다."""
import math
from itertools import pairwise

import pytest
from pinky_fsm.geometry import (
    Polyline,
    poses_along,
    poses_backing,
    quaternion_yaw,
    rounded,
    rounded_rest,
    yaw_quaternion,
)


def test_poses_along_spaces_points_at_most_one_step_apart():
    poses = poses_along([(0.0, 0.0), (0.4, 0.0), (0.4, 0.3)], final_yaw=1.0, step=0.05)
    gaps = [math.dist(a[:2], b[:2]) for a, b in pairwise(poses)]
    assert max(gaps) <= 0.05 + 1e-9
    assert poses[0][:2] == (0.0, 0.0) and poses[-1][:2] == pytest.approx((0.4, 0.3))


def test_poses_along_faces_each_segment_and_ends_on_the_final_yaw():
    poses = poses_along([(0.0, 0.0), (0.4, 0.0), (0.4, 0.3)], final_yaw=1.0, step=0.05)
    first_leg = [p for p in poses if p[1] == 0.0 and p[0] < 0.4]
    assert all(p[2] == pytest.approx(0.0) for p in first_leg)
    corner_up = [p for p in poses if p[0] == pytest.approx(0.4) and 0.0 < p[1] < 0.3]
    assert all(p[2] == pytest.approx(math.pi / 2) for p in corner_up)
    assert poses[-1][2] == pytest.approx(1.0)


def test_poses_along_one_point_keeps_it_with_the_final_yaw():
    assert poses_along([(1.0, 2.0)], final_yaw=0.5) == [(1.0, 2.0, 0.5)]


def test_poses_along_drops_repeated_points():
    poses = poses_along([(0.0, 0.0), (0.0, 0.0), (0.1, 0.0)], final_yaw=0.0, step=0.05)
    assert [p[:2] for p in poses] == [(0.0, 0.0), pytest.approx((0.05, 0.0)),
                                      pytest.approx((0.1, 0.0))]


def test_poses_backing_keep_the_points_and_face_against_the_motion():
    """후진 경로: 로봇은 진행 방향의 반대를 본다. 마지막 점도 마지막 선분의 반대다."""
    pts = [(0.4, 0.3), (0.4, 0.0), (0.0, 0.0)]
    back = poses_backing(pts, step=0.05)
    assert [p[:2] for p in back] == [p[:2] for p in poses_along(pts, 0.0, 0.05)]
    down = [p for p in back if p[0] == pytest.approx(0.4) and p[1] > 0.0]
    assert down and all(p[2] == pytest.approx(math.pi / 2) for p in down)
    left = [p for p in back if p[1] == 0.0 and p[0] < 0.4]
    assert left and all(abs(p[2]) == pytest.approx(0.0) for p in left)


def test_project_gives_distance_along_and_off_the_line():
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    s, off = line.project((0.3, 0.1))
    assert s == pytest.approx(0.3) and off == pytest.approx(0.1)
    s, off = line.project((1.2, 0.5))
    assert s == pytest.approx(1.5) and off == pytest.approx(0.2)
    assert line.length == pytest.approx(2.0)


def test_project_before_the_start_measures_to_the_first_point():
    s, off = Polyline([(0.0, 0.0), (1.0, 0.0)]).project((-0.3, 0.4))
    assert s == 0.0 and off == pytest.approx(0.5)


def test_project_can_be_limited_to_a_window():
    """같은 곳을 두 번 지나는 선에서는 이미 지난 쪽으로 뛰지 않는다."""
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 0.2), (0.0, 0.2)])
    s, _ = line.project((0.5, 0.12))
    assert s == pytest.approx(1.7)  # 가장 가까운 것은 돌아오는 쪽
    s, _ = line.project((0.5, 0.12), s_max=1.0)
    assert s == pytest.approx(0.5)


def test_rest_starts_at_the_projection_and_keeps_later_vertices():
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert line.rest(0.25) == [pytest.approx((0.25, 0.0)), (1.0, 0.0), (1.0, 1.0)]
    assert line.rest(1.0) == [pytest.approx((1.0, 0.0)), (1.0, 1.0)]
    assert line.rest(5.0) == [(1.0, 1.0)]


def test_sample_covers_the_window_in_small_steps():
    line = Polyline([(0.0, 0.0), (1.0, 0.0)])
    pts = line.sample(0.2, 0.7, step=0.1)
    assert pts[0] == pytest.approx((0.2, 0.0)) and pts[-1] == pytest.approx((0.7, 0.0))
    assert len(pts) == 6
    assert line.sample(0.9, 1.4, step=0.1)[-1] == pytest.approx((1.0, 0.0))


def test_single_point_polyline():
    line = Polyline([(1.0, 1.0), (1.0, 1.0)])
    assert line.length == 0.0
    assert line.project((1.0, 1.3)) == (0.0, pytest.approx(0.3))
    assert line.rest(0.0) == [(1.0, 1.0)]


def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.1):
        assert quaternion_yaw(*yaw_quaternion(yaw)) == pytest.approx(yaw)


def test_heading_at_is_the_direction_of_the_segment_behind_the_point():
    """At a corner the robot came along the segment behind it; backing up goes along that one."""
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert line.heading_at(0.0) == pytest.approx(0.0)
    assert line.heading_at(0.5) == pytest.approx(0.0)
    assert line.heading_at(1.0) == pytest.approx(0.0)  # the corner itself
    assert line.heading_at(1.2) == pytest.approx(math.pi / 2)
    assert line.heading_at(9.0) == pytest.approx(math.pi / 2)
    assert Polyline([(2.0, 3.0)]).heading_at(0.0) == 0.0


def test_vertex_before_is_the_last_vertex_at_or_behind_the_point():
    line = Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert line.vertex_before(0.0) == pytest.approx(0.0)
    assert line.vertex_before(0.5) == pytest.approx(0.0)  # the first point counts
    assert line.vertex_before(1.0) == pytest.approx(1.0)  # the corner itself
    assert line.vertex_before(1.2) == pytest.approx(1.0)
    assert line.vertex_before(2.0) == pytest.approx(2.0)
    assert Polyline([(2.0, 3.0)]).vertex_before(0.5) == 0.0


# ---- 꺾이는 곳을 호로(추가 지시서 3의 2-1: 좁은 띠 안에서 제자리 회전을 하지 않는다) ----


def _turns(points):
    """이웃 선분 사이의 방향 바뀜(rad)."""
    heads = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in pairwise(points)]
    return [abs(math.remainder(h2 - h1, math.tau)) for h1, h2 in pairwise(heads)]


def test_rounded_replaces_a_corner_with_an_arc_of_the_radius():
    line = rounded([(0.35, 0.65), (0.35, 0.89), (0.18, 0.89)], radius=0.10, step=0.01)
    assert line[0] == (0.35, 0.65) and line[-1] == (0.18, 0.89)
    assert max(_turns(line)) < 0.15, "no sharp vertex is left for the robot to pivot on"
    centre = (0.25, 0.79)
    arc = [p for p in line if p[0] < 0.35 - 1e-9 and p[1] < 0.89 - 1e-9]
    assert arc and all(math.dist(p, centre) == pytest.approx(0.10) for p in arc)
    corner_gap = min(math.dist(p, (0.35, 0.89)) for p in line)
    assert corner_gap == pytest.approx(0.10 * (math.sqrt(2) - 1), abs=2e-3)


def test_rounded_shrinks_the_arc_to_fit_short_segments():
    line = rounded([(0.0, 0.0), (0.4, 0.0), (0.4, 0.06), (0.8, 0.06)], radius=0.10, step=0.01)
    assert max(_turns(line)) < 0.4
    assert min(y for _, y in line) >= -1e-9 and max(y for _, y in line) <= 0.06 + 1e-9
    assert line[-1] == (0.8, 0.06)


def test_rounded_keeps_straight_lines_and_turnarounds_and_zero_radius():
    straight = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
    assert rounded(straight, 0.1) == straight
    back = [(0.0, 0.0), (0.5, 0.0), (0.2, 0.0)]
    assert rounded(back, 0.1) == back, "a reversal cannot be rounded and stays as it is"
    corner = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)]
    assert rounded(corner, 0.0) == corner


def test_rounded_rest_follows_the_arc_from_where_the_robot_is():
    route = Polyline([(0.35, 0.65), (0.35, 0.89), (0.18, 0.89)])
    whole = rounded_rest(route, 0.0, 0.10)
    assert whole[0] == pytest.approx((0.35, 0.65)) and whole[-1] == pytest.approx((0.18, 0.89))
    near = rounded_rest(route, 0.23, 0.10)
    assert max(_turns(near)) < 0.15, "a robot close to the corner still gets the arc, not a pivot"
    assert near[0][0] < 0.35 and near[-1] == pytest.approx((0.18, 0.89))
    assert rounded_rest(Polyline([(0.0, 0.0), (1.0, 0.0)]), 0.4, 0.1) == [
        pytest.approx((0.4, 0.0)), (1.0, 0.0)]


# ---- 이음 정밀도 (추가 지시서 3의 2-4) --------------------------------------------------------

LANE = Polyline([(0.35, 0.20), (0.35, 0.89), (0.18, 0.89)])  # 곧게 0.69 m, 그리고 왼쪽으로 꺾임


def joined(route, s, here, min_off=0.02, slope=3.0, radius=0.10):
    return rounded_rest(route, s, radius, here, min_off, slope)


def test_a_robot_on_the_line_gets_the_rest_as_before():
    assert joined(LANE, 0.10, (0.35, 0.30)) == [
        pytest.approx(p) for p in rounded_rest(LANE, 0.10, 0.10)]


def test_a_robot_beside_the_line_starts_where_it_stands_and_joins_the_line_ahead():
    rest = joined(LANE, 0.10, (0.43, 0.30))  # 0.08 m 옆
    assert rest[0] == pytest.approx((0.43, 0.30))
    assert rest[1] == pytest.approx((0.35, 0.30 + 3.0 * 0.08)), "three times the offset ahead"
    assert rest[-1] == pytest.approx((0.18, 0.89))
    assert rest[2:] == rounded_rest(LANE, 0.10, 0.10)[1:], "from there on the line as it was"


def test_the_join_never_cuts_across_a_corner():
    """Close to a corner the join is the start of the arc, not a point beyond it."""
    rest = joined(LANE, 0.50, (0.45, 0.70))  # 0.10 m 옆, 호의 시작(y 0.79)까지 0.09 m
    assert rest[0] == pytest.approx((0.45, 0.70))
    assert rest[1] == pytest.approx((0.35, 0.79))


def test_a_small_offset_keeps_the_start_on_the_line():
    rest = joined(LANE, 0.10, (0.365, 0.30))  # 0.015 m 옆
    assert rest[0] == pytest.approx((0.35, 0.30))


def test_on_an_arc_the_rest_starts_beside_the_robot_not_behind_it():
    """The progress s is measured on the sharp line. Its place on the arc lies behind a robot
    that drives the arc."""
    arc = rounded(LANE.points, 0.10)
    robot = arc[len(arc) // 2]  # 호의 가운데
    s, _ = LANE.project(robot)
    assert math.dist(rounded_rest(LANE, s, 0.10)[0], robot) > 0.01, "the old start is behind"
    assert math.dist(joined(LANE, s, robot)[0], robot) < 1e-6


def test_without_a_slope_the_start_is_on_the_line_as_before():
    assert joined(LANE, 0.10, (0.43, 0.30), slope=0.0) == rounded_rest(LANE, 0.10, 0.10)


def _on_arc(deg, radius=0.10):
    """A point at deg on the circle of LANE's corner arc (centre 0.25, 0.79; 0 deg is its start)."""
    a = math.radians(deg)
    return (0.25 + radius * math.cos(a), 0.79 + radius * math.sin(a))


def test_beside_an_arc_the_join_is_ahead_on_the_arc_not_at_its_next_sample():
    robot = _on_arc(30.0, radius=0.13)  # 호의 바깥쪽으로 0.03 m
    s, _ = LANE.project(robot)
    rest = joined(LANE, s, robot)
    assert rest[0] == pytest.approx(robot)
    assert rest[1] == pytest.approx(_on_arc(30.0 + math.degrees(0.09 / 0.10)), abs=2e-3)


def test_the_join_on_an_arc_ends_with_the_arc():
    robot = _on_arc(80.0, radius=0.15)  # 0.05 m 밖: 0.15 m 앞은 호의 끝을 넘는다
    s, _ = LANE.project(robot)
    assert joined(LANE, s, robot)[1] == pytest.approx((0.25, 0.89), abs=1e-6)


def test_a_line_that_comes_back_over_itself_is_followed_from_the_second_pass():
    there_and_back = Polyline([(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)])
    assert rounded_rest(there_and_back, 1.8, 0.10) == [pytest.approx((0.2, 0.0)), (0.0, 0.0)]
    assert joined(there_and_back, 1.8, (0.2, 0.05)) == [
        pytest.approx((0.2, 0.05)), pytest.approx((0.05, 0.0)), (0.0, 0.0)]


def test_a_robot_past_the_end_is_not_sent_back_along_a_new_piece():
    line = Polyline([(0.0, 0.0), (1.0, 0.0)])
    assert joined(line, 1.0, (1.05, 0.0)) == [(1.0, 0.0)]
