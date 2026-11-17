"""route: Nav2 goal 경로를 1 Hz로 다시 계획할 때 지켜 둔 길을 이어 쓸지 정하는 순수 함수.
ROS 없이 돈다.

근거: Gazebo 장애물 판에서 Q2 (1.66, 0.16)로 갈 때 가운데 통로 (1.05, 0.55) 부근에서 길이
0.93~1.07 m인 두 길을 1초마다 번갈아 계획해 로봇이 제자리에서 방향만 바꿨다.
"""
import math

import pytest
from pode_plan_keep.route import (
    BLOCKED,
    FRESH,
    KEEP,
    OFF,
    OPEN,
    OTHER,
    SAME,
    SHORTER,
    SILENT,
    UNKNOWN,
    Grid,
    Rules,
    check_route,
    distance_to,
    first_look,
    length,
    second_look,
)

STEP = 0.05


def line(*corners, step=STEP):
    """꼭짓점들을 step 간격의 점으로 잇는다(NavFn 경로처럼 촘촘하게)."""
    points = [corners[0]]
    for a, b in zip(corners, corners[1:], strict=False):
        n = max(1, round(math.dist(a, b) / step))
        points += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                   for i in range(1, n + 1)]
    return points


GOAL = (1.0, 1.0)
EAST_THEN_NORTH = line((0.0, 0.0), (1.0, 0.0), GOAL)  # 2.0 m
NORTH_THEN_EAST = line((0.0, 0.0), (0.0, 1.0), GOAL)  # 2.0 m, 다른 길


# ---- 길이와 거리 ----

def test_length_and_distance_to_a_polyline():
    assert length(EAST_THEN_NORTH) == pytest.approx(2.0)
    assert length([(0.0, 0.0)]) == 0.0
    assert distance_to(EAST_THEN_NORTH, (0.5, 0.1)) == pytest.approx(0.1)
    assert distance_to(EAST_THEN_NORTH, (1.2, 0.5)) == pytest.approx(0.2)
    assert distance_to([(0.0, 0.0)], (0.3, 0.4)) == pytest.approx(0.5)


# ---- 처음 보기: 지켜 둔 길과 새 길 ----

def test_nothing_kept_means_the_new_plan():
    assert first_look([], None, NORTH_THEN_EAST, GOAL)[0] == FRESH


@pytest.mark.parametrize("moved, verdict", [(0.04, OTHER), (0.05, FRESH), (0.5, FRESH)])
def test_a_goal_moved_by_5_cm_or_more_is_a_new_goal(moved, verdict):
    goal = (GOAL[0] + moved, GOAL[1])
    assert first_look(EAST_THEN_NORTH, GOAL, NORTH_THEN_EAST, goal)[0] == verdict


def test_the_kept_route_starts_at_its_point_nearest_the_robot():
    robot = (0.52, 0.01)
    new = line(robot, (1.0, 0.0), GOAL)
    verdict, start = first_look(EAST_THEN_NORTH, GOAL, new, GOAL)
    assert verdict == SAME
    assert EAST_THEN_NORTH[start] == pytest.approx((0.5, 0.0))


def test_a_new_plan_within_20_cm_of_the_kept_route_is_the_same_route():
    near = line((0.0, 0.0), (0.0, 0.15), (1.0, 0.15), GOAL)  # 15 cm 옆
    assert first_look(EAST_THEN_NORTH, GOAL, near, GOAL)[0] == SAME
    far = line((0.0, 0.0), (0.0, 0.25), (1.0, 0.25), GOAL)  # 25 cm 옆
    assert first_look(EAST_THEN_NORTH, GOAL, far, GOAL)[0] == OTHER


def test_another_way_to_the_same_goal_is_another_route():
    assert first_look(EAST_THEN_NORTH, GOAL, NORTH_THEN_EAST, GOAL) == (OTHER, 0)


def test_a_robot_more_than_30_cm_off_the_kept_route_takes_the_new_plan():
    new = line((0.5, 0.35), GOAL)
    assert first_look(EAST_THEN_NORTH, GOAL, new, GOAL)[0] == OFF
    new = line((0.5, 0.25), (1.0, 0.25), GOAL)  # 25 cm 옆: 벗어난 것은 아니고 다른 길이다
    assert first_look(EAST_THEN_NORTH, GOAL, new, GOAL)[0] == OTHER


def test_a_kept_route_with_one_point_left_is_not_kept():
    robot = (1.0, 1.0)
    assert first_look(EAST_THEN_NORTH, GOAL, [robot, GOAL], GOAL)[0] == FRESH


def test_an_empty_new_plan_is_passed_on():
    assert first_look(EAST_THEN_NORTH, GOAL, [], GOAL)[0] == FRESH


def test_the_rules_can_be_changed():
    near = line((0.0, 0.0), (0.0, 0.15), (1.0, 0.15), GOAL)
    assert first_look(EAST_THEN_NORTH, GOAL, near, GOAL, Rules(same_route_m=0.10))[0] == OTHER


# ---- 다음 보기: 다른 길일 때 ----

def test_a_blocked_or_silent_kept_route_gives_way_to_the_new_plan():
    assert second_look(EAST_THEN_NORTH, NORTH_THEN_EAST, False) == BLOCKED
    assert second_look(EAST_THEN_NORTH, NORTH_THEN_EAST, None) == SILENT


def test_an_open_kept_route_stays_unless_the_new_plan_is_clearly_shorter():
    assert second_look(EAST_THEN_NORTH, NORTH_THEN_EAST, True) == KEEP


@pytest.mark.parametrize("kept_m, new_m, verdict", [
    (2.0, 1.61, KEEP), (2.0, 1.59, SHORTER),  # 2.0 m: 20 %인 0.40 m보다 짧아야 한다
    (1.0, 0.71, KEEP), (1.0, 0.69, SHORTER),  # 1.0 m: 0.30 m보다 짧아야 한다
])
def test_clearly_shorter_means_30_cm_or_20_percent_whichever_is_more(kept_m, new_m, verdict):
    kept = line((0.0, 0.0), (kept_m, 0.0))
    new = line((0.0, 0.0), (0.0, new_m))
    assert second_look(kept, new, True) == verdict


def test_the_measured_flip_between_0_93_and_1_07_m_keeps_either_route():
    short, long_ = line((0.0, 0.0), (0.93, 0.0)), line((0.0, 0.0), (0.0, 1.07))
    assert second_look(short, long_, True) == KEEP
    assert second_look(long_, short, True) == KEEP


# ---- 막힘: 전역 비용 지도 칸(planner NavFn과 같은 기준) ----
# OccupancyGrid 값은 Jazzy Costmap2DPublisher의 변환표를 따른다: INSCRIBED 253 -> 99, LETHAL 254 ->
# 100, 모름 255 -> -1. NavFn은 253 이상을 지나가지 못하고 allow_unknown이면 모르는 칸은 지나간다.

def grid(cells=None, yaw=0.0):
    """x -0.5~1.5, y -0.5~1.5, 5 cm(40 x 40). cells: {(x, y): 값}."""
    data = [0] * (40 * 40)
    for (x, y), value in (cells or {}).items():
        data[math.floor((y + 0.5) / 0.05) * 40 + math.floor((x + 0.5) / 0.05)] = value
    return Grid(origin_x=-0.5, origin_y=-0.5, yaw=yaw, resolution=0.05, width=40, height=40,
                data=data)


def test_an_open_route_has_no_cell_the_planner_cannot_cross():
    assert check_route(grid({(0.5, 0.5): 100}), EAST_THEN_NORTH) == OPEN


@pytest.mark.parametrize("value, blocked", [(100, True), (99, True), (98, False), (-1, False)])
def test_inscribed_and_lethal_cells_block_but_unknown_does_not(value, blocked):
    found = check_route(grid({(0.51, 0.01): value}), EAST_THEN_NORTH)
    if blocked:
        x, y, cost = found
        assert (x, y, cost) == (pytest.approx(0.5), pytest.approx(0.0), value)
    else:
        assert found == OPEN


def test_the_first_blocked_point_along_the_route_is_named():
    found = check_route(grid({(1.0, 0.5): 100, (0.3, 0.0): 99}), EAST_THEN_NORTH)
    assert found[:2] == (pytest.approx(0.3), pytest.approx(0.0)) and found[2] == 99


def test_the_threshold_can_be_changed():
    assert check_route(grid({(0.51, 0.01): 98}), EAST_THEN_NORTH, blocked_cost=98) != OPEN


def test_the_goal_point_itself_is_not_checked():
    """NavFn은 goal 칸의 비용을 보지 않고 거기서부터 퍼진다.
    끝 점 하나만 막혀 있으면 열린 길이다."""
    assert check_route(grid({GOAL: 100}), EAST_THEN_NORTH) == OPEN


def test_a_point_outside_the_costmap_is_blocked():
    route = line((0.0, 0.0), (1.6, 0.0), (1.6, 0.5))
    x, y, cost = check_route(grid(), route)
    assert x >= 1.5 and cost is None


def test_no_costmap_or_a_turned_costmap_gives_no_answer():
    assert check_route(None, EAST_THEN_NORTH) == UNKNOWN
    assert check_route(grid(yaw=0.1), EAST_THEN_NORTH) == UNKNOWN
