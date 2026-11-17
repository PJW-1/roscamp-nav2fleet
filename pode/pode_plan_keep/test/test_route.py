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
    OTHER,
    SAME,
    SHORTER,
    SILENT,
    Rules,
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
