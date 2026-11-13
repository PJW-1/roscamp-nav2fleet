"""다른 로봇의 몸을 costmap에 찍을 점으로 바꾸는 계산(ROS 없이 돈다)."""
import math

import pytest

from pinky_peer_obstacles.body import body_points, others


def test_the_other_robots_are_every_name_but_mine():
    assert others("pinky1, pinky2,pinky3", "pinky2") == ["pinky1", "pinky3"]


def test_an_empty_list_has_no_other_robots():
    assert others("", "pinky1") == []


def test_a_name_listed_twice_is_one_robot():
    assert others("pinky1,pinky3,pinky3", "pinky1") == ["pinky3"]


def test_the_points_fill_a_disc_around_the_body_centre():
    points = body_points(1.0, 0.5, 0.0, radius_m=0.08, step_m=0.02, centre_m=0.0)
    assert all(math.dist(p, (1.0, 0.5)) <= 0.08 + 1e-9 for p in points)
    # 원 안의 어느 자리든 점에서 한 칸(대각선 반) 안이다: 빈 데가 없다
    for angle in range(0, 360, 15):
        for r in (0.0, 0.03, 0.06):
            spot = (1.0 + r * math.cos(math.radians(angle)),
                    0.5 + r * math.sin(math.radians(angle)))
            assert min(math.dist(spot, p) for p in points) <= 0.02 * math.sqrt(0.5) + 1e-9


def test_the_rim_of_the_disc_is_marked_too():
    points = body_points(0.0, 0.0, 0.0, radius_m=0.08, step_m=0.02, centre_m=0.0)
    for angle in range(0, 360, 10):
        rim = (0.08 * math.cos(math.radians(angle)), 0.08 * math.sin(math.radians(angle)))
        assert min(math.dist(rim, p) for p in points) <= 0.02


@pytest.mark.parametrize("yaw, centre", [(0.0, (0.982, 0.5)), (math.pi / 2, (1.0, 0.482)),
                                         (math.pi, (1.018, 0.5))])
def test_the_disc_sits_on_the_body_centre_behind_the_wheel_axis(yaw, centre):
    """base_footprint는 바퀴 축에 있고 몸은 그 뒤로 더 길다: 원의 가운데는 뒤로 0.018 m다."""
    points = body_points(1.0, 0.5, yaw, radius_m=0.08, step_m=0.02, centre_m=-0.018)
    mean = (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))
    assert math.dist(mean, centre) < 1e-6
    assert all(math.dist(p, centre) <= 0.08 + 1e-9 for p in points)
