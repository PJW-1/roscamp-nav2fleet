"""다른 로봇의 몸을 costmap에 찍을 점으로 바꾸는 계산(ROS 없이 돈다)."""
import math

import pytest
from pode_peer_obstacles.body import CENTRE_M, RADIUS_M, body_points, others


def test_the_other_robots_are_every_name_but_mine():
    assert others("pode1, pode2,pode3", "pode2") == ["pode1", "pode3"]


def test_an_empty_list_has_no_other_robots():
    assert others("", "pode1") == []


def test_a_name_listed_twice_is_one_robot():
    assert others("pode1,pode3,pode3", "pode1") == ["pode3"]


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


# local_costmap 다각형 발자국(nav2_params.yaml)과 같은 꼭짓점
FOOTPRINT = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049),
             (-0.071, 0.042), (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042),
             (-0.049, -0.049), (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031),
             (0.040, -0.021)]


def test_the_default_disc_covers_the_body_with_a_small_margin():
    """P-3 앞 멈춤(2026-11-17): 0.08은 몸보다 옆 2.4 cm 커서 좁은 틈을 막았다. 몸 가운데에서 가장 먼
    꼭짓점(0.073 m)을 덮고 여유는 2 mm만 둔다."""
    farthest = max(math.dist(p, (CENTRE_M, 0.0)) for p in FOOTPRINT)
    assert RADIUS_M == 0.075
    assert farthest < RADIUS_M <= farthest + 0.003
