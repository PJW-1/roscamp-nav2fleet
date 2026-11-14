"""비전 대역의 기하: 상자 면 위의 점 가운데 카메라 원뿔 안의 것만 고르는지 본다(ROS 없음)."""
import math

import pytest
from pinky_avoidance.vision_cone import (
    Box,
    box_from_marker,
    face_points,
    make_cone,
    seen_points,
    to_body,
    yaw_of,
)

CONE = make_cone(70.0, 1.0)
BOX = Box(x=0.5, y=0.0, yaw=0.0, size_x=0.12, size_y=0.12, height=0.12)  # 로봇 앞 0.5 m
STEP = 0.02


def test_a_box_straight_ahead_shows_only_its_near_face():
    points = seen_points([BOX], (0.0, 0.0, 0.0), CONE, STEP)
    assert points
    assert all(x == pytest.approx(0.44) for x, _, _ in points), "only the face toward the robot"
    ys = sorted({round(y, 6) for _, y, _ in points})
    assert ys[0] == pytest.approx(-0.06) and ys[-1] == pytest.approx(0.06)
    zs = sorted({round(z, 6) for _, _, z in points})
    assert zs[0] == pytest.approx(0.0) and zs[-1] == pytest.approx(0.12)
    assert all(abs(b - a) <= STEP + 1e-9 for a, b in zip(ys, ys[1:], strict=False))


def test_a_box_seen_from_a_corner_shows_two_faces():
    pose = (0.0, -0.5, math.atan2(0.5, 0.5))
    points = seen_points([BOX], pose, CONE, STEP)
    near_x = [p for p in points if p[0] == pytest.approx(0.44)]
    near_y = [p for p in points if p[1] == pytest.approx(-0.06)]
    assert len(near_x) > 7 and len(near_y) > 7, "both faces toward the robot"
    assert all(p in near_x or p in near_y for p in points), "nothing from the far faces"


@pytest.mark.parametrize("pose", [(0.0, 0.0, math.pi), (0.0, 0.0, math.pi / 2),
                                  (0.0, 0.0, -math.pi / 2)])
def test_a_box_behind_or_beside_the_camera_is_not_seen(pose):
    assert seen_points([BOX], pose, CONE, STEP) == []


def test_a_box_beyond_the_range_is_not_seen():
    far = Box(x=1.2, y=0.0, yaw=0.0, size_x=0.12, size_y=0.12, height=0.12)
    assert seen_points([far], (0.0, 0.0, 0.0), CONE, STEP) == []
    assert seen_points([far], (0.0, 0.0, 0.0), make_cone(70.0, 1.5), STEP)


def test_only_the_part_of_a_face_inside_the_cone_is_seen():
    wide = Box(x=0.5, y=0.0, yaw=0.0, size_x=0.1, size_y=2.0, height=0.1)
    points = seen_points([wide], (0.0, 0.0, 0.0), CONE, STEP)
    half = math.radians(35.0)
    assert points
    assert all(abs(math.atan2(y, x)) <= half + 1e-9 for x, y, _ in points)
    assert all(math.hypot(x, y) <= 1.0 + 1e-9 for x, y, _ in points)
    assert max(abs(y) for _, y, _ in points) == pytest.approx(0.45 * math.tan(half), abs=STEP)


def test_a_turned_box_has_its_faces_turned_too():
    turned = Box(x=0.0, y=0.0, yaw=math.pi / 4, size_x=0.2, size_y=0.2, height=0.1)
    corners = {(round(x, 6), round(y, 6)) for x, y, _ in face_points(turned, 0.2)}
    r = 0.1 * math.sqrt(2.0)
    assert (0.0, round(r, 6)) in corners and (round(r, 6), 0.0) in corners


def test_the_points_sit_on_the_box_height_from_its_marker_centre():
    box = box_from_marker(0.5, 0.0, 0.06, 0.0, 0.0, 0.0, 1.0, 0.12, 0.12, 0.12)
    assert box.base_z == pytest.approx(0.0) and box.height == pytest.approx(0.12)
    raised = box_from_marker(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.12, 0.12, 0.2)
    zs = [z for _, _, z in face_points(raised, STEP)]
    assert min(zs) == pytest.approx(0.4) and max(zs) == pytest.approx(0.6)


def test_the_yaw_comes_from_the_quaternion():
    assert yaw_of(0.0, 0.0, math.sin(0.3), math.cos(0.3)) == pytest.approx(0.6)
    box = box_from_marker(0.0, 0.0, 0.05, 0.0, 0.0, math.sin(-0.5), math.cos(-0.5), 1, 1, 0.1)
    assert box.yaw == pytest.approx(-1.0)


def test_points_in_the_robot_frame():
    pose = (1.0, 2.0, math.pi / 2)
    [(x, y, z)] = to_body([(1.0, 2.5, 0.1)], pose)
    assert (x, y, z) == pytest.approx((0.5, 0.0, 0.1))
    [(x, y, _)] = to_body([(0.5, 2.0, 0.0)], pose)
    assert (x, y) == pytest.approx((0.0, 0.5))


def test_no_boxes_no_points():
    assert seen_points([], (0.0, 0.0, 0.0), CONE, STEP) == []


@pytest.mark.parametrize("fov, reach", [(0.0, 1.0), (180.0, 1.0), (70.0, 0.0),
                                        (float("nan"), 1.0), (70.0, float("inf"))])
def test_a_cone_must_be_a_forward_cone(fov, reach):
    with pytest.raises(ValueError):
        make_cone(fov, reach)


def test_a_step_must_be_positive():
    with pytest.raises(ValueError, match="step_m"):
        face_points(BOX, 0.0)
