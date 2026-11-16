"""가짜 Pode(ROS)의 /{pode}/relocalize 서버: 선 로봇은 약 2 s 뒤 성공, 움직이면 BUSY."""
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import wait_until  # noqa: E402
from fake_robots.pode_relocalize import RELOCALIZE_S  # noqa: E402
from fleet_interfaces.action import Relocalize  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from test_ros_pode import CONTRACT_OBJ, result_of, start  # noqa: E402


def client(p):
    c = ActionClient(p.node, Relocalize, CONTRACT_OBJ.name("relocalize", pode="pode1"))
    assert c.wait_for_server(10.0)
    return c


def test_a_standing_robot_relocalizes_in_about_2_s_with_a_valid_pose(harness, tmp_path):
    p = start(harness, tmp_path)
    c = client(p)
    began = time.monotonic()
    fut = c.send_goal_async(Relocalize.Goal(allow_spin=True, allow_global=False))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    res = result_of(fut.result()).result
    assert time.monotonic() - began >= RELOCALIZE_S - 0.1
    assert (res.success, res.reason, res.pose_valid) == (True, "", True)
    assert (res.pose.x, res.pose.y) == pytest.approx((res.start_pose.x, res.start_pose.y))


def test_a_moving_robot_ends_the_relocalize_with_busy(harness, tmp_path):
    p = start(harness, tmp_path)
    c = client(p)
    assert p.goal("O_Q").accepted
    fut = c.send_goal_async(Relocalize.Goal())
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    res = result_of(fut.result(), timeout=1.0).result
    assert (res.success, res.reason) == (False, Relocalize.Result.REASON_BUSY)
