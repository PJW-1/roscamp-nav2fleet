"""MoveTo mode nav_goal (contract 0.11-draft proposal) through the real action interfaces.
FakeNav stands in for bt_navigator's navigate_to_pose and the controller's follow_path."""
import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("nav2_msgs.action")

from conftest import wait_until  # noqa: E402
from fleet_interfaces.action import MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from geometry_msgs.msg import Pose2D  # noqa: E402
from probe import WAYPOINTS, lane_goal  # noqa: E402
from test_ros_fsm import start  # noqa: E402


def nav_goal(target_id, theta=math.pi / 2):
    x, y = WAYPOINTS[target_id]
    return MoveTo.Goal(target_id=target_id, target_pose=Pose2D(x=x, y=y, theta=theta),
                       mode=MoveTo.Goal.MODE_NAV_GOAL)


def test_a_nav_goal_drives_with_navigate_to_pose_and_reports_the_distance_left(harness):
    fake, probe = start(harness)
    feedback = []
    fut = probe.move.send_goal_async(nav_goal("P-4"),
                                     feedback_callback=lambda m: feedback.append(
                                         m.feedback.distance_remaining))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_MOVING, 2.0)
    res = probe.result(fut.result())
    assert (res.result.success, res.result.reason) == (True, "")
    assert len(fake.nav_goals) == 1 and fake.goals == [], "no FollowPath of its own"
    assert fake.nav_goals[0][1] == pytest.approx((*WAYPOINTS["P-4"], math.pi / 2))
    assert feedback and 0.0 < max(feedback) <= 0.41
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_IDLE, 1.0)


def test_a_nav_goal_without_progress_reports_blocked_and_goes_on_when_clear(harness):
    fake, probe = start(harness, blocked_after_s=1.0)
    fake.obstacle = WAYPOINTS["P-4"]
    h = probe.send(nav_goal("P-4"))
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 5.0)
    assert not h.get_result_async().done(), "the goal keeps running"
    assert fake.backups == [] and fake.clears == [], "no pinky_fsm self-recovery in nav_goal"
    fake.obstacle = None
    assert wait_until(lambda: probe.last_state().state != RobotState.STATE_BLOCKED, 3.0)
    res = probe.result(h)
    assert res.result.success is True and len(fake.nav_goals) == 1


def test_cancel_stops_navigate_to_pose(harness):
    fake, probe = start(harness)
    h = probe.send(nav_goal("P-4"))
    assert wait_until(lambda: fake.nav_goals, 2.0)
    time.sleep(0.2)
    h.cancel_goal_async()
    res = probe.result(h)
    assert (res.result.success, res.result.reason) == (False, MoveTo.Result.REASON_CANCELED)
    assert wait_until(lambda: fake.nav_goal_cancels and fake.v == 0.0, 2.0)


def test_heartbeat_loss_stops_navigate_to_pose(harness):
    fake, probe = start(harness, heartbeat_timeout_s=1.0)
    fake.speed = 0.1
    res = probe.result(probe.send(nav_goal("P-4")))
    assert (res.result.success, res.result.reason) == (False, MoveTo.Result.REASON_HEARTBEAT_LOST)
    assert wait_until(lambda: fake.nav_goal_cancels and fake.v == 0.0, 2.0)


def test_a_follow_path_goal_after_a_nav_goal_waits_for_navigate_to_pose_to_end(harness):
    """The two would fight over the controller: cancel NavigateToPose, wait for its result and
    a standing robot, then send FollowPath."""
    fake, probe = start(harness)
    fake.speed = 0.2
    first = probe.send(nav_goal("P-4"))
    assert wait_until(lambda: fake.pose[1] > 1.65, 3.0)
    second = probe.send(lane_goal(["P-3", "P-4"], path=False))
    old = probe.result(first)
    assert (old.result.success, old.result.reason) == (False, MoveTo.Result.REASON_CANCELED)
    assert probe.result(second).result.success is True
    assert fake.nav_goal_cancels, "NavigateToPose was cancelled first"
    assert fake.goals and fake.goals[0][0] >= fake.nav_goal_cancels[0]


def test_an_empty_mode_is_todays_follow_path(harness):
    fake, probe = start(harness)
    goal = lane_goal(["P-3", "P-4"])
    assert goal.mode == ""
    assert probe.result(probe.send(goal)).result.success is True
    assert fake.nav_goals == [] and fake.goals
