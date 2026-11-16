"""order model (contract 0.11-draft proposal) through the real action interfaces: the robot stops
at the end of base, reports it in feedback, and an update supersedes the old goal."""
import math

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("nav2_msgs.action")

from conftest import wait_until  # noqa: E402
from fleet_interfaces.action import MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState, Segment  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from probe import WAYPOINTS  # noqa: E402
from test_ros_fsm import start  # noqa: E402


def seg(seq, a, b):
    (ax, ay), (bx, by) = WAYPOINTS[a], WAYPOINTS[b]
    s = Segment(seq_id=seq, mode=Segment.MODE_FOLLOW_PATH, controller_id="FollowPath",
                end_node_id=b)
    s.path.header.frame_id = "map"
    yaw = math.atan2(by - ay, bx - ax)
    for x, y in ((ax, ay), (bx, by)):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        s.path.poses.append(p)
    return s


def order_goal(base, horizon, update=0, stitch=""):
    return MoveTo.Goal(order_id="o1", order_update_id=update, stitch_node_id=stitch,
                       base=base, horizon=horizon)


def test_the_robot_waits_at_the_end_of_base_and_an_update_takes_it_on(harness):
    fake, probe = start(harness)
    feedback = []
    fut = probe.move.send_goal_async(order_goal([seg(1, "P-3", "P-4")], [seg(2, "P-4", "P-5")]),
                                     feedback_callback=lambda m: feedback.append(m.feedback))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    first = fut.result()
    assert wait_until(lambda: any(f.waiting_at_base_end for f in feedback), 10.0)
    last = feedback[-1]
    assert (last.last_done_seq_id, last.horizon_warning) == (1, "")
    assert math.dist(fake.pose[:2], WAYPOINTS["P-4"]) < 0.15, "it stopped at the base end"
    assert probe.last_state().state == RobotState.STATE_MOVING
    assert not first.get_result_async().done(), "the goal waits for an update"
    second = probe.send(order_goal([seg(2, "P-4", "P-5")], [], update=1, stitch="P-4"))
    old = probe.result(first).result
    assert (old.success, old.reason, old.last_done_seq_id) == (
        False, MoveTo.Result.REASON_SUPERSEDED, 1)
    new = probe.result(second).result
    assert (new.success, new.last_done_seq_id) == (True, 2)


def test_a_wrong_update_is_refused_and_the_order_goes_on(harness):
    _, probe = start(harness)
    first = probe.send(order_goal([seg(1, "P-3", "P-4")], []))
    bad = probe.result(probe.send(order_goal([seg(2, "P-4", "P-5")], [], update=1,
                                             stitch="P-9"))).result
    assert not bad.success and bad.reason.startswith(
        MoveTo.Result.REASON_ORDER_UPDATE_REJECTED + ": ")
    res = probe.result(first).result
    assert (res.success, res.last_done_seq_id) == (True, 1)
