"""order 모델(계약 0.11-draft 제안)을 노드가 나르는 방식. MoveOrderCmd, 피드백, update, 파라미터."""
import os
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.commands import MoveCmd, MoveOrderCmd, SegmentOut  # noqa: E402
from fleet_interfaces.action import MoveTo  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")

BASE = (SegmentOut(1, "nav_goal", "C3", ("WIDE",), x=1.0, y=0.5, yaw=0.0),)


def order(goal_id, update_id=0, base=BASE):
    return MoveOrderCmd("pinky1", goal_id, "M-1", update_id, "C3" if update_id else "", base)


def feedback(seq, waiting=False, warning=""):
    return SimpleNamespace(feedback=MoveTo.Feedback(
        distance_remaining=0.3, last_done_seq_id=seq, waiting_at_base_end=waiting,
        horizon_warning=warning))


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pinky1]"])
    made = FleetManagerNode()
    made.core = MagicMock()
    made._move["pinky1"] = MagicMock()
    yield made
    made.destroy_node()
    rclpy.shutdown()


def test_an_order_command_becomes_one_move_goal_with_a_feedback_callback(node):
    client = node._move["pinky1"]
    node._dispatch(order(7), 1.0)
    (goal,), kwargs = client.send_goal_async.call_args
    assert isinstance(goal, MoveTo.Goal)
    assert (goal.order_id, goal.target_id, len(goal.base)) == ("M-1", "C3", 1)
    assert callable(kwargs["feedback_callback"])


def test_a_plain_move_command_gets_no_feedback_callback(node):
    client = node._move["pinky1"]
    node._dispatch(MoveCmd("pinky1", 7, "N_W", 0.4, 0.65, 0.0), 1.0)
    assert client.send_goal_async.call_args.kwargs == {}


def test_order_feedback_reaches_the_core_only_when_a_value_changed(node):
    client = node._move["pinky1"]
    node._dispatch(order(7), 1.0)
    callback = client.send_goal_async.call_args.kwargs["feedback_callback"]
    for msg in (feedback(0), feedback(0), feedback(1), feedback(1, True), feedback(1, True),
                feedback(1, True, "HORIZON_INVALID: x")):
        callback(msg)
    assert [c.args for c in node.core.on_move_feedback.call_args_list] == [
        ("pinky1", 7, 0, False, ""), ("pinky1", 7, 1, False, ""), ("pinky1", 7, 1, True, ""),
        ("pinky1", 7, 1, True, "HORIZON_INVALID: x")]


def test_feedback_of_two_goals_is_told_apart(node):
    client = node._move["pinky1"]
    node._dispatch(order(7), 1.0)
    first = client.send_goal_async.call_args.kwargs["feedback_callback"]
    node._dispatch(order(8, 1), 2.0)
    second = client.send_goal_async.call_args.kwargs["feedback_callback"]
    first(feedback(2))
    second(feedback(2))
    assert [c.args[1] for c in node.core.on_move_feedback.call_args_list] == [7, 8]


def test_an_update_leaves_the_old_goal_alone_and_its_superseded_result_reaches_the_core(node):
    old = SimpleNamespace(cancel_goal_async=MagicMock())
    node._goals.sent(("pinky1", 7))
    node._goals.accepted(("pinky1", 7), old)
    node._dispatch(order(8, 1), 2.0)
    old.cancel_goal_async.assert_not_called()
    assert not node._goals.canceling(("pinky1", 7))
    response = SimpleNamespace(status=4, result=MoveTo.Result(
        success=False, reason="SUPERSEDED", last_done_seq_id=3))
    done = SimpleNamespace(result=lambda: response)
    node._goal_result(("pinky1", 7), node._move_done, node._robot_rejected, done)
    node.core.on_move_result.assert_called_once_with("pinky1", 7, False, "SUPERSEDED", 3)
    node.core.on_goal_rejected.assert_not_called()


def test_order_parameters_override_the_params_file_only_when_given():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pinky1]", "-p", "use_order_model:=true",
                     "-p", "lookahead_s:=3.5"])
    try:
        node = FleetManagerNode()
        try:
            assert node.core.params.use_order_model is True
            assert node.core.params.lookahead_s == 3.5
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", "robots:=[pinky1]"])
    try:
        node = FleetManagerNode()
        try:
            assert node.core.params.use_order_model is False
            assert node.core.params.lookahead_s == 2.0
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
