"""order 모델을 노드가 나르는 방식: MoveOrderCmd, 피드백, update, 파라미터."""
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
    return MoveOrderCmd("pode1", goal_id, "M-1", update_id, "C3" if update_id else "", base)


def feedback(seq, waiting=False, warning="", left=0.25):
    return SimpleNamespace(feedback=MoveTo.Feedback(
        distance_remaining=left, last_done_seq_id=seq, waiting_at_base_end=waiting,
        horizon_warning=warning))


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]"])
    made = FleetManagerNode()
    made.core = MagicMock()
    made._move["pode1"] = MagicMock()
    yield made
    made.destroy_node()
    rclpy.shutdown()


def test_an_order_command_becomes_one_move_goal_with_a_feedback_callback(node):
    client = node._move["pode1"]
    node._dispatch(order(7), 1.0)
    (goal,), kwargs = client.send_goal_async.call_args
    assert isinstance(goal, MoveTo.Goal)
    assert (goal.order_id, goal.target_id, len(goal.base)) == ("M-1", "C3", 1)
    assert callable(kwargs["feedback_callback"])


def test_a_plain_move_command_gets_no_feedback_callback(node):
    client = node._move["pode1"]
    node._dispatch(MoveCmd("pode1", 7, "N_W", 0.4, 0.65, 0.0), 1.0)
    assert client.send_goal_async.call_args.kwargs == {}


def test_every_order_feedback_reaches_the_core_so_the_distance_stays_fresh(node):
    client = node._move["pode1"]
    node._dispatch(order(7), 1.0)
    callback = client.send_goal_async.call_args.kwargs["feedback_callback"]
    for msg in (feedback(0), feedback(0), feedback(0, left=0.125), feedback(1, True),
                feedback(1, True, "HORIZON_INVALID: x")):
        callback(msg)
    assert node.core.on_move_feedback.call_args_list == [], "the callback only queues"
    node._drain()
    assert [c.args for c in node.core.on_move_feedback.call_args_list] == [
        ("pode1", 7, 0, False, "", 0.25), ("pode1", 7, 0, False, "", 0.25),
        ("pode1", 7, 0, False, "", 0.125), ("pode1", 7, 1, True, "", 0.25),
        ("pode1", 7, 1, True, "HORIZON_INVALID: x", 0.25)]


def test_feedback_of_two_goals_is_told_apart(node):
    client = node._move["pode1"]
    node._dispatch(order(7), 1.0)
    first = client.send_goal_async.call_args.kwargs["feedback_callback"]
    node._dispatch(order(8, 1), 2.0)
    second = client.send_goal_async.call_args.kwargs["feedback_callback"]
    first(feedback(2))
    second(feedback(2))
    node._drain()
    assert [c.args[1] for c in node.core.on_move_feedback.call_args_list] == [7, 8]


def test_an_update_leaves_the_old_goal_alone_and_its_superseded_result_reaches_the_core(node):
    old = SimpleNamespace(cancel_goal_async=MagicMock())
    node._goals.sent(("pode1", 7))
    node._goals.accepted(("pode1", 7), old)
    node._dispatch(order(8, 1), 2.0)
    old.cancel_goal_async.assert_not_called()
    assert not node._goals.canceling(("pode1", 7))
    response = SimpleNamespace(status=4, result=MoveTo.Result(
        success=False, reason="SUPERSEDED", last_done_seq_id=3))
    done = SimpleNamespace(result=lambda: response)
    node._goal_result(("pode1", 7), node._move_done, node._robot_rejected, done)
    node.core.on_move_result.assert_called_once_with("pode1", 7, False, "SUPERSEDED", 3)
    node.core.on_goal_rejected.assert_not_called()


def test_order_parameters_override_the_params_file_only_when_given():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", "use_order_model:=false",
                     "-p", "lookahead_s:=3.5"])
    try:
        node = FleetManagerNode()
        try:
            assert node.core.params.use_order_model is False
            assert node.core.params.lookahead_s == 3.5
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]"])
    try:
        node = FleetManagerNode()
        try:
            assert node.core.params.use_order_model is True  # 기본 켬
            assert node.core.params.lookahead_s == 2.0
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
