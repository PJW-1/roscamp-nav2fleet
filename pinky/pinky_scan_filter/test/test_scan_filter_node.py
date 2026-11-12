"""ScanFilter의 TF 다루기(ROS 2 rclpy가 있을 때만).

- 스캔 콜백이 스캔 시각의 TF를 기다리는 동안 들어온 TF로 자세를 정한다(TF는 도우미 노드가 따로
  도는 스레드로 받는다. 거르개 노드를 TransformListener에 넘기면 main의 executor와 다툰다).
- 스캔 시각의 TF가 없고 가장 최근 TF가 스캔 시각과 MAX_TF_AGE_S보다 멀면 거르지 않는다.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("tf2_ros")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402


@pytest.fixture
def running():
    rclpy.init()
    from pinky_scan_filter.node import ScanFilter
    node = ScanFilter()
    sender = rclpy.create_node("tf_sender")
    executor = SingleThreadedExecutor()
    executor.add_node(node)  # main()처럼 거르개 노드를 executor에 넣는다
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node, sender
    executor.shutdown()
    node.destroy_node()
    sender.destroy_node()
    rclpy.try_shutdown()


def _tf(sender, stamp, x):
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "map", "laser"
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


def test_a_tf_arriving_while_the_scan_waits_places_the_scan(running):
    """on_scan처럼 거르개 노드의 executor 콜백 안에서 기다린다. 그동안 같은 executor는 다른 콜백을
    돌리지 못하므로 TF는 도우미 노드의 스레드로만 들어올 수 있다."""
    node, sender = running
    cast = TransformBroadcaster(sender)
    stamp = Time(seconds=100).to_msg()
    got = {}

    def in_callback():
        timer.cancel()
        threading.Timer(0.02, lambda: cast.sendTransform(_tf(sender, stamp, 1.5))).start()
        got["pose"] = node._lidar_pose("laser", stamp)

    timer = node.create_timer(0.01, in_callback)
    end = time.monotonic() + 2.0
    while "pose" not in got and time.monotonic() < end:
        time.sleep(0.01)
    assert got.get("pose") is not None and got["pose"][0] == pytest.approx(1.5)


def test_the_latest_tf_far_from_the_scan_time_means_no_filtering(running):
    from pinky_scan_filter.node import MAX_TF_AGE_S
    node, sender = running
    cast = TransformBroadcaster(sender)
    cast.sendTransform(_tf(sender, Time(seconds=100).to_msg(), 1.0))
    time.sleep(0.2)
    near = Time(seconds=100 + MAX_TF_AGE_S / 2).to_msg()
    far = Time(seconds=100 + MAX_TF_AGE_S * 3).to_msg()
    assert node._lidar_pose("laser", near)[0] == pytest.approx(1.0)
    assert node._lidar_pose("laser", far) is None
