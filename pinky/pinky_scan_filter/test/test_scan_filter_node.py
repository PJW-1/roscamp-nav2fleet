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
    assert node.tf_source == "scan_time"  # 최신 TF로 물러선 것이 아니다


def test_the_latest_tf_far_from_the_scan_time_means_no_filtering(running):
    from pinky_scan_filter.node import MAX_TF_AGE_S
    node, sender = running
    cast = TransformBroadcaster(sender)
    cast.sendTransform(_tf(sender, Time(seconds=100).to_msg(), 1.0))
    time.sleep(0.2)
    near = Time(seconds=100 + MAX_TF_AGE_S / 2).to_msg()
    far = Time(seconds=100 + MAX_TF_AGE_S * 3).to_msg()
    assert node._lidar_pose("laser", near)[0] == pytest.approx(1.0)
    assert node.tf_source == "latest"
    assert node._lidar_pose("laser", far) is None


def test_under_launch_remaps_the_tf_node_has_its_own_name_and_the_robots_tf():
    """launch는 이름(__node), 네임스페이스, /tf remap을 프로세스 인자로 준다. 도우미 노드가 그
    이름을 물려받으면 같은 이름의 노드가 둘이 된다(G09: ros2 param get이 도우미 노드에 닿았다)."""
    rclpy.init(args=["--ros-args", "-r", "__node:=scan_filter", "-r", "__ns:=/pinky1",
                     "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"])
    try:
        from pinky_scan_filter.node import ScanFilter
        node = ScanFilter()
        assert node.get_fully_qualified_name() == "/pinky1/scan_filter"
        assert node.tf_node.get_fully_qualified_name() == "/pinky1/scan_filter_tf"
        assert node.tf_node.resolve_topic_name("/tf") == "/pinky1/tf"
        assert node.tf_node.resolve_topic_name("/tf_static") == "/pinky1/tf_static"
        thread = node.listener.dedicated_listener_thread
        node.destroy_node()
        assert not thread.is_alive()  # TF 스레드도 함께 끝난다
    finally:
        rclpy.try_shutdown()


@pytest.fixture
def filtering():
    """띠가 있는 거르개. 지도: 2 x 1 m, 1 cm, 북쪽 벽 y 0.99~1.00. 라이다는 map (1.0, 0.89)에서
    북쪽을 본다(TF 시각 100 s). 발행은 가짜 publisher가 받는다."""
    from nav_msgs.msg import OccupancyGrid
    rclpy.init(args=["--ros-args", "-p", "strips:=0,0.79,2,1"])
    from pinky_scan_filter.node import ScanFilter
    node = ScanFilter()
    sender = rclpy.create_node("tf_sender")
    grid = OccupancyGrid()
    grid.info.resolution, grid.info.width, grid.info.height = 0.01, 200, 100
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (200 * 99) + [100] * 200
    node.on_map(grid)
    sent = []

    class FakePub:
        def publish(self, msg):
            sent.append(msg)

    node.pub = FakePub()
    t = _tf(sender, Time(seconds=100).to_msg(), 1.0)
    t.transform.translation.y = 0.89
    t.transform.rotation.z, t.transform.rotation.w = 0.7071068, 0.7071068
    TransformBroadcaster(sender).sendTransform(t)
    time.sleep(0.3)
    yield node, sent
    node.destroy_node()
    sender.destroy_node()
    rclpy.try_shutdown()


def _scan(sec, nanosec=0):
    from sensor_msgs.msg import LaserScan
    scan = LaserScan()
    scan.header.frame_id = "laser"
    scan.header.stamp.sec, scan.header.stamp.nanosec = sec, nanosec
    scan.angle_min, scan.angle_increment = 0.0, 1.5707963
    scan.range_min, scan.range_max = 0.05, 3.0
    scan.ranges = [0.085, 0.3]  # 북쪽 벽 곁 (1.0, 0.975), 서쪽 빈 곳 (0.7, 0.89)
    return scan


def test_on_scan_turns_a_beam_by_the_wall_into_inf_with_the_scan_time_tf(filtering):
    node, sent = filtering
    node.on_scan(_scan(100))
    assert node.tf_source == "scan_time"
    assert sent[-1].ranges[0] == float("inf") and sent[-1].ranges[1] == pytest.approx(0.3)


def test_on_scan_passes_the_scan_through_when_the_latest_tf_is_stale(filtering):
    node, sent = filtering
    node.on_scan(_scan(100, 300_000_000))
    assert list(sent[-1].ranges) == pytest.approx([0.085, 0.3])


def test_on_scan_still_filters_with_the_latest_tf_exactly_max_age_away(filtering):
    from pinky_scan_filter.node import MAX_TF_AGE_S
    node, sent = filtering
    node.on_scan(_scan(100, int(MAX_TF_AGE_S * 1e9)))
    assert node.tf_source == "latest"
    assert sent[-1].ranges[0] == float("inf")


def test_on_scan_with_no_stamp_passes_the_scan_through(filtering):
    """stamp 0을 TF2는 "가장 최근"으로 읽는다. 스캔 시각을 모르니 거르지 않는다."""
    node, sent = filtering
    node.on_scan(_scan(0))
    assert list(sent[-1].ranges) == pytest.approx([0.085, 0.3])
