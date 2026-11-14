"""VisionStandIn 노드: 상자 표시와 실제 자세로 원뿔 안의 면 점을 내는지 본다(rclpy 필요)."""
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs_py")

from builtin_interfaces.msg import Time  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _running(frame="map"):
    rclpy.init()
    from pinky_avoidance.vision_stand_in import VisionStandIn
    node = VisionStandIn(namespace="/pinky1", parameter_overrides=[
        Parameter("rate_hz", value=20.0), Parameter("frame_id", value=frame)])
    other = rclpy.create_node("other_side")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(other)
    return node, other, executor


@pytest.fixture
def running():
    node, other, executor = _running()
    yield node, other, executor
    executor.shutdown()
    node.destroy_node()
    other.destroy_node()
    rclpy.try_shutdown()


def _spin_until(executor, done, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not done():
        executor.spin_once(timeout_sec=0.05)
    return done()


def _boxes(*centres, deleteall=True):
    out = MarkerArray()
    if deleteall:
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
    for number, (x, y) in enumerate(centres):
        cube = Marker()
        cube.header.frame_id = "map"
        cube.ns, cube.id, cube.type, cube.action = "obstacles", number, Marker.CUBE, Marker.ADD
        cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = x, y, 0.06
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = 0.12
        label = Marker()
        label.header.frame_id = "map"
        label.ns, label.id, label.type = "obstacle_labels", number, Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y = x, y
        out.markers += [cube, label]
    return out


def _truth(x, y, yaw, sec):
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.header.stamp = Time(sec=sec, nanosec=5)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


def _points(cloud):
    return [(float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(cloud, field_names=("x", "y", "z"))
            if abs(float(p[0])) < 100.0]


def _wire(other):
    clouds = []
    other.create_subscription(PointCloud2, "/pinky1/vision_obstacles", clouds.append, 10)
    markers = other.create_publisher(MarkerArray, "/sim/obstacle_markers", MARKER_QOS)
    truth = other.create_publisher(Odometry, "/pinky1/sim/truth", 10)
    return clouds, markers, truth


def test_a_box_ahead_comes_out_as_points_on_its_near_face_with_the_pose_stamp(running):
    node, other, executor = running
    clouds, markers, truth = _wire(other)
    markers.publish(_boxes((0.9, 0.5), (0.2, 0.5)))
    _spin_until(executor, lambda: len(node.boxes) == 2)
    assert len(node.boxes) == 2, "the label markers are not boxes"

    def latest_with_points():
        truth.publish(_truth(0.4, 0.5, 0.0, 7))
        return clouds and _points(clouds[-1])

    assert _spin_until(executor, latest_with_points)
    cloud = clouds[-1]
    assert cloud.header.frame_id == "map"
    assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec) == (7, 5)
    assert [f.name for f in cloud.fields] == ["x", "y", "z"]
    points = _points(cloud)
    assert all(x == pytest.approx(0.84) for x, _, _ in points), "the box behind is not seen"
    assert all(0.0 <= z <= 0.12 + 1e-6 for _, _, z in points)


def test_nothing_is_sent_before_a_pose_arrives_and_an_empty_view_keeps_the_far_point(running):
    node, other, executor = running
    clouds, markers, truth = _wire(other)
    markers.publish(_boxes((0.9, 0.5)))
    _spin_until(executor, lambda: False, seconds=0.5)
    assert clouds == []
    truth.publish(_truth(0.4, 0.5, math.pi, 3))
    assert _spin_until(executor, lambda: clouds)
    assert _points(clouds[-1]) == []
    assert len(list(point_cloud2.read_points(clouds[-1]))) == 1, "never an empty cloud"


def test_deleteall_clears_the_boxes(running):
    node, other, executor = running
    _, markers, _ = _wire(other)
    markers.publish(_boxes((0.9, 0.5)))
    assert _spin_until(executor, lambda: len(node.boxes) == 1)
    markers.publish(_boxes())
    assert _spin_until(executor, lambda: not node.boxes)


def test_the_robot_frame_option_puts_the_points_relative_to_the_pose():
    node, other, executor = _running(frame="base_footprint")
    try:
        clouds, markers, truth = _wire(other)
        markers.publish(_boxes((0.9, 0.5)))
        _spin_until(executor, lambda: len(node.boxes) == 1)

        def latest_with_points():
            truth.publish(_truth(0.4, 0.5, 0.0, 2))
            return clouds and _points(clouds[-1])

        assert _spin_until(executor, latest_with_points)
        assert clouds[-1].header.frame_id == "base_footprint"
        assert all(x == pytest.approx(0.44) for x, _, _ in _points(clouds[-1]))
    finally:
        executor.shutdown()
        node.destroy_node()
        other.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize("name, value", [("frame_id", "odom"), ("rate_hz", 2.0),
                                         ("fov_deg", 200.0)])
def test_bad_parameters_stop_the_node(name, value):
    rclpy.init()
    try:
        from pinky_avoidance.vision_stand_in import VisionStandIn
        with pytest.raises(ValueError, match=name):
            VisionStandIn(parameter_overrides=[Parameter(name, value=value)])
    finally:
        rclpy.try_shutdown()
