"""PeerObstacles 노드: 다른 로봇의 보고 자세가 점구름으로 나가는지 본다(ROS 2 rclpy가 있을 때만)."""
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")
pytest.importorskip("sensor_msgs_py")

from fleet_interfaces.msg import RobotState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402

STATE_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


@pytest.fixture
def running():
    rclpy.init()
    from pinky_peer_obstacles.node import PeerObstacles
    node = PeerObstacles(parameter_overrides=[
        Parameter("robot_id", value="pinky1"),
        Parameter("robots", value="pinky1,pinky2,pinky3"),
        Parameter("rate_hz", value=20.0)])
    other = rclpy.create_node("other_side")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(other)
    yield node, other, executor
    executor.shutdown()
    node.destroy_node()
    other.destroy_node()
    rclpy.try_shutdown()


def _state(robot, x, y, yaw=0.0):
    msg = RobotState()
    msg.robot_id = robot
    msg.pose.x, msg.pose.y, msg.pose.theta = x, y, yaw
    return msg


def _spin_until(executor, done, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not done():
        executor.spin_once(timeout_sec=0.05)
    return done()


def _points(cloud):
    return [(float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(cloud, field_names=("x", "y", "z"))]


def test_the_bodies_of_the_other_robots_go_out_as_points_in_the_map_frame(running):
    node, other, executor = running
    clouds = []
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    senders = {name: other.create_publisher(RobotState, f"/{name}/state", STATE_QOS)
               for name in ("pinky1", "pinky2", "pinky3")}

    def seen():
        senders["pinky1"].publish(_state("pinky1", 0.2, 0.2))
        senders["pinky2"].publish(_state("pinky2", 1.0, 0.5))
        senders["pinky3"].publish(_state("pinky3", 1.6, 0.3))
        return bool(clouds) and len(_points(clouds[-1])) > 40

    assert _spin_until(executor, seen)
    cloud = clouds[-1]
    assert cloud.header.frame_id == "map"
    points = _points(cloud)
    near = {name: [p for p in points if math.dist(p[:2], spot) < 0.12]
            for name, spot in (("pinky1", (0.2, 0.2)), ("pinky2", (1.0, 0.5)),
                               ("pinky3", (1.6, 0.3)))}
    assert not near["pinky1"], "my own body is not an obstacle"
    assert len(near["pinky2"]) > 20 and len(near["pinky3"]) > 20
    assert len(near["pinky2"]) + len(near["pinky3"]) == len(points)
    assert all(0.0 < p[2] < 0.5 for p in points), "inside the costmap's height window"


def test_before_any_report_the_cloud_is_empty(running):
    node, other, executor = running
    clouds = []
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    assert _spin_until(executor, lambda: len(clouds) >= 2)
    assert all(not _points(cloud) for cloud in clouds)


def test_a_robot_that_went_silent_stays_where_it_last_was(running):
    """조용해진 로봇은 그 자리에 서 있다(통신이 끊기면 로봇은 선다): 자리를 지우지 않는다."""
    node, other, executor = running
    clouds = []
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    sender = other.create_publisher(RobotState, "/pinky2/state", STATE_QOS)

    def seen():
        sender.publish(_state("pinky2", 1.0, 0.5))
        return bool(clouds) and bool(_points(clouds[-1]))

    assert _spin_until(executor, seen)
    count = len(clouds)
    assert _spin_until(executor, lambda: len(clouds) >= count + 5)
    assert _points(clouds[-1]), "still marked after the reports stopped"


def test_the_sweep_scan_clears_all_around_the_robot(running):
    """peer 층은 이 스캔으로 매번 둘레를 다 지우고 지금 자리만 다시 찍는다: 지나간 자리가 남지 않는다."""
    node, other, executor = running
    scans = []
    other.create_subscription(LaserScan, "peer_sweep", scans.append, 5)
    assert _spin_until(executor, lambda: bool(scans))
    scan = scans[-1]
    assert scan.header.frame_id == "base_footprint"
    assert len(scan.ranges) >= 1440 and all(math.isinf(r) for r in scan.ranges)
    turn = scan.angle_increment * len(scan.ranges)
    assert turn == pytest.approx(2.0 * math.pi, rel=1e-6)
    assert scan.range_max > 3.0, "longer than the layer's raytrace range (3 m)"
    # 3 m 밖 둘레에서도 빔 사이가 local costmap 한 칸(0.025 m)보다 좁다
    assert 3.0 * scan.angle_increment < 0.025


def test_one_process_can_run_a_node_for_every_robot():
    """시뮬레이션은 한 프로세스(sim_stand_in)에 로봇마다 노드를 둔다: 저마다 제 namespace에 남의
    몸만 낸다."""
    rclpy.init()
    from pinky_peer_obstacles.node import make_fleet
    fleet = make_fleet("pinky1,pinky2")
    other = rclpy.create_node("other_side")
    executor = SingleThreadedExecutor()
    for node in (*fleet, other):
        executor.add_node(node)
    try:
        assert [node.get_namespace() for node in fleet] == ["/pinky1", "/pinky2"]
        assert [node.robot_id for node in fleet] == ["pinky1", "pinky2"]
        clouds = {"pinky1": [], "pinky2": []}
        for name, got in clouds.items():
            other.create_subscription(PointCloud2, f"/{name}/peer_obstacles", got.append, 5)
        sender = other.create_publisher(RobotState, "/pinky2/state", STATE_QOS)

        def seen():
            sender.publish(_state("pinky2", 1.0, 0.5))
            return (bool(clouds["pinky1"]) and bool(_points(clouds["pinky1"][-1]))
                    and bool(clouds["pinky2"]))

        assert _spin_until(executor, seen)
        assert all(math.dist(p[:2], (1.0, 0.5)) < 0.12 for p in _points(clouds["pinky1"][-1]))
        assert not _points(clouds["pinky2"][-1]), "pinky2 does not mark itself"
    finally:
        executor.shutdown()
        for node in (*fleet, other):
            node.destroy_node()
        rclpy.try_shutdown()
