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


def _state(robot, x, y, yaw=0.0, state="MOVING"):
    msg = RobotState()
    msg.robot_id = robot
    msg.state = state
    msg.pose.x, msg.pose.y, msg.pose.theta = x, y, yaw
    return msg


def _spin_until(executor, done, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end and not done():
        executor.spin_once(timeout_sec=0.05)
    return done()


def _all_points(cloud):
    return [(float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(cloud, field_names=("x", "y", "z"))]


def _points(cloud):
    """몸을 찍는 점들(늘 함께 나가는 먼 점 하나는 뺀다)."""
    return [p for p in _all_points(cloud) if math.hypot(p[0], p[1]) < 100.0]


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


def test_before_any_report_no_body_is_marked_but_the_cloud_is_never_empty(running):
    """Nav2의 점구름 반복자는 빈 data에서 front()를 부른다: 빈 구름을 보내지 않는다. 늘 먼 점 하나를
    함께 낸다. costmap은 obstacle_max_range(10 m) 밖이라 찍지 않는다."""
    node, other, executor = running
    clouds = []
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    assert _spin_until(executor, lambda: len(clouds) >= 2)
    for cloud in clouds:
        assert not _points(cloud)
        [far] = _all_points(cloud)
        assert math.hypot(far[0], far[1]) >= 1000.0


@pytest.mark.parametrize("report", [_state("pinky2", 0.0, 0.0, 0.0, state="LOCALIZING"),
                                    _state("pinky2", 0.0, 0.0, 0.0, state="IDLE"),
                                    _state("pinky2", 0.7, 0.2, 0.0, state="LOCALIZING")])
def test_a_report_without_a_known_pose_marks_nothing(running, report):
    """pinky_fsm은 위치를 모르거나 LOCALIZING이면 자세 칸을 0, 0, 0으로 둔다: 그것을 map 원점의
    몸으로 찍지 않는다."""
    node, other, executor = running
    clouds = []
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    sender = other.create_publisher(RobotState, "/pinky2/state", STATE_QOS)

    def sent():
        sender.publish(report)
        return len(clouds) >= 6

    assert _spin_until(executor, sent)
    assert not _points(clouds[-1])


def test_a_robot_that_loses_its_pose_stays_where_it_last_was(running):
    node, other, executor = running
    clouds = []
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    sender = other.create_publisher(RobotState, "/pinky2/state", STATE_QOS)

    def seen():
        sender.publish(_state("pinky2", 1.0, 0.5))
        return bool(clouds) and bool(_points(clouds[-1]))

    assert _spin_until(executor, seen)
    count = len(clouds)

    def lost():
        sender.publish(_state("pinky2", 0.0, 0.0, 0.0, state="LOCALIZING"))
        return len(clouds) >= count + 5

    assert _spin_until(executor, lost)
    points = _points(clouds[-1])
    assert points and all(math.dist(p[:2], (1.0, 0.5)) < 0.12 for p in points)


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


def test_the_sweep_is_stamped_a_little_in_the_past(running):
    """costmap은 스캔 시각의 TF(odom <- base_footprint)로 빔을 놓는다. 지금 시각의 TF는 아직 오지
    않았을 수 있어 조금 지난 시각을 적는다(진짜 라이다 스캔도 받을 때는 지난 시각이다)."""
    node, other, executor = running
    scans, clouds = [], []
    other.create_subscription(LaserScan, "peer_sweep", scans.append, 5)
    other.create_subscription(PointCloud2, "peer_obstacles", clouds.append, 5)
    assert _spin_until(executor, lambda: bool(scans) and bool(clouds))

    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    by_stamp = {round(seconds(c.header.stamp), 6) for c in clouds}
    assert any(round(seconds(s.header.stamp) + 0.2, 6) in by_stamp for s in scans)


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
