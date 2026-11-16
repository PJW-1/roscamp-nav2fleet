# ---- 시뮬레이션 전용 임시 비전 대역: 상자 표시와 실제 자세로 비전 장애물 점구름을 낸다 ----
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .vision_cone import box_from_marker, make_cone, seen_points, to_body, yaw_of

MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
FRAMES = ("map", "base_footprint")
MAP_FRAMES = ("map", "/map")
POINT_Z_M = 0.05
FAR_POINT = (1000.0, 1000.0, POINT_Z_M)


# ---- 노드 ----
class VisionStandIn(Node):
    def __init__(self, **kwargs):
        super().__init__("vision_stand_in", **kwargs)
        declare = self.declare_parameter
        self.robot_id = declare("robot_id", "").value or self.get_namespace().strip("/")
        self.cone = make_cone(float(declare("fov_deg", 70.0).value),
                              float(declare("range_m", 1.0).value))
        self.step_m = float(declare("step_m", 0.02).value)
        self.frame_id = declare("frame_id", "base_footprint").value
        if self.frame_id not in FRAMES:
            raise ValueError(f"frame_id must be one of {FRAMES}, got {self.frame_id!r}")
        markers_topic = declare("markers_topic", "/sim/obstacle_markers").value
        pose_topic = declare("pose_topic", "").value or f"/{self.robot_id}/sim/truth"
        rate_hz = float(declare("rate_hz", 10.0).value)
        if not rate_hz >= 5.0:
            raise ValueError(f"rate_hz must be 5 or more, got {rate_hz!r}")
        self.boxes = {}
        self.pose = None
        self.stamp = None
        self.fresh = False
        self.create_subscription(MarkerArray, markers_topic, self.on_markers, MARKER_QOS)
        self.create_subscription(Odometry, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "vision_obstacles", 5)
        self.create_timer(1.0 / rate_hz, self.publish)
        self.get_logger().info(f"{self.robot_id}: boxes from {markers_topic}, pose from "
                               f"{pose_topic}, {self.frame_id} frame")

    def on_markers(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                self.boxes.clear()
                continue
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETE:
                self.boxes.pop(key, None)
                continue
            if marker.type != Marker.CUBE or marker.header.frame_id not in MAP_FRAMES:
                continue
            p, q, s = marker.pose.position, marker.pose.orientation, marker.scale
            self.boxes[key] = box_from_marker(p.x, p.y, p.z, q.x, q.y, q.z, q.w, s.x, s.y, s.z)

    def on_pose(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_of(q.x, q.y, q.z, q.w))
        self.stamp = msg.header.stamp
        self.fresh = True

    def publish(self):
        if not self.fresh:
            return
        self.fresh = False
        points = seen_points(list(self.boxes.values()), self.pose, self.cone, self.step_m)
        if self.frame_id != "map":
            points = to_body(points, self.pose)
        points.append(FAR_POINT)
        header = Header(stamp=self.stamp, frame_id=self.frame_id)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))


# ---- 실행 ----
def main():
    rclpy.init()
    node = VisionStandIn()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
