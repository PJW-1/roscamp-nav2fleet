# ---- 맵을 받아 스캔의 벽 인접 점을 걸러 내보내는 노드 ----
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .wall_band import WallBand, filter_ranges, origin_yaw, parse_strips

# ---- 상수 ----
OCCUPIED = 65
TF_WAIT = Duration(seconds=0.05)
MAX_TF_AGE_S = 0.1


# ---- ScanFilter 노드 생성과 정리 ----
class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")
        self.band_m = self.declare_parameter("wall_band_m", 0.05).value
        self.strips = parse_strips(self.declare_parameter("strips", "").value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.band = None
        self.tf_source = ""
        self.tf = Buffer()
        self.tf_node = rclpy.create_node(
            f"{self.get_name()}_tf", namespace=self.get_namespace(), context=self.context,
            use_global_arguments=False,
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
            parameter_overrides=[self.get_parameter("use_sim_time")])
        self.listener = TransformListener(self.tf, self.tf_node, spin_thread=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "map_fine", self.on_map, latched)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, "scan_filtered", 5)
        self.get_logger().info(f"wall_band_m {self.band_m}, strips {self.strips}")

    def destroy_node(self):
        self.listener.executor.shutdown()
        self.listener.dedicated_listener_thread.join(timeout=2.0)
        self.tf_node.destroy_node()
        super().destroy_node()

    # ---- 맵 수신과 벽 밴드 생성 ----
    def on_map(self, msg: OccupancyGrid):
        info = msg.info
        o = info.origin.orientation
        if abs(origin_yaw(o.z, o.w)) > 1e-6:
            self.band = None
            self.get_logger().warn("map_fine origin is rotated; scans pass through unfiltered",
                                   once=True)
            return
        occupied = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width) >= OCCUPIED
        self.band = WallBand(occupied, (info.origin.position.x, info.origin.position.y),
                             info.resolution, self.band_m, self.strips)
        self.get_logger().info(f"map {info.width}x{info.height} at {info.resolution} m")

    # ---- 라이다 위치를 TF에서 조회 ----
    def _lidar_pose(self, frame: str, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            self.get_logger().warn("scan has no stamp; not filtering", throttle_duration_sec=5.0)
            self.tf_source = ""
            return None
        try:
            t = self.tf.lookup_transform(self.map_frame, frame, Time.from_msg(stamp),
                                         timeout=TF_WAIT).transform
        except TransformException:
            try:
                latest = self.tf.lookup_transform(self.map_frame, frame, Time())
            except TransformException as e:
                self.get_logger().warn(f"no TF {self.map_frame} <- {frame}: {e}",
                                       throttle_duration_sec=5.0)
                return None
            gap = Time.from_msg(latest.header.stamp) - Time.from_msg(stamp)
            age = abs(gap.nanoseconds) * 1e-9
            if age > MAX_TF_AGE_S:
                self.get_logger().warn(f"latest TF is {age:.2f} s from the scan; not filtering",
                                       throttle_duration_sec=5.0)
                return None
            self.get_logger().warn("no TF at the scan time; using the latest",
                                   throttle_duration_sec=5.0)
            t = latest.transform
            self.tf_source = "latest"
        else:
            self.tf_source = "scan_time"
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    # ---- 스캔 필터링과 발행 ----
    def on_scan(self, scan: LaserScan):
        pose = (self._lidar_pose(scan.header.frame_id, scan.header.stamp)
                if self.band is not None and self.strips else None)
        if pose is not None:
            ranges = filter_ranges(self.band, scan.ranges, scan.angle_min, scan.angle_increment,
                                   scan.range_min, scan.range_max, pose)
            scan.ranges = ranges.tolist()
        self.pub.publish(scan)


# ---- 노드 실행 진입점 ----
def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
