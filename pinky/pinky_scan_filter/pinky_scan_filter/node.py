"""스캔 거르개 노드(추가지시서3 7-2). 어느 점을 지울지는 wall_band가 정하고, 여기는 ROS 연결만 한다.

- 받는 것: `scan`(LaserScan), `map_fine`(OccupancyGrid, 1 cm 지도, latched), TF map <- 라이다.
- 내는 것: `scan_filtered`(LaserScan). 지운 빔은 거리를 inf로 둔다. costmap 장애물 층은
  inf_is_valid가 거짓이라 그 빔으로 칠하지도 지우지도 않는다. AMCL은 원래 `scan`을 그대로 읽는다.
- 파라미터: `wall_band_m`(0.05. 지시서 제안 0.04, Gazebo D2 측정으로 고름),
  `strips`("x0,y0,x1,y1; ..." 좁은 띠 사각형).
  `strips`가 비었거나, 지도를 아직 못 받았거나, 지도 원점이 돌아가 있거나(yaw가 0이 아님), TF를 못
  찾으면 스캔을 그대로 낸다. 거르지 않은 스캔은 더 자주 서게 할 뿐이라 안전한 쪽이다.
- 점은 스캔 시각의 TF(map <- 라이다, 0.05 s 기다림)로 놓는다. 그 시각의 TF가 없을 때만 가장
  최근 TF를 쓰고, 그것도 스캔 시각과 MAX_TF_AGE_S보다 멀면 거르지 않는다. stamp가 0인 스캔도
  거르지 않는다.
- TF는 도우미 노드(scan_filter_tf)가 따로 도는 스레드로 받는다. 거르개 노드를 TransformListener에
  넘기면 main의 executor와 그 노드를 다퉈(rclpy Node.executor) 기다리는 동안 버퍼가 차지 않는다.
"""
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

OCCUPIED = 65  # map_server의 occupied_thresh 0.65
TF_WAIT = Duration(seconds=0.05)  # 스캔 시각의 TF를 이만큼 기다린다
# 최신 TF가 스캔 시각과 이보다 멀면 거르지 않는다: 0.1 s면 제자리 회전(최대 약 1 rad/s)에서도 1 m 앞
# 점이 0.1 m 안쪽으로만 옮겨진다. 더 멀면 실제 물체 점이 벽 띠로 옮겨져 지워질 수 있다
MAX_TF_AGE_S = 0.1


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")
        self.band_m = self.declare_parameter("wall_band_m", 0.05).value
        self.strips = parse_strips(self.declare_parameter("strips", "").value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.band = None
        self.tf_source = ""  # 마지막 자세가 어느 TF에서 왔나: scan_time, latest(시험과 로그용)
        self.tf = Buffer()
        # TF는 도우미 노드가 따로 도는 스레드로 받는다: on_scan이 스캔 시각의 TF를 기다리는 동안에도
        # 버퍼가 찬다. 프로세스 인자(launch의 __node 이름 바꾸기)는 받지 않아 이름이 겹치지 않고,
        # 네임스페이스, use_sim_time, /tf remap(nav2.launch.py와 같은 것)은 거르개 노드와 같게 준다
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

    def _lidar_pose(self, frame: str, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            # TF2는 시각 0을 "가장 최근"으로 읽는다. 스캔 시각을 모르니 거르지 않는다
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

    def on_scan(self, scan: LaserScan):
        pose = (self._lidar_pose(scan.header.frame_id, scan.header.stamp)
                if self.band is not None and self.strips else None)
        if pose is not None:
            ranges = filter_ranges(self.band, scan.ranges, scan.angle_min, scan.angle_increment,
                                   scan.range_min, scan.range_max, pose)
            scan.ranges = ranges.tolist()
        self.pub.publish(scan)


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
