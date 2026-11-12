"""스캔 거르개 노드(추가지시서3 7-2). 어느 점을 지울지는 wall_band가 정하고, 여기는 ROS 연결만 한다.

- 받는 것: `scan`(LaserScan), `map_fine`(OccupancyGrid, 1 cm 지도, latched), TF map <- 라이다.
- 내는 것: `scan_filtered`(LaserScan). 지운 빔은 거리를 inf로 둔다. costmap 장애물 층은
  inf_is_valid가 거짓이라 그 빔으로 칠하지도 지우지도 않는다. AMCL은 원래 `scan`을 그대로 읽는다.
- 파라미터: `wall_band_m`(0.05. 지시서 제안 0.04, Gazebo D2 측정으로 고름),
  `strips`("x0,y0,x1,y1; ..." 좁은 띠 사각형).
  `strips`가 비었거나, 지도를 아직 못 받았거나, TF를 못 찾으면 스캔을 그대로 낸다.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .wall_band import WallBand, beam_points, parse_strips

OCCUPIED = 65  # map_server의 occupied_thresh 0.65


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")
        self.band_m = self.declare_parameter("wall_band_m", 0.05).value
        self.strips = parse_strips(self.declare_parameter("strips", "").value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.band = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "map_fine", self.on_map, latched)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, "scan_filtered", 5)
        self.get_logger().info(f"wall_band_m {self.band_m}, strips {self.strips}")

    def on_map(self, msg: OccupancyGrid):
        info = msg.info
        occupied = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width) >= OCCUPIED
        self.band = WallBand(occupied, (info.origin.position.x, info.origin.position.y),
                             info.resolution, self.band_m, self.strips)
        self.get_logger().info(f"map {info.width}x{info.height} at {info.resolution} m")

    def _lidar_pose(self, frame: str):
        try:
            t = self.tf.lookup_transform(self.map_frame, frame, Time()).transform
        except TransformException as e:
            self.get_logger().warn(f"no TF {self.map_frame} <- {frame}: {e}",
                                   throttle_duration_sec=5.0)
            return None
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def on_scan(self, scan: LaserScan):
        pose = self._lidar_pose(scan.header.frame_id) if self.band is not None else None
        if pose is not None and self.strips:
            ranges = np.asarray(scan.ranges, dtype=float)
            xs, ys = beam_points(ranges, scan.angle_min, scan.angle_increment, pose)
            drop = self.band.drops(xs, ys) & np.isfinite(ranges)
            if drop.any():
                ranges[drop] = math.inf
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
