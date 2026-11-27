#!/usr/bin/env python3
"""지금 map→base_footprint, 라이다 자리, 마지막 amcl_pose를 찍는다. 쓰기: pose_now.py pode2"""
import math
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


ns = sys.argv[1] if len(sys.argv) > 1 else "pode2"
rclpy.init()
node = rclpy.create_node("pose_now", cli_args=["--ros-args", "-r", f"/tf:=/{ns}/tf",
                                                "-r", f"/tf_static:=/{ns}/tf_static"])
buf = Buffer()
TransformListener(buf, node)
amcl = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(PoseWithCovarianceStamped, f"/{ns}/amcl_pose", amcl.append, qos)
tr = None
end = time.monotonic() + 10
while time.monotonic() < end and not (tr and amcl):
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tr = buf.lookup_transform("map", "base_footprint", Time())
    except Exception:  # noqa: BLE001
        pass
print("wall", datetime.now().strftime("%H:%M:%S"))
if tr:
    t = tr.transform.translation
    th = yaw(tr.transform.rotation)
    print(f"map->base_footprint x {t.x:.3f} y {t.y:.3f} yaw {math.degrees(th):.1f} "
          f"(stamp {tr.header.stamp.sec}.{tr.header.stamp.nanosec:09d})")
    lasers = [f for f in buf.all_frames_as_yaml().split("\n") if "laser" in f or "lidar" in f]
    for line in lasers[:2]:
        name = line.split(":")[0].strip().strip("'")
        try:
            lt = buf.lookup_transform("base_footprint", name, Time()).transform.translation
            lw = buf.lookup_transform("map", name, Time()).transform.translation
            print(f"{name}: base에서 ({lt.x:+.3f}, {lt.y:+.3f}), map에서 x {lw.x:.3f} y {lw.y:.3f}")
        except Exception as e:  # noqa: BLE001
            print(f"{name}: {e}")
else:
    print("map->base_footprint 없음")
if amcl:
    m = amcl[-1]
    p, c = m.pose.pose, m.pose.covariance
    print(f"amcl_pose x {p.position.x:.3f} y {p.position.y:.3f} yaw {math.degrees(yaw(p.orientation)):.1f} "
          f"std {math.sqrt(max(c[0], 0)) * 100:.1f}/{math.sqrt(max(c[7], 0)) * 100:.1f} cm "
          f"(stamp {m.header.stamp.sec}.{m.header.stamp.nanosec:09d})")
else:
    print("amcl_pose 없음")
node.destroy_node()
rclpy.try_shutdown()
