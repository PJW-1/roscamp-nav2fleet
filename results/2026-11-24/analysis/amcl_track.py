#!/usr/bin/env python3
"""bag에서 amcl_pose와 odom을 시간순으로 뽑아 지정 구간을 표로 낸다. 읽기만 한다."""
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

bag, t_from, t_to = sys.argv[1], sys.argv[2], sys.argv[3]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
import datetime
def hms(ns):
    return datetime.datetime.fromtimestamp(ns / 1e9).strftime("%H:%M:%S.%f")[:-4]
def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
last_odom = None
while r.has_next():
    topic, data, t = r.read_next()
    s = hms(t)[:8]
    if topic.endswith("/odom"):
        last_odom = deserialize_message(data, Odometry)
        continue
    if not topic.endswith("/amcl_pose") or not (t_from <= s <= t_to):
        continue
    m = deserialize_message(data, PoseWithCovarianceStamped)
    p, c = m.pose.pose.position, m.pose.covariance
    o = last_odom.pose.pose.position if last_odom else None
    v = last_odom.twist.twist if last_odom else None
    print(f"{hms(t)} amcl {p.x:.3f} {p.y:.3f} {yaw(m.pose.pose.orientation):7.1f}  sd {math.sqrt(c[0]):.3f} {math.sqrt(c[7]):.3f}"
          + (f"  odom {o.x:.3f} {o.y:.3f} v {v.linear.x:.3f} w {v.angular.z:.2f}" if o else ""))
