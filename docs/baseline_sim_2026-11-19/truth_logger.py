"""Gazebo 실제 위치(/podeN/sim/truth)를 CSV로 남긴다.

    python3 truth_logger.py OUT.csv DOMAIN [로봇 수]
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

out = open(sys.argv[1], "a", buffering=1)
if out.tell() == 0:
    out.write("wall,sim_t,robot,x,y,yaw\n")
rclpy.init(domain_id=int(sys.argv[2]))
node = rclpy.create_node("truth_logger_scratch")
last = {}


def cb(robot):
    def f(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - last.get(robot, -1.0) < 0.05 and t >= last.get(robot, -1.0):
            return
        last[robot] = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        out.write(f"{time.time():.2f},{t:.3f},{robot},{p.x:.4f},{p.y:.4f},{yaw:.4f}\n")
    return f


for i in range(1, int(sys.argv[3]) + 1 if len(sys.argv) > 3 else 4):
    node.create_subscription(Odometry, f"/pode{i}/sim/truth", cb(f"pode{i}"), qos_profile_sensor_data)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
