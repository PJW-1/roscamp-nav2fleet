#!/usr/bin/env python3
"""scan으로 로봇이 P-1(서쪽 벽 약 0.12, 남쪽 벽 약 0.08, 북쪽 방향)에 3초 이상 가만히 놓였는지 본다. 읽기만 한다."""
import math, sys, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ns, limit = sys.argv[1], float(sys.argv[2])
rclpy.init(); node = Node("at_p1_probe"); last = {}
node.create_subscription(LaserScan, f"/{ns}/scan", lambda m: last.update(m=m), qos_profile_sensor_data)

def at(m, deg):
    out = []
    for i, r in enumerate(m.ranges):
        a = math.degrees(m.angle_min + i * m.angle_increment)
        if abs((a - deg + 180) % 360 - 180) <= 2 and math.isfinite(r) and r >= m.range_min:
            out.append(r)
    return statistics.median(out) if out else float("nan")

t0, ok_since = time.time(), None
while time.time() - t0 < limit:
    last.clear()
    while "m" not in last:
        rclpy.spin_once(node, timeout_sec=0.2)
    s, w = at(last["m"], 0.0), at(last["m"], -90.0)
    good = 0.05 <= s <= 0.12 and 0.09 <= w <= 0.16
    ok_since = (ok_since or time.time()) if good else None
    if ok_since and time.time() - ok_since >= 3.0:
        print(f"at P-1: south {s:.3f} west {w:.3f}"); sys.exit(0)
    time.sleep(0.3)
print(f"not at P-1 within {limit:g} s: south {s:.3f} west {w:.3f}"); sys.exit(1)
