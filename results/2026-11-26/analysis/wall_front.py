#!/usr/bin/env python3
"""정면 벽까지 거리: 정면 빔 중앙값과, 정면 ±W도 점에 직선을 맞춘 수직 거리·기울기를 낸다. 읽기만 한다."""
import argparse, json, math, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ap = argparse.ArgumentParser()
ap.add_argument("--ns", default="pode2"); ap.add_argument("--scans", type=int, default=50)
ap.add_argument("--front", type=float, default=180.0, help="로봇 정면의 라이다 각도 deg")
ap.add_argument("--width", type=float, default=15.0); ap.add_argument("--label", default="")
ap.add_argument("--out", default="")
a = ap.parse_args()
rclpy.init(); node = Node("wall_front_probe"); scans = []
node.create_subscription(LaserScan, f"/{a.ns}/scan", scans.append, qos_profile_sensor_data)
while rclpy.ok() and len(scans) < a.scans:
    rclpy.spin_once(node, timeout_sec=1.0)
node.destroy_node(); rclpy.shutdown()
beam0, xs, ys, per_scan = [], [], [], []
for s in scans:
    sx, sy = [], []
    for i, r in enumerate(s.ranges):
        if not math.isfinite(r) or r < s.range_min:
            continue
        ang = s.angle_min + i * s.angle_increment
        d = math.degrees(math.remainder(ang - math.radians(a.front), math.tau))
        if abs(d) <= 0.26:
            beam0.append(r)
        if abs(d) <= a.width:
            # 로봇 정면을 +x로 둔 좌표
            t = math.radians(d); sx.append(r * math.cos(t)); sy.append(r * math.sin(t))
    xs += sx; ys += sy
    if len(sx) > 5:
        per_scan.append(statistics.median(sx))
# x = c + m*y 로 맞춘다(벽이 정면에 거의 수직)
my, mx = statistics.fmean(ys), statistics.fmean(xs)
syy = sum((y - my) ** 2 for y in ys); sxy = sum((y - my) * (x - mx) for x, y in zip(xs, ys))
m = sxy / syy; c = mx - m * my
perp = c / math.sqrt(1 + m * m); yaw = math.degrees(math.atan(m))
res = [x - (c + m * y) for x, y in zip(xs, ys)]
out = {"label": a.label, "scans": len(scans), "front_deg": a.front,
       "beam0_median_m": round(statistics.median(beam0), 4) if beam0 else None,
       "beam0_stdev_m": round(statistics.pstdev(beam0), 4) if beam0 else None,
       "line_perp_m": round(perp, 4), "line_x_at_front_m": round(c, 4),
       "wall_tilt_deg": round(yaw, 2), "fit_points": len(xs),
       "fit_resid_rms_m": round(math.sqrt(statistics.fmean([e * e for e in res])), 4)}
t = json.dumps(out, indent=1); print(t)
if a.out:
    with open(a.out, "a") as f: f.write(json.dumps(out) + "\n")
