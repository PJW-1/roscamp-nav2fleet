#!/usr/bin/env python3
"""scan N장에서 유효하지 않은 빔(inf/nan/0)의 각도 구간과 가까운 점을 낸다. 읽기만 한다."""
import argparse, json, math, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ap = argparse.ArgumentParser()
ap.add_argument("--ns", default="pode2"); ap.add_argument("--scans", type=int, default=30)
ap.add_argument("--near", type=float, default=0.30); ap.add_argument("--out", default="")
a = ap.parse_args()
rclpy.init(); node = Node("scan_gaps_probe"); scans = []
node.create_subscription(LaserScan, f"/{a.ns}/scan", scans.append, qos_profile_sensor_data)
while rclpy.ok() and len(scans) < a.scans:
    rclpy.spin_once(node, timeout_sec=1.0)
node.destroy_node(); rclpy.shutdown()
s0 = scans[0]; n = len(s0.ranges)
bad = [0] * n; near = [[] for _ in range(n)]; raw = {}
for s in scans:
    for i, r in enumerate(s.ranges):
        if not math.isfinite(r) or r < s.range_min:
            bad[i] += 1
            k = "inf" if math.isinf(r) else ("nan" if math.isnan(r) else f"{r:.3f}")
            raw[k] = raw.get(k, 0) + 1
        elif r <= a.near:
            near[i].append(r)
ang = lambda i: round(math.degrees(s0.angle_min + i * s0.angle_increment), 2)
def runs(flags):
    out, start = [], None
    for i in range(n + 1):
        f = i < n and flags[i]
        if f and start is None: start = i
        if not f and start is not None: out.append((start, i - 1)); start = None
    return out
gaps = [{"from_deg": ang(x), "to_deg": ang(y), "beams": y - x + 1,
         "bad_ratio": round(sum(bad[x:y + 1]) / ((y - x + 1) * len(scans)), 2)}
        for x, y in runs([b >= len(scans) * 0.5 for b in bad])]
nears = []
for x, y in runs([len(v) >= len(scans) * 0.5 for v in near]):
    rs = [r for v in near[x:y + 1] for r in v]
    nears.append({"from_deg": ang(x), "to_deg": ang(y), "beams": y - x + 1,
                  "range_median_m": round(statistics.median(rs), 4), "range_min_m": round(min(rs), 4)})
mins = sorted((min(v), ang(i)) for i, v in enumerate(near) if v)[:5]
out = {"scans": len(scans), "beams": n, "invalid_values": raw, "invalid_runs_over_half": gaps,
       "near_runs_m": a.near, "near_runs": nears, "closest5": [{"m": round(r, 4), "deg": d} for r, d in mins]}
t = json.dumps(out, indent=1); print(t)
if a.out: open(a.out, "w").write(t + "\n")
