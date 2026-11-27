#!/usr/bin/env python3
"""scan N장에서 가까운 점(기둥 등)을 각도로 묶어 각도·거리를 낸다. 읽기만 한다."""
import argparse
import json
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clusters(hits, gap_deg):
    """(각도 deg, 거리) 목록을 각도 간격으로 묶는다."""
    groups, cur = [], []
    for a, r in sorted(hits):
        if cur and a - cur[-1][0] > gap_deg:
            groups.append(cur)
            cur = []
        cur.append((a, r))
    if cur:
        groups.append(cur)
    # -180/180 경계에서 이어지는 묶음을 합친다
    if len(groups) > 1 and groups[0][0][0] + 360 - groups[-1][-1][0] <= gap_deg:
        groups[0] = [(a - 360, r) for a, r in groups[-1]] + groups[0]
        groups.pop()
    return groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="pode1")
    ap.add_argument("--scans", type=int, default=20)
    ap.add_argument("--max", type=float, default=0.15)
    ap.add_argument("--front", type=float, default=0.0, help="정면 값을 볼 각도 deg")
    ap.add_argument("--front-width", type=float, default=2.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    rclpy.init()
    node = Node("near_points_probe")
    scans = []
    node.create_subscription(LaserScan, f"/{args.ns}/scan", scans.append, qos_profile_sensor_data)
    while rclpy.ok() and len(scans) < args.scans:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

    hits, front = [], []
    for s in scans:
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min:
                continue
            a = math.degrees(s.angle_min + i * s.angle_increment)
            if r <= args.max:
                hits.append((a, r))
            d = (a - args.front + 180) % 360 - 180
            if abs(d) <= args.front_width:
                front.append((d, r))
    out = {"scans": len(scans), "max_m": args.max, "clusters": []}
    for g in clusters(hits, gap_deg=3.0):
        angles, ranges = [a for a, _ in g], [r for _, r in g]
        out["clusters"].append({
            "angle_from_deg": round(min(angles), 2), "angle_to_deg": round(max(angles), 2),
            "angle_mid_deg": round(statistics.median(angles), 2),
            "range_median_m": round(statistics.median(ranges), 4),
            "range_min_m": round(min(ranges), 4), "range_max_m": round(max(ranges), 4),
            "hits": len(g), "hits_per_scan": round(len(g) / max(1, len(scans)), 1)})
    if front:
        rs = [r for _, r in front]
        zero = [r for d, r in front if abs(d) <= 0.26]
        out["front"] = {"angle_deg": args.front, "width_deg": args.front_width,
                        "median_m": round(statistics.median(rs), 4),
                        "beam0_median_m": round(statistics.median(zero), 4) if zero else None,
                        "stdev_m": round(statistics.pstdev(rs), 4), "samples": len(rs)}
    text = json.dumps(out, indent=1, ensure_ascii=False)
    print(text)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
