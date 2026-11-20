#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time

GAZEBO_SCAN = {"beams": 640, "angle_span_deg": 360.0, "range_min": 0.05, "range_max": 12.0,
               "rate_hz": 10.0}


# ---- 요약 ----
def summarize(stamps, msgs):
    last = msgs[-1]
    finite = [r for m in msgs for r in m["ranges"] if math.isfinite(r)]
    span = stamps[-1] - stamps[0]
    return {
        "messages": len(msgs),
        "beams": sorted({len(m["ranges"]) for m in msgs}),
        "angle_min_deg": math.degrees(last["angle_min"]),
        "angle_max_deg": math.degrees(last["angle_max"]),
        "angle_increment_deg": math.degrees(last["angle_increment"]),
        "angle_span_deg": math.degrees(last["angle_increment"]) * len(last["ranges"]),
        "range_min": last["range_min"],
        "range_max": last["range_max"],
        "rate_hz": (len(msgs) - 1) / span if span > 0 else math.nan,
        "finite_fraction": len(finite) / max(1, sum(len(m["ranges"]) for m in msgs)),
        "largest_seen_m": max(finite) if finite else math.nan,
        "scan_time_s": last["scan_time"],
        "frame_id": last["frame_id"],
    }


def main(argv):
    p = argparse.ArgumentParser(description="record the real scan's beam count, angles, ranges,"
                                            " rate")
    p.add_argument("--ns", default="pode1")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--out", default="", help="also write the summary as JSON")
    args = p.parse_args(argv)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("scan_info")
    stamps, msgs = [], []

    def on_scan(m):
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        msgs.append({"ranges": list(m.ranges), "angle_min": m.angle_min,
                     "angle_max": m.angle_max, "angle_increment": m.angle_increment,
                     "range_min": m.range_min, "range_max": m.range_max,
                     "scan_time": m.scan_time, "frame_id": m.header.frame_id})

    topic = f"/{args.ns}/scan"
    node.create_subscription(LaserScan, topic, on_scan, qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    if len(msgs) < 2:
        print(f"fewer than 2 messages on {topic} in {args.seconds} s")
        return 1
    info = summarize(stamps, msgs)
    info["topic"] = topic
    info["gazebo_reference"] = GAZEBO_SCAN
    text = json.dumps(info, indent=1)
    print(text)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
