#!/usr/bin/env python3
"""실물 bag으로 AMCL을 PC에서 다시 돌려 지도끼리 비교한다.

prep: 원본 bag에서 scan, odom, tf(live map → odom만 빼고), tf_static만 남긴 bag을 만든다.
probe: 재생 중 정한 시각의 map → base_footprint를 읽어 JSON으로 남긴다.
score: 원본 bag의 live AMCL과 재생 결과를 줄자 정답과 비교해 표로 낸다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path


# ---- 공통: 2D 자세 계산 ----
def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _compose(a, b):
    """a ∘ b. 자세는 (x, y, yaw)."""
    x, y, t = a
    return (x + math.cos(t) * b[0] - math.sin(t) * b[1],
            y + math.sin(t) * b[0] + math.cos(t) * b[1], t + b[2])


def _reader(bag: str, topics: list[str]):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


# ---- prep ----
def prep(args) -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    ns = f"/{args.ns}"
    fix = args.range_scale != 1.0 or args.range_offset != 0.0
    keep = [f"{ns}/scan", f"{ns}/odom", f"{ns}/tf", f"{ns}/tf_static"]
    r = _reader(args.bag, [])
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=args.out, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for meta in r.get_all_topics_and_types():
        if meta.name in keep:
            w.create_topic(meta)
    r.set_filter(rosbag2_py.StorageFilter(topics=keep))
    dropped = 0
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == f"{ns}/tf":
            msg = deserialize_message(data, TFMessage)
            kept = [tr for tr in msg.transforms
                    if (tr.header.frame_id, tr.child_frame_id) != (args.map_frame, args.odom_frame)]
            dropped += len(msg.transforms) - len(kept)
            if not kept:
                continue
            msg.transforms = kept
            data = serialize_message(msg)
        elif fix and topic == f"{ns}/scan":
            # 읽은 값 = scale × 실제 + offset 을 거꾸로 풀어 실제 거리로 바꾼다(inf, nan은 그대로)
            msg = deserialize_message(data, LaserScan)
            msg.ranges = [(v - args.range_offset) / args.range_scale if math.isfinite(v) and v > 0
                          else v for v in msg.ranges]
            data = serialize_message(msg)
        w.write(topic, data, t)
    del w
    note = f", scan 거리 보정 scale {args.range_scale} offset {args.range_offset}" if fix else ""
    print(f"[prep] {args.out}: live {args.map_frame} → {args.odom_frame} {dropped}개를 뺐다{note}")


# ---- probe ----
def probe(args) -> None:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.parameter import Parameter
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
    ns = f"/{args.ns}"
    rclpy.init()
    node = rclpy.create_node(
        "amcl_replay_probe", parameter_overrides=[
            Parameter("use_sim_time", value=True)],
        cli_args=["--ros-args", "-r", f"/tf:={ns}/tf", "-r", f"/tf_static:={ns}/tf_static"])
    buf = Buffer()
    TransformListener(buf, node)
    # tf_static은 bag 맨 앞에 한 번뿐이라 구간 중간부터 틀면 빠진다: 직접 낸다
    r = _reader(args.bag, [f"{ns}/tf_static"])
    statics = []
    while r.has_next():
        statics += deserialize_message(r.read_next()[1], TFMessage).transforms
    static_pub = StaticTransformBroadcaster(node)
    static_pub.sendTransform(statics)
    track: list[list[float]] = []

    def on_pose(m):
        p, c = m.pose.pose, m.pose.covariance
        track.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x,
                      p.position.y, math.degrees(_yaw(p.orientation)),
                      math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0))])
    node.create_subscription(PoseWithCovarianceStamped, f"{ns}/amcl_pose", on_pose, 10)

    pending = sorted(float(t) for t in args.times.split(","))
    out = {"times": {}, "track": track, "map": args.map_label}
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and pending and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = node.get_clock().now().nanoseconds * 1e-9
        if pending and now >= pending[0] + args.settle:
            t = pending.pop(0)
            try:
                tr = buf.lookup_transform("map", "base_footprint", Time(seconds=t))
                q = tr.transform.rotation
                out["times"][f"{t:.3f}"] = [tr.transform.translation.x,
                                            tr.transform.translation.y,
                                            math.degrees(_yaw(q))]
            except Exception as e:  # noqa: BLE001 - 못 읽은 시각은 이유를 남긴다
                out["times"][f"{t:.3f}"] = str(e)
    failed = [t for t, v in out["times"].items() if not isinstance(v, list)]
    if pending or failed:
        out["error"] = f"시간 안에 못 읽은 시각 {pending}, 조회 실패 {failed}"
    Path(args.out).write_text(json.dumps(out))
    node.destroy_node()
    rclpy.try_shutdown()
    print(f"[probe] {args.out}: {out['times']}", flush=True)
    sys.exit(1 if pending or failed else 0)


# ---- score ----
def _live(bag: str, ns: str, times: list[float]) -> dict[float, tuple]:
    """원본 bag의 live map→odom과 odom→base_footprint를 이어 정한 시각의 자세를 구한다."""
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    m2o: list[tuple] = []
    o2b: list[tuple] = []
    r = _reader(bag, [f"/{ns}/tf"])
    while r.has_next():
        _, data, _ = r.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            pose = (tr.transform.translation.x, tr.transform.translation.y,
                    _yaw(tr.transform.rotation))
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                m2o.append((s, pose))
            elif tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
                o2b.append((s, pose))
    out = {}
    for t in times:
        # probe의 tf 조회와 같게: 그 시각 이하에서 마지막 map→odom(AMCL은 tolerance만큼 앞당긴 시각으로 낸다)
        a = [p for s, p in m2o if s <= t]
        b = min(o2b, key=lambda sp: abs(sp[0] - t))
        out[t] = _compose(a[-1], b[1]) if a else None
    return out


def score(args) -> None:
    truth = json.loads(Path(args.truth).read_text())
    times = [float(v["t"]) for v in truth.values()]
    rows = []
    live = _live(args.bag, args.ns, times)
    for name, v in truth.items():
        p = live[float(v["t"])]
        rows.append(("live", "원래 AMCL", name, p and (p[0], p[1], math.degrees(p[2])), v))
    for f in sorted(args.runs):
        d = json.loads(Path(f).read_text())
        for name, v in truth.items():
            key = f"{float(v['t']):.3f}"
            if key not in d["times"]:
                continue  # 이 판이 틀지 않은 구간
            p = d["times"][key]
            rows.append((d["map"], Path(f).stem, name, p if isinstance(p, list) else None, v))
    print("| 지도 | 회차 | 자리 | AMCL x, y, yaw | 정답 x, y | dx, dy (cm) | 거리 (cm) |")
    print("|---|---|---|---|---|---|---|")
    summary: dict[tuple, list[float]] = {}
    for m, run, name, p, v in rows:
        if p is None:
            print(f"| {m} | {run} | {name} | 못 읽음 | | | |")
            continue
        dx, dy = (p[0] - v["x"]) * 100, (p[1] - v["y"]) * 100
        e = math.hypot(dx, dy)
        summary.setdefault((m, name), []).append(e)
        print(f"| {m} | {run} | {name} | {p[0]:.3f}, {p[1]:.3f}, {p[2]:.1f}° |"
              f" {v['x']:.3f}, {v['y']:.3f} | {dx:+.1f}, {dy:+.1f} | {e:.1f} |")
    print()
    print("| 지도 | 자리 | 회차 | 평균 거리 (cm) | 최대 (cm) |")
    print("|---|---|---|---|---|")
    for (m, name), es in summary.items():
        print(f"| {m} | {name} | {len(es)} | {sum(es) / len(es):.1f} | {max(es):.1f} |")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("prep")
    p.add_argument("bag")
    p.add_argument("out")
    p.add_argument("--ns", default="pode2")
    p.add_argument("--map-frame", default="map")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--range-scale", type=float, default=1.0,
                   help="읽은 값 = scale × 실제 + offset 일 때 scale (pode2 11-24: 0.958)")
    p.add_argument("--range-offset", type=float, default=0.0, help="같은 식의 offset (m)")
    p.set_defaults(fn=prep)
    p = sub.add_parser("probe")
    p.add_argument("--ns", default="pode2")
    p.add_argument("--bag", required=True, help="tf_static을 읽을 bag")
    p.add_argument("--times", required=True, help="bag 시각(초, 쉼표로)")
    p.add_argument("--map-label", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--settle", type=float, default=1.5, help="시각을 지나고 기다릴 초")
    p.add_argument("--timeout", type=float, default=900.0, help="벽시계 초")
    p.set_defaults(fn=probe)
    p = sub.add_parser("score")
    p.add_argument("--bag", required=True, help="원본 bag(live AMCL 포함)")
    p.add_argument("--ns", default="pode2")
    p.add_argument("--truth", required=True, help='{"Q2": {"t": 초, "x": , "y": }, ...}')
    p.add_argument("runs", nargs="*")
    p.set_defaults(fn=score)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
