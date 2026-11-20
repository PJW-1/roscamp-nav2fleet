#!/usr/bin/env python3
import argparse
import bisect
import csv
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ROUTE = ("W", "W_X", "O_X", "O")
COLUMNS = ("kind", "t_mono", "t_ros", "t_stamp", "x", "y", "yaw")
TURN_MIN_DEG = 2.0
CORNER_PAD_M = 0.10
NEAR_M = 0.25


# ---- 계획 경로 ----
def planned_path(route=ROUTE, layout="v7"):
    if (REPO / "pc" / "fleet_core").is_dir():
        sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
    from fleet_core.fleet_map import load_fleet_map
    site = load_fleet_map(layout)
    points = []
    for a, b in zip(route, route[1:], strict=False):
        edge = site.edge(a, b)
        if edge is None:
            raise ValueError(f"{site.name} has no edge {a} -> {b}")
        pts = list(edge.points)
        points += pts[1:] if points and points[-1] == pts[0] else pts
    return points, {n: site.node_pose(n) for n in route}


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def cumulative(points):
    s = [0.0]
    for (ax, ay), (bx, by) in zip(points, points[1:], strict=False):
        s.append(s[-1] + math.hypot(bx - ax, by - ay))
    return s


def corners(points, turn_min_deg=TURN_MIN_DEG, pad=CORNER_PAD_M):
    s = cumulative(points)
    turns = []
    for i in range(1, len(points) - 1):
        h0 = math.atan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0])
        h1 = math.atan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0])
        turns.append((i, _wrap(h1 - h0)))
    out, cur = [], None
    for i, turn in turns:
        if abs(math.degrees(turn)) < turn_min_deg:
            cur = None
            continue
        sign = math.copysign(1.0, turn)
        if cur and cur["sign"] == sign and cur["last"] == i - 1:
            cur["last"], cur["turn"] = i, cur["turn"] + turn
        else:
            cur = {"first": i, "last": i, "sign": sign, "turn": turn}
            out.append(cur)
    for c in out:
        c["s_from"], c["s_to"] = s[c["first"]] - pad, s[c["last"]] + pad
        c["at"] = points[(c["first"] + c["last"]) // 2]
    return out


def project(points, p):
    s = cumulative(points)
    best = None
    for i, ((ax, ay), (bx, by)) in enumerate(zip(points, points[1:], strict=False)):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        u = ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2
        inside = 0.0 <= u <= 1.0
        u = min(1.0, max(0.0, u))
        qx, qy = ax + u * dx, ay + u * dy
        dist = math.hypot(p[0] - qx, p[1] - qy)
        if best is None or dist < best[0] - 1e-12:
            seg_len = math.sqrt(length2)
            lateral = (dx * (p[1] - ay) - dy * (p[0] - ax)) / seg_len
            end_out = (i == 0 and u == 0.0 and not inside) or (
                i == len(points) - 2 and u == 1.0 and not inside)
            best = (dist, s[i] + u * seg_len, lateral, end_out)
    return best


def deviations(points, samples, near=NEAR_M):
    cs = corners(points)
    worst = {"outward": -math.inf, "abs": 0.0, "far": 0}
    per = [{"turn_deg": math.degrees(c["turn"]), "at": c["at"], "outward": -math.inf, "n": 0}
           for c in cs]
    used = 0
    for p in samples:
        dist, s, lateral, end_out = project(points, p)
        if end_out:
            continue
        used += 1
        worst["far"] += dist > near
        worst["abs"] = max(worst["abs"], abs(lateral))
        for c, out in zip(cs, per, strict=True):
            if c["s_from"] <= s <= c["s_to"]:
                out["outward"] = max(out["outward"], -c["sign"] * lateral)
                out["n"] += 1
                worst["outward"] = max(worst["outward"], out["outward"])
    return worst, per, used


# ---- 기록 읽기 ----
def map_samples(rows):
    odom = [r for r in rows if r["kind"] == "odom"]
    tf = sorted((r for r in rows if r["kind"] == "map_odom"), key=lambda r: float(r["t_ros"]))
    if tf and odom:
        times = [float(r["t_ros"]) for r in tf]
        out = []
        for r in odom:
            i = bisect.bisect_right(times, float(r["t_ros"])) - 1
            if i < 0:
                continue
            m = tf[i]
            mx, my, myaw = float(m["x"]), float(m["y"]), float(m["yaw"])
            ox, oy = float(r["x"]), float(r["y"])
            c, s = math.cos(myaw), math.sin(myaw)
            out.append((mx + c * ox - s * oy, my + s * ox + c * oy))
        return out, "odom + map->odom tf"
    amcl = [(float(r["x"]), float(r["y"])) for r in rows if r["kind"] == "amcl"]
    return amcl, "amcl_pose"


def overlay(args):
    with open(args.csv, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    points, nodes = planned_path(tuple(args.route.split(",")), args.layout)
    samples, source = map_samples(rows)
    if not samples:
        print("no map-frame samples (need map_odom tf rows or amcl rows)")
        return 1
    worst, per, used = deviations(points, samples)
    outward = ("no samples" if math.isinf(worst["outward"])
               else f"{worst['outward'] * 100:.1f} cm")
    lines = [f"path {' -> '.join(nodes)} ({args.layout}), {used}/{len(samples)} samples beside the"
             f" path ({worst['far']} farther than {NEAR_M * 100:.0f} cm), source {source}",
             f"largest outward deviation at a corner: {outward},"
             f" largest |lateral| anywhere: {worst['abs'] * 100:.1f} cm"]
    for i, c in enumerate(per, 1):
        out = "no samples" if c["n"] == 0 else f"{c['outward'] * 100:.1f} cm"
        lines.append(f"corner {i} at ({c['at'][0]:.3f}, {c['at'][1]:.3f}), turn"
                     f" {c['turn_deg']:+.0f} deg: outward {out} ({c['n']} samples)")
    print("\n".join(lines))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(*zip(*points, strict=True), "k--", lw=1, label="planned")
    ax.plot(*zip(*samples, strict=True), "-", color="tab:red", lw=0.9, label=f"robot ({source})")
    for name, (x, y) in nodes.items():
        ax.plot(x, y, "ko", ms=3)
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(4, 4), fontsize=8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title(lines[1])
    out = Path(args.png or Path(args.csv).with_suffix(".png"))
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    Path(str(out.with_suffix("")) + "_overlay.txt").write_text("\n".join(lines) + "\n",
                                                                encoding="utf-8")
    print(f"wrote {out}")
    return 0


# ---- 기록 ----
def record(args):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("path_overlay_record")
    out = Path(args.out or f"path_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    fh = out.open("w", newline="", encoding="utf-8")
    writer = csv.writer(fh)
    writer.writerow(COLUMNS)

    def yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def put(kind, stamp, x, y, q):
        now = node.get_clock().now().nanoseconds * 1e-9
        writer.writerow((kind, f"{time.monotonic():.6f}", f"{now:.6f}",
                         f"{stamp.sec + stamp.nanosec * 1e-9:.6f}", f"{x:.4f}", f"{y:.4f}",
                         f"{yaw(q):.4f}"))

    def on_odom(msg):
        p = msg.pose.pose
        put("odom", msg.header.stamp, p.position.x, p.position.y, p.orientation)

    def on_amcl(msg):
        p = msg.pose.pose
        put("amcl", msg.header.stamp, p.position.x, p.position.y, p.orientation)

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id.lstrip("/") == "map" and t.child_frame_id.lstrip("/") == "odom":
                tr = t.transform.translation
                put("map_odom", t.header.stamp, tr.x, tr.y, t.transform.rotation)

    ns = args.ns
    node.create_subscription(Odometry, f"/{ns}/odom", on_odom, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, f"/{ns}/amcl_pose", on_amcl,
                             qos_profile_sensor_data)
    node.create_subscription(TFMessage, f"/{ns}/tf", on_tf, qos_profile_sensor_data)
    print(f"recording /{ns}/odom, /{ns}/amcl_pose, /{ns}/tf (map->odom) to {out}; Ctrl+C stops")
    end = time.monotonic() + args.seconds if args.seconds > 0 else math.inf
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"wrote {out}")
    return 0


def main(argv):
    p = argparse.ArgumentParser(description="record a drive and overlay it on the planned path")
    sub = p.add_subparsers(dest="mode", required=True)
    r = sub.add_parser("record")
    r.add_argument("--ns", default="pode1")
    r.add_argument("--out", default="")
    r.add_argument("--seconds", type=float, default=0.0, help="0: until Ctrl+C")
    o = sub.add_parser("overlay")
    o.add_argument("csv")
    o.add_argument("--route", default=",".join(ROUTE))
    o.add_argument("--layout", default="v7")
    o.add_argument("--png", default="")
    args = p.parse_args(argv)
    return record(args) if args.mode == "record" else overlay(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
