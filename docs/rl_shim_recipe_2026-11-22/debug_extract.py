import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "nav_bench"))
import metrics as mt  # noqa: E402

LOGGER = "pode1.controller_server"
THRESH, DISENGAGE, AHEAD, DT, SAMPLE = 0.785, 0.3925, 1.0, 0.05, 0.15
CAUSES = (("collision ahead", "collision"), ("Failed to transform", "tf"),
          ("too short", "short_path"))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def cause_of(text):
    return next((c for key, c in CAUSES if key in text), "other")


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {"log": [], "costmap": [], "odom": []}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, types[topic])
        ts = t * 1e-9
        if topic == "/rosout" and msg.name == LOGGER:
            if "what():" in msg.msg:
                kind = "goal_catch" if "goal point" in msg.msg else "catch"
                out["log"].append((ts, kind, msg.msg.split("what():", 1)[1].strip()))
            elif "is at the new path's rough heading" in msg.msg:
                out["log"].append((ts, "normal", ""))
            elif "is not within the new path's rough heading" in msg.msg:
                out["log"].append((ts, "rotating", ""))
        elif topic.endswith("local_costmap/costmap"):
            i = msg.info
            grid = np.array(msg.data, dtype=np.int16).reshape(i.height, i.width)
            out["costmap"].append((ts, i.resolution, i.origin.position.x, i.origin.position.y,
                                   grid))
        elif topic.endswith("/odom"):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            out["odom"].append((ts, p.x, p.y, yaw))
    return out


def before(rows, t):
    best = None
    for r in rows:
        if r[0] > t:
            break
        best = r
    return best


def nearest(rows, t):
    return min(rows, key=lambda r: abs(r[0] - t)) if rows else None


def heading_error(rec, t, pose):
    plan = before(sorted(rec["plans"], key=lambda p: p[0]), t)
    if plan is None:
        return None
    pts = plan[1]
    sx, sy = pts[0]
    samp = next(((x, y) for x, y in pts[1:] if math.hypot(x - sx, y - sy) >= SAMPLE), pts[-1])
    _, x, y, yaw = pose
    return wrap(math.atan2(samp[1] - y, samp[0] - x) - yaw)


def cells_on_line(a, b):
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx + dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def outline_hit(grid, res, ox, oy, body, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in body]
    cells = []
    for px, py in pts:
        j, i = math.floor((px - ox) / res), math.floor((py - oy) / res)
        if not (0 <= i < grid.shape[0] and 0 <= j < grid.shape[1]):
            return ("out_of_map", round(px, 3), round(py, 3))
        cells.append((j, i))
    for a, b in zip(cells, cells[1:] + cells[:1], strict=True):
        for j, i in cells_on_line(a, b):
            v = grid[i, j]
            if v >= 100 or v < 0:
                kind = "lethal" if v >= 100 else "unknown"
                return (kind, round(ox + (j + 0.5) * res, 3), round(oy + (i + 0.5) * res, 3))
    return None


def shim_check(cm, od, body, err, w):
    _, res, ox, oy, grid = cm
    _, x, y, yaw0 = od
    remaining = abs(err) - THRESH
    t = 0.0
    while t < AHEAD - 1e-9:
        t += DT
        yaw = yaw0 + w * t
        if wrap(yaw0 - yaw) >= remaining:
            return {"stop_s": round(t, 2), "hit": None}
        hit = outline_hit(grid, res, ox, oy, body, x, y, yaw)
        if hit:
            return {"hit_s": round(t, 2), "hit": hit}
    return {"stop_s": round(t, 2), "hit": None}


def rotations(rec):
    runs, cur = [], None
    for t, v, w in rec["cmd"]:
        if t < rec["t_start"] or t > rec["t_end"]:
            continue
        if v == 0.0 and w != 0.0:
            cur = [t, t] if cur is None else [cur[0], t]
        elif cur is not None:
            runs.append((cur[0], cur[1], t))
            cur = None
    if cur is not None:
        runs.append((cur[0], cur[1], cur[1]))
    return runs


def leg_events(rec, bag, body):
    t0, t1 = rec["t_start"], rec["t_end"]
    truth = rec["truth"]
    rows = []
    for start, last, end in rotations(rec):
        pose = nearest(truth, end)
        err = heading_error(rec, end, pose)
        logs = [g for g in bag["log"] if last - 0.1 <= g[0] <= end + 0.25 and g[1] != "rotating"]
        shim = sum(1 for g in bag["log"] if start - 0.1 <= g[0] <= last + 0.1
                   and g[1] == "rotating")
        w = [c[2] for c in rec["cmd"] if c[0] <= last][-1]
        plan = before(sorted(rec["plans"], key=lambda q: q[0]), end)
        at_goal = plan is not None and math.dist(pose[1:3], plan[1][-1]) < 0.15
        row = {"t_rot": round(start - t0, 2), "t_end": round(end - t0, 2),
               "rot_s": round(last - start, 2), "pose": [round(v, 3) for v in pose[1:]],
               "err": None if err is None else round(err, 3), "w": round(w, 2),
               "goal_turn": at_goal, "shim_rotating_lines": shim,
               "early": not at_goal and err is not None and abs(err) > DISENGAGE + 0.05,
               "log": [(round(g[0] - t0, 2), g[1], cause_of(g[2]) if g[1] == "catch" else "", g[2])
                       for g in logs]}
        cm, od = before(bag["costmap"], end), nearest(bag["odom"], end)
        if cm and od and err is not None:
            row["costmap_age_s"] = round(end - cm[0], 2)
            row["check"] = shim_check(cm, od, body, err, math.copysign(abs(w) or 1.0, err))
        rows.append(row)
    catches = [g for g in bag["log"] if t0 <= g[0] <= t1 and g[1] == "catch"]
    return rows, catches


def main():
    out = Path(sys.argv[1])
    if (out / "bag").exists():
        bag = read_bag(out / "bag")
        if not bag["log"] or not bag["costmap"] or not bag["odom"]:
            sys.exit(f"bag is missing data: {len(bag['log'])} shim log lines, "
                     f"{len(bag['costmap'])} costmaps, {len(bag['odom'])} odom")
    elif "--legs-only" in sys.argv:
        bag = {"log": [], "costmap": [], "odom": []}
    else:
        sys.exit(f"no bag in {out} (pass --legs-only to read the legs alone)")
    body = mt.footprint()
    report = {}
    for legs in sorted(out.glob("legs_*")):
        for p in sorted(legs.glob("*.json")):
            rec = json.loads(p.read_text())
            rows, catches = leg_events(rec, bag, body)
            key = f"{legs.name}/{p.name}"
            report[key] = {"status": rec["status"], "rotations": rows,
                           "catches": [(round(g[0] - rec["t_start"], 2), cause_of(g[2]), g[2])
                                       for g in catches]}
            print(f"{key}: {rec['status']}, {len(rows)} rotations, {len(catches)} catch lines")
            for r in rows:
                causes = ",".join(c for _, kind, c, _ in r["log"] if kind == "catch") or (
                    "normal" if any(k == "normal" for _, k, _, _ in r["log"]) else "-")
                kind = "EARLY" if r["early"] else "goal " if r["goal_turn"] else "end  "
                print(f"  rot {r['t_rot']:6.2f}-{r['t_end']:6.2f} s ({r['rot_s']:.2f} s) "
                      f"{kind} err {r['err']} w {r['w']} "
                      f"pose {r['pose']} log {causes} check {r.get('check')}")
    (out / "debug_extract.json").write_text(json.dumps(report, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
