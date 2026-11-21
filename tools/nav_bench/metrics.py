import math
import statistics
import sys
from bisect import bisect_left
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import movers as mv  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NAV2 = ROOT / "pode" / "pode_nav_config" / "params" / "nav2_params.yaml"
ARENA = ROOT / "pc" / "fleet_core" / "fleet_core" / "data" / "fleet_map_v7.yaml"
WALL = 0.05


# ---- 몸과 경기장 ----
def footprint(path=NAV2):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return [tuple(p) for p in yaml.safe_load(
        data["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])]


def arena_rects(path=ARENA):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    x0, y0, x1, y1 = data["bounds"]
    walls = [(x0 - WALL, y0 - WALL, x1 + WALL, y0), (x0 - WALL, y1, x1 + WALL, y1 + WALL),
             (x0 - WALL, y0, x0, y1), (x1, y0, x1 + WALL, y1)]
    shelves = [tuple(s) for s in data.get("sim_shelves", data["shelves"])]
    return [mv.rect(*r) for r in walls + shelves]


# ---- 지표 ----
def sign_changes(values, deadband):
    last, count = 0, 0
    for v in values:
        if abs(v) < deadband:
            continue
        s = 1 if v > 0 else -1
        if last and s != last:
            count += 1
        last = s
    return count


def blocked_events(truth, window_s, move_m):
    count, i = 0, 0
    while i < len(truth):
        j = i
        while j < len(truth) and truth[j][0] - truth[i][0] < window_s:
            j += 1
        if j >= len(truth):
            break
        if max(math.dist(truth[i][1:3], p[1:3]) for p in truth[i:j + 1]) < move_m:
            count += 1
            i = j
        else:
            i += 1
    return count


def path_length(truth):
    return sum(math.dist(a[1:3], b[1:3]) for a, b in zip(truth, truth[1:], strict=False))


def body_clearance(truth, body, statics, movers, mover_track):
    times = [m[0] for m in mover_track]
    out = []
    for t, x, y, yaw in truth:
        shape = mv.to_world(body, x, y, yaw)
        d = min((mv.poly_dist(shape, r) for r in statics), default=math.inf)
        if movers and mover_track:
            k = min(bisect_left(times, t), len(times) - 1)
            for m, pose in zip(movers, mover_track[k][1:], strict=False):
                d = min(d, mv.poly_dist(shape, mv.mover_shape(m, tuple(pose))))
        out.append((t, d))
    return out


def events_below(series, limit):
    count, below = 0, False
    for _, d in series:
        if d <= limit and not below:
            count += 1
        below = d <= limit
    return count


def within(rows, t0, t1):
    return [r for r in rows if t0 <= r[0] <= t1]


def leg_metrics(rec, cfg, body, statics):
    movers = [cfg["movers"][n] for n in rec.get("movers_names", [])]
    t0, t1 = rec["t_start"], rec["t_end"]
    truth = within(rec["truth"], t0, t1)
    cmd = within(rec["cmd"], t0, t1)
    wz = [c[2] for c in cmd]
    dts = sorted(b[0] - a[0] for a, b in zip(cmd, cmd[1:], strict=False))
    clear = body_clearance(truth, body, statics, movers, rec["movers"])
    scans = [s[1] for s in within(rec["scan_min"], t0, t1) if s[1] is not None]
    goal = cfg["goal"]
    end_err = math.dist(truth[-1][1:3], (goal["x"], goal["y"])) if truth else None
    ok = rec["status"] == "SUCCEEDED" and end_err is not None and end_err <= cfg["arrive_m"]
    return {
        "scenario": rec["scenario"], "controller": rec["controller"], "repeat": rec["repeat"],
        "status": rec["status"], "ok": ok, "end_err_m": end_err,
        "retried": rec.get("retried", False), "pose_errors": rec.get("pose_errors", 0),
        "time_s": rec["t_end"] - rec["t_start"],
        "path_m": path_length(truth),
        "lidar_min_m": min(scans) if scans else None,
        "body_min_m": min(d for _, d in clear) if clear else None,
        "collisions": events_below(clear, cfg["collision_m"]),
        "blocked": blocked_events(truth, cfg["blocked_window_s"], cfg["blocked_move_m"]),
        "doridori": sign_changes(wz, cfg["doridori_deadband"]),
        "cpu_pct": None if not rec.get("cpu_s") or not rec.get("wall_s") else
        100.0 * rec["cpu_s"] / rec["wall_s"],
        "cmd_dt_p95_ms": 1000 * dts[int(0.95 * (len(dts) - 1))] if dts else None,
    }


def summary(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None
    return statistics.fmean(vals), (statistics.stdev(vals) if len(vals) > 1 else 0.0)
