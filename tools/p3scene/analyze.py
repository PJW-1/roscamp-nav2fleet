"""run_p3scene.sh 결과를 판마다 지표로 만들고 설정마다 표로 묶는다.

    python3 analyze.py <결과 폴더> [--ws <작업공간>]
쓰는 것: <결과 폴더>/trials.csv, summary.json, 화면에 표.

지표(판마다):
- time_s: goal을 보낸 때부터 결과까지(Gazebo 시간), status, BT 복구 수(피드백)
- route: pode2가 pode3의 x를 지날 때 y가 진열대 B 아래(0.365)면 gap, 위(0.405)면 north
- pode2 로그 줄 수(판의 벽시계 구간): collision ahead, patience, 계획 실패, progress 실패
- 서 있던 시간: 직선 속도 < 0.01 m/s이고 회전 < 0.05 rad/s가 1 s 넘게 이어진 구간의 합, 가장 긴 것
- 최소 몸 간격(실제 위치): pode2-pode3 발자국 다각형, pode2-월드 상자(진열대, 벽).
  발자국은 nav2_params.yaml local_costmap footprint(padding 없음). 겹치면 0 이하.
"""
import argparse
import csv
import json
import math
import re
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

SHELF_B_LOW, SHELF_B_HIGH = 0.365, 0.405
STAND_V, STAND_W, STAND_MIN_S = 0.01, 0.05, 1.0
LONG_STOP_S = 5.0
FLAG_GAP_M = 0.005
PATTERNS = {
    "collision": re.compile(r"collision ahead", re.I),
    "patience": re.compile(r"patience", re.I),
    "plan_fail": re.compile(r"failed to create plan|no valid path|planning algorithm .*failed|"
                            r"failed to compute path|GridBased plugin failed", re.I),
    "progress_fail": re.compile(r"failed to make progress", re.I),
}
LOG_LINE = re.compile(r"\[(?:ERROR|WARN|INFO|DEBUG|FATAL)\] \[(\d+\.\d+)\] \[([^\]]+)\]")


# ---- 기하 ----
def footprint(ws: Path) -> list:
    data = yaml.safe_load((ws / "pode/pode_nav_config/params/nav2_params.yaml").read_text())
    return [tuple(p) for p in yaml.safe_load(
        data["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])]


def boxes(ws: Path, model="arena") -> dict:
    """월드의 상자 충돌체(이름 -> 꼭짓점 넷, map 틀)."""
    root = ET.parse(ws / "sim/fleet_gz_sim/worlds/arena_v7.sdf").getroot()
    out = {}
    for m in root.iter("model"):
        if m.get("name") != model:
            continue
        for c in m.iter("collision"):
            size = c.find("geometry/box/size")
            if size is None:
                continue
            x, y, _z, _r, _p, yaw = map(float, c.find("pose").text.split())
            sx, sy, _ = map(float, size.text.split())
            corners = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
            out[c.get("name")] = place(corners, (x, y, yaw))
    return out


def place(poly, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in poly]


def _seg_point(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _inside(p, poly):
    x, y = p
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def _cross(a, b, c, d):
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return o(a, b, c) * o(a, b, d) < 0 and o(c, d, a) * o(c, d, b) < 0


def gap(A, B) -> float:
    """두 다각형(볼록이 아니어도 된다) 사이 거리. 떨어져 있으면 정확하다. 겹치면 0 이하:
    다른 쪽 안에 든 꼭짓점의 가장 깊은 거리에 음수를 붙인다(깊이의 아래 한계)."""
    ea, eb = list(zip(A, A[1:] + A[:1])), list(zip(B, B[1:] + B[:1]))
    deep = [min(_seg_point(p, *e) for e in eb) for p in A if _inside(p, B)]
    deep += [min(_seg_point(p, *e) for e in ea) for p in B if _inside(p, A)]
    if deep:
        return -max(deep)
    if any(_cross(*e, *f) for e in ea for f in eb):
        return 0.0
    return min(min(_seg_point(p, *e) for p in A for e in eb),
               min(_seg_point(p, *e) for p in B for e in ea))


# ---- 궤적 ----
def stops(track):
    """서 있던 구간들 [(시작, 길이)]. track: (t, x, y, yaw)."""
    out, start = [], None
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(track, track[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        v = math.hypot(x1 - x0, y1 - y0) / dt
        w = abs(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))) / dt
        still = v < STAND_V and w < STAND_W
        if still and start is None:
            start = t0
        elif not still and start is not None:
            out.append((start, t0 - start))
            start = None
    if start is not None:
        out.append((start, track[-1][0] - start))
    return [(s, d) for s, d in out if d > STAND_MIN_S]


def resample(track, dt=0.1):
    """시간 간격을 고르게(속도 잡음 줄이기)."""
    out, nxt = [], None
    for p in track:
        if nxt is None or p[0] >= nxt:
            out.append(p)
            nxt = p[0] + dt
    return out


def route(track, x3):
    """pode3의 x를 처음 지날 때 pode2의 y로 gap/north."""
    for (_, xa, ya, _), (_, xb, yb, _) in zip(track, track[1:]):
        if (xa - x3) * (xb - x3) <= 0 and xa != xb:
            y = ya + (yb - ya) * (x3 - xa) / (xb - xa)
            return ("gap" if y < SHELF_B_LOW else "north" if y > SHELF_B_HIGH else "edge"), y
    return "none", None


def log_counts(lines, t0, t1, robot="pode2"):
    counts = dict.fromkeys(PATTERNS, 0)
    for stamp, logger, text in lines:
        if t0 <= stamp <= t1 + 0.5 and logger.startswith(robot + "."):
            for k, pat in PATTERNS.items():
                if pat.search(text):
                    counts[k] += 1
    return counts


def read_log(path: Path):
    out = []
    if not path.exists():
        return out
    for line in path.read_text(errors="replace").splitlines():
        m = LOG_LINE.search(line)
        if m:
            out.append((float(m.group(1)), m.group(2), line[m.end():]))
    return out


def trial_row(rec, fp, world, lines):
    row = {k: rec.get(k) for k in ("config", "trial", "status", "duration_s", "recoveries")}
    row["set"] = rec.get("pode1_set", "A")
    row["pode3_set"] = "/".join(str(v) for v in rec["pode3_set"])
    if not rec.get("ok"):
        row["status"] = rec.get("error", "failed")
        return row
    t2 = resample(rec["truth"]["pode2"])
    t3 = rec["truth"]["pode3"]
    p3 = t3[-1][1:] if t3 else rec["pode3_truth"]
    row["pode3_truth"] = "/".join(f"{v:.3f}" for v in (p3[0], p3[1], math.degrees(p3[2])))
    body3 = place(fp, p3)
    g3, gw, gnamed = math.inf, math.inf, {}
    for _, x, y, a in rec["truth"]["pode2"]:
        body2 = place(fp, (x, y, a))
        if math.hypot(x - p3[0], y - p3[1]) < 0.4:
            g3 = min(g3, gap(body2, body3))
        for name, box in world.items():
            if min(math.hypot(x - bx, y - by) for bx, by in box) < 1.2 or name.startswith("wall"):
                d = gap(body2, box)
                gnamed[name] = min(gnamed.get(name, math.inf), d)
    gw = min(gnamed.values()) if gnamed else math.inf
    row["gap_pode3_m"] = round(g3, 4)
    row["gap_world_m"] = round(gw, 4)
    row["gap_world_box"] = min(gnamed, key=gnamed.get) if gnamed else ""
    row["gap_shelf_2_m"] = round(gnamed.get("shelf_2", math.inf), 4)
    row["gap_wall_south_m"] = round(gnamed.get("wall_south", math.inf), 4)
    row["route"], y = route(t2, p3[0])
    row["y_at_pode3"] = None if y is None else round(y, 3)
    plan = rec["plans"][0][1] if rec.get("plans") else []
    row["plan_route"], _ = route([(0, x, y, 0) for x, y in plan], p3[0])
    st = stops(t2)
    row["stand_s"] = round(sum(d for _, d in st), 1)
    row["longest_stop_s"] = round(max((d for _, d in st), default=0.0), 1)
    row.update(log_counts(lines, rec["wall_t0"], rec["wall_t1"]))
    end = rec.get("pode2_end") or (0, 0, 0)
    row["end_err_m"] = round(math.hypot(end[0] - 1.66, end[1] - 0.16), 3)
    issues = []
    if row["status"] != "succeeded":
        issues.append(row["status"])
    if row["recoveries"]:
        issues.append(f"recoveries {row['recoveries']}")
    if row["longest_stop_s"] > LONG_STOP_S:
        issues.append(f"stop {row['longest_stop_s']} s")
    if row["gap_pode3_m"] <= FLAG_GAP_M:
        issues.append(f"pode3 gap {row['gap_pode3_m']}")
    if row["gap_world_m"] <= FLAG_GAP_M:
        issues.append(f"{row['gap_world_box']} gap {row['gap_world_m']}")
    row["issues"] = "; ".join(issues)
    return row


def summary(rows):
    ok = [r for r in rows if r.get("route") is not None]
    times = [r["duration_s"] for r in ok if r["status"] == "succeeded"]
    return {
        "trials": len(rows),
        "succeeded": len(times),
        "median_s": round(statistics.median(times), 1) if times else None,
        "max_s": max(times) if times else None,
        "gap": sum(r["route"] == "gap" for r in ok),
        "north": sum(r["route"] == "north" for r in ok),
        "aborts": sum(r["status"] in ("aborted", "timeout", "canceled") for r in rows),
        "with_issue": sum(bool(r.get("issues")) for r in rows),
        "collision_lines": sum(r.get("collision", 0) for r in ok),
        "stand_s_median": round(statistics.median([r["stand_s"] for r in ok]), 1) if ok else None,
        "min_gap_pode3_m": min((r["gap_pode3_m"] for r in ok), default=None),
        "min_gap_world_m": min((r["gap_world_m"] for r in ok), default=None),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--ws", default=str(Path(__file__).resolve().parents[2]))
    a = ap.parse_args()
    out, ws = Path(a.out), Path(a.ws)
    fp, world = footprint(ws), boxes(ws)
    rows, summ = [], {}
    for d in sorted(p for p in out.iterdir() if (p / "trials.jsonl").exists()):
        lines = read_log(d / "launch.log")
        cfg_rows = [trial_row(json.loads(t), fp, world, lines)
                    for t in (d / "trials.jsonl").read_text().splitlines() if t.strip()]
        rows += cfg_rows
        for group in sorted({r["set"] for r in cfg_rows}):
            summ[f"{d.name}/{group}"] = summary([r for r in cfg_rows if r["set"] == group])
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(out / "trials.csv", "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    (out / "summary.json").write_text(json.dumps({"configs": summ, "trials": rows}, indent=1))
    cols = ["trials", "succeeded", "median_s", "max_s", "gap", "north", "aborts", "with_issue",
            "collision_lines", "stand_s_median", "min_gap_pode3_m", "min_gap_world_m"]
    print("| cfg | " + " | ".join(cols) + " |")
    print("|" + "---|" * (len(cols) + 1))
    for name, s in summ.items():
        print(f"| {name} | " + " | ".join(str(s[c]) for c in cols) + " |")
    print()
    for r in rows:
        print(f"{r['config']}/{r['set']} t{r['trial']} p3={r.get('pode3_truth')} {r['status']} "
              f"{r.get('duration_s')} s route={r.get('route')}(plan {r.get('plan_route')}) "
              f"y={r.get('y_at_pode3')} coll={r.get('collision')} pat={r.get('patience')} "
              f"plan_fail={r.get('plan_fail')} prog={r.get('progress_fail')} "
              f"stand={r.get('stand_s')}/{r.get('longest_stop_s')} s "
              f"gap3={r.get('gap_pode3_m')} world={r.get('gap_world_m')}({r.get('gap_world_box')}) "
              f"shelf2={r.get('gap_shelf_2_m')} wallS={r.get('gap_wall_south_m')} "
              f"| {r.get('issues', '')}")


if __name__ == "__main__":
    main()
