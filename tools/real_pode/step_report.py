#!/usr/bin/env python3
import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

RESPONSE_RATIO = 0.5
ACCEL_WINDOW_S = 0.1


# ---- CSV 읽기 ----
def load_rows(paths):
    rows = []
    for path in paths:
        with open(path, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                if not r["set"]:
                    continue
                rows.append(r)
    return rows


def _f(r, key):
    return float(r[key]) if r[key] != "" else math.nan


def segments(rows):
    out = defaultdict(lambda: {"cmd": [], "odom": []})
    for r in rows:
        key = (r["set"], int(r["repeat"]), int(r["step"]), r["label"])
        seg = out[key]
        seg[r["kind"]].append(r)
    found = []
    for (set_name, repeat, step, label), seg in sorted(out.items(), key=lambda kv: kv[0][1:3]):
        steps = [c for c in seg["cmd"] if c["phase"] == "step"]
        if not steps:
            continue
        t0 = _f(steps[0], "t_ros")
        later = [c for c in seg["cmd"] if c["phase"] in ("stop", "home") and _f(c, "t_ros") > t0]
        t1 = min(_f(c, "t_ros") for c in later) if later else _f(steps[-1], "t_ros")
        stops = [c for c in seg["cmd"] if c["phase"] == "stop"]
        t2 = max(_f(c, "t_ros") for c in stops) if stops else t1
        odom = sorted(seg["odom"], key=lambda r: _f(r, "t_ros"))
        found.append({
            "set": set_name, "repeat": repeat, "step": step, "label": label,
            "v": _f(steps[0], "cmd_v"), "w": _f(steps[0], "cmd_w"), "t0": t0, "t1": t1, "t2": t2,
            "t": np.array([_f(r, "t_ros") for r in odom]),
            "ov": np.array([_f(r, "odom_v") for r in odom]),
            "ow": np.array([_f(r, "odom_w") for r in odom]),
            "x": np.array([_f(r, "x") for r in odom]),
            "y": np.array([_f(r, "y") for r in odom]),
        })
    return found


# ---- 계단 응답 ----
def crossing(t, y, level, t_from, t_to):
    idx = np.nonzero((t >= t_from) & (t <= t_to) & (y >= level))[0]
    if idx.size == 0:
        return math.nan
    i = idx[0]
    if i == 0 or t[i - 1] < t_from or y[i - 1] >= level:
        return float(t[i])
    frac = (level - y[i - 1]) / (y[i] - y[i - 1])
    return float(t[i - 1] + frac * (t[i] - t[i - 1]))


def slopes(t, y, window=ACCEL_WINDOW_S):
    out = np.full(t.size, math.nan)
    for i in range(t.size):
        sel = (t >= t[i] - window / 2) & (t <= t[i] + window / 2)
        if sel.sum() >= 3:
            out[i] = np.polyfit(t[sel] - t[i], y[sel], 1)[0]
    return out


def step_metrics(t, y, target, t0, t1, t2=None):
    if target == 0 or t.size == 0:
        return {}
    s = math.copysign(1.0, target)
    a = abs(target)
    yy = s * y
    t10 = crossing(t, yy, 0.1 * a, t0, t1)
    t90 = crossing(t, yy, 0.9 * a, t0, t1)
    tail = (t >= t0 + 0.5 * (t1 - t0)) & (t <= t1)
    ratio = float(np.mean(yy[tail]) / a) if tail.any() else math.nan
    acc = slopes(t, yy)
    up = (t >= t0) & (t <= t1)
    acc_up = slopes(t[up], yy[up])
    down = (t > t1) & (t <= (t2 if t2 is not None else t1 + 1.0))
    return {
        "latency": t10 - t0,
        "rise": t90 - t10,
        "ratio": ratio,
        "acc_max": float(np.nanmax(acc_up)) if np.isfinite(acc_up).any() else math.nan,
        "dec_max": float(-np.nanmin(acc[down])) if np.isfinite(acc[down]).any() else math.nan,
    }


def response_of(seg):
    if seg["set"] == "linear" or (seg["set"] != "angular" and seg["w"] == 0):
        return seg["v"], seg["ov"]
    return seg["w"], seg["ow"]


# ---- 데드밴드 ----
def deadband(segs, threshold=RESPONSE_RATIO):
    groups = defaultdict(list)
    for seg in segs:
        if seg["set"] != "small_rot":
            continue
        m = step_metrics(seg["t"], seg["ow"], seg["w"], seg["t0"], seg["t1"], seg["t2"])
        groups[(round(seg["v"], 3), round(seg["w"], 3))].append(m.get("ratio", math.nan))
    table = {k: float(np.nanmean(v)) for k, v in groups.items()}
    result = {}
    for v in sorted({k[0] for k in table}):
        ok = [w for (vv, w), r in sorted(table.items()) if vv == v and r >= threshold]
        result[v] = min(ok) if ok else math.nan
    return table, result


# ---- 원호 ----
def fit_circle(x, y):
    a = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    c, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy = c[0] / 2, c[1] / 2
    return float(math.sqrt(c[2] + cx * cx + cy * cy)), (float(cx), float(cy))


def arc_metrics(seg, settle_s=0.5):
    sel = (seg["t"] >= seg["t0"] + settle_s) & (seg["t"] <= seg["t1"])
    if sel.sum() < 5:
        return {}
    r_fit, _ = fit_circle(seg["x"][sel], seg["y"][sel])
    mw = float(np.mean(seg["ow"][sel]))
    r_twist = abs(float(np.mean(seg["ov"][sel])) / mw) if mw else math.nan
    r_cmd = abs(seg["v"] / seg["w"])
    return {"r_cmd": r_cmd, "r_fit": r_fit, "r_twist": r_twist, "fit_ratio": r_fit / r_cmd}


# ---- 보고서 ----
def _fmt(x, nd=3):
    return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def _reach(x):
    return "미도달" if math.isnan(x) else f"{x:.3f}"


def summarize(segs):
    lines = []
    steps = [s for s in segs if s["set"] in ("linear", "angular")]
    if steps:
        lines += ["## 계단 응답", "",
                  "| set | label | 반복 | 지연 s | 상승 s | 정상 비 | 최대 가속 | 최대 감속 |",
                  "|---|---|---|---|---|---|---|---|"]
        per = defaultdict(list)
        for s in steps:
            target, y = response_of(s)
            m = step_metrics(s["t"], y, target, s["t0"], s["t1"], s["t2"])
            per[(s["set"], s["label"])].append(m)
            lines.append(f"| {s['set']} | {s['label']} | {s['repeat']} | {_reach(m['latency'])} |"
                         f" {_reach(m['rise'])} | {_fmt(m['ratio'])} | {_fmt(m['acc_max'], 2)} |"
                         f" {_fmt(m['dec_max'], 2)} |")
        lines += ["", "평균(반복 묶음):", "",
                  "| set | label | 지연 s | 상승 s | 정상 비 | 최대 가속 | 최대 감속 |",
                  "|---|---|---|---|---|---|---|"]
        for (set_name, label), ms in per.items():
            avg = {k: float(np.nanmean([m[k] for m in ms])) for k in ms[0]}
            for k in ("latency", "rise"):
                if any(math.isnan(m[k]) for m in ms):
                    avg[k] = math.nan
            lines.append(f"| {set_name} | {label} | {_reach(avg['latency'])} |"
                         f" {_reach(avg['rise'])} | {_fmt(avg['ratio'])} |"
                         f" {_fmt(avg['acc_max'], 2)} | {_fmt(avg['dec_max'], 2)} |")
        lines += ["", "가속 단위: linear는 m/s², angular는 rad/s². 지연은 명령에서 odom이"
                  " 목표의 10%에 닿기까지, 상승은 10%에서 90%까지. 둘 다 명령을 내는 동안에서만"
                  " 찾고, 그 안에 닿지 못하면 미도달(평균은 한 번이라도 못 닿으면 미도달).", ""]
    table, db = deadband(segs)
    if table:
        lines += ["## 작은 회전(데드밴드)", "", "| v | ω 명령 | 평균 ω 비 |", "|---|---|---|"]
        for (v, w), r in sorted(table.items()):
            lines.append(f"| {v:.2f} | {w:.2f} | {_fmt(r)} |")
        lines.append("")
        for v, w in db.items():
            lines.append(f"- v={v:.2f}: 반응하는 가장 작은 ω = {_fmt(w, 2)} rad/s"
                         f" (평균 비 ≥ {RESPONSE_RATIO})")
        lines.append("")
    arcs = [s for s in segs if s["set"] == "arc"]
    if arcs:
        lines += ["## 원호", "",
                  "| label | 반복 | R 명령 | R 궤적 맞춤 | R v÷ω(odom) | 맞춤/명령 |",
                  "|---|---|---|---|---|---|"]
        sides = defaultdict(list)
        for s in arcs:
            m = arc_metrics(s)
            if not m:
                continue
            sides["left" if s["w"] > 0 else "right"].append(m["r_fit"])
            lines.append(f"| {s['label']} | {s['repeat']} | {_fmt(m['r_cmd'])} |"
                         f" {_fmt(m['r_fit'])} | {_fmt(m['r_twist'])} | {_fmt(m['fit_ratio'])} |")
        if sides.get("left") and sides.get("right"):
            left, right = float(np.mean(sides["left"])), float(np.mean(sides["right"]))
            lines += ["", f"- 왼쪽 평균 R {left:.3f} m, 오른쪽 {right:.3f} m,"
                      f" 차이 {left - right:+.3f} m ({(left - right) / right * 100:+.1f}%)"]
        lines.append("")
    return "\n".join(lines)


def plot(segs, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    files = []
    for set_name in ("linear", "angular", "small_rot"):
        group = [s for s in segs if s["set"] == set_name]
        if not group:
            continue
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for s in group:
            target, y = response_of(s)
            sel = (s["t"] >= s["t0"] - 0.3) & (s["t"] <= s["t2"])
            ax.plot(s["t"][sel] - s["t0"], y[sel], lw=0.8)
            ax.hlines(target, 0, s["t1"] - s["t0"], colors="0.4", linestyles="dashed", lw=0.6)
        ax.set_xlabel("time from command (s)")
        ax.set_ylabel("odom v (m/s)" if set_name == "linear" else "odom ω (rad/s)")
        ax.set_title(f"{set_name} steps (dashed: command)")
        ax.grid(alpha=0.3)
        path = Path(out_dir) / f"{set_name}.png"
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        files.append(path)
    arcs = [s for s in segs if s["set"] == "arc"]
    if arcs:
        fig, ax = plt.subplots(figsize=(6, 6))
        for s in arcs:
            sel = (s["t"] >= s["t0"]) & (s["t"] <= s["t1"])
            ax.plot(s["x"][sel] - s["x"][sel][0], s["y"][sel] - s["y"][sel][0], lw=0.8,
                    color="tab:blue" if s["w"] > 0 else "tab:red")
        ax.set_aspect("equal")
        ax.set_title("arcs from each start (blue left, red right), odom frame")
        ax.grid(alpha=0.3)
        path = Path(out_dir) / "arc.png"
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        files.append(path)
    return files


def main(argv):
    p = argparse.ArgumentParser(description="report from cmd_step_probe CSV files")
    p.add_argument("csv", nargs="+")
    p.add_argument("--out", default=".", help="folder for report.md and PNG files")
    p.add_argument("--title", default="cmd_vel 계단 응답")
    args = p.parse_args(argv)
    segs = segments(load_rows(args.csv))
    if not segs:
        print("no step segments in the CSV files")
        return 1
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    pngs = plot(segs, out)
    text = (f"# {args.title}\n\n입력: {', '.join(Path(c).name for c in args.csv)}\n\n"
            + summarize(segs) + "\n" + "".join(f"![{p.stem}]({p.name})\n" for p in pngs))
    (out / "report.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
