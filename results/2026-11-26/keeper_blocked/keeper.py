#!/usr/bin/env python3
"""11-26 실물 두 판: nav_goal 조각별 /plan 수와 뒤집힘(first_look = OTHER).

/plan은 planner_server 출력이다. keeper(PLAN_KEEP=1)가 돌려준 길(/plan_kept)은 bag에 없어,
로봇이 받은 길의 뒤집힘은 keeper 규칙(first_look -> second_look)을 다시 돌려 추정한다.
막힘 칸 검사는 전역 costmap이 없어 '열림'으로 둔다(남은 경우는 OFF와 SHORTER).
뒤집힘마다 실제 궤적(map->base)이 두 길 가운데 어디를 따랐는지도 본다.

사용: python3 keeper.py bags.pkl 출력폴더
"""
import json
import math
import pickle
import re
import sys
from bisect import bisect_right
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "pode" / "pode_plan_keep"))
from pode_plan_keep.route import (OFF, OTHER, SHORTER, distance_to,  # noqa: E402
                                  first_look, length, second_look)

DATA = Path("/home/ane/ros2camp/real_data/2026-11-26")
RUNS = {"run1": ("fleet_run1_launch.log", "fleet_run1.jsonl", False),
        "run2": ("fleet_run2_launch.log", "fleet_run2.jsonl", True),
        "express": ("fleet_express_launch.log", "fleet_express.jsonl", False)}
SEG = re.compile(r"\[(\d+\.\d+)\].*SEGMENT_DONE (\S+) seq (\d+) \((\w+)\) to (\S+)")


def kst(t):
    s = (t + 9 * 3600) % 86400
    return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:05.2f}"


# ---- 관제 기록에서 nav_goal 조각(로봇, 목표, 시작, 끝) ----
def nav_legs(log, jsonl):
    targets = {}
    for line in open(DATA / jsonl):
        d = json.loads(line)
        if d.get("out") == "MoveOrderCmd":
            for s in d["base"] + d.get("horizon", []):
                if s["mode"] == "nav_goal":
                    targets[(d["order_id"], s["seq_id"])] = (s["end_node"], s["x"], s["y"])
    done = {}
    for line in open(DATA / log):
        m = SEG.search(line)
        if m:
            done[(m[2], int(m[3]))] = (float(m[1]), m[4], m[5])
    legs = []
    for (order, seq), (t_end, mode, node) in sorted(done.items(), key=lambda kv: kv[1][0]):
        if mode != "nav_goal":
            continue
        t0 = done.get((order, seq - 1), (None,))[0]
        _, x, y = targets[(order, seq)]
        legs.append({"robot": order.split("-")[0], "order": order, "to": node, "goal": (x, y),
                     "t0": t0, "t1": t_end})
    return legs


# ---- map->base 궤적(map->odom 최근 값 + odom->base) ----
def trajectory(bag):
    mo = [(t, x, y, a) for t, p, c, x, y, a in bag["tf"] if p == "map" and c == "odom"]
    mo.sort()
    keys = [m[0] for m in mo]
    out = []
    for t, x, y, a, _, _ in bag["odom"]:
        i = bisect_right(keys, t) - 1
        if i < 0:
            continue
        _, mx, my, ma = mo[i]
        c, s = math.cos(ma), math.sin(ma)
        out.append((t, mx + c * x - s * y, my + s * x + c * y))
    return out


# ---- 뒤집힘 뒤 실제 궤적이 따른 길 ----
def followed(traj, t, old_rest, new, horizon_s=4.0):
    pts = [(x, y) for tt, x, y in traj if t <= tt <= t + horizon_s]
    if len(pts) < 5:
        return "n/a", None, None
    d_old = sum(distance_to(old_rest, p) for p in pts) / len(pts)
    d_new = sum(distance_to(new, p) for p in pts) / len(pts)
    pick = "old" if d_old < d_new - 0.02 else "new" if d_new < d_old - 0.02 else "both"
    return pick, d_old, d_new


def analyse(bags):
    rows, flips = [], []
    for run, (log, jsonl, keep_on) in RUNS.items():
        for leg in nav_legs(log, jsonl):
            bag = bags[leg["robot"]]
            plans = [(t, pts) for t, _, _, pts in bag["plan"]
                     if leg["t0"] - 0.5 <= t <= leg["t1"] + 0.5 and pts
                     and math.dist(pts[-1], leg["goal"]) < 0.06]
            traj = trajectory(bag)
            n_flip, n_off, n_kept_flip, prev, kept = 0, 0, 0, None, None
            for t, pts in plans:
                if prev is not None:
                    kind, start = first_look(prev, prev[-1], pts, pts[-1])
                    if kind == OTHER:
                        n_flip += 1
                        rest = prev[start:]
                        pick, d_old, d_new = followed(traj, t, rest, pts)
                        flips.append({"run": run, "robot": leg["robot"], "to": leg["to"],
                                      "t": kst(t), "old_m": round(length(rest), 2),
                                      "new_m": round(length(pts), 2), "followed": pick,
                                      "d_old": d_old, "d_new": d_new, "old": rest, "new": pts,
                                      "traj": [(x, y) for tt, x, y in traj
                                               if leg["t0"] <= tt <= leg["t1"]]})
                    elif kind == OFF:
                        n_off += 1
                prev = pts
                # keeper 규칙 재생: 받은 길 기준 뒤집힘(막힘 검사는 열림으로 둠)
                if kept is None:
                    kept = pts
                    continue
                kind, start = first_look(kept, kept[-1], pts, pts[-1])
                if kind == OTHER:
                    verdict = second_look(kept[start:], pts, True)
                    if verdict == SHORTER:
                        n_kept_flip += 1
                        kept = pts
                    else:
                        kept = kept[start:]
                else:
                    kept = pts
            rows.append({"run": run, "keeper": keep_on, "robot": leg["robot"], "to": leg["to"],
                         "start": kst(leg["t0"]), "s": round(leg["t1"] - leg["t0"], 1),
                         "plans": len(plans), "flips": n_flip, "off": n_off,
                         "kept_flips_est": n_kept_flip})
    return rows, flips


# ---- 그림: 뒤집힘 앞뒤 경로와 실제 궤적 ----
def plot_flips(flips, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if not flips:
        return
    fig, axes = plt.subplots(1, len(flips), figsize=(6 * len(flips), 4))
    for ax, f in zip(axes if len(flips) > 1 else [axes], flips):
        ax.plot(*zip(*f["old"]), color="#1f77b4", label=f"previous plan, rest {f['old_m']} m")
        ax.plot(*zip(*f["new"]), color="#d62728", label=f"new plan {f['new_m']} m")
        ax.plot(*zip(*f["traj"]), color="#555", lw=1, ls="--", label="driven (map->base)")
        ax.plot(*f["new"][0], "ko")
        ax.set_aspect("equal")
        ax.set_xlim(0, 2.0)
        ax.set_ylim(0, 1.0)
        ax.set_title(f"{f['run']} {f['robot']} -> {f['to']} {f['t']}")
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "flips_run1_pode1.png", dpi=90)
    plt.close(fig)


def main():
    bags = pickle.load(open(sys.argv[1], "rb"))
    out = Path(sys.argv[2])
    rows, flips = analyse(bags)
    plot_flips(flips, out)
    print("run keeper robot to start s plans flips off kept_flips_est")
    for r in rows:
        print(*r.values())
    print("\nflips:")
    for f in flips:
        print(f["run"], f["robot"], f["to"], f["t"], "old", f["old_m"], "new", f["new_m"],
              "followed", f["followed"], f"d_old {f['d_old']:.3f} d_new {f['d_new']:.3f}"
              if f["d_old"] is not None else "")
    with open(out / "keeper_legs.json", "w") as fh:
        json.dump(rows, fh, indent=1)


if __name__ == "__main__":
    main()
