"""작업 4 Gazebo 표: RL+shim 4b 대 RL+shim a+c+d(acd2), 칸마다 판별 값과 두 표본 비교.

  python3 gz_results.py <gz_run.sh OUT> [dest]   (WSL, ROS install을 source한 셸)

구간마다: 성공, 벽·장애물·로봇 접촉(item 8 leg_row), 몸 최소 간격, 분당 w 부호 반전, 이른
넘김(early_count).
시나리오마다 acd2 - 4b: 성공 차이는 Wald SE와 양측 Fisher p, 시간·반전·간격·이른 넘김은 Welch (차이,
SE).
"""

import csv
import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "nav_bench"))
sys.path.insert(0, str(ROOT / "docs" / "rl_shim_recipe_2026-11-22"))
import debug_extract as de  # noqa: E402
import metrics as mt  # noqa: E402
import movers as mv  # noqa: E402
from item8_results import leg_row  # noqa: E402

A, B = "FollowPathRLShim", "FollowPathRLShimACD2"
NAMES = {A: "RL+shim 4b", B: "RL+shim acd2"}
BATCHES = (("q2e3", ROOT / "results/2026-11-22/compare_012/q2e3.yaml"),
           ("main", ROOT / "tools/nav_bench/scenarios.yaml"))
EMPTY = {"log": [], "costmap": [], "odom": []}


def legs(out):
    body, statics = mt.footprint(), mt.arena_rects()
    rows = []
    for batch, cfg_path in BATCHES:
        cfg = mv.load(cfg_path)
        for p in sorted((out / batch / "legs").glob("*.json")):
            rec = json.loads(p.read_text())
            m = mt.leg_metrics(rec, cfg, body, statics)
            r = leg_row(rec, cfg, body, statics)
            events, _ = de.leg_events(rec, EMPTY, body)
            minutes = (rec["t_end"] - rec["t_start"]) / 60.0
            rows.append({
                "scenario": "q2e3" if batch == "q2e3" else rec["scenario"],
                "controller": rec["controller"], "repeat": rec["repeat"], "ok": r["ok"],
                "status": r["status"], "time_s": round(r["time_s"], 2),
                "wall_obstacle_contacts": r["walls"], "robot_contacts": r["robot"],
                "body_min_m": None if r["body_min"] is None else round(r["body_min"], 4),
                "flips": m["doridori"],
                "flips_per_min": round(m["doridori"] / minutes, 2) if minutes > 0 else None,
                "early_handovers": sum(1 for e in events if e["early"])})
    return rows


def fisher_p(k1, n1, k2, n2):
    """2x2 표(두 쪽 각각 n 가운데 성공 k)의 양측 Fisher 정확 p."""
    total, ks = n1 + n2, k1 + k2
    def pr(x):
        return math.comb(n1, x) * math.comb(n2, ks - x) / math.comb(total, ks)
    p0 = pr(k1)
    lo, hi = max(0, ks - n2), min(ks, n1)
    return min(1.0, sum(pr(x) for x in range(lo, hi + 1) if pr(x) <= p0 * (1 + 1e-9)))


def welch(a, b):
    d, se, _ = mt.welch(a, b)
    return (None, None) if d is None else (round(d, 3), round(se, 3))


def mean(v):
    v = [x for x in v if x is not None]
    return round(statistics.fmean(v), 3) if v else None


def summarize(rows):
    cells, comps = [], []
    for sc in sorted({r["scenario"] for r in rows}):
        by = {}
        for ctl in (A, B):
            g = [r for r in rows if r["scenario"] == sc and r["controller"] == ctl]
            if not g:
                continue
            by[ctl] = g
            ok = [r for r in g if r["ok"]]
            mins = [r["body_min_m"] for r in g if r["body_min_m"] is not None]
            cells.append({
                "scenario": sc, "name": NAMES[ctl], "runs": len(g), "successes": len(ok),
                "collision_runs": sum(1 for r in g
                                      if r["wall_obstacle_contacts"] + r["robot_contacts"] > 0),
                "timeouts": sum(1 for r in g if r["status"] == "TIMEOUT"),
                "mean_time_s": mean([r["time_s"] for r in ok]),
                "flips_per_min": mean([r["flips_per_min"] for r in g]),
                "min_clearance_m": min(mins) if mins else None,
                "median_leg_clearance_m": round(statistics.median(mins), 4) if mins else None,
                "early_handovers_per_run": mean([r["early_handovers"] for r in g])})
        if len(by) < 2:
            continue
        a, b = by[A], by[B]
        ka, kb = sum(r["ok"] for r in a), sum(r["ok"] for r in b)
        pa, pb = ka / len(a), kb / len(b)
        se = math.sqrt(pa * (1 - pa) / len(a) + pb * (1 - pb) / len(b))
        comps.append({
            "scenario": sc, "metric": "success_pp", "diff_acd2_minus_4b": round(100 * (pb - pa), 1),
            "se": round(100 * se, 1), "fisher_p": round(fisher_p(kb, len(b), ka, len(a)), 3)})
        for metric, key, sel in (("time_s (successful)", "time_s", True),
                                 ("flips_per_min", "flips_per_min", False),
                                 ("leg_min_clearance_m", "body_min_m", False),
                                 ("early_handovers", "early_handovers", False)):
            xa = [r[key] for r in a if r["ok"] or not sel]
            xb = [r[key] for r in b if r["ok"] or not sel]
            d, s = welch(xb, xa)
            comps.append({"scenario": sc, "metric": metric, "diff_acd2_minus_4b": d, "se": s,
                          "fisher_p": ""})
    return cells, comps


def write(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def table(rows):
    if not rows:
        return "(none)"
    keys = list(rows[0])
    out = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
    out += ["| " + " | ".join("" if r[k] is None else str(r[k]) for k in keys) + " |"
            for r in rows]
    return "\n".join(out)


def main():
    out = Path(sys.argv[1])
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent
    rows = legs(out)
    cells, comps = summarize(rows)
    write(dest / "gz_legs.csv", rows)
    write(dest / "gz_cells.csv", cells)
    write(dest / "gz_compare.csv", comps)
    print(table(cells))
    print()
    print(table(comps))


if __name__ == "__main__":
    main()
