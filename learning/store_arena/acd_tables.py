"""CSV tables for the a+c+d seed runs (work 4, 2026-11-24) from finished eval runs; no Isaac needed.

  python learning/store_arena/acd_tables.py <recipe runs dir> <acd24 runs dir> <out dir> [partial]

Policies: 4b (ppo_k), ctrl (item 5 control), c (item 6 "no intent", seeds 1-3), acd200 (item 5
a+c+d, 200 iterations, seed 1) and acd (this work, seeds 1-3, 500 iterations). Cells: random,
head-on and handover at 0.12 m/s; random and head-on at 0.08 and 0.16 (policies without the
speed_cap input ran there with play.py --capped). Every eval is play.py --episodes 1 --seed 7 over
4096 envs, so runs of the same cell see the same episodes (same_episodes checks the tracked envs)
and pair by env id.

cells.csv   one row per policy x seed x cell: rates, gap < 0.10 m to a robot, w flips per minute
paired.csv  acd seed s minus a baseline in the same cell. Against a single checkpoint (4b, ctrl,
            acd200): per seed the paired episode difference and its SE; row "mean" is the seed mean
            with SE sd(seed diffs)/sqrt(3). Against c (3 seeds of its own): row "mean" is
            mean(acd) - mean(c) with SE sqrt(var_acd/3 + var_c/3) over seed success rates.
            noise: "outside" when |diff| > 2 SE, else "inside" (decided before the results, B84).
"""

import json
import os
import sys

import numpy as np
from report_tables import paired, write

CELLS = [(0.12, "rand"), (0.12, "head"), (0.12, "ho")] + [
    (c, s) for c in (0.08, 0.16) for s in ("rand", "head")
]
SEEDS = (1, 2, 3)
SINGLE = ("4b", "ctrl", "acd200")
GAP = "gap -inf-0.10 m"


def cap_tag(cap):
    return {0.08: "08", 0.12: "", 0.16: "16"}[cap]


def run_dir(rec, acd, pol, seed, cap, s):
    ct = cap_tag(cap)
    if pol == "acd":
        return f"{acd}/e_acd_s{seed}_{ct}{s}"
    if pol == "acd200":
        return f"{rec}/e_combo_acd{ct}_{s}"
    if pol == "c":
        if cap == 0.12 and s != "ho":
            return f"{rec}/e_i6_noint_s{seed}_{s}"
        return f"{acd}/e_c_s{seed}_{ct}{s}"
    tag = {"4b": "k", "ctrl": "ctrl"}[pol]
    return f"{rec}/e_{tag}_{s}" if cap == 0.12 else f"{acd}/e_{tag}_{ct}{s}"


def seeds_of(pol):
    return SEEDS if pol in ("c", "acd") else (None,)


def same_episodes(da, db):
    """The tracked envs start the same route with the movers and robots in the same places."""
    za, zb = np.load(da + "/rollouts.npz"), np.load(db + "/rollouts.npz")
    n = min(za["route"].shape[1], zb["route"].shape[1])  # --track 1024 (recipe) or 16 (here)
    for k in ("route", "mover_xy", "robot_xy"):
        if not np.array_equal(za[k][0][:n], zb[k][0][:n]):
            return False
    return True


def cell_row(d, pol, seed, cap, s):
    j = json.load(open(d + "/eval.json"))
    rt, g = j["rates"], j["by_robot_gap"][GAP]
    return {
        "policy": pol,
        "seed": "" if seed is None else seed,
        "cap_mps": cap,
        "set": s,
        "success": rt["success"],
        "collision": rt["collision"],
        "timeout": rt["timeout"],
        "gap_lt_0.10_success": g["success_rate"],
        "gap_lt_0.10_episodes": sum(g[k] for k in ("success", "collision", "oob", "timeout")),
        "w_flips_per_min": j["w_flips_per_min"],
        "w_flips_per_min_success": j["w_flips_per_min_success"],
        "success_time_s_mean": j["success_time_s_mean"],
        "capped": j.get("capped", False),
    }


def r4(x):
    return round(float(x), 4)


def noise(m, se):
    return "outside" if abs(m) > 2 * se else "inside"


def tables(rec, acd, partial=False):
    cells, pairs, checked = [], [], []
    for cap, s in CELLS:
        have = {}
        for pol in ("4b", "ctrl", "c", "acd200", "acd"):
            for seed in seeds_of(pol):
                d = run_dir(rec, acd, pol, seed, cap, s)
                if partial and not os.path.exists(d + "/eval.json"):
                    continue
                cells.append(cell_row(d, pol, seed, cap, s))
                have[(pol, seed)] = d
        acd_seeds = [k[1] for k in have if k[0] == "acd"]
        if not acd_seeds:
            continue
        ref = have[("acd", acd_seeds[0])]
        for d in have.values():
            if d != ref and not same_episodes(ref, d):
                raise ValueError(f"{ref} and {d} do not start the same episodes")
            checked.append(d)
        for base in SINGLE:
            if (base, None) not in have:
                continue
            diffs = []
            for seed in acd_seeds:
                m, se = paired(have[("acd", seed)], have[(base, None)])
                diffs.append(m)
                pairs.append(
                    {
                        "cap_mps": cap,
                        "set": s,
                        "vs": base,
                        "seed": seed,
                        "diff_pp": r4(m),
                        "se_pp": r4(se),
                        "noise": "",
                    }
                )
            if len(diffs) > 1:
                m, se = np.mean(diffs), np.std(diffs, ddof=1) / np.sqrt(len(diffs))
                pairs.append(
                    {
                        "cap_mps": cap,
                        "set": s,
                        "vs": base,
                        "seed": "mean",
                        "diff_pp": r4(m),
                        "se_pp": r4(se),
                        "noise": noise(m, se),
                    }
                )
        c_seeds = [k[1] for k in have if k[0] == "c"]
        for seed in acd_seeds:
            if seed in c_seeds:
                m, se = paired(have[("acd", seed)], have[("c", seed)])
                pairs.append(
                    {
                        "cap_mps": cap,
                        "set": s,
                        "vs": "c",
                        "seed": seed,
                        "diff_pp": r4(m),
                        "se_pp": r4(se),
                        "noise": "",
                    }
                )
        if len(acd_seeds) > 1 and len(c_seeds) > 1:
            rate = {
                r["policy"] + str(r["seed"]): r["success"]
                for r in cells
                if r["cap_mps"] == cap and r["set"] == s
            }
            a = 100 * np.array([rate[f"acd{k}"] for k in acd_seeds])
            c = 100 * np.array([rate[f"c{k}"] for k in c_seeds])
            m = a.mean() - c.mean()
            se = np.sqrt(a.var(ddof=1) / len(a) + c.var(ddof=1) / len(c))
            pairs.append(
                {
                    "cap_mps": cap,
                    "set": s,
                    "vs": "c",
                    "seed": "mean",
                    "diff_pp": r4(m),
                    "se_pp": r4(se),
                    "noise": noise(m, se),
                }
            )
    return cells, pairs, checked


if __name__ == "__main__":
    rec, acd, out = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out, exist_ok=True)
    cells, pairs, checked = tables(rec, acd, partial="partial" in sys.argv[4:])
    print(f"same episodes checked in {len(checked)} runs")
    write(f"{out}/cells.csv", cells)
    write(f"{out}/paired.csv", pairs)
