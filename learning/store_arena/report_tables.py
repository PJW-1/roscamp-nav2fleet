"""CSV tables for the recipe report (items 5, 6, 7) from finished eval runs; no Isaac needed.

  python learning/store_arena/report_tables.py <runs dir> <out dir> [item5] [item7] [item6]

Every eval is play.py --episodes 1 --seed 7 over 4096 envs, so two runs of the same set see the
same episodes and are paired by env id: d_i = success_a - success_b per episode, difference
mean(d), SE sd(d)/sqrt(n).
"""

import csv
import json
import os
import sys

import numpy as np
from ceiling import ceiling, load

SETS = ("rand", "head", "ho")
SET_NAMES = {"rand": "random", "head": "head-on", "ho": "handover"}
ITEM5 = [  # (row name, run tag, cap) -- e_<tag>_<set>
    ("ctrl", "ctrl", 0.12),
    ("ppo_k (start)", "k", 0.12),
    ("a (1x)", "a", 0.12),
    ("a3", "a3", 0.12),
    ("a10", "a10", 0.12),
    ("b", "b", 0.12),
    ("c", "c", 0.12),
    ("d", "d", 0.12),
    ("a1x+c", "combo_ac", 0.12),
    ("a1x+c+d", "combo_acd", 0.12),
    ("d", "d08", 0.08),
    ("d", "d16", 0.16),
    ("a1x+c+d", "combo_acd08", 0.08),
    ("a1x+c+d", "combo_acd16", 0.16),
]
ITEM7 = [("ppo_k", "k"), ("c", "c"), ("a1x+c", "combo_ac"), ("a1x+c+d", "combo_acd")]
ITEM6 = {"no intent": "noint", "intent": "int", "intent + priority": "intprio"}
GAP = "gap -inf-0.10 m"


def run_dir(r, tag, s):
    return f"{r}/e_{tag}_{s}"


def rates(d):
    j = json.load(open(d + "/eval.json"))
    return j, j["rates"]


def success(d):
    env, code, keys = load(d)
    return env, (code == keys.index("success")).astype(float)


def paired(da, db):
    ea, sa = success(da)
    eb, sb = success(db)
    if not np.array_equal(ea, eb):
        raise ValueError(f"{da} and {db} do not count the same envs")
    d = sa - sb
    return 100 * d.mean(), 100 * d.std(ddof=1) / np.sqrt(len(d))


def r4(x):
    return "" if x is None else round(float(x), 4)


def item5(r):
    rows = []
    for name, tag, cap in ITEM5:
        row = {"run": name, "tag": tag, "cap_mps": cap}
        for s in SETS:  # fixed column order; blanks where a set or a pairing does not apply
            for k in (
                "success",
                "collision",
                "timeout",
                "diff_vs_ctrl_pp",
                "diff_se_pp",
                "w_flips_per_min",
            ):
                row[f"{s}_{k}"] = ""
        for s in SETS:
            d = run_dir(r, tag, s)
            if not os.path.exists(d + "/eval.json"):
                continue
            j, rt = rates(d)
            row[f"{s}_success"] = rt["success"]
            row[f"{s}_collision"] = rt["collision"]
            row[f"{s}_timeout"] = rt["timeout"]
            if cap == 0.12 and tag != "ctrl":
                m, se = paired(d, run_dir(r, "ctrl", s))
                row[f"{s}_diff_vs_ctrl_pp"], row[f"{s}_diff_se_pp"] = r4(m), r4(se)
            row[f"{s}_w_flips_per_min"] = j["w_flips_per_min"]
        rows.append(row)
    return rows


def item7(r):
    rows = []
    for s in SETS:
        se, sc, _ = load(f"{r}/s_{s}")
        for name, tag in ITEM7:
            pe, pc, keys = load(run_dir(r, tag, s))
            c = ceiling(pe, pc, se, sc, keys)
            rows.append({"set": SET_NAMES[s], "policy": name, **c})
    return rows


def item6(r, partial=False):
    """partial: skip runs not evaluated yet (a preview while the queue runs)."""
    rows, pair_rows = [], []
    for s in ("rand", "head"):
        diffs = {}
        for name, v in ITEM6.items():
            for seed in (1, 2, 3):
                d = run_dir(r, f"i6_{v}_s{seed}", s)
                if partial and not os.path.exists(d + "/eval.json"):
                    continue
                j, rt = rates(d)
                g = j["by_robot_gap"][GAP]
                n_gap = sum(g[k] for k in ("success", "collision", "oob", "timeout"))
                rows.append(
                    {
                        "variant": name,
                        "seed": seed,
                        "set": SET_NAMES[s],
                        "success": rt["success"],
                        "collision": rt["collision"],
                        "timeout": rt["timeout"],
                        "gap_lt_0.10_success": g["success_rate"],
                        "gap_lt_0.10_episodes": n_gap,
                        "w_flips_per_min": j["w_flips_per_min"],
                    }
                )
                if v != "noint":
                    m, se = paired(d, run_dir(r, f"i6_noint_s{seed}", s))
                    diffs.setdefault(name, []).append(m)
                    pair_rows.append(
                        {
                            "variant": name,
                            "vs": "no intent",
                            "set": SET_NAMES[s],
                            "seed": seed,
                            "diff_pp": r4(m),
                            "se_pp": r4(se),
                        }
                    )
        for name, ds in diffs.items():
            if len(ds) < 2:  # no spread across seeds from one seed
                continue
            ds = np.array(ds)
            pair_rows.append(
                {
                    "variant": name,
                    "vs": "no intent",
                    "set": SET_NAMES[s],
                    "seed": "mean",
                    "diff_pp": r4(ds.mean()),
                    "se_pp": r4(ds.std(ddof=1) / np.sqrt(len(ds))),
                }
            )
    return rows, pair_rows


def write(path, rows):
    keys = []
    for row in rows:
        keys += [k for k in row if k not in keys]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {path} ({len(rows)} rows)")


if __name__ == "__main__":
    r, out = sys.argv[1], sys.argv[2]
    which = sys.argv[3:] or ["item5", "item7", "item6"]
    os.makedirs(out, exist_ok=True)
    if "item5" in which:
        write(f"{out}/item5.csv", item5(r))
    if "item7" in which:
        write(f"{out}/item7.csv", item7(r))
    if "item6" in which:
        rows, pair_rows = item6(r, partial="partial" in which)
        write(f"{out}/item6.csv", rows)
        write(f"{out}/item6_paired.csv", pair_rows)
