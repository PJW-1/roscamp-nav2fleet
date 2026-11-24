"""작업 4 a+c+d 시드 실행의 CSV 표를 끝난 평가에서 만든다 (Isaac 불필요).

  python learning/store_arena/acd_tables.py <recipe runs dir> <acd24 runs dir> <out dir> [partial]

정책: 4b, ctrl, c (시드 1-3), acd200, acd (시드 1-3). 칸: 0.12 m/s 의 random·head-on·handover,
0.08·0.16 의 random·head-on (ctrl, c 는 0.12 만, c 는 handover 제외: measured()).
평가는 모두 play.py --episodes 1 --seed 7, 4096 env 라 같은 칸의 실행은 같은 에피소드를 env 번호로
짝짓는다.

cells.csv   정책 x 시드 x 칸마다 한 줄: 비율, 로봇과 간격 < 0.10 m, 분당 w 반전
paired.csv  같은 칸에서 acd 시드 s - 기준. 단일 체크포인트 기준은 시드별 짝 차이와 SE, "mean" 행은
            시드 평균 (SE sd/sqrt(3)). c 기준은 mean(acd) - mean(c) (SE sqrt(var_acd/3 + var_c/3)).
            noise: |diff| > 2 SE 이면 "outside", 아니면 "inside".
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


def measured(pol, cap, s):
    """설계상 평가한 칸: ctrl 과 c 는 0.12 만, c 는 handover 제외."""
    if pol in ("ctrl", "c") and cap != 0.12:
        return False
    return not (pol == "c" and s == "ho")


def same_episodes(da, db):
    """추적한 env 들이 같은 경로, 같은 자리의 움직이는 물체·로봇으로 시작하는지."""
    za, zb = np.load(da + "/rollouts.npz"), np.load(db + "/rollouts.npz")
    n = min(za["route"].shape[1], zb["route"].shape[1])  # --track 1024 (recipe) 또는 16 (여기)
    if n == 0:  # 추적한 것이 없으면 같은 에피소드라는 근거 없음
        return False
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
            if not measured(pol, cap, s):
                continue
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
