#!/usr/bin/env python3
"""뒤집힘으로 센 경로 쌍과 그 시간대 실제 궤적을 지도 위에 그린다.

사용: python3 plot_flips.py 판폴더 출력.png [--robot podeN] [--t0 초 --t1 초]
시간은 sim 초. 기본은 뒤집힘이 가장 많은 로봇의 30초 창.
"""
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

import flip_metrics as fm  # noqa: E402

MAP_YAML = fm.REPO / "sim/fleet_gz_sim/maps/arena_v7_a.yaml"
WINDOW_S = 30.0
# 그림 글자는 영어(한글 글꼴이 없는 PC가 있다)


def pick_window(times, width=WINDOW_S):
    """뒤집힘이 가장 많이 든 width초 창의 시작 시각."""
    best, best_n = times[0], 0
    for t in times:
        n = sum(t <= u < t + width for u in times)
        if n > best_n:
            best, best_n = t, n
    return best - 2.0


def draw_map(ax):
    meta = yaml.safe_load(MAP_YAML.read_text())
    img = plt.imread(MAP_YAML.parent / meta["image"])
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    h, w = img.shape[:2]
    ax.imshow(img, cmap="gray", origin="upper", extent=(ox, ox + w * res, oy, oy + h * res))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("out")
    ap.add_argument("--robot")
    ap.add_argument("--t0", type=float)
    ap.add_argument("--t1", type=float)
    a = ap.parse_args()

    truth, _, _, bag = fm.load_run_data(a.run)
    if bag is None:
        raise SystemExit("bag이 없다")
    flips, _, _ = fm.collect_flips(bag)
    robot = a.robot or max(fm.ROBOTS, key=lambda r: len(flips[r]))
    mine = flips[robot]
    if not mine and (a.t0 is None or a.t1 is None):
        raise SystemExit(f"{robot}에 뒤집힘이 없다")
    t0 = a.t0 if a.t0 is not None else pick_window([f["t"] for f in mine])
    t1 = a.t1 if a.t1 is not None else t0 + WINDOW_S + 2.0
    sel = [f for f in mine if t0 <= f["t"] <= t1]

    fig, ax = plt.subplots(figsize=(10, 8))
    draw_map(ax)
    for i, f in enumerate(sel):
        for pts, color, label in ((f["prev"], "tab:blue", "previous plan"), (f["new"], "tab:red", "new plan")):
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=color, lw=1.2, alpha=0.5, label=label if i == 0 else None)
    t, x, y, _ = truth[robot]
    m = (t >= t0) & (t <= t1)
    ax.plot(x[m], y[m], color="black", lw=2, label=f"{robot} truth")
    if m.any():
        ax.plot(x[m][0], y[m][0], "go", ms=9, label="start")
        ax.plot(x[m][-1], y[m][-1], "ks", ms=8, label="end")
    xs = np.concatenate([x[m]] + [np.array([p[0] for p in f["new"]]) for f in sel]) if m.any() else x
    ys = np.concatenate([y[m]] + [np.array([p[1] for p in f["new"]]) for f in sel]) if m.any() else y
    ax.set_xlim(xs.min() - 0.15, xs.max() + 0.15)
    ax.set_ylim(ys.min() - 0.15, ys.max() + 0.15)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{Path(a.run).name} {robot} t={t0:.0f}~{t1:.0f}s, flips {len(sel)}")
    ax.legend(loc="upper right")
    fig.savefig(a.out, dpi=130, bbox_inches="tight")
    print(f"{a.out}: {robot} t={t0:.1f}~{t1:.1f}s flips {len(sel)}")


if __name__ == "__main__":
    main()
