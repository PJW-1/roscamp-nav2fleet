import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parent
SERIES = {"rand": ("무작위", "#2a78d6", "o"), "head": ("마주 오기", "#eb6834", "s"),
          "ho": ("인계 출발", "#1baf7a", "^")}
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

plt.rcParams.update({
    "font.family": "Malgun Gothic", "axes.unicode_minus": False, "font.size": 10,
    "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.facecolor": SURFACE, "figure.facecolor": SURFACE, "axes.titlecolor": INK,
})


def rows(name):
    path = HERE / name
    if not path.is_file():
        return None
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def tidy(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


# ---- 5번: 대조군 대비 짝지은 성공 차이 ----
def item5_diff(data):
    runs = [r for r in data if r["cap_mps"] == "0.12" and r["rand_diff_vs_ctrl_pp"]]
    fig, ax = plt.subplots(figsize=(7.2, 0.42 * len(runs) + 1.4))
    offsets = {"rand": -0.22, "head": 0.0, "ho": 0.22}
    for key, (label, color, marker) in SERIES.items():
        ys, xs, es = [], [], []
        for i, r in enumerate(runs):
            if r[f"{key}_diff_vs_ctrl_pp"]:
                ys.append(i + offsets[key])
                xs.append(float(r[f"{key}_diff_vs_ctrl_pp"]))
                es.append(float(r[f"{key}_diff_se_pp"]))
        ax.errorbar(xs, ys, xerr=es, fmt=marker, color=color, ecolor=color, elinewidth=2,
                    capsize=0, markersize=8, markeredgecolor=SURFACE, markeredgewidth=1.5,
                    label=label)
    ax.axvline(0, color=MUTED, linewidth=1)
    ax.set_yticks(range(len(runs)), [r["run"] for r in runs])
    ax.invert_yaxis()
    ax.set_xlabel("대조군 대비 성공 차이 (%p, 막대는 ±1 SE)")
    ax.set_title("5번 후보: 0.12 m/s, 4096 에피소드 짝지은 비교", loc="left", pad=26)
    ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False, ncols=3)
    tidy(ax)
    fig.tight_layout()
    fig.savefig(HERE / "item5_success_diff.png", dpi=150)
    plt.close(fig)


# ---- 5번: 분당 w 부호 바뀜 ----
def item5_flips(data):
    runs = [r for r in data if r["cap_mps"] == "0.12"]
    fig, ax = plt.subplots(figsize=(7.2, 0.36 * len(runs) + 1.2))
    values = [float(r["rand_w_flips_per_min"]) for r in runs]
    ax.barh(range(len(runs)), values, height=0.6, color=SERIES["rand"][1])
    for i, v in enumerate(values):
        ax.text(v + 1, i, f"{v:.0f}", va="center", color=INK)
    ax.set_yticks(range(len(runs)), [r["run"] for r in runs])
    ax.invert_yaxis()
    ax.set_xlabel("분당 w 부호 바뀜 (무작위 세트, 낮을수록 덜 흔들림)")
    ax.set_title("5번 후보의 흔들림", loc="left")
    tidy(ax)
    fig.tight_layout()
    fig.savefig(HERE / "item5_flips.png", dpi=150)
    plt.close(fig)


# ---- 6번: 변형 x seed 성공률 ----
VARIANTS = [("no intent", "의도 없음"), ("intent", "의도"), ("intent + priority", "의도 + 우선순위")]
SETS6 = [("random", "무작위", "#2a78d6", "o"), ("head-on", "마주 오기", "#eb6834", "s")]


def item6_success(data):
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.6))
    for ax, (key, label, color, marker) in zip(axes, SETS6):
        for i, (variant, _) in enumerate(VARIANTS):
            vals = [float(r["success"]) * 100 for r in data
                    if r["variant"] == variant and r["set"] == key]
            if not vals:
                continue
            xs = [i + (j - (len(vals) - 1) / 2) * 0.12 for j in range(len(vals))]
            ax.scatter(xs, vals, s=40, color=color, marker=marker, edgecolors=SURFACE,
                       linewidths=1.5, zorder=3)
            mean = sum(vals) / len(vals)
            ax.hlines(mean, i - 0.28, i + 0.28, color=INK, linewidth=2, zorder=2)
            ax.text(i + 0.32, mean, f"{mean:.1f}", va="center", color=INK, fontsize=9)
        ax.set_xticks(range(len(VARIANTS)), [v[1] for v in VARIANTS])
        ax.set_xlim(-0.5, len(VARIANTS) - 0.3)
        ax.set_title(f"{label} 성공률 (%)", loc="left")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.grid(axis="y", color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
    fig.suptitle("6번 의도 비교: 점은 학습 seed 하나, 가로선은 seed 평균", x=0.01, ha="left",
                 color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(HERE / "item6_success.png", dpi=150)
    plt.close(fig)


# ---- 6번: 의도 없음 대비 짝지은 차이 ----
def item6_paired(data):
    groups = [(v, k) for v, _ in VARIANTS[1:] for k, *_ in SETS6
              if any(r["variant"] == v and r["set"] == k for r in data)]
    if not groups:
        return
    fig, ax = plt.subplots(figsize=(7.2, 0.62 * len(groups) + 1.6))
    names = dict(VARIANTS)
    styles = {k: (label, color, marker) for k, label, color, marker in SETS6}
    shown = set()
    for i, (variant, key) in enumerate(groups):
        label, color, marker = styles[key]
        rows_ = [r for r in data if r["variant"] == variant and r["set"] == key]
        seeds = [r for r in rows_ if r["seed"] != "mean"]
        for j, r in enumerate(seeds):
            y = i - 0.24 + j * 0.12
            ax.errorbar(float(r["diff_pp"]), y, xerr=float(r["se_pp"]), fmt=marker, color=color,
                        alpha=0.45, markersize=5, elinewidth=1, capsize=0)
        for r in rows_:
            if r["seed"] == "mean":
                ax.errorbar(float(r["diff_pp"]), i + 0.22, xerr=float(r["se_pp"]), fmt=marker,
                            color=color, markersize=9, elinewidth=2.5, capsize=0,
                            markeredgecolor=SURFACE, markeredgewidth=1.5,
                            label=None if key in shown else label)
                shown.add(key)
    ax.axvline(0, color=MUTED, linewidth=1)
    ax.set_yticks(range(len(groups)), [f"{names[v]} · {styles[k][0]}" for v, k in groups])
    ax.invert_yaxis()
    ax.set_xlabel("의도 없음 대비 성공 차이 (%p)\n"
                  "흐린 점: seed별(막대 ±1 에피소드 SE) · 진한 점: seed 평균(막대 ±1 seed 사이 SE)")
    ax.set_title("6번 짝지은 차이", loc="left", pad=26)
    if shown:
        ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False, ncols=2)
    tidy(ax)
    fig.tight_layout()
    fig.savefig(HERE / "item6_paired.png", dpi=150)
    plt.close(fig)


# ---- 8번: Gazebo 다섯 컨트롤러 ----
CONTROLLERS = [("FollowPathGoal", "RPP", "#2a78d6", "o"), ("FollowPathMPPI", "MPPI", "#eb6834", "s"),
               ("FollowPathRLShim", "RL+shim 4b", "#1baf7a", "^"),
               ("FollowPathRLShimC", "RL+shim c", "#eda100", "D"),
               ("FollowPathRLShimACD", "RL+shim acd", "#e87ba4", "v")]
CELLS8 = [("item8_legs_q2e3.csv", "a_hall", "Q2 → E-3"), ("item8_legs_main.csv", "a_hall", "a_hall"),
          ("item8_legs_main.csv", "e_corridor", "e_corridor"), ("item8_legs_main.csv", "d_robot", "d_robot"),
          ("item8_legs_main.csv", "c_moving", "c_moving")]


def item8_compare():
    legs = {name: rows(name) for name in {c[0] for c in CELLS8}}
    if not all(legs.values()):
        return
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.2), sharey=True)
    for j, (key, label, color, marker) in enumerate(CONTROLLERS):
        for i, (name, scen, _) in enumerate(CELLS8):
            rs = [r for r in legs[name] if r["scenario"] == scen and r["controller"] == key]
            ok = [r for r in rs if r["ok"] == "True"]
            y = i + (j - 2) * 0.14
            flips = sum(float(r["doridori"]) for r in rs) / sum(float(r["time_s"]) for r in rs) * 60
            t = sum(float(r["time_s"]) for r in ok) / len(ok)
            for ax, x in zip(axes, (flips, t)):
                ax.scatter(x, y, s=42, color=color, marker=marker, edgecolors=SURFACE, linewidths=1.5,
                           zorder=3, label=label if i == 0 and ax is axes[0] else None)
    axes[0].set_yticks(range(len(CELLS8)), [c[2] for c in CELLS8])
    axes[0].invert_yaxis()
    axes[0].set_xlabel("분당 w 부호 바뀜(도리도리, 낮을수록 덜 흔들림)")
    axes[1].set_xlabel("성공 판의 평균 시간 (sim 초)")
    axes[0].set_title("8번 Gazebo: 흔들림", loc="left", pad=30)
    axes[1].set_title("8번 Gazebo: 시간", loc="left", pad=30)
    for ax in axes:
        tidy(ax)
    axes[0].legend(loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False, ncols=5,
                   columnspacing=1.0, handletextpad=0.3)
    fig.tight_layout()
    fig.savefig(HERE / "item8_compare.png", dpi=150)
    plt.close(fig)


def main():
    item5 = rows("item5.csv")
    if item5:
        item5_diff(item5)
        item5_flips(item5)
    item6 = rows("item6.csv")
    if item6:
        item6_success(item6)
    paired = rows("item6_paired.csv")
    if paired:
        item6_paired(paired)
    item8_compare()


if __name__ == "__main__":
    main()
