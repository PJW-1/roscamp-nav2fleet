"""goal-reach 실행 그래프 (순수 Python + matplotlib, Isaac 불필요).

  curve <out.png> <label>=<run_dir>/metrics.csv [...]  학습 시간/스텝별 성공률
  traj <out.png> <play_dir>/rollouts.npz               추적 env 의 상단 경로
  anim <out.mp4|.gif> <play_dir>/rollouts.npz [env]    env 하나의 상단 애니메이션
"""

import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def read_metrics(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2)


def curve(out, specs):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), dpi=130)
    for i, spec in enumerate(specs):
        label, path = spec.split("=", 1)
        m = read_metrics(path)
        c = SERIES[i % len(SERIES)]
        axes[0].plot(m["wall_s"] / 60.0, 100 * m["success_rate"], color=c, lw=2, label=label)
        axes[1].plot(m["env_steps"] / 1e6, 100 * m["success_rate"], color=c, lw=2, label=label)
    # 상승은 첫 1분 안에 일어남: 처음 2분 / 10 M 스텝만 표시 (나머지는 CSV)
    axes[0].set_xlim(0, min(axes[0].get_xlim()[1], 2.0))
    axes[1].set_xlim(0, min(axes[1].get_xlim()[1], 10.0))
    axes[0].set_xlabel("training wall time [min] (env build excluded; first 2 min)", color=INK)
    axes[1].set_xlabel("env steps [million] (20 Hz control steps x envs; first 10 M)", color=INK)
    for ax in axes:
        ax.set_ylabel("episodes ending in success [%]", color=INK)
        ax.set_ylim(-2, 102)
        style(ax)
    if len(specs) > 1:
        axes[1].legend(frameon=False, loc="lower right", labelcolor=INK)
    fig.suptitle("Pinky goal reach on Newton: success rate during PPO training", color=INK)
    fig.tight_layout()
    fig.savefig(out)
    print("wrote", out)


def traj(out, npz):
    d = np.load(npz)
    xy, goal, done = d["xy"], d["goal"], d["done"]
    T, K = xy.shape[:2]
    fig, ax = plt.subplots(figsize=(6, 6), dpi=130)
    for k in range(K):
        c = SERIES[k % len(SERIES)]
        cuts = np.flatnonzero(done[:, k]) + 1
        start = 0
        for end in cuts:  # 끝난 에피소드만, 끝에 진행 중인 것은 뺌
            seg = xy[start:end, k]
            if len(seg) > 1:
                ax.plot(seg[:, 0], seg[:, 1], color=c, lw=1.2, alpha=0.9)
                g = goal[start, k]
                ax.add_patch(plt.Circle(g, float(d["success_radius"]), fill=False, color=c, lw=1.2))
            start = end
    ax.plot(0, 0, marker="o", color=INK, ms=6)
    ax.set_aspect("equal")
    ax.set_xlim(-2.3, 2.3)
    ax.set_ylim(-2.3, 2.3)
    ax.set_xlabel("x [m]", color=INK)
    ax.set_ylabel("y [m]", color=INK)
    ax.set_title(
        f"trained policy, {K} envs: paths from the start (black dot) to goals (circles = 0.05 m)",
        color=INK,
        fontsize=9,
    )
    style(ax)
    fig.tight_layout()
    fig.savefig(out)
    print("wrote", out)


def anim(out, npz, k=0):
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

    d = np.load(npz)
    xy, yaw, goal, done = d["xy"][:, k], d["yaw"][:, k], d["goal"][:, k], d["done"][:, k]
    dt = float(d["dt"])
    fig, ax = plt.subplots(figsize=(5, 5), dpi=110)
    style(ax)
    ax.set_aspect("equal")
    ax.set_xlim(-2.3, 2.3)
    ax.set_ylim(-2.3, 2.3)
    (path,) = ax.plot([], [], color=SERIES[0], lw=1.5)
    (body,) = ax.plot([], [], color=INK, lw=2)
    gc = plt.Circle((0, 0), float(d["success_radius"]), color=SERIES[7])
    ax.add_patch(gc)
    txt = ax.text(-2.2, 2.05, "", color=INK, fontsize=9)
    starts = np.r_[0, np.flatnonzero(done) + 1]

    def frame(t):
        s = starts[starts <= t].max()
        p = xy[s : t + 1]
        path.set_data(p[:, 0], p[:, 1])
        c, sn = np.cos(yaw[t]), np.sin(yaw[t])
        L, W = 0.06, 0.055  # 발자국 그림의 반길이 / 반폭
        pts = np.array([[L, 0], [-L, W], [-L, -W], [L, 0]]) @ np.array([[c, sn], [-sn, c]]) + xy[t]
        body.set_data(pts[:, 0], pts[:, 1])
        gc.center = tuple(goal[t])
        txt.set_text(f"t = {t * dt:5.1f} s   dist = {np.linalg.norm(goal[t] - xy[t]):.2f} m")
        return path, body, gc, txt

    a = FuncAnimation(fig, frame, frames=len(xy), blit=True)
    w = FFMpegWriter(fps=round(1 / dt)) if out.endswith(".mp4") else PillowWriter(fps=round(1 / dt))
    a.save(out, writer=w)
    print("wrote", out)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "curve":
        curve(sys.argv[2], sys.argv[3:])
    elif cmd == "traj":
        traj(sys.argv[2], sys.argv[3])
    elif cmd == "anim":
        anim(sys.argv[2], sys.argv[3], int(sys.argv[4]) if len(sys.argv) > 4 else 0)
