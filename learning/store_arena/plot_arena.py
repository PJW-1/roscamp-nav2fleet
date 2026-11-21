"""Plots of arena_check.py output (no Isaac needed).

  scene:   top-down view of the yaml boxes the env was built from, the robot of env 0, and its
           simulated lidar hits next to the 2D ray cast of the same boxes
  contact: per scripted drive, contact force per body against the gap between the robot centre
           and the obstacle face, and a small CSV of the first step above the collision force

  curves:  training curves of a chunked run (run_train.sh): success, collision and timeout rate
           and success episode time against training minutes, chunks joined end to end
  traj:    top-down paths of the tracked envs of play.py with their global paths and boxes
  routes:  the global paths route_spec plans (ArenaSpec defaults): every node pair without boxes,
           and a few layouts with boxes

  python learning/store_arena/plot_arena.py scene <scene.json> <out.png>
  python learning/store_arena/plot_arena.py contact <contact_push.json> <out.png> <out.csv>
  python learning/store_arena/plot_arena.py curves <out.png> <run_dir> [<out.csv>]
  python learning/store_arena/plot_arena.py traj <out.png> <rollouts.npz> [episodes]
  python learning/store_arena/plot_arena.py routes <out.png>
  python learning/store_arena/plot_arena.py anim <out.gif> <rollouts.npz> [env]
"""

import csv
import glob
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

COLORS = {"wall": "#8c8c99", "shelf": "#c08a4d"}
BODY_R = 0.085  # robot body radius used by the fleet map clearance rules [m]


def plot_scene(src: str, out: str) -> None:
    with open(src, encoding="utf-8") as fh:
        rep = json.load(fh)
    fig, ax = plt.subplots(figsize=(9, 5.2), dpi=120)
    for b in rep["boxes"]:
        ax.add_patch(
            Rectangle(
                (b["x0"], b["y0"]),
                b["x1"] - b["x0"],
                b["y1"] - b["y0"],
                color=COLORS[b["kind"]],
                alpha=0.9,
                lw=0,
            )
        )
    e0 = rep["env0"]
    x, y = e0["pos"]
    yaw = e0["yaw"]
    ax.add_patch(Circle((x, y), BODY_R, fill=False, color="#1f5fbf", lw=1.5))
    ax.annotate(
        "",
        xy=(x + 0.12 * math.cos(yaw), y + 0.12 * math.sin(yaw)),
        xytext=(x, y),
        arrowprops={"arrowstyle": "->", "color": "#1f5fbf", "lw": 1.5},
    )
    lx, ly = x - 0.017 * math.cos(yaw), y - 0.017 * math.sin(yaw)
    n = len(e0["lidar"])
    for key, style in (("lidar_expected", {"c": "#222222", "s": 10, "marker": "x", "lw": 0.6}),
                       ("lidar", {"c": "#d62728", "s": 4})):
        pts = [
            (lx + r * math.cos(yaw + math.radians(-180 + i * 360 / n)),
             ly + r * math.sin(yaw + math.radians(-180 + i * 360 / n)))
            for i, r in enumerate(e0[key])
            if math.isfinite(r)
        ]
        label = "simulated lidar hits" if key == "lidar" else "2D ray cast of the yaml boxes"
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], label=label, **style)
    err = rep["scan_vs_yaml_all_envs"]
    ax.set_title(
        f"store arena, env 0 of {rep['num_envs']}; lidar of all envs vs yaml ray cast: "
        f"{err['rays_off_both_2mm_casts']} of {err['rays_compared']} rays off by > 1 cm "
        "(boxes +-2 mm)",
        fontsize=10,
    )
    ax.set_xlim(-0.1, 2.1)
    ax.set_ylim(-0.1, 1.1)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (map frame = env frame)")
    ax.set_ylabel("y [m]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(out)
    print(f"wrote {out}")


def plot_contact(src: str, out: str, out_csv: str, threshold: float = 1.0) -> None:
    with open(src, encoding="utf-8") as fh:
        rep = json.load(fh)
    cases = rep["cases"]
    fig, axes = plt.subplots(2, 4, figsize=(13, 5.6), dpi=110, sharey=True)
    rows = []
    for ax, (name, log) in zip(axes.flat, cases.items(), strict=True):
        t = [r["t"] for r in log]
        for body in rep["contact_bodies"]:
            f = [r["forces_N"][body] for r in log]
            if max(f) > 0:
                ax.plot(t, f, label=body, lw=1.2)
        gap = ax.twinx()
        gap.plot(t, [r["centre_to_face_m"] for r in log], color="#555555", lw=0.8, ls="--")
        gap.plot(t, [r["lidar_min_m"] for r in log], color="#2ca02c", lw=0.8, ls=":")
        gap.set_ylim(0, 0.35)
        ax.set_title(name, fontsize=8)
        ax.set_yscale("symlog", linthresh=1.0)
        ax.axhline(threshold, color="red", lw=0.6)
        ax.legend(fontsize=6, loc="upper left")
        first = next((r for r in log if max(r["forces_N"].values()) > threshold), None)
        if first:
            body = max(first["forces_N"], key=first["forces_N"].get)
            rows.append(
                {
                    "case": name,
                    "t_s": first["t"],
                    "body": body,
                    "force_N": first["forces_N"][body],
                    "centre_to_face_m": first["centre_to_face_m"],
                    "lidar_min_m": first["lidar_min_m"],
                    "max_force_N": round(max(max(r["forces_N"].values()) for r in log), 2),
                    "end_centre_to_face_m": log[-1]["centre_to_face_m"],
                }
            )
        else:
            rows.append({"case": name, "t_s": "", "body": "none"})
    for ax in axes[:, 0]:
        ax.set_ylabel("contact force [N]")
    for ax in axes[1]:
        ax.set_xlabel("t [s]")
    fig.suptitle(
        "scripted drives into walls/shelves: force per body (solid; red line = 1 N collision "
        "force), "
        "centre-to-face gap (grey dashed) and lidar minimum (green dotted) on the right axis [m]",
        fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(out)
    keys = list(rows[0])
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        wr.writeheader()
        wr.writerows(rows)
    print(f"wrote {out}, {out_csv}")


def read_chunks(run_dir: str) -> dict:
    """metrics.csv of c1, c2, ... joined: wall time and iterations continue across chunks."""
    cols, wall0, it0 = {}, 0.0, 0
    dirs = sorted(glob.glob(os.path.join(run_dir, "c*")), key=lambda d: int(d.rsplit("c", 1)[1]))
    for d in dirs:
        with open(os.path.join(d, "metrics.csv"), encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
        for r in rows:
            r = {k: float(v) for k, v in r.items()}
            r["wall_s"] += wall0
            r["iteration"] += it0
            for k, v in r.items():
                cols.setdefault(k, []).append(v)
        wall0, it0 = cols["wall_s"][-1], int(cols["iteration"][-1])
    return cols


def plot_curves(out: str, run_dir: str, out_csv: str = "") -> None:
    m = read_chunks(run_dir)
    t = [w / 60 for w in m["wall_s"]]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4), dpi=120)
    for key, col, lab in (
        ("success_rate", "#1baf7a", "success"),
        ("collision_rate", "#e34948", "collision"),
        ("timeout_rate", "#eda100", "timeout"),
    ):
        a1.plot(t, m[key], color=col, lw=1.4, label=lab)
    a1.set_ylim(0, 1)
    a1.set_xlabel("training time [min] (env build excluded)")
    a1.set_ylabel("share of episodes ended")
    a1.legend(frameon=False)
    a1.grid(alpha=0.3)
    a2.plot(t, m["success_time_s"], color="#2a78d6", lw=1.4)
    a2.set_xlabel("training time [min]")
    a2.set_ylabel("successful episode time [s]")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out)
    if out_csv:  # every 10th row, small enough for the results folder
        keys = list(m)
        with open(out_csv, "w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh)
            wr.writerow(keys)
            for i in range(0, len(t), 10):
                wr.writerow([round(m[k][i], 4) for k in keys])
    print(f"wrote {out}")


def plot_traj(out: str, src: str, episodes: int = 1) -> None:
    import numpy as np

    z = np.load(src)
    xy, done, route = z["xy"], z["done"], z["route"]
    k = xy.shape[1]
    cols = int(math.ceil(k / 4))
    fig, axes = plt.subplots(4, cols, figsize=(3.4 * cols, 7.6), dpi=110)
    for e, ax in enumerate(axes.T.ravel()[:k]):
        for b in z["boxes"]:
            ax.add_patch(Rectangle(b[:2], b[2] - b[0], b[3] - b[1], color="#8c8c99", lw=0))
        ends = np.flatnonzero(done[:, e])[:episodes]
        start = 0
        for j, end in enumerate(ends):
            r = int(route[end, e])
            p = z["path"][r, : z["npts"][r]]
            ax.plot(p[:, 0], p[:, 1], "--", color="#52514e", lw=0.8)
            for c, sz in zip(z["obstacle_xy"][r], z["obstacle_sizes"], strict=True):
                if not np.isnan(c[0]):
                    ax.add_patch(
                        Rectangle(c - sz / 2, sz, sz, color="#2a78d6", alpha=0.6 if j == 0 else 0.3)
                    )
            seg = xy[start : end + 1, e]
            if "mover_xy" in z.files:  # mover tracks of this episode, thin
                for m in range(z["mover_xy"].shape[2]):
                    if z["mover_on"][start, e, m]:
                        mt = z["mover_xy"][start:end, e, m]
                        ax.plot(mt[:, 0], mt[:, 1], color="#d6336c", lw=2.5, alpha=0.35)
            ax.plot(seg[:-1, 0], seg[:-1, 1], color="#eb6834", lw=1.4)
            ax.plot(*p[0], "o", color="#1baf7a", ms=4)
            ax.add_patch(Circle(p[-1], 0.05, fill=False, color="#e34948", lw=1))
            ax.set_title(f"env {e}: {(end - start) * float(z['dt']):.1f} s", fontsize=8)
            start = end + 1
        ax.set_xlim(-0.08, 2.06)
        ax.set_ylim(-0.08, 1.06)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out)
    print(f"wrote {out}")


def plot_routes(out: str) -> None:
    import numpy as np

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import arena_spec as A
    import route_spec as R

    site = A.load_site()
    boxes = A.arena_boxes(site=site)
    static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes])
    r = R.build_routes(A.ARENA, site, static)
    fig, axes = plt.subplots(2, 3, figsize=(13, 5.4), dpi=110)
    rng = np.random.default_rng(1)
    picks = rng.choice(np.flatnonzero(np.arange(r.count) % r.layouts != 0), 5, replace=False)
    for i, ax in enumerate(axes.ravel()):
        for b in boxes:
            ax.add_patch(
                Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0, color=COLORS[b.kind], lw=0)
            )
        sel = range(0, r.count, r.layouts) if i == 0 else [picks[i - 1]]
        for j in sel:
            p = r.path[j, : r.npts[j]]
            ax.plot(p[:, 0], p[:, 1], color="#2a78d6", lw=0.8 if i == 0 else 1.5, alpha=0.7)
            if i:
                for c, sz in zip(r.obstacle_xy[j], r.obstacle_sizes, strict=True):
                    if not np.isnan(c[0]):
                        ax.add_patch(Rectangle(c - sz / 2, sz, sz, color="#e34948", alpha=0.7))
                s_, g_ = r.start_goal(j)
                ax.plot(*s_, "o", color="#1baf7a", ms=5)
                ax.plot(*g_, "*", color="#0b0b0b", ms=8)
                a, b = r.pairs[j // r.layouts]
                ax.set_title(
                    f"{r.node_names[a]} -> {r.node_names[b]}, layout {j % r.layouts}, "
                    f"{r.total[j]:.2f} m",
                    fontsize=9,
                )
        if i == 0:
            ax.set_title(f"{len(r.pairs)} node pairs, no boxes", fontsize=9)
        for name, xy in zip(r.node_names, r.node_xy, strict=True):
            ax.plot(*xy, ".", color="#0b0b0b", ms=3)
            if i == 0:
                ax.annotate(name, xy, fontsize=6, xytext=(2, 2), textcoords="offset points")
        ax.set_xlim(-0.08, 2.06)
        ax.set_ylim(-0.08, 1.06)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out)
    print(f"wrote {out}")


def plot_anim(out: str, src: str, env: int = 0, seconds: float = 30.0) -> None:
    """Top-down GIF of one tracked env from rollouts.npz: robot, its path, boxes and movers."""
    import numpy as np
    from matplotlib.animation import FuncAnimation, PillowWriter

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import mover_spec as M

    z = np.load(src)
    xy, done, route, dt = z["xy"][:, env], z["done"][:, env], z["route"][:, env], float(z["dt"])
    steps = min(len(xy), int(seconds / dt))
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=90)
    every = 2  # 10 fps of a 20 Hz rollout, real time

    def draw(f):
        k = f * every
        if done[k] and k > 0:  # a done frame already holds the next episode's reset state
            k -= 1
        ax.clear()
        for b in z["boxes"]:
            ax.add_patch(Rectangle(b[:2], b[2] - b[0], b[3] - b[1], color="#8c8c99", lw=0))
        r = int(route[k])
        p = z["path"][r, : z["npts"][r]]
        ax.plot(p[:, 0], p[:, 1], "--", color="#52514e", lw=0.8)
        for c, sz in zip(z["obstacle_xy"][r], z["obstacle_sizes"], strict=True):
            if not np.isnan(c[0]):
                ax.add_patch(Rectangle(c - sz / 2, sz, sz, color="#2a78d6", alpha=0.6))
        for m, shape in enumerate(M.MOVERS):
            if z["mover_on"][k, env, m]:
                ax.add_patch(Circle(z["mover_xy"][k, env, m], shape.radius, color="#d6336c"))
        start = k
        while start > 0 and not done[start - 1]:
            start -= 1
        ax.plot(xy[start : k + 1, 0], xy[start : k + 1, 1], color="#eb6834", lw=1.2)
        ax.add_patch(Circle(xy[k], 0.085, fill=False, color="#0b0b0b", lw=1.2))
        ax.add_patch(Circle(p[-1], 0.05, fill=False, color="#e34948", lw=1))
        ax.set_xlim(-0.08, 2.06)
        ax.set_ylim(-0.08, 1.06)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f"t {k * dt:4.1f} s   (movers drawn as their clearance circles)", fontsize=8)

    FuncAnimation(fig, draw, frames=steps // every).save(out, writer=PillowWriter(fps=10))
    print(f"wrote {out}")


if __name__ == "__main__":
    if sys.argv[1] == "scene":
        plot_scene(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "contact":
        plot_contact(sys.argv[2], sys.argv[3], sys.argv[4])
    elif sys.argv[1] == "curves":
        plot_curves(sys.argv[2], sys.argv[3], *sys.argv[4:5])
    elif sys.argv[1] == "routes":
        plot_routes(sys.argv[2])
    elif sys.argv[1] == "anim":
        plot_anim(sys.argv[2], sys.argv[3], *(int(x) for x in sys.argv[4:5]))
    elif sys.argv[1] == "traj":
        plot_traj(sys.argv[2], sys.argv[3], *(int(x) for x in sys.argv[4:5]))
    else:
        raise SystemExit(__doc__)
