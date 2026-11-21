"""Plots of arena_check.py output (no Isaac needed).

  scene:   top-down view of the yaml boxes the env was built from, the robot of env 0, and its
           simulated lidar hits next to the 2D ray cast of the same boxes
  contact: per scripted drive, contact force per body against the gap between the robot centre
           and the obstacle face, and a small CSV of the first step above the collision force

  python learning/store_arena/plot_arena.py scene <scene.json> <out.png>
  python learning/store_arena/plot_arena.py contact <contact_push.json> <out.png> <out.csv>
"""

import csv
import json
import math
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


if __name__ == "__main__":
    if sys.argv[1] == "scene":
        plot_scene(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "contact":
        plot_contact(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        raise SystemExit(__doc__)
