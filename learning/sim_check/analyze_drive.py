"""Summarise a pinky_newton_drive.py CSV: per-phase steady-state numbers and two plots.

Usage: python analyze_drive.py <log.csv> <out_dir> [--steady 2.0]
Steady window = last <steady> seconds of each phase. Prints a Markdown table and writes
<tag>_phases.csv, <tag>_traj.png, <tag>_speeds.png.
"""

import argparse
import os

import numpy as np

R = 0.028
B_CMD = 0.0961
CMD = {
    "settle": (0.0, 0.0),
    "straight_0.10": (0.10, 0.0),
    "straight_0.20": (0.20, 0.0),
    "stop1": (0.0, 0.0),
    "spin_1.0": (0.0, 1.0),
    "arc_left": (0.12, 0.6),
    "arc_right": (0.12, -0.6),
    "stop2": (0.0, 0.0),
}

p = argparse.ArgumentParser()
p.add_argument("csv")
p.add_argument("out")
p.add_argument("--steady", type=float, default=2.0)
a = p.parse_args()
tag = os.path.basename(a.csv).replace("_log.csv", "")

raw = np.genfromtxt(a.csv, delimiter=",", names=True, dtype=None, encoding="utf-8")
phases = list(dict.fromkeys(raw["phase"]))
envs = np.unique(raw["env"])
out_rows = []
print(f"## {tag}: {len(envs)} envs, steady window = last {a.steady} s of each phase\n")
hdr = (
    "| phase | v_cmd | v_meas (ratio) | w_cmd | w_meas (ratio) | b_eff from wheels [m] "
    "| |vy| mean [m/s] | z mean / std [mm] | roll std / pitch std [deg] "
    "| wheel L/R actual / cmd [rad/s] | moved in window [mm] "
    "| moved in last 0.5 s [mm] | max tilt in phase [deg] |"
)
print(hdr)
print("|---" * (hdr.count("|") - 1) + "|")
for ph in phases:
    m = raw["phase"] == ph
    t = raw["t"][m]
    t_end = t.max()
    w = m & (raw["t"] > t_end - a.steady)
    d = raw[w]
    v_cmd, w_cmd = CMD[ph]
    vx = d["vx_b"].mean()
    wz = d["wz_b"].mean()
    vy = np.abs(d["vy_b"]).mean()
    wl, wr = d["wl"].mean(), d["wr"].mean()
    wlc, wrc = d["wl_cmd"].mean(), d["wr_cmd"].mean()
    b_eff = R * (wr - wl) / wz if abs(w_cmd) > 0 and abs(wz) > 1e-6 else float("nan")
    # per-env std over time, then mean over envs
    z_std = np.mean([d["z"][d["env"] == e].std() for e in envs])
    roll_std = np.degrees(np.mean([d["roll"][d["env"] == e].std() for e in envs]))
    pitch_std = np.degrees(np.mean([d["pitch"][d["env"] == e].std() for e in envs]))
    moved = []
    for e in envs:
        de = d[d["env"] == e]
        moved.append(np.hypot(de["x"][-1] - de["x"][0], de["y"][-1] - de["y"][0]))
    moved = np.mean(moved) * 1000
    tail = raw[m & (raw["t"] > t_end - 0.5)]
    drift = (
        np.mean(
            [
                np.hypot(np.ptp(tail["x"][tail["env"] == e]), np.ptp(tail["y"][tail["env"] == e]))
                for e in envs
            ]
        )
        * 1000
    )
    max_tilt = np.degrees(max(np.abs(raw["pitch"][m]).max(), np.abs(raw["roll"][m]).max()))
    rv = vx / v_cmd if v_cmd else float("nan")
    rw = wz / w_cmd if w_cmd else float("nan")
    print(
        f"| {ph} | {v_cmd:.2f} | {vx:.4f} ({rv:.3f}) | {w_cmd:.2f} | {wz:.4f} ({rw:.3f}) "
        f"| {b_eff:.4f} | {vy:.4f} "
        f"| {d['z'].mean() * 1000:.2f} / {z_std * 1000:.3f} | {roll_std:.3f} / {pitch_std:.3f} "
        f"| {wl:.2f}/{wr:.2f} vs {wlc:.2f}/{wrc:.2f} | {moved:.1f} | {drift:.2f} | {max_tilt:.2f} |"
    )
    out_rows.append(
        (
            ph,
            v_cmd,
            vx,
            rv,
            w_cmd,
            wz,
            rw,
            b_eff,
            vy,
            d["z"].mean(),
            z_std,
            roll_std,
            pitch_std,
            wl,
            wr,
            wlc,
            wrc,
            moved,
            drift,
            max_tilt,
        )
    )

allz = raw["z"]
print(
    f"\nwhole run: max |roll| = {np.degrees(np.abs(raw['roll']).max()):.3f} deg, "
    f"max |pitch| = {np.degrees(np.abs(raw['pitch']).max()):.3f} deg, z range = "
    f"{allz.min() * 1000:.2f}..{allz.max() * 1000:.2f} mm, "
    f"finite = {bool(np.all(np.isfinite(raw['x'])))}"
)
spread = []
for ph in phases:
    m = raw["phase"] == ph
    t_end = raw["t"][m].max()
    last = raw[m & (raw["t"] == t_end)]
    spread.append(
        (ph, np.ptp(last["x"]) * 1000, np.ptp(last["y"]) * 1000, np.degrees(np.ptp(last["yaw"])))
    )
print(
    "env-to-env spread at phase end (x/y mm, yaw deg): "
    + "; ".join(f"{s[0]} {s[1]:.2f}/{s[2]:.2f}/{s[3]:.3f}" for s in spread)
)

os.makedirs(a.out, exist_ok=True)
with open(os.path.join(a.out, f"{tag}_phases.csv"), "w", encoding="utf-8") as f:
    f.write(
        "phase,v_cmd,v_meas,v_ratio,w_cmd,w_meas,w_ratio,b_eff,vy_abs_mean,z_mean,z_std,roll_std_deg,"
        "pitch_std_deg,wl,wr,wl_cmd,wr_cmd,moved_mm,moved_last0.5s_mm,max_tilt_deg\n"
    )
    for r in out_rows:
        f.write(r[0] + "," + ",".join(f"{x:.6g}" for x in r[1:]) + "\n")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

fig, ax = plt.subplots(figsize=(6, 6))
for e in envs:
    de = raw[raw["env"] == e]
    ax.plot(de["x"], de["y"], lw=1, label=f"env {e}" if e < 4 else None)
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title(f"{tag}: base_link path (all envs overlaid)")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(a.out, f"{tag}_traj.png"), dpi=110)

d0 = raw[raw["env"] == envs[0]]
v_cmd = np.array([CMD[ph][0] for ph in d0["phase"]])
w_cmd = np.array([CMD[ph][1] for ph in d0["phase"]])
fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
axs[0].plot(d0["t"], d0["vx_b"], label="vx body")
axs[0].plot(d0["t"], v_cmd, "k--", label="v cmd")
axs[0].plot(d0["t"], d0["vy_b"], label="vy body")
axs[0].set_ylabel("m/s")
axs[0].legend(loc="upper right")
axs[1].plot(d0["t"], d0["wz_b"], label="wz body")
axs[1].plot(d0["t"], w_cmd, "k--", label="w cmd")
axs[1].set_ylabel("rad/s")
axs[1].legend(loc="upper right")
axs[2].plot(d0["t"], (d0["z"] - d0["z"].mean()) * 1000, label="z - mean [mm]")
axs[2].plot(d0["t"], np.degrees(d0["roll"]), label="roll [deg]")
axs[2].plot(d0["t"], np.degrees(d0["pitch"]), label="pitch [deg]")
axs[2].legend(loc="upper right")
axs[2].set_xlabel("sim time [s]")
for ax in axs:
    ax.grid(True, alpha=0.3)
axs[0].set_title(f"{tag}: env 0 body-frame speeds and attitude")
fig.tight_layout()
fig.savefig(os.path.join(a.out, f"{tag}_speeds.png"), dpi=110)
print(f"\nwrote {tag}_phases.csv, {tag}_traj.png, {tag}_speeds.png to {a.out}")
