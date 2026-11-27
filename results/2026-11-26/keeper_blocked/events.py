#!/usr/bin/env python3
"""11-26 LOCAL_YIELD 사건 세 개: 자세, 움직임, cmd_vel, 상태, scan 가까운 점, 상대 로봇.

사용: python3 events.py bags.pkl 출력폴더
"""
import math
import pickle
import sys
from bisect import bisect_right
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ---- 상수: 라이다 자리, pode1 트레이 기둥, local footprint ----
LIDAR_X = -0.017  # base에서 라이다, yaw 180도
PILLARS = [(-0.074, 0.043), (-0.084, 0.036), (-0.081, -0.038), (-0.062, -0.035)]
PILLAR_NEAR_M = 0.02
FOOT = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049),
        (-0.071, 0.042), (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042),
        (-0.049, -0.049), (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031),
        (0.040, -0.021)]
SPIN_R = max(math.hypot(x, y) for x, y in FOOT)  # 제자리 회전 반지름 0.0845
LOG = Path("/home/ane/ros2camp/real_data/2026-11-26/bags")

EVENTS = [
    {"name": "E1_express_pode2", "robot": "pode2", "peer": None, "t_yield": 1795688178.16,
     "t_end": 1795688181.25, "log": LOG / "pode2_logs/launch_191408.log"},
    {"name": "E2_run1_pode1_E-2", "robot": "pode1", "peer": "pode2", "t_yield": 1795691414.74,
     "t_end": 1795691419.80, "log": LOG / "pode1_logs/launch_200634.log"},
    {"name": "E3_run1_pode1_Q2", "robot": "pode1", "peer": "pode2", "t_yield": 1795691344.77,
     "t_end": 1795691345.98, "log": LOG / "pode1_logs/launch_200634.log"},
]


def kst(t):
    s = (t + 9 * 3600) % 86400
    return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:05.2f}"


def latest(rows, keys, t):
    i = bisect_right(keys, t) - 1
    return rows[i] if i >= 0 else None


# ---- 한 로봇의 시계열 색인 ----
class Robot:
    def __init__(self, bag):
        self.mo = sorted((t, x, y, a) for t, p, c, x, y, a in bag["tf"]
                         if p == "map" and c == "odom")
        self.mo_t = [m[0] for m in self.mo]
        self.odom = bag["odom"]
        self.odom_t = [o[0] for o in self.odom]
        self.cmd = bag["cmd_vel"]
        self.cmd_t = [c[0] for c in self.cmd]
        self.state = bag["state"]
        self.state_t = [s[0] for s in self.state]
        self.scan = sorted(bag.get("scan", []))
        self.scan_t = [s[0] for s in self.scan]

    def pose(self, t):
        m, o = latest(self.mo, self.mo_t, t), latest(self.odom, self.odom_t, t)
        if m is None or o is None:
            return None
        _, mx, my, ma = m
        _, x, y, a, v, w = o
        c, s = math.cos(ma), math.sin(ma)
        return (mx + c * x - s * y, my + s * x + c * y, math.remainder(ma + a, math.tau), v, w)

    def scan_base(self, t):
        sc = latest(self.scan, self.scan_t, t)
        if sc is None:
            return None, []
        ts, _, a0, da, ranges = sc
        pts = []
        for i, r in enumerate(ranges):
            if not (0.02 < r < 3.0):
                continue
            a = a0 + i * da
            pts.append((LIDAR_X - r * math.cos(a), -r * math.sin(a)))
        return ts, pts


# ---- 다각형까지 거리(안이면 음수) ----
def poly_dist(p, poly=FOOT):
    inside, best = False, math.inf
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > p[1]) != (y2 > p[1]):
            if p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
        dx, dy = x2 - x1, y2 - y1
        u = max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - x1 - u * dx, p[1] - y1 - u * dy))
    return -best if inside else best


def is_pillar(p):
    return any(math.dist(p, q) < PILLAR_NEAR_M for q in PILLARS)


# ---- scan 요약: 기둥 점, 기둥 아닌 가장 가까운 점, 앞 0.3 m 점 ----
def scan_summary(pts, pillar_robot):
    pil = [p for p in pts if pillar_robot and is_pillar(p)]
    rest = [p for p in pts if not (pillar_robot and is_pillar(p))]
    near = min(rest, key=poly_dist) if rest else None
    front = [p for p in rest if p[0] > 0.04 and abs(p[1]) < 0.06 and p[0] < 0.34]
    spin = [p for p in rest if math.hypot(*p) < SPIN_R + 0.025]
    return {"pillar_n": len(pil),
            "pillar_in_spin": sum(math.hypot(*p) < SPIN_R + 0.0125 for p in pil),
            "near": near, "near_d": poly_dist(near) if near else None,
            "front_min": min((p[0] for p in front), default=None), "spin_n": len(spin)}


def rpp_warnings(log):
    out = []
    for line in open(log):
        if "detected collision ahead" in line:
            out.append(float(line.split("[")[3].split("]")[0]))
    return out


def event_table(ev, bots):
    me, peer = bots[ev["robot"]], bots[ev["peer"]] if ev["peer"] else None
    warns = [w for w in rpp_warnings(ev["log"]) if ev["t_yield"] - 5 <= w <= ev["t_end"] + 2]
    lines = [f"## {ev['name']}", "", "RPP 'detected collision ahead' 시각: "
             + (", ".join(kst(w) for w in warns) or "없음"), "",
             "| KST | state | map x | map y | yaw° | odom v | odom w | cmd v | cmd w | 상대 거리 m"
             " | 기둥 점(회전원 안) | 기둥 아닌 최근접 점(base) | 발자국까지 m | 앞 0.3 m 최소 x |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    t = ev["t_yield"] - 4.0
    while t <= ev["t_end"] + 1.5:
        p = me.pose(t)
        st = latest(me.state, me.state_t, t)
        cm = latest(me.cmd, me.cmd_t, t)
        cmd = (cm[1], cm[2]) if cm and t - cm[0] < 0.3 else (0.0, 0.0)
        pd = ""
        if peer:
            q = peer.pose(t)
            pd = f"{math.dist(p[:2], q[:2]):.2f}" if q and p else ""
        ts, pts = me.scan_base(t)
        s = scan_summary(pts, ev["robot"] == "pode1") if pts and ts and t - ts < 0.3 else None
        sc = ("", "", "", "") if s is None else (
            f"{s['pillar_n']} ({s['pillar_in_spin']})",
            f"({s['near'][0]:+.3f}, {s['near'][1]:+.3f})" if s["near"] else "",
            f"{s['near_d']:.3f}" if s["near_d"] is not None else "",
            f"{s['front_min']:.3f}" if s["front_min"] is not None else "-")
        lines.append(f"| {kst(t)} | {st[1] if st else ''}{'/' + st[2] if st and st[2] else ''}"
                     f" | {p[0]:.3f} | {p[1]:.3f} | {math.degrees(p[2]):.0f} | {p[3]:.3f}"
                     f" | {p[4]:.2f} | {cmd[0]:.3f} | {cmd[1]:.2f} | {pd} | " + " | ".join(sc)
                     + " |")
        t += 0.25
    return lines, warns


# ---- 그림: 지도 틀 궤적, 사건 순간 scan, 발자국, 상대 로봇 ----
def figure(ev, bots, out):
    me = bots[ev["robot"]]
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    ax = axes[0]
    tr = [me.pose(t) for t in frange(ev["t_yield"] - 6, ev["t_end"] + 2, 0.1)]
    ax.plot([p[0] for p in tr if p], [p[1] for p in tr if p], "-", color="#1f77b4",
            label=f"{ev['robot']} path")
    p = me.pose(ev["t_yield"])
    _, pts = me.scan_base(ev["t_yield"])
    c, s = math.cos(p[2]), math.sin(p[2])
    wp = [(p[0] + c * x - s * y, p[1] + s * x + c * y) for x, y in pts]
    ax.scatter([q[0] for q in wp], [q[1] for q in wp], s=3, color="#555", label="scan at LOCAL_YIELD")
    fp = [(p[0] + c * x - s * y, p[1] + s * x + c * y) for x, y in FOOT + FOOT[:1]]
    ax.plot([q[0] for q in fp], [q[1] for q in fp], color="#d62728", label="local footprint")
    if ev["peer"]:
        q = bots[ev["peer"]].pose(ev["t_yield"])
        if q:
            ax.plot(q[0], q[1], "s", color="#2ca02c", label=f"{ev['peer']}")
    ax.set_xlim(p[0] - 0.6, p[0] + 0.6)
    ax.set_ylim(p[1] - 0.6, p[1] + 0.6)
    ax.set_aspect("equal")
    ax.set_title(f"{ev['name']} map ({kst(ev['t_yield'])})")
    ax.legend(fontsize=7)
    ax = axes[1]
    ts = list(frange(ev["t_yield"] - 4, ev["t_end"] + 1.5, 0.05))
    po = [me.pose(t) for t in ts]
    cm = [latest(me.cmd, me.cmd_t, t) for t in ts]
    rel = [t - ev["t_yield"] for t in ts]
    ax.plot(rel, [q[3] if q else 0 for q in po], label="odom v (m/s)")
    ax.plot(rel, [q[4] / 10 if q else 0 for q in po], label="odom w /10 (rad/s)")
    ax.plot(rel, [c_[1] if c_ and t - c_[0] < 0.3 else 0 for c_, t in zip(cm, ts)], "--",
            label="cmd v")
    ax.plot(rel, [c_[2] / 10 if c_ and t - c_[0] < 0.3 else 0 for c_, t in zip(cm, ts)], "--",
            label="cmd w /10")
    for w in rpp_warnings(ev["log"]):
        if ev["t_yield"] - 4 <= w <= ev["t_end"] + 1.5:
            ax.axvline(w - ev["t_yield"], color="r", alpha=0.3)
    ax.axvline(0, color="k")
    ax.axvline(ev["t_end"] - ev["t_yield"], color="k", ls=":")
    ax.set_xlabel("s from LOCAL_YIELD (red = RPP collision ahead)")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / f"{ev['name']}.png", dpi=90)
    plt.close(fig)


def frange(a, b, d):
    while a <= b:
        yield a
        a += d


def main():
    bags = pickle.load(open(sys.argv[1], "rb"))
    out = Path(sys.argv[2])
    bots = {r: Robot(b) for r, b in bags.items()}
    md = []
    for ev in EVENTS:
        lines, _ = event_table(ev, bots)
        md += lines + [""]
        figure(ev, bots, out)
    (out / "events_tables.md").write_text("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
