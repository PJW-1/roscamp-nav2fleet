#!/usr/bin/env python3
"""11-23 끔 2 판 pode1 rosbag의 급행 scan을 지도마다 위치를 따로 다시 맞춰 겹친다.

AMCL을 다시 돌리는 대신 scan 하나씩 AMCL 점수가 가장 큰 자리를 찾는다.

사용(ROS Jazzy를 source한 셸):
  python3 scan_overlay.py BAG_DIR OUT_DIR 이름=지도.yaml ...

- 기록된 위치: tf map->odom(그 scan 시각 직전 값)과 odom->base_footprint(보간),
  정적 base->rplidar_link.
- 급행 곧은 구간: 기록 위치 x 0.80~1.45, y 0.83~0.91, 방향이 동서에서 10도 안, 회전 0.15 rad/s 아래.
- 지도마다 위치 다시 맞추기: AMCL likelihood_field 모델(nav2_params의 sigma_hit 0.05, z_hit 0.5,
  z_rand 0.5, laser_likelihood_max_dist 2.0)로 scan 하나씩 점수를 가장 크게 하는 (dx, dy, dyaw)를
  기록 위치 둘레 ±10 cm, ±6도에서 찾는다. 모든 빔을 쓴다(AMCL은 60개).
- 잰 것(지도마다): 맞춘 위치가 기록 위치에서 옆으로 얼마나 옮겼나, 옆으로만 옮기며 점수가 최고의
  98 % 이상인 폭(AMCL이 옆 위치를 가르지 못하는 폭), 벽 점·진열대 점 가운데값과 그려진 면의 차이,
  점마다 그려진 면까지 부호 있는 거리(+ 면 앞 빈칸, - 그려진 점유 칸 안).
- 지도와 상관없는 것: scan 하나에서 잰 급행 폭(벽 점 y 가운데값 - 진열대 점 y 가운데값),
  서쪽 HALL에서 로봇 바로 남북 가장 가까운 점 사이(안쪽 세로, 줄자 0.98: lidar 거리 치우침 확인).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage, optimize

LASER_XY = (-0.017, 0.0)  # base_link->rplidar_mount, 방향 pi(rplidar_link)
LASER_YAW = math.pi
SIGMA, Z_HIT, Z_RAND, MAX_RANGE = 0.05, 0.5, 0.5, 100.0
GRID = 0.005  # 거리장 칸(지도를 이 칸으로 나눠 그림)
X_LO, X_HI = 0.80, 1.45
SELF_M = 0.10  # 이보다 가까운 점은 로봇 자기 몸(급행에서 y 0.84, 0.91 줄로 찍힘)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()
             if t.name in ("/pode1/scan", "/pode1/tf")}
    scans, odom, m2o = [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/pode1/scan":
            m = deserialize_message(data, types[topic])
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angle_min,
                          m.angle_increment, np.array(m.ranges, dtype=float), m.range_min))
        elif topic == "/pode1/tf":
            for tr in deserialize_message(data, types[topic]).transforms:
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                p, q = tr.transform.translation, tr.transform.rotation
                if tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
                    odom.append((st, p.x, p.y, yaw_of(q)))
                elif tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    m2o.append((st, p.x, p.y, yaw_of(q)))
    return scans, np.array(sorted(odom)), np.array(sorted(m2o))


def compose(a, b):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (x + b[0] * c - b[1] * s, y + b[0] * s + b[1] * c, t + b[2])


def recorded_pose(t, odom, m2o):
    """scan 시각의 map->base_footprint와 그때 회전 속도(odom 차분)."""
    k = np.searchsorted(m2o[:, 0], t, side="right") - 1
    if k < 0 or t - m2o[k, 0] > 1.0:
        return None
    j = np.searchsorted(odom[:, 0], t)
    if j <= 0 or j >= len(odom):
        return None
    a, b = odom[j - 1], odom[j]
    f = (t - a[0]) / max(b[0] - a[0], 1e-9)
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    base = (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw)
    j0, j1 = max(0, j - 5), min(len(odom) - 1, j + 5)
    d = odom[j1, 3] - odom[j0, 3]
    w = math.atan2(math.sin(d), math.cos(d)) / max(odom[j1, 0] - odom[j0, 0], 1e-6)
    return compose(tuple(m2o[k, 1:]), base), w


def laser_points(scan):
    _, amin, inc, ranges, rmin = scan
    ang = amin + inc * np.arange(len(ranges))
    ok = np.isfinite(ranges) & (ranges > max(rmin, SELF_M)) & (ranges < 3.0)
    r, a = ranges[ok], ang[ok] + LASER_YAW
    return np.stack([LASER_XY[0] + r * np.cos(a), LASER_XY[1] + r * np.sin(a)], axis=1)


class Field:
    def __init__(self, path):
        meta = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        img = np.array(Image.open(Path(path).parent / meta["image"]))[::-1]
        res = float(meta["resolution"])
        occ = (255 - img.astype(float)) / 255.0 > meta["occupied_thresh"]
        k = int(round(res / GRID))
        self.occ = np.kron(occ, np.ones((k, k), dtype=bool))  # 칸을 GRID로 나눔(같은 경계)
        self.ox, self.oy = meta["origin"][:2]
        self.out = ndimage.distance_transform_edt(~self.occ) * GRID  # 빈칸에서 가장 가까운 점유까지
        self.inn = ndimage.distance_transform_edt(self.occ) * GRID  # 점유 칸 안에서 빈칸까지

    def idx(self, pts):
        i = np.clip(((pts[:, 0] - self.ox) / GRID).astype(int), 0, self.occ.shape[1] - 1)
        j = np.clip(((pts[:, 1] - self.oy) / GRID).astype(int), 0, self.occ.shape[0] - 1)
        return j, i

    def dist(self, pts):
        return self.out[self.idx(pts)]

    def signed(self, pts):
        j, i = self.idx(pts)
        return np.where(self.occ[j, i], -self.inn[j, i] + GRID / 2, self.out[j, i] - GRID / 2)


def to_map(pose, pts):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def score(field, pose, pts):
    d = field.dist(to_map(pose, pts))
    p = Z_HIT * np.exp(-d * d / (2 * SIGMA * SIGMA)) + Z_RAND / MAX_RANGE
    return float(np.sum(p ** 3))


def refit(field, pose, pts):
    best, best_s = pose, score(field, pose, pts)
    for dx in (-0.06, -0.03, 0.0, 0.03, 0.06):
        for dy in (-0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06):
            for dt in (-0.05, 0.0, 0.05):
                cand = (pose[0] + dx, pose[1] + dy, pose[2] + dt)
                s = score(field, cand, pts)
                if s > best_s:
                    best, best_s = cand, s
    res = optimize.minimize(lambda v: -score(field, (v[0], v[1], v[2]), pts), np.array(best),
                            method="Powell", bounds=[(pose[0] - 0.1, pose[0] + 0.1),
                                                     (pose[1] - 0.1, pose[1] + 0.1),
                                                     (pose[2] - 0.105, pose[2] + 0.105)],
                            options={"xtol": 1e-4, "ftol": 1e-6})
    return tuple(res.x)


def groups(world):
    inx = (world[:, 0] > 0.75) & (world[:, 0] < 1.50)
    return {"wall": inx & (world[:, 1] > 0.90) & (world[:, 1] < 1.05),
            "shelf": inx & (world[:, 1] > 0.72) & (world[:, 1] < 0.86)}


def q(v, p):
    return float(np.percentile(v, p)) if len(v) else float("nan")


def stats(v):
    return {"n": len(v), "median": q(v, 50), "p10": q(v, 10), "p90": q(v, 90)}


def drawn_faces(field, x=1.1, ref=0.868):
    """x 기둥에서 ref가 든 빈칸 구간의 아래 끝(진열대 A 북쪽 면)과 위 끝(북쪽 벽 면)."""
    col = field.occ[:, int((x - field.ox) / GRID)]
    k = int((ref - field.oy) / GRID)
    lo, hi = k, k
    while not col[lo - 1]:
        lo -= 1
    while not col[hi + 1]:
        hi += 1
    return field.oy + lo * GRID, field.oy + (hi + 1) * GRID


def flat_width(field, fit, pts, step=0.0025, span=0.08, keep=0.98):
    """맞춘 위치에서 옆(y)으로만 옮겨 잰 점수가 최고의 keep 이상인 y 폭(옆 위치를 못 가르는 폭)."""
    dys = np.arange(-span, span + 1e-9, step)
    s = np.array([score(field, (fit[0], fit[1] + d, fit[2]), pts) for d in dys])
    good = dys[s >= keep * s.max()]
    return float(good.max() - good.min() + step)


def range_check(chosen_all):
    """서쪽 HALL(x 0.15~0.45)에서 로봇 바로 남북의 가장 가까운 점 둘 사이 = 안쪽 세로(줄자 0.98)."""
    out = []
    for scan, pose in chosen_all:
        if not (0.15 < pose[0] < 0.45 and 0.25 < pose[1] < 0.75):
            continue
        world = to_map(pose, laser_points(scan))
        near = np.abs(world[:, 0] - pose[0]) < 0.015
        north = world[near & (world[:, 1] > pose[1] + 0.1), 1]
        south = world[near & (world[:, 1] < pose[1] - 0.1), 1]
        if len(north) and len(south):
            out.append(float(north.min() - south.max()))
    return out


def main(argv):
    bag, out = Path(argv[0]), Path(argv[1])
    out.mkdir(parents=True, exist_ok=True)
    maps = [(a.split("=", 1)[0], Field(a.split("=", 1)[1])) for a in argv[2:]]
    scans, odom, m2o = read_bag(bag)
    chosen, still = [], []
    for scan in scans:
        got = recorded_pose(scan[0], odom, m2o)
        if got is None:
            continue
        pose, w = got
        heading = abs(math.sin(pose[2]))
        if abs(w) < 0.15:
            still.append((scan, pose))
        if (X_LO < pose[0] < X_HI and 0.83 < pose[1] < 0.91
                and heading < math.sin(math.radians(10)) and abs(w) < 0.15):
            chosen.append((scan, pose))
    t0 = scans[0][0]
    report = {"scans_total": len(scans), "scans_express": len(chosen), "maps": {}}
    widths = []
    for scan, pose in chosen:
        world = to_map(pose, laser_points(scan))
        g = groups(world)
        if g["wall"].sum() >= 5 and g["shelf"].sum() >= 5:
            widths.append(np.median(world[g["wall"], 1]) - np.median(world[g["shelf"], 1]))
    report["scan_express_width"] = stats(widths)
    report["scan_inner_height_west_hall"] = stats(range_check(still))
    report["express_passes_start_s"] = sorted({int(s[0] - t0) // 20 * 20 for s, _ in chosen})
    figs = {}
    for name, field in maps:
        shelf_face, wall_face = drawn_faces(field)
        rows = {"wall": [], "shelf": []}
        dys, dxs, flats, wall_off, shelf_off, fit_pts = [], [], [], [], [], []
        for k, (scan, pose) in enumerate(chosen):
            pts = laser_points(scan)
            fit = refit(field, pose, pts)
            dxs.append(fit[0] - pose[0])
            dys.append(fit[1] - pose[1])
            flats.append(flat_width(field, fit, pts))
            world = to_map(fit, pts)
            g = groups(world)
            for key in rows:
                rows[key].extend(field.signed(world[g[key]]).tolist())
            if g["wall"].sum() >= 5 and g["shelf"].sum() >= 5:
                wall_off.append(float(np.median(world[g["wall"], 1]) - wall_face))
                shelf_off.append(float(np.median(world[g["shelf"], 1]) - shelf_face))
            if k % 4 == 0:
                fit_pts.append(world[g["wall"] | g["shelf"]])
        entry = {"drawn_express": [shelf_face, wall_face, wall_face - shelf_face],
                 "dy_vs_recorded": stats(dys), "dx_vs_recorded": stats(dxs),
                 "lateral_flat_width": stats(flats),
                 "wall_points_minus_drawn_wall": stats(wall_off),
                 "shelf_points_minus_drawn_shelf": stats(shelf_off)}
        for key, v in rows.items():
            entry["signed_" + key] = stats(v)
        report["maps"][name] = entry
        figs[name] = (field, np.concatenate(fit_pts) if fit_pts else np.zeros((0, 2)), rows)
        print(name, json.dumps(entry, ensure_ascii=False), flush=True)
    (out / "scan_overlay.json").write_text(json.dumps(report, ensure_ascii=False, indent=1),
                                           encoding="utf-8")
    plot(out / "scan_overlay.png", figs)
    print(json.dumps({k: v for k, v in report.items() if k != "maps"}, ensure_ascii=False))


def plot(path, figs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(figs)
    fig, axes = plt.subplots(2, n, figsize=(4.2 * n, 6.4), gridspec_kw={"height_ratios": [3, 2]})
    for c, (name, (field, pts, rows)) in enumerate(figs.items()):
        ax = axes[0, c]
        x0, x1, y0, y1 = 0.70, 1.55, 0.70, 1.02
        i0, i1 = int((x0 - field.ox) / GRID), int((x1 - field.ox) / GRID)
        j0, j1 = int((y0 - field.oy) / GRID), int((y1 - field.oy) / GRID)
        ax.imshow(field.occ[j0:j1, i0:i1], origin="lower", extent=(x0, x1, y0, y1), cmap="Greys",
                  vmin=0, vmax=1.6, aspect="auto")
        ax.scatter(pts[:, 0], pts[:, 1], s=0.3, c="#d1495b")
        ax.axhline(0.868, color="#2e86ab", lw=0.6, ls="--")
        ax.set_title(name, fontsize=8)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.tick_params(labelsize=7)
        bx = axes[1, c]
        bins = np.arange(-0.05, 0.0501, 0.0025)
        for key, color in (("wall", "#2e86ab"), ("shelf", "#d1495b")):
            bx.hist(np.clip(rows[key], -0.05, 0.05), bins=bins, color=color, alpha=0.6,
                    label={"wall": "north wall", "shelf": "shelf A north face"}[key])
        bx.axvline(0, color="k", lw=0.6)
        bx.set_xlabel("signed distance to drawn face (m), + = in free space", fontsize=7)
        bx.tick_params(labelsize=7)
        bx.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(path, dpi=90)


if __name__ == "__main__":
    main(sys.argv[1:])
