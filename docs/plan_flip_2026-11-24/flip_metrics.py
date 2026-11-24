#!/usr/bin/env python3
"""경로 뒤집힘 판 기록 분석: 판 폴더마다 지표 한 줄과 접두별 평균/합계 표.

사용: python3 flip_metrics.py 판폴더... [--csv 출력.csv]
bag은 ROS 환경(rosbag2_py, fleet_interfaces)을 source한 뒤 읽는다.
"""
import argparse
import csv
import json
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "pode" / "pode_plan_keep"))
from pode_plan_keep.route import OFF, OTHER, first_look  # noqa: E402

HALL_YAML = REPO / "pc/fleet_core/fleet_core/data/fleet_map_v7.yaml"
ROBOTS = ("pode1", "pode2", "pode3")
DIST_JUMP_M = 0.10
LIN_STOP, ANG_SPIN, SPIN_MIN_S = 0.02, 0.3, 0.5
SHIM_LIN, SHIM_ANG, SHIM_MIN_S = 0.001, 0.3, 0.3


# ---- 경로 뒤집힘 ----
def count_flips(plans):
    """plans: [(t, [(x, y), ...])] 시간순. 반환: (뒤집힘 목록, off_route 수)."""
    flips, off, prev = [], 0, None
    for t, pts in plans:
        if not pts:
            continue
        if prev is not None:
            kind, _ = first_look(prev, prev[-1], pts, pts[-1])
            if kind == OTHER:
                flips.append({"t": t, "prev": prev, "new": pts})
            elif kind == OFF:
                off += 1
        prev = pts
    return flips, off


def collect_flips(bag):
    """로봇별 뒤집힘. plan_kept가 있으면 그것(따라간 경로) 기준, 없으면 plan.
    반환: (센 뒤집힘, planner 기준 뒤집힘, off_route 수)."""
    used, planner, off_n = {}, {}, 0
    for r in ROBOTS:
        pl = bag["plan"].get(r, []) if bag else []
        kp = bag["kept"].get(r, []) if bag else []
        planner[r], off_p = count_flips(pl)
        used[r], off_u = count_flips(kp) if kp else (planner[r], off_p)
        off_n += off_u
    return used, planner, off_n


# ---- 구간 세기 ----
def count_runs(t, cond, min_s):
    """cond가 이어지는 구간(길이 min_s 이상)의 (시작, 끝) 목록. 시간 간격이 크면 끊는다."""
    runs, start, last = [], None, None
    for ti, c in zip(t, cond):
        if c:
            if start is not None and ti - last > 0.5:
                if last - start >= min_s:
                    runs.append((start, last))
                start = None
            if start is None:
                start = ti
            last = ti
        elif start is not None:
            if last - start >= min_s:
                runs.append((start, last))
            start = None
    if start is not None and last - start >= min_s:
        runs.append((start, last))
    return runs


def smooth_velocity(t, x, y, yaw, win=0.1, dt=0.05):
    """균일 격자로 보간한 뒤 +-win 초 중심차분. 반환: (격자 시간, 선속도, 각속도)."""
    g = np.arange(t[0], t[-1], dt)
    gx, gy = np.interp(g, t, x), np.interp(g, t, y)
    gyaw = np.interp(g, t, np.unwrap(yaw))
    k = max(1, round(win / dt))
    n = len(g)
    if n <= 2 * k:
        return g, np.zeros(n), np.zeros(n)
    a, b = np.arange(0, n) - k, np.arange(0, n) + k
    a, b = np.clip(a, 0, n - 1), np.clip(b, 0, n - 1)
    span = (b - a) * dt
    v = np.hypot(gx[b] - gx[a], gy[b] - gy[a]) / span
    w = np.abs(gyaw[b] - gyaw[a]) / span
    return g, v, w


def in_rects(x, y, rects):
    m = np.zeros(len(x), bool)
    for x0, y0, x1, y1 in rects:
        m |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    return m


def hall_spins(truth, nav_iv, rects):
    """truth: {robot: (t, x, y, yaw)}, nav_iv: {robot: 구간 목록} 또는 None(HALL만).
    반환: (수, 총 시간)."""
    n, total = 0, 0.0
    for r, (t, x, y, yaw) in truth.items():
        if len(t) < 5:
            continue
        g, v, w = smooth_velocity(t, x, y, yaw)
        cond = (v < LIN_STOP) & (w > ANG_SPIN)
        cond &= in_rects(np.interp(g, t, x), np.interp(g, t, y), rects)
        if nav_iv is not None:
            on = np.zeros(len(g), bool)
            for a, b in nav_iv.get(r, []):
                on |= (g >= a) & (g <= b)
            cond &= on
        for a, b in count_runs(g, cond, SPIN_MIN_S):
            n += 1
            total += b - a
    return n, total


def hall_spin_metric(truth, rows, rects):
    """관제 주문 기록으로 nav_goal 구간을 정해 HALL 회전을 센다. 기록이 없으면 셀 수 없음(None)."""
    iv = nav_intervals(rows)
    if iv is None:
        return None, None, "none"
    n, total = hall_spins(truth, iv, rects)
    return n, total, "fleet_record"


def pick_cmds(bag):
    """로봇마다 cmd_vel, 없으면 cmd_vel_nav."""
    if not bag:
        return {}
    robots = set(bag["cmd"]) | set(bag["cmd_nav"])
    return {r: bag["cmd"].get(r) or bag["cmd_nav"].get(r) for r in sorted(robots)}


def shim_spins(cmds):
    """cmds: {robot: [(t, lin_x, ang_z)]}. 반환: 수."""
    n = 0
    for rows in cmds.values():
        t = [r[0] for r in rows]
        c = [abs(r[1]) < SHIM_LIN and abs(r[2]) > SHIM_ANG for r in rows]
        n += len(count_runs(t, c, SHIM_MIN_S))
    return n


def dist_jumps(series):
    """series: {그룹 키: [값...]} 시간순. 음수와 None은 뺀다. 이웃 차 > 0.10 m의 수."""
    n = 0
    for vals in series.values():
        v = [x for x in vals if x is not None and x >= 0]
        n += sum(abs(b - a) > DIST_JUMP_M for a, b in zip(v, v[1:]))
    return n


# ---- 관제 기록(fleet_record.jsonl) ----
def read_record(path):
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text().splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows


def nav_intervals(rows):
    """MoveOrderCmd의 구간 방식과 feedback의 last_done_seq로 nav_goal 수행 구간을 낸다.
    반환: {robot: [(t0, t1)]} 또는 명령이 없으면 None."""
    cur, out, open_t = {}, defaultdict(list), {}
    chain = {}  # 로봇마다 지금 order가 쓴 goal_id들(갱신 전 옛 goal 포함)
    seen = False

    def active(segs, done):
        for s in segs:
            if s["seq_id"] > done:
                return s["mode"] == "nav_goal"
        return False

    def set_state(r, t, on):
        if on and r not in open_t:
            open_t[r] = t
        elif not on and r in open_t:
            out[r].append((open_t.pop(r), t))

    for d in rows:
        t = d.get("t")
        if d.get("out") == "MoveOrderCmd":
            seen = True
            r = d["robot"]
            segs, done = d["base"], 0
            old = cur.get(r)
            if d.get("update_id", 0) > 0 and old is not None and old[3] == d.get("order_id"):
                # 갱신: 로봇은 옛 base의 남은 조각을 마친 뒤 이어 간다(옛 goal은 곧 SUPERSEDED)
                done = old[2]
                segs = [s for s in old[1] if s["seq_id"] > done] + list(d["base"])
                chain[r].add(d["goal_id"])
            else:
                chain[r] = {d["goal_id"]}
            cur[r] = (d["goal_id"], segs, done, d.get("order_id"))
            set_state(r, t, active(segs, done))
        elif d.get("in") == "on_move_feedback":
            r, gid, done = d["args"][0], d["args"][1], d["args"][2]
            if r in cur and gid in chain.get(r, ()):
                # 갱신 직후 먼저 오는 옛 goal의 feedback도 같은 order의 진행이다
                done = max(done, cur[r][2])
                cur[r] = (cur[r][0], cur[r][1], done, cur[r][3])
                set_state(r, t, active(cur[r][1], done))
        elif d.get("in") == "on_move_result":
            r, gid = d["args"][0], d["args"][1]
            if (r in cur and gid != cur[r][0] and gid in chain.get(r, ())
                    and len(d["args"]) > 4 and d["args"][3] == "SUPERSEDED"):
                done = max(d["args"][4], cur[r][2])
                cur[r] = (cur[r][0], cur[r][1], done, cur[r][3])
                set_state(r, t, active(cur[r][1], done))
            elif r in cur and cur[r][0] == gid:
                del cur[r]
                set_state(r, t, False)
    if not seen:
        return None
    last = rows[-1].get("t", 0.0) if rows else 0.0
    for r in list(open_t):
        set_state(r, last, False)
    return dict(out)


def record_recoveries(rows):
    prev, n = {}, 0
    for d in rows:
        if d.get("in") == "on_robot_state":
            r, s = d["args"][0], d["args"][4]
            if prev.get(r) == "MOVING" and s == "RECOVERING":
                n += 1
            prev[r] = s
    return n, bool(prev)


def record_move_distances(rows, by_seq=False):
    s = defaultdict(list)
    for d in rows:
        if d.get("in") == "on_move_feedback" and len(d["args"]) > 5:
            a = d["args"]
            s[(a[0], a[1], a[2] if by_seq else None)].append(a[5])
    return s


def log_recoveries(launch_out):
    pat = re.compile(r"state MOVING -> RECOVERING")
    return sum(1 for line in launch_out.splitlines() if pat.search(line))


# ---- bag ----
def read_bag(bag_dir, w2s):
    """필요한 토픽만 읽는다. 시간은 truth의 wall->sim_t 보간으로 sim 초로 바꾼다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    res = {"plan": defaultdict(list), "kept": defaultdict(list),
           "cmd": defaultdict(list), "cmd_nav": defaultdict(list),
           "move": defaultdict(list), "nav": defaultdict(list)}
    msg_types = {}
    pat = re.compile(r"^/(pode\d+)/(plan|plan_kept|cmd_vel|cmd_vel_nav|move_to/_action/feedback"
                     r"|navigate_to_pose/_action/feedback)$")
    want = {}
    for name in types:
        m = pat.match(name)
        if m:
            kind = {"plan": "plan", "plan_kept": "kept", "cmd_vel": "cmd", "cmd_vel_nav": "cmd_nav",
                    "move_to/_action/feedback": "move",
                    "navigate_to_pose/_action/feedback": "nav"}[m.group(2)]
            want[name] = (m.group(1), kind)
            msg_types[name] = get_message(types[name])
    rd.set_filter(rosbag2_py.StorageFilter(topics=list(want)))
    while rd.has_next():
        name, data, ts = rd.read_next()
        r, kind = want[name]
        m = deserialize_message(data, msg_types[name])
        t = float(w2s(ts / 1e9))
        if kind in ("plan", "kept"):
            res[kind][r].append((t, [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        elif kind in ("cmd", "cmd_nav"):
            res[kind][r].append((t, m.linear.x, m.angular.z))
        elif kind == "move":
            res[kind][r].append((t, bytes(m.goal_id.uuid), m.feedback.distance_remaining))
        else:
            res[kind][r].append((t, bytes(m.goal_id.uuid), m.feedback.distance_remaining))
    return res


# ---- 시간 창(--until): 중간에 끝난 판도 같은 앞 구간으로 견준다 ----
STAMP = re.compile(r"^\[[^\]]+\] \[[A-Z]+\] \[(\d+\.\d+)\]")
FLEET_EVENT = re.compile(r"\[fleet_manager\]: ([A-Z_]+) ?(.*)$")


def launch_lines_until(launch, w2s, until):
    """launch.out 줄 가운데 sim 시각이 until 이하인 것(시각 없는 줄은 둔다). until None이면 전부."""
    lines = launch.splitlines()
    if until is None:
        return lines
    out = []
    for line in lines:
        m = STAMP.match(line)
        if m is None or w2s(float(m.group(1))) <= until:
            out.append(line)
    return out


def events_from_launch(launch, w2s):
    """events.csv가 없는 판(감시가 끊은 판): fleet_manager 로그 줄에서 사건을 읽는다."""
    ev = []
    for line in launch.splitlines():
        s, e = STAMP.match(line), FLEET_EVENT.search(line)
        if s and e:
            ev.append({"time_s": float(w2s(float(s.group(1)))), "code": e.group(1),
                       "detail": e.group(2)})
    return ev


def pnc_of(events):
    """판정과 같은 셈: 이유가 PATH_NOT_CONNECTED인 GOAL_RETRY와 GOAL_FAILED."""
    return sum(1 for e in events if e["code"] in ("GOAL_RETRY", "GOAL_FAILED")
               and "PATH_NOT_CONNECTED" in e["detail"])


def wall_to_sim(wall, sim):
    """truth의 wall->sim 보간. 기록 밖은 끝 두 점의 기울기로 늘려 잡는다(끝값에 고정하지 않는다)."""
    def w2s(w):
        if len(wall) < 2:
            return float(sim[0]) if len(sim) else float(w)
        if w < wall[0]:
            return float(sim[0] + (w - wall[0]) * (sim[1] - sim[0]) / (wall[1] - wall[0]))
        if w > wall[-1]:
            return float(sim[-1] + (w - wall[-1]) * (sim[-1] - sim[-2]) / (wall[-1] - wall[-2]))
        return float(np.interp(w, wall, sim))
    return w2s


def rtf_of(truth, wall):
    """로봇마다 첫 시각 가운데 가장 이른 것부터 마지막 시각 가운데 가장 늦은 것까지 / 벽시계."""
    spans = [(a[0][0], a[0][-1]) for a in truth.values() if len(a[0])]
    if not spans or len(wall) < 2 or wall[-1] <= wall[0]:
        return float("nan")
    sim0, sim1 = min(a for a, _ in spans), max(b for _, b in spans)
    return round(float(sim1 - sim0) / float(wall[-1] - wall[0]), 3)


def cut_bag(bag, until):
    if bag is None or until is None:
        return bag
    return {kind: {r: [x for x in lst if x[0] <= until] for r, lst in by.items()}
            for kind, by in bag.items()}


def cut_truth(truth, until):
    if until is None:
        return truth
    return {r: tuple(c[a[0] <= until] for c in a) for r, a in truth.items()}


def cut_rows(rows, until):
    if until is None:
        return rows
    return [d for d in rows if d.get("t") is None or d["t"] <= until]


# ---- 판 하나 ----
def read_truth(path):
    rows = defaultdict(list)
    wall, sim = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                row = (float(r["sim_t"]), float(r["x"]), float(r["y"]), float(r["yaw"]))
                w = float(r["wall"])
            except (ValueError, KeyError):
                continue
            rows[r["robot"]].append(row)
            wall.append(w)
            sim.append(float(r["sim_t"]))
    truth = {r: tuple(np.array(c) for c in zip(*v)) for r, v in rows.items()}
    return truth, np.array(wall), np.array(sim)


def mean_col(path, col, skip=1):
    vals = []
    with open(path) as f:
        for i, r in enumerate(csv.DictReader(f)):
            if i >= skip:
                try:
                    vals.append(float(r[col]))
                except (ValueError, KeyError):
                    pass
    return float(np.mean(vals)) if vals else float("nan")


def load_run_data(run):
    """판 폴더에서 분석에 쓰는 원자료를 한 번에 읽는다(그림 스크립트도 쓴다)."""
    run = Path(run)
    truth, wall, sim = read_truth(run / "truth.csv")
    w2s = lambda w: np.interp(w, wall, sim)  # noqa: E731
    bag = read_bag(run / "bag", w2s) if (run / "bag").exists() else None
    return truth, wall, sim, bag


def analyze(run, until=None):
    """until(sim 초)이 있으면 그 시각까지만 센다. 그때는 summary가 없는(감시가 끊은) 판도 센다."""
    run = Path(run)
    summary = run / "result/summary.json"
    summ = json.loads(summary.read_text()) if summary.exists() else {}
    truth_all, wall, sim, bag = load_run_data(run)
    w2s = wall_to_sim(wall, sim)
    truth, bag = cut_truth(truth_all, until), cut_bag(bag, until)
    rows = cut_rows(read_record(run / "result/fleet_record.jsonl"), until)
    full_launch = (run / "launch.out").read_text(errors="replace")
    launch = "\n".join(launch_lines_until(full_launch, w2s, until))
    notes = []
    # pass는 판 전체의 결과다. 출고 수와 판 시간은 창으로 자를 수 없어 창이 있으면 비운다
    o = {"run": run.name, "pass": bool(summ["pass"]) if summ else False,
         "shipped": summ.get("orders_shipped") if until is None else None,
         "duration_s": summ.get("duration_s") if until is None else None,
         "until_s": until}
    if not summ:
        notes.append("summary 없음(끝나지 않은 판): 통과 아님으로 둠")

    # 뒤집힘
    fl, fp_all, off_n = collect_flips(bag)
    by_robot = {r: len(fl[r]) for r in ROBOTS}
    spots = [(r, f["t"], f["new"][0], f["new"][-1]) for r in ROBOTS for f in fl[r]]
    planner_n = sum(len(v) for v in fp_all.values())
    o["flips"] = sum(by_robot.values())
    o["flips_planner"] = planner_n
    o["off_route"] = off_n
    o["flips_by_robot"] = ";".join(f"{r}:{n}" for r, n in by_robot.items() if n)
    o["flip_spots"] = ";".join(
        f"{r}@{t:.0f}s({a[0]:.2f},{a[1]:.2f})->({b[0]:.2f},{b[1]:.2f})" for r, t, a, b in spots)
    if bag is None:
        notes.append("bag 없음: 뒤집힘 0")

    # HALL 제자리 회전
    rects = yaml.safe_load(HALL_YAML.read_text())["zones"]["HALL"]["rects"]
    o["hall_spins"], o["hall_spin_s"], o["nav_goal_src"] = hall_spin_metric(truth, rows, rects)
    if o["hall_spin_s"] is not None:
        o["hall_spin_s"] = round(o["hall_spin_s"], 1)
    else:
        notes.append("nav_goal 구간 못 정함: HALL 회전은 셀 수 없음")

    # recoveries
    n, ok = record_recoveries(rows)
    log_n = log_recoveries(launch)
    o["recoveries"] = n if ok else log_n
    o["recoveries_log"] = log_n

    # 사건 수
    ev_path = run / "result/events.csv"
    if ev_path.exists():
        with open(ev_path) as f:
            ev = [dict(e, time_s=float(e["time_s"])) for e in csv.DictReader(f)]
    else:
        ev = events_from_launch(full_launch, w2s)
        notes.append("events.csv 없음: launch.out의 fleet_manager 줄에서 사건을 읽음")
    if until is not None:
        ev = [e for e in ev if e["time_s"] <= until]
    cnt = Counter(e["code"] for e in ev)
    o["blocked_wait"], o["goal_retry"], o["isolated"] = (
        cnt["BLOCKED_WAIT"], cnt["GOAL_RETRY"], cnt["ROBOT_ISOLATED"])
    if until is None and summ:
        o["path_not_connected"] = summ.get("checks", {}).get("path_not_connected", {}).get("value")
    else:
        o["path_not_connected"] = pnc_of(ev)
    o["pnc_retry"] = sum(1 for e in ev if e["code"] == "GOAL_RETRY"
                         and e["detail"].startswith("PATH_NOT_CONNECTED"))

    # distance 튐
    ser = record_move_distances(rows)
    if ser:
        o["dist_jumps"] = dist_jumps(ser)
        o["dist_jumps_by_seq"] = dist_jumps(record_move_distances(rows, by_seq=True))
        o["dist_src"] = "fleet_record"
    else:
        s = defaultdict(list)
        for r, lst in (bag["move"].items() if bag else []):
            for t, gid, d in lst:
                s[(r, gid)].append(d)
        o["dist_jumps"] = dist_jumps(s)
        o["dist_jumps_by_seq"] = None
        o["dist_src"] = "bag move_to" if s else "none"
    s = defaultdict(list)
    for r, lst in (bag["nav"].items() if bag else []):
        for t, gid, d in lst:
            s[(r, gid)].append(d)
    o["nav_dist_jumps"] = dist_jumps(s)

    # RotationShim 제자리 회전
    o["shim_spins"] = shim_spins(pick_cmds(bag))

    # 나머지
    o["collision_ahead"] = sum(1 for line in launch.splitlines() if "collision ahead" in line)
    o["rtf"] = rtf_of(truth_all, wall)  # 판 전체
    o["pc_cpu"] = round(mean_col(run / "cpu.csv", "pc_cpu_pct"), 1) if (run / "cpu.csv").exists() else None
    o["domain_cpu"] = (round(mean_col(run / "cpu.csv", "domain_cpu_pct"), 1)
                       if (run / "cpu.csv").exists() else None)
    sc = 0.0
    if (run / "cpu_proc.csv").exists():
        with open(run / "cpu_proc.csv") as f:
            for r in csv.DictReader(f):
                if "sim_stand_in" in r["process"]:
                    sc += float(r["cpu_s"])
    o["stand_in_cpu_s"] = round(sc, 1)
    if until is not None:
        # CPU 기록은 판 전체 값이라 창 값처럼 보이지 않게 비운다(CPU는 창 없이 견준다)
        o["pc_cpu"] = o["domain_cpu"] = o["stand_in_cpu_s"] = None
    o["notes"] = "; ".join(notes)
    return o


# ---- 표 ----
COLS = ["run", "pass", "shipped", "duration_s", "flips", "flips_planner", "off_route", "hall_spins",
        "hall_spin_s", "recoveries", "blocked_wait", "goal_retry", "isolated", "path_not_connected",
        "pnc_retry", "dist_jumps", "nav_dist_jumps", "shim_spins", "collision_ahead", "rtf", "pc_cpu",
        "domain_cpu", "stand_in_cpu_s"]
EXTRA = ["until_s", "recoveries_log", "dist_jumps_by_seq", "dist_src", "nav_goal_src", "flips_by_robot",
         "flip_spots", "notes"]


def prefix(name):
    return re.sub(r"\d+$", "", name)


def group_rows(results):
    g = defaultdict(list)
    for o in results:
        g[prefix(o["run"])].append(o)
    out = []
    for p, lst in g.items():
        for kind, fn in (("mean", np.mean), ("sum", np.sum)):
            row = {"run": f"{p}* {kind}(n={len(lst)})"}
            for c in COLS[1:]:
                v = [float(x[c]) for x in lst if isinstance(x.get(c), (int, float, bool))]
                row[c] = round(float(fn(v)), 2) if v else None
            out.append(row)
    return out


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "Y" if v else "N"
    if isinstance(v, float):
        return f"{v:.1f}" if abs(v) >= 100 else f"{v:g}"
    return str(v)


def print_table(rows):
    cells = [COLS] + [[fmt(r.get(c)) for c in COLS] for r in rows]
    w = [max(len(row[i]) for row in cells) for i in range(len(COLS))]
    for row in cells:
        print("  ".join(x.ljust(w[i]) if i == 0 else x.rjust(w[i]) for i, x in enumerate(row)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--csv")
    ap.add_argument("--until", type=float, help="이 sim 초까지만 센다(끝나지 않은 판도 넣는다)")
    a = ap.parse_args()
    results = []
    for r in a.runs:
        if a.until is None and not (Path(r) / "result/summary.json").exists():
            print(f"건너뜀(끝나지 않음): {r}", file=sys.stderr)
            continue
        results.append(analyze(r, a.until))
    if not results:
        sys.exit("끝난 판이 없다")
    print_table(results)
    print()
    print_table(group_rows(results))
    print()
    for o in results:
        print(f"[{o['run']}] 뒤집힘 {o['flips_by_robot'] or '-'} | {o['flip_spots'] or '-'}")
        print(f"  nav_goal={o['nav_goal_src']} dist={o['dist_src']} recov_log={o['recoveries_log']} "
              f"dist_by_seq={o['dist_jumps_by_seq']} {o['notes']}")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            wr = csv.DictWriter(f, COLS + EXTRA, extrasaction="ignore")
            wr.writeheader()
            wr.writerows(results)


if __name__ == "__main__":
    main()
