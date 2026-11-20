import csv
import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
KIT = ROOT / "tools" / "real_pode"
sys.path.insert(0, str(KIT))

import cmd_step_probe as probe  # noqa: E402
import path_overlay  # noqa: E402

np = pytest.importorskip("numpy")
import step_report  # noqa: E402

TEST_DOMAIN = "63"


# ---- 일정 ----
def test_linear_schedule_has_every_step_three_times():
    sched = probe.build_schedule("linear")
    steps = [s for s in sched if s.phase == "step"]
    assert [s.v for s in steps] == list(probe.LINEAR_STEPS) * 3
    assert all(s.duration == 2.0 and s.w == 0.0 for s in steps)
    assert [s.phase for s in sched[:4]] == ["pause", "step", "stop", "home"]
    assert sched[0].duration == 2.0 and sched[2].duration == 2.0


@pytest.mark.parametrize("name", probe.SETS)
def test_every_default_set_stays_inside_the_default_limit(name):
    text, worst = probe.describe(probe.build_schedule(name), 0.6)
    assert worst <= 0.6
    assert "OVER LIMIT" not in text


def test_small_rotation_covers_both_speeds():
    steps = [s for s in probe.build_schedule("small_rot", repeats=1) if s.phase == "step"]
    assert {(s.v, s.w) for s in steps} == {(v, w) for v in (0.0, 0.12)
                                           for w in (0.05, 0.1, 0.2, 0.3)}


def test_arc_runs_left_and_right():
    steps = [s for s in probe.build_schedule("arc", repeats=1) if s.phase == "step"]
    assert [(s.v, s.w) for s in steps] == [(0.10, 0.5), (0.10, -0.5)]


def test_dry_run_prints_schedule(capsys):
    assert probe.main(["--set", "linear", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "v0.25" in out and "distance limit 0.60 m" in out


def test_a_step_over_the_limit_is_refused(capsys):
    assert probe.main(["--set", "linear", "--limit", "0.3"]) == probe.EXIT_REFUSED
    assert "refusing" in capsys.readouterr().out


# ---- 안전 ----
def test_distance_limit():
    assert not probe.over_limit((0, 0), (0.3, 0.4), 0.6)
    assert probe.over_limit((0, 0), (0.37, 0.48), 0.6)


def test_other_publishers_are_found_but_the_watchdog_is_allowed():
    assert probe.foreign_publishers([("cmd_watchdog", b"1"), ("cmd_step_probe", b"2")],
                                    own=(b"2",)) == []
    assert probe.foreign_publishers([("controller_server", b"3"), ("cmd_watchdog", b"1")]) == [
        "controller_server"]


def test_another_probe_is_foreign_even_with_our_name():
    assert probe.foreign_publishers([("cmd_step_probe", b"9")]) == ["cmd_step_probe"]
    assert probe.foreign_publishers([("cmd_step_probe", b"2"), ("cmd_step_probe", b"9")],
                                    own=(b"2",)) == ["cmd_step_probe"]


def test_non_finite_odom_and_commands_are_caught():
    assert probe.all_finite((0.1, 0.0, 1.0, 2.0, 0.3))
    for bad in (math.nan, math.inf, -math.inf):
        assert not probe.all_finite((0.1, 0.0, bad, 2.0, 0.3))
    assert probe.over_limit((0, 0), (math.nan, 0.0), 0.6)
    assert probe.EXIT_BAD_ODOM not in (probe.EXIT_ABORTED, probe.EXIT_REFUSED)


def test_stale_odom():
    assert probe.odom_stale(None, 5.0, 1.0)
    assert probe.odom_stale(3.0, 4.5, 1.0)
    assert not probe.odom_stale(4.0, 4.5, 1.0)


def test_zero_burst():
    sent = []
    probe.send_zeros(lambda v, w: sent.append((v, w)), sleep=lambda s: None)
    assert sent == [(0.0, 0.0)] * probe.ZERO_BURST


def test_zero_burst_survives_a_logging_error():
    sent = []

    def broken_log():
        raise OSError("disk full")

    probe.send_zeros(lambda v, w: sent.append((v, w)), sleep=lambda s: None, after=broken_log)
    assert sent == [(0.0, 0.0)] * probe.ZERO_BURST


def test_zero_burst_ignores_ctrl_c_and_sigterm():
    sent = []
    before = signal.getsignal(signal.SIGINT)

    def publish(v, w):
        if not sent:
            os.kill(os.getpid(), signal.SIGINT)
            os.kill(os.getpid(), signal.SIGTERM)
        sent.append((v, w))

    probe.send_zeros(publish, sleep=lambda s: time.sleep(0.01))
    assert sent == [(0.0, 0.0)] * probe.ZERO_BURST
    assert signal.getsignal(signal.SIGINT) is before


def test_home_move_that_gets_farther_is_caught():
    assert not probe.home_diverged(0.30, 0.28)
    assert probe.home_diverged(0.34, 0.28)
    assert probe.EXIT_HOME_DIVERGED not in (probe.EXIT_ABORTED, probe.EXIT_BAD_ODOM)


@pytest.mark.parametrize("start", [(0.5, 0.0, 0.0), (-0.3, 0.2, 2.0), (0.0, 0.0, 3.0)])
def test_home_move_reaches_the_start(start):
    x, y, yaw = start
    dt = 0.05
    for _ in range(2000):
        v, w, done = probe.home_command((x, y, yaw), (0.0, 0.0, 0.0))
        if done:
            break
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
    assert done
    assert math.hypot(x, y) < 0.03 and abs(probe.wrap(yaw)) < 0.05


# ---- 합성 기록 ----
def _row(kind, t, seg, cmd, odom=("", "", "", "", "")):
    return {"kind": kind, "t_mono": t, "t_ros": f"{t:.6f}", "t_stamp": "", "set": seg.set,
            "repeat": seg.repeat, "step": seg.step, "phase": seg.phase, "label": seg.label,
            "cmd_v": cmd[0], "cmd_w": cmd[1], "odom_v": odom[0], "odom_w": odom[1],
            "x": odom[2], "y": odom[3], "yaw": odom[4], "disp": ""}


def synth(schedule, plant, rate=30.0, cmd_rate=20.0, noise=0.0005, seed=1, home=False):
    rng = random.Random(seed)
    rows, t = [], 0.0
    x = y = yaw = 0.0
    for seg in schedule:
        if seg.phase == "home" and not home:
            continue
        t_end = t + seg.duration
        tc = t
        to = t + rng.random() / rate
        state = plant.setdefault("_state", {"v": 0.0, "w": 0.0, "since": None})
        while tc < t_end:
            rows.append(_row("cmd", tc, seg, (seg.v, seg.w)))
            tc += 1.0 / cmd_rate
        prev = t
        while to < t_end:
            v, w = plant["f"](seg, to - t, state)
            dt = to - prev
            yaw += w * dt
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            prev = to
            rows.append(_row("odom", to, seg, (seg.v, seg.w),
                             (v + rng.gauss(0, noise), w + rng.gauss(0, noise), x, y, yaw)))
            to += 1.0 / rate
        t = t_end
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=probe.COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return path


LAT, ACC, ACC_W, GAIN = 0.12, 0.8, 4.0, 0.95


def ramp_plant(seg, dt, state):
    target = (seg.v if seg.v else seg.w) * GAIN
    if seg.phase != "step":
        return 0.0, 0.0
    acc = ACC if seg.v else ACC_W
    level = math.copysign(min(abs(target), max(0.0, acc * (dt - LAT))), target)
    return (level, 0.0) if seg.v else (0.0, level)


def test_step_metrics_recover_latency_rise_ratio_and_accel(tmp_path):
    sched = probe.build_schedule("linear", repeats=1)
    rows = synth(sched, {"f": ramp_plant})
    segs = step_report.segments(step_report.load_rows([write_csv(rows, tmp_path / "s.csv")]))
    for seg in segs:
        if seg["v"] < 0.15:
            continue
        m = step_report.step_metrics(seg["t"], seg["ov"], seg["v"], seg["t0"], seg["t1"],
                                     seg["t2"])
        assert m["latency"] == pytest.approx(LAT + 0.1 * seg["v"] / ACC, abs=0.01)
        assert m["rise"] == pytest.approx(0.8 * seg["v"] / ACC, abs=0.015)
        assert m["ratio"] == pytest.approx(GAIN, abs=0.01)
        assert m["acc_max"] == pytest.approx(ACC, rel=0.1)


def test_angular_steps_use_odom_w(tmp_path):
    rows = synth(probe.build_schedule("angular", repeats=1), {"f": ramp_plant})
    segs = step_report.segments(step_report.load_rows([write_csv(rows, tmp_path / "a.csv")]))
    for seg in segs:
        target, y = step_report.response_of(seg)
        m = step_report.step_metrics(seg["t"], y, target, seg["t0"], seg["t1"], seg["t2"])
        assert target == seg["w"]
        assert m["latency"] == pytest.approx(LAT + 0.1 * seg["w"] / ACC_W, abs=0.01)
        assert m["rise"] == pytest.approx(0.8 * seg["w"] / ACC_W, abs=0.015)
        assert m["ratio"] == pytest.approx(GAIN, abs=0.01)
        assert m["acc_max"] == pytest.approx(ACC_W, rel=0.1)


def short_plant(seg, dt, state):
    if seg.phase == "step":
        return min(0.8 * seg.v, ACC * max(0.0, dt - LAT)), 0.0
    return (0.10, 0.0) if seg.phase == "home" else (0.0, 0.0)


def test_rise_is_not_reached_when_only_the_home_move_passes_90_percent(tmp_path):
    sched = [s for s in probe.build_schedule("linear", repeats=1, home_s=2.0)
             if s.label == "v0.10"]
    rows = synth(sched, {"f": short_plant}, home=True)
    segs = step_report.segments(step_report.load_rows([write_csv(rows, tmp_path / "h.csv")]))
    assert len(segs) == 1
    seg = segs[0]
    assert np.nanmax(seg["ov"][seg["t"] > seg["t2"]]) > 0.09
    m = step_report.step_metrics(seg["t"], seg["ov"], seg["v"], seg["t0"], seg["t1"], seg["t2"])
    assert math.isfinite(m["latency"])
    assert math.isnan(m["rise"])
    assert m["ratio"] == pytest.approx(0.8, abs=0.01)
    assert "미도달" in step_report.summarize(segs)


def dead_plant(seg, dt, state):
    if seg.phase != "step":
        return 0.0, 0.0
    floor = 0.15 if seg.v == 0 else 0.07
    return seg.v, (0.98 * seg.w if abs(seg.w) >= floor else 0.0)


def test_deadband_is_the_smallest_responding_turn(tmp_path):
    rows = synth(probe.build_schedule("small_rot", repeats=2), {"f": dead_plant})
    segs = step_report.segments(step_report.load_rows([write_csv(rows, tmp_path / "d.csv")]))
    table, db = step_report.deadband(segs)
    assert db == {0.0: 0.2, 0.12: 0.1}
    assert table[(0.0, 0.1)] == pytest.approx(0.0, abs=0.02)


def arc_plant(seg, dt, state):
    if seg.phase != "step":
        return 0.0, 0.0
    radius = abs(seg.v / seg.w) * (0.9 if seg.w > 0 else 1.0)
    return seg.v, math.copysign(seg.v / radius, seg.w)


def test_arc_radius_and_left_right_difference(tmp_path):
    rows = synth(probe.build_schedule("arc", repeats=1), {"f": arc_plant})
    segs = step_report.segments(step_report.load_rows([write_csv(rows, tmp_path / "c.csv")]))
    got = {("left" if s["w"] > 0 else "right"): step_report.arc_metrics(s) for s in segs}
    assert got["left"]["r_fit"] == pytest.approx(0.18, rel=0.01)
    assert got["right"]["r_fit"] == pytest.approx(0.20, rel=0.01)
    assert got["left"]["r_twist"] == pytest.approx(0.18, rel=0.02)
    text = step_report.summarize(segs)
    assert "차이 -0.020 m" in text


def test_report_writes_markdown_and_png(tmp_path):
    rows = synth(probe.build_schedule("linear", repeats=1), {"f": ramp_plant})
    rows += synth(probe.build_schedule("arc", repeats=1), {"f": arc_plant})
    csv_path = write_csv(rows, tmp_path / "all.csv")
    pytest.importorskip("matplotlib")
    assert step_report.main([str(csv_path), "--out", str(tmp_path / "rep")]) == 0
    assert (tmp_path / "rep" / "report.md").read_text(encoding="utf-8").count("| linear |") >= 5
    assert (tmp_path / "rep" / "linear.png").is_file() and (tmp_path / "rep" / "arc.png").is_file()


# ---- 경로 겹쳐 보기 ----
def test_planned_path_is_read_from_the_map():
    points, nodes = path_overlay.planned_path()
    assert points[0] == nodes["W"] and points[-1] == nodes["O"]
    cs = path_overlay.corners(points)
    assert len(cs) == 1 and math.degrees(cs[0]["turn"]) == pytest.approx(90, abs=3)


def _offset_path(points, bump):
    out = []
    s = path_overlay.cumulative(points)
    for (ax, ay), (bx, by), s0, s1 in zip(points, points[1:], s, s[1:], strict=False):
        n = max(2, int((s1 - s0) / 0.005))
        dx, dy = (bx - ax) / (s1 - s0), (by - ay) / (s1 - s0)
        for k in range(n):
            u = k / n
            off = bump(s0 + u * (s1 - s0))
            out.append((ax + u * (bx - ax) + dy * off, ay + u * (by - ay) - dx * off))
    return out


def test_outward_deviation_at_the_corner():
    points, _ = path_overlay.planned_path()
    c = path_overlay.corners(points)[0]
    mid = (c["s_from"] + c["s_to"]) / 2
    samples = _offset_path(points, lambda s: 0.02 * math.exp(-((s - mid) / 0.03) ** 2))
    worst, per, used = path_overlay.deviations(points, samples)
    assert worst["outward"] == pytest.approx(0.02, abs=0.001)
    assert per[0]["outward"] == pytest.approx(0.02, abs=0.001)
    assert used > 0.9 * len(samples)


def test_inward_cut_is_not_outward():
    points, _ = path_overlay.planned_path()
    c = path_overlay.corners(points)[0]
    mid = (c["s_from"] + c["s_to"]) / 2
    samples = _offset_path(points, lambda s: -0.02 * math.exp(-((s - mid) / 0.03) ** 2))
    worst, _, _ = path_overlay.deviations(points, samples)
    assert worst["outward"] < 0.001
    assert worst["abs"] == pytest.approx(0.02, abs=0.001)


def _bump_at(mid, amp):
    return lambda s: amp * math.exp(-((s - mid) / 0.03) ** 2)


def test_a_30_cm_outward_excursion_at_the_corner_is_reported():
    points, _ = path_overlay.planned_path()
    c = path_overlay.corners(points)[0]
    mid = (c["s_from"] + c["s_to"]) / 2
    samples = _offset_path(points, _bump_at(mid, 0.30))
    worst, per, used = path_overlay.deviations(points, samples)
    assert worst["outward"] == pytest.approx(0.30, abs=0.005)
    assert per[0]["outward"] == pytest.approx(0.30, abs=0.005)
    assert worst["abs"] == pytest.approx(0.30, abs=0.005)


def test_a_30_cm_excursion_on_a_straight_is_reported():
    points, _ = path_overlay.planned_path()
    s_end = path_overlay.cumulative(points)[-1]
    samples = _offset_path(points, _bump_at(s_end - 0.6, 0.30))
    worst, per, _ = path_overlay.deviations(points, samples)
    assert worst["abs"] == pytest.approx(0.30, abs=0.005)
    assert per[0]["outward"] < 0.001


def test_overlay_never_prints_minus_infinity(tmp_path, capsys):
    pytest.importorskip("matplotlib")
    points, _ = path_overlay.planned_path()
    (ax, ay), (bx, by) = points[-2], points[-1]
    path = tmp_path / "straight.csv"
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=path_overlay.COLUMNS)
        w.writeheader()
        for k in range(20):
            u = k / 19
            w.writerow({"kind": "amcl", "t_mono": k, "t_ros": k, "t_stamp": k,
                        "x": ax + u * (bx - ax), "y": ay + u * (by - ay), "yaw": 0})
    args = path_overlay.argparse.Namespace(csv=str(path), route=",".join(path_overlay.ROUTE),
                                           layout="v7", png=str(tmp_path / "o.png"))
    assert path_overlay.overlay(args) == 0
    out = capsys.readouterr().out
    assert "inf" not in out
    assert "no samples" in out


def test_odom_samples_go_through_map_to_odom():
    rows = [{"kind": "map_odom", "t_ros": "1.0", "x": "1.0", "y": "2.0", "yaw": str(math.pi / 2)},
            {"kind": "odom", "t_ros": "1.5", "x": "0.5", "y": "0.0", "yaw": "0"},
            {"kind": "odom", "t_ros": "0.5", "x": "9", "y": "9", "yaw": "0"}]
    samples, source = path_overlay.map_samples(rows)
    assert source.startswith("odom")
    assert samples == [pytest.approx((1.0, 2.5))]


# ---- rclpy로 안전 확인(도메인 63) ----
FAKE = r'''
import math, sys, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
ns, log, mode = sys.argv[1], sys.argv[2], sys.argv[3]
rclpy.init()
node = rclpy.create_node("fake_base")
s = {"v": 0.0, "w": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "moved": False}
fh = open(log, "w")
def on_cmd(m):
    s["v"], s["w"] = m.linear.x, m.angular.z
    s["moved"] |= m.linear.x != 0 or m.angular.z != 0
    fh.write(f"{m.linear.x} {m.angular.z}\n"); fh.flush()
node.create_subscription(Twist, f"/{ns}/cmd_vel", on_cmd, 10)
pub = node.create_publisher(Odometry, f"/{ns}/odom", 10)
if mode == "intruder":
    other = rclpy.create_node("intruder")
    other.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
def tick():
    dt = 1 / 30
    v = -s["v"] if mode == "reverse" else s["v"]
    s["yaw"] += s["w"] * dt
    s["x"] += v * math.cos(s["yaw"]) * dt + (0.05 if mode == "jump" and s["moved"] else 0)
    s["y"] += v * math.sin(s["yaw"]) * dt
    m = Odometry()
    m.pose.pose.position.x, m.pose.pose.position.y = s["x"], s["y"]
    if mode == "nan" and s["moved"]:
        m.pose.pose.position.x = math.nan
    q = m.pose.pose.orientation
    q.z, q.w = math.sin(s["yaw"] / 2), math.cos(s["yaw"] / 2)
    m.twist.twist.linear.x, m.twist.twist.angular.z = s["v"], s["w"]
    pub.publish(m)
node.create_timer(1 / 30, tick)
try:
    rclpy.spin(node)
except BaseException:
    pass
'''


@pytest.fixture
def ros_env(tmp_path):
    pytest.importorskip("rclpy")
    env = dict(os.environ, ROS_DOMAIN_ID=TEST_DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               PYTHONUNBUFFERED="1")
    env.pop("ROS_DISCOVERY_SERVER", None)
    env.pop("ROS_STATIC_PEERS", None)
    fake = tmp_path / "fake_base.py"
    fake.write_text(FAKE, encoding="utf-8")
    procs = []

    def start(ns, mode):
        log = tmp_path / f"{ns}.log"
        p = subprocess.Popen([sys.executable, str(fake), ns, str(log), mode], env=env)
        procs.append(p)
        return log

    yield env, start
    for p in procs:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()


def _probe(env, ns, out, *extra):
    return subprocess.Popen([sys.executable, str(KIT / "cmd_step_probe.py"), "--ns", ns,
                             "--set", "angular", "--repeats", "1", "--step-s", "0.5",
                             "--pause-s", "0.5", "--out", str(out), *extra],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def _cmds(log):
    return [tuple(map(float, line.split())) for line in log.read_text().splitlines() if line]


def test_probe_refuses_when_cmd_vel_has_another_publisher(ros_env, tmp_path):
    env, start = ros_env
    ns = f"t{os.getpid()}a"
    log = start(ns, "intruder")
    p = _probe(env, ns, tmp_path / "a.csv")
    out, _ = p.communicate(timeout=30)
    assert p.returncode == probe.EXIT_REFUSED, out
    assert "intruder" in out
    assert _cmds(log) == []


def test_probe_stops_at_the_distance_limit_and_sends_zeros(ros_env, tmp_path):
    env, start = ros_env
    ns = f"t{os.getpid()}b"
    log = start(ns, "jump")
    p = _probe(env, ns, tmp_path / "b.csv", "--limit", "0.3")
    out, _ = p.communicate(timeout=30)
    assert p.returncode == probe.EXIT_ABORTED, out
    assert "limit" in out
    time.sleep(0.3)
    assert _cmds(log)[-probe.ZERO_BURST:] == [(0.0, 0.0)] * probe.ZERO_BURST


def test_probe_sends_zeros_on_ctrl_c(ros_env, tmp_path):
    env, start = ros_env
    ns = f"t{os.getpid()}c"
    log = start(ns, "plain")
    p = _probe(env, ns, tmp_path / "c.csv", "--step-s", "5")
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        if log.exists() and any(c != (0.0, 0.0) for c in _cmds(log)):
            break
        time.sleep(0.05)
    else:
        p.kill()
        pytest.fail("probe never sent a moving command")
    p.send_signal(signal.SIGINT)
    out, _ = p.communicate(timeout=15)
    assert p.returncode == probe.EXIT_INTERRUPTED, out
    time.sleep(0.3)
    assert _cmds(log)[-probe.ZERO_BURST:] == [(0.0, 0.0)] * probe.ZERO_BURST
    rows = list(csv.DictReader(open(tmp_path / "c.csv", encoding="utf-8")))
    assert {r["kind"] for r in rows} == {"cmd", "odom"}


def test_a_second_probe_is_refused(ros_env, tmp_path):
    env, start = ros_env
    ns = f"t{os.getpid()}d"
    log = start(ns, "plain")
    first = _probe(env, ns, tmp_path / "d1.csv", "--step-s", "8")
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        if log.exists() and any(c != (0.0, 0.0) for c in _cmds(log)):
            break
        time.sleep(0.05)
    else:
        first.kill()
        pytest.fail("first probe never sent a moving command")
    second = _probe(env, ns, tmp_path / "d2.csv")
    out, _ = second.communicate(timeout=30)
    first.send_signal(signal.SIGINT)
    first_out, _ = first.communicate(timeout=15)
    assert second.returncode == probe.EXIT_REFUSED, out
    assert "cmd_step_probe" in out
    assert first.returncode == probe.EXIT_INTERRUPTED, first_out


def test_probe_stops_on_non_finite_odom(ros_env, tmp_path):
    env, start = ros_env
    ns = f"t{os.getpid()}e"
    log = start(ns, "nan")
    p = _probe(env, ns, tmp_path / "e.csv", "--home-s", "5")
    out, _ = p.communicate(timeout=30)
    assert p.returncode == probe.EXIT_BAD_ODOM, out
    time.sleep(0.3)
    cmds = _cmds(log)
    assert all(math.isfinite(v) and math.isfinite(w) for v, w in cmds)
    assert cmds[-probe.ZERO_BURST:] == [(0.0, 0.0)] * probe.ZERO_BURST


def test_probe_stops_when_the_home_move_gets_farther(ros_env, tmp_path):
    env, start = ros_env
    ns = f"t{os.getpid()}f"
    log = start(ns, "reverse")
    p = _probe(env, ns, tmp_path / "f.csv", "--set", "linear", "--step-s", "0.5",
               "--home-s", "10")
    out, _ = p.communicate(timeout=60)
    assert p.returncode == probe.EXIT_HOME_DIVERGED, out
    time.sleep(0.3)
    assert _cmds(log)[-probe.ZERO_BURST:] == [(0.0, 0.0)] * probe.ZERO_BURST
