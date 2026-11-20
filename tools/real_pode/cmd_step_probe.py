#!/usr/bin/env python3
import argparse
import csv
import json
import math
import signal
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

SETS = ("linear", "angular", "small_rot", "arc")
LINEAR_STEPS = (0.05, 0.10, 0.15, 0.20, 0.25)
ANGULAR_STEPS = (0.5, 1.0, 1.5)
SMALL_ROT_STEPS = (0.05, 0.1, 0.2, 0.3)
SMALL_ROT_SPEEDS = (0.0, 0.12)
ALLOWED_PUBLISHERS = ("cmd_watchdog",)
ZERO_BURST = 5
ZERO_GAP_S = 0.02
EXIT_DONE, EXIT_USAGE, EXIT_REFUSED, EXIT_ABORTED, EXIT_INTERRUPTED = 0, 2, 3, 4, 130
COLUMNS = ("kind", "t_mono", "t_ros", "t_stamp", "set", "repeat", "step", "phase", "label",
           "cmd_v", "cmd_w", "odom_v", "odom_w", "x", "y", "yaw", "disp")


# ---- 일정 ----
@dataclass
class Segment:
    set: str
    repeat: int
    step: int
    phase: str
    label: str
    v: float
    w: float
    duration: float


def build_schedule(set_name, repeats=3, step_s=2.0, pause_s=2.0, small_s=3.0, arc_s=4.0,
                   arc_v=0.10, arc_w=0.5, go_home=True, home_s=30.0):
    if set_name not in SETS:
        raise ValueError(f"unknown set {set_name!r}; use one of {', '.join(SETS)}")
    if set_name == "linear":
        moves = [(f"v{v:.2f}", v, 0.0, step_s) for v in LINEAR_STEPS]
    elif set_name == "angular":
        moves = [(f"w{w:.2f}", 0.0, w, step_s) for w in ANGULAR_STEPS]
    elif set_name == "small_rot":
        moves = [(f"v{v:.2f}_w{w:.2f}", v, w, small_s)
                 for v in SMALL_ROT_SPEEDS for w in SMALL_ROT_STEPS]
    else:
        moves = [(f"arc_{side}_v{arc_v:.2f}_w{arc_w:.2f}", arc_v, sign * arc_w, arc_s)
                 for side, sign in (("left", 1.0), ("right", -1.0))]
    out, step = [], 0
    for repeat in range(1, repeats + 1):
        for label, v, w, duration in moves:
            step += 1
            out.append(Segment(set_name, repeat, step, "pause", label, 0.0, 0.0, pause_s))
            out.append(Segment(set_name, repeat, step, "step", label, v, w, duration))
            out.append(Segment(set_name, repeat, step, "stop", label, 0.0, 0.0, pause_s))
            if go_home:
                out.append(Segment(set_name, repeat, step, "home", label, 0.0, 0.0, home_s))
    return out


def reach(seg):
    if seg.phase != "step":
        return 0.0
    length = abs(seg.v) * seg.duration
    if abs(seg.w) < 1e-9 or seg.v == 0.0:
        return length
    radius = abs(seg.v / seg.w)
    turned = min(abs(seg.w) * seg.duration, math.pi)
    return 2.0 * radius * math.sin(turned / 2.0)


def describe(schedule, limit):
    lines, t = [], 0.0
    lines.append(f"{'t_s':>7} {'set':<9} {'rep':>3} {'step':>4} {'phase':<5} {'label':<24}"
                 f" {'v':>6} {'w':>6} {'dur':>5} {'reach_m':>7}")
    worst = 0.0
    for seg in schedule:
        far = reach(seg)
        worst = max(worst, far)
        dur = "<=" + f"{seg.duration:.0f}" if seg.phase == "home" else f"{seg.duration:.1f}"
        flag = "  OVER LIMIT" if far > limit else ""
        lines.append(f"{t:7.1f} {seg.set:<9} {seg.repeat:>3} {seg.step:>4} {seg.phase:<5}"
                     f" {seg.label:<24} {seg.v:6.2f} {seg.w:6.2f} {dur:>5} {far:7.3f}{flag}")
        if seg.phase != "home":
            t += seg.duration
    lines.append(f"open-loop time {t:.0f} s (+ home moves), largest step reach {worst:.3f} m,"
                 f" distance limit {limit:.2f} m")
    return "\n".join(lines), worst


# ---- 안전 ----
def displacement(start, pose):
    return math.hypot(pose[0] - start[0], pose[1] - start[1])


def over_limit(start, pose, limit):
    return displacement(start, pose) > limit


def foreign_publishers(node_names, allowed=ALLOWED_PUBLISHERS, own=""):
    return sorted(n for n in node_names if n not in allowed and n != own)


def odom_stale(last_odom_mono, now_mono, timeout_s):
    return last_odom_mono is None or now_mono - last_odom_mono > timeout_s


def send_zeros(publish, sleep=time.sleep, count=ZERO_BURST, gap=ZERO_GAP_S):
    for _ in range(count):
        publish(0.0, 0.0)
        sleep(gap)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def home_command(pose, home, max_v=0.10, max_w=0.6, min_w=0.3, pos_tol=0.03, yaw_tol=0.05):
    dx, dy = home[0] - pose[0], home[1] - pose[1]
    dist = math.hypot(dx, dy)

    def turn(err):
        w = max(-max_w, min(max_w, 2.0 * err))
        if abs(w) < min_w:
            w = math.copysign(min_w, err)
        return w

    if dist < pos_tol:
        err = wrap(home[2] - pose[2])
        if abs(err) < yaw_tol:
            return 0.0, 0.0, True
        return 0.0, turn(err), False
    err = wrap(math.atan2(dy, dx) - pose[2])
    backward = abs(err) > math.pi / 2
    if backward:
        err = wrap(err - math.pi)
    if abs(err) > 0.35:
        return 0.0, turn(err), False
    v = min(max_v, max(0.03, 1.5 * dist))
    return (-v if backward else v), max(-max_w, min(max_w, 2.0 * err)), False


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---- 인자 ----
def parse_args(argv):
    p = argparse.ArgumentParser(description="cmd_vel step probe (Nav2 must be off)")
    p.add_argument("--ns", default="pode1")
    p.add_argument("--set", dest="set_name", required=True, choices=SETS)
    p.add_argument("--out", default="", help="CSV path (default step_<set>_<time>.csv)")
    p.add_argument("--limit", type=float, default=0.6, help="stop if odom moves this far (m)")
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--step-s", type=float, default=2.0)
    p.add_argument("--pause-s", type=float, default=2.0)
    p.add_argument("--small-s", type=float, default=3.0)
    p.add_argument("--arc-s", type=float, default=4.0)
    p.add_argument("--arc-v", type=float, default=0.10)
    p.add_argument("--arc-w", type=float, default=0.5)
    p.add_argument("--home-s", type=float, default=30.0, help="give up a home move after this")
    p.add_argument("--no-home", action="store_true", help="do not drive back to the start")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--odom-timeout", type=float, default=1.0)
    p.add_argument("--allow", default=",".join(ALLOWED_PUBLISHERS),
                   help="other cmd_vel publisher node names that may stay")
    p.add_argument("--use-sim-time", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args(argv)
    if not (0.0 < args.limit <= 1.5) or args.repeats < 1 or args.rate <= 0:
        p.error("need 0 < --limit <= 1.5, --repeats >= 1, --rate > 0")
    return args


def schedule_of(args):
    return build_schedule(args.set_name, args.repeats, args.step_s, args.pause_s, args.small_s,
                          args.arc_s, args.arc_v, args.arc_w, not args.no_home, args.home_s)


# ---- 실행 ----
class Aborted(Exception):
    pass


def run(args, schedule):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("cmd_step_probe", parameter_overrides=[
        Parameter("use_sim_time", value=args.use_sim_time)])
    log = node.get_logger()
    cmd_topic, odom_topic = f"/{args.ns}/cmd_vel", f"/{args.ns}/odom"
    allowed = tuple(n for n in args.allow.split(",") if n)
    state = {"odom": None, "mono": None, "seg": None, "home": None}
    out = Path(args.out or f"step_{args.set_name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    fh = out.open("w", newline="", encoding="utf-8")
    writer = csv.writer(fh)
    writer.writerow(COLUMNS)

    def ros_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def row(kind, cmd, odom=None, stamp=""):
        seg = state["seg"]
        meta = (seg.set, seg.repeat, seg.step, seg.phase, seg.label) if seg else ("",) * 5
        pose = odom or ("", "", "", "", "")
        disp = displacement(state["home"], odom[2:]) if odom and state["home"] else ""
        writer.writerow((kind, f"{time.monotonic():.6f}", f"{ros_now():.6f}", stamp, *meta,
                         f"{cmd[0]:.4f}", f"{cmd[1]:.4f}", *pose, disp))

    sent = [0.0, 0.0]

    def on_odom(msg):
        p, t = msg.pose.pose, msg.twist.twist
        odom = (t.linear.x, t.angular.z, p.position.x, p.position.y, yaw_of(p.orientation))
        state["odom"], state["mono"] = odom, time.monotonic()
        stamp = f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.6f}"
        row("odom", sent, odom, stamp)

    node.create_subscription(Odometry, odom_topic, on_odom, qos_profile_sensor_data)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def others():
        names = [i.node_name for i in node.get_publishers_info_by_topic(cmd_topic)]
        return foreign_publishers(names, allowed, own=node.get_name())

    pub = None

    def publish(v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        pub.publish(msg)
        sent[0], sent[1] = float(v), float(w)
        row("cmd", sent)

    code = EXIT_DONE
    try:
        spin_for(2.0)
        found = others()
        if found:
            log.error(f"refusing: {cmd_topic} already has publishers {found}; stop Nav2 first")
            code = EXIT_REFUSED
            return code
        if state["odom"] is None:
            spin_for(3.0)
        if state["odom"] is None or (args.use_sim_time and ros_now() == 0.0):
            log.error(f"refusing: no odom on {odom_topic} (or no /clock with --use-sim-time)")
            code = EXIT_REFUSED
            return code
        pub = node.create_publisher(Twist, cmd_topic, 10)
        spin_for(0.5)
        o = state["odom"]
        state["home"] = (o[2], o[3], o[4])
        log.info(f"start pose {state['home']}, {len(schedule)} segments, limit {args.limit} m")
        period = 1.0 / args.rate
        last_check = time.monotonic()
        for seg in schedule:
            state["seg"] = seg
            t_end = ros_now() + seg.duration
            next_tick = ros_now()
            while True:
                rclpy.spin_once(node, timeout_sec=0.002)
                now_mono, now = time.monotonic(), ros_now()
                o = state["odom"]
                if over_limit(state["home"], o[2:], args.limit):
                    raise Aborted(f"odom moved {displacement(state['home'], o[2:]):.3f} m"
                                  f" > limit {args.limit} m")
                if odom_stale(state["mono"], now_mono, args.odom_timeout):
                    raise Aborted(f"no odom for {args.odom_timeout} s")
                if now_mono - last_check > 1.0:
                    last_check = now_mono
                    found = others()
                    if found:
                        raise Aborted(f"another publisher appeared on {cmd_topic}: {found}")
                if seg.phase == "home":
                    v, w, done = home_command(o[2:], state["home"])
                    if done:
                        break
                    if now >= t_end:
                        raise Aborted(f"home move did not finish in {seg.duration} s")
                else:
                    if now >= t_end:
                        break
                    v, w = seg.v, seg.w
                if now >= next_tick:
                    publish(v, w)
                    next_tick = max(next_tick + period, now - period)
        state["seg"] = None
        log.info(f"done, wrote {out}")
    except Aborted as e:
        log.error(f"ABORT: {e}")
        code = EXIT_ABORTED
    except KeyboardInterrupt:
        log.warn("interrupted")
        code = EXIT_INTERRUPTED
    finally:
        if pub is not None:
            send_zeros(publish)
        fh.close()
        Path(str(out) + ".json").write_text(json.dumps({
            "args": vars(args), "exit": code, "segments": [asdict(s) for s in schedule]},
            indent=1), encoding="utf-8")
        node.destroy_node()
        rclpy.try_shutdown()
    return code


def main(argv):
    args = parse_args(argv)
    schedule = schedule_of(args)
    text, worst = describe(schedule, args.limit)
    if args.dry_run:
        print(text)
        return EXIT_DONE
    if worst > args.limit:
        print(text)
        print(f"refusing: a step reaches {worst:.3f} m > --limit {args.limit} m")
        return EXIT_REFUSED

    def on_term(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    return run(args, schedule)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
