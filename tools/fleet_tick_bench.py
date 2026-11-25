#!/usr/bin/env python3
"""Fleet manager tick benchmark with N fake Podes in the headless fake world (no Gazebo).

Measures the wall time of every FleetManagerNode._on_tick call while fake robots drive and
orders keep coming, for each robot count, and prints one Markdown row per count.

  source /opt/ros/jazzy/setup.bash; source <ws>/install/setup.bash
  W=$(pwd); export PYTHONPATH=$W/pc/fleet_core:$W/pc/fleet_manager:$W/pode/pode_fsm:$W/sim/fake_robots:$PYTHONPATH
  export ROS_DOMAIN_ID=70 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  python3 tools/fleet_tick_bench.py --robots 3 10 20 --out /tmp/bench.json

How a case runs (each count in its own child process of this tool, one after another):
  1. FleetManagerNode._on_tick is wrapped with a timer before the node is built, then the node
     is built in this process with robots pode1..podeN.
  2. sim/fake_robots/launch/fakes.launch.py of this tree starts N fake Podes, the fake workcell,
     the fake ship arm and the fake central (wall clock, no monitor) in a new process group.
  3. Spinning follows the node module: make_executor() if it exists, otherwise rclpy.spin.
  4. After every robot has reported (cap --ready-timeout), --warmup seconds pass, then ticks
     are kept for --seconds. Then the process group gets SIGINT, SIGTERM, SIGKILL in turn.

Maps: 3 or fewer robots use the d2 layout as is. Every shipped layout has only three parking
nodes (P-1..P-3) and a 2 x 1 m yard, so more robots use a generated copy of d2 whose yard is
stretched south with parking rows P-4..P-N (HALL capacity N). The map and scenario files are
written to a temp dir; repo files are not touched.

Queue: if the node has `_inbox` (IR-30), its `peak` is reported and `qsize()` is sampled each tick.
"""
from __future__ import annotations

import argparse
import inspect
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
FAKES_LAUNCH = REPO / "sim/fake_robots/launch/fakes.launch.py"
CONTRACT = REPO / "interfaces/fleet_interfaces/contract.yaml"
BASE_LAYOUT = "d2"
PITCH_M = 0.30  # 추가 주차 칸 간격
COLS = 6  # 추가 주차 칸 한 줄 수
MOVED_M = 0.10  # 창 안에서 이만큼 넘게 벗어나면 움직인 로봇(자세 잡음 0.03 m보다 크게)
ITEMS = [{"A": 1, "C": 1}, {"B": 1}, {"D": 2}, {"A": 1, "B": 1}, {"C": 1}, {"B": 1, "D": 1}]


# ---- 지도와 시나리오 만들기 ----
def bench_map(count: int, out_dir: Path) -> str:
    """count대가 설 주차 칸이 있는 지도. 3대 이하는 d2 그대로."""
    from fleet_core.fleet_map import fleet_map_from_text
    from fleet_core.fleet_map.fleet_map import DATA
    if count <= 3:
        return BASE_LAYOUT
    d = yaml.safe_load((DATA / f"fleet_map_{BASE_LAYOUT}.yaml").read_text(encoding="utf-8"))
    rows = -(-(count - 3) // COLS)
    depth = rows * PITCH_M

    def up(y):
        return round(y + depth, 4)

    def rect(r):
        return [r[0], up(r[1]), r[2], up(r[3])]

    d["bounds"] = [d["bounds"][0], 0.0, d["bounds"][2], up(d["bounds"][3])]
    d["shelves"] = [rect(s) for s in d["shelves"]]
    for zone in d["zones"].values():
        zone["rects"] = [rect(r) for r in zone["rects"]]
    hall = d["zones"]["HALL"]
    hall["rects"][0][1] = 0.0  # 마당을 남쪽 벽까지 늘린다
    hall["capacity"] = max(int(hall.get("capacity", 1)), count)
    for node in d["nodes"].values():
        node["y"] = up(node["y"])
    for path in d["paths"].values():
        path["points"] = [[p[0], up(p[1])] if isinstance(p, list) else p for p in path["points"]]
    for k in range(4, count + 1):
        i = k - 4
        name = f"P-{k}"
        d["nodes"][name] = {"x": round(0.15 + (i % COLS) * PITCH_M, 3),
                            "y": round(0.15 + (i // COLS) * PITCH_M, 3),
                            "kind": "parking", "yaw_deg": 90}
        d["nav"][0]["nodes"].append(name)
        d["parking"].append(name)
    d["name"] = f"bench_{BASE_LAYOUT}_{count}"
    text = yaml.safe_dump(d, sort_keys=False)
    fleet_map_from_text(text)  # 지도 검사에 걸리면 여기서 멈춘다
    path = out_dir / f"fleet_map_bench_{count}.yaml"
    path.write_text(text, encoding="utf-8")
    return str(path)


def bench_scenario(count: int, order_every_s: float, span_s: float, out_dir: Path) -> str:
    n = int(span_s / order_every_s) + 1
    data = {
        "seed": 1,
        "speed_factor": 1.0,
        "timeout_s": span_s + 600,
        "stock": {"arm_left": {"A": 9999, "B": 9999}, "arm_right": {"C": 9999, "D": 9999}},
        "orders": [{"at_s": round(i * order_every_s, 3), "id": f"B-{i + 1:04d}",
                    "items": ITEMS[i % len(ITEMS)]} for i in range(n)],
        "robots": {f"pode{k}": {"start": f"P-{k}", "speed_scale": 1.0, "pose_noise_m": 0.03}
                   for k in range(1, count + 1)},
        "arm": {"seconds_per_item": 4.0, "swap_seconds": 8.0, "fail_rate": 0.0},
        "dock": {"duration_s": 3.0, "error_cm": 1.0},
        "faults": [],
    }
    path = out_dir / f"scenario_bench_{count}.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return str(path)


# ---- 틱 시간 재기 ----
SAMPLES: list[tuple[float, float, int | None]] = []  # (시작 monotonic, 걸린 초, 큐 길이)


def patch_tick(cls) -> None:
    """타이머가 만들 때 메서드를 잡으므로 노드를 만들기 전에 클래스 메서드를 감싼다."""
    original = cls._on_tick

    def timed(self, *args, **kwargs):
        inbox = getattr(self, "_inbox", None)
        q = inbox.qsize() if inbox is not None and hasattr(inbox, "qsize") else None
        t0 = time.perf_counter()
        try:
            return original(self, *args, **kwargs)
        finally:
            SAMPLES.append((t0, time.perf_counter() - t0, q))

    cls._on_tick = timed


def spin_mode(module):
    """main()과 같은 방식으로 돌린다: make_executor가 있으면 그것, 없으면 rclpy.spin."""
    import rclpy
    make = getattr(module, "make_executor", None)
    if callable(make):
        def run(node):
            params = inspect.signature(make).parameters
            executor = make(node) if params else make()
            if node not in executor.get_nodes():
                executor.add_node(node)
            executor.spin()
        return "make_executor", run
    return "rclpy.spin", rclpy.spin


def percentile(values, q):
    if not values:
        return math.nan
    s = sorted(values)
    return s[min(len(s) - 1, max(0, math.ceil(q / 100.0 * len(s)) - 1))]


def stop_group(proc) -> str:
    """내가 띄운 런치만 끈다: 런치에 SIGINT(자식에게 넘긴다), 남으면 묶음에 SIGTERM, SIGKILL."""
    steps = ((lambda s: os.kill(proc.pid, s), signal.SIGINT, 20.0),
             (lambda s: os.killpg(proc.pid, s), signal.SIGTERM, 5.0),
             (lambda s: os.killpg(proc.pid, s), signal.SIGKILL, 5.0))
    for send, sig, wait_s in steps:
        try:
            send(sig)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            proc.poll()
            try:
                os.killpg(proc.pid, 0)  # 묶음에 아직 누가 있는지
            except ProcessLookupError:
                return signal.Signals(sig).name
            time.sleep(0.2)
    return "left over"


# ---- 한 경우 돌리기(자식 프로세스) ----
def run_case(args) -> dict:
    import rclpy
    from fleet_manager import fleet_manager_node as fmn

    count = args.case
    tmp = Path(tempfile.mkdtemp(prefix=f"fleet_tick_bench_{count}_"))
    span = args.ready_timeout + args.warmup + args.seconds + 30.0
    map_file = bench_map(count, tmp)
    scenario = bench_scenario(count, args.order_every, span, tmp)
    robots = [f"pode{k}" for k in range(1, count + 1)]

    patch_tick(fmn.FleetManagerNode)
    ros_args = ["--ros-args", "-p", f"robots:=[{','.join(robots)}]", "-p", f"map_file:={map_file}",
                "-p", f"contract_file:={CONTRACT}", "-p", "use_sim_time:=false"]
    rclpy.init(args=["fleet_tick_bench", *ros_args])
    node = fmn.FleetManagerNode()
    tick_hz = float(node.get_parameter("tick_hz").value)
    mode, spin = spin_mode(fmn)

    log = open(tmp / "fakes.log", "w")
    cmd = ["ros2", "launch", str(FAKES_LAUNCH), f"scenario:={scenario}",
           f"fake_robots:={' '.join(robots)}", "use_sim_clock:=false", "monitor:=false",
           f"map_file:={map_file}", f"contract_file:={CONTRACT}"]
    load_start = os.getloadavg()
    fakes = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    marks: dict[str, float] = {}
    moved = {r: 0.0 for r in robots}  # 창 시작 자리에서 가장 멀리 간 거리

    def watch():
        t0 = time.monotonic()
        marks["spin"] = time.perf_counter()
        def states():  # 구조가 바뀌어 dict를 새로 만들어도 따라가게 매번 읽는다
            return getattr(node, "_pode_states", None)

        while time.monotonic() - t0 < args.ready_timeout:
            if states() is not None and all(r in states() for r in robots):
                break
            if states() is None and time.monotonic() - t0 >= 5.0:
                break
            time.sleep(0.2)
        marks["ready_s"] = time.monotonic() - t0
        marks["all_reported"] = states() is not None and all(r in states() for r in robots)
        time.sleep(args.warmup)
        marks["start"] = time.perf_counter()
        first = {}
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            for r in robots:
                msg = (states() or {}).get(r)
                if msg is None:
                    continue
                p = (msg.pose.x, msg.pose.y)
                first.setdefault(r, p)
                moved[r] = max(moved[r], math.dist(p, first[r]))
            time.sleep(1.0)
        marks["end"] = time.perf_counter()
        marks["load_end"] = os.getloadavg()
        rclpy.try_shutdown()

    threading.Thread(target=watch, daemon=True).start()
    try:
        spin(node)
    except Exception:
        if rclpy.ok():
            raise
    finally:  # 예외나 Ctrl-C에도 가짜 로봇을 남기지 않는다
        stopped = stop_group(fakes)
        log.close()
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.try_shutdown()

    inbox = getattr(node, "_inbox", None)
    kept = [(t, d, q) for t, d, q in SAMPLES if marks["start"] <= t < marks["end"]]
    ms = [d * 1000.0 for _, d, _ in kept]
    qs = [q for _, _, q in kept if q is not None]
    period_ms = 1000.0 / tick_hz
    return {
        "robots": count,
        "spin_mode": mode,
        "map": map_file,
        "seconds": args.seconds,
        "warmup_s": args.warmup,
        "ready_s": round(marks.get("ready_s", math.nan), 2),
        "all_reported": marks.get("all_reported"),
        "tick_hz": tick_hz,
        "ticks": len(ms),
        "expected_ticks": int(args.seconds * tick_hz),
        "p50_ms": round(statistics.median(ms), 3) if ms else None,
        "p99_ms": round(percentile(ms, 99), 3) if ms else None,
        "max_ms": round(max(ms), 3) if ms else None,
        "mean_ms": round(statistics.fmean(ms), 3) if ms else None,
        "over_period": sum(1 for v in ms if v > period_ms),
        "queue_peak": getattr(inbox, "peak", None) if inbox is not None else "n/a",
        "queue_tick_max": max(qs) if qs else "n/a",
        "robots_moved": sum(1 for v in moved.values() if v > MOVED_M),
        "max_away_m": {r: round(v, 2) for r, v in moved.items()},
        "load_start": [round(v, 2) for v in load_start],
        "load_end": [round(v, 2) for v in marks.get("load_end", (math.nan,) * 3)],
        "cpu_count": os.cpu_count(),
        "fakes_stopped_by": stopped,
        "order_every_s": args.order_every,
        "tmp_dir": str(tmp),
        "node_file": fmn.__file__,
    }


# ---- 결과 표 ----
HEADER = ("| robots | spin | ticks | p50 ms | p99 ms | max ms | over period | queue peak "
          "| moved | load start | load end |\n"
          "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")


def row(r: dict) -> str:
    return (f"| {r['robots']} | {r['spin_mode']} | {r['ticks']} | {r['p50_ms']} | {r['p99_ms']} "
            f"| {r['max_ms']} | {r['over_period']} | {r['queue_peak']} "
            f"| {r['robots_moved']}/{r['robots']} | {r['load_start'][0]} | {r['load_end'][0]} |")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--robots", type=int, nargs="+", default=[3, 10, 20],
                   help="robot counts, run one after another (default: 3 10 20)")
    p.add_argument("--seconds", type=float, default=90.0, help="measured time per case")
    p.add_argument("--warmup", type=float, default=15.0,
                   help="seconds after every robot has reported, not measured")
    p.add_argument("--ready-timeout", type=float, default=60.0,
                   help="longest wait for every robot to report before the warm-up")
    p.add_argument("--order-every", type=float, default=3.0, help="seconds between new orders")
    p.add_argument("--out", required=True, help="JSON output path")
    p.add_argument("--case", type=int, default=None, help=argparse.SUPPRESS)
    p.add_argument("--case-out", default=None, help=argparse.SUPPRESS)
    args = p.parse_args()

    if args.case is not None:
        result = run_case(args)
        Path(args.case_out).write_text(json.dumps(result, indent=2), encoding="utf-8")
        return 0

    results = []
    for count in args.robots:
        case_out = Path(tempfile.mkstemp(prefix=f"fleet_tick_case_{count}_", suffix=".json")[1])
        cmd = [sys.executable, __file__, "--case", str(count), "--case-out", str(case_out),
               "--seconds", str(args.seconds), "--warmup", str(args.warmup),
               "--ready-timeout", str(args.ready_timeout), "--order-every", str(args.order_every),
               "--out", args.out]
        print(f"[bench] {count} robots, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}",
              flush=True)
        # 노드 로그는 터미널 대신 파일로(터미널 출력 속도가 틱 시간에 섞이지 않게)
        case_log = case_out.with_suffix(".log")
        with open(case_log, "w") as log:
            rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode
        print(f"[bench] node log {case_log}", flush=True)
        text = case_out.read_text(encoding="utf-8") if case_out.exists() else ""
        if rc != 0 or not text:
            print(f"[bench] {count} robots failed (exit {rc})", flush=True)
            results.append({"robots": count, "error": f"exit {rc}"})
            continue
        r = json.loads(text)
        results.append(r)
        print(f"[bench] {count} robots: spin {r['spin_mode']}, ready {r['ready_s']} s, "
              f"fakes stopped by {r['fakes_stopped_by']}", flush=True)
        if not r.get("all_reported"):  # 일부 로봇만 뜬 판은 비교에 쓰지 않는다
            print(f"[bench] WARNING {count} robots: not every robot reported before measuring",
                  flush=True)

    out = {"cpu_count": os.cpu_count(), "python": sys.version.split()[0],
           "argv": sys.argv, "cases": results}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(HEADER)
    for r in results:
        print(row(r) if "error" not in r else f"| {r['robots']} | failed: {r['error']} |")
    return 0 if all("error" not in r for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
