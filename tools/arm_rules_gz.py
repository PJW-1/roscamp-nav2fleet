"""Gazebo나 가짜 시스템 + 가짜 팔로 팔 관제 합의 (a)~(d)를 한 판씩 확인한다. 가짜 브리지를 같이 띄워 운반 건 보고를 받는다.

    ros2 launch fleet_gz_sim gz.launch.py layout:=v7 scenario:=tools/arm_rules_gz/gz_v7_arm_b.yaml sim_dock_stub:=true
    ros2 launch iponic_bringup pc.launch.py use_fake:=true monitor:=false scenario:=tools/arm_rules_gz/fake_v7_arm_b.yaml
    python3 tools/arm_rules_gz.py --case b --out <결과 폴더>      # 같은 ROS_DOMAIN_ID

(a) 스스로 멈춤 → [복구] → 같은 목표로 이어 담고 출고
(b) 4개 중 2개 담은 뒤 재시작 → [복구] 대기 → [복구] → 남은 2개로 새 목표 → 출고
(c) 쥔 채 재시작 → HELD(ARM_FAULT)
(d) 담는 동안 ReportTrip FILLING placed가 1, 2, 3으로 오름
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from fake_robots.fake_bridge_node import FakeBridgeNode
from fleet_interfaces.msg import ArmState, FleetEvent
from fleet_interfaces.srv import ForceInput, RobotCommand, SetMode

ARMS = ("arm_left", "arm_right")
TRIPS = {"a": "R-A", "b": "R-B", "c": "R-C", "d": "R-D"}
FINAL = ("SHIPPED", "PARTIAL", "HELD", "CANCELLED")


# ---- 팔 상태 · 사건 받기, 훅 · [복구] 부르기 ----
class Driver(Node):
    def __init__(self):
        super().__init__("arm_rules_driver")
        self.lock = threading.Lock()
        self.states: dict[str, ArmState] = {}
        self.boots: dict[str, list[str]] = {a: [] for a in ARMS}
        self.totals: list[tuple[float, str, str, int]] = []  # (시각, 팔, 운반 건, placed_total)
        self.events: list[dict] = []
        for arm in ARMS:
            self.create_subscription(ArmState, f"/{arm}/state",
                                     lambda m, a=arm: self._state(a, m), qos_profile_sensor_data)
        self.create_subscription(FleetEvent, "/central/events", self._event,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.mode = {a: self.create_client(SetMode, f"/{a}/set_mode") for a in ARMS}
        self.force = {a: self.create_client(ForceInput, f"/{a}/force") for a in ARMS}
        self.resume = self.create_client(RobotCommand, "/fleet/resume_arm")

    def _state(self, arm, msg: ArmState) -> None:
        with self.lock:
            last = self.states.get(arm)
            self.states[arm] = msg
            if msg.boot_id and (not self.boots[arm] or self.boots[arm][-1] != msg.boot_id):
                self.boots[arm].append(msg.boot_id)
            if last is None or (last.order_id, last.placed_total) != (msg.order_id,
                                                                      msg.placed_total):
                self.totals.append((time.monotonic(), arm, msg.order_id, msg.placed_total))

    def _event(self, msg: FleetEvent) -> None:
        with self.lock:
            self.events.append({"t": time.monotonic(), "level": msg.level,
                                "source": msg.source, "code": msg.code, "detail": msg.detail})

    def state(self, arm) -> ArmState | None:
        with self.lock:
            return self.states.get(arm)

    def codes(self) -> list[str]:
        with self.lock:
            return [e["code"] for e in self.events]


def call(client, request, timeout=10.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"{client.srv_name} not available")
    future = client.call_async(request)
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    if not future.done():
        raise RuntimeError(f"{client.srv_name} gave no answer in {timeout} s")
    return future.result()


def wait_for(what, check, timeout, step=0.2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = check()
        if value:
            return value
        time.sleep(step)
    raise TimeoutError(f"not reached in {timeout:.0f} s: {what}")


# ---- 판마다의 진행 ----
class Case:
    def __init__(self, name, driver: Driver, bridge: FakeBridgeNode):
        self.name, self.d, self.b = name, driver, bridge
        self.trip = TRIPS[name]
        self.checks: dict[str, dict] = {}
        self.notes: list[str] = []

    def check(self, key, ok, value) -> None:
        self.checks[key] = {"ok": bool(ok), "value": value}
        print(f"[{'ok' if ok else 'FAIL'}] {key}: {value}", flush=True)

    def note(self, text) -> None:
        self.notes.append(text)
        print(text, flush=True)

    def trips(self) -> list:
        with self.b._lock:
            return [r for r in self.b.trips if r.trip_id == self.trip]

    def final(self):
        return next((r for r in self.trips() if r.status in FINAL), None)

    def hook(self, arm, key, value="") -> None:
        req = ForceInput.Request(request_id=str(uuid.uuid4()), robot_id=arm,
                                 kind=ForceInput.Request.KIND_CONDITION, key=key, value=value)
        res = call(self.d.force[arm], req)
        self.note(f"hook {arm} {key}={value!r}: accepted={res.accepted} {res.reason}")
        if not res.accepted:
            raise RuntimeError(f"hook {key} refused: {res.reason} {res.detail}")

    def test_mode(self) -> None:
        for arm in ARMS:
            res = call(self.d.mode[arm], SetMode.Request(request_id=str(uuid.uuid4()),
                                                          mode="TEST"))
            self.note(f"{arm} set_mode TEST: accepted={res.accepted} {res.reason}")
        wait_for("arms in TEST", lambda: all((self.d.state(a) or ArmState()).mode == "TEST"
                                              for a in ARMS), 10)

    def resume_arm(self, arm="arm_left"):
        res = call(self.d.resume, RobotCommand.Request(request_id=str(uuid.uuid4()),
                                                       robot_id=arm), timeout=40.0)
        self.note(f"/fleet/resume_arm {arm}: accepted={res.accepted} {res.reason} {res.detail}")
        return res

    def filling(self, arm="arm_left", total=None):
        def seen():
            s = self.d.state(arm)
            return (s is not None and s.order_id == self.trip
                    and (total is None or s.placed_total >= total))
        return wait_for(f"{arm} filling {self.trip} placed_total>={total}", seen, 600)

    def run(self) -> None:
        wait_for("both arm states", lambda: all(self.d.state(a) for a in ARMS), 180)
        getattr(self, f"case_{self.name}")()

    # (a) 스스로 멈춤 → [복구] → 같은 목표
    def case_a(self) -> None:
        self.test_mode()
        self.hook("arm_left", "self_stop", "1")
        wait_for("arm_left PAUSED", lambda: (self.d.state("arm_left").state == "PAUSED"
                                             and self.d.state("arm_left").order_id == self.trip),
                 600)
        paused_at = self.d.state("arm_left").placed_total
        boot = self.d.state("arm_left").boot_id
        time.sleep(4.0)
        self.check("stays PAUSED until [복구]", self.d.state("arm_left").state == "PAUSED",
                   self.d.state("arm_left").state)
        self.check("no final report while paused", self.final() is None,
                   [r.status for r in self.trips()])
        res = self.resume_arm("arm_left")
        self.check("resume accepted", res.accepted, f"{res.reason} {res.detail}")
        wait_for("arm_left leaves PAUSED", lambda: self.d.state("arm_left").state != "PAUSED", 40)
        final = wait_for("final report", self.final, 600)
        self.check("SHIPPED", final.status == "SHIPPED", final.status)
        self.check("same boot, no restart", self.d.state("arm_left").boot_id == boot
                   and "ARM_RESTARTED" not in self.d.codes(), boot)
        totals = [t for _, a, o, t in self.d.totals if a == "arm_left" and o == self.trip]
        self.check("same goal: placed_total never went back after the pause",
                   all(b >= a for a, b in zip(totals, totals[1:])) and paused_at >= 1, totals)
        self.check("FILL_RESUMED event", "FILL_RESUMED" in self.d.codes(), self.d.codes()[-8:])

    # (b) 2개 담은 뒤 재시작 → [복구] 대기 → [복구] → 남은 2개로 새 목표
    def case_b(self) -> None:
        self.test_mode()
        self.filling("arm_left", 2)
        boots = len(self.d.boots["arm_left"])
        self.hook("arm_left", "restart", "gap=1.0")
        wait_for("new boot_id", lambda: len(self.d.boots["arm_left"]) > boots, 60)
        wait_for("ARM_RESTARTED", lambda: "ARM_RESTARTED" in self.d.codes(), 30)
        time.sleep(5.0)
        self.check("no hold, no new goal while waiting for [복구]",
                   self.final() is None and self.d.state("arm_left").order_id == "",
                   [r.status for r in self.trips()])
        res = self.resume_arm("arm_left")
        self.check("resume accepted", res.accepted, f"{res.reason} {res.detail}")
        self.filling("arm_left", 1)
        final = wait_for("final report", self.final, 600)
        placed = {i.item_id if hasattr(i, "item_id") else i.kind: i.count for i in final.placed}
        self.check("SHIPPED with all four", final.status == "SHIPPED"
                   and sum(placed.values()) == 4, (final.status, placed))
        refill = [t for _, a, o, t in self.d.totals if a == "arm_left" and o == self.trip]
        self.check("new goal counted only the two left", max(refill[refill.index(0, 1):] or [0])
                   == 2 if 0 in refill[1:] else False, refill)

    # (c) 쥔 채 재시작 → HELD
    def case_c(self) -> None:
        self.test_mode()
        self.filling("arm_left", 1)
        self.hook("arm_left", "restart", "gap=1.0,holding=1")
        final = wait_for("final report", self.final, 120)
        self.check("HELD ARM_FAULT", (final.status, final.reason) == ("HELD", "ARM_FAULT"),
                   (final.status, final.reason))
        time.sleep(15.0)
        self.check("no new goal after the hold", self.d.state("arm_left").order_id == "",
                   self.d.state("arm_left").order_id)

    # (d) 담는 동안 ReportTrip FILLING placed가 1, 2, 3
    def case_d(self) -> None:
        final = wait_for("final report", self.final, 900)
        sums = [sum(i.count for i in r.placed) for r in self.trips() if r.status == "FILLING"]
        self.check("FILLING placed goes 1, 2, 3", [s for s in sums if s] == [1, 2, 3], sums)
        self.check("SHIPPED", final.status == "SHIPPED", final.status)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--case", choices=sorted(TRIPS), required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    rclpy.init()
    driver, bridge = Driver(), FakeBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    case = Case(args.case, driver, bridge)
    error = ""
    try:
        case.run()
    except Exception as exc:  # noqa: BLE001
        error = f"{type(exc).__name__}: {exc}"
        print(f"[FAIL] {error}", flush=True)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with bridge._lock:
        trips = [{"status": r.status, "reason": r.reason, "robot": r.robot_id,
                  "placed": {getattr(i, "item_id", "") or getattr(i, "kind", ""): i.count
                             for i in r.placed}} for r in bridge.trips]
        commands = [{"command": c.command, "target": getattr(c, "target", ""),
                     "result": c.result, "reason": c.reason} for c in bridge.commands]
    summary = {"case": args.case, "ok": not error and all(c["ok"] for c in case.checks.values()),
               "error": error, "checks": case.checks, "notes": case.notes, "trips": trips,
               "commands": commands, "events": driver.events,
               "totals": [list(t) for t in driver.totals], "boots": driver.boots}
    (out / f"arm_rules_{args.case}.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"result {'PASS' if summary['ok'] else 'FAIL'} -> {out}/arm_rules_{args.case}.json",
          flush=True)
    executor.shutdown()
    rclpy.try_shutdown()
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
