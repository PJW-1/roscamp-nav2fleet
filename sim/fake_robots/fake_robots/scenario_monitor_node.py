"""ScenarioMonitor: 토픽 데이터를 심판에 넘기고 결과 파일을 쓰며 0 또는 1로 끝난다.

운영자 역할도 한다: 치워진 로봇의 구역은 confirm_clear로, 다시 놓인 로봇은 reinstate로 부른다.
판단은 operator_clear.py에 있다.
"""
from __future__ import annotations

import csv
import json
import sys
import uuid
from pathlib import Path

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.msg import (
    ArmState,
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    RobotState,
    RobotStatusArray,
    ZoneStateArray,
)
from fleet_interfaces.srv import ConfirmClear, Reinstate, RobotCommand
from rclpy.node import Node

from fake_robots.common import (
    TRUTH_TOPIC,
    ScenarioClock,
    load_contract_param,
    load_params_param,
    load_scenario_param,
    load_site_param,
)
from fake_robots.judge import ScenarioJudge
from fake_robots.motion import FaultSchedule
from fake_robots.operator_clear import OperatorClear
from fake_robots.safety_meter import SafetyMeter


def _write_csv(path: Path, header: list[str], rows) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


class ScenarioMonitorNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("scenario_monitor", parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("result_dir", "sim_result")
        self.declare_parameter("margin_m", -1.0)
        self.declare_parameter("grace_s", 2.0)
        contract = load_contract_param(self)
        site = load_site_param(self)
        scenario = load_scenario_param(self, site)
        params = load_params_param(self)
        radius = float(params["robot_radius_m"])
        margin = float(self.get_parameter("margin_m").value)
        self.judge = ScenarioJudge(
            site, scenario,
            margin_m=margin if margin >= 0 else radius + float(params["release_margin_extra_m"]),
            grace_s=float(self.get_parameter("grace_s").value),
            radius_m=radius)
        self.safety = SafetyMeter(site, radius,
                                  on_course=lambda robot: not self.judge.off_course(robot))
        self._sc = ScenarioClock(self)
        self._result_dir = Path(self.get_parameter("result_dir").value)
        self._seen: dict[str, str] = {}
        self._status_rows: list[list] = []
        self.exit_code: int | None = None
        self._sc_robots = list(scenario.robots)
        # 치우기는 시각이나 사건(after_event)으로 정해진다. 로봇 노드와 같은 일정으로 푼다
        self._removals = [f for f in scenario.faults if f.type == "operator_remove"]
        self._removal_times = {f.robot: FaultSchedule(scenario.faults, f.robot)
                               for f in self._removals}
        self._confirm = self.create_client(ConfirmClear, contract.name("confirm_clear"))
        self._reinstates = sorted(((f.at_s, f.robot, f.start, f.basket) for f in scenario.faults
                                   if f.type == "operator_reinstate"), key=lambda r: r[0])
        self._reinstate = self.create_client(Reinstate, contract.name("reinstate"))
        # [장애 처리 완료]: 시각이나 그 팔의 사건(after_event) 뒤 at_s초에 한 번 부른다
        self._resolves = [f for f in scenario.faults if f.type == "operator_fault_resolved"]
        self._resolve_at: dict[int, float] = {id(f): f.at_s for f in self._resolves
                                              if not f.after_event}
        self._fault_resolved = self.create_client(RobotCommand, contract.name("fault_resolved"))
        self._operator = OperatorClear()

        def sub(msg_type, key, callback, **names):
            self.create_subscription(
                msg_type, contract.name(key, **names), callback, to_ros_qos(contract.qos(key)))

        for robot in scenario.robots:
            sub(RobotState, "robot_state", self._on_robot, pode=robot)
            self.create_subscription(
                RobotState, TRUTH_TOPIC.format(robot=robot), self._on_truth,
                to_ros_qos(contract.qos("robot_state")))
        sub(ZoneStateArray, "zones", self._on_zones)
        # 작업대 팔 상태는 토픽에서 바로 본다(상태판은 배차보다 늦게 바뀔 수 있음)
        for arm in sorted(self.judge.fill_arms):
            sub(ArmState, "arm_state",
                lambda msg, a=arm: self.judge.on_arm_state(a, msg.state, self._sc.now()), arm=arm)
        sub(FleetEvent, "events", self._on_event)
        sub(AssignmentArray, "assignments", self._on_assignments)
        sub(RobotStatusArray, "robots", self._on_robots)
        sub(OrderBoard, "order_board", self._on_board)
        self.create_timer(0.1, self._tick)

    def _changed(self, key: str, value: str) -> bool:
        if self._seen.get(key) == value:
            return False
        self._seen[key] = value
        return True

    def _on_robot(self, msg: RobotState) -> None:
        self.judge.on_robot_state(self._sc.now(), msg.robot_id, msg.pose.x, msg.pose.y,
                                  msg.state)

    def _on_truth(self, msg: RobotState) -> None:
        self.judge.on_true_pose(msg.robot_id, msg.pose.x, msg.pose.y)
        self.safety.on_pose(msg.robot_id, msg.pose.x, msg.pose.y)

    def _on_zones(self, msg: ZoneStateArray) -> None:
        holders = {z.zone_id: z.holder for z in msg.zones}
        self.judge.on_zones(self._sc.now(), holders)
        self.safety.on_zones(self._sc.now(), holders)

    def _on_event(self, msg: FleetEvent) -> None:
        self.judge.on_event(self._sc.now(), msg.level, msg.source, msg.code, msg.detail)
        self._operator.on_event(msg.source, msg.code, msg.detail)
        for schedule in self._removal_times.values():
            schedule.on_event(self._sc.now(), msg.source, msg.code, msg.detail)
        for f in self._resolves:
            if (f.after_event and id(f) not in self._resolve_at and msg.code == f.after_event
                    and msg.source == f.arm and f.after_detail in msg.detail):
                self._resolve_at[id(f)] = self._sc.now() + f.at_s

    def _on_assignments(self, msg: AssignmentArray) -> None:
        self.judge.on_assignments({a.robot_id: a.task_type for a in msg.assignments},
                                  self._sc.now())
        text = ";".join(f"{a.robot_id}:{a.task_type}:{a.station_id}:{a.order_id}"
                        for a in msg.assignments)
        if self._changed("assignments", text):
            self.judge.note(self._sc.now(), "assignments", "ASSIGNMENTS", text)

    def _on_robots(self, msg: RobotStatusArray) -> None:
        """로봇 상태가 바뀔 때마다 한 행. 늘 바뀌는 값은 뺀다."""
        for r in msg.robots:
            row = [r.robot_id, r.fsm_state, r.task_type, r.order_id, r.fault, r.fault_reason,
                   r.wait_zone, " ".join(r.wait_robots)]
            if self._changed(f"status:{r.robot_id}", "|".join(row)):
                self._status_rows.append([round(self._sc.now(), 3), *row])

    def _on_board(self, msg: OrderBoard) -> None:
        self.judge.on_board([o.order_id for o in msg.orders])
        if self._changed("run_mode", msg.run_mode):
            self.judge.note(self._sc.now(), "order_board", "RUN_MODE", msg.run_mode)

    def _clear_removed_robots(self, t: float) -> None:
        """로봇이 실제로 코스에서 빠지고 호출에 답할 쪽이 생길 때까지 기다린다.

        남은 구역이 있으면 같은 NOTE를 한 줄 더 남기고 다시 부른다(상한 뒤 멈춤).
        """
        def start(f) -> float:
            return self._removal_times[f.robot].start_of(f)

        self._removals.sort(key=start)
        while self._removals and start(self._removals[0]) <= t:
            robot = self._removals[0].robot
            if not self.judge.off_course(robot) or not self._confirm.service_is_ready():
                break
            self._removals.pop(0)
            self._operator.start(robot, t)
        if not self._confirm.service_is_ready():
            return
        for robot in sorted(self._sc_robots):
            zones = self._operator.next_clear(robot, self.judge.held_by(robot), t)
            if self._operator.gave_up(robot):
                self.judge.note(t, robot, "OPERATOR_GAVE_UP",
                                f"zones left after {self._operator.max_clears} clears")
            if zones is None:
                continue
            self.judge.note(t, robot, "OPERATOR_REMOVE", " ".join(zones))
            future = self._confirm.call_async(ConfirmClear.Request(
                robot_id=robot, zone_ids=zones, note="operator_remove"))
            future.add_done_callback(lambda f, r=robot: self._on_cleared(r, f))

    def _on_cleared(self, robot: str, future) -> None:
        response = future.result()
        self._operator.on_cleared(robot, response.accepted, list(response.released_zones),
                                  self._sc.now())
        self.judge.note(self._sc.now(), robot, "CONFIRM_CLEAR",
                        f"accepted={response.accepted} {' '.join(response.released_zones)}")

    def _resolve_arm_faults(self, t: float) -> None:
        """운영자의 [장애 처리 완료]: 때가 되면 /fleet/fault_resolved를 그 팔로 한 번 부른다."""
        if not self._resolves or not self._fault_resolved.service_is_ready():
            return
        waiting = []
        for f in self._resolves:
            at = self._resolve_at.get(id(f))
            if at is None or at > t:
                waiting.append(f)
                continue
            self.judge.on_fault_resolved(t, f.arm)
            self.judge.note(t, f.arm, "OPERATOR_FAULT_RESOLVED", "")
            future = self._fault_resolved.call_async(RobotCommand.Request(
                request_id=str(uuid.uuid4()), robot_id=f.arm))
            future.add_done_callback(lambda fu, a=f.arm: self._on_fault_resolved(a, fu))
        self._resolves = waiting

    def _on_fault_resolved(self, arm: str, future) -> None:
        response = future.result()
        self.judge.note(self._sc.now(), arm, "FAULT_RESOLVED_ANSWER",
                        f"accepted={response.accepted} {response.reason}".strip())

    def _reinstate_put_back_robots(self, t: float) -> None:
        """앞선 치우기가 받아들여지고 로봇이 코스에 다시 보일 때 부른다.

        때가 된 항목을 모두 본다. 준비 안 된 항목은 남기고 다른 로봇 것은 처리한다.
        """
        waiting = []
        for item in self._reinstates:
            at_s, robot, node, basket = item
            if (at_s > t or not self._operator.ready(robot, self.judge.held_by(robot), t)
                    or self.judge.off_course(robot)
                    or not self._reinstate.service_is_ready()):
                waiting.append(item)
                continue
            self._operator.reinstated(robot)
            self.judge.put_back(t, robot, node)
            self.judge.note(t, robot, "OPERATOR_REINSTATE", f"basket={basket}")
            future = self._reinstate.call_async(Reinstate.Request(
                robot_id=robot, basket=basket, note="operator_reinstate"))
            future.add_done_callback(lambda f, r=robot: self._on_reinstated(r, f))
        self._reinstates = waiting

    def _on_reinstated(self, robot: str, future) -> None:
        response = future.result()
        self.judge.note(self._sc.now(), robot, "REINSTATE",
                        f"accepted={response.accepted} {response.reason}")

    def _tick(self) -> None:
        if self.exit_code is not None:
            return
        t = self._sc.now()
        self.judge.tick(t)
        self.safety.tick(t)
        self._clear_removed_robots(t)
        self._reinstate_put_back_robots(t)
        self._resolve_arm_faults(t)
        if self.judge.finished(t):
            self._finish(t)

    def _finish(self, t: float) -> None:
        summary = self.judge.summary(t)
        out = self._result_dir
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        (out / "safety.json").write_text(json.dumps(self.safety.summary(), indent=2),
                                         encoding="utf-8")
        _write_csv(out / "events.csv", ["time_s", "level", "source", "code", "detail"],
                   self.judge.event_rows)
        _write_csv(out / "zones.csv", ["time_s", "zone", "old_holder", "new_holder"],
                   self.judge.zone_rows)
        _write_csv(out / "robots.csv", ["time_s", "robot", "x", "y", "state"],
                   self.judge.robot_rows)
        _write_csv(out / "releases.csv", ["time_s", "zone", "robot", "true_m", "reported_m"],
                   self.judge.release_rows)
        _write_csv(out / "fleet_robots.csv",
                   ["time_s", "robot", "fsm_state", "task_type", "order_id", "fault",
                    "fault_reason", "wait_zone", "wait_robots"], self._status_rows)
        verdict = "PASS" if summary["pass"] else "FAIL"
        self.get_logger().info(
            f"scenario {verdict}: shipped {summary['orders_shipped']}, partial "
            f"{summary['orders_partial']}, held {summary['orders_held']} of "
            f"{summary['orders_total']} in {t:.0f} s, results in {out}")
        for line in summary["violations"]:
            self.get_logger().warn(line)
        self.exit_code = 0 if summary["pass"] else 1


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioMonitorNode()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    code = 1 if node.exit_code is None else node.exit_code
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
