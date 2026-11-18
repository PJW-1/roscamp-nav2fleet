"""RealDockWait: 실물 로봇마다 /<robot>/dock(Dock)을 PC에서 대신 받는 도킹 대역.

판정과 쓰는 조건은 fake_robots.dock_wait에 있다. 로봇은 DOCK_STUB=1로 떠야 한다(제 dock 서버를
/<robot>/dock_fsm으로 비킨다). 그렇지 않으면 같은 이름의 서버가 둘이 된다.
"""
from __future__ import annotations

import threading
import time

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock
from fleet_interfaces.msg import RobotState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fake_robots.common import load_contract_param, load_scenario_param, load_site_param
from fake_robots.dock_wait import dock_done, dock_outcome, dock_phase, pose_fresh

STEP_S = 0.02


class RealDockWaitNode(Node):
    def __init__(self, **kwargs):
        super().__init__("real_dock_wait", **kwargs)
        if bool(self.get_parameter("use_sim_time").value):  # 시뮬 시계로는 돌지 않는다
            raise RuntimeError("use_sim_time is true: real_dock_wait runs only on the wall clock")
        robots = str(self.declare_parameter("robots", "").value).split()
        if not robots:
            raise ValueError("parameter robots is required, e.g. 'pode1 pode2'")
        contract = load_contract_param(self)
        self._site = load_site_param(self)
        self._duration = float(load_scenario_param(self, self._site).dock.duration_s)
        self._lock = threading.Lock()
        self._pose: dict[str, tuple[float, float, float]] = {}  # x, y, 받은 시각
        self._turn: dict[str, int] = {}
        self._alive = True
        group = ReentrantCallbackGroup()
        self._servers = []
        for robot in robots:
            self.create_subscription(
                RobotState, contract.name("robot_state", pode=robot),
                lambda msg, r=robot: self._on_state(r, msg),
                to_ros_qos(contract.qos("robot_state")))
            self._servers.append(ActionServer(
                self, Dock, contract.name("dock", pode=robot),
                execute_callback=lambda handle, r=robot: self._dock(r, handle),
                goal_callback=lambda _req: GoalResponse.ACCEPT,
                cancel_callback=lambda _h: CancelResponse.ACCEPT,
                callback_group=group))
        self.get_logger().warn(
            f"REAL DOCK WAIT for {robots}: dock = wait {self._duration:.1f} s, then the robot's "
            "own pose must be near the dock. The robots must run with DOCK_STUB=1.")

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        for server in self._servers:
            server.destroy()
        return super().destroy_node()

    def _on_state(self, robot: str, msg: RobotState) -> None:
        with self._lock:
            self._pose[robot] = (float(msg.pose.x), float(msg.pose.y), time.monotonic())

    # ---- 도킹 한 번: 기다린 뒤 자세 확인 ----
    def _dock(self, robot: str, handle):
        with self._lock:
            turn = self._turn[robot] = self._turn.get(robot, 0) + 1
        station = self._site.stations.get(handle.request.dock_id)
        if station is None:
            self.get_logger().warn(f"{robot}: dock {handle.request.dock_id!r} is not a station")
            handle.abort()
            return Dock.Result(success=False)
        target = self._site.node_pose(station.dock)
        start, phase = time.monotonic(), ""
        while True:
            with self._lock:
                replaced = self._turn[robot] != turn
            if not self._alive or not rclpy.ok():
                handle.abort()
                return Dock.Result(success=False)
            if handle.is_cancel_requested:
                handle.canceled()
                return Dock.Result(success=False)
            if replaced:  # 새 목표가 오면 이전 목표는 실패로 끝난다
                handle.abort()
                return Dock.Result(success=False)
            elapsed = time.monotonic() - start
            now_phase = dock_phase(elapsed, self._duration)
            if now_phase != phase:
                phase = now_phase
                handle.publish_feedback(Dock.Feedback(phase=phase))
            if dock_done(elapsed, self._duration):
                break
            time.sleep(STEP_S)
        with self._lock:
            seen = self._pose.get(robot)
        if seen is None or not pose_fresh(seen[2], time.monotonic()):
            self.get_logger().warn(f"{robot}: no recent state, dock fails")
            handle.abort()
            return Dock.Result(success=False)
        ok, ex, ey, eyaw = dock_outcome(seen[:2], target)
        self.get_logger().info(f"{robot}: dock {handle.request.dock_id} "
                               f"{'ok' if ok else 'FAILED'} ({ex:.1f}, {ey:.1f}) cm")
        handle.succeed() if ok else handle.abort()
        return Dock.Result(success=ok, error_x_cm=ex, error_y_cm=ey, error_yaw_deg=eyaw)


def main(args=None):
    rclpy.init(args=args)
    node = RealDockWaitNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
