"""RealDockWait: 실물 로봇마다 /<robot>/dock(Dock)을 PC에서 대신 받는 도킹 대역.

판정과 쓰는 조건은 fake_robots.dock_wait에 있다. 로봇은 DOCK_STUB=1로 떠야 한다(제 dock 서버를
/<robot>/dock_fsm으로 비킨다). 그렇지 않으면 같은 이름의 서버가 둘이 된다. 그 경우를 몇 초마다
찾아 오류로 알린다(other_dock_servers).
"""
from __future__ import annotations

import threading
import time

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock
from fleet_interfaces.msg import RobotState
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fake_robots.common import load_contract_param, load_scenario_param, load_site_param
from fake_robots.dock_wait import dock_done, dock_outcome, dock_phase, pose_fresh

STEP_S = 0.02
DUPLICATE_CHECK_S = 3.0  # 같은 이름의 dock 서버를 찾는 주기
DOCK_TYPE = "fleet_interfaces/action/Dock"
THREADS = 8  # 로봇마다 도킹 하나가 스레드 하나를 쥔다. 상태 수신과 취소가 밀리지 않게 넉넉히


class RealDockWaitNode(Node):
    def __init__(self, **kwargs):
        super().__init__("real_dock_wait", **kwargs)
        if bool(self.get_parameter("use_sim_time").value):  # 시뮬 시계로는 돌지 않는다
            raise RuntimeError("use_sim_time is true: real_dock_wait runs only on the wall clock")
        self.add_on_set_parameters_callback(self._keep_wall_clock)
        robots = str(self.declare_parameter("robots", "").value).split()
        if not robots:
            raise ValueError("parameter robots is required, e.g. 'pode1 pode2'")
        contract = load_contract_param(self)
        self._site = load_site_param(self)
        self._duration = float(load_scenario_param(self, self._site).dock.duration_s)
        self._names = {robot: contract.name("dock", pode=robot) for robot in robots}
        self._lock = threading.Lock()
        self._idle = threading.Condition(self._lock)
        self._pose: dict[str, tuple[float, float, float]] = {}  # x, y, 받은 시각
        self._turn: dict[str, int] = {}  # 로봇마다 마지막으로 받은 목표의 차례
        self._goal_turn: dict[bytes, int] = {}
        self._running = 0
        self._alive = True
        self._warned: set[str] = set()
        group = ReentrantCallbackGroup()
        self._servers = []
        for robot in robots:
            self.create_subscription(
                RobotState, contract.name("robot_state", pode=robot),
                lambda msg, r=robot: self._on_state(r, msg),
                to_ros_qos(contract.qos("robot_state")))
            self._servers.append(ActionServer(
                self, Dock, self._names[robot],
                execute_callback=lambda handle, r=robot: self._dock(r, handle),
                goal_callback=lambda _req: GoalResponse.ACCEPT,
                cancel_callback=lambda _h: CancelResponse.ACCEPT,
                handle_accepted_callback=lambda handle, r=robot: self._accepted(r, handle),
                callback_group=group))
        self.create_timer(DUPLICATE_CHECK_S, self._warn_duplicates)
        self.get_logger().warn(
            f"REAL DOCK WAIT for {robots}: dock = wait {self._duration:.1f} s, then the robot's "
            "own pose must be near the dock. The robots must run with DOCK_STUB=1.")

    @staticmethod
    def _keep_wall_clock(params) -> SetParametersResult:
        for param in params:
            if param.name == "use_sim_time" and param.value is not False:
                return SetParametersResult(successful=False,
                                           reason="real_dock_wait runs only on the wall clock")
        return SetParametersResult(successful=True)

    # ---- 끝내기: 돌던 목표가 실패로 끝날 때까지 기다린 뒤 서버를 지운다 ----
    def stop(self) -> None:
        self._alive = False

    def running_goals(self) -> int:
        with self._lock:
            return self._running

    def wait_idle(self, timeout: float) -> bool:
        with self._idle:
            return self._idle.wait_for(lambda: self._running == 0, timeout)

    def destroy_node(self):
        self.stop()
        if not self.wait_idle(2.0):
            self.get_logger().warn("a dock goal did not end in 2 s")
        for server in self._servers:
            server.destroy()
        return super().destroy_node()

    # ---- 같은 이름의 dock 서버(DOCK_STUB 없이 뜬 로봇) ----
    def other_dock_servers(self) -> list[str]:
        mine = (self.get_name(), self.get_namespace())
        found = set()
        for name, ns in self.get_node_names_and_namespaces():
            if (name, ns) == mine:
                continue
            try:
                served = get_action_server_names_and_types_by_node(self, name, ns)
            except Exception:  # 그새 사라진 노드
                continue
            names = {n for n, types in served if DOCK_TYPE in types}
            found |= {robot for robot, action in self._names.items() if action in names}
        return sorted(found)

    def _warn_duplicates(self) -> None:
        if not self._alive:
            return
        found = set(self.other_dock_servers())
        for robot in sorted(found - self._warned):
            self.get_logger().error(
                f"{robot}: another node also serves {self._names[robot]}. Restart that robot "
                "with DOCK_STUB=1, or its pode_fsm answers docks too (DOCK_UNAVAILABLE)")
        self._warned = found

    def _on_state(self, robot: str, msg: RobotState) -> None:
        if msg.robot_id != robot:
            return
        with self._lock:
            if msg.state == RobotState.STATE_LOCALIZING:  # 위치를 잃으면 직전 자세도 버린다
                self._pose.pop(robot, None)
                return
            self._pose[robot] = (float(msg.pose.x), float(msg.pose.y), time.monotonic())

    # ---- 도킹 한 번: 받은 순서로 차례를 매기고, 기다린 뒤 자세 확인 ----
    def _accepted(self, robot: str, handle) -> None:
        with self._lock:
            turn = self._turn[robot] = self._turn.get(robot, 0) + 1
            self._goal_turn[bytes(handle.goal_id.uuid)] = turn
        handle.execute()

    def _end(self, robot: str, handle, turn: int) -> Dock.Result | None:
        """끝내야 하면 실패 결과, 아니면 None. 잠금 안에서 부른다."""
        if not self._alive or not rclpy.ok():
            handle.abort()
            return Dock.Result(success=False)
        if handle.is_cancel_requested:
            handle.canceled()
            return Dock.Result(success=False)
        if self._turn[robot] != turn:  # 새 목표가 오면 이전 목표는 실패로 끝난다
            handle.abort()
            return Dock.Result(success=False)
        return None

    def _dock(self, robot: str, handle):
        with self._lock:
            self._running += 1
            turn = self._goal_turn.pop(bytes(handle.goal_id.uuid))
        try:
            return self._run(robot, handle, turn)
        finally:
            with self._idle:
                self._running -= 1
                self._idle.notify_all()

    def _run(self, robot: str, handle, turn: int):
        station = self._site.stations.get(handle.request.dock_id)
        if station is None:
            self.get_logger().warn(f"{robot}: dock {handle.request.dock_id!r} is not a station")
            handle.abort()
            return Dock.Result(success=False)
        target = self._site.node_pose(station.dock)
        start, phase = time.monotonic(), ""
        while True:
            with self._lock:
                ended = self._end(robot, handle, turn)
            if ended is not None:
                return ended
            elapsed = time.monotonic() - start
            now_phase = dock_phase(elapsed, self._duration)
            if now_phase != phase:
                phase = now_phase
                handle.publish_feedback(Dock.Feedback(phase=phase))
            if dock_done(elapsed, self._duration):
                break
            time.sleep(STEP_S)
        with self._lock:  # 마지막 확인과 결과를 한 잠금 안에서
            ended = self._end(robot, handle, turn)
            if ended is not None:
                return ended
            seen = self._pose.get(robot)
            if seen is None or not pose_fresh(seen[2], time.monotonic()):
                self.get_logger().warn(f"{robot}: no recent state, dock fails")
                handle.abort()
                return Dock.Result(success=False)
            ok, ex, ey, eyaw = dock_outcome(seen[:2], target)
            handle.succeed() if ok else handle.abort()
        self.get_logger().info(f"{robot}: dock {handle.request.dock_id} "
                               f"{'ok' if ok else 'FAILED'} ({ex:.1f}, {ey:.1f}) cm")
        return Dock.Result(success=ok, error_x_cm=ex, error_y_cm=ey, error_yaw_deg=eyaw)


def main(args=None):
    rclpy.init(args=args)
    node = RealDockWaitNode()
    executor = MultiThreadedExecutor(num_threads=THREADS)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()  # 돌던 콜백이 끝난 뒤에 노드를 지운다
        node.destroy_node()
        rclpy.try_shutdown()
