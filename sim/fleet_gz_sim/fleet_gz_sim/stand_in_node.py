"""Gazebo 3대 시뮬 전용 노드 둘. 실물에서는 쓰지 않는다.
sim_stand_in: 로봇마다 실제 위치 중계(/sim/truth/{pode}), 시뮬 배터리(/{pode}/sim/battery), 선택적으로 도크 대역과 peer_obstacles 노드를 둔다.
wait_ready: 모든 로봇의 /{pode}/state가 IDLE로 1 s 이어지면 0으로 끝난다(ready.py). timeout_s(벽시계) 안에 안 되면 1.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from fake_robots.battery import Battery
from fake_robots.common import (
    TRUTH_TOPIC,
    load_contract_param,
    load_site_param,
)
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock
from fleet_interfaces.msg import RobotState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from fleet_gz_sim.dock_stand_in import (
    ClockWatch,
    dock_done,
    dock_outcome,
    dock_phase,
    require_dock_stub_allowed,
    stub_may_answer,
)
from fleet_gz_sim.fleet_setup import stand_in_inputs
from fleet_gz_sim.launch_parts import BATTERY_TOPIC
from fleet_gz_sim.quaternion import yaw_of
from fleet_gz_sim.ready import everyone_idle

READY_HOLD_S = 1.0
REPORT_FRESH_S = 0.5  # pode_fsm은 state를 5 Hz로 낸다
CLOCK_STALL_S = 5.0  # Gazebo 시각이 이만큼(wall 초) 안 움직이면 도크 실패
BATTERY_PERIOD_S = 0.5  # 배터리를 내는 간격(Gazebo 시간)
STAND_STILL_M = 0.01  # 한 간격 동안 이보다 덜 움직였으면 서 있다


class SimStandIn(Node):
    def __init__(self, **kwargs):
        super().__init__("sim_stand_in", **kwargs)
        self.declare_parameter("robots", ["pode1", "pode2", "pode3"])
        stub = bool(self.declare_parameter("dock_stub", False).value)
        self.declare_parameter("peer_obstacles", False)  # main이 읽는다(peer_nodes)
        self.declare_parameter("plan_keep_nodes", False)  # main()이 읽는다
        if stub:  # 무엇보다 먼저: 격리된 Gazebo 실행 밖에서는 도크를 받지 않는다
            require_dock_stub_allowed(bool(self.get_parameter("use_sim_time").value),
                                      os.environ, rmw=rclpy.get_rmw_implementation_identifier(),
                                      workdir=Path.cwd())
            self.add_on_set_parameters_callback(self._keep_sim_time)
        contract = load_contract_param(self)
        self._site = load_site_param(self)
        path = self.declare_parameter("scenario_file", "").value
        seed = int(self.declare_parameter("scenario_seed", -1).value)
        if stub and not path:
            raise ValueError("parameter scenario_file is required with dock_stub")
        scenario = stand_in_inputs(path, None if seed < 0 else seed) if path else None
        self._duration = scenario.dock_s if stub else 0.0
        self._lock = threading.Lock()
        self._pose: dict[str, tuple[float, float, float]] = {}
        self._batteries: dict[str, tuple[Battery, object]] = {}  # 로봇: (모형, 발행자)
        self._battery_at: dict[str, tuple[float, float]] = {}  # 지난 배터리 계산 때 자리
        self._turn: dict[str, int] = {}  # 로봇마다 마지막으로 받은 도크 목표 번호
        qos = to_ros_qos(contract.qos("robot_state"))
        group = ReentrantCallbackGroup()
        self.dock_servers = []
        for robot in self.get_parameter("robots").value:
            pub = self.create_publisher(RobotState, TRUTH_TOPIC.format(robot=robot), qos)
            self.create_subscription(Odometry, f"/{robot}/sim/truth",
                                     lambda msg, r=robot, p=pub: self._on_truth(r, p, msg), 10)
            if scenario is not None:
                spots = [self._site.node_pose(w) for w in self._site.parking]
                mine = [f for f in scenario.faults if f.get("robot") == robot]
                self._batteries[robot] = (Battery(mine, spots), self.create_publisher(
                    BatteryState, BATTERY_TOPIC.format(robot=robot), qos_profile_sensor_data))
            if not stub:
                continue
            self.dock_servers.append(ActionServer(
                self, Dock, contract.name("dock", pode=robot),
                execute_callback=lambda handle, r=robot: self._dock(r, handle),
                goal_callback=lambda _goal: GoalResponse.ACCEPT,
                cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                callback_group=group))
        if self._batteries:
            self.create_timer(BATTERY_PERIOD_S, self._publish_batteries)
        robots = self.get_parameter("robots").value
        what = "SIM DOCK STUB and truth" if stub else "truth (dock left to pode_fsm)"
        self.get_logger().info(f"sim stand-in: {what} for {robots}")

    @staticmethod
    def _keep_sim_time(params) -> SetParametersResult:
        for param in params:
            if param.name == "use_sim_time" and param.value is not True:
                return SetParametersResult(successful=False,
                                           reason="the dock stub runs only on Gazebo time")
        return SetParametersResult(successful=True)

    def _on_truth(self, robot: str, pub, msg: Odometry) -> None:
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        with self._lock:
            self._pose[robot] = pose
        out = RobotState(robot_id=robot, state=RobotState.STATE_IDLE, stamp=msg.header.stamp)
        out.pose.x, out.pose.y, out.pose.theta = pose
        pub.publish(out)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_batteries(self) -> None:
        now = self._now()
        if now <= 0.0:
            return  # /clock를 아직 못 받았다
        for robot, (battery, pub) in self._batteries.items():
            with self._lock:
                pose = self._pose.get(robot)
            if pose is None:
                continue  # 실제 위치를 아직 모른다
            x, y = pose[0], pose[1]
            before = self._battery_at.get(robot)
            self._battery_at[robot] = (x, y)
            standing = before is not None and math.dist(before, (x, y)) < STAND_STILL_M
            level = battery.percent(now, x, y, standing)
            msg = BatteryState(percentage=level / 100.0, present=True)
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)

    def _dock(self, robot: str, handle):
        with self._lock:
            turn = self._turn[robot] = self._turn.get(robot, 0) + 1
        if not stub_may_answer(self.get_clock().ros_time_is_active, self._now()):
            self.get_logger().warn(f"{robot}: no Gazebo time, dock fails")
            handle.abort()
            return Dock.Result(success=False)
        station = self._site.stations.get(handle.request.dock_id)
        if station is None:
            self.get_logger().warn(f"{robot}: dock {handle.request.dock_id!r} is not a station")
            handle.abort()
            return Dock.Result(success=False)
        target = self._site.node_pose(station.dock)
        start, phase = self._now(), ""
        watch = ClockWatch(CLOCK_STALL_S)
        while rclpy.ok():
            with self._lock:
                replaced = self._turn[robot] != turn
            if handle.is_cancel_requested:
                handle.canceled()
                return Dock.Result(success=False)
            if replaced:  # 새 목표가 오면 이전 목표는 실패로 끝난다(FakePode와 같다)
                handle.abort()
                return Dock.Result(success=False)
            if watch.stalled(self._now(), time.monotonic()):
                self.get_logger().warn(f"{robot}: Gazebo time stopped, dock fails")
                handle.abort()
                return Dock.Result(success=False)
            elapsed = self._now() - start
            now_phase = dock_phase(elapsed, self._duration)
            if now_phase != phase:
                phase = now_phase
                handle.publish_feedback(Dock.Feedback(phase=phase))
            if dock_done(elapsed, self._duration):
                break
            time.sleep(0.02)
        with self._lock:
            pose = self._pose.get(robot)
        if pose is None:
            self.get_logger().warn(f"{robot}: no truth pose yet, dock fails")
            handle.abort()
            return Dock.Result(success=False)
        ok, ex, ey, eyaw = dock_outcome(pose, target)
        self.get_logger().info(f"{robot}: dock {handle.request.dock_id} "
                               f"{'ok' if ok else 'FAILED'} ({ex:.1f}, {ey:.1f}) cm")
        handle.succeed() if ok else handle.abort()
        return Dock.Result(success=ok, error_x_cm=ex, error_y_cm=ey, error_yaw_deg=eyaw)


def peer_nodes(node) -> list:
    """peer_obstacles가 참이면 로봇마다의 pode_peer_obstacles 노드들, 아니면 빈 목록."""
    if not node.get_parameter("peer_obstacles").value:
        return []
    from pode_peer_obstacles.node import make_fleet
    robots = ",".join(node.get_parameter("robots").value)
    return make_fleet(robots, node.get_parameter("use_sim_time").value, node.context)


def plan_keep_nodes(node) -> list:
    """node의 plan_keep_nodes가 참이면 로봇마다의 길 유지 노드(pode_plan_keep), 아니면 빈 목록."""
    if not node.get_parameter("plan_keep_nodes").value:
        return []
    from pode_plan_keep.node import make_fleet
    return make_fleet(list(node.get_parameter("robots").value), True,
                      node.get_parameter("use_sim_time").value, node.context)


def main(args=None):
    rclpy.init(args=args)
    node = SimStandIn()
    keepers = plan_keep_nodes(node)
    peers = peer_nodes(node) + keepers
    # 길 유지 노드가 있으면: 도크 대역과 길 유지 노드의 실행 콜백은 끝날 때까지 스레드를 쥔다.
    # 로봇마다 도크 하나와 길 유지 요청 둘(pode_plan_keep MAX_RUNNING)이 쥐어도 응답과 취소
    # 콜백이 돌 스레드가 남게 한다(CPU 수로 정하는 기본값은 작을 수 있다). 없으면 main과 같다
    robots = len(node.get_parameter("robots").value)
    executor = (MultiThreadedExecutor(num_threads=robots * 3 + 4) if keepers
                else MultiThreadedExecutor())
    for each in (node, *peers):
        executor.add_node(each)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()  # 돌던 콜백이 끝난 뒤에 노드를 지운다
        for each in (node, *peers):
            each.destroy_node()
        rclpy.try_shutdown()


def wait_ready(args=None):
    rclpy.init(args=args)
    node = Node("wait_ready")
    node.declare_parameter("robots", ["pode1", "pode2", "pode3"])
    node.declare_parameter("timeout_s", 180.0)
    contract = load_contract_param(node)
    robots = list(node.get_parameter("robots").value)
    timeout = float(node.get_parameter("timeout_s").value)
    last: dict[str, tuple[str, float]] = {}  # 로봇마다 (상태, 받은 벽시계 시각)
    for robot in robots:
        node.create_subscription(
            RobotState, contract.name("robot_state", pode=robot),
            lambda msg, r=robot: last.__setitem__(r, (msg.state, time.monotonic())),
            to_ros_qos(contract.qos("robot_state")))
    deadline, since, code = time.monotonic() + timeout, None, 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            ready = everyone_idle(last, robots, time.monotonic(), REPORT_FRESH_S)
            since = (since or time.monotonic()) if ready else None
            if since is not None and time.monotonic() - since >= READY_HOLD_S:
                node.get_logger().info(f"all robots IDLE: {robots}")
                code = 0
                break
        else:
            states = {r: last[r][0] for r in last}
            node.get_logger().error(f"robots not IDLE after {timeout:.0f} s: {states}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)
