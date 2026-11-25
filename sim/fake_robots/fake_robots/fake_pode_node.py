"""FakePode: robot_state, move_to, dock을 가진 가짜 이동 로봇. 일정대로 고장을 낸다.

retreat는 보는 방향 그대로 뒤로 가고, 뒤에 물체가 있으면 서서 OBSTACLE_BEHIND로 끝낸다.
MoveTo mode가 nav_goal이면 path 없이 스스로 길을 짠다(nav_goal_drive.py).
"""
from __future__ import annotations

import random
import threading
import time
import uuid
from dataclasses import fields

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import Dock, MoveTo, Retreat
from fleet_interfaces.msg import FleetEvent, RobotState
from fleet_interfaces.srv import SetMode
from pode_fsm.admin import AdminState
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from fake_robots.ahead import seen, where_now, yields
from fake_robots.battery import Battery
from fake_robots.common import (
    TRUTH_TOPIC,
    ScenarioClock,
    load_contract_param,
    load_params_param,
    load_scenario_param,
    load_site_param,
    node_name,
)
from fake_robots.motion import DelayLine, FaultSchedule, Motion
from fake_robots.pode_goal import NAV_ABORT_DELAY_S, _Goal
from fake_robots.pode_goals import PodeGoals
from fake_robots.pode_order import RETREAT, PodeOrder
from fake_robots.pode_report import PodeReport
from fake_robots.pode_retreat import PodeRetreat
from fake_robots.recovery import ObstacleRecovery, RecoveryParams
from fake_robots.world.free_space import FreeSpace

OFF_COURSE_M = 1.0  # 운영자가 치운 로봇이 가는 곳(지도 모서리 밖)
CONTROL_RATE_HZ = 20.0


class FakePodeNode(PodeGoals, PodeOrder, PodeRetreat, PodeReport, Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__(node_name("fake", "robot_id", parameter_overrides),
                         parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("robot_id", "pode1")
        self.declare_parameter("state_rate_hz", 5.0)
        self.declare_parameter("heartbeat_timeout_s", -1.0)  # <0: 파일 값, 0: 안 멈춤
        self.robot_id = self.get_parameter("robot_id").value
        contract = load_contract_param(self)
        site = load_site_param(self)
        scenario = load_scenario_param(self, site)
        cfg = scenario.robots[self.robot_id]
        self._cfg, self._dock_cfg = cfg, scenario.dock
        self._rng = random.Random(f"{scenario.seed}-{self.robot_id}")
        self._faults = FaultSchedule(scenario.faults, self.robot_id)
        # 충전 자리는 주차 지점이다. low_battery가 없으면 늘 90
        self._battery = Battery([f for f in scenario.faults if f.robot == self.robot_id],
                                [site.node_pose(n) for n in site.parking])
        x, y = site.node_pose(cfg.start)
        self._motion = Motion(x, y)
        self._reports = DelayLine(cfg.report_delay_s)
        x0, y0, _, _ = site.bounds()
        self._off_course = (x0 - OFF_COURSE_M, y0 - OFF_COURSE_M)
        self._site = site
        self._removed = False
        # 앞 물체와 자기 복구. 값은 노드 파라미터로 바꾼다(recovery_attempts 0: 복구 없음)
        self._recovery = ObstacleRecovery(RecoveryParams(**{
            f.name: self.declare_parameter(f.name, f.default).value
            for f in fields(RecoveryParams)}))
        self._space = FreeSpace(site, self._recovery.p.robot_radius_m)  # nav_goal 모드의 길 찾기
        self._others = {}  # 다른 로봇의 진실 자세와 받은 때, 속도(ahead.seen)
        self._sc = ScenarioClock(self)
        self._lock = threading.RLock()
        self._alive = True
        self._active: _Goal | None = None
        self._goals: dict[bytes, _Goal] = {}
        self._rest = RobotState.STATE_IDLE
        self._admin = AdminState(self.robot_id, str(uuid.uuid4()))  # 운영 모드와 요청 번호 기억
        self._last_t = self._sc.now()
        params = load_params_param(self)
        timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self._beat_timeout = timeout if timeout >= 0 else float(params["heartbeat_timeout_s"])
        # path가 로봇 자리에서 이보다 멀면 따라가지 않는다(관제 lane_error_warn_m)
        self._connect_tolerance = float(params["lane_error_warn_m"])
        # 실물 Pode처럼: 다음 조각을 받았으면 nav_goal 끝 노드 이만큼 앞에서 넘어가고(pode_fsm
        # pass_m), 아니면 끝에 서서 이 각속도로 목표 방향으로 돈다(RPP). 값은 관제 파일에서 읽는다
        self._pass_m = float(params["nav_pass_m"])
        self._turn_rps = float(params["nav_turn_rps"])
        self._last_beat = time.monotonic()

        self._state_pub = self.create_publisher(
            RobotState, contract.name("robot_state", pode=self.robot_id),
            to_ros_qos(contract.qos("robot_state")))
        self._truth_pub = self.create_publisher(
            RobotState, TRUTH_TOPIC.format(robot=self.robot_id),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(
            Header, contract.name("heartbeat"), self._on_heartbeat,
            to_ros_qos(contract.qos("heartbeat")))
        if any(f.robot == self.robot_id and (f.after_event or f.until_event)
               for f in scenario.faults):
            # 사건으로 시작하거나 끝나는 고장만 관제 사건을 듣는다(scenario_cfg.Fault)
            self.create_subscription(
                FleetEvent, contract.name("events"), self._on_event,
                to_ros_qos(contract.qos("events")))
        for other in scenario.robots:
            if other != self.robot_id:
                self.create_subscription(
                    RobotState, TRUTH_TOPIC.format(robot=other), self._on_other,
                    to_ros_qos(contract.qos("robot_state")))
        group = ReentrantCallbackGroup()
        self._move_server = self._server(
            MoveTo, contract.name("move_to", pode=self.robot_id), "move", group)
        self._dock_server = self._server(
            Dock, contract.name("dock", pode=self.robot_id), "dock", group)
        self._retreat_server = self._server(
            Retreat, contract.name("retreat", pode=self.robot_id), "retreat", group)
        self.create_service(
            SetMode, contract.name("robot_set_mode", robot=self.robot_id), self._on_set_mode,
            callback_group=group)
        self.create_timer(1.0 / CONTROL_RATE_HZ, self._tick)
        self.create_timer(1.0 / float(self.get_parameter("state_rate_hz").value),
                          self._publish_state)

    def _server(self, action_type, name, kind, group):
        return ActionServer(
            self, action_type, name,
            execute_callback=lambda gh: self._execute(gh, action_type),
            goal_callback=lambda _req: self._goal_policy(),
            cancel_callback=lambda _gh: self._cancel_policy(),
            handle_accepted_callback=lambda gh: self._accepted(gh, kind),
            callback_group=group)

    # ---- 수명주기 ----

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        self._move_server.destroy()
        self._dock_server.destroy()
        self._retreat_server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    def _on_other(self, msg: RobotState) -> None:
        self._others[msg.robot_id] = seen(
            self._others.get(msg.robot_id), msg.pose.x, msg.pose.y, self._sc.now())

    def _on_event(self, msg) -> None:
        with self._lock:
            self._faults.on_event(self._sc.now(), msg.source, msg.code, msg.detail)

    # ---- 하트비트 ----

    def _on_set_mode(self, request, response):
        with self._lock:
            reply = self._admin.set_mode(request.request_id, request.mode, time.monotonic())
        response.accepted = reply.accepted
        response.reason = reply.reason
        response.detail = reply.detail
        response.request_id = request.request_id
        return response

    def _on_heartbeat(self, _msg) -> None:
        if not self._faults.comms_down(self._sc.now()):
            self._last_beat = time.monotonic()

    def _heartbeat_lost(self) -> bool:
        """실제 시계. 프로세스를 감시하는 것이고, 부하가 걸리면 시뮬 시간이 앞서가기 때문이다."""
        return 0 < self._beat_timeout < time.monotonic() - self._last_beat

    # ---- 목표 처리 ----

    def _others_discs(self) -> list[tuple[float, float, float]]:
        radius = self._recovery.p.robot_radius_m
        t = self._sc.now() + 1.0 / CONTROL_RATE_HZ
        return [(*where_now(view, t), radius) for view in list(self._others.values())]

    def _yields(self, t: float) -> bool:
        first = [view for name, view in list(self._others.items()) if name < self.robot_id]
        return yields(self._motion.x, self._motion.y, first, t)

    # ---- 시뮬레이션 ----

    def _tick(self) -> None:
        with self._lock:
            t = self._sc.now()
            dt, self._last_t = t - self._last_t, t
            goal = self._active
            if self._faults.removed(t):
                self._remove(goal)
                return
            if self._removed:
                self._put_back(self._faults.put_back(t))
            # order는 조각을 달릴 때만 가는 중이다(base 끝 대기는 서 있는 로봇). 물체 자리는
            # 조각을 시작할 때 잡는다(pode_order._start_segment)
            moving = (goal is not None and not goal.done.is_set() and goal.kind == "move"
                      and goal.seg_mode != RETREAT
                      and (goal.order is None or goal.order.sub is not None))
            self._recovery.watch(t, self._faults.obstacle(t), moving, self._motion)
            if dt <= 0 or goal is None or goal.done.is_set() or self._faults.stuck(t):
                return
            if self._heartbeat_lost():
                self._end(goal, False, MoveTo.Result.REASON_HEARTBEAT_LOST)
                return
            if goal.kind == "move" and goal.order is not None:
                self._order_step(t, dt, goal)
            elif goal.kind == "move":
                out = self._move_step(t, dt, goal)
                if out is not None:
                    self._end(goal, *out)
            elif goal.kind == "retreat":
                self._retreat_step(t, dt, goal)
            elif t >= goal.end_at:
                e = self._dock_cfg.error_cm
                goal.error = (self._rng.gauss(e, 0.1 * e), self._rng.gauss(e, 0.1 * e),
                              self._rng.gauss(0.5, 0.1))
                self._end(goal, True)

    def _move_step(self, t: float, dt: float, goal: _Goal):
        """이동 한 틱. 끝나면 (성공, 이유), 아니면 None. order 조각도 이것으로 달린다."""
        others = self._others_discs()
        if goal.nav is not None and goal.nav.update(t, self._motion, others):
            return False, MoveTo.Result.REASON_NAV_ABORTED
        if goal.abort_at is not None and t >= goal.abort_at:
            if self._recovery.nav_aborted(t, self._motion, self._faults.behind(t)):
                return False, MoveTo.Result.REASON_NAV_ABORTED
            # 시도를 시작했다. 고장은 이미 났으니 취소돼도 돌려주지 않는다
            goal.abort_at, goal.abort_from, goal.aborting = None, None, True
        if (goal.aborting and goal.abort_at is None
                and not self._recovery.backing_up):
            goal.abort_at = t + NAV_ABORT_DELAY_S  # 다시 보낸 FollowPath도 중단된다
        # 물체가 있으면 선다. 시도가 남았으면 후진하고 다시 가 보고, 없으면 BLOCKED.
        # 앞에 다른 로봇이 있어도 선다. drive는 속도 0으로 불러 물체 앞에서 센 시간과 후진을 잇는다
        clear = self._nothing_ahead(goal, t, self._cfg.speed_mps * dt, others)
        self._recovery.drive(t, dt, self._motion, self._cfg.speed_mps if clear else 0.0,
                             others, self._faults.behind(t))
        if not self._motion.moving and (goal.nav is None or goal.nav.has_way):
            if (goal.nav is not None and goal.end_yaw is not None and self._turn_rps > 0.0
                    and not self._motion.turn(goal.end_yaw, self._turn_rps, dt)):
                return None
            self._recovery.succeeded()
            return True, ""
        return None

    def _remove(self, goal: _Goal | None) -> None:
        """운영자가 로봇을 코스에서 들어내 전원을 껐다."""
        if self._removed:
            return
        self._removed = True
        if goal is not None and not goal.done.is_set():
            self._end(goal, False, MoveTo.Result.REASON_NAV_ABORTED)
        self._motion.clear_target()
        self._motion.x, self._motion.y = self._off_course
        self._recovery.forget()  # 앞 물체는 들어낸 자리에 두고 간다

    def _put_back(self, node) -> None:
        """운영자가 로봇을 이 지점에 다시 놓고 켰다. 바구니는 시나리오가 정한 대로다."""
        self._removed = False
        self._motion.clear_target()
        self._motion.x, self._motion.y = self._site.node_pose(node)
        self._rest = RobotState.STATE_IDLE
        self._reports.clear()


def main(args=None):
    rclpy.init(args=args)
    node = FakePodeNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
