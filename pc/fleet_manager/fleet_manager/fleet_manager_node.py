# ---- fleet_core의 FleetManager를 ROS 노드로 감싼다 ----
from __future__ import annotations

import argparse
import math
import threading
import time
import uuid
from collections import deque
from functools import partial, wraps

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.common.commands import (
    CANCELED,
    NO_ANSWER,
    RESUME_SET_PAUSE_OFF,
    CancelCmd,
    CheckClearCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    MoveOrderCmd,
    ReportOrderCmd,
    ReportTripCmd,
    RetreatCmd,
    SwapCmd,
)
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.common.params import REPORT_REPLY_LIMIT_S, load_params, timing_warnings
from fleet_core.fleet_map import FILL
from fleet_core.manager import FleetManager
from fleet_core.traffic.site import load_site
from fleet_interfaces.action import Dock, FillBasket, MoveTo, Retreat, SwapBasket
from fleet_interfaces.msg import (
    ArmState,
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    RobotState,
    RobotStatusArray,
    SystemMode,
    ZoneStateArray,
)
from fleet_interfaces.srv import (
    CheckClear,
    CheckStock,
    ConfirmClear,
    ForceInput,
    Reinstate,
    ReportOrder,
    RobotCommand,
    SetLock,
    SetMode,
    SetPause,
    TripCommand,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Header

from fleet_manager import adapters
from fleet_manager.admin import (
    AdminCommands,
    Codes,
    Facts,
    Reply,
    Targets,
    judge_set_lock,
    judge_target,
    judge_test_only,
)
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox
from fleet_manager.inbox import Inbox
from fleet_manager.mode_sync import ModeSync
from fleet_manager.order_goals import apply_overrides
from fleet_manager.reports import BridgeReports
from fleet_manager.robot_calls import (
    ACCEPTED,
    REJECTED,
    TIMEOUT,
    UNCONFIRMED,
    CallParams,
    RobotCalls,
)
from fleet_manager.stock_check import LIMIT_S as STOCK_LIMIT_S
from fleet_manager.stock_check import StockChecks, shortfall


SLOW_TICK_WARN_S = 5.0  # 느린 틱 경고 사이 최소 간격
COMMAND_REPORTS_KEPT = 100  # 브리지 송신기를 잇기 전 기억하는 ReportCommand 수
POD_MOVING = (RobotState.STATE_MOVING, RobotState.STATE_DOCKING, RobotState.STATE_RECOVERING)
ARM_STOPPED = (ArmState.STATE_IDLE, ArmState.STATE_PAUSED, ArmState.STATE_ERROR)
TRIP_COMMANDS = ("resume_fill", "force_ship", "force_partial", "return_to_shelf", "resume_held",
                 "requeue_trip")


def _core_callback(method):
    """코어 그룹 콜백 표시: 지금 코어를 쥔 스레드를 남긴다 (코어는 이 스레드에서만 바뀐다)."""
    @wraps(method)
    def run(self, *args):
        self._core_thread = threading.get_ident()
        try:
            return method(self, *args)
        finally:
            self._core_thread = None
    return run


# ---- FleetManagerNode 생성과 인터페이스 설정 ----
class FleetManagerNode(Node):
    def __init__(self, record=None):
        super().__init__("fleet_manager")
        self.declare_parameter("robots", ["pode1", "pode2", "pode3"])
        self.declare_parameter("contract_file", "")
        self.declare_parameter("map_file", "")
        self.declare_parameter("params_file", "")
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("robots_hz", 5.0)  # 팀 명세 F-11: /fleet/robots 5 Hz 안팎
        # 관리 명령 (W-30 · W-31, Q-09 잠정 값)
        self.declare_parameter("admin_cache_size", 256)
        self.declare_parameter("admin_cache_s", 600.0)
        self.declare_parameter("robot_call_try_s", CallParams.try_s)
        self.declare_parameter("robot_call_gap_s", CallParams.gap_s)
        self.declare_parameter("robot_call_tries", CallParams.tries)
        self.declare_parameter("robot_call_confirm_s", CallParams.confirm_s)
        self.declare_parameter("robot_call_max_in_flight", CallParams.max_in_flight)
        unset = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("use_order_model", None, unset)
        self.declare_parameter("lookahead_s", None, unset)
        robots = list(self.get_parameter("robots").value)
        robots_hz = self.get_parameter("robots_hz").value
        if not (math.isfinite(robots_hz) and robots_hz > 0.0):
            raise ValueError(f"robots_hz must be > 0, got {robots_hz!r}")

        contract_file = self.get_parameter("contract_file").value or (
            get_package_share_directory("fleet_interfaces") + "/contract.yaml")
        self.contract = load_contract(contract_file)
        self.site = load_site(self.get_parameter("map_file").value or None)
        params = apply_overrides(
            load_params(self.get_parameter("params_file").value or None),
            self.get_parameter("use_order_model").value,
            self.get_parameter("lookahead_s").value)
        for text in timing_warnings(params):
            self.get_logger().warning(text)
        self.core = FleetManager(self.site, params, robots, self._now)
        self._record_file = None
        if record:
            self.core.check_recordable()
            self._record_file = open(record, "w", buffering=1, encoding="utf-8")
            self.core.record_to(lambda line: self._record_file.write(line + "\n"))

        self._goals = GoalTracker()
        self._reports = ReportOutbox()
        self._reached = set()
        self._last_zones = None
        self._last_assignments = None
        self._pode_states = {}
        self._arm_states = {}
        self._arm_fed = {}
        self._arm_lost = set()  # 상태가 끊겨 코어에 알린 팔
        self._started_at = time.monotonic()  # 한 번도 상태가 없는 팔은 이때부터 센다
        self._setup_admin(robots)
        # IR-30: 코어 그룹만 코어와 노드 상태를 바꾸고, 받은 편지함 그룹은 큐에 넣기만 한다
        self._core_group = MutuallyExclusiveCallbackGroup()
        self._inbox_group = MutuallyExclusiveCallbackGroup()
        self._inbox = Inbox()
        self._core_thread = None
        self._tick_period_s = 1.0 / self.get_parameter("tick_hz").value
        self._slow_warned_at = -math.inf
        self._create_interfaces(robots)
        self.create_timer(self._tick_period_s, self._on_tick, callback_group=self._core_group)
        self.create_timer(1.0 / robots_hz, self._publish_robots, callback_group=self._core_group)
        self._silent_until = (time.monotonic() + params.heartbeat_timeout_s
                              + params.heartbeat_period_s)
        # 코어가 멈추면 heartbeat도 끊겨 로봇이 IR-34대로 서도록 일부러 코어 그룹에 둔다
        self.create_timer(params.heartbeat_period_s, self._publish_heartbeat,
                          clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=self._core_group)

    def destroy_node(self) -> None:
        if self._record_file:
            self._record_file.close()
        super().destroy_node()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def silent(self) -> bool:
        return time.monotonic() < self._silent_until

    def _qos(self, key):
        return to_ros_qos(self.contract.qos(key))

    def _create_interfaces(self, robots) -> None:
        name = self.contract.name
        self._zones_pub = self.create_publisher(
            ZoneStateArray, name("zones"), self._qos("zones"))
        self._assignments_pub = self.create_publisher(
            AssignmentArray, name("assignments"), self._qos("assignments"))
        self._robots_pub = self.create_publisher(
            RobotStatusArray, name("robots"), self._qos("robots"))
        self._heartbeat_pub = self.create_publisher(
            Header, name("heartbeat"), self._qos("heartbeat"))
        self._events_pub = self.create_publisher(FleetEvent, name("events"), self._qos("events"))
        inbox = self._inbox_group
        self.create_subscription(
            OrderBoard, name("order_board"), partial(self._post, self._on_order_board),
            self._qos("order_board"), callback_group=inbox)
        self._move = {}
        self._dock = {}
        self._retreat = {}
        self._check_clear = {}
        for robot in robots:
            self.create_subscription(
                RobotState, name("robot_state", pode=robot),
                partial(self._post, self._on_robot_state, robot), self._qos("robot_state"),
                callback_group=inbox)
            self._move[robot] = ActionClient(self, MoveTo, name("move_to", pode=robot),
                                             callback_group=inbox)
            self._dock[robot] = ActionClient(self, Dock, name("dock", pode=robot),
                                             callback_group=inbox)
            self._retreat[robot] = ActionClient(self, Retreat, name("retreat", pode=robot),
                                                callback_group=inbox)
            self._check_clear[robot] = self.create_client(
                CheckClear, name("check_clear", pode=robot), callback_group=inbox)
        for arm in self.contract.placeholders.get("arm", []):
            self.create_subscription(
                ArmState, name("arm_state", arm=arm), partial(self._post_arm_state, arm),
                self._qos("arm_state"), callback_group=inbox)
        self._fill = {}
        self._swap = {}
        for station in self.site.stations.values():
            if station.action == FILL:
                self._fill[station.id] = ActionClient(self, FillBasket, name("fill_basket"),
                                                      callback_group=inbox)
            else:
                self._swap[station.id] = ActionClient(
                    self, SwapBasket, name("swap_basket", arm=station.server),
                    callback_group=inbox)
        self._report_client = self.create_client(ReportOrder, name("report_order"),
                                                 callback_group=inbox)
        # 담기 전 결품 확인: 브리지에 서버가 없으면 확인 없이 보낸다 (W-40)
        self._stock_client = self.create_client(CheckStock, name("check_stock"),
                                                callback_group=inbox)
        self._stock = StockChecks()
        # 운영 모드: 브리지가 내면 로봇마다 set_mode, 받기 전에는 아무것도 하지 않는다 (IR-73)
        arms = list(self.contract.placeholders.get("arm", []))
        self._mode_sync = ModeSync(robots, arms)
        self._held_fills: list[FillCmd] = []  # 작업대 팔의 모드 보고를 기다리는 담기 목표
        self._set_mode = {r: self.create_client(SetMode, name("robot_set_mode", robot=r),
                                                callback_group=inbox)
                          for r in list(robots) + arms}
        self._mode_futures = {}
        self.create_subscription(SystemMode, name("system_mode"),
                                 partial(self._post, self._on_system_mode),
                                 self._qos("system_mode"), callback_group=inbox)
        core = self._core_group
        self.create_service(ConfirmClear, name("confirm_clear"), self._on_confirm_clear,
                            callback_group=core)
        self.create_service(Reinstate, name("reinstate"), self._on_reinstate, callback_group=core)
        for key in TRIP_COMMANDS:
            self.create_service(TripCommand, name(key), partial(self._on_trip_command, key),
                                callback_group=core)
        self._create_admin_interfaces()
        # 브리지 보고(IR-22·23·12): Hello로 epoch를 받기 전에는 쌓아 둔다
        self._bridge = BridgeReports(self, self.core.params, list(robots) + arms, self._post,
                                     getattr(self.core, "on_open_trips", None))

    # ---- 관리 명령: 판단, 같은 번호, 로봇 부르기 (W-30 ~ W-34) ----
    def _setup_admin(self, robots) -> None:
        get = lambda key: self.get_parameter(key).value  # noqa: E731
        codes = Codes(self.contract.reason_codes)
        self.command_reports = deque(maxlen=COMMAND_REPORTS_KEPT)
        self._admin = AdminCommands(codes, self.report_command, time.monotonic,
                                    int(get("admin_cache_size")), float(get("admin_cache_s")))
        self._robot_calls = RobotCalls(
            CallParams(float(get("robot_call_try_s")), float(get("robot_call_gap_s")),
                       int(get("robot_call_tries")), float(get("robot_call_confirm_s")),
                       int(get("robot_call_max_in_flight"))),
            codes, self.report_command, self._command_event)
        pods = list(robots)
        arms = list(self.contract.placeholders.get("arm", []))
        within = lambda key, field, names: frozenset(  # noqa: E731
            n for n in names if n in self._instances(key, field))
        self._admin_targets = Targets(
            set_lock=within("robot_set_lock", "robot", pods + arms),
            fault_resolved=within("arm_fault_resolved", "arm", arms),
            charge_end=within("pode_charge_end", "pode", pods),
            force_input=within("robot_force", "robot", pods + arms))
        self._pause_arms = within("arm_set_pause", "arm", arms)
        self._released = {}  # 락이 풀린 로봇 -> 풀 때 본 boot_id

    def _instances(self, key, field) -> list:
        entry = self.contract.entries[key]
        return list(entry.instances.get(field, self.contract.placeholders.get(field, [])))

    def _create_admin_interfaces(self) -> None:
        name, inbox, core, t = (self.contract.name, self._inbox_group, self._core_group,
                                self._admin_targets)
        self._lock_clients = {r: self.create_client(SetLock, name("robot_set_lock", robot=r),
                                                    callback_group=inbox)
                              for r in sorted(t.set_lock)}
        self._fault_clients = {a: self.create_client(
            RobotCommand, name("arm_fault_resolved", arm=a), callback_group=inbox)
            for a in sorted(t.fault_resolved)}
        self._charge_clients = {p: self.create_client(
            RobotCommand, name("pode_charge_end", pode=p), callback_group=inbox)
            for p in sorted(t.charge_end)}
        self._force_clients = {r: self.create_client(ForceInput, name("robot_force", robot=r),
                                                     callback_group=inbox)
                               for r in sorted(t.force_input)}
        self._pause_clients = {a: self.create_client(
            SetPause, name("arm_set_pause", arm=a), callback_group=inbox)
            for a in sorted(self._pause_arms)}
        self.create_service(SetLock, name("set_lock"), self._on_set_lock, callback_group=core)
        self.create_service(RobotCommand, name("fault_resolved"), self._on_fault_resolved,
                            callback_group=core)
        self.create_service(RobotCommand, name("charge_end"), self._on_charge_end,
                            callback_group=core)
        self.create_service(ForceInput, name("force_input"), self._on_force_input,
                            callback_group=core)
        self.create_service(RobotCommand, name("resume_arm"), self._on_resume_arm,
                            callback_group=core)

    def _command(self, key, **fields) -> str:
        """ReportCommand의 command: 계약 서비스 이름의 끝 (예: set_lock)."""
        return self.contract.name(key, **fields).rsplit("/", 1)[-1]

    def _command_event(self, level, code, detail, source) -> None:
        self._dispatch(EventOut(level, code, detail, source), self._now())

    # ---- 받은 편지함: 넣기(받은 편지함 그룹, 아무 스레드)와 꺼내 처리하기(코어 그룹) ----
    def _post(self, handler, *args) -> None:
        self._inbox.put(handler, args)

    def _drain(self) -> None:
        for handler, args in self._inbox.take():
            handler(*args)

    # ---- 받은 사건 처리: 큐에서 꺼낸 뒤 코어 그룹에서만 돈다 ----
    def _on_robot_state(self, robot, msg: RobotState) -> None:
        self._pode_states[robot] = msg
        self._mode_sync.on_state(robot, msg.mode, msg.boot_id, time.monotonic())
        self._observe_lock(robot, msg)
        if not self.silent() and self._reachable(robot):
            self.core.on_robot_state(robot, *adapters.robot_report(msg),
                                     adapters.robot_report_id(msg),
                                     *adapters.robot_obstacle(msg),
                                     adapters.robot_local_yield(msg))

    def _post_arm_state(self, arm, msg: ArmState) -> None:
        # 끊김은 도착 시각으로 센다(코어가 늦게 꺼내도)
        self._post(self._on_arm_state, arm, msg, time.monotonic())

    def _on_arm_state(self, arm, msg: ArmState, at: float | None = None) -> None:
        now = time.monotonic() if at is None else at
        before = self._arm_heard_at(arm)
        self._arm_states[arm] = (msg, now)
        self._mode_sync.on_state(arm, msg.mode, msg.boot_id, now)
        self._observe_lock(arm, msg)
        if self.silent():
            return
        # 틱 검사 전에 한도를 넘겨 들어온 상태도 끊김을 먼저 알린다
        if (before is not None and arm not in self._arm_lost
                and now - before > self.core.params.comm_timeout_s):
            self._tell_arm_lost(arm)
        # 5 Hz 상태 가운데 바뀐 것만 코어에 넣는다(기록이 부풀지 않게)
        seen = (msg.state, msg.detail, msg.holding, msg.boot_id, msg.lock)
        if self._arm_fed.get(arm) != seen:
            self._arm_fed[arm] = seen
            self._arm_lost.discard(arm)
            self.core.on_arm_state(arm, *seen)

    def _arm_heard_at(self, arm) -> float | None:
        # 작업대 팔은 한 번도 보내지 않았어도 시작 시각부터 센다
        start = self._started_at if arm in self._pause_arms else None
        return self._arm_states.get(arm, (None, start))[1]

    def _tell_arm_lost(self, arm) -> None:
        self._arm_lost.add(arm)
        self._arm_fed.pop(arm, None)
        self.core.on_arm_state_lost(arm)

    def _check_arms_lost(self) -> None:
        """comm_timeout_s 동안 상태가 없으면 한 번 알린다. 다음 상태는 바뀐 것으로 다시 넣는다."""
        limit = self.core.params.comm_timeout_s
        for arm in sorted(set(self._arm_states) | self._pause_arms):
            if arm not in self._arm_lost and time.monotonic() - self._arm_heard_at(arm) > limit:
                self._tell_arm_lost(arm)

    def _on_system_mode(self, msg: SystemMode) -> None:
        if self._mode_sync.on_system_mode(msg.mode):
            self.get_logger().info(f"system mode {msg.mode}, telling the robots")

    def _arm_rows(self) -> list:
        rows = []
        for arm in self.contract.placeholders.get("arm", []):
            msg, at = self._arm_states.get(arm, (None, None))
            fresh = at is not None and time.monotonic() - at <= self.core.params.comm_timeout_s
            rows.append((arm, msg, fresh, *self.core.arm_task(arm)))
        return rows

    def _reachable(self, robot) -> bool:
        if (robot not in self._reached and self._move[robot].server_is_ready()
                and self._dock[robot].server_is_ready()):
            self._reached.add(robot)
        return robot in self._reached

    def _on_order_board(self, msg: OrderBoard) -> None:
        self.core.on_order_board(adapters.order_board_mode(msg), adapters.order_board_orders(msg))

    @_core_callback
    def _on_confirm_clear(self, request, response):
        self._drain()

        def apply():
            released = self.core.confirm_clear(*adapters.confirm_clear_args(request))
            return Reply(released is not None, self.core.last_clear_code,
                         self.core.last_clear_reason, tuple(released or ()))
        rid = _request_id(request)
        reply = self._admin.local(self._command("confirm_clear"), rid, request.robot_id, apply)
        response = adapters.confirm_clear_response(
            response, list(reply.extra) if reply.accepted else None, reply.reason, rid)
        response.detail = reply.detail
        return response

    @_core_callback
    def _on_reinstate(self, request, response):
        self._drain()

        def apply():
            accepted = self.core.reinstate(*adapters.reinstate_args(request))
            return Reply(accepted, self.core.last_reinstate_code, self.core.last_reinstate_reason)
        rid = _request_id(request)
        return _answer(response, rid, self._admin.local(
            self._command("reinstate"), rid, request.robot_id, apply))

    @_core_callback
    def _on_set_lock(self, request, response):
        self._drain()
        rid, robot, locked = _request_id(request), request.robot_id, bool(request.locked)

        def judge():
            # 같은 로봇의 set_lock은 하나씩: 앞 명령이 끝나야 다음을 받는다
            busy = self._robot_calls.pending(self._command("set_lock"), robot)
            if busy is not None:
                return Reply(False, self._admin.codes.NOT_READY, f"set_lock {busy} still pending")
            return judge_set_lock(self._admin.codes, self._admin_targets, robot, locked,
                                  request.confirm_drop, self._lock_facts)
        return _answer(response, rid, self._admin.robot(
            self._command("set_lock"), rid, robot, judge,
            partial(self._send_set_lock, rid, robot, locked, bool(request.confirm_drop))))

    @_core_callback
    def _on_fault_resolved(self, request, response):
        self._drain()
        rid, robot = _request_id(request), request.robot_id
        judge = lambda: judge_target(  # noqa: E731
            self._admin.codes, self._admin_targets.fault_resolved, robot)
        send = lambda: self._call_robot(  # noqa: E731
            rid, "fault_resolved", robot, self._fault_clients,
            RobotCommand.Request(request_id=rid, robot_id=robot),
            # 보낼 때 ERROR였던 팔은 받았다는 답도 ERROR를 벗어나야 확정 (첫 요청부터 30 s)
            observe=self._left_error if self._in_error(robot) else None, confirm_accepted=True)
        return _answer(response, rid, self._admin.robot(
            self._command("fault_resolved"), rid, robot, judge, send))

    @_core_callback
    def _on_charge_end(self, request, response):
        self._drain()
        rid, robot = _request_id(request), request.robot_id
        judge = lambda: judge_test_only(  # noqa: E731
            self._admin.codes, self._admin_targets.charge_end, robot,
            self._mode_sync.system_mode)
        send = lambda: self._call_robot(  # noqa: E731
            rid, "charge_end", robot, self._charge_clients,
            RobotCommand.Request(request_id=rid, robot_id=robot),
            then=partial(self._charge_result, robot))
        return _answer(response, rid, self._admin.robot(
            self._command("charge_end"), rid, robot, judge, send))

    @_core_callback
    def _on_force_input(self, request, response):
        self._drain()
        rid, robot = _request_id(request), request.robot_id
        judge = lambda: judge_test_only(  # noqa: E731
            self._admin.codes, self._admin_targets.force_input, robot,
            self._mode_sync.system_mode)
        send = lambda: self._call_robot(  # noqa: E731
            rid, "force_input", robot, self._force_clients,
            ForceInput.Request(request_id=rid, robot_id=robot, kind=request.kind,
                               key=request.key, value=request.value))
        return _answer(response, rid, self._admin.robot(
            self._command("force_input"), rid, robot, judge, send))

    @_core_callback
    def _on_resume_arm(self, request, response):
        """관리자 [복구]: 코어가 두 팔 set_pause(false) · 새 담기 목표 · 거절을 정한다."""
        self._drain()
        rid, target = _request_id(request), request.robot_id
        command, how = self._command("resume_arm"), []

        def judge():
            # 앞 [복구]의 set_pause가 끝나야 다음을 받는다
            for arm in sorted(self._pause_clients):
                busy = self._robot_calls.pending(self._command("arm_set_pause", arm=arm), arm)
                if busy is not None:
                    return Reply(False, self._admin.codes.NOT_READY,
                                 f"set_pause {busy} on {arm} still pending")
            answer, reason = self.core.on_resume_arm(target)
            if not answer:
                return Reply(False, reason, f"resume_arm refused on {target!r}")
            how.append(answer)
            return Reply(True)

        def send():
            if how[0] == RESUME_SET_PAUSE_OFF:
                self._send_set_pause_off(rid, target)
            else:
                # 남은 상품의 새 담기 목표는 코어가 이미 냈다
                self.report_command(rid, command, target, ACCEPTED, "", f"{how[0]} queued")
        return _answer(response, rid, self._admin.robot(command, rid, target, judge, send))

    def _send_set_pause_off(self, rid, target) -> None:
        """두 팔에 set_pause(false). 같은 번호의 끝 상태는 한 번이라 두 결과를 모아 알린다."""
        command, arms = self._command("resume_arm"), sorted(self._pause_clients)
        ends, unsure = {}, []

        def report(arm, _rid, _command, _target, result, reason="", detail=""):
            if result == UNCONFIRMED:
                if not unsure:
                    unsure.append(arm)
                    self.report_command(rid, command, target, result, reason, f"{arm}: {detail}")
                return
            ends[arm] = (result, reason, detail)
            if len(ends) == len(arms):
                self.report_command(rid, command, target, *_joined(ends))
        for arm in arms:
            self._robot_calls.add(
                rid, self._command("arm_set_pause", arm=arm), arm, self._pause_clients[arm],
                SetPause.Request(request_id=rid, paused=False), time.monotonic(),
                observe=self._left_paused if self._paused(arm) else None,
                key=(rid, arm), report=partial(report, arm))

    # ---- 로봇 부르기와 락 상태 ----
    def _call_robot(self, rid, key, robot, clients, request, observe=None, then=None,
                    confirm_accepted=False) -> None:
        self._robot_calls.add(rid, self._command(key), robot, clients[robot], request,
                              time.monotonic(), observe, then,
                              confirm_accepted=confirm_accepted)

    def _send_set_lock(self, rid, robot, locked, confirm_drop) -> None:
        was = robot in self._released
        if not locked:
            # 풀기는 받자마자 배정에서 뺀다. 로봇이 거절하면 되돌린다
            self._set_released(robot, True, f"set_lock {rid}")
        self._call_robot(rid, "set_lock", robot, self._lock_clients,
                         SetLock.Request(request_id=rid, robot_id=robot, locked=locked,
                                         confirm_drop=confirm_drop),
                         observe=lambda state: bool(state.lock) == locked,
                         then=partial(self._lock_result, robot, locked, was))

    def _lock_result(self, robot, locked, was_released, accepted) -> None:
        if accepted:
            self._set_released(robot, not locked, "set_lock confirmed")
        elif accepted is False and not locked:
            self._set_released(robot, was_released, "the robot refused set_lock")
        # 결과 모름(None)이면 그대로 둔다: 풀렸을 수 있다

    def _charge_result(self, robot, accepted) -> None:
        # 충전은 충전 칸에 들어가는 것만 보여 주므로, 로봇이 받아들이면 관제가 충전을 끝낸다
        if accepted:
            self.core.end_charge(robot, "charge_end confirmed")

    def _set_released(self, robot, released, detail) -> None:
        if released:
            if robot not in self._released:
                self._released[robot] = self._boot_id(robot)
        else:
            self._released.pop(robot, None)
        if robot in self.core.traffic.robots:
            self.core.set_lock_released(robot, released, detail)
        # 양팔 묶음. 쥔 채 락 해제는 PAUSED로 이어 가고 목표 끝에 보류

    def _in_error(self, arm) -> bool:
        msg = self._arm_states.get(arm, (None,))[0]
        return msg is not None and msg.state == ArmState.STATE_ERROR

    def _paused(self, arm) -> bool:
        msg = self._arm_states.get(arm, (None,))[0]
        return msg is not None and msg.state == ArmState.STATE_PAUSED

    @staticmethod
    def _left_paused(state) -> bool:
        # 결과 모름은 PAUSED였던 팔이 PAUSED를 벗어나면 확정한다
        return state.state != ArmState.STATE_PAUSED

    @staticmethod
    def _left_error(state) -> bool:
        # 결과 모름은 ERROR였던 팔이 ERROR를 벗어나면 확정한다 (Phody DECIDE-5 제안)
        return state.state != ArmState.STATE_ERROR

    def _boot_id(self, robot) -> str:
        msg = self._pode_states.get(robot) or self._arm_states.get(robot, (None,))[0]
        return msg.boot_id if msg is not None else ""

    def _lock_facts(self, robot) -> Facts:
        """멈춤: Pode는 관제 goal이 없고 MOVING · DOCKING · RECOVERING이 아님, 팔은 IDLE · PAUSED · ERROR."""
        if robot in self.core.traffic.robots:
            msg = self._pode_states.get(robot)
            view = self.core.traffic.robot_view(robot)
            if msg is None or view.get("comm"):
                return Facts(False, why=f"no fresh state from {robot}")
            if msg.state in POD_MOVING:
                return Facts(False, why=f"{robot} is {msg.state}")
            if view.get("goal_id") is not None:
                return Facts(False, why=f"{robot} has goal {view['goal_id']}")
            return Facts(True)
        msg, at = self._arm_states.get(robot, (None, None))
        if msg is None or time.monotonic() - at > self.core.params.comm_timeout_s:
            return Facts(False, why=f"no fresh state from {robot}")
        if msg.state not in ARM_STOPPED:
            return Facts(False, holding=bool(msg.holding), why=f"{robot} is {msg.state}")
        return Facts(True, holding=bool(msg.holding))

    def _observe_lock(self, robot, msg) -> None:
        for client, future in self._robot_calls.observe(robot, msg, time.monotonic()):
            client.remove_pending_request(future)
        boot = self._released.get(robot)
        # 다시 켜진 로봇은 잠긴 채 시작한다 (RobotState.lock: 켜질 때 true)
        if boot is not None and msg.lock and msg.boot_id and msg.boot_id != boot:
            self._set_released(robot, False, "rebooted with the lock on")

    def _step_robot_calls(self) -> None:
        step = self._robot_calls.step(time.monotonic(), lambda client: client.service_is_ready())
        for client, future in step.drop:
            client.remove_pending_request(future)
        for key, client, request in step.send:
            future = client.call_async(request)
            self._robot_calls.track(key, future)
            future.add_done_callback(partial(self._post, self._robot_answered, key))

    def _robot_answered(self, key, future) -> None:
        try:
            reply = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"command {key} got no reply: {exc}")
            self._robot_calls.on_failed(key, future)
            return
        # 답을 받을 때의 마지막 상태로도 확정한다(상태가 답보다 먼저 바뀐 경우)
        target = self._robot_calls.target(key)
        view = self._arm_states.get(target, (None,))[0] if target else None
        for client, old in self._robot_calls.on_answer(key, future, reply.accepted,
                                                       reply.reason, reply.detail,
                                                       time.monotonic(), view):
            client.remove_pending_request(old)

    @_core_callback
    def _on_trip_command(self, key, request, response):
        """운반 건 명령 6개: 판단은 주문 쪽, 처리 직후 ReportCommand 한 번 (W-32)."""
        self._drain()
        command = self._command(key)

        def apply():
            if key == "force_ship":
                accepted, reason = self.core.force_ship(adapters.force_ship_args(request))
                return Reply(accepted, "" if accepted else reason)
            accepted, reason, detail = self.core.on_trip_command(
                command, request.trip_id, adapters.from_order_items(request.mark_missing))
            return Reply(accepted, "" if accepted else reason, detail)
        rid = _request_id(request)
        return _answer(response, rid, self._admin.local(command, rid, request.trip_id, apply))

    # ---- 주기 틱: 받은 사건 처리, 명령 내보내기, 취소 재시도, 주문 보고, 상태 발행 ----
    @_core_callback
    def _on_tick(self) -> None:
        started = time.monotonic()
        waiting = self._inbox.qsize()
        self._drain()
        self._step_robot_calls()
        now = self._now()
        if now > 0.0 and not self.silent():
            self._sync_modes(time.monotonic())
            self._check_arms_lost()
            for command in self.core.tick():
                self._dispatch(command, now)
            # 코어가 낸 취소 · 타임아웃을 먼저 처리한 뒤 보낸다
            self._release_fills()
            self._step_stock_checks(time.monotonic())
            self._retry_cancels(now)
            self._flush_reports(now)
            self._bridge.flush()
            self._publish_snapshots(now)
        ended = time.monotonic()
        took = ended - started
        if took > self._tick_period_s and ended - self._slow_warned_at >= SLOW_TICK_WARN_S:
            self._slow_warned_at = ended
            self.get_logger().warning(
                f"tick took {took * 1000:.1f} ms, over the {self._tick_period_s * 1000:.0f} ms "
                f"period ({waiting} events waited, inbox peak {self._inbox.peak})")

    # ---- 운영 모드: set_mode 부르기, 모드가 다른 Pode를 코어에 알리기 ----
    def _sync_modes(self, now: float) -> None:
        mode = self._mode_sync.system_mode
        missing = set(self._mode_sync.no_server)
        ready = lambda r: self._set_mode[r].service_is_ready()  # noqa: E731
        calls = self._mode_sync.due(now, ready)
        for robot in self._mode_sync.expired:
            old = self._mode_futures.pop(robot, None)
            if old is not None and not old.done():
                self._set_mode[robot].remove_pending_request(old)
        for robot, request_id in calls:
            client = self._set_mode[robot]
            old = self._mode_futures.pop(robot, None)
            if old is not None and not old.done():
                client.remove_pending_request(old)
            future = client.call_async(SetMode.Request(request_id=request_id, mode=mode))
            future.add_done_callback(partial(self._post, self._mode_answered, robot, request_id))
            self._mode_futures[robot] = future
        for robot in sorted(self._mode_sync.no_server - missing):
            self.get_logger().warning(f"{robot} has no set_mode server, mode {mode} not told")
        for robot, blocked, detail in self._mode_sync.changes():
            self.core.set_mode_blocked(robot, blocked, detail)

    def _mode_answered(self, robot, request_id, future) -> None:
        try:
            reply = future.result()
        except Exception as exc:  # noqa: BLE001
            # 답이 없던 것으로 두어 같은 번호로 다시 보낸다 (IR-20)
            self.get_logger().warning(f"set_mode on {robot} failed: {exc}")
            return
        self._mode_sync.on_answer(robot, request_id)
        if not reply.accepted:
            self.get_logger().warning(
                f"{robot} refused mode: {reply.reason} {reply.detail}".rstrip())

    def _retry_cancels(self, now: float) -> None:
        params = self.core.params
        # 코어가 취소하지 않고 정한 결과 잃은 goal의 handle은 정하는 시간의 두 배 뒤 놓는다
        self._goals.drop_stale_lost(now, 2 * params.robot_timeout_settle_s)
        for server, goal_id in self._goals.retry_cancels(now, params.cancel_retry_s,
                                                         params.cancel_warn_after):
            waited = params.cancel_warn_after * params.cancel_retry_s
            detail = (f"goal {goal_id} on {server} has no result about {waited:.0f} s after it was "
                      "canceled; the cancel stays open while a result can still come")
            self._dispatch(EventOut("WARN", "CANCEL_UNCONFIRMED", detail, server), now)
        for robot, goal_id in self._goals.take_dropped():
            self.core.on_goal_lost(robot, goal_id)

    def _flush_reports(self, now: float) -> None:
        params = self.core.params
        # IR-41: 응답 한도를 넘긴 요청은 클라이언트에서 버린다
        for old in self._reports.expire(now, REPORT_REPLY_LIMIT_S):
            self._report_client.remove_pending_request(old)
        for report_id, command in self._reports.due(now, self._report_client.service_is_ready(),
                                                    params.report_retry_s,
                                                    params.report_max_in_flight):
            future = self._report_client.call_async(adapters.report_request(command))
            self._reports.track(report_id, future)
            future.add_done_callback(partial(self._post, self._report_answered, report_id,
                                             command))

    def _report_answered(self, report_id, command, future) -> None:
        try:
            accepted = adapters.report_accepted(future.result())
        except Exception as exc:
            self.get_logger().warning(f"report of {command.order_id} got no reply: {exc}")
            self._reports.forget(report_id, future)
            return
        others = self._reports.finish(report_id, future)
        if others is None:
            return
        # 끝난 보고의 다른 요청은 답을 기다리지 않는다
        for old in others:
            self._report_client.remove_pending_request(old)
        if not accepted:
            self._dispatch(EventOut("ERROR", "ORDER_REPORT_REFUSED",
                                    f"{command.order_id} {command.status} refused by central, "
                                    "nobody will work or report it now", command.robot),
                           self._now())

    def _publish_snapshots(self, now: float) -> None:
        zones = self.core.zones_view()
        if self.core.traffic.everyone_placed and zones != self._last_zones:
            self._last_zones = zones
            self._zones_pub.publish(adapters.zones_to_msg(zones, now))
        assignments = self.core.assignments_view()
        if assignments != self._last_assignments:
            self._last_assignments = assignments
            self._assignments_pub.publish(adapters.assignments_to_msg(assignments, now))

    @_core_callback
    def _publish_heartbeat(self) -> None:
        if not self.silent() and self.core.traffic.everyone_placed:
            self._heartbeat_pub.publish(Header(stamp=self.get_clock().now().to_msg(),
                                               frame_id=self.get_name()))

    @_core_callback
    def _publish_robots(self) -> None:
        now = self._now()
        if now > 0.0:
            self._robots_pub.publish(adapters.robots_to_msg(
                self.core.robots_view(), now, self._pode_states, self._arm_rows()))

    # ---- 코어 명령 처리: 액션, 막힘 지점 확인, 취소, 주문 보고, 사건 ----
    def _dispatch(self, command, now: float) -> None:
        match command:
            case MoveCmd():
                self._send(self._move[command.robot], adapters.move_goal(command),
                           (command.robot, command.goal_id), self._move_done,
                           self._robot_rejected)
            case MoveOrderCmd():
                key = (command.robot, command.goal_id)
                self._send(self._move[command.robot], adapters.move_order_goal(command), key,
                           self._move_done, self._robot_rejected,
                           feedback=partial(self._post, self._on_move_feedback, key))
            case DockCmd():
                self._send(self._dock[command.robot], adapters.dock_goal(command),
                           (command.robot, command.goal_id), self._dock_done,
                           self._robot_rejected)
            case RetreatCmd():
                self._send(self._retreat[command.robot], adapters.retreat_goal(command),
                           (command.robot, command.goal_id), self._retreat_done,
                           self._robot_rejected)
            case FillCmd():
                self._fill_or_hold(command)
            case SwapCmd():
                self._send(self._swap[command.station_id], adapters.swap_goal(command),
                           (command.arm, command.goal_id), self._swap_done,
                           self._swap_rejected)
            case CheckClearCmd():
                self._ask_clear(command)
            case CancelCmd():
                if not self._drop_held_fill(command):
                    self._goals.cancel((command.robot, command.goal_id), now)
            case ReportOrderCmd():
                self._reports.add(command)
                self._bridge.trip(command)
            case ReportTripCmd():
                self._bridge.trip(command)
            case EventOut():
                self._events_pub.publish(adapters.event_to_msg(command, now))
                self._bridge.event(command)
                self._log_event(command)

    # ---- 담기: 작업대 팔이 시스템 모드를 보고하기 전에는 보내지 않는다 (IR-72, PI-20) ----
    def _arms_waiting(self, command) -> list[str]:
        arms = self.site.stations[command.station_id].arms
        return [arm for arm in arms if not self._mode_sync.settled(arm)]

    def _fill_or_hold(self, command) -> None:
        waiting = self._arms_waiting(command)
        if not waiting:
            self._check_then_send(command)
            return
        self._held_fills.append(command)
        self.get_logger().info(
            f"fill goal {command.goal_id} waits until {', '.join(waiting)} report mode "
            f"{self._mode_sync.system_mode}")

    def _release_fills(self) -> None:
        if not self._held_fills:
            return
        # 팔 ERROR · 초기화로 코어가 이미 포기한 목표는 취소 명령 없이 버린다
        for command in self._held_fills:
            if not self.core.arm.holds(command.goal_id):
                self.get_logger().info(f"held fill goal {command.goal_id} dropped: "
                                       "the core gave the job up")
        live = [c for c in self._held_fills if self.core.arm.holds(c.goal_id)]
        ready = [c for c in live if not self._arms_waiting(c)]
        self._held_fills = [c for c in live if all(c is not r for r in ready)]
        for command in ready:
            self._check_then_send(command)

    def _send_fill(self, command) -> None:
        server = self.site.stations[command.station_id].server
        key = (server, command.goal_id)
        self._send(self._fill[command.station_id], adapters.fill_goal(command), key,
                   self._fill_done, self._fill_rejected,
                   feedback=partial(self._post, self._on_fill_feedback, key))

    def _drop_held_fill(self, command) -> bool:
        """보내기 전에 취소된 담기 목표는 버리고 취소된 결과로 끝낸다."""
        for held in self._held_fills:
            server = self.site.stations[held.station_id].server
            if (server, held.goal_id) == (command.robot, command.goal_id):
                self._held_fills.remove(held)
                self.core.on_fill_result(held.goal_id, False, {}, {}, CANCELED)
                return True
        checked = self._stock.command(command.goal_id)
        if checked is None or self.site.stations[checked.station_id].server != command.robot:
            return False
        self._drop_stock_future(self._stock.take(command.goal_id)[1])
        self.core.on_fill_result(command.goal_id, False, {}, {}, CANCELED)
        return True

    # ---- 담기 전 결품 확인 (W-40 · PI-28): 모자라면 보내지 않고 OUT_OF_STOCK 결과로 끝낸다 ----
    def _check_then_send(self, command) -> None:
        if not self._stock_client.service_is_ready():
            self._send_fill(command)
            return
        try:
            future = self._stock_client.call_async(adapters.check_stock_request(command))
        except Exception as exc:  # noqa: BLE001
            self._stock_unanswered(command, f"check_stock was not sent: {exc}")
            return
        self._stock.ask(command, future, time.monotonic())
        future.add_done_callback(partial(self._stock_arrived, command.goal_id))

    def _stock_arrived(self, goal_id, future) -> None:
        """3초 판정은 처리 시각이 아니라 답이 온 시각으로 한다."""
        self._post(self._stock_answered, goal_id, future, time.monotonic())

    def _stock_answered(self, goal_id, future, arrived_at: float) -> None:
        check = self._stock.answered(goal_id, future)
        if check is None:
            return  # 한도가 지났거나 취소 · 포기로 버린 확인
        command = check.command
        if not self.core.arm.holds(goal_id):
            # 같은 inbox 묶음에서 팔 ERROR · 초기화로 코어가 먼저 포기했다
            self.get_logger().info(f"fill goal {goal_id} stock answer dropped: "
                                   "the core gave the job up")
            return
        if arrived_at - check.asked_at > STOCK_LIMIT_S:
            self._stock_unanswered(command, f"no stock answer within {STOCK_LIMIT_S:.0f} s")
            return
        try:
            accepted, available, reason = adapters.check_stock_answer(future.result())
        except Exception as exc:  # noqa: BLE001
            self._stock_unanswered(command, f"check_stock got no reply: {exc}")
            return
        if not accepted:
            self._stock_unanswered(command, f"check_stock refused: {reason}".rstrip())
            return
        short = shortfall(command.items, available)
        if not short:
            if self._arms_waiting(command):
                # 확인 중 모드가 바뀌면 팔이 새 모드를 보고할 때까지 다시 잡아 둔다 (PI-20)
                self._fill_or_hold(command)
                return
            self._send_fill(command)
            return
        text = ", ".join(f"{kind} {count}" for kind, count in sorted(short.items()))
        self._dispatch(EventOut(
            "WARN", FillBasket.Result.REASON_OUT_OF_STOCK,
            f"fill goal {command.goal_id} for {command.order_id} not sent, short: {text}",
            command.robot), self._now())
        # 보내지 않은 담기: placed는 비고 missing은 모자란 수, 다음 판단은 주문 코어
        self.core.on_fill_result(command.goal_id, False, {}, short,
                                 FillBasket.Result.REASON_OUT_OF_STOCK)

    def _stock_unanswered(self, command, why: str) -> None:
        """IR-40: 다시 보내지 않고 담기를 잡아 둔 채 알린다."""
        self._stock.hold(command)
        self._dispatch(EventOut(
            "WARN", "STOCK_CHECK_NO_ANSWER",
            f"fill goal {command.goal_id} for {command.order_id} waits: {why}",
            command.robot), self._now())

    def _step_stock_checks(self, now: float) -> None:
        for check in self._stock.expired(now):
            self._drop_stock_future(check.future)
            self._stock_unanswered(check.command, f"no stock answer within {STOCK_LIMIT_S:.0f} s")
        # 팔 ERROR · 초기화로 코어가 포기한 담기는 확인도 버린다
        for command, future in self._stock.given_up(self.core.arm.holds):
            self._drop_stock_future(future)
            self.get_logger().info(f"fill goal {command.goal_id} stock check dropped: "
                                   "the core gave the job up")

    def _drop_stock_future(self, future) -> None:
        if future is not None and not future.done():
            self._stock_client.remove_pending_request(future)

    def report_command(self, request_id: str, command: str, target: str, result: str,
                       reason: str = "", detail: str = "") -> None:
        """관리 명령 결과를 브리지에 보고한다(ReportCommand). 코어 그룹에서만 부른다."""
        self.command_reports.append((request_id, command, target, result, reason, detail))
        self.get_logger().info(
            f"command {command} {request_id} on {target}: {result} {reason} {detail}".rstrip())
        self._bridge.command(request_id, command, target, result, reason, detail)

    def _ask_clear(self, command) -> None:
        client = self._check_clear[command.robot]
        if not client.service_is_ready():
            self.core.on_clear_check(command.robot, command.check_id, NO_ANSWER)
            return
        try:
            future = client.call_async(adapters.check_clear_request(command))
        except Exception as exc:
            self.get_logger().warning(f"check_clear on {command.robot} was not sent: {exc}")
            self.core.on_clear_check(command.robot, command.check_id, NO_ANSWER)
            return
        future.add_done_callback(partial(self._post, self._clear_answered, command))

    def _clear_answered(self, command, future) -> None:
        try:
            answer = adapters.check_clear_answer(future.result())
        except Exception as exc:
            self.get_logger().warning(f"check_clear on {command.robot} got no reply: {exc}")
            answer = NO_ANSWER
        self.core.on_clear_check(command.robot, command.check_id, answer)

    def _log_event(self, event) -> None:
        text = f"{event.code} {event.detail}"
        if event.level == "ERROR":
            self.get_logger().error(text)
        elif event.level == "WARN":
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    # ---- 액션 goal 보내기와 feedback, 수락, 결과 받기 ----
    def _on_move_feedback(self, key, msg) -> None:
        self.core.on_move_feedback(key[0], key[1], *adapters.move_feedback(msg.feedback))

    def _on_fill_feedback(self, key, msg) -> None:
        fb = msg.feedback
        self.core.on_fill_feedback(key[1], fb.arm_id, fb.phase, int(fb.placed_total))

    def _send(self, client, goal, key, on_result, on_rejected, feedback=None) -> None:
        if not client.server_is_ready():
            if on_rejected == self._robot_rejected:
                self.core.on_goal_not_sent(key[0], key[1])
            else:
                on_rejected(key)
            return
        self._goals.sent(key)
        future = (client.send_goal_async(goal, feedback_callback=feedback) if feedback
                  else client.send_goal_async(goal))
        future.add_done_callback(
            partial(self._post, self._goal_response, key, on_result, on_rejected))

    def _goal_response(self, key, on_result, on_rejected, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"goal {key} failed to send: {exc}")
            self._goals.finished(key)
            on_rejected(key)
            return
        if not handle.accepted:
            self._goals.finished(key)
            on_rejected(key)
            return
        self._goals.accepted(key, handle)
        handle.get_result_async().add_done_callback(
            partial(self._post, self._goal_result, key, on_result, on_rejected))

    def _goal_result(self, key, on_result, on_rejected, future) -> None:
        try:
            result = adapters.goal_result(future.result())
        except Exception as exc:
            self.get_logger().error(f"goal {key} lost its result: {exc}")
            if on_rejected == self._robot_rejected and self._goals.canceling(key):
                self._goals.lost(key)
                return
            if on_rejected == self._robot_rejected:
                # 받아들인 로봇 goal의 결과를 잃음은 거절이 아니라 결과 모름(IR-62). 취소할 handle은 쥔다
                self._goals.result_lost(key)
                self.core.on_goal_lost(key[0], key[1])
                return
            self._goals.finished(key)
            on_rejected(key)
            return
        self._goals.finished(key)
        on_result(key, result)

    # ---- 액션 거절과 결과를 코어에 전달 ----
    def _robot_rejected(self, key) -> None:
        self.core.on_goal_rejected(key[0], key[1])

    def _fill_rejected(self, key) -> None:
        self.core.on_fill_result(key[1], False, {}, {}, "REJECTED")

    def _swap_rejected(self, key) -> None:
        self.core.on_swap_result(key[1], False, False, False, "REJECTED")

    def _move_done(self, key, result) -> None:
        self.core.on_move_result(key[0], key[1], *adapters.move_result(result))

    def _retreat_done(self, key, result) -> None:
        handler = getattr(self.core, "on_retreat_result", None)
        if handler is None:
            self.get_logger().warning(f"retreat {key} ended but the core has no "
                                      "on_retreat_result; result dropped")
            return
        handler(key[0], key[1], *adapters.retreat_result(result))

    def _dock_done(self, key, result) -> None:
        self.core.on_dock_result(key[0], key[1], *adapters.dock_result(result))

    def _fill_done(self, key, result) -> None:
        self.core.on_fill_result(key[1], *adapters.fill_result(result))

    def _swap_done(self, key, result) -> None:
        self.core.on_swap_result(key[1], *adapters.swap_result(result))


def _request_id(request) -> str:
    """IR-20: 번호가 없는 요청은 새 번호를 붙여 따로 다룬다 (응답에 실어 돌려준다)."""
    return request.request_id or str(uuid.uuid4())


def _joined(ends) -> tuple[str, str, str]:
    """팔마다 끝 상태를 하나로: 거절 > 시간 초과 > 받음."""
    for result in (REJECTED, TIMEOUT):
        bad = [(arm, end) for arm, end in sorted(ends.items()) if end[0] == result]
        if bad:
            return result, bad[0][1][1], "; ".join(f"{arm}: {end[2]}" for arm, end in bad)
    return ACCEPTED, "", ""


def _answer(response, request_id, reply):
    """관리 명령 공통 응답 (명세 4.4)."""
    response.accepted, response.reason = reply.accepted, reply.reason
    response.request_id, response.detail = str(request_id), reply.detail
    return response


# ---- 노드 실행 진입점 ----
def make_executor() -> MultiThreadedExecutor:
    """IR-30: 코어 그룹과 받은 편지함 그룹이 따로 돌도록 여러 스레드로 돈다."""
    return MultiThreadedExecutor(num_threads=4)


def main() -> None:
    parser = argparse.ArgumentParser(description="The FMS node.")
    parser.add_argument("--record", default=None, metavar="PATH",
                        help="keep a record for tools/replay.py, one JSON line per input")
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = FleetManagerNode(record=args.record)
    executor = make_executor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
