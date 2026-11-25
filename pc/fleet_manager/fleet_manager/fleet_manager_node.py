# ---- fleet_core의 FleetManager를 ROS 노드로 감싼다 ----
from __future__ import annotations

import argparse
import math
import threading
import time
from functools import partial, wraps

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.common.commands import (
    NO_ANSWER,
    CancelCmd,
    CheckClearCmd,
    DockCmd,
    EventOut,
    FillCmd,
    MoveCmd,
    MoveOrderCmd,
    ReportOrderCmd,
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
    ConfirmClear,
    Reinstate,
    ReportOrder,
    SetMode,
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
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox
from fleet_manager.inbox import Inbox
from fleet_manager.mode_sync import ModeSync
from fleet_manager.order_goals import apply_overrides


SLOW_TICK_WARN_S = 5.0  # 느린 틱 경고 사이 최소 간격


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
                ArmState, name("arm_state", arm=arm), partial(self._post, self._on_arm_state, arm),
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
        # 운영 모드: 브리지가 내면 로봇마다 set_mode, 받기 전에는 아무것도 하지 않는다 (IR-73)
        arms = list(self.contract.placeholders.get("arm", []))
        self._mode_sync = ModeSync(robots, arms)
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
        self.create_service(TripCommand, name("force_ship"), self._on_force_ship,
                            callback_group=core)

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
        if not self.silent() and self._reachable(robot):
            self.core.on_robot_state(robot, *adapters.robot_report(msg),
                                     adapters.robot_report_id(msg),
                                     *adapters.robot_obstacle(msg),
                                     adapters.robot_local_yield(msg))

    def _on_arm_state(self, arm, msg: ArmState) -> None:
        self._arm_states[arm] = (msg, time.monotonic())
        self._mode_sync.on_state(arm, msg.mode, msg.boot_id, time.monotonic())

    def _on_system_mode(self, msg: SystemMode) -> None:
        if self._mode_sync.on_system_mode(msg.mode):
            self.get_logger().info(f"system mode {msg.mode}, telling the robots")

    def _arm_rows(self) -> list:
        rows = []
        for arm in self.contract.placeholders.get("arm", []):
            msg, at = self._arm_states.get(arm, (None, None))
            fresh = at is not None and time.monotonic() - at <= self.core.params.comm_timeout_s
            rows.append((arm, msg, fresh))
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
        released = self.core.confirm_clear(*adapters.confirm_clear_args(request))
        return adapters.confirm_clear_response(response, released, self.core.last_clear_reason,
                                               request.request_id)

    @_core_callback
    def _on_reinstate(self, request, response):
        self._drain()
        accepted = self.core.reinstate(*adapters.reinstate_args(request))
        return adapters.reinstate_response(response, accepted, self.core.last_reinstate_reason,
                                           request.request_id)

    @_core_callback
    def _on_force_ship(self, request, response):
        self._drain()
        accepted, reason = self.core.force_ship(adapters.force_ship_args(request))
        return adapters.force_ship_response(response, accepted, reason, request.request_id)

    # ---- 주기 틱: 받은 사건 처리, 명령 내보내기, 취소 재시도, 주문 보고, 상태 발행 ----
    @_core_callback
    def _on_tick(self) -> None:
        started = time.monotonic()
        waiting = self._inbox.qsize()
        self._drain()
        now = self._now()
        if now > 0.0 and not self.silent():
            self._sync_modes(time.monotonic())
            for command in self.core.tick():
                self._dispatch(command, now)
            self._retry_cancels(now)
            self._flush_reports(now)
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
                server = self.site.stations[command.station_id].server
                self._send(self._fill[command.station_id], adapters.fill_goal(command),
                           (server, command.goal_id), self._fill_done, self._fill_rejected)
            case SwapCmd():
                self._send(self._swap[command.station_id], adapters.swap_goal(command),
                           (command.arm, command.goal_id), self._swap_done,
                           self._swap_rejected)
            case CheckClearCmd():
                self._ask_clear(command)
            case CancelCmd():
                self._goals.cancel((command.robot, command.goal_id), now)
            case ReportOrderCmd():
                self._reports.add(command)
            case EventOut():
                self._events_pub.publish(adapters.event_to_msg(command, now))
                self._log_event(command)

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
