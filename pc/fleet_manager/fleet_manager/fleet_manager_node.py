# ---- fleet_core의 FleetManager를 ROS 노드로 감싼다 ----
from __future__ import annotations

import argparse
import time
from functools import partial

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
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.traffic.site import load_site
from fleet_interfaces.action import Dock, FillBasket, MoveTo, Retreat, SwapBasket
from fleet_interfaces.msg import (
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    RobotState,
    RobotStatusArray,
    ZoneStateArray,
)
from fleet_interfaces.srv import CheckClear, ConfirmClear, Reinstate, ReportOrder
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Header

from fleet_manager import adapters
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox
from fleet_manager.order_goals import FeedbackFilter, apply_overrides


# ---- FleetManagerNode 생성과 인터페이스 설정 ----
class FleetManagerNode(Node):
    def __init__(self, record=None):
        super().__init__("fleet_manager")
        self.declare_parameter("robots", ["pinky1", "pinky2", "pinky3"])
        self.declare_parameter("contract_file", "")
        self.declare_parameter("map_file", "")
        self.declare_parameter("params_file", "")
        self.declare_parameter("tick_hz", 10.0)
        unset = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("use_order_model", None, unset)
        self.declare_parameter("lookahead_s", None, unset)
        robots = list(self.get_parameter("robots").value)

        contract_file = self.get_parameter("contract_file").value or (
            get_package_share_directory("fleet_interfaces") + "/contract.yaml")
        self.contract = load_contract(contract_file)
        self.site = load_site(self.get_parameter("map_file").value or None)
        params = apply_overrides(
            load_params(self.get_parameter("params_file").value or None),
            self.get_parameter("use_order_model").value,
            self.get_parameter("lookahead_s").value)
        self.core = FleetManager(self.site, params, robots, self._now)
        self._record_file = None
        if record:
            self.core.check_recordable()
            self._record_file = open(record, "w", buffering=1, encoding="utf-8")
            self.core.record_to(lambda line: self._record_file.write(line + "\n"))

        self._goals = GoalTracker()
        self._reports = ReportOutbox()
        self._order_feedback = FeedbackFilter()
        self._reached = set()
        self._last_zones = None
        self._last_assignments = None
        self._create_interfaces(robots)
        self.create_timer(1.0 / self.get_parameter("tick_hz").value, self._on_tick)
        self.create_timer(1.0, self._publish_robots)
        self._silent_until = (time.monotonic() + params.heartbeat_timeout_s
                              + params.heartbeat_period_s)
        self.create_timer(params.heartbeat_period_s, self._publish_heartbeat,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

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
        self.create_subscription(
            OrderBoard, name("order_board"), self._on_order_board, self._qos("order_board"))
        self._move = {}
        self._dock = {}
        self._retreat = {}
        self._check_clear = {}
        for robot in robots:
            self.create_subscription(
                RobotState, name("robot_state", pinky=robot),
                partial(self._on_robot_state, robot), self._qos("robot_state"))
            self._move[robot] = ActionClient(self, MoveTo, name("move_to", pinky=robot))
            self._dock[robot] = ActionClient(self, Dock, name("dock", pinky=robot))
            self._retreat[robot] = ActionClient(self, Retreat, name("retreat", pinky=robot))
            self._check_clear[robot] = self.create_client(CheckClear,
                                                          name("check_clear", pinky=robot))
        self._fill = {}
        self._swap = {}
        for station in self.site.stations.values():
            if station.action == "fill":
                self._fill[station.id] = ActionClient(self, FillBasket, name("fill_basket"))
            else:
                self._swap[station.id] = ActionClient(
                    self, SwapBasket, name("swap_basket", arm=station.server))
        self._report_client = self.create_client(ReportOrder, name("report_order"))
        self.create_service(ConfirmClear, name("confirm_clear"), self._on_confirm_clear)
        self.create_service(Reinstate, name("reinstate"), self._on_reinstate)

    # ---- 토픽과 서비스 수신 콜백 ----
    def _on_robot_state(self, robot, msg: RobotState) -> None:
        if not self.silent() and self._reachable(robot):
            self.core.on_robot_state(robot, *adapters.robot_report(msg),
                                     adapters.robot_report_id(msg),
                                     *adapters.robot_obstacle(msg),
                                     adapters.robot_local_yield(msg))

    def _reachable(self, robot) -> bool:
        if (robot not in self._reached and self._move[robot].server_is_ready()
                and self._dock[robot].server_is_ready()):
            self._reached.add(robot)
        return robot in self._reached

    def _on_order_board(self, msg: OrderBoard) -> None:
        self.core.on_order_board(adapters.order_board_mode(msg), adapters.order_board_orders(msg))

    def _on_confirm_clear(self, request, response):
        released = self.core.confirm_clear(*adapters.confirm_clear_args(request))
        return adapters.confirm_clear_response(response, released, self.core.last_clear_reason)

    def _on_reinstate(self, request, response):
        accepted = self.core.reinstate(*adapters.reinstate_args(request))
        return adapters.reinstate_response(response, accepted, self.core.last_reinstate_reason)

    # ---- 주기 틱: 명령 내보내기, 취소 재시도, 주문 보고, 상태 발행 ----
    def _on_tick(self) -> None:
        now = self._now()
        if now <= 0.0 or self.silent():
            return
        for command in self.core.tick():
            self._dispatch(command, now)
        self._retry_cancels(now)
        self._flush_reports(now)
        self._publish_snapshots(now)

    def _retry_cancels(self, now: float) -> None:
        params = self.core.params
        for server, goal_id in self._goals.retry_cancels(now, params.cancel_retry_s,
                                                         params.cancel_warn_after):
            waited = params.cancel_warn_after * params.cancel_retry_s
            detail = (f"goal {goal_id} on {server} has no result {waited:.0f} s after it was "
                      "canceled; the cancel stays open while a result can still come")
            self._dispatch(EventOut("WARN", "CANCEL_UNCONFIRMED", detail, server), now)
        for robot, goal_id in self._goals.take_dropped():
            self.core.on_goal_lost(robot, goal_id)

    def _flush_reports(self, now: float) -> None:
        for report_id, command in self._reports.due(now, self._report_client.service_is_ready(),
                                                    self.core.params.report_retry_s):
            future = self._report_client.call_async(adapters.report_request(command))
            future.add_done_callback(partial(self._report_answered, report_id, command))

    def _report_answered(self, report_id, command, future) -> None:
        try:
            accepted = adapters.report_accepted(future.result())
        except Exception as exc:
            self.get_logger().warning(f"report of {command.order_id} got no reply: {exc}")
            return
        if self._reports.done(report_id) and not accepted:
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

    def _publish_heartbeat(self) -> None:
        if not self.silent() and self.core.traffic.everyone_placed:
            self._heartbeat_pub.publish(Header(stamp=self.get_clock().now().to_msg(),
                                               frame_id=self.get_name()))

    def _publish_robots(self) -> None:
        now = self._now()
        if now > 0.0:
            self._robots_pub.publish(adapters.robots_to_msg(self.core.robots_view(), now))

    # ---- 코어 명령을 액션, 서비스, 로그로 내보냄 ----
    def _dispatch(self, command, now: float) -> None:
        if isinstance(command, MoveCmd):
            self._send(self._move[command.robot], adapters.move_goal(command),
                       (command.robot, command.goal_id), self._move_done, self._robot_rejected)
        elif isinstance(command, MoveOrderCmd):
            key = (command.robot, command.goal_id)
            self._send(self._move[command.robot], adapters.move_order_goal(command), key,
                       self._move_done, self._robot_rejected,
                       feedback=partial(self._on_move_feedback, key))
        elif isinstance(command, DockCmd):
            self._send(self._dock[command.robot], adapters.dock_goal(command),
                       (command.robot, command.goal_id), self._dock_done, self._robot_rejected)
        elif isinstance(command, RetreatCmd):
            self._send(self._retreat[command.robot], adapters.retreat_goal(command),
                       (command.robot, command.goal_id), self._retreat_done,
                       self._robot_rejected)
        elif isinstance(command, FillCmd):
            server = self.site.stations[command.station_id].server
            self._send(self._fill[command.station_id], adapters.fill_goal(command),
                       (server, command.goal_id), self._fill_done, self._fill_rejected)
        elif isinstance(command, SwapCmd):
            self._send(self._swap[command.station_id], adapters.swap_goal(command),
                       (command.arm, command.goal_id), self._swap_done, self._swap_rejected)
        elif isinstance(command, CheckClearCmd):
            self._ask_clear(command)
        elif isinstance(command, CancelCmd):
            self._goals.cancel((command.robot, command.goal_id), now)
        elif isinstance(command, ReportOrderCmd):
            self._reports.add(command)
        elif isinstance(command, EventOut):
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
        future.add_done_callback(partial(self._clear_answered, command))

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
        values = adapters.move_feedback(msg.feedback)
        if self._order_feedback.changed(key, values):
            self.core.on_move_feedback(key[0], key[1], *values)

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
        future.add_done_callback(partial(self._goal_response, key, on_result, on_rejected))

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
            partial(self._goal_result, key, on_result, on_rejected))

    def _goal_result(self, key, on_result, on_rejected, future) -> None:
        self._order_feedback.forget(key)
        try:
            result = adapters.goal_result(future.result())
        except Exception as exc:
            self.get_logger().error(f"goal {key} lost its result: {exc}")
            if on_rejected == self._robot_rejected and self._goals.canceling(key):
                self._goals.lost(key)
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
def main() -> None:
    parser = argparse.ArgumentParser(description="The FMS node.")
    parser.add_argument("--record", default=None, metavar="PATH",
                        help="keep a record for tools/replay.py, one JSON line per input")
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = FleetManagerNode(record=args.record)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
