# ---- fleet_core 값과 ROS 메시지·액션·서비스를 서로 변환 ----
from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from fleet_core.common.commands import OrderRecord, ReportTripCmd
from fleet_interfaces.action import Dock, FillBasket, MoveTo, Retreat, SwapBasket
from fleet_interfaces.msg import (
    ArmState,
    Assignment,
    AssignmentArray,
    FleetEvent,
    OrderBoard,
    OrderItem,
    RobotStatus,
    RobotStatusArray,
    Segment,
    ZoneState,
    ZoneStateArray,
)
from fleet_interfaces.srv import (
    CheckClear,
    CheckStock,
    Hello,
    ReportCommand,
    ReportEvent,
    ReportOrder,
    ReportTrip,
    TripCommand,
)
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

# ---- 공통 상수와 시간·주문 품목 변환 ----
SOURCE = "fleet_manager"
REASON_OF_STATUS = {
    GoalStatus.STATUS_CANCELED: MoveTo.Result.REASON_CANCELED,
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def to_time(seconds: float) -> Time:
    sec, nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def to_order_items(items: dict[str, int]) -> list[OrderItem]:
    return [OrderItem(kind=kind, count=int(count)) for kind, count in items.items()]


def from_order_items(items) -> dict[str, int]:
    counts: dict[str, int] = {}
    for item in items:
        counts[item.kind] = counts.get(item.kind, 0) + int(item.count)
    return counts


# ---- 담기 전 결품 확인 (W-40) ----
def check_stock_request(cmd) -> CheckStock.Request:
    return CheckStock.Request(trip_id=cmd.order_id, items=to_order_items(cmd.items))


def check_stock_answer(response) -> tuple[bool, dict[str, int], str]:
    return bool(response.accepted), from_order_items(response.available), response.reason


# ---- 코어 상태와 이벤트를 ROS 메시지로 변환 ----
def zones_to_msg(views, now: float) -> ZoneStateArray:
    zones = []
    for view in views:
        zone = ZoneState(zone_id=view.zone_id, holder=view.holder or "", queue=list(view.queue))
        if view.acquired_at is not None:
            zone.acquired_at = to_time(view.acquired_at)
        zones.append(zone)
    return ZoneStateArray(zones=zones, stamp=to_time(now))


def assignments_to_msg(views, now: float) -> AssignmentArray:
    items = [
        Assignment(robot_id=v.robot_id, order_id=v.order_id or "",
                   station_id=v.station_id or "", task_type=v.task_type)
        for v in views
    ]
    return AssignmentArray(assignments=items, stamp=to_time(now))


def event_to_msg(event, now: float) -> FleetEvent:
    return FleetEvent(
        level=event.level,
        source=event.robot or SOURCE,
        code=event.code,
        detail=event.detail,
        stamp=to_time(now),
    )


def _pode_status(v, state) -> RobotStatus:
    row = RobotStatus(robot_id=v.robot_id, fsm_state=v.fsm_state, task_type=v.task_type,
                      order_id=v.order_id, fault=v.fault, fault_reason=v.fault_reason,
                      wait_zone=v.wait_zone, wait_robots=list(v.wait_robots),
                      base=list(v.base), horizon=list(v.horizon), priority=int(v.priority),
                      battery=float(v.battery), x=float(v.x), y=float(v.y), yaw=float(v.yaw),
                      kind=RobotStatus.KIND_POD, lock=True)
    if state is not None:
        row.lock, row.mode, row.forced, row.boot_id = (
            bool(state.lock), state.mode, list(state.forced), state.boot_id)
    return row


def arm_status(arm: str, state, fresh: bool, phase: str = "", trip: str = "") -> RobotStatus:
    """phase · trip은 코어의 담기 진행."""
    row = RobotStatus(robot_id=arm, kind=RobotStatus.KIND_ARM, battery=-1.0,
                      fault=RobotStatus.FAULT_COMM_LOST, lock=True, task_type=phase,
                      order_id=trip)
    if state is None:
        return row
    row.fsm_state, row.fault_reason = state.state, state.detail
    row.lock, row.mode, row.forced, row.boot_id = (
        bool(state.lock), state.mode, list(state.forced), state.boot_id)
    if fresh:
        row.fault = (RobotStatus.FAULT_ERROR if state.state == ArmState.STATE_ERROR
                     else RobotStatus.FAULT_NONE)
    return row


def robots_to_msg(views, now: float, pode_states=None, arms=()) -> RobotStatusArray:
    pode_states = pode_states or {}
    robots = [_pode_status(v, pode_states.get(v.robot_id)) for v in views]
    robots += [arm_status(*row) for row in arms]
    return RobotStatusArray(robots=robots, stamp=to_time(now))


# ---- 로봇이 보낸 상태 메시지에서 값 추출 ----
def robot_report(msg) -> tuple[float, float, float, str, float]:
    return msg.pose.x, msg.pose.y, msg.pose.theta, msg.state, msg.battery


def robot_report_id(msg) -> str:
    return msg.robot_id


def robot_obstacle(msg) -> tuple[float, float, str]:
    if not msg.obstacle_kind:
        return math.nan, math.nan, ""
    return float(msg.obstacle_x), float(msg.obstacle_y), msg.obstacle_kind


def robot_local_yield(msg) -> str:
    return msg.local_yield


# ---- CheckClear 요청과 답, 액션 결과와 feedback 변환 ----
def check_clear_request(command):
    return CheckClear.Request(x=float(command.x), y=float(command.y), r=float(command.radius_m))


def check_clear_answer(response) -> str:
    return response.answer


def goal_result(response):
    result = response.result
    if getattr(result, "reason", None) == "":
        result.reason = REASON_OF_STATUS.get(response.status, "")
    return result


def move_result(result) -> tuple[bool, str, int]:
    return result.success, result.reason, int(result.last_done_seq_id)


def move_feedback(feedback) -> tuple[int, bool, str, float]:
    return (int(feedback.last_done_seq_id), bool(feedback.waiting_at_base_end),
            feedback.horizon_warning, float(feedback.distance_remaining))


def retreat_result(result) -> tuple[bool, str]:
    return result.success, result.reason


def dock_result(result) -> tuple[bool, float, float, float]:
    return result.success, result.error_x_cm, result.error_y_cm, result.error_yaw_deg


def fill_result(result) -> tuple[bool, dict[str, int], dict[str, int], str, dict[str, int],
                                 dict[str, int]]:
    return (result.success, from_order_items(result.placed), from_order_items(result.missing),
            result.reason, from_order_items(result.placed_confirmed),
            from_order_items(result.uncertain))


def swap_result(result) -> tuple[bool, bool, bool, str]:
    return result.success, result.took_full, result.put_empty_done, result.reason


# ---- 서비스 요청과 응답 변환 ----
def confirm_clear_args(request) -> tuple[str, list[str], str]:
    return request.robot_id, list(request.zone_ids), request.note


def confirm_clear_response(response, released: list[str] | None, reason: str = "",
                           request_id: str = ""):
    response.accepted = released is not None
    response.released_zones = list(released or [])
    response.reason = "" if released is not None else str(reason)
    response.request_id = str(request_id)
    return response


def reinstate_args(request) -> tuple[str, str, str]:
    return request.robot_id, request.basket, request.note


def reinstate_response(response, accepted: bool, reason: str = "", request_id: str = ""):
    response.accepted = bool(accepted)
    response.reason = "" if accepted else str(reason)
    response.request_id = str(request_id)
    return response


def force_ship_args(request: TripCommand.Request) -> str:
    return request.trip_id


def force_ship_response(response, accepted: bool, reason: str = "", request_id: str = ""):
    response.accepted = bool(accepted)
    response.reason = "" if accepted else str(reason)
    response.request_id = str(request_id)
    return response


# ---- 주문 보드 메시지 변환 ----
def order_board_mode(msg: OrderBoard) -> str:
    return msg.run_mode


def _board_items(items) -> dict[str, int]:
    counts = from_order_items(items)
    for item in items:
        if int(item.count) <= 0:
            counts[item.kind] = min(int(item.count), counts[item.kind])
    return counts


def order_board_orders(msg: OrderBoard) -> list[OrderRecord]:
    return [OrderRecord(o.order_id, _board_items(o.items)) for o in msg.orders]


# ---- 코어 명령을 액션 goal로 변환 ----
def _poses(path) -> list[Pose]:
    poses = []
    for x, y, yaw in path:
        pose = Pose()
        pose.position.x, pose.position.y = float(x), float(y)
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        poses.append(pose)
    return poses


def move_goal(cmd) -> MoveTo.Goal:
    goal = MoveTo.Goal(target_id=cmd.target_id, final_goal=bool(cmd.final_goal))
    goal.target_pose.x = float(cmd.x)
    goal.target_pose.y = float(cmd.y)
    goal.target_pose.theta = float(cmd.yaw)
    goal.path = _poses(cmd.path)
    goal.mode = cmd.mode
    goal.priority = int(cmd.priority)
    return goal


def _stamped(x, y, yaw) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose = _poses([(x, y, yaw)])[0]
    return pose


def _segment(seg) -> Segment:
    msg = Segment(seq_id=int(seg.seq_id), mode=seg.mode, controller_id=seg.controller_id,
                  end_node_id=seg.end_node, zone_ids=list(seg.zones))
    if seg.path:
        msg.path = Path(poses=[_stamped(*point) for point in seg.path])
        msg.path.header.frame_id = "map"
    if seg.mode == "nav_goal":
        msg.target_pose = _stamped(seg.x, seg.y, seg.yaw)
    return msg


def move_order_goal(cmd) -> MoveTo.Goal:
    goal = MoveTo.Goal(order_id=cmd.order_id, order_update_id=int(cmd.update_id),
                       stitch_node_id=cmd.stitch_node, base=[_segment(s) for s in cmd.base],
                       horizon=[_segment(s) for s in cmd.horizon],
                       priority=int(cmd.priority))
    if cmd.base:
        last = cmd.base[-1]
        goal.target_id = last.end_node
        end = (last.x, last.y, last.yaw) if last.mode == "nav_goal" else (
            tuple(last.path[-1]) if last.path else None)
        if end is not None:
            goal.target_pose.x, goal.target_pose.y, goal.target_pose.theta = map(float, end)
    return goal


def retreat_goal(cmd) -> Retreat.Goal:
    return Retreat.Goal(distance_m=float(cmd.distance_m), speed_mps=float(cmd.speed_mps),
                        path=_poses(cmd.path))


def dock_goal(cmd) -> Dock.Goal:
    return Dock.Goal(dock_id=cmd.dock_id)


def fill_goal(cmd) -> FillBasket.Goal:
    return FillBasket.Goal(order_id=cmd.order_id, robot_id=cmd.robot, station_id=cmd.station_id,
                           items=to_order_items(cmd.items))


def swap_goal(cmd) -> SwapBasket.Goal:
    return SwapBasket.Goal(order_id=cmd.order_id, robot_id=cmd.robot, station_id=cmd.station_id,
                           take_full=bool(cmd.take_full), put_empty=bool(cmd.put_empty))


# ---- 주문 보고 서비스 변환 ----
def report_accepted(response) -> bool:
    return response.accepted


def report_request(cmd) -> ReportOrder.Request:
    return ReportOrder.Request(
        order_id=cmd.order_id,
        robot_id=cmd.robot,
        status=cmd.status,
        hold_reason=cmd.hold_reason,
        placed=to_order_items(cmd.placed),
        missing=to_order_items(cmd.missing),
        detail=cmd.detail,
    )


# ---- 운반 건 보고(ReportTrip)의 상태와 사유: 밖으로는 CANCELLED ----
NOTHING_TO_FILL = "NOTHING_TO_FILL"


def trip_status(status: str, hold_reason: str = "") -> tuple[str, str]:
    if status == ReportOrder.Request.STATUS_CANCELED:
        return ReportTrip.Request.STATUS_CANCELLED, NOTHING_TO_FILL
    if status == ReportOrder.Request.STATUS_HELD:
        return ReportTrip.Request.STATUS_HELD, hold_reason
    return status, ""


# ---- 브리지 보고 요청: 번호(epoch, n)는 보낼 때 노드가 채운다 ----
BRIDGE_SOURCE = "fleet"  # ReportEvent.source, Hello.node
DELIVERED_RESULTS = ("DUPLICATE", "LATE_REPORT")


def trip_report_request(cmd, stamp_s: float) -> ReportTrip.Request:
    # 진행 단계(ReportTripCmd)는 그대로, 끝 상태(ReportOrderCmd)는 CANCELLED 철자와 사유로 바꾼다
    if isinstance(cmd, ReportTripCmd):
        trip_id, status, reason = cmd.trip_id, cmd.status, cmd.reason
    else:
        trip_id = cmd.order_id
        status, reason = trip_status(cmd.status, cmd.hold_reason)
    return ReportTrip.Request(trip_id=trip_id, status=status, reason=reason, robot_id=cmd.robot,
                              placed=to_order_items(cmd.placed),
                              missing=to_order_items(cmd.missing), stamp=to_time(stamp_s))


def trip_report_delivered(response) -> tuple[bool, bool]:
    # (전달됨, 늦은 보고). DUPLICATE · LATE_REPORT도 전달된 것이다
    late = response.result == "LATE_REPORT"
    return bool(response.accepted) or response.result in DELIVERED_RESULTS, late


def report_event_request(event, event_id: str, stamp_s: float, robots) -> ReportEvent.Request:
    robot = event.robot if event.robot in robots else ""
    return ReportEvent.Request(event_id=event_id, level=event.level,
                               source=event.robot or BRIDGE_SOURCE, code=event.code,
                               robot_id=robot, trip_id="", detail=event.detail,
                               stamp=to_time(stamp_s))


def report_command_request(request_id: str, command: str, target: str, result: str,
                           reason: str, detail: str, stamp_s: float) -> ReportCommand.Request:
    return ReportCommand.Request(request_id=request_id, command=command, target=target,
                                 result=result, reason=reason, detail=detail,
                                 stamp=to_time(stamp_s))


def hello_request(boot_id: str) -> Hello.Request:
    return Hello.Request(node=BRIDGE_SOURCE, boot_id=boot_id)


def open_trips(response) -> list[dict]:
    # TODO(orders): 주문 층이 받을 타입을 정하면 바꾼다. 지금은 값만 옮긴다
    return [{"trip_id": t.trip_id, "order_id": t.order_id, "trip_index": int(t.trip_index),
             "trip_count": int(t.trip_count), "items": from_order_items(t.items),
             "status": t.status, "reason": t.reason, "robot_id": t.robot_id,
             "placed": from_order_items(t.placed), "missing": from_order_items(t.missing),
             "report_epoch": int(t.report_epoch), "report_n": int(t.report_n)}
            for t in response.trips]
