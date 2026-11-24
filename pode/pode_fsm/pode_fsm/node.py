# ---- Pode 상태 기계 ROS 노드 ----
from __future__ import annotations

import dataclasses
import math
import threading
import time
import uuid

import rclpy
from fleet_interfaces.msg import RobotState, RobotStatusArray
from fleet_interfaces.srv import CheckClear, RobotCommand, SetLock, SetMode
from geometry_msgs.msg import Pose2D
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from pode_fsm.admin import AdminState
from pode_fsm.backup import BackUp, CancelBackUp, reason_of
from pode_fsm.backup_ros import BackUpRunner
from pode_fsm.commands import CancelNav, ClearCostmap, LimitSpeed, SendPath
from pode_fsm.contract import default_contract_path, load_contract, to_ros_qos
from pode_fsm.costmap import Grid, obstacle_on, to_frame
from pode_fsm.follow_ros import FollowPathRunner
from pode_fsm.geometry import pose_here, quaternion_yaw
from pode_fsm.local_yield import peers_from
from pode_fsm.no_motion import (
    BACKUP,
    DOCK,
    FOLLOW,
    BackUpStub,
    FollowStub,
    NavGoalStub,
    NoMotion,
    NoMotionParams,
)
from pode_fsm.nav_goal import CancelNavGoal, SendNavGoal
from pode_fsm.nav_goal_ros import NavGoalRunner
from pode_fsm.obstacle_report import CLEAR, UNKNOWN, clear_answer, nearest_ahead
from pode_fsm.order_fsm import OrderFsm
from pode_fsm.order_ros import OrderMaps, OrderMarkers
from pode_fsm.params_ros import declare_fsm_params
from pode_fsm.results import RECOVERING, Finish
from pode_fsm.server_ros import GoalServers

# ---- 배터리 미상 값과 마커 발행 주기 ----
UNKNOWN_BATTERY = -1.0
MARKER_PERIOD_S = 0.5


# ---- Pode 상태 기계 노드 PodeFsmNode ----
class PodeFsmNode(Node):
    def __init__(self, **kwargs):
        super().__init__("pode_fsm", **kwargs)
        # ---- 노드 파라미터 선언 ----
        declare = self.declare_parameter
        # TODO(no-motion): 실주행으로 돌아가면 no_motion:=false(기본)
        no_motion = bool(declare("no_motion", False).value)
        nm_defaults = NoMotionParams()
        declare("no_motion_move_s", nm_defaults.move_s)
        declare("no_motion_retreat_s", nm_defaults.retreat_s)
        declare("no_motion_dock_s", nm_defaults.dock_s)
        declare("no_motion_dock_ids", list(nm_defaults.dock_ids))
        declare("initial_x", 0.12)
        declare("initial_y", 0.10)
        declare("initial_yaw", 1.5708)
        declare("robot_id", self.get_namespace().strip("/") or "pode1")
        declare("contract_file", "")
        declare("map_frame", "map")
        declare("base_frame", "base_footprint")
        declare("follow_path_action", "follow_path")
        declare("backup_action", "backup")
        declare("navigate_to_pose_action", "navigate_to_pose")
        declare("clear_costmap_service", "local_costmap/clear_entirely_local_costmap")
        declare("controller_id", "FollowPath")
        declare("through_controller_id", "")
        declare("reverse_controller_id", "FollowPathReverse")
        declare("goal_checker_id", "general_goal_checker")
        declare("reverse_goal_checker_id", "retreat_goal_checker")
        declare("free_heading_goal_checker_id", "free_heading_goal_checker")
        declare("progress_checker_id", "progress_checker")
        declare("odom_topic", "odom")
        declare("costmap_topic", "local_costmap/costmap")
        declare("speed_limit_topic", "speed_limit")
        declare("battery_topic", "")
        declare("state_rate_hz", 5.0)
        declare("control_rate_hz", 20.0)
        declare("pose_timeout_s", 1.0)
        declare("obstacle_look_m", 0.30)
        declare("map_topic", "map")
        declare("keepout_mask_topic", "keepout_filter_mask")
        declare("order_markers_topic", "order_markers")
        fsm_params, order_params = declare_fsm_params(self)
        if no_motion:
            # 바퀴가 없으니 로봇단 교착 처리(후진 양보)는 끈다
            fsm_params = dataclasses.replace(fsm_params, local_yield=False)
        # ---- 파라미터 읽기와 상태 기계, 주문 지도, 마커 만들기 ----
        get = self.get_parameter
        self.robot_id = get("robot_id").value
        self._map_frame = get("map_frame").value
        self._base_frame = get("base_frame").value
        self._pose_timeout = float(get("pose_timeout_s").value)
        self._look_m = float(get("obstacle_look_m").value)
        self._nav_ids = (get("controller_id").value, get("goal_checker_id").value,
                         get("progress_checker_id").value)
        self._through_id = get("through_controller_id").value
        self._reverse_id = (get("reverse_controller_id").value,
                            get("reverse_goal_checker_id").value)
        group = ReentrantCallbackGroup()
        self._maps = OrderMaps(self, get("map_topic").value, get("keepout_mask_topic").value,
                               callback_group=group)
        self._fsm = OrderFsm(
            fsm_params, order_params,
            horizon_check=self._maps.check, robot_id=self.robot_id)
        self._nav_ready = False
        self._fsm.set_nav_ready(False)
        self._markers = OrderMarkers(self, get("order_markers_topic").value, self._map_frame)
        self._last_state = ("", "")
        contract = load_contract(get("contract_file").value or default_contract_path())

        # ---- 공유 상태 초기화 ----
        self._lock = threading.RLock()
        self._alive = True
        self._late: list[tuple[int, str]] = []
        self._late_backup: list[tuple[int, str]] = []
        self._late_goal: list[int] = []
        self._speed = (0.0, 0.0)
        self._costmap: tuple[Grid, str, float] | None = None
        self._battery = UNKNOWN_BATTERY
        self._admin = AdminState(self.robot_id, str(uuid.uuid4()))

        # ---- TF, 발행자, 구독자 만들기 ----
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._speed_pub = self.create_publisher(SpeedLimit, get("speed_limit_topic").value, 10)
        self._state_pub = self.create_publisher(
            RobotState, contract.name("robot_state", pode=self.robot_id),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(Header, contract.name("heartbeat"), self._on_heartbeat,
                                 to_ros_qos(contract.qos("heartbeat")), callback_group=group)
        self.create_subscription(RobotStatusArray, contract.name("robots"), self._on_fleet_robots,
                                 to_ros_qos(contract.qos("robots")), callback_group=group)
        self.create_subscription(Odometry, get("odom_topic").value, self._on_odom,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            OccupancyGrid, get("costmap_topic").value, self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), callback_group=group)
        if get("battery_topic").value:
            self.create_subscription(BatteryState, get("battery_topic").value,
                                     self._on_battery, qos_profile_sensor_data)
        # ---- Nav2 액션 실행기(동작 없음이면 대역), 서비스, 액션 서버 만들기 ----
        self._nm: NoMotion | None = None
        if no_motion:
            self._start_no_motion()
        else:
            self._start_runners(group)
        self._clear = self.create_client(ClearEntireCostmap, get("clear_costmap_service").value,
                                         callback_group=group)
        self.create_service(CheckClear, contract.name("check_clear", pode=self.robot_id),
                            self._on_check_clear, callback_group=group)
        self.create_service(SetLock, contract.name("robot_set_lock", robot=self.robot_id),
                            self._on_set_lock, callback_group=group)
        self.create_service(SetMode, contract.name("robot_set_mode", robot=self.robot_id),
                            self._on_set_mode, callback_group=group)
        self.create_service(RobotCommand, contract.name("pode_charge_end", pode=self.robot_id),
                            self._on_charge_end, callback_group=group)
        self._servers = GoalServers(self, contract, self.robot_id, group, self._lock, self._fsm,
                                    self._pose, self._run, lambda: self._alive, self._nm)
        # ---- 제어, 상태, 마커 타이머 시작 ----
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(get("control_rate_hz").value), self._tick,
                          callback_group=group, clock=steady)
        self.create_timer(1.0 / float(get("state_rate_hz").value), self._publish_state,
                          callback_group=group, clock=steady)
        self.create_timer(MARKER_PERIOD_S, self._publish_markers, callback_group=group,
                          clock=steady)
        mode = " (NO MOTION: goals succeed after a wait, nothing drives)" if no_motion else ""
        self.get_logger().info(f"pode_fsm for {self.robot_id} ready{mode}")

    def _start_runners(self, group) -> None:
        get = self.get_parameter
        self._follow = FollowPathRunner(
            self, get("follow_path_action").value, self._map_frame, self._nav_ids,
            self._through_id, self._reverse_id, get("free_heading_goal_checker_id").value,
            self._on_follow_rejected,
            self._on_follow_result, self._on_follow_cancel, callback_group=group)
        self._backup = BackUpRunner(self, get("backup_action").value, self._on_backup_result,
                                    callback_group=group)
        self._goals = NavGoalRunner(self, get("navigate_to_pose_action").value, self._map_frame,
                                    self._on_goal_feedback, self._on_goal_result,
                                    self._on_goal_cancel, callback_group=group)

    # ---- 동작 없음 대역 만들기 ----
    # TODO(no-motion): 실주행으로 돌아가면 지운다
    def _start_no_motion(self) -> None:
        get = self.get_parameter
        start = (get("initial_x").value, get("initial_y").value, get("initial_yaw").value)
        self._nm = NoMotion(start, NoMotionParams(
            move_s=float(get("no_motion_move_s").value),
            retreat_s=float(get("no_motion_retreat_s").value),
            dock_s=float(get("no_motion_dock_s").value),
            dock_ids=tuple(str(d) for d in get("no_motion_dock_ids").value)))
        self._follow = FollowStub(self._nm, time.monotonic)
        self._backup = BackUpStub(self._nm, time.monotonic)
        self._goals = NavGoalStub(self._nm, time.monotonic)
        # 대역은 처음부터 준비됨: 첫 tick 전에 온 goal도 NAV_UNAVAILABLE로 끝나지 않게
        self._note_nav_ready()

    # ---- 때가 된 대역 일을 Nav2 결과처럼 FSM에 알린다 ----
    def _step_no_motion(self, now: float) -> None:
        for job in self._nm.step(now):
            if job.kind == DOCK:
                if not job.canceled:
                    self._run(self._fsm.dock_done(job.key))
            elif job.kind == FOLLOW:
                if job.canceled:
                    self._on_follow_cancel(job.key)
                self._on_follow_result(job.key, "canceled" if job.canceled else "succeeded",
                                       0, False)
            elif job.kind == BACKUP:
                self._on_backup_result(job.key, "canceled" if job.canceled else "succeeded",
                                       0, 0.0)
            else:
                if job.canceled:
                    self._on_goal_cancel(job.key)
                self._on_goal_result(job.key, "canceled" if job.canceled else "succeeded",
                                     False)

    # ---- 노드 정지와 정리 ----
    def stop(self) -> None:
        with self._lock:
            self._alive = False
            self._follow.cancel_all()
            self._backup.cancel_all()
            self._goals.cancel_all()

    def destroy_node(self):
        self.stop()
        self._servers.destroy()
        self._follow.destroy()
        self._backup.destroy()
        self._goals.destroy()
        return super().destroy_node()

    # ---- 하트비트, 상대 로봇, 속도, 코스트맵, 배터리 구독 콜백 ----
    def _on_heartbeat(self, _msg) -> None:
        with self._lock:
            self._fsm.heartbeat(time.monotonic())

    def _on_fleet_robots(self, msg: RobotStatusArray) -> None:
        peers = peers_from(msg.robots)
        with self._lock:
            self._fsm.see_peers(time.monotonic(), peers)

    def _on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self._speed = (math.hypot(t.linear.x, t.linear.y), t.angular.z)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        o = msg.info.origin
        grid = Grid(msg.info.resolution, o.position.x, o.position.y,
                    quaternion_yaw(o.orientation.x, o.orientation.y, o.orientation.z,
                                   o.orientation.w),
                    msg.info.width, msg.info.height, msg.data)
        self._costmap = (grid, msg.header.frame_id, time.monotonic())

    def _on_battery(self, msg: BatteryState) -> None:
        if not math.isnan(msg.percentage):
            self._battery = float(msg.percentage) * 100.0

    # ---- TF로 로봇 위치와 코스트맵 변환 얻기 ----
    def _pose(self):
        if self._nm is not None:
            x, y, yaw = self._nm.pose
            return (x, y, yaw, self.get_clock().now().to_msg())
        try:
            tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(tf.header.stamp)
        if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > self._pose_timeout:
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w), tf.header.stamp)

    def _costmap_tf(self, frame):
        if not frame or frame == self._map_frame:
            return (0.0, 0.0, 0.0)
        try:
            tf = self._tf.lookup_transform(frame, self._map_frame, Time())
        except TransformException:
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w))

    # ---- 앞 장애물 위치 찾기와 CheckClear 서비스 응답 ----
    def _obstacle_ahead(self, pose):
        costmap = self._costmap
        if costmap is None or pose is None:
            return None
        grid, frame, _ = costmap
        tf = self._costmap_tf(frame)
        if tf is None:
            return None
        [(x, y)] = to_frame([pose[:2]], tf)
        found = nearest_ahead(grid, x, y, pose[2] + tf[2], self._look_m)
        if found is None:
            return None
        dx, dy = found[0] - tf[0], found[1] - tf[1]
        c, s = math.cos(-tf[2]), math.sin(-tf[2])
        return (c * dx - s * dy, s * dx + c * dy)

    def _on_check_clear(self, request, response):
        if self._nm is not None:
            response.answer = CLEAR
            return response
        costmap = self._costmap
        response.answer = UNKNOWN
        if costmap is not None:
            grid, frame, at = costmap
            tf = self._costmap_tf(frame)
            if tf is not None:
                [(x, y)] = to_frame([(request.x, request.y)], tf)
                response.answer = clear_answer(grid, x, y, float(request.r),
                                               age_s=time.monotonic() - at)
        return response

    # ---- 관리 명령 서비스: 락, 운영 모드, 충전 끝내기 ----
    def _on_set_lock(self, request, response):
        with self._lock:
            stopped = not self._fsm.has_goal()
            reply = self._admin.set_lock(request.request_id, request.robot_id, request.locked,
                                         stopped, time.monotonic())
        return _answer(response, request.request_id, reply)

    def _on_set_mode(self, request, response):
        with self._lock:
            reply = self._admin.set_mode(request.request_id, request.mode, time.monotonic())
        return _answer(response, request.request_id, reply)

    def _on_charge_end(self, request, response):
        with self._lock:
            reply = self._admin.charge_end(request.request_id, request.robot_id,
                                           float(self._battery), time.monotonic())
        return _answer(response, request.request_id, reply)

    # ---- 경로 점에 장애물이 있는지 검사 ----
    def _obstacle(self, points) -> bool:
        costmap = self._costmap
        if self._nm is not None or costmap is None or not points:
            return False
        grid, frame, _ = costmap
        if frame and frame != self._map_frame:
            tf = self._costmap_tf(frame)
            if tf is None:
                return False
            points = to_frame(points, tf)
        return obstacle_on(grid, points)

    # ---- 상태 기계 명령을 액션, 서비스, 토픽으로 실행 ----
    def _run(self, cmds) -> None:
        for cmd in cmds:
            if isinstance(cmd, SendPath):
                if not self._follow.send(cmd.seq, cmd.poses, cmd.final_goal, cmd.reverse,
                                         cmd.free_heading):
                    self.get_logger().warn("follow_path server is not available")
                    self._late.append((cmd.seq, "unavailable"))
            elif isinstance(cmd, CancelNav):
                self._follow.cancel_all()
            elif isinstance(cmd, BackUp):
                self.get_logger().info(
                    f"backup #{cmd.seq} {cmd.distance_m:.3f} m (self-recovery attempts "
                    f"{self._fsm.recovery_attempts_used})")
                if not self._backup.send(cmd.seq, cmd.distance_m, cmd.speed_mps,
                                         cmd.time_allowance_s):
                    self.get_logger().warn("backup server is not available")
                    self._late_backup.append((cmd.seq, "unavailable"))
            elif isinstance(cmd, CancelBackUp):
                self._backup.cancel_all()
            elif isinstance(cmd, SendNavGoal):
                if not self._goals.send(cmd.seq, cmd.pose):
                    self.get_logger().warn("navigate_to_pose server is not available")
                    self._late_goal.append(cmd.seq)
            elif isinstance(cmd, CancelNavGoal):
                self._goals.cancel_all()
            elif isinstance(cmd, ClearCostmap):
                self._clear_costmap()
            elif isinstance(cmd, LimitSpeed):
                msg = SpeedLimit(percentage=False, speed_limit=float(cmd.mps))
                msg.header.stamp = self.get_clock().now().to_msg()
                self._speed_pub.publish(msg)
            elif isinstance(cmd, Finish):
                self._servers.finish(cmd.goal_id, cmd.success, cmd.reason, cmd.last_done_seq_id)
                if not cmd.success:
                    self.get_logger().info(f"goal ended: {cmd.reason}")

    def _clear_costmap(self) -> None:
        if not self._clear.service_is_ready():
            self.get_logger().warn("clear costmap service is not available")
            return
        self._clear.call_async(ClearEntireCostmap.Request())

    # ---- 후진, NavigateToPose, FollowPath 결과 콜백 ----
    def _on_backup_result(self, seq: int, outcome: str, error_code: int,
                          travelled: float) -> None:
        reason = reason_of(outcome, error_code)
        self.get_logger().info(f"backup #{seq} {outcome} {reason or 'OK'} "
                               f"after {travelled:.3f} m")
        with self._lock:
            pose = self._pose()
            self._run(self._fsm.backup_result(seq, outcome, time.monotonic(),
                                              pose_here(pose), error_code))

    def _on_goal_feedback(self, seq: int, distance: float) -> None:
        with self._lock:
            self._fsm.nav_goal_feedback(seq, distance)

    def _on_goal_result(self, seq: int, outcome: str, lost: bool) -> None:
        with self._lock:
            pose = self._pose()
            self._run(self._fsm.nav_goal_result(seq, outcome, time.monotonic(),
                                                pose_here(pose), lost=lost))

    def _on_goal_cancel(self, seq: int) -> None:
        with self._lock:
            self._run(self._fsm.nav_goal_cancel_accepted(seq, time.monotonic()))

    def _on_follow_rejected(self, seq: int) -> None:
        with self._lock:
            self._late.append((seq, "rejected"))

    def _on_follow_result(self, seq: int, outcome: str, code: int, lost: bool) -> None:
        with self._lock:
            pose = self._pose()
            here = pose_here(pose)
            if lost:
                cmds = self._fsm.nav_lost(seq, time.monotonic(), here, self._obstacle)
            else:
                cmds = self._fsm.nav_result(seq, outcome, time.monotonic(), here,
                                            self._obstacle, code)
            self._run(cmds)

    def _on_follow_cancel(self, seq: int) -> None:
        with self._lock:
            self._run(self._fsm.nav_cancel_accepted(seq, time.monotonic()))

    # ---- Nav2 서버 준비: 둘 다 보이기 전에는 LOCALIZING으로 보고한다 ----
    def _note_nav_ready(self) -> None:
        if self._nav_ready:
            return
        if self._follow.ready() and self._goals.ready():
            self._nav_ready = True
            self.get_logger().info("follow_path and navigate_to_pose servers are ready")
        self._fsm.set_nav_ready(self._nav_ready)

    # ---- 제어 주기: 미뤄 둔 결과 처리 후 tick ----
    def _tick(self) -> None:
        with self._lock:
            if not self._alive:
                return
            pose = self._pose()
            here = pose_here(pose)
            now = time.monotonic()
            late, self._late = self._late, []
            for seq, outcome in late:
                self._run(self._fsm.nav_result(seq, outcome, now, here, self._obstacle))
            late_backup, self._late_backup = self._late_backup, []
            for seq, outcome in late_backup:
                self._run(self._fsm.backup_result(seq, outcome, now, here))
            late_goal, self._late_goal = self._late_goal, []
            for seq in late_goal:
                self._run(self._fsm.nav_goal_result(seq, "unavailable", now, here))
            self._note_nav_ready()
            self._run(self._fsm.tick(now, here, self._speed, self._obstacle))
            if self._nm is not None:
                # heartbeat 끊김을 먼저 본 뒤에 끝난 대역 일을 알린다
                self._step_no_motion(now)
            self._log_state()

    # ---- 복구 상태 변화 로그 ----
    def _log_state(self) -> None:
        state = self._fsm.state()
        step = self._fsm.local_yield_step()
        if (state, step) != self._last_state and RECOVERING in (state, self._last_state[0]):
            self.get_logger().info(f"state {self._last_state[0]} -> {state} {step} "
                                   f"(self-recovery attempts used "
                                   f"{self._fsm.recovery_attempts_used})")
        self._last_state = (state, step)

    # ---- 주문 경로 마커와 RobotState 발행 ----
    def _publish_markers(self) -> None:
        with self._lock:
            self._fsm.recheck_horizon()
            base, horizon = self._fsm.order_view()
            pose = self._pose() if base or horizon else None
        start = (round(pose[0], 1), round(pose[1], 1)) if pose else None
        self._markers.publish(start, base, horizon)

    def _publish_state(self) -> None:
        with self._lock:
            state, goal = self._fsm.state(), self._fsm.current_goal()
            step = self._fsm.local_yield_step()
            pose = self._pose()
            admin = self._admin
            msg = RobotState(robot_id=self.robot_id, battery=float(self._battery), state=state,
                             current_goal=goal, obstacle_x=math.nan, obstacle_y=math.nan,
                             local_yield=step, boot_id=admin.boot_id, lock=admin.lock,
                             mode=admin.mode, forced=list(admin.forced),
                             last_request_id=admin.last_request_id)
        seen = self._obstacle_ahead(pose) if state == RobotState.STATE_BLOCKED else None
        if seen is not None:
            msg.obstacle_x, msg.obstacle_y = float(seen[0]), float(seen[1])
            msg.obstacle_kind = RobotState.OBSTACLE_UNKNOWN
        if pose is not None and state != RobotState.STATE_LOCALIZING:
            msg.pose = Pose2D(x=float(pose[0]), y=float(pose[1]), theta=float(pose[2]))
            msg.stamp = pose[3]
        else:
            msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)


# ---- 관리 명령 공통 응답 채우기 ----
def _answer(response, request_id: str, reply):
    response.accepted, response.reason = reply.accepted, reply.reason
    response.request_id, response.detail = request_id, reply.detail
    return response


# ---- 노드 실행 진입점 ----
def main(args=None):
    rclpy.init(args=args)
    node = PodeFsmNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
