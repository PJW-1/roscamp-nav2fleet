# ---- 핑키 상태 기계 ROS 노드 ----
from __future__ import annotations

import math
import threading
import time

import rclpy
from fleet_interfaces.msg import RobotState, RobotStatusArray
from fleet_interfaces.srv import CheckClear
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

from pinky_fsm.backup import BackUp, CancelBackUp, reason_of
from pinky_fsm.backup_ros import BackUpRunner
from pinky_fsm.contract import default_contract_path, load_contract, to_ros_qos
from pinky_fsm.core import (
    RECOVERING,
    CancelNav,
    ClearCostmap,
    Finish,
    LimitSpeed,
    SendPath,
)
from pinky_fsm.costmap import Grid, obstacle_on, to_frame
from pinky_fsm.follow_ros import FollowPathRunner
from pinky_fsm.geometry import quaternion_yaw
from pinky_fsm.local_yield import peers_from
from pinky_fsm.nav_goal import CancelNavGoal, SendNavGoal
from pinky_fsm.nav_goal_ros import NavGoalRunner
from pinky_fsm.obstacle_report import UNKNOWN, clear_answer, nearest_ahead
from pinky_fsm.order_fsm import OrderFsm
from pinky_fsm.order_ros import OrderMaps, OrderMarkers
from pinky_fsm.params_ros import declare_fsm_params
from pinky_fsm.server_ros import GoalServers

# ---- 배터리 미상 값과 마커 발행 주기 ----
UNKNOWN_BATTERY = -1.0
MARKER_PERIOD_S = 0.5


# ---- 핑키 상태 기계 노드 PinkyFsmNode ----
class PinkyFsmNode(Node):
    def __init__(self, **kwargs):
        super().__init__("pinky_fsm", **kwargs)
        # ---- 노드 파라미터 선언 ----
        declare = self.declare_parameter
        declare("robot_id", self.get_namespace().strip("/") or "pinky1")
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
        declare("progress_checker_id", "progress_checker")
        declare("odom_topic", "odom")
        declare("costmap_topic", "local_costmap/costmap")
        declare("speed_limit_topic", "speed_limit")
        declare("battery_topic", "")
        declare("state_rate_hz", 10.0)
        declare("control_rate_hz", 20.0)
        declare("pose_timeout_s", 1.0)
        declare("obstacle_look_m", 0.30)
        declare("map_topic", "map")
        declare("keepout_mask_topic", "keepout_filter_mask")
        declare("order_markers_topic", "order_markers")
        fsm_params, order_params = declare_fsm_params(self)
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

        # ---- TF, 발행자, 구독자 만들기 ----
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._speed_pub = self.create_publisher(SpeedLimit, get("speed_limit_topic").value, 10)
        self._state_pub = self.create_publisher(
            RobotState, contract.name("robot_state", pinky=self.robot_id),
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
        # ---- Nav2 액션 실행기, 서비스, 액션 서버 만들기 ----
        self._follow = FollowPathRunner(
            self, get("follow_path_action").value, self._map_frame, self._nav_ids,
            self._through_id, self._reverse_id, self._on_follow_rejected,
            self._on_follow_result, self._on_follow_cancel, callback_group=group)
        self._backup = BackUpRunner(self, get("backup_action").value, self._on_backup_result,
                                    callback_group=group)
        self._goals = NavGoalRunner(self, get("navigate_to_pose_action").value, self._map_frame,
                                    self._on_goal_feedback, self._on_goal_result,
                                    self._on_goal_cancel, callback_group=group)
        self._clear = self.create_client(ClearEntireCostmap, get("clear_costmap_service").value,
                                         callback_group=group)
        self.create_service(CheckClear, contract.name("check_clear", pinky=self.robot_id),
                            self._on_check_clear, callback_group=group)
        self._servers = GoalServers(self, contract, self.robot_id, group, self._lock, self._fsm,
                                    self._pose, self._run, lambda: self._alive)
        # ---- 제어, 상태, 마커 타이머 시작 ----
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(get("control_rate_hz").value), self._tick,
                          callback_group=group, clock=steady)
        self.create_timer(1.0 / float(get("state_rate_hz").value), self._publish_state,
                          callback_group=group, clock=steady)
        self.create_timer(MARKER_PERIOD_S, self._publish_markers, callback_group=group,
                          clock=steady)
        self.get_logger().info(f"pinky_fsm for {self.robot_id} ready")

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

    # ---- 경로 점에 장애물이 있는지 검사 ----
    def _obstacle(self, points) -> bool:
        costmap = self._costmap
        if costmap is None or not points:
            return False
        grid, frame, _ = costmap
        if frame and frame != self._map_frame:
            try:
                tf = self._tf.lookup_transform(frame, self._map_frame, Time())
            except TransformException:
                return False
            q, p = tf.transform.rotation, tf.transform.translation
            points = to_frame(points, (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w)))
        return obstacle_on(grid, points)

    # ---- 상태 기계 명령을 액션, 서비스, 토픽으로 실행 ----
    def _run(self, cmds) -> None:
        for cmd in cmds:
            if isinstance(cmd, SendPath):
                if not self._follow.send(cmd.seq, cmd.poses, cmd.final_goal, cmd.reverse):
                    self.get_logger().warn("follow_path server is not available")
                    self._late.append((cmd.seq, "rejected"))
            elif isinstance(cmd, CancelNav):
                self._follow.cancel_all()
            elif isinstance(cmd, BackUp):
                self.get_logger().info(
                    f"backup #{cmd.seq} {cmd.distance_m:.3f} m (self-recovery attempts "
                    f"{self._fsm.recovery_attempts_used})")
                if not self._backup.send(cmd.seq, cmd.distance_m, cmd.speed_mps,
                                         cmd.time_allowance_s):
                    self.get_logger().warn("backup server is not available")
                    self._late_backup.append((cmd.seq, "rejected"))
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
                                              pose[:3] if pose else None, error_code))

    def _on_goal_feedback(self, seq: int, distance: float) -> None:
        with self._lock:
            self._fsm.nav_goal_feedback(seq, distance)

    def _on_goal_result(self, seq: int, outcome: str, lost: bool) -> None:
        with self._lock:
            pose = self._pose()
            self._run(self._fsm.nav_goal_result(seq, outcome, time.monotonic(),
                                                pose[:3] if pose else None, lost=lost))

    def _on_goal_cancel(self, seq: int) -> None:
        with self._lock:
            self._run(self._fsm.nav_goal_cancel_accepted(seq, time.monotonic()))

    def _on_follow_rejected(self, seq: int) -> None:
        with self._lock:
            self._late.append((seq, "rejected"))

    def _on_follow_result(self, seq: int, outcome: str, code: int, lost: bool) -> None:
        with self._lock:
            pose = self._pose()
            here = pose[:3] if pose else None
            if lost:
                cmds = self._fsm.nav_lost(seq, time.monotonic(), here, self._obstacle)
            else:
                cmds = self._fsm.nav_result(seq, outcome, time.monotonic(), here,
                                            self._obstacle, code)
            self._run(cmds)

    def _on_follow_cancel(self, seq: int) -> None:
        with self._lock:
            self._run(self._fsm.nav_cancel_accepted(seq, time.monotonic()))

    # ---- 제어 주기: 미뤄 둔 결과 처리 후 tick ----
    def _tick(self) -> None:
        with self._lock:
            if not self._alive:
                return
            pose = self._pose()
            here = pose[:3] if pose else None
            now = time.monotonic()
            late, self._late = self._late, []
            for seq, outcome in late:
                self._run(self._fsm.nav_result(seq, outcome, now, here, self._obstacle))
            late_backup, self._late_backup = self._late_backup, []
            for seq, outcome in late_backup:
                self._run(self._fsm.backup_result(seq, outcome, now, here))
            late_goal, self._late_goal = self._late_goal, []
            for seq in late_goal:
                self._run(self._fsm.nav_goal_result(seq, "rejected", now, here))
            self._run(self._fsm.tick(now, here, self._speed, self._obstacle))
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
        msg = RobotState(robot_id=self.robot_id, battery=float(self._battery), state=state,
                         current_goal=goal, obstacle_x=math.nan, obstacle_y=math.nan,
                         local_yield=step)
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


# ---- 노드 실행 진입점 ----
def main(args=None):
    rclpy.init(args=args)
    node = PinkyFsmNode()
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
