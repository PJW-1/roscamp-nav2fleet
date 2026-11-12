"""pinky_fsm 노드. 핑키 한 대의 move_to, dock을 받아 Nav2 FollowPath로 넘기고 상태를 보고한다.

판단은 모두 core.PinkyFsm이 한다. 이 파일은 ROS와 잇기만 한다.
- 액션 서버: 계약의 move_to(MoveTo), dock(Dock), retreat(Retreat, 버전 2 1-A, 계약 0.9-draft 제안.
  path가 있으면 후진 controller reverse_controller_id로 따른다)
- 발행: 계약의 robot_state(RobotState, 기본 10 Hz), Nav2 controller의 speed_limit
  (잠금 경계 목표만, 버전 2 4절 제안), order_markers(MarkerArray, order 모델 표시)
- 구독: 계약의 heartbeat, odom(속도), local costmap(BLOCKED 판단), map과 keepout 마스크
  (order 모델 horizon 미리 검사), robots(/fleet/robots: 로봇단 교착 처리에서 앞을 막은 로봇과 그
  priority. 추가 지시서 3의 7-4, 계약 0.13-draft)
- MoveTo의 order 칸(계약 0.11-draft 제안)은 order_ros가 바꾸고 order_fsm.OrderFsm이 판단한다
- 액션 클라이언트: Nav2 controller의 follow_path(FollowPath, follow_ros.py), behavior_server의
  backup(BackUp, 자가 복구. 핑키_계약 3절 7번 2026-11-09 제안), bt_navigator의
  navigate_to_pose(NavigateToPose, MoveTo mode nav_goal. 계약 0.11-draft 제안, nav_goal_ros.py)
- 서비스 클라이언트: local costmap 비우기(ClearEntireCostmap, 자가 복구)
- 위치: TF map → base_footprint(AMCL). 시간이 지난 TF는 위치를 모르는 것으로 친다

로봇 namespace 안에서 돌고 /tf, /tf_static을 namespace의 tf, tf_static으로 바꿔 띄운다.
"""
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

UNKNOWN_BATTERY = -1.0  # 배터리 토픽이 없을 때. 관제는 보여 주기만 한다
MARKER_PERIOD_S = 0.5


class PinkyFsmNode(Node):
    def __init__(self, **kwargs):
        super().__init__("pinky_fsm", **kwargs)
        declare = self.declare_parameter
        declare("robot_id", self.get_namespace().strip("/") or "pinky1")
        declare("contract_file", "")
        declare("map_frame", "map")
        declare("base_frame", "base_footprint")
        declare("follow_path_action", "follow_path")
        declare("backup_action", "backup")
        declare("navigate_to_pose_action", "navigate_to_pose")  # MoveTo mode nav_goal
        declare("clear_costmap_service", "local_costmap/clear_entirely_local_costmap")
        declare("controller_id", "FollowPath")
        # Controller for paths that end at a lock boundary (MoveTo.final_goal false): an RPP
        # without the approach slow-down, so the speed limit is the only slow-down there.
        # Empty = off: every path uses controller_id, as before. Nav2 must load this plugin
        declare("through_controller_id", "")
        # 물러나기 경로(Retreat.path)를 후진으로 따르는 controller. pinky_nav_config가 늘 띄운다
        declare("reverse_controller_id", "FollowPathReverse")
        declare("goal_checker_id", "general_goal_checker")
        # 후진 경로의 goal checker. 후진은 제자리에서 방향을 고칠 수 없으니 방향을 보지 않는 것
        # (pinky_nav_config retreat_goal_checker). 비우면 goal_checker_id 그대로
        declare("reverse_goal_checker_id", "retreat_goal_checker")
        declare("progress_checker_id", "progress_checker")
        declare("odom_topic", "odom")
        declare("costmap_topic", "local_costmap/costmap")
        declare("speed_limit_topic", "speed_limit")  # nav2 controller_server speed_limit_topic
        declare("battery_topic", "")
        declare("state_rate_hz", 10.0)
        declare("control_rate_hz", 20.0)
        declare("pose_timeout_s", 1.0)
        # BLOCKED와 함께 보고할 장애물 칸을 가던 길 앞 이만큼 안에서 찾는다(계약 0.12-draft)
        declare("obstacle_look_m", 0.30)
        # order 모델(계약 0.11-draft 제안): horizon 미리 검사에 쓰는 지도와 keepout 마스크, 표시
        declare("map_topic", "map")
        declare("keepout_mask_topic", "keepout_filter_mask")  # 비면 keepout 검사를 하지 않는다
        declare("order_markers_topic", "order_markers")
        fsm_params, order_params = declare_fsm_params(self)
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
        self._last_state = ("", "")  # (state, local_yield) 로그용
        contract = load_contract(get("contract_file").value or default_contract_path())

        self._lock = threading.RLock()
        self._alive = True
        self._late: list[tuple[int, str]] = []  # 다음 주기에 core에 넘길 FollowPath 결과
        self._late_backup: list[tuple[int, str]] = []  # 다음 주기에 넘길 BackUp 결과
        self._late_goal: list[int] = []  # 다음 주기에 거절로 넘길 NavigateToPose(서버 없음)
        self._speed = (0.0, 0.0)
        # (격자, 틀, 받은 시각). 한 번에 바꾸고 한 번에 읽어 짝이 맞게 둔다
        self._costmap: tuple[Grid, str, float] | None = None
        self._battery = UNKNOWN_BATTERY

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
        # Nav2 costmap은 reliable로 발행한다. volatile로 받으면 transient_local 발행과도 맞는다
        self.create_subscription(
            OccupancyGrid, get("costmap_topic").value, self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), callback_group=group)
        if get("battery_topic").value:
            self.create_subscription(BatteryState, get("battery_topic").value,
                                     self._on_battery, qos_profile_sensor_data)
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
        # 관제의 CheckClear(계약 0.12-draft): 막힘 표시의 점 둘레가 비었는가
        self.create_service(CheckClear, contract.name("check_clear", pinky=self.robot_id),
                            self._on_check_clear, callback_group=group)
        self._servers = GoalServers(self, contract, self.robot_id, group, self._lock, self._fsm,
                                    self._pose, self._run, lambda: self._alive)
        # 판단과 보고는 벽시계로 돈다(heartbeat는 벽시계로 재야 하고, 시뮬 시계가 멈춰도 보고한다)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(get("control_rate_hz").value), self._tick,
                          callback_group=group, clock=steady)
        self.create_timer(1.0 / float(get("state_rate_hz").value), self._publish_state,
                          callback_group=group, clock=steady)
        self.create_timer(MARKER_PERIOD_S, self._publish_markers, callback_group=group,
                          clock=steady)
        self.get_logger().info(f"pinky_fsm for {self.robot_id} ready")

    # -- 수명 ------------------------------------------------------------------------------

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

    # -- 입력 ------------------------------------------------------------------------------

    def _on_heartbeat(self, _msg) -> None:
        with self._lock:
            self._fsm.heartbeat(time.monotonic())

    def _on_fleet_robots(self, msg: RobotStatusArray) -> None:
        """관제가 아는 로봇들의 위치와 priority(로봇단 교착 처리). 받은 시각으로 오래됐는지 본다."""
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
        # 받은 시각도 같이 둔다. 오래된 격자로는 CheckClear에 답하지 않는다
        self._costmap = (grid, msg.header.frame_id, time.monotonic())

    def _on_battery(self, msg: BatteryState) -> None:
        if not math.isnan(msg.percentage):
            self._battery = float(msg.percentage) * 100.0

    def _pose(self):
        """map 틀의 (x, y, yaw, stamp). 모르면 None."""
        try:
            tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None
        stamp = Time.from_msg(tf.header.stamp)
        if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > self._pose_timeout:
            return None  # AMCL이 멈췄다
        q, p = tf.transform.rotation, tf.transform.translation
        return (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w), tf.header.stamp)

    def _costmap_tf(self, frame):
        """costmap 틀에서 본 map 틀의 자세 (x, y, yaw). 같은 틀이면 0이고, 모르면 None."""
        if not frame or frame == self._map_frame:
            return (0.0, 0.0, 0.0)
        try:
            tf = self._tf.lookup_transform(frame, self._map_frame, Time())
        except TransformException:
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w))

    def _obstacle_ahead(self, pose):
        """BLOCKED로 선 로봇 앞의 가장 가까운 장애물 칸(map 좌표, 계약 0.12-draft). 모르면 None."""
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
        dx, dy = found[0] - tf[0], found[1] - tf[1]  # costmap 틀에서 map 틀로 되돌린다
        c, s = math.cos(-tf[2]), math.sin(-tf[2])
        return (c * dx - s * dy, s * dx + c * dy)

    def _on_check_clear(self, request, response):
        """관제의 CheckClear(계약 0.12-draft): map 좌표 (x, y)의 반경 r 안에 장애물 칸이 있는가."""
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

    # -- core 명령 실행 ----------------------------------------------------------------------

    def _run(self, cmds) -> None:
        """self._lock을 쥔 채 부른다."""
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
            if lost:  # 취소가 받아들여질 때까지 core가 이 goal을 기다린다
                cmds = self._fsm.nav_lost(seq, time.monotonic(), here, self._obstacle)
            else:
                cmds = self._fsm.nav_result(seq, outcome, time.monotonic(), here,
                                            self._obstacle, code)
            self._run(cmds)

    def _on_follow_cancel(self, seq: int) -> None:
        with self._lock:
            self._run(self._fsm.nav_cancel_accepted(seq, time.monotonic()))

    # -- 주기 ------------------------------------------------------------------------------

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

    def _log_state(self) -> None:
        """자가 복구가 들고 나는 것을 남긴다(시뮬 기록과 맞춰 보려고)."""
        state = self._fsm.state()
        step = self._fsm.local_yield_step()
        if (state, step) != self._last_state and RECOVERING in (state, self._last_state[0]):
            self.get_logger().info(f"state {self._last_state[0]} -> {state} {step} "
                                   f"(self-recovery attempts used "
                                   f"{self._fsm.recovery_attempts_used})")
        self._last_state = (state, step)

    def _publish_markers(self) -> None:
        with self._lock:
            self._fsm.recheck_horizon()  # 지도나 마스크가 order 뒤에 왔을 수 있다
            base, horizon = self._fsm.order_view()
            pose = self._pose() if base or horizon else None
        # 첫 nav_goal 선은 로봇에서 시작한다. 10 cm로 잘라 움직일 때마다 내지 않는다
        start = (round(pose[0], 1), round(pose[1], 1)) if pose else None
        self._markers.publish(start, base, horizon)

    def _publish_state(self) -> None:
        with self._lock:
            state, goal = self._fsm.state(), self._fsm.current_goal()
            step = self._fsm.local_yield_step()  # 로봇단 교착 처리의 단계(계약 0.13-draft)
            pose = self._pose()
        msg = RobotState(robot_id=self.robot_id, battery=float(self._battery), state=state,
                         current_goal=goal, obstacle_x=math.nan, obstacle_y=math.nan,
                         local_yield=step)
        seen = self._obstacle_ahead(pose) if state == RobotState.STATE_BLOCKED else None
        if seen is not None:
            msg.obstacle_x, msg.obstacle_y = float(seen[0]), float(seen[1])
            msg.obstacle_kind = RobotState.OBSTACLE_UNKNOWN  # 비전이 들어오면 채운다
        if pose is not None and state != RobotState.STATE_LOCALIZING:
            msg.pose = Pose2D(x=float(pose[0]), y=float(pose[1]), theta=float(pose[2]))
            msg.stamp = pose[3]  # 위치를 잰 시각(계약 2절)
        else:
            msg.stamp = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PinkyFsmNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
