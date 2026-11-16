# ---- Relocalize 액션 서버: AMCL 무이동 갱신, Nav2 Spin, 전역 재초기화 ----
from __future__ import annotations

import collections
import time
from collections.abc import Callable

import rclpy
from builtin_interfaces.msg import Duration
from fleet_interfaces.action import Relocalize
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from nav2_msgs.action import Spin as SpinAction
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import TransformException

from pode_fsm.action_client import GoalRunner
from pode_fsm.geometry import quaternion_yaw
from pode_fsm.params import RelocalizeParams
from pode_fsm.relocalize import (
    BUSY,
    CANCELED,
    CancelSpin,
    Done,
    Estimate,
    GlobalInit,
    MapField,
    Nomotion,
    Relocalizer,
    Spin,
    distance_field,
    is_busy,
    scan_match,
    scan_points,
    std_from_covariance,
)

# ---- 루프 주기, 피드백 주기, 지도 QoS, 스캔 보관 수 ----
LOOP_S = 0.02
FEEDBACK_PERIOD_S = 0.2
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SCANS_KEPT = 20
SCAN_MATCH_SLOP_S = 0.15  # AMCL 추정과 같은 시각의 스캔이 없을 때 쓸 수 있는 시각 차


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ---- Nav2 Spin 실행기 ----
class SpinRunner(GoalRunner):
    _label = "spin"

    def __init__(self, node, action_name: str, on_result: Callable[[int, str, int], None],
                 callback_group=None):
        super().__init__(node, SpinAction, action_name, callback_group)
        self._on_result = on_result

    def send(self, seq: int, yaw: float, allowance_s: float) -> bool:
        if not self._client.server_is_ready():
            return False
        sec = int(allowance_s)
        goal = SpinAction.Goal(target_yaw=float(yaw), time_allowance=Duration(
            sec=sec, nanosec=int((allowance_s - sec) * 1e9)))
        return self._send_goal(seq, goal)

    def _rejected(self, seq: int) -> None:
        self._on_result(seq, "rejected", 0)

    def _finished(self, seq: int, outcome: str, code: int) -> None:
        self._on_result(seq, outcome, code)

    def _lost(self, seq: int) -> None:
        # 결과를 잃었다. 취소는 공통 실행기가 보냈다
        self._on_result(seq, "aborted", 0)


# ---- 액션 서버 ----
class RelocalizeServer:
    def __init__(self, node, name: str, p: RelocalizeParams, group, lock, base_frame: str,
                 tf_buffer, state: Callable[[], str], topics: dict[str, str]):
        self._node, self.p = node, p
        self._base_frame, self._tf = base_frame, tf_buffer
        self._state = state
        # 노드의 RLock을 같이 쓴다: 바쁨 검사와 이동 goal 수락이 서로 끼어들지 않는다
        self._lock = lock
        self._machine: Relocalizer | None = None
        self._spin_seq = 0
        self._spin_open = False  # 보낸 회전의 결과를 아직 못 받았다
        self._used_stamp = None  # 마지막으로 판정에 넘긴 amcl_pose 시각
        self._start_pose = None
        self._latest = None  # 마지막으로 받은 amcl_pose 자세(돌지 않을 때도)
        self._field: MapField | None = None
        self._laser: dict[str, tuple[float, float, float]] = {}
        self._scans: collections.deque = collections.deque(maxlen=SCANS_KEPT)
        self._last_scan = 0.0
        self._last_pose = 0.0
        self._since = 0.0
        self._nomotion = node.create_client(Empty, topics["nomotion"], callback_group=group)
        self._global = node.create_client(Empty, topics["global"], callback_group=group)
        self._spin = SpinRunner(node, topics["spin"], self._on_spin_result, callback_group=group)
        node.create_subscription(OccupancyGrid, topics["map"], self._on_map, LATCHED,
                                 callback_group=group)
        node.create_subscription(LaserScan, topics["scan"], self._on_scan,
                                 qos_profile_sensor_data, callback_group=group)
        node.create_subscription(PoseWithCovarianceStamped, topics["amcl_pose"],
                                 self._on_amcl_pose, LATCHED, callback_group=group)
        self._server = ActionServer(
            node, Relocalize, name, execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT, callback_group=group)

    @property
    def active(self) -> bool:
        return self._machine is not None

    def stop(self) -> None:
        self._spin.cancel_all()

    def destroy(self) -> None:
        self._server.destroy()
        self._spin.destroy()

    # ---- 지도, 스캔, AMCL 추정 구독 ----
    def _on_map(self, msg: OccupancyGrid) -> None:
        info, o = msg.info, msg.info.origin
        dist = distance_field(info.resolution, info.width, info.height, msg.data,
                              self.p.occupied_threshold, 2.0 * self.p.match_tol_m)
        yaw = quaternion_yaw(o.orientation.x, o.orientation.y, o.orientation.z,
                             o.orientation.w)
        self._field = MapField(info.resolution, o.position.x, o.position.y, yaw, dist)

    def _on_scan(self, msg: LaserScan) -> None:
        t = _seconds(msg.header.stamp)
        with self._lock:
            if t <= self._last_scan:
                return
            self._last_scan = t
            self._scans.append((t, msg))
            if self._machine is not None:
                self._run(self._machine.scan(time.monotonic()))

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        t = _seconds(msg.header.stamp)
        q, pos = msg.pose.pose.orientation, msg.pose.pose.position
        pose = (pos.x, pos.y, quaternion_yaw(q.x, q.y, q.z, q.w))
        with self._lock:
            self._latest = pose
            # 새 갱신: 이번 실행 시작 뒤이고 마지막으로 쓴 것보다 새 시각
            if self._machine is None or t < self._since or t <= self._last_pose:
                return
            self._last_pose = t
            std_xy, std_yaw = std_from_covariance(msg.pose.covariance)
            e = Estimate(pose[0], pose[1], pose[2], std_xy, std_yaw, self._match(t, pose))
            self._used_stamp = msg.header.stamp
            self._run(self._machine.estimate(time.monotonic(), e))

    # ---- 그 추정과 같은 시각 스캔의 일치율 ----
    def _match(self, t: float, pose) -> float:
        if self._field is None or not self._scans:
            return 0.0
        st, scan = min(self._scans, key=lambda s: abs(s[0] - t))
        if abs(st - t) > SCAN_MATCH_SLOP_S:
            return 0.0
        laser = self._laser_pose(scan.header.frame_id)
        if laser is None:
            return 0.0
        pts = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                          scan.range_min, scan.range_max)
        return scan_match(self._field, pts, pose, laser, self.p.match_tol_m)

    def _laser_pose(self, frame: str):
        if frame in self._laser:
            return self._laser[frame]
        if not frame or frame == self._base_frame:
            self._laser[frame] = (0.0, 0.0, 0.0)
            return self._laser[frame]
        try:
            tf = self._tf.lookup_transform(self._base_frame, frame, Time())
        except TransformException as exc:
            self._node.get_logger().warn(f"relocalize: no {self._base_frame}->{frame}: {exc}",
                                         throttle_duration_sec=2.0)
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        self._laser[frame] = (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w))
        return self._laser[frame]

    # ---- 단계 기계 명령 실행 ----
    def _run(self, cmds) -> None:
        for cmd in cmds:
            if isinstance(cmd, Nomotion):
                if self._nomotion.service_is_ready():
                    self._nomotion.call_async(Empty.Request())
            elif isinstance(cmd, Spin):
                self._spin_seq += 1
                self._spin_open = self._spin.send(self._spin_seq, cmd.yaw, cmd.allowance_s)
                if not self._spin_open:
                    self._node.get_logger().warn("relocalize: spin server is not available")
                    self._run(self._machine.spin_result(time.monotonic(), "rejected", 0))
            elif isinstance(cmd, CancelSpin):
                self._spin.cancel_all()
            elif isinstance(cmd, GlobalInit):
                self._call_global()

    def _call_global(self) -> None:
        if not self._global.service_is_ready():
            self._run(self._machine.global_ready(time.monotonic(), False))
            return
        self._global.call_async(Empty.Request()).add_done_callback(self._on_global_done)

    def _on_global_done(self, future) -> None:
        ok = future.exception() is None
        with self._lock:
            if self._machine is None:
                return
            # 재초기화 앞의 추정은 버린다
            self._since = _seconds(self._node.get_clock().now().to_msg())
            self._run(self._machine.global_ready(time.monotonic(), ok))

    def _on_spin_result(self, seq: int, outcome: str, code: int) -> None:
        with self._lock:
            if seq != self._spin_seq:
                return
            self._spin_open = False
            if self._machine is None:
                return
            self._run(self._machine.spin_result(time.monotonic(), outcome, code))

    # ---- goal 실행: 바쁨 검사, 서비스 확인, 루프, 결과 ----
    def _execute(self, handle):
        req = handle.request
        with self._lock:
            if is_busy(self._state(), self._machine is not None):
                handle.abort()
                return Relocalize.Result(success=False, reason=BUSY, stage="")
            machine = Relocalizer(self.p, req.allow_spin, req.allow_global,
                                  float(req.timeout_s), time.monotonic())
            self._machine = machine
            self._since = _seconds(self._node.get_clock().now().to_msg())
            self._used_stamp = None
            # 시작 자세: 시작 때 마지막 amcl_pose. 없으면 pose_valid는 거짓(계약)
            self._start_pose = self._latest
        try:
            ready = self._localizer_ready(req.allow_global)
            with self._lock:
                self._run(machine.start(ready))
            self._node.get_logger().info(
                f"relocalize started (spin {req.allow_spin}, global {req.allow_global}, "
                f"timeout {req.timeout_s:.1f} s)")
            last_fb = 0.0
            while machine.result is None:
                if not rclpy.ok():
                    with self._lock:
                        self._run(machine.cancel(time.monotonic()))
                    break
                now = time.monotonic()
                with self._lock:
                    if handle.is_cancel_requested:
                        self._run(machine.cancel(now))
                    self._run(machine.tick(now))
                if machine.result is None and now - last_fb >= FEEDBACK_PERIOD_S:
                    last_fb = now
                    e = machine.last
                    handle.publish_feedback(Relocalize.Feedback(
                        stage=machine.stage, std_xy_m=float(e.std_xy) if e else -1.0,
                        scan_match=float(e.match) if e else -1.0))
                time.sleep(LOOP_S)
            self._wait_spin_stopped()
        finally:
            with self._lock:
                self._machine = None
                stamp, start = self._used_stamp, self._start_pose
        return self._finish(handle, machine, stamp, start)

    # ---- 결과 전에 회전이 멈췄는지(결과가 왔는지) 기다리기 ----
    def _wait_spin_stopped(self) -> None:
        end = time.monotonic() + self.p.spin_stop_wait_s
        while self._spin_open and time.monotonic() < end and rclpy.ok():
            time.sleep(LOOP_S)
        if self._spin_open:
            self._node.get_logger().warn("relocalize: spin did not confirm the stop in time")

    def _localizer_ready(self, allow_global: bool) -> bool:
        wait = self.p.service_wait_s
        if not self._nomotion.wait_for_service(timeout_sec=wait):
            self._node.get_logger().warn("relocalize: request_nomotion_update is missing")
            return False
        if allow_global and not self._global.wait_for_service(timeout_sec=wait):
            self._node.get_logger().warn("relocalize: reinitialize_global_localization missing")
            return False
        if self._field is None:
            self._node.get_logger().warn("relocalize: no map yet")
            return False
        return True

    def _finish(self, handle, machine: Relocalizer, stamp, start):
        done: Done | None = machine.result
        res = Relocalize.Result(success=False, reason=CANCELED, stage=machine.stage,
                                fresh_updates=int(machine.fresh_updates))
        if done is not None:
            res.success, res.reason = done.success, done.reason
            e = done.estimate
            if e is not None and stamp is not None and start is not None:
                res.pose_valid = True
                res.stamp = stamp
                res.pose = Pose2D(x=float(e.x), y=float(e.y), theta=float(e.yaw))
                res.start_pose = Pose2D(x=float(start[0]), y=float(start[1]),
                                        theta=float(start[2]))
                res.std_xy_m, res.std_yaw_rad = float(e.std_xy), float(e.std_yaw)
                res.scan_match = float(e.match)
        self._node.get_logger().info(
            f"relocalize ended: {'OK' if res.success else res.reason} std_xy "
            f"{res.std_xy_m:.3f} m, std_yaw {res.std_yaw_rad:.3f} rad, "
            f"scan_match {res.scan_match:.2f}")
        try:
            if res.success:
                handle.succeed()
            elif res.reason == CANCELED and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        except Exception:  # noqa: BLE001 - 종료 중에는 상태 발행이 실패할 수 있다
            pass
        return res
