"""relocalize 액션 서버를 실제 ROS 인터페이스로 시험한다. AMCL과 behavior_server 대신 FakeAmcl."""
import math
import threading
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("nav2_msgs.action")
try:
    from fleet_interfaces.action import Relocalize
except ImportError:  # 새 action이 없는 옛 설치
    pytest.skip("fleet_interfaces has no Relocalize", allow_module_level=True)

from conftest import CONTRACT, wait_until  # noqa: E402
from fake_nav import FakeNav  # noqa: E402
from fleet_interfaces.action import MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # noqa: E402
from nav2_msgs.action import Spin  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from probe import Probe, lane_goal  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_srvs.srv import Empty  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

RES, SIZE = 0.05, 60  # 3 m 방, 테두리 한 칸이 벽
LO, HI = 0.025, SIZE * RES - 0.025
N_BEAMS = 180


# ---- 가짜 AMCL: 지도, 스캔, 무이동 갱신, 전역 재초기화, Spin ----
class FakeAmcl(Node):
    def __init__(self, nav, offset=(0.0, 0.0), std_xy=0.03, std_yaw=0.03, spin="succeed",
                 global_service=True, initial_pose=True):
        super().__init__("fake_amcl")
        self.nav, self.offset = nav, offset
        self.std_xy, self.std_yaw = std_xy, std_yaw
        self.spin_mode = spin
        self.nomotion_calls = 0
        self.global_calls = 0
        self.spins = []
        self.spin_ends = []  # (시각, 끝난 모양)
        self._force = False
        self._lock = threading.Lock()
        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map = self.create_publisher(OccupancyGrid, "map", latched)
        self._map.publish(self._room())
        self._scan = self.create_publisher(LaserScan, "scan", 10)
        # 실제 AMCL처럼 amcl_pose는 transient_local
        self._pose = self.create_publisher(PoseWithCovarianceStamped, "amcl_pose", latched)
        if initial_pose:
            self._publish_pose(self.get_clock().now().to_msg())
        static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "base_footprint", "laser"
        t.transform.rotation.w = 1.0
        static.sendTransform(t)
        self.create_service(Empty, "request_nomotion_update", self._on_nomotion,
                            callback_group=group)
        if global_service:
            self.create_service(Empty, "reinitialize_global_localization", self._on_global,
                                callback_group=group)
        self._spin = ActionServer(
            self, Spin, "spin", execute_callback=self._execute_spin,
            goal_callback=self._on_spin_goal, cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_timer(0.1, self._publish_scan, callback_group=group)

    def _room(self):
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.info.resolution, m.info.width, m.info.height = RES, SIZE, SIZE
        m.info.origin.orientation.w = 1.0
        m.data = [100 if i in (0, SIZE - 1) or j in (0, SIZE - 1) else 0
                  for j in range(SIZE) for i in range(SIZE)]
        return m

    def _on_nomotion(self, _req, resp):
        with self._lock:
            self.nomotion_calls += 1
            self._force = True
        return resp

    def _on_global(self, _req, resp):
        with self._lock:
            self.global_calls += 1
        return resp

    def _on_spin_goal(self, goal):
        self.spins.append(goal.target_yaw)
        return GoalResponse.REJECT if self.spin_mode == "reject" else GoalResponse.ACCEPT

    def _execute_spin(self, handle):
        end = time.monotonic() + (30.0 if self.spin_mode == "long" else 0.5)
        while time.monotonic() < end:
            if handle.is_cancel_requested:
                time.sleep(0.2)  # 멈추는 데 걸리는 시간
                self.spin_ends.append((time.monotonic(), "canceled"))
                handle.canceled()
                return Spin.Result()
            time.sleep(0.02)
        self.spin_ends.append((time.monotonic(), self.spin_mode))
        if self.spin_mode == "collide":
            handle.abort()
            return Spin.Result(error_code=Spin.Result.COLLISION_AHEAD)
        handle.succeed()
        return Spin.Result()

    # ---- 참 자세에서 잰 스캔과, 갱신을 요청받았으면 AMCL 추정 ----
    def _publish_scan(self):
        x, y, yaw = self.nav.pose
        stamp = self.get_clock().now().to_msg()
        s = LaserScan()
        s.header.stamp, s.header.frame_id = stamp, "laser"
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / N_BEAMS
        s.angle_max = s.angle_min + s.angle_increment * (N_BEAMS - 1)
        s.range_min, s.range_max = 0.05, 12.0
        s.ranges = [self._ray(x, y, yaw + s.angle_min + k * s.angle_increment)
                    for k in range(N_BEAMS)]
        self._scan.publish(s)
        with self._lock:
            force, self._force = self._force, False
        if force:
            self._publish_pose(stamp)

    def _publish_pose(self, stamp):
        x, y, yaw = self.nav.pose
        p = PoseWithCovarianceStamped()
        p.header.stamp, p.header.frame_id = stamp, "map"
        p.pose.pose.position.x = x + self.offset[0]
        p.pose.pose.position.y = y + self.offset[1]
        p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.std_xy ** 2
        cov[35] = self.std_yaw ** 2
        p.pose.covariance = cov
        self._pose.publish(p)

    @staticmethod
    def _ray(x, y, a):
        c, s = math.cos(a), math.sin(a)
        ds = []
        if c > 1e-9:
            ds.append((HI - x) / c)
        if c < -1e-9:
            ds.append((LO - x) / c)
        if s > 1e-9:
            ds.append((HI - y) / s)
        if s < -1e-9:
            ds.append((LO - y) / s)
        return float(min(ds))


# ---- 관제 쪽 relocalize 클라이언트 ----
class RelocClient(Node):
    def __init__(self):
        super().__init__("reloc_client")
        self.client = ActionClient(self, Relocalize, "/pode1/relocalize")
        self.feedback = []

    def send(self, timeout_s=10.0, allow_spin=True, allow_global=False):
        assert self.client.wait_for_server(10.0)
        goal = Relocalize.Goal(allow_spin=allow_spin, allow_global=allow_global,
                               timeout_s=float(timeout_s))
        fut = self.client.send_goal_async(goal, feedback_callback=self._fb)
        assert wait_until(fut.done, 5.0)
        handle = fut.result()
        assert handle.accepted
        return handle

    def _fb(self, msg):
        self.feedback.append(msg.feedback)

    @staticmethod
    def result(handle, timeout=20.0):
        fut = handle.get_result_async()
        assert wait_until(fut.done, timeout), "no result"
        return fut.result()


def start(harness, amcl=True, **amcl_kw):
    from pode_fsm.node import PodeFsmNode
    nav = harness.add(FakeNav())
    fake = harness.add(FakeAmcl(nav, **amcl_kw)) if amcl else None
    harness.add(PodeFsmNode(parameter_overrides=harness.params(
        robot_id="pode1", contract_file=str(CONTRACT), heartbeat_timeout_s=0.0,
        local_yield=False, relocalize_service_wait_s=0.5)))
    probe = harness.add(Probe())
    client = harness.add(RelocClient())
    assert probe.ready()
    assert wait_until(lambda: (s := probe.last_state()) is not None
                      and s.state == RobotState.STATE_IDLE, 10.0)
    return nav, fake, probe, client


def test_converges_standing_and_reports_localizing_meanwhile(harness):
    nav, fake, probe, client = start(harness)
    t0 = time.monotonic()
    res = client.result(client.send()).result
    assert res.success is True and res.reason == ""
    assert res.stage == Relocalize.Feedback.STAGE_NOMOTION and res.pose_valid is True
    assert res.fresh_updates >= 5
    assert res.stamp.sec > 0 or res.stamp.nanosec > 0
    assert (res.start_pose.x, res.start_pose.y) == (pytest.approx(nav.pose[0], abs=1e-3),
                                                    pytest.approx(nav.pose[1], abs=1e-3))
    assert (res.pose.x, res.pose.y) == (pytest.approx(nav.pose[0], abs=1e-3),
                                        pytest.approx(nav.pose[1], abs=1e-3))
    assert res.scan_match >= 0.95
    assert res.std_xy_m == pytest.approx(0.03, abs=1e-3)
    assert fake.nomotion_calls >= 5
    assert fake.spins == []  # 무이동 단계에서 끝났다
    assert RobotState.STATE_LOCALIZING in probe.states_since(t0)
    assert client.feedback and client.feedback[0].stage == Relocalize.Feedback.STAGE_NOMOTION
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_IDLE, 2.0)


def test_scan_mismatch_with_small_covariance_fails(harness):
    _, _, _, client = start(harness, offset=(0.4, 0.0))
    res = client.result(client.send(timeout_s=2.0, allow_spin=False)).result
    assert res.success is False and res.reason == Relocalize.Result.REASON_SCAN_MISMATCH
    assert res.scan_match < 0.75


def test_busy_while_moving(harness):
    _, _, probe, client = start(harness)
    probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6"]))
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_MOVING, 2.0)
    res = client.result(client.send()).result
    assert res.success is False and res.reason == Relocalize.Result.REASON_BUSY
    assert res.stage == "" and res.pose_valid is False


def test_second_relocalize_is_busy(harness):
    _, _, _, client = start(harness, std_xy=0.5)
    first = client.send(timeout_s=3.0, allow_spin=False)
    time.sleep(0.3)
    res = client.result(client.send()).result
    assert res.reason == Relocalize.Result.REASON_BUSY and res.stage == ""
    assert client.result(first).result.reason == Relocalize.Result.REASON_NOT_CONVERGED


def test_no_amcl_pose_at_start_means_pose_not_valid(harness):
    _, _, _, client = start(harness, initial_pose=False)
    res = client.result(client.send()).result
    assert res.success is True and res.pose_valid is False


def test_timeout_stops_the_spin_before_the_result(harness):
    _, fake, _, client = start(harness, std_xy=0.5, spin="long")
    h = client.send(timeout_s=3.0)
    res = client.result(h)
    t_res = time.monotonic()
    # 시간이 다 됐다. 사유는 마지막 추정으로 정한다(std 0.5 m라 NOT_CONVERGED)
    assert res.result.reason == Relocalize.Result.REASON_NOT_CONVERGED
    assert res.result.stage == Relocalize.Feedback.STAGE_SPIN
    assert fake.spin_ends and fake.spin_ends[0][1] == "canceled"
    assert fake.spin_ends[0][0] <= t_res


def test_move_goal_is_rejected_while_relocalizing(harness):
    _, _, probe, client = start(harness, std_xy=0.5)
    h = client.send(timeout_s=3.0, allow_spin=False)
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_LOCALIZING, 2.0)
    fut = probe.move.send_goal_async(lane_goal(["P-3", "P-4"]))
    assert wait_until(fut.done, 5.0)
    assert fut.result().accepted is False
    res = client.result(h).result
    assert res.reason == Relocalize.Result.REASON_NOT_CONVERGED


def test_no_localizer_without_amcl_services(harness):
    _, _, _, client = start(harness, amcl=False)
    res = client.result(client.send()).result
    assert res.success is False and res.reason == Relocalize.Result.REASON_NO_LOCALIZER


def test_spins_when_nomotion_does_not_converge(harness):
    _, fake, _, client = start(harness, std_xy=0.5)
    h = client.send(timeout_s=8.0)
    assert wait_until(lambda: len(fake.spins) == 1, 5.0)
    assert fake.spins[0] == pytest.approx(2 * math.pi)
    fake.std_xy = 0.03  # 돈 뒤 모였다
    res = client.result(h).result
    assert res.success is True
    assert any(f.stage == Relocalize.Feedback.STAGE_SPIN for f in client.feedback)


def test_spin_rejected_is_spin_blocked(harness):
    _, fake, _, client = start(harness, std_xy=0.5, spin="reject")
    res = client.result(client.send(timeout_s=4.0)).result
    assert res.reason == Relocalize.Result.REASON_SPIN_BLOCKED
    assert len(fake.spins) == 1


def test_spin_collision_goes_global_when_allowed(harness):
    _, fake, _, client = start(harness, std_xy=0.5, spin="collide")
    h = client.send(timeout_s=6.0, allow_global=True)
    assert wait_until(lambda: fake.global_calls == 1, 5.0)
    fake.std_xy = 0.03
    res = client.result(h).result
    assert res.success is True
    assert len(fake.spins) == 1  # 전역 단계에서는 다시 돌지 않는다
    assert any(f.stage == Relocalize.Feedback.STAGE_GLOBAL for f in client.feedback)


def test_cancel_stops_the_spin_and_ends_canceled(harness):
    _, fake, probe, client = start(harness, std_xy=0.5, spin="long")
    h = client.send(timeout_s=8.0)
    assert wait_until(lambda: len(fake.spins) == 1, 5.0)
    h.cancel_goal_async()
    res = client.result(h)
    t_res = time.monotonic()
    assert res.result.reason == Relocalize.Result.REASON_CANCELED
    assert fake.spin_ends and fake.spin_ends[0][1] == "canceled" and fake.spin_ends[0][0] <= t_res
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_IDLE, 2.0)


def test_move_works_again_after_relocalize(harness):
    _, _, probe, client = start(harness)
    assert client.result(client.send()).result.success
    res = probe.result(probe.send(lane_goal(["P-3", "P-4"])))
    assert res.result.success is True and isinstance(res.result, MoveTo.Result)
