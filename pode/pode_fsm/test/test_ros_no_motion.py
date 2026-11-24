"""동작 없음 모드의 pode_fsm과 FakePode가 같은 목표에 같은 순서의 피드백, 결과, 상태를 내는지 본다.

구역 지도(기본 v7)에서 같은 목표를 차례로 보낸다: 고정 경로, Nav2 goal, 대체, 취소,
PATH_NOT_CONNECTED, 경로 물러나기, 곧은 물러나기, order(base 끝 대기, update, 취소), 도킹,
heartbeat 끊김. 피드백은 같은 값이 이어지면 하나로 묶어 순서만 비교한다(개수는 시간에 따라 다르다).
일부러 다른 점은 KNOWN_DIFFERENCES에 두고 시험으로 못 박는다.
"""
import math
import time

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("fake_robots.fake_pode_node")

from conftest import CONTRACT, Harness, wait_until  # noqa: E402
from fleet_core.traffic.site import load_site  # noqa: E402
from fleet_interfaces.action import Dock, MoveTo, Retreat  # noqa: E402
from fleet_interfaces.msg import RobotState, Segment  # noqa: E402
from fleet_interfaces.srv import CheckClear  # noqa: E402
from geometry_msgs.msg import Point, Pose, Pose2D, PoseStamped  # noqa: E402
from pode_fsm.contract import load_contract, to_ros_qos  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

C = load_contract(CONTRACT)
SITE = load_site(None)
START = "P-1"
# 시험을 줄이려고 기다림을 짧게 준다(기본은 2.0, 2.0, 3.0 s)
WAITS = {"no_motion_move_s": 1.0, "no_motion_retreat_s": 0.8, "no_motion_dock_s": 0.6}
DOCK_S = 0.6  # FakePode 시나리오의 dock.duration_s

# 일부러 다른 점. 시험과 함께 고친다
KNOWN_DIFFERENCES = {
    "distance_remaining": "no_motion은 자세가 끝날 때 한 번에 바뀌어 남은 거리가 그때까지 그대로다. "
                          "FakePode는 달리면서 줄어든다(필드와 순서는 같다).",
    "dock DONE": "no_motion은 성공 직전에 DONE 단계를 늘 보낸다(PC 도킹 대역 real_dock_wait와 같다). "
                 "FakePode는 틱 순서에 따라 DONE을 빠뜨릴 수 있다.",
    "dock error": "no_motion은 오차 0 cm로 성공한다(로봇이 이미 목표 자리). FakePode는 "
                  "dock.error_cm 둘레의 잡음을 준다.",
    "dock station": "no_motion은 스테이션과의 거리를 보지 않고, 모르는 dock_id도 경고 로그만 남기고 "
                    "성공한다(2026-11-24 결정). PC 도킹 대역은 0.15 m 밖이나 모르는 이름이면 실패한다.",
    "pose after heartbeat loss": "no_motion은 끝나기 전에 멈추면 자세가 시작 자리 그대로다. "
                                 "FakePode는 가던 자리에 선다.",
    "battery": "no_motion은 배터리를 모름(-1)으로 알린다(battery_topic이 없을 때와 같다). "
               "FakePode는 시나리오 배터리(90)다.",
    "check_clear": "FakePode에는 check_clear 서비스가 없다. no_motion은 늘 clear로 답한다.",
    "order current_goal": "order을 달리는 동안 pode_fsm은 지금 달리는 조각 묶음의 끝 노드(E-1, 다음 "
                          "O_OUT)를, FakePode는 base 끝 노드(O_OUT)를 current_goal로 알린다. "
                          "실주행 pode_fsm도 같다(동작 없음과 무관). 2026-11-24 결정: 실주행처럼 "
                          "조각 묶음의 끝을 유지한다.",
    "state rate": "no_motion 기본 5 Hz(설계 4.5), FakePode 기본 10 Hz.",
}


# ---- 목표 만들기 ----
def pose_of(node):
    return SITE.node_pose(node)


def follow_goal(*nodes):
    pts = [pose_of(n) for n in nodes]
    (ax, ay), (bx, by) = pts[-2], pts[-1]
    goal = MoveTo.Goal(target_id=nodes[-1], mode=MoveTo.Goal.MODE_FOLLOW_PATH,
                       target_pose=Pose2D(x=bx, y=by, theta=math.atan2(by - ay, bx - ax)))
    goal.path = [Pose(position=Point(x=x, y=y)) for x, y in pts]
    return goal


def nav_goal(node, theta=0.0):
    x, y = pose_of(node)
    return MoveTo.Goal(target_id=node, mode=MoveTo.Goal.MODE_NAV_GOAL,
                       target_pose=Pose2D(x=x, y=y, theta=theta))


def _stamped(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def seg(seq, mode, a, b):
    s = Segment(seq_id=seq, mode=mode, controller_id="FollowPath", end_node_id=b)
    s.path.header.frame_id = "map"
    (ax, ay), (bx, by) = pose_of(a), pose_of(b)
    yaw = math.atan2(by - ay, bx - ax)
    if mode == Segment.MODE_NAV_GOAL:
        s.target_pose = _stamped(bx, by, yaw)
    else:
        s.path.poses = [_stamped(ax, ay, yaw), _stamped(bx, by, yaw)]
    return s


def order_goal(base, horizon=(), update=0, stitch="", order_id="pode1-1"):
    goal = MoveTo.Goal(order_id=order_id, order_update_id=update, stitch_node_id=stitch,
                       base=list(base), horizon=list(horizon))
    goal.target_id = base[-1].end_node_id
    goal.target_pose.x, goal.target_pose.y = pose_of(base[-1].end_node_id)
    return goal


def retreat_goal(distance, speed, *nodes):
    goal = Retreat.Goal(distance_m=distance, speed_mps=speed)
    goal.path = [Pose(position=Point(x=x, y=y)) for x, y in (pose_of(n) for n in nodes)]
    return goal


# ---- 관제 흉내 ----
class Fleet(Node):
    def __init__(self, beat_hz=0.0):
        super().__init__("no_motion_probe")
        self.states: list[tuple[float, RobotState]] = []
        self.create_subscription(RobotState, C.name("robot_state", pode="pode1"),
                                 lambda m: self.states.append((time.monotonic(), m)),
                                 to_ros_qos(C.qos("robot_state")))
        self.move = ActionClient(self, MoveTo, C.name("move_to", pode="pode1"))
        self.dock = ActionClient(self, Dock, C.name("dock", pode="pode1"))
        self.retreat = ActionClient(self, Retreat, C.name("retreat", pode="pode1"))
        self.check = self.create_client(CheckClear, C.name("check_clear", pode="pode1"))
        self._beat = self.create_publisher(Header, C.name("heartbeat"),
                                           to_ros_qos(C.qos("heartbeat")))
        self.beating = beat_hz > 0
        self.last_beat = 0.0
        if beat_hz > 0:
            self.create_timer(1.0 / beat_hz, self._send_beat)

    def _send_beat(self):
        if self.beating:
            self._beat.publish(Header(frame_id="fleet_manager"))
            self.last_beat = time.monotonic()

    def ready(self):
        return all(c.wait_for_server(10.0) for c in (self.move, self.dock, self.retreat))

    def send(self, goal, client=None):
        """보내고 (핸들, 피드백 목록, 보내기 직전 시각)."""
        feedback = []
        t0 = time.monotonic()
        fut = (client or self.move).send_goal_async(
            goal, feedback_callback=lambda m: feedback.append(m.feedback))
        assert wait_until(fut.done, 5.0), "goal not answered"
        assert fut.result().accepted
        return fut.result(), feedback, t0

    @staticmethod
    def result(handle, timeout=20.0):
        fut = handle.get_result_async()
        assert wait_until(fut.done, timeout), "no result"
        return fut.result().result

    def last(self):
        return self.states[-1][1] if self.states else None

    def states_between(self, t0, t1):
        """t0 직전의 마지막 상태부터 t1까지."""
        before = [m for t, m in self.states if t < t0][-1:]
        return before + [m for t, m in self.states if t0 <= t <= t1]


def collapse(items):
    out = []
    for item in items:
        if not out or out[-1] != item:
            out.append(item)
    return out


def move_tokens(feedback):
    return collapse([(f.last_done_seq_id, f.waiting_at_base_end, f.horizon_warning)
                     for f in feedback])


def finish(fleet, sent, target=None):
    """한 목표의 비교 값: 결과, 묶은 피드백, 묶은 상태, 달리는 동안의 current_goal, 끝 자세."""
    handle, feedback, t0 = sent
    res = fleet.result(handle)
    t1 = time.monotonic()
    time.sleep(0.5)  # 끝난 뒤 상태도 본다
    seen = fleet.states_between(t0, time.monotonic())
    out = {"result": tuple(getattr(res, k) for k in ("success", "reason", "last_done_seq_id")
                           if hasattr(res, k)),
           "states": collapse([m.state for m in seen]),
           "goal": sorted({m.current_goal for m in seen
                           if m.state in (RobotState.STATE_MOVING, RobotState.STATE_DOCKING)})}
    if isinstance(res, MoveTo.Result):
        out["feedback"] = move_tokens(feedback)
        out["distance"] = [f.distance_remaining for f in feedback]
    elif isinstance(res, Dock.Result):
        out["feedback"] = collapse([f.phase for f in feedback])
    else:
        out["feedback"] = len(feedback)
    if target is not None:
        last = fleet.last()
        out["at_target"] = math.dist((last.pose.x, last.pose.y), pose_of(target)) < 0.02
    out["ended_at"] = t1
    return out


def stable(rec):
    """비교에 쓰는 칸만. 거리 값과 시각은 아래에서 따로 본다."""
    return {k: v for k, v in rec.items() if k not in ("distance", "ended_at")}


# ---- 두 로봇 띄우기 ----
def scenario_file(tmp_path):
    robot = {"start": START, "speed_scale": 1.0, "pose_noise_m": 0.0, "stop_error_m": 0.0}
    data = {"seed": 3, "speed_factor": 1.0, "timeout_s": 600,
            "stock": {"arm_left": {"A": 3}, "arm_right": {"C": 3}},
            "robots": {"pode1": robot, "pode2": {**robot, "start": "P-2"},
                       "pode3": {**robot, "start": "P-3"}},
            "arm": {"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 0.0},
            "dock": {"duration_s": DOCK_S, "error_cm": 1.0}}
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def fake_pode(harness, tmp_path, heartbeat_timeout_s):
    from fake_robots.fake_pode_node import FakePodeNode
    harness.add(FakePodeNode(parameter_overrides=harness.params(
        robot_id="pode1", contract_file=str(CONTRACT), scenario_file=scenario_file(tmp_path),
        state_rate_hz=10.0, heartbeat_timeout_s=heartbeat_timeout_s)))


def no_motion(harness, _tmp_path, heartbeat_timeout_s, short=True, **over):
    from pode_fsm.node import PodeFsmNode
    x, y = pose_of(START)
    values = dict(robot_id="pode1", contract_file=str(CONTRACT), no_motion=True,
                  initial_x=x, initial_y=y, initial_yaw=0.0,
                  heartbeat_timeout_s=heartbeat_timeout_s, **(WAITS if short else {}))
    values.update(over)
    return harness.add(PodeFsmNode(parameter_overrides=harness.params(**values)))


def run(rclpy_session, tmp_path, make, steps, heartbeat_timeout_s=0.0, beat_hz=0.0, **over):
    harness = Harness(rclpy_session)
    try:
        make(harness, tmp_path, heartbeat_timeout_s, **over)
        fleet = harness.add(Fleet(beat_hz))
        assert fleet.ready()
        assert wait_until(lambda: (s := fleet.last()) is not None
                          and s.state == RobotState.STATE_IDLE, 10.0)
        return steps(fleet)
    finally:
        harness.close()


# ---- 같은 목표 순서 ----
def zone_steps(fleet):
    out = {}
    out["follow"] = finish(fleet, fleet.send(follow_goal("P-1", "E-1")), "E-1")
    out["nav"] = finish(fleet, fleet.send(nav_goal("O_OUT")), "O_OUT")
    first = fleet.send(nav_goal("O_IN"))
    time.sleep(0.3)
    second = fleet.send(nav_goal("E-1"))
    out["replaced"] = finish(fleet, first)
    out["replacing"] = finish(fleet, second, "E-1")
    h = fleet.send(nav_goal("E-2"))
    time.sleep(0.4)
    h[0].cancel_goal_async()
    out["canceled"] = finish(fleet, h)
    out["not_connected"] = finish(fleet, fleet.send(follow_goal("Q2", "Q1")))
    out["back_to_E1"] = finish(fleet, fleet.send(nav_goal("E-1", math.pi / 2)), "E-1")
    out["retreat_path"] = finish(fleet, fleet.send(retreat_goal(0.14, 0.05, "E-1", "P-1"),
                                                   fleet.retreat), "P-1")
    out["retreat_straight"] = finish(fleet, fleet.send(Retreat.Goal(distance_m=0.03,
                                                                    speed_mps=0.05),
                                                       fleet.retreat))
    out["dock"] = finish(fleet, fleet.send(Dock.Goal(dock_id="WORK"), fleet.dock))
    out["after_dock"] = finish(fleet, fleet.send(follow_goal("P-1", "E-1")))
    return out


def order_steps(fleet):
    out = {}
    first = fleet.send(order_goal(
        [seg(1, Segment.MODE_FOLLOW_PATH, "P-1", "E-1"), seg(2, Segment.MODE_NAV_GOAL, "E-1",
                                                             "O_OUT")],
        [seg(3, Segment.MODE_NAV_GOAL, "O_OUT", "O_IN")]))
    assert wait_until(lambda: any(f.waiting_at_base_end for f in first[1]), 20.0)
    out["waiting_state"] = fleet.last().state
    out["waiting_goal"] = fleet.last().current_goal
    second = fleet.send(order_goal([seg(3, Segment.MODE_NAV_GOAL, "O_OUT", "O_IN")],
                                   update=1, stitch="O_OUT"))
    out["superseded"] = finish(fleet, first)
    out["stitched"] = finish(fleet, second, "O_IN")
    h = fleet.send(order_goal([seg(1, Segment.MODE_NAV_GOAL, "O_IN", "E-2")],
                              order_id="pode1-2"))
    time.sleep(0.4)
    h[0].cancel_goal_async()
    out["order_canceled"] = finish(fleet, h)
    return out


def heartbeat_steps(fleet):
    assert wait_until(lambda: fleet.last_beat > 0.0, 5.0)
    sent = fleet.send(nav_goal("E-3"))
    time.sleep(0.5)
    fleet.beating = False
    stopped_at = fleet.last_beat
    rec = finish(fleet, sent)
    rec["lost_after_s"] = rec["ended_at"] - stopped_at
    last = fleet.last()
    rec["pose"] = (last.pose.x, last.pose.y)
    time.sleep(1.5)  # 멈춘 뒤에 기다리던 일이 끝나도 자세가 바뀌지 않는다
    rec["pose_later"] = (fleet.last().pose.x, fleet.last().pose.y)
    return rec


# ---- 비교 ----
def test_zone_goals_give_the_same_sequences_as_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, zone_steps)
    real = run(rclpy_session, tmp_path, no_motion, zone_steps)
    for name in fake:
        f, r = stable(fake[name]), stable(real[name])
        if name == "dock":  # KNOWN_DIFFERENCES["dock DONE"]
            f["feedback"] = [p for p in f["feedback"] if p != "DONE"]
            assert r["feedback"] == ["APPROACH", "ALIGN", "DONE"]
            r["feedback"] = r["feedback"][:-1]
        assert r == f, name
    assert fake["follow"]["result"] == (True, "", 0)
    assert fake["follow"]["states"] == ["IDLE", "MOVING", "IDLE"]
    assert fake["follow"]["goal"] == ["E-1"] and fake["follow"]["at_target"]
    assert fake["nav"]["result"] == (True, "", 0) and fake["nav"]["at_target"]
    assert fake["replaced"]["result"] == (False, "CANCELED", 0)
    assert fake["replacing"]["result"] == (True, "", 0)
    assert fake["canceled"]["result"] == (False, "CANCELED", 0)
    assert fake["not_connected"]["result"] == (False, "PATH_NOT_CONNECTED", 0)
    assert fake["not_connected"]["states"] == ["IDLE"]
    assert fake["retreat_path"]["result"] == (True, "")
    assert fake["retreat_path"]["states"] == ["IDLE", "MOVING", "IDLE"]
    assert fake["retreat_straight"]["result"] == (True, "")
    assert fake["dock"]["result"] == (True,)
    assert fake["dock"]["states"] == ["IDLE", "DOCKING", "DOCKED"]
    assert fake["dock"]["goal"] == ["WORK"]
    assert fake["after_dock"]["states"][:2] == ["DOCKED", "MOVING"]
    # 피드백 묶음: 이동 중에는 (0, False, "") 하나뿐
    assert fake["follow"]["feedback"] == [(0, False, "")]


def test_distance_remaining_is_a_known_difference(rclpy_session, tmp_path):
    """KNOWN_DIFFERENCES["distance_remaining"]: 끝날 때까지 그대로, 0 이상."""
    real = run(rclpy_session, tmp_path, no_motion,
               lambda f: finish(f, f.send(nav_goal("O_OUT")), "O_OUT"))
    first = real["distance"][0]
    assert first == pytest.approx(math.dist(pose_of(START), pose_of("O_OUT")), abs=1e-3)
    assert all(d == pytest.approx(first, abs=1e-6) for d in real["distance"])


def test_order_goals_give_the_same_sequences_as_fake_pode(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, order_steps)
    real = run(rclpy_session, tmp_path, no_motion, order_steps)
    for name in fake:
        f = stable(fake[name]) if isinstance(fake[name], dict) else fake[name]
        r = stable(real[name]) if isinstance(real[name], dict) else real[name]
        if name == "superseded":  # KNOWN_DIFFERENCES["order current_goal"]
            assert "E-1" in r["goal"] and "E-1" not in f["goal"]
            r["goal"] = [g for g in r["goal"] if g != "E-1"]
        assert r == f, name
    assert fake["waiting_state"] == RobotState.STATE_MOVING
    assert fake["waiting_goal"] == "O_OUT"
    assert fake["superseded"]["result"] == (False, "SUPERSEDED", 2)
    assert fake["superseded"]["feedback"] == [(0, False, ""), (1, False, ""), (2, True, "")]
    assert fake["stitched"]["result"] == (True, "", 3)
    assert fake["order_canceled"]["result"] == (False, "CANCELED", 0)


def test_heartbeat_loss_stops_like_fake_pode_after_1_5_s(rclpy_session, tmp_path):
    fake = run(rclpy_session, tmp_path, fake_pode, heartbeat_steps, heartbeat_timeout_s=1.5,
               beat_hz=10.0)
    real = run(rclpy_session, tmp_path, no_motion, heartbeat_steps, heartbeat_timeout_s=1.5,
               beat_hz=10.0, no_motion_move_s=6.0)
    keys = ("result", "states", "goal", "feedback")
    assert {k: real[k] for k in keys} == {k: fake[k] for k in keys}
    assert real["result"] == (False, "HEARTBEAT_LOST", 0)
    assert real["states"] == ["IDLE", "MOVING", "IDLE"]
    assert 1.4 <= real["lost_after_s"] <= 2.2
    assert 1.4 <= fake["lost_after_s"] <= 2.2
    # KNOWN_DIFFERENCES["pose after heartbeat loss"]
    assert real["pose"] == pytest.approx(pose_of(START), abs=1e-6)
    assert real["pose_later"] == pytest.approx(pose_of(START), abs=1e-6)
    assert math.dist(fake["pose"], pose_of(START)) > 0.05


def _fsm_alone(rclpy_session, check, **over):
    harness = Harness(rclpy_session)
    try:
        no_motion(harness, None, 0.0, **over)
        fleet = harness.add(Fleet())
        assert fleet.ready()
        return check(fleet)
    finally:
        harness.close()


def test_state_comes_about_five_times_a_second_with_the_last_goal_pose(rclpy_session):
    def check(fleet):
        assert wait_until(lambda: fleet.states, 5.0)
        t0 = time.monotonic()
        time.sleep(3.0)
        n = len([1 for t, _ in fleet.states if t >= t0])
        before = fleet.last()
        rec = finish(fleet, fleet.send(nav_goal("E-2", 1.0)), "E-2")
        return n, before, rec, fleet.last()

    n, before, rec, after = _fsm_alone(rclpy_session, check)
    assert 12 <= n <= 18, n  # 3 s x 5 Hz
    assert (before.pose.x, before.pose.y, before.pose.theta) == pytest.approx(
        (*pose_of(START), 0.0), abs=1e-6)
    assert before.robot_id == "pode1" and before.state == RobotState.STATE_IDLE
    assert rec["result"] == (True, "", 0) and rec["at_target"]
    assert (after.pose.x, after.pose.y, after.pose.theta) == pytest.approx(
        (*pose_of("E-2"), 1.0), abs=1e-6)
    assert after.battery == -1.0  # KNOWN_DIFFERENCES["battery"]


def test_check_clear_always_answers_clear(rclpy_session):
    """KNOWN_DIFFERENCES["check_clear"]: FakePode에는 없다."""
    def check(fleet):
        assert fleet.check.wait_for_service(5.0)
        answers = []
        for x, y, r in ((0.12, 0.10, 0.2), (5.0, 5.0, 0.1), (pose_of("W")[0], 0.5, 1.0)):
            fut = fleet.check.call_async(CheckClear.Request(x=x, y=y, r=r))
            assert wait_until(fut.done, 5.0)
            answers.append(fut.result().answer)
        return answers

    assert _fsm_alone(rclpy_session, check) == [CheckClear.Response.ANSWER_CLEAR] * 3


def test_an_unknown_dock_id_still_succeeds(rclpy_session):
    """2026-11-24 결정: dock_id가 예상과 달라도 성공시키고 경고 로그만 남긴다."""
    def check(fleet):
        handle, _, _ = fleet.send(Dock.Goal(dock_id="NOPE"), fleet.dock)
        ok = fleet.result(handle).success
        assert wait_until(lambda: fleet.last().state == RobotState.STATE_DOCKED, 2.0)
        return ok

    assert _fsm_alone(rclpy_session, check) is True


def test_default_waits_are_two_two_and_three_seconds(rclpy_session):
    def check(fleet):
        out = {}
        for name, client, goal in (
                ("move", None, follow_goal("P-1", "E-1")),
                ("retreat", fleet.retreat, Retreat.Goal(distance_m=0.05, speed_mps=0.05)),
                ("dock", fleet.dock, Dock.Goal(dock_id="WORK"))):
            handle, _, t0 = fleet.send(goal, client)
            assert fleet.result(handle).success
            out[name] = time.monotonic() - t0
        return out

    took = _fsm_alone(rclpy_session, check, short=False)
    assert 1.9 <= took["move"] <= 2.6
    assert 1.9 <= took["retreat"] <= 2.6
    assert 2.9 <= took["dock"] <= 3.6


def test_a_job_due_in_the_tick_that_finds_the_heartbeat_lost_ends_heartbeat_lost(rclpy_session):
    """같은 틱에 기다림이 끝나고 heartbeat도 끊겼으면 성공이 아니라 HEARTBEAT_LOST(FakePode와 같게).
    실행기를 돌리지 않고 시계를 손으로 움직인다."""
    from pode_fsm import node as node_mod
    from pode_fsm.node import PodeFsmNode
    x, y = pose_of(START)
    node = PodeFsmNode(parameter_overrides=Harness.params(None, **dict(
        robot_id="pode1", contract_file=str(CONTRACT), no_motion=True, initial_x=x,
        initial_y=y, initial_yaw=0.0, heartbeat_timeout_s=1.5, no_motion_move_s=2.0)))
    clock = [1000.0]
    real_monotonic = node_mod.time.monotonic
    node_mod.time.monotonic = lambda: clock[0]
    node._goals._clock = lambda: clock[0]
    finished = []
    node._servers.finish = lambda goal_id, ok, reason, done=0: finished.append((ok, reason))
    try:
        node._on_heartbeat(None)
        with node._lock:
            node._run(node._fsm.move("g", clock[0], "E-2", (*pose_of("E-2"), 0.0), [],
                                     node._pose(), mode="nav_goal"))
        node._tick()
        clock[0] += 2.0  # 기다림도 끝나고 heartbeat도 1.5 s 넘게 끊겼다
        node._tick()
        assert finished == [(False, "HEARTBEAT_LOST")]
        assert node._nm.pose[:2] == pytest.approx((x, y))
        clock[0] += 1.0
        node._tick()
        assert node._nm.pose[:2] == pytest.approx((x, y))
    finally:
        node_mod.time.monotonic = real_monotonic
        node.destroy_node()
