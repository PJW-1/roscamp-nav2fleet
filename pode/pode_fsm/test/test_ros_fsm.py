"""실제 액션·토픽 인터페이스로 pode_fsm 노드를 시험한다. Nav2 대신 FakeNav를 쓴다."""
import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")
pytest.importorskip("nav2_msgs.action")

from conftest import CONTRACT, wait_until  # noqa: E402
from fake_nav import FakeNav  # noqa: E402
from fleet_interfaces.action import Dock, MoveTo  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from probe import WAYPOINTS, Probe, lane_goal  # noqa: E402


def start(harness, heartbeat_timeout_s=0.0, nav=True, beat_hz=0.0, **fsm_params):
    from pode_fsm.node import PodeFsmNode
    # 로봇단 교착 처리를 끈 자가 복구를 본다(켠 것은 test_ros_local_yield.py)
    fsm_params.setdefault("local_yield", False)
    fake = harness.add(FakeNav()) if nav else None
    harness.add(PodeFsmNode(parameter_overrides=harness.params(
        robot_id="pode1", contract_file=str(CONTRACT),
        heartbeat_timeout_s=heartbeat_timeout_s, **fsm_params)))
    probe = harness.add(Probe(beat_hz=beat_hz))
    assert probe.ready()
    if nav:
        assert wait_until(lambda: (s := probe.last_state()) is not None
                          and s.state == RobotState.STATE_IDLE, 10.0)
    return fake, probe


def test_reports_localizing_without_a_pose(harness):
    _, probe = start(harness, nav=False)
    assert wait_until(lambda: len(probe.states) >= 3, 5.0)
    s = probe.last_state()
    assert s.state == RobotState.STATE_LOCALIZING and s.robot_id == "pode1"


def test_state_is_published_at_five_hertz_with_the_map_pose(harness):
    _, probe = start(harness)
    t0 = time.monotonic()
    time.sleep(2.0)
    n = len([1 for t, _ in probe.states if t >= t0])
    assert 9 <= n <= 11
    s = probe.last_state()
    assert (s.pose.x, s.pose.y) == (pytest.approx(0.35, abs=1e-3), pytest.approx(1.55, abs=1e-3))
    assert s.current_goal == ""


def test_a_fresh_node_reports_locked_unknown_mode_and_a_boot_id(harness):
    _, probe = start(harness, nav=False)
    assert wait_until(lambda: len(probe.states) >= 2, 5.0)
    first, last = probe.states[0][1], probe.last_state()
    assert (last.lock, last.mode, list(last.forced), last.detail) == (True, "UNKNOWN", [], "")
    assert len(last.boot_id) == 36 and first.boot_id == last.boot_id


def test_the_admin_services_answer_with_the_request_id_and_the_state_shows_them(harness):
    import rclpy.node
    from fleet_interfaces.srv import RobotCommand, SetLock, SetMode
    _, probe = start(harness)
    caller = harness.add(rclpy.node.Node("admin_caller"))

    def call(srv, name, request):
        client = caller.create_client(srv, name)
        assert client.wait_for_service(timeout_sec=5.0), name
        future = client.call_async(request)
        assert wait_until(future.done, 5.0), name
        return future.result()

    mode = call(SetMode, "/pode1/set_mode", SetMode.Request(request_id="m-1", mode="TEST"))
    assert (mode.accepted, mode.request_id) == (True, "m-1")
    unlock = call(SetLock, "/pode1/set_lock",
                  SetLock.Request(request_id="l-1", robot_id="pode1", locked=False))
    assert (unlock.accepted, unlock.reason, unlock.request_id) == (True, "", "l-1")
    charge = call(RobotCommand, "/pode1/charge_end",
                  RobotCommand.Request(request_id="c-1", robot_id="pode1"))
    assert (charge.accepted, charge.reason) == (False, "BATTERY_LOW"), "battery unknown"
    assert wait_until(lambda: (s := probe.last_state()) is not None
                      and s.last_request_id == "l-1", 3.0)
    s = probe.last_state()
    assert (s.lock, s.mode) == (False, "TEST")


def test_tf_and_odom_are_taken_by_the_input_node_not_the_main_node(harness):
    # 자주 오는 TF·odom이 주 실행기를 깨우지 않게 입력 노드가 받는다
    fake, probe = start(harness)
    fsm = harness.nodes[1]
    frequent = {"/tf", "/tf_static", "/odom"}
    assert not {s.topic_name for s in fsm.subscriptions} & frequent
    assert {s.topic_name for s in fsm._inputs.node.subscriptions} >= frequent
    # 그래도 자세(TF)와 속도(odom)는 그대로 들어온다
    s = probe.last_state()
    assert (s.pose.x, s.pose.y) == (pytest.approx(0.35, abs=1e-3), pytest.approx(1.55, abs=1e-3))
    probe.send(lane_goal(["P-3", "P-4"]))
    assert wait_until(lambda: fsm._speed[0] > 0.05, 3.0)


def test_the_input_node_follows_the_namespace_and_tf_remaps_like_the_real_launch(rclpy_session):
    # 실물 launch처럼 이름공간과 /tf:=tf를 주면 입력 노드도 /<ns>/tf를 받는다
    from pode_fsm.node import PodeFsmNode
    from rclpy.parameter import Parameter
    node = PodeFsmNode(namespace="pode9", cli_args=[
        "--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
        parameter_overrides=[Parameter("contract_file", value=str(CONTRACT))])
    try:
        assert node._inputs.node.get_namespace() == "/pode9"
        assert {s.topic_name for s in node._inputs.node.subscriptions} == {
            "/pode9/tf", "/pode9/tf_static", "/pode9/odom"}
    finally:
        node.stop()
        node.destroy_node()


def test_a_failed_start_leaves_no_input_thread_running(rclpy_session):
    # 초기화가 실패하면 입력 노드 스레드도 남지 않는다
    import threading

    from pode_fsm.node import PodeFsmNode
    from rclpy.parameter import Parameter

    def spinning():
        return [t for t in threading.enumerate() if "(_spin)" in t.name]
    before = len(spinning())
    with pytest.raises(ZeroDivisionError):
        PodeFsmNode(parameter_overrides=[Parameter("contract_file", value=str(CONTRACT)),
                                         Parameter("control_rate_hz", value=0.0)])
    assert len(spinning()) == before


def test_path_goal_arrives_within_the_tolerance(harness):
    _, probe = start(harness)
    h = probe.send(lane_goal(["P-3", "P-4"]))
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_MOVING, 2.0)
    assert probe.last_state().current_goal == "P-4"
    res = probe.result(h)
    assert res.result.success is True and res.result.reason == ""
    time.sleep(0.3)
    s = probe.last_state()
    assert math.dist((s.pose.x, s.pose.y), WAYPOINTS["P-4"]) <= 0.15
    assert s.state == RobotState.STATE_IDLE


def test_extension_replaces_the_goal_without_stopping(harness):
    """경로를 늘린 새 move_to가 와도 서지 않는다."""
    fake, probe = start(harness)
    first = probe.send(lane_goal(["P-3", "P-4"]))
    time.sleep(0.4)
    t_ext = time.monotonic()
    second = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    old = probe.result(first)
    assert old.result.success is False and old.result.reason == MoveTo.Result.REASON_CANCELED
    new = probe.result(second)
    assert new.result.success is True
    assert fake.cancels == []  # FollowPath를 취소하지 않고 새 목표를 바로 보냈다
    assert len(fake.goals) >= 2  # 셋째부터는 끝 근처 재전송이다
    assert not fake.stopped_between(t_ext - 0.2, t_ext + 0.4)


def test_cancel_stops_and_returns_canceled(harness):
    fake, probe = start(harness)
    h = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    time.sleep(0.3)
    h.cancel_goal_async()
    res = probe.result(h)
    assert res.result.success is False and res.result.reason == MoveTo.Result.REASON_CANCELED
    assert wait_until(lambda: len(fake.cancels) >= 1, 2.0)
    assert wait_until(lambda: fake.v == 0.0, 1.0)
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_IDLE, 1.0)


def test_disconnected_path_fails_and_is_not_followed(harness):
    """로봇에서 0.15 m 넘게 떨어진 경로는 따라가지 않는다."""
    fake, probe = start(harness)
    goal = lane_goal(["P-5", "P-6"])  # 로봇은 P-3에 있다
    res = probe.result(probe.send(goal))
    assert res.result.success is False
    assert res.result.reason == MoveTo.Result.REASON_PATH_NOT_CONNECTED
    assert fake.goals == []


def test_point_goal_drives_to_the_target(harness):
    _, probe = start(harness)
    res = probe.result(probe.send(lane_goal(["P-3", "P-4"], path=False)))
    assert res.result.success is True


def test_heartbeat_loss_stops_the_robot(harness):
    fake, probe = start(harness, heartbeat_timeout_s=1.0)
    t0 = time.monotonic()
    h = probe.send(lane_goal(["P-3", "P-4", "P-5", "P-6", "N"]))
    res = probe.result(h)
    took = time.monotonic() - t0
    assert res.result.success is False
    assert res.result.reason == MoveTo.Result.REASON_HEARTBEAT_LOST
    assert 0.9 <= took <= 1.5
    assert wait_until(lambda: fake.v == 0.0 and len(fake.cancels) >= 1, 1.0)


def test_heartbeat_keeps_the_goal_running(harness):
    _, probe = start(harness, heartbeat_timeout_s=1.0, beat_hz=2.0)
    res = probe.result(probe.send(lane_goal(["P-3", "P-4", "P-5"])))
    assert res.result.success is True


def test_obstacle_ahead_reports_blocked_and_resumes(harness):
    """Pode_계약 4절: 서고 2초 안에 BLOCKED, goal은 열어 두고, 비키면 이어 간다(recovery_attempts=0)."""
    fake, probe = start(harness, recovery_attempts=0)
    fake.obstacle = (0.35, 1.95)  # P-4 위
    h = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: fake.v == 0.0 and fake.pose[1] > 1.6, 5.0)
    t_stop = time.monotonic()
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 2.5)
    assert time.monotonic() - t_stop <= 2.0 + 0.2
    time.sleep(1.5)  # FakeNav는 1초 뒤 FollowPath를 실패로 끝낸다
    assert not h.get_result_async().done()
    assert probe.last_state().state == RobotState.STATE_BLOCKED
    fake.obstacle = None
    res = probe.result(h, timeout=10.0)
    assert res.result.success is True
    assert len(fake.goals) >= 2  # 비킨 뒤 남은 경로를 다시 보냈다


def test_default_distance_rule_only_clears_the_costmap_close_to_the_path_start(harness):
    """경로 시작에서 0.15 m는 release + eps(0.23 m) 안이라 후진 없이 지우기만 한다."""
    fake, probe = start(harness)
    fake.obstacle = (0.35, 1.95)
    probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: len(fake.clears) >= 1, 6.0)
    assert fake.backups == []


def test_obstacle_ahead_backs_up_twice_then_reports_blocked(harness):
    """Pode_계약 3절 7번: 물러나고 지도를 비운 뒤 두 번까지 다시 가며 RECOVERING을 보고하고, 그래도 막히면 BLOCKED다."""
    # FakeNav의 레인은 짧다. release_distance_m=0으로 물러날 자리를 만든다(거리 규칙은 test_core)
    fake, probe = start(harness, release_distance_m=0.0)
    fake.obstacle = (0.35, 1.95)  # P-4 위. FakeNav는 0.25 m 앞(y 1.70)에서 선다
    h = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_RECOVERING, 5.0)
    assert wait_until(lambda: probe.last_state().state == RobotState.STATE_BLOCKED, 12.0)
    assert [d for _, d, _ in fake.backups] == [pytest.approx(0.12, abs=0.015)] * 2
    assert all(v == pytest.approx(0.08) for _, _, v in fake.backups)
    assert len(fake.clears) == 2
    assert not h.get_result_async().done()  # 목표는 열어 둔다(4절)
    fake.obstacle = None
    res = probe.result(h, timeout=10.0)
    assert res.result.success is True


def test_something_behind_stops_the_backup_and_the_path_is_tried_again(harness):
    fake, probe = start(harness, release_distance_m=0.0)
    fake.obstacle = (0.35, 1.95)
    fake.wall_behind_m = 0.01  # Nav2 BackUp: COLLISION_AHEAD로 끝난다
    probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: len(fake.clears) >= 1, 6.0)
    assert fake.backup_ends[0] == ("collision", 714)
    assert wait_until(lambda: len(fake.goals) >= 2, 1.0)  # 남은 경로를 다시 보냈다
    assert probe.last_state().state == RobotState.STATE_RECOVERING


def test_dock_fails_honestly_while_pode_dock_is_missing(harness):
    _, probe = start(harness)
    res = probe.result(probe.send(Dock.Goal(dock_id="WORK"), client=probe.dock))
    assert res.result.success is False


def _retreat_client(probe):
    from fleet_interfaces.action import Retreat
    from probe import C
    from rclpy.action import ActionClient
    client = ActionClient(probe, Retreat, C.name("retreat", pode="pode1"))
    assert client.wait_for_server(10.0)
    return client


def test_a_retreat_stops_the_move_and_backs_up_with_the_same_backup_code(harness):
    """계약 0.9-draft의 Retreat: 이동을 멈추고 Nav2 BackUp으로 곧게 물러난다."""
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    move = probe.send(lane_goal(["P-3", "P-4", "P-5"]))
    assert wait_until(lambda: len(fake.goals) >= 1, 5.0)
    h = probe.send(Retreat.Goal(distance_m=0.1, speed_mps=0.1), client=_retreat_client(probe))
    assert probe.result(move, timeout=5.0).result.reason == MoveTo.Result.REASON_CANCELED
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (True, "")
    assert [(d, v) for _, d, v in fake.backups] == [(pytest.approx(0.1), pytest.approx(0.1))]


def test_something_behind_ends_a_retreat_with_obstacle_behind(harness):
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    fake.wall_behind_m = 0.01
    h = probe.send(Retreat.Goal(distance_m=0.1, speed_mps=0.1), client=_retreat_client(probe))
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (False, Retreat.Result.REASON_OBSTACLE_BEHIND)


def test_the_lock_does_not_change_while_a_retreat_runs(harness):
    """W-34: 락은 멈춘 로봇에서만 바뀐다. Retreat도 진행 중인 목표다."""
    import rclpy.node
    from fleet_interfaces.action import Retreat
    from fleet_interfaces.srv import SetLock
    fake, probe = start(harness)
    caller = harness.add(rclpy.node.Node("lock_caller"))
    client = caller.create_client(SetLock, "/pode1/set_lock")
    assert client.wait_for_service(timeout_sec=5.0)
    h = probe.send(Retreat.Goal(distance_m=0.2, speed_mps=0.05), client=_retreat_client(probe))
    assert wait_until(lambda: len(fake.backups) >= 1, 5.0)
    future = client.call_async(SetLock.Request(request_id="l-1", robot_id="pode1", locked=False))
    assert wait_until(future.done, 5.0)
    assert (future.result().accepted, future.result().reason) == (False, "NOT_STOPPED")
    assert not h.get_result_async().done()
    assert probe.last_state().lock is True


def _back_path(*ids):
    from geometry_msgs.msg import Pose
    out = []
    for i in ids:
        p = Pose()
        p.position.x, p.position.y = WAYPOINTS[i]
        out.append(p)
    return out


def test_a_retreat_with_a_path_backs_along_it_on_the_reverse_controller(harness):
    """Retreat.path: 지나온 길을 거꾸로 적은 경로를 FollowPathReverse로 후진해 따른다(BackUp 없음, 돌지 않음)."""
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    h = probe.send(Retreat.Goal(distance_m=0.4, speed_mps=0.05, path=_back_path("P-4", "P-3")),
                   client=_retreat_client(probe))
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (True, "")
    assert fake.controller_ids[-1] == "FollowPathReverse" and fake.backups == []
    # 후진은 끝 방향을 보지 않는 goal checker를 쓰고, 앞으로 가는 경로는 원래 checker를 쓴다
    assert fake.goal_checker_ids[-1] == "retreat_goal_checker"
    assert set(fake.goal_checker_ids[:-1]) == {"general_goal_checker"}
    assert all(yaw == pytest.approx(math.pi / 2) for _, _, yaw in fake.path)
    assert math.dist(fake.pose[:2], WAYPOINTS["P-3"]) <= 0.05
    assert fake.pose[2] == pytest.approx(math.pi / 2)


def test_something_behind_on_the_path_ends_a_retreat_with_obstacle_behind(harness):
    """FakeNav는 RPP처럼 움직이는 쪽(후진이면 뒤)의 물체에 서고 NO_VALID_CONTROL로 끝낸다."""
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    fake.obstacle = (0.35, 1.60)  # 뒤, P-3 근처
    h = probe.send(Retreat.Goal(distance_m=0.4, speed_mps=0.05, path=_back_path("P-4", "P-3")),
                   client=_retreat_client(probe))
    res = probe.result(h, timeout=10.0).result
    assert (res.success, res.reason) == (False, Retreat.Result.REASON_OBSTACLE_BEHIND)
    assert fake.pose[1] > 1.80


def test_a_retreat_path_away_from_the_robot_fails(harness):
    from fleet_interfaces.action import Retreat
    fake, probe = start(harness)
    h = probe.send(Retreat.Goal(distance_m=0.4, speed_mps=0.05, path=_back_path("P-6", "P-5")),
                   client=_retreat_client(probe))
    res = probe.result(h, timeout=5.0).result
    assert (res.success, res.reason) == (False, Retreat.Result.REASON_FAILED)
    assert fake.goals == [] and fake.backups == []


def _runner(handles=None, pending=()):
    """ROS 클라이언트 없이 장부와 콜백만 본다."""
    import threading
    from types import SimpleNamespace

    from pode_fsm.follow_ros import FollowPathRunner
    infos, seen = [], SimpleNamespace(rejected=[], results=[], cancels=[])
    runner = object.__new__(FollowPathRunner)
    runner._node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(warn=infos.append, info=infos.append))
    runner._lock = threading.Lock()
    runner._handles, runner._pending = dict(handles or {}), set(pending)
    runner._cancel_on_accept = set()
    runner._on_rejected = seen.rejected.append
    runner._on_result = lambda *a: seen.results.append(a)
    runner._on_cancel_accepted = seen.cancels.append
    return runner, infos, seen


def _broken():
    raise RuntimeError("lost")


def test_a_follow_path_goal_whose_send_fails_counts_as_rejected():
    """future 예외로 콜백이 죽으면 그 FollowPath는 끝내 결과가 없다."""
    from types import SimpleNamespace
    runner, infos, seen = _runner(pending={3})
    runner._on_response(3, SimpleNamespace(result=_broken))
    assert seen.rejected == [3] and runner._pending == set() and infos


class _CancelFuture:
    def __init__(self, response=None, error=None):
        self.callbacks, self._response, self._error = [], response, error

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def result(self):
        if self._error:
            raise self._error
        return self._response


def test_only_an_accepted_cancel_lets_the_core_forget_a_follow_path():
    """취소 응답에 그 goal이 들어 있을 때만 core에 알린다(거절, 오류는 알리지 않는다)."""
    from action_msgs.srv import CancelGoal
    runner, infos, seen = _runner()
    yes = CancelGoal.Response(goals_canceling=[_goal_info()])
    runner._on_cancel(3, _CancelFuture(yes))
    runner._on_cancel(4, _CancelFuture(CancelGoal.Response(
        return_code=CancelGoal.Response.ERROR_REJECTED)))
    runner._on_cancel(5, _CancelFuture(error=RuntimeError("lost")))
    assert seen.cancels == [3] and infos


def test_a_cancel_answered_goal_terminated_or_unknown_also_lets_the_core_forget():
    """이미 끝났거나 서버가 모르는 goal은 돌지 않는다. 결과를 잃은 goal이 끝나 있으면
    취소는 이렇게 답하고, 그때 잊지 않으면 그 뒤 모든 Retreat가 TIMEOUT이다."""
    from action_msgs.srv import CancelGoal
    runner, _, seen = _runner(handles={3: object(), 4: object()})
    for seq, code in ((3, CancelGoal.Response.ERROR_GOAL_TERMINATED),
                      (4, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)):
        runner._on_cancel(seq, _CancelFuture(CancelGoal.Response(return_code=code)))
    assert seen.cancels == [3, 4] and not runner._handles


def _goal_info():
    from action_msgs.msg import GoalInfo
    return GoalInfo()


def test_every_follow_path_cancel_reports_its_answer():
    from types import SimpleNamespace
    futures = {}
    handles = {7: SimpleNamespace(cancel_goal_async=lambda: futures.setdefault(7,
                                                                             _CancelFuture()))}
    runner, _, _ = _runner(handles=handles, pending={8})
    runner._on_cancel = lambda seq, fut: None
    runner.cancel_all()
    assert len(futures[7].callbacks) == 1 and runner._cancel_on_accept == {8}


def test_a_lost_follow_path_result_keeps_the_goal_and_cancels_it():
    """결과를 잃어도 goal은 돌 수 있다. 핸들을 두고 취소를 보내며, core에는
    nav_lost로 알린다(이동은 aborted처럼 끝나지만 취소가 받아들여질 때까지 잊지 않는다)."""
    from types import SimpleNamespace
    futures = []
    handle = SimpleNamespace(cancel_goal_async=lambda: futures.append(_CancelFuture())
                             or futures[-1])
    runner, infos, seen = _runner(handles={3: handle})
    runner._on_cancel = lambda seq, fut: None
    runner._on_done(3, SimpleNamespace(result=_broken))
    assert seen.results == [(3, "aborted", 0, True)]
    assert runner._handles == {3: handle} and infos
    assert len(futures) == 1 and len(futures[0].callbacks) == 1


def test_a_goal_at_a_lock_boundary_publishes_a_speed_limit_and_lifts_it_at_the_end(harness):
    """horizon 최소판: final_goal이 거짓이면 끝 근처에서 speed_limit으로 줄이고 목표가 끝나면 푼다."""
    fake, probe = start(harness)
    goal = lane_goal(["P-3", "P-4"])
    goal.final_goal = False
    res = probe.result(probe.send(goal))
    assert res.result.success is True
    assert wait_until(lambda: fake.speed_limits and fake.speed_limits[-1] == 0.0, 2.0)
    assert any(0.0 < v < 0.3 for v in fake.speed_limits)


def test_the_speed_limit_latency_margin_is_a_node_parameter(harness):
    from pode_fsm.core import FsmParams
    from pode_fsm.node import PodeFsmNode

    def node(**overrides):
        return harness.add(PodeFsmNode(parameter_overrides=harness.params(
            robot_id="pode1", contract_file=str(CONTRACT), **overrides)))
    assert node().get_parameter("horizon_latency_s").value == FsmParams().horizon_latency_s
    assert node(horizon_latency_s=0.1)._fsm.p.horizon_latency_s == 0.1


def test_a_final_goal_publishes_no_speed_limit(harness):
    fake, probe = start(harness)
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    time.sleep(0.3)
    assert all(v == 0.0 for v in fake.speed_limits), "처음에 푸는 0만 있다"


def test_without_a_through_controller_every_goal_uses_the_normal_controller(harness):
    """through_controller_id가 비면 옛 동작 그대로다."""
    fake, probe = start(harness)
    goal = lane_goal(["P-3", "P-4"])
    goal.final_goal = False
    assert probe.result(probe.send(goal)).result.success is True
    assert fake.controller_ids and set(fake.controller_ids) == {"FollowPath"}


def test_a_goal_at_a_lock_boundary_uses_the_through_controller(harness):
    """끝이 최종이 아닌 goal은 접근 감속이 없는 둘째 RPP로 가고 sqrt(2 a d) 속도 제한만 쓴다."""
    fake, probe = start(harness, through_controller_id="FollowPathThrough")
    goal = lane_goal(["P-3", "P-4"])
    goal.final_goal = False
    assert probe.result(probe.send(goal)).result.success is True
    assert fake.controller_ids and set(fake.controller_ids) == {"FollowPathThrough"}
    assert any(0.0 < v < 0.3 for v in fake.speed_limits)


def test_a_final_goal_keeps_the_normal_controller_with_a_through_controller_set(harness):
    fake, probe = start(harness, through_controller_id="FollowPathThrough")
    assert probe.result(probe.send(lane_goal(["P-3", "P-4"]))).result.success is True
    assert fake.controller_ids and set(fake.controller_ids) == {"FollowPath"}


def test_a_final_goal_replacing_a_partial_one_switches_back_to_the_normal_controller(harness):
    fake, probe = start(harness, through_controller_id="FollowPathThrough")
    partial = lane_goal(["P-3", "P-4", "P-5"])
    partial.final_goal = False
    probe.send(partial)
    assert wait_until(lambda: fake.controller_ids, 5.0)
    res = probe.result(probe.send(lane_goal(["P-3", "P-4", "P-5"])))
    assert res.result.success is True
    assert fake.controller_ids[0] == "FollowPathThrough"
    assert fake.controller_ids[-1] == "FollowPath"
    assert not fake.cancels, "the switch replaces the goal without stopping (guide 2.1)"
