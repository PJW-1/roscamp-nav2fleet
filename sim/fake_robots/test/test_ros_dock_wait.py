"""실제 Dock 액션으로 실물 시험용 PC 도킹 대역(real_dock_wait)을 시험한다."""
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_interfaces.action import Dock  # noqa: E402
from fleet_interfaces.msg import RobotState  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)
# v7 WORK 도크 지점 W는 지도에서 읽는다
# (2026-11-18 저녁 W x 1.88 -> 1.86: 2 cm 동쪽 보고가 4 cm 오차가 돼 실패했다)
WORK_DOCK = load_fleet_map("v7").node_pose("W")


def start(harness, tmp_path, robots="pode1 pode2", duration=0.6, **extra):
    import rclpy.node
    from fake_robots.real_dock_wait_node import RealDockWaitNode
    scn = write_scenario(tmp_path, map="v7", dock={"duration_s": duration, "error_cm": 1.0})
    harness.add(RealDockWaitNode(parameter_overrides=harness.params(
        robots=robots, scenario_file=scn, map_file="v7", contract_file=str(CONTRACT), **extra)))
    probe = harness.add(rclpy.node.Node("dock_probe"))
    pubs = {r: probe.create_publisher(RobotState, C.name("robot_state", pode=r),
                                      to_ros_qos(C.qos("robot_state")))
            for r in robots.split()}
    clients = {r: ActionClient(probe, Dock, C.name("dock", pode=r)) for r in robots.split()}
    for client in clients.values():
        assert client.wait_for_server(10.0)
    return pubs, clients


def report(pub, robot, x, y):
    msg = RobotState(robot_id=robot, state="IDLE")
    msg.pose.x, msg.pose.y = x, y
    pub.publish(msg)


def dock(client, dock_id="WORK"):
    fb = []
    fut = client.send_goal_async(Dock.Goal(dock_id=dock_id), feedback_callback=fb.append)
    assert wait_until(fut.done, 5.0)
    handle = fut.result()
    assert handle.accepted
    return handle, fb


def result_of(handle, timeout=10.0):
    res = handle.get_result_async()
    assert wait_until(res.done, timeout)
    return res.result().result


def keep_reporting(pub, robot, x, y, handle_box):
    """결과가 나올 때까지 0.1 s마다 자세를 낸다(실물 state처럼)."""
    import threading
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            report(pub, robot, x, y)
            stop.wait(0.1)
    threading.Thread(target=loop, daemon=True).start()
    handle_box.append(stop)


def test_robot_at_the_dock_succeeds_after_the_wait(harness, tmp_path):
    import time
    pubs, clients = start(harness, tmp_path)
    stops = []
    keep_reporting(pubs["pode1"], "pode1", WORK_DOCK[0] + 0.02, WORK_DOCK[1], stops)
    t0 = time.monotonic()
    handle, fb = dock(clients["pode1"])
    result = result_of(handle)
    took = time.monotonic() - t0
    stops[0].set()
    assert result.success is True
    assert result.error_x_cm == pytest.approx(2.0, abs=0.01)
    assert took >= 0.6
    assert [f.feedback.phase for f in fb] == ["APPROACH", "ALIGN", "DONE"]


def test_robot_away_from_the_dock_fails(harness, tmp_path):
    pubs, clients = start(harness, tmp_path)
    stops = []
    keep_reporting(pubs["pode2"], "pode2", 0.38, 0.10, stops)
    handle, _ = dock(clients["pode2"])
    result = result_of(handle)
    stops[0].set()
    assert result.success is False


def test_no_state_fails(harness, tmp_path):
    _, clients = start(harness, tmp_path)
    handle, _ = dock(clients["pode1"])
    assert result_of(handle).success is False


def test_unknown_dock_fails_at_once(harness, tmp_path):
    pubs, clients = start(harness, tmp_path, duration=5.0)
    report(pubs["pode1"], "pode1", *WORK_DOCK)
    handle, _ = dock(clients["pode1"], dock_id="NOPE")
    assert result_of(handle, timeout=2.0).success is False


def test_robots_are_served_separately(harness, tmp_path):
    pubs, clients = start(harness, tmp_path)
    stops = []
    keep_reporting(pubs["pode1"], "pode1", *WORK_DOCK, stops)
    keep_reporting(pubs["pode2"], "pode2", 0.38, 0.10, stops)
    h1, _ = dock(clients["pode1"])
    h2, _ = dock(clients["pode2"])
    r1, r2 = result_of(h1), result_of(h2)
    for s in stops:
        s.set()
    assert (r1.success, r2.success) == (True, False)


def test_cancel_ends_the_goal(harness, tmp_path):
    pubs, clients = start(harness, tmp_path, duration=5.0)
    stops = []
    keep_reporting(pubs["pode1"], "pode1", *WORK_DOCK, stops)
    handle, _ = dock(clients["pode1"])
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 3.0)
    res = handle.get_result_async()
    assert wait_until(res.done, 3.0)
    stops[0].set()
    assert res.result().result.success is False


def test_refuses_sim_time(rclpy_session, tmp_path):
    from fake_robots.real_dock_wait_node import RealDockWaitNode
    from rclpy.parameter import Parameter
    scn = write_scenario(tmp_path, map="v7")
    with pytest.raises(RuntimeError, match="use_sim_time"):
        RealDockWaitNode(parameter_overrides=[
            Parameter("robots", value="pode1"), Parameter("scenario_file", value=scn),
            Parameter("map_file", value="v7"), Parameter("use_sim_time", value=True)])


def test_a_newer_goal_replaces_the_older_one(harness, tmp_path):
    pubs, clients = start(harness, tmp_path, duration=1.0)
    stops = []
    keep_reporting(pubs["pode1"], "pode1", *WORK_DOCK, stops)
    first, _ = dock(clients["pode1"])
    second, _ = dock(clients["pode1"])
    r1, r2 = result_of(first), result_of(second)
    stops[0].set()
    assert (r1.success, r2.success) == (False, True)


def test_localizing_state_is_not_a_pose(harness, tmp_path):
    pubs, clients = start(harness, tmp_path)
    import threading
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            msg = RobotState(robot_id="pode1", state="LOCALIZING")
            msg.pose.x, msg.pose.y = WORK_DOCK
            pubs["pode1"].publish(msg)
            stop.wait(0.1)
    threading.Thread(target=loop, daemon=True).start()
    handle, _ = dock(clients["pode1"])
    result = result_of(handle)
    stop.set()
    assert result.success is False


def test_stop_ends_a_running_goal_and_waits_for_it(harness, tmp_path):
    pubs, clients = start(harness, tmp_path, duration=30.0)
    node = harness.nodes[0]
    report(pubs["pode1"], "pode1", *WORK_DOCK)
    handle, _ = dock(clients["pode1"])
    assert wait_until(lambda: node.running_goals() == 1, 3.0)
    node.stop()
    assert node.wait_idle(3.0)
    assert result_of(handle, timeout=3.0).success is False


def test_sim_time_cannot_be_switched_on_later(harness, tmp_path):
    from rclpy.parameter import Parameter
    start(harness, tmp_path)
    node = harness.nodes[0]
    results = node.set_parameters([Parameter("use_sim_time", value=True)])
    assert results[0].successful is False
    assert node.get_parameter("use_sim_time").value is False


def test_reports_a_second_dock_server_for_the_same_robot(harness, tmp_path):
    import rclpy.node
    from rclpy.action import ActionServer
    start(harness, tmp_path)
    node = harness.nodes[0]
    assert node.other_dock_servers() == []
    robot_side = harness.add(rclpy.node.Node("pode_fsm_stand_in"))
    ActionServer(robot_side, Dock, C.name("dock", pode="pode2"),
                 execute_callback=lambda h: Dock.Result(success=False))
    assert wait_until(lambda: node.other_dock_servers() == ["pode2"], 10.0)


def test_losing_localization_drops_the_last_pose(harness, tmp_path):
    pubs, clients = start(harness, tmp_path, duration=1.0)
    node = harness.nodes[0]

    def stored():  # 도크에서 보낸 정상 자세가 노드에 들어갔는지
        report(pubs["pode1"], "pode1", *WORK_DOCK)
        return "pode1" in node._pose
    assert wait_until(stored, 5.0)
    import threading
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            pubs["pode1"].publish(RobotState(robot_id="pode1", state="LOCALIZING"))
            stop.wait(0.1)
    handle, _ = dock(clients["pode1"])
    threading.Thread(target=loop, daemon=True).start()  # 기다리는 동안 위치를 잃는다
    # 시간 초과가 아니라 자세를 버려서 실패해야 한다
    dropped = wait_until(lambda: "pode1" not in node._pose, 5.0)
    result = result_of(handle)
    stop.set()
    assert dropped
    assert result.success is False
