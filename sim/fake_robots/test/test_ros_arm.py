"""실제 액션으로 FakeArm(arm_ship)과 시뮬 시계를 시험한다."""
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_interfaces.action import SwapBasket  # noqa: E402
from fleet_interfaces.msg import ArmState  # noqa: E402
from fleet_interfaces.srv import ForceInput, SetLock, SetMode  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)


def start_arm(harness, tmp_path, **scenario):
    import rclpy.node
    from fake_robots.fake_arm_node import FakeArmNode
    scn = write_scenario(tmp_path, **scenario)
    harness.add(FakeArmNode(parameter_overrides=harness.params(arm_id="arm_ship",
                                                               scenario_file=scn)))
    probe = harness.add(rclpy.node.Node("arm_probe"))
    client = ActionClient(probe, SwapBasket, C.name("swap_basket", arm="arm_ship"))
    assert client.wait_for_server(10.0)
    return client


def send(client, take_full=True, put_empty=True):
    fb = []
    fut = client.send_goal_async(
        SwapBasket.Goal(order_id="O-1", robot_id="pode1", station_id="OUTBOUND",
                        take_full=take_full, put_empty=put_empty),
        feedback_callback=fb.append)
    assert wait_until(fut.done, 5.0)
    handle = fut.result()
    assert handle.accepted
    return handle, fb


def result_of(handle):
    res = handle.get_result_async()
    assert wait_until(res.done, 20.0)
    return res.result().result


def swap(client, **kwargs):
    handle, fb = send(client, **kwargs)
    return result_of(handle), fb


def test_full_swap_does_both_steps(harness, tmp_path):
    client = start_arm(harness, tmp_path)
    result, fb = swap(client)
    assert result.success is True and result.took_full is True and result.put_empty_done is True
    assert result.reason == ""
    assert [f.feedback.phase for f in fb] == ["TAKE_FULL", "PUT_EMPTY"]


def test_only_put_empty_leaves_take_full_alone(harness, tmp_path):
    client = start_arm(harness, tmp_path)
    result, fb = swap(client, take_full=False, put_empty=True)
    assert result.success is True and result.took_full is False and result.put_empty_done is True
    assert [f.feedback.phase for f in fb] == ["PUT_EMPTY"]


def test_only_take_full_leaves_put_empty_alone(harness, tmp_path):
    client = start_arm(harness, tmp_path)
    result, _ = swap(client, take_full=True, put_empty=False)
    assert result.success is True and result.took_full is True and result.put_empty_done is False


def test_swap_partial_takes_the_full_basket_but_fails_the_empty_one(harness, tmp_path):
    client = start_arm(harness, tmp_path, faults=[
        {"at_s": 0, "type": "swap_partial", "count": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = swap(client)
    assert result.success is False and result.took_full is True
    assert result.put_empty_done is False and result.reason
    # 재시도에는 열기 단계만 남고 고장은 이미 다 썼다.
    retry, _ = swap(client, take_full=False, put_empty=True)
    assert retry.success is True and retry.took_full is False and retry.put_empty_done is True


def test_swap_partial_with_count_left_also_fails_the_retry(harness, tmp_path):
    client = start_arm(harness, tmp_path, faults=[
        {"at_s": 0, "type": "swap_partial", "count": 2}],
        expect={"orders_shipped_min": 1})
    swap(client)
    retry, _ = swap(client, take_full=False, put_empty=True)
    assert retry.success is False and retry.took_full is False and retry.put_empty_done is False


def test_swap_partial_is_kept_for_a_goal_that_places_an_empty_basket(harness, tmp_path):
    client = start_arm(harness, tmp_path, faults=[
        {"at_s": 0, "type": "swap_partial", "count": 1}],
        expect={"orders_shipped_min": 1})
    take_only, _ = swap(client, take_full=True, put_empty=False)
    assert take_only.success is True
    put, _ = swap(client, take_full=False, put_empty=True)
    assert put.success is False


def test_fail_rate_fails_the_goal(harness, tmp_path):
    client = start_arm(harness, tmp_path, arm={
        "seconds_per_item": 0.2, "swap_seconds": 0.2, "fail_rate": 1.0})
    result, _ = swap(client)
    assert result.success is False and result.reason
    assert not (result.took_full and result.put_empty_done)


def test_cancel_reports_what_was_done(harness, tmp_path):
    client = start_arm(harness, tmp_path, arm={
        "seconds_per_item": 0.2, "swap_seconds": 4.0, "fail_rate": 0.0})
    handle, _ = send(client)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    result = result_of(handle)
    assert result.success is False and result.reason == "CANCELED"
    assert result.took_full is False and result.put_empty_done is False


def test_a_second_goal_is_rejected_while_busy(harness, tmp_path):
    client = start_arm(harness, tmp_path, arm={
        "seconds_per_item": 0.2, "swap_seconds": 1.0, "fail_rate": 0.0})
    first, _ = send(client)
    fut = client.send_goal_async(SwapBasket.Goal(take_full=True, put_empty=True))
    assert wait_until(fut.done, 5.0)
    assert not fut.result().accepted
    assert result_of(first).success is True


@pytest.mark.parametrize("arm", ["arm_left", "arm_right"])
def test_a_workspace_arm_is_refused(harness, tmp_path, arm):
    from fake_robots.fake_arm_node import FakeArmNode
    scn = write_scenario(tmp_path)
    with pytest.raises(ValueError):
        FakeArmNode(parameter_overrides=harness.params(arm_id=arm, scenario_file=scn))


def test_sim_clock_runs_at_speed_factor(harness):
    import rclpy.node
    from fake_robots.sim_clock_node import SimClockNode
    from rosgraph_msgs.msg import Clock
    harness.add(SimClockNode(parameter_overrides=harness.params(speed_factor=10.0)))
    probe = harness.add(rclpy.node.Node("clock_probe"))
    stamps = []
    probe.create_subscription(Clock, "/clock", lambda m: stamps.append(
        (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    assert wait_until(lambda: stamps, 5.0)
    time.sleep(1.0)
    (w0, s0), (w1, s1) = stamps[0], stamps[-1]
    assert (s1 - s0) / (w1 - w0) == pytest.approx(10.0, rel=0.25)


def test_sim_clock_holds_zero_during_the_start_delay(harness):
    import rclpy.node
    from fake_robots.sim_clock_node import SimClockNode
    from rosgraph_msgs.msg import Clock
    harness.add(SimClockNode(parameter_overrides=harness.params(
        speed_factor=10.0, start_delay_s=1.5)))
    probe = harness.add(rclpy.node.Node("clock_probe"))
    stamps = []
    probe.create_subscription(Clock, "/clock", lambda m: stamps.append(
        (time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    assert wait_until(lambda: stamps, 5.0)
    first_wall = stamps[0][0]
    assert stamps[0][1] == 0.0
    assert wait_until(lambda: stamps[-1][1] > 0.0, 5.0)
    assert next(w for w, s in stamps if s > 0.0) - first_wall > 0.5, "it held zero for a while"



def test_arm_ship_starts_unknown_and_takes_a_mode(harness, tmp_path):
    import rclpy.node
    start_arm(harness, tmp_path)
    probe = harness.add(rclpy.node.Node("arm_mode_probe"))
    seen = []
    probe.create_subscription(ArmState, C.name("arm_state", arm="arm_ship"), seen.append,
                              to_ros_qos(C.qos("arm_state")))
    assert wait_until(lambda: seen, 5.0)
    assert seen[-1].mode == "UNKNOWN" and len(seen[-1].boot_id) == 36
    client = probe.create_client(SetMode, C.name("robot_set_mode", robot="arm_ship"))
    assert client.wait_for_service(10.0)
    fut = client.call_async(SetMode.Request(request_id="s1", mode="TEST"))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    assert wait_until(lambda: seen[-1].mode == "TEST", 5.0)
    fut = client.call_async(SetMode.Request(request_id="s2", mode="FAST"))
    assert wait_until(fut.done, 5.0)
    assert not fut.result().accepted and fut.result().reason == "MODE_MISMATCH"


def test_arm_ship_releases_its_lock_and_refuses_a_swap_meanwhile(harness, tmp_path):
    import rclpy.node
    client = start_arm(harness, tmp_path)
    probe = harness.add(rclpy.node.Node("arm_lock_probe"))
    seen = []
    probe.create_subscription(ArmState, C.name("arm_state", arm="arm_ship"), seen.append,
                              to_ros_qos(C.qos("arm_state")))
    assert wait_until(lambda: seen and seen[-1].lock, 5.0)
    lock = probe.create_client(SetLock, C.name("robot_set_lock", robot="arm_ship"))
    assert lock.wait_for_service(10.0)

    def set_lock(locked, request_id):
        fut = lock.call_async(SetLock.Request(request_id=request_id, robot_id="arm_ship",
                                              locked=locked))
        assert wait_until(fut.done, 5.0)
        return fut.result()

    assert set_lock(False, "l1").accepted
    assert wait_until(lambda: not seen[-1].lock, 5.0)
    assert seen[-1].last_request_id == "l1"
    fut = client.send_goal_async(SwapBasket.Goal(take_full=True, put_empty=True))
    assert wait_until(fut.done, 5.0)
    assert not fut.result().accepted
    # 같은 번호는 같은 답, 다시 반영하지 않는다
    assert set_lock(True, "l1").accepted
    time.sleep(0.5)
    assert not seen[-1].lock
    assert set_lock(True, "l2").accepted
    assert result_of(send(client)[0]).success is True


def test_arm_ship_is_not_stopped_once_a_swap_is_accepted(harness, tmp_path):
    import rclpy.node
    from fake_robots.fake_arm_node import FakeArmNode
    from rclpy.action import GoalResponse
    node = harness.add(FakeArmNode(parameter_overrides=harness.params(
        arm_id="arm_ship", scenario_file=write_scenario(tmp_path))))
    probe = harness.add(rclpy.node.Node("arm_race_probe"))
    lock = probe.create_client(SetLock, C.name("robot_set_lock", robot="arm_ship"))
    assert lock.wait_for_service(10.0)
    try:
        assert node._on_goal(None) == GoalResponse.ACCEPT  # 받았지만 상태는 아직 IDLE
        fut = lock.call_async(SetLock.Request(request_id="l1", robot_id="arm_ship",
                                              locked=False))
        assert wait_until(fut.done, 5.0)
        assert (fut.result().accepted, fut.result().reason) == (False, "NOT_STOPPED")
        assert node._admin.lock
    finally:
        node._busy = False


def test_arm_ship_in_error_is_not_stopped_once_a_swap_is_accepted(harness, tmp_path):
    import rclpy.node
    from fake_robots.fake_arm_node import FakeArmNode
    from rclpy.action import GoalResponse
    node = harness.add(FakeArmNode(parameter_overrides=harness.params(
        arm_id="arm_ship", scenario_file=write_scenario(tmp_path))))
    probe = harness.add(rclpy.node.Node("arm_error_probe"))
    lock = probe.create_client(SetLock, C.name("robot_set_lock", robot="arm_ship"))
    assert lock.wait_for_service(10.0)
    node._show_error()
    try:
        assert node._on_goal(None) == GoalResponse.ACCEPT  # ERROR여도 받고 움직인다
        fut = lock.call_async(SetLock.Request(request_id="l1", robot_id="arm_ship",
                                              locked=False))
        assert wait_until(fut.done, 5.0)
        assert (fut.result().accepted, fut.result().reason) == (False, "NOT_STOPPED")
    finally:
        node._busy = False


def test_arm_ship_force_needs_the_test_mode_and_shows_in_its_state(harness, tmp_path):
    import rclpy.node
    start_arm(harness, tmp_path)
    probe = harness.add(rclpy.node.Node("arm_force_probe"))
    seen = []
    probe.create_subscription(ArmState, C.name("arm_state", arm="arm_ship"), seen.append,
                              to_ros_qos(C.qos("arm_state")))
    assert wait_until(lambda: seen, 5.0)
    clients = {srv: probe.create_client(srv, C.name(key, robot="arm_ship"))
               for srv, key in ((ForceInput, "robot_force"), (SetMode, "robot_set_mode"))}

    def ask(srv, **request):
        assert clients[srv].wait_for_service(10.0)
        fut = clients[srv].call_async(srv.Request(**request))
        assert wait_until(fut.done, 5.0)
        return fut.result()

    def force(request_id, kind="CONDITION", value="1"):
        return ask(ForceInput, request_id=request_id, robot_id="arm_ship", kind=kind,
                   key="grip_fail", value=value)

    assert force("f1").reason == "NOT_TEST_MODE"
    assert ask(SetMode, request_id="m1", mode="TEST").accepted
    assert force("f2").accepted
    assert wait_until(lambda: list(seen[-1].forced) == ["grip_fail"], 5.0)
    assert force("f3", kind="CLEAR", value="").accepted
    assert wait_until(lambda: list(seen[-1].forced) == [], 5.0)
