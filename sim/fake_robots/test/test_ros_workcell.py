"""실제 액션으로 FakeWorkcell을 시험한다."""
import threading
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_interfaces.action import FillBasket  # noqa: E402
from fleet_interfaces.msg import ArmState, OrderItem  # noqa: E402
from fleet_interfaces.srv import ForceInput, RobotCommand, SetLock, SetMode  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)


class Cell:
    def __init__(self, harness, tmp_path, node_params=None, **scenario):
        import rclpy.node
        from fake_robots.fake_workcell_node import FakeWorkcellNode
        scn = write_scenario(tmp_path, **scenario)
        self.node = harness.add(FakeWorkcellNode(
            parameter_overrides=harness.params(scenario_file=scn, **(node_params or {}))))
        probe = harness.add(rclpy.node.Node("cell_probe"))
        self.probe = probe
        self.client = ActionClient(probe, FillBasket, C.name("fill_basket"))
        assert self.client.wait_for_server(10.0)

    def send(self, **items):
        fb = []
        goal = FillBasket.Goal(order_id="O-1", robot_id="pode1", station_id="WORK",
                               items=[OrderItem(kind=k, count=c) for k, c in items.items()])
        fut = self.client.send_goal_async(goal, feedback_callback=fb.append)
        assert wait_until(fut.done, 5.0)
        return fut.result(), fb

    def fill(self, **items):
        handle, fb = self.send(**items)
        assert handle.accepted
        return result_of(handle), fb

    def shelf(self, arm):
        with self.node._stock_lock:
            return {k: c for k, c in self.node._stock[arm].items() if c > 0}


def result_of(handle):
    res = handle.get_result_async()
    assert wait_until(res.done, 30.0)
    return res.result().result


def counts(items):
    return {i.kind: i.count for i in items}


def test_items_from_both_arms_are_placed(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, fb = cell.fill(A=2, C=1)
    assert result.success is True and counts(result.placed) == {"A": 2, "C": 1}
    assert result.missing == [] and result.reason == ""
    assert fb and fb[-1].feedback.placed_total == 3
    assert {f.feedback.arm_id for f in fb} == {"arm_left", "arm_right"}


def test_a_kind_not_in_stock_ends_up_missing(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(A=1, Z=2)
    assert result.success is True  # 시도를 마침
    assert counts(result.placed) == {"A": 1} and counts(result.missing) == {"Z": 2}


def test_more_than_the_stock_places_what_there_is(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(B=3)  # arm_left에는 B가 하나
    assert result.success is True
    assert counts(result.placed) == {"B": 1} and counts(result.missing) == {"B": 2}


def test_nothing_in_stock_places_nothing(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(Z=1)
    assert result.success is True and result.placed == [] and counts(result.missing) == {"Z": 1}


def test_a_kind_on_both_shelves_uses_both_arms(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm_left": {"A": 1}, "arm_right": {"A": 1}})
    result, fb = cell.fill(A=2)
    assert result.success is True and counts(result.placed) == {"A": 2}
    assert {f.feedback.arm_id for f in fb} == {"arm_left", "arm_right"}
    assert cell.shelf("arm_left") == {} and cell.shelf("arm_right") == {}


def test_stock_is_used_up_across_goals(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    assert counts(cell.fill(A=3)[0].placed) == {"A": 3}
    second, _ = cell.fill(A=1)
    assert second.placed == [] and counts(second.missing) == {"A": 1}


def test_arms_never_place_at_once_but_do_pick_at_once(harness, tmp_path):
    from fake_robots.fake_workcell_node import STATE_PICKING, STATE_PLACING
    cell = Cell(harness, tmp_path, stock={"arm_left": {"A": 6}, "arm_right": {"C": 6}},
                arm={"seconds_per_item": 0.3, "swap_seconds": 0.4, "fail_rate": 0.0})
    log = []
    order = threading.Lock()
    set_state = cell.node._set_state

    def recorded(arm, state):
        with order:
            set_state(arm, state)
            log.append((time.monotonic(), arm, state))

    cell.node._set_state = recorded
    result, _ = cell.fill(A=4, C=4)
    assert counts(result.placed) == {"A": 4, "C": 4}
    arms = ("arm_left", "arm_right")
    started = {}
    spans = {arm: [] for arm in arms}
    picking_overlap = False
    state_now = dict.fromkeys(arms, "")
    for t, arm, state in log:
        if state == STATE_PLACING and arm not in started:
            started[arm] = t
        elif state != STATE_PLACING and arm in started:
            spans[arm].append((started.pop(arm), t))
        state_now[arm] = state
        picking_overlap |= all(now == STATE_PICKING for now in state_now.values())
    assert len(spans["arm_left"]) == 4 and len(spans["arm_right"]) == 4
    for a_start, a_end in spans["arm_left"]:
        for b_start, b_end in spans["arm_right"]:
            assert a_end <= b_start or b_end <= a_start, "both arms were PLACING at once"
    assert picking_overlap, "the arms never picked in parallel"


def test_the_lock_is_what_keeps_them_apart(harness, tmp_path):
    """다른 쪽이 잠금을 쥐면 아무도 바구니에 놓지 못한다."""
    cell = Cell(harness, tmp_path)
    assert cell.node._basket_space.acquire(timeout=1.0)
    handle, _ = cell.send(A=1, C=1)
    assert handle.accepted
    done = handle.get_result_async()
    assert not wait_until(done.done, 1.0)
    cell.node._basket_space.release()
    assert wait_until(done.done, 10.0)
    assert counts(done.result().result.placed) == {"A": 1, "C": 1}


def test_arm_fail_returns_failure_with_nothing_placed(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm_left", "type": "arm_fail", "count": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = cell.fill(A=2)
    assert result.success is False and result.placed == [] and result.reason


def test_arm_fail_returns_what_was_placed_so_far(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm_left", "type": "arm_fail", "count": 1, "after_items": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = cell.fill(A=2)
    assert result.success is False and counts(result.placed) == {"A": 1}
    assert result.missing == []
    assert cell.shelf("arm_left") == {"A": 2, "B": 1}


def test_arm_fail_runs_out_after_its_count(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm_left", "type": "arm_fail", "count": 3}],
        expect={"orders_shipped_min": 1})
    assert [cell.fill(A=1)[0].success for _ in range(4)] == [False, False, False, True]


def test_arm_fail_on_the_other_arm_is_not_used_up_by_this_order(harness, tmp_path):
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm_right", "type": "arm_fail", "count": 1}],
        expect={"orders_shipped_min": 1})
    assert cell.fill(A=1)[0].success is True
    assert cell.fill(C=1)[0].success is False


def test_fail_rate_returns_failure(harness, tmp_path):
    cell = Cell(harness, tmp_path,
                arm={"seconds_per_item": 0.2, "swap_seconds": 0.4, "fail_rate": 1.0})
    result, _ = cell.fill(A=2)
    assert result.success is False and result.reason
    assert sum(i.count for i in result.placed) < 2


def test_cancel_returns_canceled_with_what_was_placed(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm_left": {"A": 6}},
                arm={"seconds_per_item": 0.4, "swap_seconds": 0.4, "fail_rate": 0.0})
    handle, fb = cell.send(A=5)
    assert handle.accepted
    # 적어도 하나는 바구니에 있다 (피드백은 CAPTURING부터 온다)
    assert wait_until(lambda: any(f.feedback.placed_total for f in fb), 10.0)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    result = result_of(handle)
    assert result.success is False and result.reason == "CANCELED"
    assert 1 <= counts(result.placed)["A"] < 5 and result.missing == []
    # 잃는 것은 없다. 바구니에 못 닿은 것은 선반에 남는다.
    assert cell.shelf("arm_left") == {"A": 6 - counts(result.placed)["A"]}


def test_slow_cancel_keeps_the_server_busy_until_its_result(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm_left": {"A": 6}},
                arm={"seconds_per_item": 0.2, "swap_seconds": 0.4, "cancel_seconds": 1.5})
    handle, fb = cell.send(A=5)
    assert handle.accepted
    assert wait_until(lambda: fb, 10.0)
    canceled_at = time.monotonic()
    assert wait_until(handle.cancel_goal_async().done, 5.0)
    time.sleep(0.6)  # 팔 스레드는 이미 멈췄고 취소는 그래도 처리된다
    second, _ = cell.send(A=1)
    assert not second.accepted, "busy while the arm stops"
    result = result_of(handle)
    assert time.monotonic() - canceled_at >= 1.4
    assert result.success is False and result.reason == "CANCELED"
    third, _ = cell.send(A=1)
    assert third.accepted, "free again once the result is out"


def test_a_second_goal_is_rejected_while_busy(harness, tmp_path):
    cell = Cell(harness, tmp_path, arm={"seconds_per_item": 0.6, "swap_seconds": 0.4})
    first, _ = cell.send(A=1)
    assert first.accepted
    second, _ = cell.send(A=1)
    assert not second.accepted
    assert result_of(first).success is True



# ---- 실물 Phody처럼: 팔마다 set_mode, 상태의 mode와 boot_id, 모드가 안 맞으면 담기 거절 ----


def set_mode(node, arm, mode, request_id=""):
    client = node.create_client(SetMode, C.name("robot_set_mode", robot=arm))
    assert client.wait_for_service(10.0)
    fut = client.call_async(SetMode.Request(request_id=request_id, mode=mode))
    assert wait_until(fut.done, 5.0)
    return fut.result()


def arm_states(cell):
    seen = {}
    for arm in ("arm_left", "arm_right"):
        cell.probe.create_subscription(
            ArmState, C.name("arm_state", arm=arm),
            lambda m, a=arm: seen.setdefault(a, []).append(m), to_ros_qos(C.qos("arm_state")))
    return seen


def test_each_arm_starts_unknown_and_takes_a_mode_like_pode_fsm(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    seen = arm_states(cell)
    assert wait_until(lambda: len(seen) == 2, 5.0)
    first = seen["arm_left"][-1]
    assert first.mode == "UNKNOWN" and seen["arm_right"][-1].mode == "UNKNOWN"
    assert len(first.boot_id) == 36
    ok = set_mode(cell.probe, "arm_left", "NORMAL", "m1")
    assert ok.accepted and ok.request_id == "m1"
    assert wait_until(lambda: seen["arm_left"][-1].mode == "NORMAL", 5.0)
    assert seen["arm_left"][-1].last_request_id == "m1"
    assert seen["arm_right"][-1].mode == "UNKNOWN", "one arm at a time"
    assert seen["arm_left"][-1].boot_id == first.boot_id
    bad = set_mode(cell.probe, "arm_right", "FAST", "m2")
    assert not bad.accepted and bad.reason == "MODE_MISMATCH"
    again = set_mode(cell.probe, "arm_left", "BOGUS", "m1")
    assert again.accepted, "the same request_id gets the same answer"


def test_with_the_mode_gate_a_fill_before_set_mode_aborts_with_mode_mismatch(harness, tmp_path):
    cell = Cell(harness, tmp_path, node_params={"mode_gate": True})
    handle, _ = cell.send(A=2, C=1)
    assert handle.accepted
    result = result_of(handle)
    assert result.success is False and result.reason == "MODE_MISMATCH"
    assert result.placed == [] and counts(result.missing) == {"A": 2, "C": 1}
    assert cell.shelf("arm_left") == {"A": 3, "B": 1}, "nothing was picked"


def test_with_the_mode_gate_the_two_arms_must_share_one_mode(harness, tmp_path):
    cell = Cell(harness, tmp_path, node_params={"mode_gate": True})
    assert set_mode(cell.probe, "arm_left", "NORMAL").accepted
    assert set_mode(cell.probe, "arm_right", "TEST").accepted
    assert cell.fill(A=1)[0].reason == "MODE_MISMATCH"
    assert set_mode(cell.probe, "arm_right", "NORMAL").accepted
    result, _ = cell.fill(A=1)
    assert result.success is True and counts(result.placed) == {"A": 1}


def test_without_the_mode_gate_an_unknown_mode_still_fills(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    assert cell.fill(A=1)[0].success is True


def test_the_result_confirms_what_was_placed_and_nothing_is_uncertain(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    result, _ = cell.fill(A=2, C=1)
    assert counts(result.placed_confirmed) == counts(result.placed) == {"A": 2, "C": 1}
    assert result.uncertain == []


def test_a_failed_fill_confirms_what_it_placed_so_far(harness, tmp_path):
    failed = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm_left", "type": "arm_fail", "count": 1, "after_items": 1}],
        expect={"orders_shipped_min": 1})
    result, _ = failed.fill(A=2)
    assert counts(result.placed_confirmed) == {"A": 1} and result.uncertain == []


def test_an_arm_error_shows_only_after_the_result_went_out(harness, tmp_path):
    from fake_robots.fake_workcell_node import STATE_ERROR
    cell = Cell(harness, tmp_path, faults=[
        {"at_s": 0, "arm": "arm_left", "type": "arm_fail", "count": 1}],
        expect={"orders_shipped_min": 1})
    states = []
    set_state = cell.node._set_state
    cell.node._set_state = lambda arm, state: (states.append(state), set_state(arm, state))
    handle, _ = cell.send(A=1)
    res = handle.get_result_async()
    assert wait_until(res.done, 30.0)
    assert STATE_ERROR not in states, "the fleet gets the result first and retries"
    assert wait_until(lambda: STATE_ERROR in states, 5.0)


# ---- 팔마다 관리 명령 서버: set_lock · fault_resolved (IR-21 · IR-61) ----


def call(node, srv, name, **request):
    client = node.create_client(srv, name)
    assert client.wait_for_service(10.0)
    fut = client.call_async(srv.Request(**request))
    assert wait_until(fut.done, 5.0)
    return fut.result()


def set_lock(cell, arm, locked, request_id, confirm_drop=False):
    return call(cell.probe, SetLock, C.name("robot_set_lock", robot=arm), request_id=request_id,
                robot_id=arm, locked=locked, confirm_drop=confirm_drop)


def test_a_stopped_arm_releases_its_lock_and_the_state_shows_it(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    seen = arm_states(cell)
    assert wait_until(lambda: len(seen) == 2 and seen["arm_left"][-1].lock, 5.0)
    res = set_lock(cell, "arm_left", False, "l1")
    assert res.accepted and res.request_id == "l1"
    assert wait_until(lambda: not seen["arm_left"][-1].lock, 5.0)
    assert seen["arm_left"][-1].last_request_id == "l1"
    assert seen["arm_right"][-1].lock, "one arm at a time"
    # 같은 번호는 같은 답, 다시 반영하지 않는다
    assert set_lock(cell, "arm_left", True, "l1").accepted
    time.sleep(0.5)
    assert not seen["arm_left"][-1].lock


def test_a_fill_is_rejected_while_either_arm_is_released(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    assert set_lock(cell, "arm_right", False, "l1").accepted
    handle, _ = cell.send(A=1)
    assert not handle.accepted
    assert cell.shelf("arm_left") == {"A": 3, "B": 1}, "nothing was picked"
    assert set_lock(cell, "arm_right", True, "l2").accepted
    assert cell.fill(A=1)[0].success is True


def test_an_arm_holding_an_item_needs_confirm_drop(harness, tmp_path):
    from fake_robots.fake_workcell_node import STATE_PLACING
    cell = Cell(harness, tmp_path)
    cell.node._set_state("arm_left", STATE_PLACING)
    res = set_lock(cell, "arm_left", False, "l1")
    assert (res.accepted, res.reason) == (False, "HOLDING_ITEM")
    res = set_lock(cell, "arm_left", False, "l2", confirm_drop=True)
    assert (res.accepted, res.reason) == (False, "NOT_STOPPED")
    assert cell.node._admin["arm_left"].lock


def test_fault_resolved_clears_the_error_of_that_arm(harness, tmp_path):
    from fake_robots.fake_workcell_node import STATE_ERROR, STATE_IDLE
    cell = Cell(harness, tmp_path)
    cell.node._show_errors(["arm_left", "arm_right"])
    res = call(cell.probe, RobotCommand, C.name("arm_fault_resolved", arm="arm_left"),
               request_id="r1", robot_id="arm_left")
    assert res.accepted and res.request_id == "r1"
    assert cell.node._state == {"arm_left": STATE_IDLE, "arm_right": STATE_ERROR}


def test_an_accepted_fill_keeps_the_arms_from_releasing_before_they_move(harness, tmp_path):
    from rclpy.action import GoalResponse
    cell = Cell(harness, tmp_path)
    try:
        assert cell.node._on_goal(None) == GoalResponse.ACCEPT  # 받았지만 두 팔은 아직 IDLE
        res = set_lock(cell, "arm_left", False, "l1")
        assert (res.accepted, res.reason) == (False, "NOT_STOPPED")
        assert cell.node._admin["arm_left"].lock
    finally:
        cell.node._busy = cell.node._moving = False


def test_an_accepted_fill_keeps_an_arm_in_error_from_releasing(harness, tmp_path):
    from rclpy.action import GoalResponse
    cell = Cell(harness, tmp_path)
    cell.node._show_errors(["arm_left"])
    try:
        assert cell.node._on_goal(None) == GoalResponse.ACCEPT  # ERROR여도 받고 움직인다
        res = set_lock(cell, "arm_left", False, "l1")
        assert (res.accepted, res.reason) == (False, "NOT_STOPPED")
    finally:
        cell.node._busy = cell.node._moving = False


def test_an_arm_that_stopped_during_a_slow_cancel_can_release(harness, tmp_path):
    cell = Cell(harness, tmp_path, stock={"arm_left": {"A": 6}},
                arm={"seconds_per_item": 0.2, "swap_seconds": 0.4, "cancel_seconds": 1.5})
    handle, fb = cell.send(A=5)
    assert wait_until(lambda: fb, 10.0)
    assert wait_until(handle.cancel_goal_async().done, 5.0)
    time.sleep(0.6)  # 팔 스레드는 멈췄고 서버는 아직 바쁘다
    assert cell.node._busy
    assert set_lock(cell, "arm_left", False, "l1").accepted
    assert result_of(handle).reason == "CANCELED"


def test_force_on_one_arm_needs_the_test_mode_and_shows_in_its_state(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    seen = arm_states(cell)
    assert wait_until(lambda: len(seen) == 2, 5.0)

    def force(request_id, kind="CONDITION", value="1"):
        return call(cell.probe, ForceInput, C.name("robot_force", robot="arm_right"),
                    request_id=request_id, robot_id="arm_right", kind=kind, key="grip_fail",
                    value=value)

    assert (force("f1").accepted, force("f1").reason) == (False, "NOT_TEST_MODE")
    assert set_mode(cell.probe, "arm_right", "TEST", "m1").accepted
    assert force("f2").accepted
    assert wait_until(lambda: list(seen["arm_right"][-1].forced) == ["grip_fail"], 5.0)
    assert list(seen["arm_left"][-1].forced) == [], "one arm at a time"
    assert force("f3", kind="CLEAR", value="").accepted
    assert wait_until(lambda: list(seen["arm_right"][-1].forced) == [], 5.0)
