"""실제 액션으로 FakeWorkcell을 시험한다."""
import threading
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract  # noqa: E402
from fleet_interfaces.action import FillBasket  # noqa: E402
from fleet_interfaces.msg import OrderItem  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

C = load_contract(CONTRACT)


class Cell:
    def __init__(self, harness, tmp_path, **scenario):
        import rclpy.node
        from fake_robots.fake_workcell_node import FakeWorkcellNode
        scn = write_scenario(tmp_path, **scenario)
        self.node = harness.add(FakeWorkcellNode(
            parameter_overrides=harness.params(scenario_file=scn)))
        probe = harness.add(rclpy.node.Node("cell_probe"))
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
    assert wait_until(lambda: fb, 10.0)  # 적어도 하나는 바구니에 있다
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
