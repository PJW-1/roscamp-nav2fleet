"""팔 관제 합의(2026-11-27)를 FakeWorkcell 시험 훅으로 재현한다."""
import itertools
import threading
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import wait_until  # noqa: E402
from fleet_core.common.contract import to_ros_qos  # noqa: E402
from fleet_interfaces.action import FillBasket  # noqa: E402
from fleet_interfaces.msg import ArmState  # noqa: E402
from fleet_interfaces.srv import ForceInput, RobotCommand, SetPause  # noqa: E402
from test_ros_workcell import C, Cell, call, counts, result_of, set_lock, set_mode  # noqa: E402

R = FillBasket.Result
F = FillBasket.Feedback
ARMS = ("arm_left", "arm_right")
LEFT_ONLY = {"arm_left": {"A": 6}}
_ids = itertools.count(1)


class States:
    """팔마다 받은 ArmState와 받은 시각."""

    def __init__(self, cell):
        self.msgs = {arm: [] for arm in ARMS}
        self.lock = threading.Lock()
        for arm in ARMS:
            cell.probe.create_subscription(
                ArmState, C.name("arm_state", arm=arm),
                lambda m, a=arm: self._add(a, m), to_ros_qos(C.qos("arm_state")))
        assert wait_until(lambda: all(self.msgs.values()), 5.0)

    def _add(self, arm, msg):
        with self.lock:
            self.msgs[arm].append((time.monotonic(), msg))

    def last(self, arm):
        with self.lock:
            return self.msgs[arm][-1][1]

    def since(self, arm, t):
        with self.lock:
            return [(at, m) for at, m in self.msgs[arm] if at >= t]

    def wait(self, arm, pred, timeout=5.0):
        return wait_until(lambda: pred(self.last(arm)), timeout)


def enter_test_mode(cell):
    for arm in ARMS:
        assert set_mode(cell.probe, arm, "TEST").accepted


def force(cell, arm, key, value=""):
    return call(cell.probe, ForceInput, C.name("robot_force", robot=arm),
                request_id=f"f{next(_ids)}", robot_id=arm, kind="CONDITION", key=key,
                value=value)


def set_pause(cell, arm, paused, request_id=None):
    return call(cell.probe, SetPause, C.name("arm_set_pause", arm=arm),
                request_id=request_id or f"p{next(_ids)}", paused=paused)


def fault_resolved(cell, arm):
    return call(cell.probe, RobotCommand, C.name("arm_fault_resolved", arm=arm),
                request_id=f"r{next(_ids)}", robot_id=arm)


def slow_cell(harness, tmp_path, seconds=0.8, **kw):
    return Cell(harness, tmp_path, stock=LEFT_ONLY,
                arm={"seconds_per_item": seconds, "swap_seconds": 0.4, "fail_rate": 0.0}, **kw)


def phases(fb, arm="arm_left"):
    return [f.feedback.phase for f in fb if f.feedback.arm_id == arm]


def returned_totals(fb):
    return [f.feedback.placed_total for f in fb if f.feedback.phase == F.PHASE_RETURNING]


# ---- 규칙 1: 결과 사유 ----


@pytest.mark.parametrize("reason", [R.REASON_PICK_FAILED, R.REASON_RECOGNITION_ERROR,
                                    R.REASON_OUT_OF_STOCK, R.REASON_ARM_FAULT])
def test_fill_fail_ends_the_goal_with_that_reason_and_what_was_placed(harness, tmp_path,
                                                                      reason):
    cell = Cell(harness, tmp_path)
    enter_test_mode(cell)
    assert force(cell, "arm_left", "fill_fail", f"{reason}@1").accepted
    result, _ = cell.fill(A=2)
    assert (result.success, result.reason) == (False, reason)
    assert counts(result.placed) == {"A": 1} and result.missing == []
    assert result.uncertain == []
    assert cell.shelf("arm_left") == {"A": 2, "B": 1}, "the failed item stays on the shelf"
    assert "fill_fail" not in cell.node._admin["arm_left"].forced_values, "used once"
    assert cell.fill(A=1)[0].success is True


def test_a_bad_hook_value_is_refused(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    enter_test_mode(cell)
    bad = force(cell, "arm_left", "fill_fail", "BOGUS")
    assert (bad.accepted, bad.reason) == (False, "NOT_FOUND")
    assert cell.node._admin["arm_left"].forced_values == {}
    assert force(cell, "arm_left", "restart", "nap=1").reason == "NOT_FOUND"
    assert force(cell, "arm_left", "recover").reason == "NOT_FOUND", "not PAUSED"


def test_not_ready_refuses_the_next_goals_at_once(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    enter_test_mode(cell)
    assert force(cell, "arm_right", "not_ready", "2").accepted
    for _ in range(2):
        result, fb = cell.fill(A=1, C=1)
        assert (result.success, result.reason) == (False, R.REASON_NOT_READY)
        assert result.placed == [] and fb == []
    assert cell.shelf("arm_left") == {"A": 3, "B": 1} and cell.shelf("arm_right") == {"C": 3}
    assert cell.fill(A=1)[0].success is True


def test_an_unknown_item_code_refuses_the_whole_goal(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    enter_test_mode(cell)
    assert force(cell, "arm_left", "unknown_items", "Z,Y").accepted
    result, fb = cell.fill(A=1, Z=1)
    assert (result.success, result.reason) == (False, R.REASON_UNKNOWN_TARGET)
    assert result.placed == [] and fb == []
    assert cell.shelf("arm_left") == {"A": 3, "B": 1}
    assert cell.fill(A=1)[0].success is True, "known codes still fill"


def test_unknown_items_also_come_from_a_parameter(harness, tmp_path):
    cell = Cell(harness, tmp_path, node_params={"unknown_items": "Q"})
    assert cell.fill(Q=1)[0].reason == R.REASON_UNKNOWN_TARGET
    assert cell.fill(Z=1)[0].success is True, "not stocked but known: missing as before"


# ---- 규칙 2: 피드백과 상태 ----


def test_feedback_walks_the_phases_and_the_state_matches(harness, tmp_path):
    cell = slow_cell(harness, tmp_path)
    states = States(cell)
    t0 = time.monotonic()
    result, fb = cell.fill(A=2)
    assert result.success is True and counts(result.placed) == {"A": 2}
    one = [F.PHASE_CAPTURING, F.PHASE_PICKING, F.PHASE_TO_BASKET, F.PHASE_PLACING,
           F.PHASE_RETURNING]
    assert phases(fb) == one * 2
    assert [f.feedback.placed_total for f in fb] == [0, 0, 0, 0, 1, 1, 1, 1, 1, 2]
    during = [m for _, m in states.since("arm_left", t0) if m.order_id == "O-1"]
    assert during, "the state carries the trip id while filling"
    assert {m.placed_total for m in during} <= {0, 1, 2}
    assert any(m.placed_total == 1 for m in during)
    held = [m for m in during if m.holding]
    assert held and all(m.gripper == ArmState.GRIPPER_CLOSED for m in held)
    assert wait_until(lambda: states.last("arm_left").order_id == "", 2.0)
    assert not states.last("arm_left").holding


# ---- 규칙 3 · 4: 스스로 멈춤, [복구], set_pause ----


def test_a_self_stop_keeps_the_goal_and_set_pause_false_goes_on(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=0.4)
    states = States(cell)
    enter_test_mode(cell)
    assert force(cell, "arm_left", "self_stop", "1").accepted
    handle, fb = cell.send(A=3)
    assert handle.accepted
    assert states.wait("arm_left", lambda m: m.state == ArmState.STATE_PAUSED)
    done = handle.get_result_async()
    seen = len(fb)
    time.sleep(1.0)
    assert not done.done() and len(fb) == seen, "alive, no progress"
    assert states.last("arm_left").placed_total == 1
    assert force(cell, "arm_left", "recover").accepted
    time.sleep(0.6)
    assert states.last("arm_left").state == ArmState.STATE_PAUSED, "[복구] stays PAUSED"
    assert not done.done()
    res = set_pause(cell, "arm_left", False, "go1")
    assert res.accepted and res.request_id == "go1"
    assert states.wait("arm_left", lambda m: m.state != ArmState.STATE_PAUSED)
    result = result_of(handle)
    assert result.success is True and counts(result.placed) == {"A": 3}
    assert returned_totals(fb) == [1, 2, 3], "the same goal goes on"


def test_set_pause_true_pauses_mid_fill_and_false_resumes_the_same_goal(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=0.4)
    states = States(cell)
    handle, fb = cell.send(A=3)
    assert wait_until(lambda: returned_totals(fb), 10.0)
    res = set_pause(cell, "arm_left", True, "s1")
    assert res.accepted
    assert states.wait("arm_left", lambda m: m.state == ArmState.STATE_PAUSED)
    done = handle.get_result_async()
    seen = len(fb)
    time.sleep(0.8)
    assert not done.done() and len(fb) == seen
    assert set_pause(cell, "arm_left", True, "s1").accepted, "same id, same answer"
    assert set_pause(cell, "arm_left", False).accepted
    assert states.wait("arm_left", lambda m: m.state != ArmState.STATE_PAUSED)
    result = result_of(handle)
    assert result.success is True and counts(result.placed) == {"A": 3}
    assert returned_totals(fb) == [1, 2, 3]


# ---- 규칙 5: [초기화] ----


def test_a_restart_drops_the_goal_silently_and_comes_back_fresh(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=2.0)
    states = States(cell)
    enter_test_mode(cell)
    old_boot = states.last("arm_left").boot_id
    handle, fb = cell.send(A=2)
    assert wait_until(lambda: F.PHASE_TO_BASKET in phases(fb), 10.0)
    t0 = time.monotonic()
    assert force(cell, "arm_left", "restart", "gap=1.0,holding=1").accepted
    assert wait_until(lambda: states.last("arm_left").boot_id != old_boot, 5.0)
    msgs = states.since("arm_left", t0)
    assert any(m.holding and m.boot_id == old_boot for _, m in msgs), "holding right before"
    gaps = [b[0] - a[0] for a, b in zip(msgs, msgs[1:])]
    assert max(gaps) >= 0.9, "no state during the restart"
    fresh = states.last("arm_left")
    assert (fresh.state, fresh.order_id, fresh.placed_total, fresh.holding) == (
        ArmState.STATE_IDLE, "", 0, False)
    assert fresh.mode == "UNKNOWN" and fresh.lock, "boot values"
    assert states.last("arm_right").boot_id == fresh.boot_id
    done = handle.get_result_async()
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    assert list(cancel.result().goals_canceling) == [], "the new process does not know it"
    time.sleep(1.0)
    assert not done.done(), "no result for the dropped goal"
    result, _ = cell.fill(A=1)
    assert result.success is True and counts(result.placed) == {"A": 1}
    assert not done.done()


def test_a_late_result_finishes_the_old_goal_after_the_restart(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=2.0)
    states = States(cell)
    enter_test_mode(cell)
    old_boot = states.last("arm_left").boot_id
    handle, fb = cell.send(A=2)
    assert wait_until(lambda: fb, 10.0)
    t0 = time.monotonic()
    assert force(cell, "arm_right", "restart", "gap=0.5,late_result=1.5").accepted
    assert wait_until(lambda: states.last("arm_left").boot_id != old_boot, 5.0)
    second, _ = cell.send(A=1)
    assert second.accepted, "takes a new goal after the restart"
    old = handle.get_result_async()
    assert wait_until(old.done, 5.0)
    assert time.monotonic() - t0 >= 1.4
    assert old.result().result.success is True
    assert result_of(second).success is True


# ---- 규칙 6: Phody 앱 락 해제 ----


def test_an_app_unlock_while_holding_pauses_and_the_item_becomes_uncertain(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=2.0)
    states = States(cell)
    enter_test_mode(cell)
    handle, fb = cell.send(A=2)
    assert wait_until(lambda: F.PHASE_TO_BASKET in phases(fb), 10.0)
    assert force(cell, "arm_left", "app_unlock").accepted
    assert states.wait("arm_left", lambda m: (m.lock, m.state, m.holding) == (
        False, ArmState.STATE_PAUSED, False))
    assert states.last("arm_left").gripper == ArmState.GRIPPER_OPEN
    res = set_pause(cell, "arm_left", False)
    assert (res.accepted, res.reason) == (False, "LOCK_RELEASED")
    assert set_lock(cell, "arm_left", True, "l1").accepted, "a PAUSED arm counts as stopped"
    assert set_pause(cell, "arm_left", False).accepted
    result = result_of(handle)
    assert counts(result.placed) == {"A": 1} and counts(result.uncertain) == {"A": 1}
    assert result.success is True and result.reason == ""


def test_an_app_unlock_and_a_cancel_reports_the_dropped_item(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=2.0)
    enter_test_mode(cell)
    handle, fb = cell.send(A=2)
    assert wait_until(lambda: F.PHASE_PLACING in phases(fb), 10.0)
    assert force(cell, "arm_left", "app_unlock").accepted
    time.sleep(0.3)
    assert wait_until(handle.cancel_goal_async().done, 5.0)
    result = result_of(handle)
    assert result.reason == R.REASON_CANCELED and counts(result.uncertain) == {"A": 1}
    assert result.placed == []


# ---- 규칙 7: ERROR와 fault_resolved ----


def test_an_arm_fault_holds_error_until_fault_resolved(harness, tmp_path):
    cell = Cell(harness, tmp_path, node_params={"fault_resolved_delay_s": 0.5})
    states = States(cell)
    enter_test_mode(cell)
    assert force(cell, "arm_left", "fill_fail", R.REASON_ARM_FAULT).accepted
    assert cell.fill(A=1)[0].reason == R.REASON_ARM_FAULT
    assert states.wait("arm_left", lambda m: m.state == ArmState.STATE_ERROR)
    time.sleep(2.6)  # 시나리오 고장(ERROR_HOLD_S)과 달리 저절로 풀리지 않는다
    assert states.last("arm_left").state == ArmState.STATE_ERROR
    res = set_pause(cell, "arm_left", True)
    assert (res.accepted, res.reason) == (False, "NOT_READY")
    t0 = time.monotonic()
    assert fault_resolved(cell, "arm_left").accepted
    assert states.wait("arm_left", lambda m: m.state == ArmState.STATE_IDLE)
    assert time.monotonic() - t0 >= 0.4, "the delay"


def test_the_error_hook_on_an_idle_arm(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    states = States(cell)
    enter_test_mode(cell)
    assert force(cell, "arm_right", "error").accepted
    assert states.wait("arm_right", lambda m: (m.state, m.detail) == (
        ArmState.STATE_ERROR, R.REASON_ARM_FAULT))
    assert states.last("arm_left").state == ArmState.STATE_IDLE
    assert fault_resolved(cell, "arm_right").accepted
    assert states.wait("arm_right", lambda m: m.state == ArmState.STATE_IDLE)


def test_a_new_goal_does_not_clear_an_unresolved_error(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    states = States(cell)
    enter_test_mode(cell)
    assert force(cell, "arm_right", "error").accepted
    assert states.wait("arm_right", lambda m: m.state == ArmState.STATE_ERROR)
    result, _ = cell.fill(C=1)
    assert (result.success, result.reason) == (False, R.REASON_ARM_FAULT)
    assert result.placed == [] and cell.shelf("arm_right") == {"C": 3}
    assert cell.node._state["arm_right"] == ArmState.STATE_ERROR
    assert fault_resolved(cell, "arm_right").accepted  # 결과 바로 뒤
    time.sleep(0.5)
    assert states.last("arm_right").state == ArmState.STATE_IDLE, "the old fault stays gone"
    assert cell.fill(C=1)[0].success is True


def test_a_delayed_fault_resolved_does_not_clear_a_newer_fault(harness, tmp_path):
    cell = Cell(harness, tmp_path, node_params={"fault_resolved_delay_s": 0.5})
    states = States(cell)
    enter_test_mode(cell)
    assert force(cell, "arm_left", "error").accepted
    assert fault_resolved(cell, "arm_left").accepted
    assert force(cell, "arm_left", "error").accepted  # 해제 전에 새 고장
    time.sleep(1.0)
    assert states.last("arm_left").state == ArmState.STATE_ERROR


def test_the_error_hook_mid_fill_ends_it_with_arm_fault(harness, tmp_path):
    cell = slow_cell(harness, tmp_path, seconds=0.8)
    states = States(cell)
    enter_test_mode(cell)
    handle, fb = cell.send(A=3)
    assert wait_until(lambda: returned_totals(fb), 10.0)
    assert force(cell, "arm_left", "error").accepted
    result = result_of(handle)
    assert (result.success, result.reason) == (False, R.REASON_ARM_FAULT)
    assert 1 <= counts(result.placed)["A"] < 3
    assert states.wait("arm_left", lambda m: m.state == ArmState.STATE_ERROR)


# ---- 규칙 8: 상태 끊김 ----


def test_state_loss_silences_one_arm_without_a_restart(harness, tmp_path):
    cell = Cell(harness, tmp_path)
    states = States(cell)
    enter_test_mode(cell)
    boot = states.last("arm_left").boot_id
    t0 = time.monotonic()
    assert force(cell, "arm_left", "state_loss", "1.2").accepted
    time.sleep(0.2)
    t_quiet = time.monotonic()
    assert wait_until(lambda: states.since("arm_left", t_quiet), 3.0)
    back = states.since("arm_left", t_quiet)[0][0]
    assert back - t0 >= 1.0
    right = [at for at, _ in states.since("arm_right", t_quiet) if at < back]
    assert len(right) >= 3, "the other arm keeps publishing"
    assert states.last("arm_left").boot_id == boot
