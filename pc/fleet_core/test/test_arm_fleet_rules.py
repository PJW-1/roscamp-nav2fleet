"""팔 관제 합의 ④ · ⑤ · ⑥ · ⑦ · ⑪ · ⑫: 작업대 막힘, 쥔 채 락 해제, 상태 끊김, 서비스 정지,
팔 재시작과 [재개], 담기 진행 보고."""
from core_helpers import cmds, codes
from fleet_core.common.commands import (
    RESUME_REFILL,
    RESUME_SET_PAUSE_OFF,
    OrderRecord,
    ReportTripCmd,
)
from test_orchestrator import O1, bound, make

O4 = OrderRecord("O-004", {"A": 2, "B": 2})


def booted(core, boot="b1"):
    for arm in ("arm_left", "arm_right"):
        core.on_arm_state(arm, "IDLE", "", False, boot, True)
    core.take_output()


def filling(orders=(O1,)):
    core, now = make(orders)
    booted(core)
    bound(core)
    core.step("r1")
    fill = cmds(core.take_output(), "FillCmd")[0]
    return core, now, fill


def restarted(orders=(O4,), placed_total=2, holding=False):
    core, now, fill = filling(orders)
    core.on_fill_feedback(fill.goal_id, "arm_left", "PLACING", placed_total)
    if holding:
        core.on_arm_state("arm_left", "PICKING", "", True, "b1", True)
    core.take_output()
    core.on_arm_state_lost("arm_left")
    core.on_arm_state("arm_left", "IDLE", "", False, "b2", True)
    return core, now, fill, core.take_output()


def holds(out):
    return [(r.status, r.hold_reason) for r in cmds(out, "ReportOrderCmd")]


# ---- ④ 작업대 팔이 ERROR면 배차하지 않음 ----
def test_an_arm_in_error_takes_back_the_open_calls_and_opens_them_again_after_it_recovers():
    core, _ = make([O1])
    booted(core)
    core.on_arm_state("arm_right", "ERROR", "X", False, "b1", True)
    out = core.take_output()
    assert len(cmds(out, "CancelDispatchCmd")) == 1
    core.on_order_board("RUN", [O1, OrderRecord("O-002", {"B": 1})])
    assert not cmds(core.take_output(), "DispatchCmd"), "no call while the workcell is blocked"
    core.on_arm_state("arm_right", "IDLE", "", False, "b1", True)
    assert len(cmds(core.take_output(), "DispatchCmd")) == 2


def test_no_fill_goal_goes_to_a_workcell_with_an_arm_in_error():
    core, _ = make([O1])
    booted(core)
    bound(core)
    core.on_arm_state("arm_left", "ERROR", "X", False, "b1", True)
    core.step("r1")
    assert not cmds(core.take_output(), "FillCmd")
    core.on_arm_state("arm_left", "IDLE", "", False, "b1", True)
    core.step("r1")
    assert len(cmds(core.take_output(), "FillCmd")) == 1


# ---- ⑤ 담기 중 쥔 채 락 해제 → 보류 ----
def test_a_lock_released_on_a_holding_arm_during_a_fill_cancels_it_and_holds_the_order():
    core, _, fill = filling()
    core.on_arm_state("arm_left", "PAUSED", "", True, "b1", False)
    out = core.take_output()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", fill.goal_id)]
    assert holds(out) == [("HELD", "ARM_FAULT")]
    assert "ARM_LOCK_RELEASED" in codes(out)


def test_a_lock_released_on_an_empty_arm_keeps_the_fill_waiting():
    core, _, _ = filling()
    core.on_arm_state("arm_left", "PAUSED", "", False, "b1", False)
    out = core.take_output()
    assert not cmds(out, "CancelCmd") and not holds(out)


# ---- ⑥ 팔 상태가 끊기면 상태 모름: 배차를 막되 담기는 보류하지 않음 ----
def test_a_lost_arm_state_blocks_dispatch_and_alerts_without_holding_the_fill():
    core, _ = make([O1, OrderRecord("O-002", {"B": 1})])
    booted(core)
    bound(core)
    core.step("r1")
    core.take_output()
    core.on_arm_state_lost("arm_left")
    out = core.take_output()
    assert "ARM_STATE_LOST" in codes(out)
    assert len(cmds(out, "CancelDispatchCmd")) == 1
    assert not holds(out)
    core.on_arm_state("arm_left", "PICKING", "", False, "b1", True)
    out = core.take_output()
    assert "ARM_STATE_BACK" in codes(out) and len(cmds(out, "DispatchCmd")) == 1


# ---- ⑦ 서비스 정지: 담기를 시작한 운반 건만 끝까지 ----
def test_service_stop_lets_a_started_fill_go_on_to_the_outbound_station():
    core, _, fill = filling()
    core.on_order_board("PAUSE", [O1])
    assert core.carries("r1")
    core.on_fill_result(fill.goal_id, True, {"A": 1, "C": 1}, {})
    core.step("r1")
    assert [c.station_id for c in cmds(core.take_output(), "DispatchCmd") if c.robot_id] == [
        "OUTBOUND"]


def test_service_stop_starts_no_new_fill_and_takes_no_new_call():
    core, _ = make([O1])
    booted(core)
    bound(core)
    core.on_order_board("PAUSE", [O1, OrderRecord("O-002", {"B": 1})])
    assert not core.carries("r1")
    core.step("r1")
    out = core.take_output()
    assert not cmds(out, "FillCmd")
    core.on_order_board("RUN", [O1, OrderRecord("O-002", {"B": 1})])
    core.step("r1")
    assert len(cmds(core.take_output(), "FillCmd")) == 1


# ---- ⑪ 팔 재시작: [재개] 대기, 남은 상품으로 새 목표, 옛 결과는 버림 ----
def test_a_restart_during_a_fill_waits_for_resume_without_a_hold_or_a_timeout():
    core, now, _, out = restarted()
    assert "ARM_RESTARTED" in codes(out) and not holds(out)
    now[0] += 1200.0 + 30.0 + 60.0
    core.check_arm_timeouts(now[0])
    out = core.take_output()
    assert "ROBOT_TIMEOUT" not in codes(out) and not holds(out)
    core.step("r1")
    assert not cmds(core.take_output(), "FillCmd"), "the order waits for [재개]"
    assert core.order_of("r1") == "O-004"


def test_resume_after_a_restart_sends_only_what_is_left_under_the_same_trip():
    core, _, fill, _ = restarted(placed_total=2)
    assert core.on_resume_arm("arm_left") == (RESUME_REFILL, "")
    core.step("r1")
    refill = cmds(core.take_output(), "FillCmd")
    assert [(c.order_id, c.items) for c in refill] == [("O-004", {"B": 2})]
    assert refill[0].goal_id != fill.goal_id
    assert core.robot_view("r1")["placed"] == {"A": 2}


def test_a_late_result_of_the_goal_lost_in_the_restart_is_dropped():
    core, _, fill, _ = restarted(placed_total=2)
    core.on_fill_result(fill.goal_id, False, {"A": 2, "B": 1}, {}, "CANCELED")
    out = core.take_output()
    assert "ARM_LATE_RESULT" in codes(out) and not holds(out)
    assert core.robot_view("r1")["placed"] == {}
    assert core.on_resume_arm("arm_left") == (RESUME_REFILL, "")
    core.step("r1")
    assert [c.items for c in cmds(core.take_output(), "FillCmd")] == [{"B": 2}]


def test_a_restarted_trip_ships_once_the_rest_is_placed():
    core, _, _, _ = restarted(placed_total=2)
    core.on_resume_arm("workcell")
    core.step("r1")
    refill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(refill.goal_id, True, {"B": 2}, {})
    core.step("r1")
    goal = [c for c in cmds(core.take_output(), "DispatchCmd") if c.robot_id][0]
    core.on_dispatch_result(goal.goal_id, True, "r1")
    core.step("r1")
    swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_swap_result(swap.goal_id, True, True, True)
    done = cmds(core.take_output(), "ReportOrderCmd")
    assert [(r.status, r.placed) for r in done] == [("SHIPPED", {"A": 2, "B": 2})]


def test_an_arm_holding_an_item_before_the_restart_holds_the_trip():
    core, _, _, out = restarted(placed_total=1, holding=True)
    assert holds(out) == [("HELD", "ARM_FAULT")]
    assert [r.placed for r in cmds(out, "ReportOrderCmd")] == [{"A": 1}]
    assert core.on_resume_arm("arm_left") == ("", "ORDER_STATE")


def test_resume_with_the_goal_still_alive_asks_both_arms_to_go_on():
    core, _, _ = filling()
    assert core.on_resume_arm("arm_right") == (RESUME_SET_PAUSE_OFF, "")
    assert not cmds(core.take_output(), "FillCmd")


def test_resume_without_a_fill_or_for_an_unknown_arm_is_refused():
    core, _ = make([O1])
    assert core.on_resume_arm("arm_left") == ("", "ORDER_STATE")
    assert core.on_resume_arm("arm_nope") == ("", "NOT_FOUND")


# ---- ⑫ 담기 진행: 담은 수가 늘 때마다 FILLING 보고, 상태판 팔 줄 ----
def test_fill_feedback_reports_the_trip_filling_as_the_count_goes_up():
    core, _, fill = filling()
    reports = []
    for total in (0, 1, 1, 2):
        core.on_fill_feedback(fill.goal_id, "arm_left", "PLACING", total)
        reports += cmds(core.take_output(), "ReportTripCmd")
    assert reports == [ReportTripCmd("O-001", "r1", "FILLING", {"A": 1}, {"C": 1}),
                       ReportTripCmd("O-001", "r1", "FILLING", {"A": 1, "C": 1}, {})]
    assert core.arm_task("arm_left") == ("PLACING", "O-001")
    assert core.arm_task("arm_right") == ("", "O-001")


def test_progress_after_a_refill_counts_what_was_placed_before():
    core, _, _, _ = restarted(placed_total=2)
    core.on_resume_arm("arm_left")
    core.step("r1")
    refill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_feedback(refill.goal_id, "arm_right", "PLACING", 1)
    assert cmds(core.take_output(), "ReportTripCmd") == [
        ReportTripCmd("O-004", "r1", "FILLING", {"A": 2, "B": 1}, {"B": 1})]


def test_feedback_of_an_unknown_goal_is_ignored():
    core, _, fill = filling()
    core.on_fill_feedback(fill.goal_id + 99, "arm_left", "PLACING", 1)
    assert not core.take_output()
    assert core.arm_task("arm_left") == ("", "O-001")
