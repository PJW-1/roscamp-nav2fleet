"""주문 보류(설계 v2.4): 용량 초과, 하나도 못 담음, 로봇 고장, 팔 고장, 강제 출고."""
from core_helpers import cmds, codes
from fleet_core.common.commands import (
    CANCELED,
    HOLD_ARM_FAULT,
    HOLD_ROBOT_FAULT,
    DispatchCmd,
    OrderRecord,
    ReleaseCmd,
    RobotFaultCmd,
)
from fleet_core.common.params import load_params
from test_orchestrator import O1, O2, bound, filling, make, only, outbound_goal, swapping

CAPACITY = load_params().basket_capacity
BIG = OrderRecord("O-BIG", {"A": CAPACITY + 1})


def reports(out):
    return cmds(out, "ReportOrderCmd")


def calls(out):
    """이름 없는 WORK 배차(새 주문 호출)."""
    return [c for c in only(out, DispatchCmd) if not c.robot_id]


def fail_fill(core, now, fill, placed_first=None, robot="r1"):
    """채우기를 재시도 한도를 넘게 실패시킨다(재시도는 NOT_READY만). 첫 시도에만 placed_first를 담는다."""
    placed = placed_first or {}
    for _ in range(core.params.move_retry_limit):
        core.on_fill_result(fill.goal_id, False, placed, {}, "NOT_READY")
        placed = {}
        now[0] += core.params.arm_retry_wait_s
        core.step(robot)
        fill = cmds(core.take_output(), "FillCmd")[0]
    core.on_fill_result(fill.goal_id, False, placed, {}, "NOT_READY")
    return core.take_output()


def arm_fault_held(placed_first=None):
    core, now, fill = filling()
    out = fail_fill(core, now, fill, placed_first)
    return core, now, out


def time_out(core, now, limit):
    now[0] += limit + 0.1
    core.check_arm_timeouts(now[0])
    core.take_output()
    now[0] += core.params.robot_timeout_settle_s + 0.1
    core.check_arm_timeouts(now[0])
    return core.take_output()


def full_and_heading_out():
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    goal = outbound_goal(core)
    return core, now, goal


# ---- ① 바구니 용량 초과 ----


def test_an_order_over_capacity_gets_no_robot_and_one_warning():
    core, _ = make([BIG, O1])
    out = core.take_output()
    assert len(calls(out)) == 1, "only O-001 asks for a robot"
    warns = [e for e in cmds(out, "EventOut") if e.code == "ORDER_OVER_CAPACITY"]
    assert len(warns) == 1 and warns[0].level == "WARN" and "O-BIG" in warns[0].detail
    assert not reports(out)
    core.on_order_board("RUN", [BIG, O1])
    assert "ORDER_OVER_CAPACITY" not in codes(core.take_output()), "once per order"
    assert [o.order_id for o in core._pending()] == ["O-001"]


def test_an_order_over_capacity_is_never_bound():
    core, _ = make([BIG])
    core.take_output()
    assert core._open_calls == []
    assert core._pending() == []


def test_an_order_off_the_board_is_forgotten_for_the_capacity_warning():
    core, _ = make([BIG])
    core.take_output()
    core.on_order_board("RUN", [])
    core.on_order_board("RUN", [BIG])
    assert "ORDER_OVER_CAPACITY" in codes(core.take_output())


# ---- ② 하나도 못 담음 ----


def test_nothing_placed_reports_canceled_with_the_whole_order_missing():
    core, _, fill = filling((O1, O2))
    core.take_output()
    core.on_fill_result(fill.goal_id, True, {}, {"A": 1, "C": 1})
    core.step("r1")
    out = core.take_output()
    [report] = reports(out)
    assert (report.status, report.placed, report.missing) == (CANCELED, {}, {"A": 1, "C": 1})
    assert report.hold_reason == ""
    assert report.detail.startswith("nothing could be placed")
    assert only(out, ReleaseCmd) == [ReleaseCmd("r1")]
    assert core.robot_view("r1")["basket"] == "EMPTY"
    assert "O-001" not in [o.order_id for o in core._pending()], "a final report"


# ---- ③ ship_partial 없음 ----


def test_there_is_no_ship_partial_parameter():
    assert not hasattr(load_params(), "ship_partial")


def test_a_short_basket_always_ships_as_partial():
    core, now, goal = full_and_heading_out()
    assert core.robot_view("r1")["basket"] == "FULL"
    core.on_dispatch_result(goal.goal_id, True, "r1")
    core.step("r1")
    swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_swap_result(swap.goal_id, True, True, True)
    [report] = reports(core.take_output())
    assert (report.status, report.missing, report.hold_reason) == ("PARTIAL", {"C": 1}, "")


# ---- ④ 운반 중 Pode 고장 ----


def test_an_isolated_robot_with_a_filled_basket_holds_the_order_once():
    core, now, _ = full_and_heading_out()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    [report] = reports(out)
    assert (report.status, report.hold_reason) == ("HELD", HOLD_ROBOT_FAULT)
    assert (report.placed, report.missing) == ({"A": 1}, {"C": 1})
    assert "ORDER_REQUEUED" not in codes(out)
    assert not calls(out), "not worked again"
    assert core._held == {"O-001": ("r1", HOLD_ROBOT_FAULT)}
    core.on_order_board("RUN", [O1])
    assert not calls(core.take_output())
    assert core._pending() == []
    core.on_robot_isolated("r1", "GOAL_FAILED")
    assert not reports(core.take_output()), "reported once"


def test_an_isolated_robot_with_nothing_placed_puts_the_order_back():
    core, _ = make([O1])
    core.take_output()
    bound(core)
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    assert "ORDER_REQUEUED" in codes(out) and not reports(out)
    assert len(calls(out)) == 1


def test_a_cleared_robot_with_a_filled_basket_holds_the_order():
    core, _, _ = full_and_heading_out()
    core.on_robot_removed("r1")
    out = core.take_output()
    [report] = reports(out)
    assert (report.status, report.hold_reason, report.placed) == (
        "HELD", HOLD_ROBOT_FAULT, {"A": 1})
    assert not calls(out)


def test_a_loaded_robot_cleared_twice_holds_its_order_once_and_keeps_it_held():
    core, _, _ = full_and_heading_out()
    core.on_robot_removed("r1")
    assert [(r.status, r.hold_reason) for r in reports(core.take_output())] == [
        ("HELD", HOLD_ROBOT_FAULT)]
    core.on_robot_removed("r1")
    out = core.take_output()
    assert not reports(out) and "ORDER_REQUEUED" not in codes(out) and not calls(out)
    assert core._held == {"O-001": ("r1", HOLD_ROBOT_FAULT)}


def test_a_hold_is_forgotten_when_central_takes_the_order_off_the_board():
    core, _, _ = full_and_heading_out()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.on_order_board("RUN", [])
    assert core._held == {}


def test_isolation_during_a_swap_still_stops_the_arm_and_then_holds_the_order():
    core, _, swap = swapping()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("arm_ship", swap.goal_id)]
    assert not reports(out), "waits for the result"
    core.on_swap_result(swap.goal_id, False, False, False, "CANCELED")
    [report] = reports(core.take_output())
    assert (report.status, report.hold_reason) == ("HELD", HOLD_ROBOT_FAULT)


# ---- ⑤ 담는 중 Pode 고장 ----


def test_isolation_during_a_fill_waits_for_the_arm_and_holds_what_it_placed():
    core, _, fill = filling()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    assert not cmds(out, "CancelCmd") and not reports(out), "the arm goes on"
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {"C": 1})
    out = core.take_output()
    [report] = reports(out)
    assert (report.status, report.hold_reason) == ("HELD", HOLD_ROBOT_FAULT)
    assert (report.placed, report.missing) == ({"A": 1}, {"C": 1})
    assert "FILL_RETRY" not in codes(out)


def test_isolation_during_a_fill_that_placed_nothing_puts_the_order_back():
    core, _, fill = filling()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.take_output()
    core.on_fill_result(fill.goal_id, False, {}, {}, "GRIPPER")
    out = core.take_output()
    assert "ORDER_REQUEUED" in codes(out) and not reports(out)
    assert "FILL_RETRY" not in codes(out)
    assert len(calls(out)) == 1


def test_a_clear_during_a_fill_drops_the_arm_and_holds_what_was_placed_so_far():
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "NOT_READY")
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    retry = cmds(core.take_output(), "FillCmd")[0]
    core.on_robot_removed("r1")
    out = core.take_output()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("workcell", retry.goal_id)]
    [report] = reports(out)
    assert (report.status, report.hold_reason, report.placed) == (
        "HELD", HOLD_ROBOT_FAULT, {"A": 1})


# ---- ⑥ 팔 고장 ----


def test_an_arm_past_its_retries_holds_the_order_and_keeps_the_robot_at_the_station():
    core, now, out = arm_fault_held({"A": 1})
    assert "FILL_FAILED" in codes(out)
    assert not only(out, RobotFaultCmd), "the robot is fine"
    assert not only(out, ReleaseCmd)
    [report] = reports(out)
    assert (report.status, report.hold_reason) == ("HELD", HOLD_ARM_FAULT)
    assert (report.placed, report.missing) == ({"A": 1}, {"C": 1})
    assert core.order_of("r1") == "O-001", "still bound"
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    assert not core.take_output(), "the arm is not asked again"
    core.on_order_board("RUN", [O1, O2])
    assert len(calls(core.take_output())) == 1, "only O-002 asks for a robot"


def test_an_arm_that_never_answers_holds_the_order_without_a_robot_fault():
    core, now, _ = filling()
    out = time_out(core, now, core.params.fill_timeout_s)
    assert "ARM_NO_RESULT" in codes(out)
    assert not only(out, RobotFaultCmd)
    [report] = reports(out)
    assert (report.status, report.hold_reason) == ("HELD", HOLD_ARM_FAULT)
    assert core.order_of("r1") == "O-001"


def test_an_outbound_arm_past_its_retries_holds_the_order_too():
    core, now, swap = swapping()
    for _ in range(core.params.move_retry_limit):
        core.on_swap_result(swap.goal_id, False, False, False, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        core.step("r1")
        swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_swap_result(swap.goal_id, False, False, False, "GRIPPER")
    out = core.take_output()
    assert not only(out, RobotFaultCmd)
    [report] = reports(out)
    assert (report.status, report.hold_reason, report.placed) == (
        "HELD", HOLD_ARM_FAULT, {"A": 1, "C": 1})
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    assert not core.take_output()


def test_an_arm_held_robot_that_gets_isolated_turns_into_a_robot_fault_hold():
    core, _, _ = arm_fault_held({"A": 1})
    core.on_robot_isolated("r1", "COMM_LOST")
    [report] = reports(core.take_output())
    assert (report.status, report.hold_reason) == ("HELD", HOLD_ROBOT_FAULT)
    assert core._held == {"O-001": ("r1", HOLD_ROBOT_FAULT)}


def test_an_arm_held_robot_with_nothing_placed_that_gets_cleared_puts_the_order_back():
    core, _, _ = arm_fault_held()
    core.on_robot_removed("r1")
    out = core.take_output()
    assert "ORDER_REQUEUED" in codes(out) and not reports(out)
    assert core._held == {}
    assert [o.order_id for o in core._pending()] == ["O-001"]


def test_isolation_after_an_arm_timeout_waits_for_the_late_fill_and_counts_it():
    core, now, fill = filling()
    time_out(core, now, core.params.fill_timeout_s)
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    assert not reports(out) and "ORDER_REQUEUED" not in codes(out), "a result is still due"
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    [report] = reports(core.take_output())
    assert (report.status, report.hold_reason, report.placed) == (
        "HELD", HOLD_ROBOT_FAULT, {"A": 1})


def test_isolation_after_a_swap_timeout_still_ships_on_a_late_take():
    core, now, swap = swapping()
    time_out(core, now, core.params.swap_timeout_s)
    core.on_robot_isolated("r1", "GOAL_FAILED")
    assert not reports(core.take_output())
    core.on_swap_result(swap.goal_id, False, True, False, "CANCELED")
    assert [r.status for r in reports(core.take_output())] == ["SHIPPED"]


def test_an_arm_timeout_on_an_isolated_robot_waits_for_the_late_result():
    core, now, fill = filling()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.take_output()
    assert not reports(time_out(core, now, core.params.fill_timeout_s))
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    [report] = reports(core.take_output())
    assert (report.hold_reason, report.placed) == (HOLD_ROBOT_FAULT, {"A": 1})


def test_a_clear_decides_an_isolated_robot_whose_arm_never_answered():
    core, now, _ = filling()
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.take_output()
    time_out(core, now, core.params.fill_timeout_s)
    core.on_robot_removed("r1")
    out = core.take_output()
    assert "ORDER_REQUEUED" in codes(out) and not reports(out)


def test_isolation_during_a_forced_swap_stops_it_even_with_an_old_result_due():
    core, now, swap = swapping()
    old = swap
    time_out(core, now, core.params.swap_timeout_s)
    assert core.on_force_ship("O-001") == (True, "")
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    new = cmds(core.take_output(), "SwapCmd")[0]
    core.on_robot_isolated("r1", "GOAL_FAILED")
    out = core.take_output()
    assert [(c.robot, c.goal_id) for c in cmds(out, "CancelCmd")] == [("arm_ship", new.goal_id)]
    core.on_swap_result(old.goal_id, False, False, False, "CANCELED")
    assert not reports(core.take_output()), "the new swap is still running"
    core.on_swap_result(new.goal_id, False, True, False, "CANCELED")
    assert [r.status for r in reports(core.take_output())] == ["SHIPPED"]


def test_a_late_fill_after_a_forced_ship_keeps_placed_and_missing_in_step():
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, {"A": 1}, {}, "NOT_READY")
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    retry = cmds(core.take_output(), "FillCmd")[0]
    time_out(core, now, core.params.fill_timeout_s)
    assert core.on_force_ship("O-001") == (True, "")
    goal = [c for c in only(core.take_output(), DispatchCmd) if c.robot_id == "r1"][0]
    core.on_fill_result(retry.goal_id, True, {"C": 1}, {})
    core.take_output()
    core.on_dispatch_result(goal.goal_id, True, "r1")
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_swap_result(swap.goal_id, True, True, True)
    [report] = reports(core.take_output())
    assert (report.status, report.placed, report.missing) == ("SHIPPED", {"A": 1, "C": 1}, {})


def test_an_order_cut_down_to_fit_under_the_same_id_gets_a_robot():
    core, _ = make([BIG])
    core.take_output()
    core.on_order_board("RUN", [OrderRecord("O-BIG", {"A": 1})])
    assert len(calls(core.take_output())) == 1


def test_a_late_fill_after_an_arm_hold_goes_on_the_held_order_without_a_new_report():
    core, now, fill = filling()
    time_out(core, now, core.params.fill_timeout_s)
    core.on_fill_result(fill.goal_id, True, {"A": 1}, {})
    out = core.take_output()
    assert "ARM_LATE_RESULT" in codes(out) and not reports(out)
    assert core.robot_view("r1")["placed"] == {"A": 1}


# ---- ⑦ 강제 출고 ----


def test_force_ship_of_an_arm_hold_with_items_sends_them_out_and_reports_partial():
    core, now, _ = arm_fault_held({"A": 1})
    assert core.on_force_ship("O-001") == (True, "")
    out = core.take_output()
    [goal] = [c for c in only(out, DispatchCmd) if c.robot_id == "r1"]
    assert goal.station_id == "OUTBOUND"
    assert core.on_force_ship("O-001") == (False, "ORDER_STATE"), "no longer held"
    core.on_dispatch_result(goal.goal_id, True, "r1")
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_swap_result(swap.goal_id, True, True, True)
    [report] = reports(core.take_output())
    assert (report.status, report.placed, report.missing) == ("PARTIAL", {"A": 1}, {"C": 1})


def test_force_ship_of_an_arm_hold_with_nothing_placed_cancels_and_releases():
    core, _, _ = arm_fault_held()
    assert core.on_force_ship("O-001") == (True, "")
    out = core.take_output()
    [report] = reports(out)
    assert (report.status, report.placed, report.missing) == (CANCELED, {}, {"A": 1, "C": 1})
    assert only(out, ReleaseCmd) == [ReleaseCmd("r1")]
    assert core.order_of("r1") == ""
    assert not [c for c in only(out, DispatchCmd) if c.robot_id]


def test_force_ship_of_an_outbound_arm_hold_tries_the_swap_again():
    core, now, swap = swapping()
    for _ in range(core.params.move_retry_limit + 1):
        core.on_swap_result(swap.goal_id, False, False, False, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        core.step("r1")
        found = cmds(core.take_output(), "SwapCmd")
        swap = found[0] if found else swap
    assert core.on_force_ship("O-001") == (True, "")
    core.take_output()
    core.step("r1")
    swap = cmds(core.take_output(), "SwapCmd")[0]
    core.on_swap_result(swap.goal_id, True, True, True)
    [report] = reports(core.take_output())
    assert report.status == "SHIPPED", "the basket held the whole order"


def test_force_ship_refusals():
    core, _, _ = full_and_heading_out()
    core.on_order_board("RUN", [O1, O2])
    core.take_output()
    assert core.on_force_ship("O-404") == (False, "NOT_FOUND")
    assert core.on_force_ship("O-002") == (False, "ORDER_STATE"), "waiting, not held"
    assert core.on_force_ship("O-001") == (False, "ORDER_STATE"), "bound and going"
    core.on_robot_isolated("r1", "GOAL_FAILED")
    core.take_output()
    assert core.on_force_ship("O-001") == (False, "NOT_READY")


def test_force_ship_of_a_finished_order_is_not_held():
    core, _, swap = swapping()
    core.on_swap_result(swap.goal_id, True, True, True)
    core.take_output()
    assert core.on_force_ship("O-001") == (False, "ORDER_STATE")


# ---- 지시서가 정하지 않은 경우(확인 필요) ----


def test_an_arm_held_order_central_closes_leaves_the_robot_waiting_for_the_operator():
    core, now, _ = arm_fault_held({"A": 1})
    core.on_order_board("RUN", [])
    out = core.take_output()
    assert not reports(out) and not only(out, ReleaseCmd)
    assert core.order_of("r1") == "" and core._held == {}
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    assert not core.take_output(), "stays at the station until cleared"
    core.on_robot_removed("r1")
    out = core.take_output()
    assert not reports(out) and "ORDER_REQUEUED" not in codes(out)


def test_an_outbound_arm_that_fails_the_put_after_the_report_holds_only_the_robot():
    core, now, swap = swapping()
    core.on_swap_result(swap.goal_id, False, True, False, "GRIPPER")
    assert [r.status for r in reports(core.take_output())] == ["SHIPPED"]
    for _ in range(core.params.move_retry_limit):
        now[0] += core.params.arm_retry_wait_s
        core.step("r1")
        swap = cmds(core.take_output(), "SwapCmd")[0]
        core.on_swap_result(swap.goal_id, False, False, False, "GRIPPER")
    out = core.take_output()
    assert "SWAP_FAILED" in codes(out)
    assert not reports(out) and not only(out, RobotFaultCmd)
    now[0] += core.params.arm_retry_wait_s
    core.step("r1")
    assert not core.take_output(), "not asked again"
    core.on_robot_removed("r1")
    assert core._robots["r1"].held == ""
