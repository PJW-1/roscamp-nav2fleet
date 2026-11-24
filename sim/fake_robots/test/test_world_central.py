"""세계 안의 가짜 central: 보류는 끝이 아니고, 보류 뒤 최종 보고는 한 번만 받는다."""
from fake_robots.world.stations import Central
from fleet_core.common.commands import ReportOrderCmd, ReportTripCmd


def report(order, status, placed=None, reason=""):
    return ReportOrderCmd(order, "pode1", status, placed or {}, {}, "", reason)


def central(count=2):
    violations = []
    orders = [{"at_s": 0, "id": f"O-{i + 1}", "items": {"A": 1}} for i in range(count)]
    c = Central(orders, violations.append)
    c.board(0.0)
    return c, violations


def ids(c, t=1.0):
    return [o.order_id for o in c.board(t)]


def test_a_held_order_stays_on_the_board_and_is_not_finished():
    c, v = central()
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}))
    assert ids(c) == ["O-1", "O-2"] and c.finished == 0
    assert not c.all_final() and v == []


def test_a_final_report_after_a_hold_closes_the_order():
    c, v = central()
    c.take_report(1.0, report("O-1", "HELD"))
    c.take_report(2.0, report("O-1", "PARTIAL", {"A": 1}))
    assert ids(c) == ["O-2"] and c.finished == 1 and v == []


def test_a_second_final_report_after_a_hold_is_a_violation():
    c, v = central()
    c.take_report(1.0, report("O-1", "HELD"))
    c.take_report(2.0, report("O-1", "PARTIAL"))
    c.board(3.0)
    c.take_report(3.0, report("O-1", "PARTIAL"))
    assert len(v) == 1 and "O-1" in v[0]


def test_canceled_is_final():
    c, v = central(1)
    c.take_report(1.0, report("O-1", "CANCELED"))
    assert ids(c) == [] and c.finished == 1 and c.all_final() and v == []


def test_two_finals_without_a_hold_are_still_a_violation():
    c, v = central(1)
    c.take_report(1.0, report("O-1", "SHIPPED"))
    c.take_report(1.5, report("O-1", "SHIPPED"))
    assert len(v) == 1 and "twice" in v[0]


def test_the_hold_is_cleared_when_the_final_report_comes():
    c, v = central()
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ARM_FAULT"))
    assert set(c.held) == {"O-1"}
    c.take_report(2.0, report("O-1", "PARTIAL", {"A": 1}))
    assert c.held == {} and [r.order_id for r in c.hold_reports] == ["O-1"] and v == []


def test_a_robot_fault_hold_waits_for_the_central_and_counts_as_settled():
    """ROBOT_FAULT 보류는 central이 손으로 닫는다. 판에 남아도 세계는 기다리지 않는다."""
    c, v = central(2)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ROBOT_FAULT"))
    c.take_report(1.0, report("O-2", "SHIPPED", {"A": 1}))
    assert ids(c, 2.0) == ["O-1"] and c.finished == 1 and c.closed_by_hand == ["O-1"]
    assert c.all_final() and v == []


def test_an_arm_fault_hold_still_waits_for_a_final_report():
    c, v = central(1)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ARM_FAULT"))
    assert c.closed_by_hand == [] and not c.all_final() and v == []


def test_a_second_hold_report_is_a_violation():
    c, v = central(1)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ARM_FAULT"))
    c.take_report(2.0, report("O-1", "HELD", {"A": 1}, "ARM_FAULT"))
    assert len(v) == 1 and "held twice" in v[0]


def test_a_final_report_after_a_robot_fault_hold_is_a_violation():
    """ROBOT_FAULT 보류 주문은 관제가 더 진행하지 않는다(central이 닫는다)."""
    c, v = central(1)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ROBOT_FAULT"))
    c.take_report(2.0, report("O-1", "SHIPPED", {"A": 1}))
    assert len(v) == 1 and "ROBOT_FAULT" in v[0] and c.finished == 0


def test_an_arm_fault_hold_may_turn_into_a_robot_fault_hold():
    """ARM_FAULT로 묶인 Pode가 담은 채 격리되거나 치워지면 코어가 ROBOT_FAULT로 다시 보고한다."""
    c, v = central(1)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ARM_FAULT"))
    c.take_report(2.0, report("O-1", "HELD", {"A": 1}, "ROBOT_FAULT"))
    assert v == [] and c.closed_by_hand == ["O-1"] and c.all_final()
    assert [r.hold_reason for r in c.hold_reports] == ["ARM_FAULT", "ROBOT_FAULT"]


def test_nothing_may_follow_a_robot_fault_hold():
    c, v = central(1)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ROBOT_FAULT"))
    c.take_report(2.0, report("O-1", "HELD", {"A": 1}, "ARM_FAULT"))
    assert len(v) == 1 and c.closed_by_hand == ["O-1"]


def test_a_refused_final_keeps_the_robot_fault_hold():
    c, v = central(1)
    c.take_report(1.0, report("O-1", "HELD", {"A": 1}, "ROBOT_FAULT"))
    c.take_report(2.0, report("O-1", "SHIPPED", {"A": 1}))
    assert len(v) == 1 and c.closed_by_hand == ["O-1"] and c.all_final()


# ---- 진행 단계 보고(ReportTripCmd): 판에 남고 끝이 아니다 ----
def trip(order, status, placed=None):
    return ReportTripCmd(order, "pode1", status, placed or {}, {})


def test_progress_reports_are_kept_in_order_and_do_not_finish_the_order():
    c, v = central()
    for t, status in enumerate(("ASSIGNED", "FILLING", "TRANSPORTING"), start=1):
        c.take_trip_report(float(t), trip("O-1", status))
    assert [r.status for _, r in c.trip_reports] == ["ASSIGNED", "FILLING", "TRANSPORTING"]
    assert [t for t, _ in c.trip_reports] == [1.0, 2.0, 3.0]
    assert ids(c) == ["O-1", "O-2"] and c.finished == 0 and v == []


def test_a_rebinding_after_a_requeue_is_not_a_violation():
    c, v = central()
    c.take_trip_report(1.0, trip("O-1", "FILLING"))
    c.take_trip_report(2.0, trip("O-1", "ASSIGNED"))
    assert v == [] and len(c.trip_reports) == 2


def test_a_progress_report_needs_a_progress_status():
    c, v = central()
    c.take_trip_report(1.0, trip("O-1", "SHIPPED"))
    assert len(v) == 1 and "O-1" in v[0] and "SHIPPED" in v[0] and c.trip_reports == []


def test_a_progress_report_for_an_order_not_on_the_board_is_a_violation():
    c, v = central(count=1)
    c.take_trip_report(1.0, trip("O-9", "ASSIGNED"))
    assert len(v) == 1 and "O-9" in v[0] and c.trip_reports == []


def test_a_progress_report_after_the_final_report_is_a_violation():
    c, v = central()
    c.take_report(1.0, report("O-1", "SHIPPED", {"A": 1}))
    c.take_trip_report(2.0, trip("O-1", "TRANSPORTING"))
    assert len(v) == 1 and "O-1" in v[0] and c.trip_reports == []


def test_a_progress_report_after_a_robot_fault_hold_is_a_violation():
    c, v = central()
    c.take_report(1.0, report("O-1", "HELD", reason="ROBOT_FAULT"))
    c.take_trip_report(2.0, trip("O-1", "FILLING"))
    assert len(v) == 1 and "ROBOT_FAULT" in v[0] and c.trip_reports == []


def test_a_progress_report_after_an_arm_fault_hold_is_kept():
    c, v = central()
    c.take_report(1.0, report("O-1", "HELD", reason="ARM_FAULT"))
    c.take_trip_report(2.0, trip("O-1", "FILLING"))
    assert v == [] and len(c.trip_reports) == 1
