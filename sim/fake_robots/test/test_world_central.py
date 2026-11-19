"""세계 안의 가짜 central: 보류는 끝이 아니고, 보류 뒤 최종 보고는 한 번만 받는다."""
from fake_robots.world.stations import Central
from fleet_core.common.commands import ReportOrderCmd


def report(order, status, placed=None):
    return ReportOrderCmd(order, "pode1", status, placed or {}, {}, "")


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
