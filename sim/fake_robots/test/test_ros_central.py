"""실제 토픽과 서비스로 FakeCentral을 시험한다."""
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import CONTRACT, wait_until, write_scenario  # noqa: E402
from fleet_core.common.contract import load_contract, to_ros_qos  # noqa: E402
from fleet_interfaces.msg import FleetEvent, OrderBoard, OrderItem  # noqa: E402
from fleet_interfaces.srv import ReportOrder, TripCommand  # noqa: E402

C = load_contract(CONTRACT)
ORDERS = [
    {"at_s": 0, "id": "O-1", "items": {"A": 1}},
    {"at_s": 0, "id": "O-2", "items": {"C": 2}},
]


class Central:
    def __init__(self, harness, tmp_path, **scenario):
        import rclpy.node
        from fake_robots.fake_central_node import FakeCentralNode
        scenario.setdefault("orders", ORDERS)
        self.node = harness.add(FakeCentralNode(parameter_overrides=harness.params(
            scenario_file=write_scenario(tmp_path, **scenario))))
        probe = self.probe = harness.add(rclpy.node.Node("central_probe"))
        self.boards, self.events = [], []
        probe.create_subscription(OrderBoard, C.name("order_board"), self.boards.append,
                                  to_ros_qos(C.qos("order_board")))
        probe.create_subscription(FleetEvent, C.name("events"), self.events.append,
                                  to_ros_qos(C.qos("events")))
        self.report_cli = probe.create_client(ReportOrder, C.name("report_order"))
        assert self.report_cli.wait_for_service(5.0)

    def ids(self):
        return [o.order_id for o in self.boards[-1].orders] if self.boards else None

    def wait_ids(self, expected, timeout=8.0):
        assert wait_until(lambda: self.ids() == expected, timeout), self.ids()

    def call(self, cli, request):
        fut = cli.call_async(request)
        assert wait_until(fut.done, 5.0)
        return fut.result()

    def serve_force_ship(self, accepted=True, reason=""):
        """매니저 몫인 /fleet/force_ship의 대역. 받은 주문 id를 모은다."""
        self.forced = []

        def handle(request, response):
            self.forced.append(request.trip_id)
            response.accepted, response.reason = accepted, reason
            return response
        self.probe.create_service(TripCommand, C.name("force_ship"), handle)

    def report(self, order_id, status, placed=None, missing=None, detail="", hold_reason="",
               robot="pode1"):
        return self.call(self.report_cli, ReportOrder.Request(
            order_id=order_id, robot_id=robot, status=status, hold_reason=hold_reason,
            placed=[OrderItem(kind=k, count=c) for k, c in (placed or {}).items()],
            missing=[OrderItem(kind=k, count=c) for k, c in (missing or {}).items()],
            detail=detail))

    def codes(self):
        return [e.code for e in self.events]

    def wait_code(self, code, timeout=5.0):
        assert wait_until(lambda: code in self.codes(), timeout), self.codes()
        return next(e for e in self.events if e.code == code)


SHIPPED = ReportOrder.Request.STATUS_SHIPPED
PARTIAL = ReportOrder.Request.STATUS_PARTIAL
HELD = ReportOrder.Request.STATUS_HELD
CANCELED = ReportOrder.Request.STATUS_CANCELED
ARM_FAULT = ReportOrder.Request.HOLD_ARM_FAULT
ROBOT_FAULT = ReportOrder.Request.HOLD_ROBOT_FAULT


def test_board_is_oldest_first_with_items_and_run_mode(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    board = c.boards[-1]
    assert board.run_mode == OrderBoard.MODE_RUN
    assert {i.kind: i.count for i in board.orders[1].items} == {"C": 2}


def test_orders_are_released_at_their_time_in_order(harness, tmp_path):
    c = Central(harness, tmp_path, orders=[
        {"at_s": 2.0, "id": "late", "items": {"A": 1}},
        {"at_s": 0, "id": "early", "items": {"A": 1}},
    ])
    c.wait_ids(["early"])
    c.wait_ids(["early", "late"], timeout=6.0)
    assert [e.detail for e in c.events if e.code == "ORDER_ACCEPTED"][-1] == "late"


def test_scenario_orders_skip_the_submit_checks(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"A": 0}, orders=[
        {"at_s": 0, "id": "big", "items": {"Z": 9}}])
    c.wait_ids(["big"])


def test_shipped_report_removes_the_order_and_deducts(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert c.report("O-1", SHIPPED, placed={"A": 1}).accepted
    c.wait_ids(["O-2"])
    assert c.node.inventory()["A"] == 2
    event = c.wait_code("ORDER_SHIPPED")
    assert event.detail.startswith("O-1")
    assert "OUT_OF_STOCK" not in c.codes() and "STOCK_MISMATCH" not in c.codes()


def test_report_of_an_unknown_order_or_status_is_refused(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert not c.report("nope", SHIPPED).accepted
    assert not c.report("O-1", "LOST").accepted
    c.wait_ids(["O-1", "O-2"])


def test_a_repeated_report_changes_nothing(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert c.report("O-1", SHIPPED, placed={"A": 1}).accepted
    assert c.report("O-1", SHIPPED, placed={"A": 1}).accepted
    c.wait_code("ORDER_SHIPPED")
    assert c.node.inventory()["A"] == 2
    assert c.codes().count("ORDER_SHIPPED") == 1


def test_missing_with_an_empty_record_is_out_of_stock(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"A": 1, "D": 0}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"A": 1, "D": 1}}])
    c.wait_ids(["O-1"])
    assert c.report("O-1", PARTIAL, placed={"A": 1}, missing={"D": 1}).accepted
    assert c.wait_code("OUT_OF_STOCK").detail.startswith("O-1")
    assert c.wait_code("ORDER_PARTIAL").detail.startswith("O-1")
    assert "STOCK_MISMATCH" not in c.codes()
    assert c.node.inventory() == {"A": 0, "D": 0}


def test_missing_with_some_in_the_record_is_a_mismatch_and_gets_corrected(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"E": 3}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"E": 1}}])
    c.wait_ids(["O-1"])
    assert c.report("O-1", CANCELED, missing={"E": 1}).accepted
    assert c.wait_code("STOCK_MISMATCH").detail.startswith("O-1")
    assert c.wait_code("ORDER_CANCELED").detail.startswith("O-1")
    assert "OUT_OF_STOCK" not in c.codes()
    assert c.node.inventory() == {"E": 0}


def test_after_a_correction_the_same_kind_is_out_of_stock(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"E": 3}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"E": 1}},
        {"at_s": 0, "id": "O-2", "items": {"E": 1}}])
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", CANCELED, missing={"E": 1})
    c.report("O-2", CANCELED, missing={"E": 1})
    c.wait_code("OUT_OF_STOCK")
    assert c.codes().count("STOCK_MISMATCH") == 1


def test_placed_is_deducted_before_missing_is_judged(harness, tmp_path):
    """기록이 놓은 수와 정확히 같았으니 부족분은 불일치가 아니다."""
    c = Central(harness, tmp_path, inventory={"A": 1}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"A": 2}}])
    c.wait_ids(["O-1"])
    c.report("O-1", PARTIAL, placed={"A": 1}, missing={"A": 1})
    c.wait_code("OUT_OF_STOCK")
    assert "STOCK_MISMATCH" not in c.codes()


def test_each_missing_kind_gets_its_own_code(harness, tmp_path):
    c = Central(harness, tmp_path, inventory={"D": 0, "E": 1}, orders=[
        {"at_s": 0, "id": "O-1", "items": {"D": 1, "E": 1}}])
    c.wait_ids(["O-1"])
    c.report("O-1", CANCELED, missing={"D": 1, "E": 1})
    c.wait_code("ORDER_CANCELED")
    by_code = {e.code: e.detail for e in c.events}
    assert " D " in by_code["OUT_OF_STOCK"] and " E " in by_code["STOCK_MISMATCH"]


def test_canceled_report_removes_the_order(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert c.report("O-1", CANCELED, missing={"A": 1}).accepted
    c.wait_ids(["O-2"])
    assert c.wait_code("ORDER_CANCELED").detail.startswith("O-1")


def test_a_held_report_keeps_the_order_and_records_the_reason(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    assert c.report("O-1", HELD, placed={"A": 1}, hold_reason=ARM_FAULT).accepted
    event = c.wait_code("ORDER_HELD")
    assert event.detail.startswith("O-1") and "hold_reason=ARM_FAULT" in event.detail
    assert c.ids() == ["O-1", "O-2"]
    assert c.node.held_reasons() == {"O-1": ARM_FAULT}
    assert c.node.inventory()["A"] == 3  # 보류는 재고를 건드리지 않는다


def test_a_repeated_held_report_gives_one_event(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ROBOT_FAULT)
    assert c.report("O-1", HELD, hold_reason=ROBOT_FAULT).accepted
    c.wait_code("ORDER_HELD")
    assert c.codes().count("ORDER_HELD") == 1


def test_a_final_report_after_a_hold_deducts_and_closes(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, placed={"A": 1}, hold_reason=ARM_FAULT)
    assert c.report("O-1", PARTIAL, placed={"A": 1}).accepted
    c.wait_ids(["O-2"])
    c.wait_code("ORDER_PARTIAL")
    assert c.node.inventory()["A"] == 2
    assert c.node.held_reasons() == {}


def test_missing_after_an_accepted_force_ship_is_not_a_stock_shortage(harness, tmp_path):
    """강제 출고의 missing은 고장 난 팔이 못 놓은 것이다. 재고를 0으로 만들지 않는다."""
    c = Central(harness, tmp_path, force_ship_after_s=0.5)
    c.serve_force_ship()
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, placed={"A": 1}, hold_reason=ARM_FAULT)
    assert "accepted=True" in c.wait_code("FORCE_SHIP", timeout=6.0).detail
    assert c.report("O-1", PARTIAL, placed={"A": 1}, missing={"A": 1}).accepted
    c.wait_code("ORDER_PARTIAL")
    assert c.node.inventory()["A"] == 2
    assert not {"OUT_OF_STOCK", "STOCK_MISMATCH"} & set(c.codes())


def test_a_final_that_beats_the_force_ship_answer_is_still_forced(harness, tmp_path):
    """빈 바구니면 관제가 강제 출고 응답과 거의 같이 CANCELED를 보낸다.
    응답보다 먼저 와도 재고 문제가 아니다."""
    import threading

    import rclpy.node
    c = Central(harness, tmp_path, force_ship_after_s=0.3)
    gate, asked, answered = threading.Event(), [], []

    def handle(request, response):
        asked.append(request.trip_id)
        gate.wait(5.0)  # 최종 보고가 먼저 닿도록 응답을 붙잡는다
        answered.append(request.trip_id)
        response.accepted = True
        return response
    server = harness.add(rclpy.node.Node("slow_force_ship"))
    server.create_service(TripCommand, C.name("force_ship"), handle)
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    assert wait_until(lambda: asked == ["O-1"], 6.0)
    try:
        assert c.report("O-1", CANCELED, missing={"A": 1}).accepted
        c.wait_code("ORDER_CANCELED")
        assert answered == [], "the answer must still be held back"
    finally:
        gate.set()
    assert c.node.inventory()["A"] == 3
    assert not {"OUT_OF_STOCK", "STOCK_MISMATCH"} & set(c.codes())


def test_missing_after_an_arm_fault_hold_without_force_ship_is_stock(harness, tmp_path):
    """강제 출고 없이 끝난 주문(치워서 되돌린 뒤 다시 채움 등)의 missing은 예전처럼 재고로 본다."""
    c = Central(harness, tmp_path)
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.wait_code("ORDER_HELD")
    assert c.report("O-1", PARTIAL, placed={"A": 1}, missing={"A": 1}).accepted
    c.wait_code("STOCK_MISMATCH")
    assert c.node.inventory()["A"] == 0


def test_missing_after_a_refused_force_ship_is_stock(harness, tmp_path):
    c = Central(harness, tmp_path, force_ship_after_s=0.5)
    c.serve_force_ship(accepted=False, reason="ORDER_STATE")
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.wait_code("FORCE_SHIP", timeout=6.0)
    assert c.report("O-1", PARTIAL, placed={"A": 1}, missing={"A": 1}).accepted
    c.wait_code("STOCK_MISMATCH")
    assert c.node.inventory()["A"] == 0


def test_no_force_ship_call_without_the_option(harness, tmp_path):
    c = Central(harness, tmp_path)
    c.serve_force_ship()
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.wait_code("ORDER_HELD")
    import time
    time.sleep(1.5)
    assert c.forced == [] and "FORCE_SHIP" not in c.codes()


def test_an_arm_fault_hold_calls_force_ship_after_the_delay(harness, tmp_path):
    c = Central(harness, tmp_path, force_ship_after_s=1.0)
    c.serve_force_ship()
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.wait_code("ORDER_HELD")
    assert c.forced == []  # 아직 지연 안
    event = c.wait_code("FORCE_SHIP", timeout=6.0)
    assert c.forced == ["O-1"]
    assert event.detail.startswith("O-1") and "accepted=True" in event.detail


def test_a_refused_force_ship_is_logged_with_its_reason(harness, tmp_path):
    c = Central(harness, tmp_path, force_ship_after_s=0.5)
    c.serve_force_ship(accepted=False, reason="ORDER_STATE")
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    event = c.wait_code("FORCE_SHIP", timeout=6.0)
    assert "accepted=False" in event.detail and "ORDER_STATE" in event.detail


def test_a_robot_fault_hold_is_not_force_shipped(harness, tmp_path):
    c = Central(harness, tmp_path, force_ship_after_s=0.5)
    c.serve_force_ship()
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ROBOT_FAULT)
    c.wait_code("ORDER_HELD")
    import time
    time.sleep(1.5)
    assert c.forced == []


def test_force_ship_waits_for_the_service_to_appear(harness, tmp_path):
    c = Central(harness, tmp_path, force_ship_after_s=0.3)
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.wait_code("ORDER_HELD")
    import time
    time.sleep(1.0)
    c.serve_force_ship()
    c.wait_code("FORCE_SHIP", timeout=8.0)
    assert c.forced == ["O-1"]


def test_force_ship_is_called_once_per_order_even_if_a_final_report_never_comes(harness, tmp_path):
    c = Central(harness, tmp_path, force_ship_after_s=0.3)
    c.serve_force_ship()
    c.wait_ids(["O-1", "O-2"])
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.report("O-1", HELD, hold_reason=ARM_FAULT)
    c.wait_code("FORCE_SHIP", timeout=6.0)
    import time
    time.sleep(1.0)
    assert c.forced == ["O-1"]
