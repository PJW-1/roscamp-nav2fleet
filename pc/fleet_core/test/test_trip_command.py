"""운반 건 명령은 주문 쪽 on_trip_command로 가고, 없으면 NOT_IMPLEMENTED (W-33 껍데기)."""
from core_helpers import codes
from test_manager import make


def test_without_an_order_handler_it_is_not_implemented():
    core, _ = make({"r1": "P-1"})
    assert not hasattr(core.orchestrator, "on_trip_command")
    assert core.on_trip_command("resume_fill", "O-001", {"A": 1}) == (
        False, "NOT_IMPLEMENTED", "주문 쪽 미구현")


def test_the_order_handler_judges_and_a_refusal_is_input_rejected():
    core, _ = make({"r1": "P-1"})
    asked = []

    def handler(command, trip_id, mark_missing):
        asked.append((command, trip_id, mark_missing))
        return (command == "resume_held", "" if command == "resume_held" else "ORDER_STATE", "x")

    core.orchestrator.on_trip_command = handler
    assert core.on_trip_command("resume_held", "O-001") == (True, "", "x")
    assert core.on_trip_command("requeue_trip", "O-002", {"B": 2}) == (False, "ORDER_STATE", "x")
    assert asked == [("resume_held", "O-001", {}), ("requeue_trip", "O-002", {"B": 2})]
    assert codes(core.tick()).count("INPUT_REJECTED") == 1
