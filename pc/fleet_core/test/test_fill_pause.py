"""담기 일시 중지 (팔 관제 합의 ①): 결과로 끝난 실패는 운반 건을 FILLING_PAUSE로 알리고
Pode를 작업대에 세워 관리자의 [담기 재개] · [보충 불가]를 기다린다. 자동 일부 출고는 없다.
PICK_FAILED는 자리만, Phody가 당분간 보내지 않음."""
import pytest
from core_helpers import cmds, codes
from fleet_core.common.commands import OrderRecord, ReportTripCmd
from test_orchestrator import O1, filling


def paused(reason="PICK_FAILED", placed=None):
    core, now, fill = filling()
    core.on_fill_result(fill.goal_id, False, placed or {"A": 1}, {}, reason)
    return core, now, core.take_output()


@pytest.mark.parametrize("reason", ["PICK_FAILED", "RECOGNITION_ERROR", "OUT_OF_STOCK"])
def test_a_failure_with_a_result_reports_the_trip_paused_with_its_reason(reason):
    _, _, out = paused(reason)
    assert cmds(out, "ReportTripCmd") == [
        ReportTripCmd("O-001", "r1", "FILLING_PAUSE", {"A": 1}, {"C": 1}, reason)]
    assert "FILL_PAUSED" in codes(out)
    assert not cmds(out, "ReportOrderCmd"), "no hold and no partial ship by itself"


def test_a_paused_fill_sends_no_goal_and_keeps_the_robot_at_the_station():
    core, now, _ = paused()
    for _ in range(3):
        now[0] += 3600.0
        core.step("r1")
        out = core.take_output()
        assert not cmds(out, "FillCmd") and not cmds(out, "DispatchCmd")
    assert core.order_of("r1") == "O-001"
    assert core.robot_view("r1")["placed"] == {"A": 1}


def test_the_paused_order_is_not_offered_to_another_robot():
    core, _, _ = paused()
    core.on_order_board("RUN", [O1, OrderRecord("O-002", {"A": 1})])
    calls = [c for c in cmds(core.take_output(), "DispatchCmd") if not c.robot_id]
    assert len(calls) == 1, "only O-002 waits"


def test_clearing_a_paused_robot_holds_what_was_placed_for_the_operator():
    core, _, _ = paused()
    core.on_robot_removed("r1")
    out = core.take_output()
    assert [(r.status, r.hold_reason) for r in cmds(out, "ReportOrderCmd")] == [
        ("HELD", "ROBOT_FAULT")]
    assert core.order_of("r1") == ""
