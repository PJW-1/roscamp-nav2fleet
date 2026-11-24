"""ReportTripCmd: 주문 코어가 운반 건의 진행 단계를 노드에 알리는 명령.

기준은 팀 레포 docs/design 브랜치 `주문_흐름_1차_인터페이스.md` 4.4 ①이다. 끝 상태(SHIPPED,
PARTIAL, CANCELED, HELD)는 지금처럼 ReportOrderCmd가 맡고, 번호와 시각은 노드 한 곳에서
붙인다(IR-22). 주문 코어가 이 명령을 언제 내는지는 통합 관제의 몫이다(orders/).
"""
import dataclasses
import json
import re
from pathlib import Path

import pytest
from fleet_core.common.commands import (
    TRIP_ASSIGNED,
    TRIP_FILLING,
    TRIP_STATUSES,
    TRIP_TRANSPORTING,
    ReportOrderCmd,
    ReportTripCmd,
)
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.manager import FleetManager

# contract 0.7 ReportTrip.srv의 STATUS_* 값 가운데 진행 단계 세 개
SRV_PROGRESS = ("ASSIGNED", "FILLING", "TRANSPORTING")
FINAL = {"SHIPPED", "PARTIAL", "HELD", "CANCELED", "CANCELLED"}


def test_the_fields_follow_the_agreed_draft():
    fields = dataclasses.fields(ReportTripCmd)
    assert [f.name for f in fields] == ["trip_id", "robot", "status", "placed", "missing",
                                        "reason"]
    assert [f.name for f in fields if f.default is not dataclasses.MISSING] == ["reason"]
    assert ReportTripCmd("T-1", "pode1", TRIP_ASSIGNED, {}, {}).reason == ""


def test_the_progress_statuses_are_spelled_like_report_trip_in_the_normal_order():
    assert TRIP_STATUSES == (TRIP_ASSIGNED, TRIP_FILLING, TRIP_TRANSPORTING)
    assert TRIP_STATUSES == SRV_PROGRESS


def test_final_statuses_stay_with_report_order():
    assert not FINAL & set(TRIP_STATUSES)


def test_it_is_frozen_like_the_other_commands():
    cmd = ReportTripCmd("T-1", "pode1", TRIP_FILLING, {}, {})
    with pytest.raises(dataclasses.FrozenInstanceError):
        cmd.status = TRIP_TRANSPORTING


# ---- 관리자(FleetManager)를 거쳐 ROS 쪽 출력까지 ----
def _manager():
    site = load_fleet_map("d2")
    core = FleetManager(site, load_params(), ["pode1"], lambda: 0.0)
    lines = []
    core.record_to(lines.append)
    x, y = site.node_pose("P-1")
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state("pode1", x, y, 0.0, "IDLE")
    core.on_order_board("RUN", [])
    core.tick()
    return core, lines


def _inject(core, items):
    """주문 코어가 다음 출력에 items를 한 번 더 낸 것처럼 한다."""
    pending = list(items)
    real = core.orchestrator.take_output

    def take():
        out = real() + pending
        pending.clear()
        return out

    core.orchestrator.take_output = take


def test_a_trip_report_from_the_order_core_reaches_the_ros_side_unchanged_and_in_order():
    core, _ = _manager()
    trip = ReportTripCmd("T-1", "pode1", TRIP_TRANSPORTING, {"A": 1}, {"B": 1})
    final = ReportOrderCmd("T-1", "pode1", "PARTIAL", {"A": 1}, {"B": 1}, "")
    _inject(core, [trip, final])
    out = core.tick()
    reports = [c for c in out if isinstance(c, (ReportTripCmd, ReportOrderCmd))]
    assert reports == [trip, final]
    assert reports[0] is trip


def test_the_record_writes_a_trip_report_like_any_other_output():
    core, lines = _manager()
    _inject(core, [ReportTripCmd("T-1", "pode1", TRIP_ASSIGNED, {}, {})])
    core.tick()
    outs = [json.loads(line) for line in lines if '"out"' in line]
    [entry] = [e for e in outs if e["out"] == "ReportTripCmd"]
    assert entry["trip_id"] == "T-1" and entry["status"] == "ASSIGNED"
    assert entry["placed"] == {} and entry["missing"] == {} and entry["reason"] == ""


# ---- 계약 0.7 ReportTrip.srv와 철자가 같다 ----
SRV = Path(__file__).parents[3] / "interfaces" / "fleet_interfaces" / "srv" / "ReportTrip.srv"


def _srv_statuses():
    found = re.findall(r'^string STATUS_(\w+)="(\w+)"', SRV.read_text(encoding="utf-8"), re.M)
    assert all(name == value for name, value in found)
    return [value for _, value in found]


def test_trip_statuses_are_report_trip_srv_constants():
    statuses = _srv_statuses()
    assert set(TRIP_STATUSES) <= set(statuses)
    # 끝 상태는 ReportOrder가, RECEIVED는 central이, FILLING_PAUSE는 다음 주 몫이다
    assert set(statuses) - set(TRIP_STATUSES) == {
        "RECEIVED", "FILLING_PAUSE", "SHIPPED", "PARTIAL", "HELD", "CANCELLED"}
