"""이름, 타입, QoS의 기준은 계약 파일 하나다."""
from pathlib import Path

import pytest
from fleet_core.common.contract import ContractError, load_contract

CONTRACT = Path(__file__).parents[3] / "interfaces" / "fleet_interfaces" / "contract.yaml"


@pytest.fixture(scope="module")
def contract():
    return load_contract(CONTRACT)


def test_force_ship_rejections_use_contract_codes(contract):
    """강제 출고 거절 사유는 계약의 request_rejected 코드만 쓴다."""
    from fleet_core.common import commands
    codes = {getattr(commands, n) for n in dir(commands) if n.startswith("FORCE_")}
    assert codes and codes <= set(contract.reason_codes["request_rejected"])


def test_clear_and_reinstate_rejections_use_contract_codes(contract):
    from fleet_core.common import commands
    codes = {getattr(commands, n) for n in dir(commands) if n.startswith("REJECT_")}
    assert codes and codes <= set(contract.reason_codes["request_rejected"])


def test_topic_name_is_built_from_key_and_placeholder(contract):
    assert contract.name("robot_state", pode="pode2") == "/pode2/state"
    assert contract.name("order_board") == "/central/orders"
    assert contract.name("swap_basket", arm="arm_ship") == "/arm_ship/swap_basket"


def test_qos_lookup_returns_the_profile_of_the_entry(contract):
    qos = contract.qos("robot_state")
    assert (qos.reliability, qos.durability, qos.depth) == ("best_effort", "volatile", 5)
    assert contract.qos("zones").durability == "transient_local"


def test_the_heartbeat_is_reliable_with_depth_one(contract):
    """빠짐 자체가 정지 조건이라 재전송이 필요하고 최신 하나만 다시 보낸다. 상태는 best_effort다."""
    qos = contract.qos("heartbeat")
    assert (qos.reliability, qos.durability, qos.depth) == ("reliable", "volatile", 1)
    assert contract.qos("robot_state").reliability == "best_effort"


def test_unknown_key_is_an_error_not_a_guess(contract):
    with pytest.raises(ContractError):
        contract.name("no_such_key")


def test_missing_placeholder_is_an_error(contract):
    with pytest.raises(ContractError):
        contract.name("robot_state")


def test_expand_lists_every_instance_with_type_and_kind(contract):
    entries = {e.name: e for e in contract.expanded()}
    assert entries["/pode3/move_to"].kind == "action"
    assert entries["/pode3/move_to"].type == "fleet_interfaces/action/MoveTo"
    assert entries["/pode3/state"].qos.reliability == "best_effort"
    assert entries["/workcell/fill_basket"].type == "fleet_interfaces/action/FillBasket"
    assert "/arm_ship/swap_basket" in entries
    assert "/arm_left/swap_basket" not in entries


def test_expand_skips_real_only_entries_when_asked(contract):
    with_real = {e.name for e in contract.expanded(include_real_only=True)}
    without = {e.name for e in contract.expanded(include_real_only=False)}
    assert "/pode1/navigate_to_pose" in with_real
    assert "/pode1/navigate_to_pose" not in without


def test_entry_with_undefined_qos_profile_is_rejected(tmp_path):
    bad = tmp_path / "contract.yaml"
    bad.write_text(
        'placeholders: {}\nqos_profiles: {}\n'
        'topics:\n  - {key: a, name: /a, type: x/msg/A, qos: NOPE}\n',
        encoding="utf-8",
    )
    with pytest.raises(ContractError):
        load_contract(bad)


# ---- 계약 0.6: 팀 레포 지시서와 글자 하나까지 같다 ----
SRV = CONTRACT.parent / "srv"
REPORT_ORDER_0_6 = """string STATUS_SHIPPED="SHIPPED"
string STATUS_PARTIAL="PARTIAL"
string STATUS_HELD="HELD"
string STATUS_CANCELED="CANCELED"
string HOLD_ROBOT_FAULT="ROBOT_FAULT"
string HOLD_ARM_FAULT="ARM_FAULT"
string order_id
string robot_id
string status
string hold_reason
OrderItem[] placed
OrderItem[] missing
string detail
---
bool accepted
"""


def test_order_report_keeps_the_team_0_6_text():
    assert (SRV / "ReportOrder.srv").read_text(encoding="utf-8") == REPORT_ORDER_0_6


# ---- 계약 0.9: 인터페이스 명세 v0.2 ----
def test_the_contract_is_version_1_0():
    import yaml
    assert yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))["version"] == 1.0


def test_contract_0_9_names_the_arms_like_the_spec(contract):
    assert contract.placeholders["arm"] == ["arm_left", "arm_right", "arm_ship"]
    assert contract.name("swap_basket", arm="arm_ship") == "/arm_ship/swap_basket"
    names = {e.name for e in contract.expanded()}
    assert {"/arm_left/state", "/arm_right/state", "/arm_ship/state"} <= names
    # 명세 v0.2 (11-24): Phony도 락 해제 · 잠금 대상이다
    assert "/arm_ship/set_lock" in names and "/arm_ship/fault_resolved" not in names
    assert {"/arm_ship/set_mode", "/arm_ship/force", "/pode1/charge_end"} <= names


SPEC_NAMES = {
    "system_mode": ("/system/mode", "msg/SystemMode"),
    "close_state": ("/bridge/close_state", "msg/CloseState"),
    "submit_trip": ("/central/submit_trip", "srv/SubmitTrip"),
    "cancel_order": ("/central/cancel_order", "srv/CancelOrder"),
    "set_run_mode": ("/central/set_run_mode", "srv/SetRunMode"),
    "set_assign_hold": ("/central/set_assign_hold", "srv/SetAssignHold"),
    "get_close_state": ("/bridge/get_close_state", "srv/GetCloseState"),
    "hello": ("/bridge/hello", "srv/Hello"),
    "list_open_trips": ("/bridge/list_open_trips", "srv/ListOpenTrips"),
    "report_trip": ("/bridge/report_trip", "srv/ReportTrip"),
    "report_command": ("/bridge/report_command", "srv/ReportCommand"),
    "report_event": ("/bridge/report_event", "srv/ReportEvent"),
    "check_stock": ("/bridge/check_stock", "srv/CheckStock"),
    "set_lock": ("/fleet/set_lock", "srv/SetLock"),
    "fault_resolved": ("/fleet/fault_resolved", "srv/RobotCommand"),
    "resume_arm": ("/fleet/resume_arm", "srv/RobotCommand"),
    "charge_end": ("/fleet/charge_end", "srv/RobotCommand"),
    "force_input": ("/fleet/force_input", "srv/ForceInput"),
    "resume_fill": ("/fleet/resume_fill", "srv/TripCommand"),
    "force_ship": ("/fleet/force_ship", "srv/TripCommand"),
    "force_partial": ("/fleet/force_partial", "srv/TripCommand"),
    "return_to_shelf": ("/fleet/return_to_shelf", "srv/TripCommand"),
    "resume_held": ("/fleet/resume_held", "srv/TripCommand"),
    "requeue_trip": ("/fleet/requeue_trip", "srv/TripCommand"),
}


@pytest.mark.parametrize("key", sorted(SPEC_NAMES))
def test_contract_0_7_has_every_spec_name_with_its_type(contract, key):
    name, kind = SPEC_NAMES[key]
    assert contract.name(key) == name
    assert contract.type(key) == f"fleet_interfaces/{kind}"


def test_every_type_the_contract_names_is_built():
    cmake = (CONTRACT.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    data = load_contract(CONTRACT)
    for entry in data.entries.values():
        package, kind, name = entry.type.split("/")
        if package == "fleet_interfaces":
            assert f'"{kind}/{name}.{kind}"' in cmake, entry.type
            assert (CONTRACT.parent / kind / f"{name}.{kind}").exists(), entry.type
    assert "ForceShip" not in cmake


def test_reason_code_table_has_the_spec_codes(contract):
    codes = {c for group in contract.reason_codes.values() for c in group}
    assert {"NOT_FOUND", "LOCK_RELEASED", "HOLDING_ITEM", "MODE_MISMATCH", "NOT_READY",
            "ROBOT_TIMEOUT", "PICK_FAILED", "FLEET_RESTART", "NOTHING_TO_FILL",
            "FORCED_PARTIAL", "LATE_REPORT", "NOT_IMPLEMENTED", "NAV_UNAVAILABLE",
            "HEARTBEAT_LOST"} <= codes
    assert len(codes) == 34


def test_entries_no_fake_node_opens_are_left_out_of_the_fake_check(contract):
    fake = {e.name for e in contract.expanded(include_real_only=False)}
    real = {e.name for e in contract.expanded(include_real_only=True)}
    assert "/bridge/hello" in real and "/bridge/hello" not in fake
    assert "/system/mode" in real and "/system/mode" not in fake
    assert "/arm_left/state" in fake and "/fleet/force_ship" in fake


# ---- 계약 0.8: 로봇 운영 모드 (SetMode, IR-73) ----
def test_every_robot_has_a_set_mode_service(contract):
    robots = ("pode1", "pode2", "pode3", "arm_left", "arm_right", "arm_ship")
    assert contract.placeholders["robot"] == list(robots)
    assert contract.name("robot_set_mode", robot="pode2") == "/pode2/set_mode"
    assert contract.type("robot_set_mode") == "fleet_interfaces/srv/SetMode"


# ---- 계약 0.7: 주문 흐름 1차 (SubmitTrip · Hello · ReportTrip · SystemMode) ----
def test_order_flow_names_are_in_the_contract():
    contract = load_contract(CONTRACT)
    assert contract.name("submit_trip") == "/central/submit_trip"
    assert contract.name("hello") == "/bridge/hello"
    assert contract.name("report_trip") == "/bridge/report_trip"
    assert contract.name("system_mode") == "/system/mode"
    assert contract.qos("system_mode").durability == "transient_local"
