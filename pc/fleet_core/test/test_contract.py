"""이름, 타입, QoS의 기준은 계약 파일 하나다."""
from pathlib import Path

import pytest
from fleet_core.common.contract import ContractError, load_contract

CONTRACT = Path(__file__).parents[3] / "interfaces" / "fleet_interfaces" / "contract.yaml"


@pytest.fixture(scope="module")
def contract():
    return load_contract(CONTRACT)


def test_topic_name_is_built_from_key_and_placeholder(contract):
    assert contract.name("robot_state", pode="pode2") == "/pode2/state"
    assert contract.name("order_board") == "/central/orders"
    assert contract.name("swap_basket", arm="arm3") == "/arm3/swap_basket"


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
    assert "/arm3/swap_basket" in entries
    assert "/arm1/swap_basket" not in entries


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
FORCE_SHIP_0_6 = """string order_id
---
string REASON_NOT_HELD="NOT_HELD"
string REASON_ROBOT_FAULT="ROBOT_FAULT"
string REASON_UNKNOWN_ORDER="UNKNOWN_ORDER"
string REASON_NOT_IMPLEMENTED="NOT_IMPLEMENTED"
bool accepted
string reason
"""


def test_order_services_match_the_team_contract_text():
    assert (SRV / "ReportOrder.srv").read_text(encoding="utf-8") == REPORT_ORDER_0_6
    assert (SRV / "ForceShip.srv").read_text(encoding="utf-8") == FORCE_SHIP_0_6


def test_contract_0_6_opens_the_force_ship_service(contract):
    import yaml
    assert yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))["version"] == 0.6
    assert contract.name("force_ship") == "/fleet/force_ship"
    assert contract.type("force_ship") == "fleet_interfaces/srv/ForceShip"
    cmake = (CONTRACT.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    assert '"srv/ForceShip.srv"' in cmake
