"""pode_fsm이 쓰는 계약 값. pc/*를 import하지 않고 contract.yaml을 직접 읽는다."""
from pathlib import Path

from pode_fsm.contract import load_contract

REPO = Path(__file__).resolve().parents[3]
INTERFACES = REPO / "interfaces" / "fleet_interfaces"
CONTRACT = load_contract(INTERFACES / "contract.yaml")


def test_names_come_from_the_contract():
    assert CONTRACT.name("robot_state", pode="pode2") == "/pode2/state"
    assert CONTRACT.name("move_to", pode="pode1") == "/pode1/move_to"
    assert CONTRACT.name("dock", pode="pode1") == "/pode1/dock"
    assert CONTRACT.name("heartbeat") == "/fleet/heartbeat"


def test_heartbeat_is_reliable_with_depth_one():
    """Wi-Fi에서 best_effort로 3번 빠지면 서므로 RELIABLE, depth 1."""
    assert CONTRACT.qos("heartbeat") == ("reliable", "volatile", 1)


def test_state_stays_best_effort():
    assert CONTRACT.qos("robot_state") == ("best_effort", "volatile", 5)


def test_move_to_has_the_path_not_connected_reason():
    """경로가 로봇 위치를 지나지 않으면 이 reason으로 실패한다."""
    action = (INTERFACES / "action" / "MoveTo.action").read_text(encoding="utf-8")
    assert 'string REASON_PATH_NOT_CONNECTED="PATH_NOT_CONNECTED"' in action
    assert "geometry_msgs/Pose[] path" in action


def test_move_to_has_the_nav_unavailable_reason():
    """Nav2 서버가 없어 보내지 못한 goal은 이 reason으로 끝난다(관제는 시도로 세지 않는다)."""
    action = (INTERFACES / "action" / "MoveTo.action").read_text(encoding="utf-8")
    assert 'string REASON_NAV_UNAVAILABLE="NAV_UNAVAILABLE"' in action


def test_retreat_has_the_nav_unavailable_reason():
    """Nav2 서버가 없어 보내지 못한 물러나기도 이 reason으로 끝난다."""
    action = (INTERFACES / "action" / "Retreat.action").read_text(encoding="utf-8")
    assert 'string REASON_NAV_UNAVAILABLE="NAV_UNAVAILABLE"' in action


def test_robot_state_has_the_states_the_fsm_reports():
    msg = (INTERFACES / "msg" / "RobotState.msg").read_text(encoding="utf-8")
    for state in ("IDLE", "MOVING", "DOCKING", "DOCKED", "BLOCKED", "LOCALIZING"):
        assert f'string STATE_{state}="{state}"' in msg
