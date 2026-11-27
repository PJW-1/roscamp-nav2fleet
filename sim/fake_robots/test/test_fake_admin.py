"""가짜 로봇 쪽 관리 명령 판단: 강제 입력, 장애 처리 완료, 팔 락 (IR-21 · IR-61 · IR-63)."""
import pytest

pytest.importorskip("fleet_interfaces.srv")

from fake_robots.fake_admin import FakeAdmin  # noqa: E402
from fleet_interfaces.srv import ForceInput  # noqa: E402
from pode_fsm import admin as pode_admin  # noqa: E402


def admin(mode="TEST"):
    a = FakeAdmin("arm_left", "boot-1")
    a.mode = mode
    return a


def test_force_needs_the_test_mode_and_lists_the_forced_key():
    a = admin("NORMAL")
    assert a.force("f1", "arm_left", "CONDITION", "battery", "20", 0.0).reason == \
        "NOT_TEST_MODE"
    a.mode = "TEST"
    assert a.force("f2", "arm_left", "CONDITION", "battery", "20", 0.0).accepted
    assert a.force("f3", "arm_left", "STATE", "PAUSED", "1", 0.0).accepted
    assert a.forced == ["battery", "PAUSED"]
    assert a.forced_values == {"battery": "20", "PAUSED": "1"}
    assert a.force("f4", "arm_left", "CLEAR", "battery", "", 0.0).accepted
    assert a.forced == ["PAUSED"]
    assert a.force("f5", "arm_left", "CLEAR", "", "", 0.0).accepted
    assert a.forced == []


def test_force_refuses_another_robot_and_an_unknown_kind():
    a = admin()
    assert a.force("f1", "pode1", "CONDITION", "x", "1", 0.0).reason == "UNKNOWN_TARGET"
    assert a.force("f2", "arm_left", "TOGGLE", "x", "1", 0.0).reason == "NOT_FOUND"


def test_the_same_request_id_is_applied_once():
    a = admin()
    a.force("same", "arm_left", "CONDITION", "battery", "20", 0.0)
    a.force("f2", "arm_left", "CLEAR", "", "", 1.0)
    assert a.force("same", "arm_left", "CONDITION", "battery", "20", 2.0).accepted
    assert a.forced == []  # 다시 반영하지 않는다


def test_fault_resolved_clears_the_latch_once():
    a = admin()
    cleared = []
    assert a.fault_resolved("r1", "arm_left", 0.0, lambda: cleared.append(1)).accepted
    assert a.fault_resolved("r1", "arm_left", 1.0, lambda: cleared.append(1)).accepted
    assert cleared == [1]
    assert a.fault_resolved("r2", "arm_right", 1.0).reason == "UNKNOWN_TARGET"


def test_an_arm_holding_an_item_releases_only_with_confirm_drop():
    a = admin()
    reply = a.set_lock_held("l1", "arm_left", False, False, True, True, 0.0)
    assert reply.reason == "HOLDING_ITEM" and a.lock
    assert a.set_lock_held("l2", "arm_left", False, True, True, True, 0.0).accepted
    assert not a.lock
    # 잠그기는 쥔 상품과 상관없다
    assert a.set_lock_held("l3", "arm_left", True, False, True, True, 0.0).accepted and a.lock


def test_a_release_while_moving_is_not_stopped():
    a = admin()
    assert a.set_lock_held("l1", "arm_left", False, False, False, False, 0.0).reason == \
        "NOT_STOPPED"


def test_the_force_kinds_match_the_interface():
    r = ForceInput.Request
    assert (r.KIND_CONDITION, r.KIND_STATE, r.KIND_CLEAR) == (
        pode_admin.FORCE_CONDITION, pode_admin.FORCE_STATE, pode_admin.FORCE_CLEAR)
