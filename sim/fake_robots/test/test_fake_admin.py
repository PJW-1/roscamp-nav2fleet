"""가짜 로봇 쪽 관리 명령 판단: 강제 입력, 장애 처리 완료, 팔 락 (IR-21 · IR-61 · IR-63)."""
import pytest

pytest.importorskip("fleet_interfaces.srv")

from fake_robots.fake_admin import ARM_STOPPED, FakeAdmin  # noqa: E402
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


def test_an_arm_counts_as_stopped_like_the_fleet_sees_it():
    assert set(ARM_STOPPED) == {"IDLE", "PAUSED", "ERROR"}
    for moving in ("PICKING", "PLACING", "SWAPPING"):
        assert moving not in ARM_STOPPED


def test_a_hook_can_refuse_a_force_and_the_value_is_not_kept():
    a = admin()
    seen = []

    def hook(kind, key, value):
        seen.append((kind, key, value))
        return "bad value" if value == "x" else ""

    reply = a.force_hooked("f1", "arm_left", "CONDITION", "fill_fail", "x", 0.0, hook)
    assert (reply.accepted, reply.reason) == (False, "NOT_FOUND") and a.forced == []
    assert a.force_hooked("f2", "arm_left", "CONDITION", "fill_fail", "ok", 0.0, hook).accepted
    assert a.forced_values == {"fill_fail": "ok"} and a.last_request_id == "f2"
    assert a.force_hooked("f3", "arm_left", "CLEAR", "", "", 0.0, hook).accepted
    assert len(seen) == 2, "CLEAR does not go to the hook"
    # 시험 모드가 아니면 훅까지 가지 않는다
    a.mode = "NORMAL"
    assert a.force_hooked("f4", "arm_left", "CONDITION", "k", "x", 0.0, hook).reason == \
        "NOT_TEST_MODE"
    assert len(seen) == 2


def test_a_refused_replacement_keeps_the_earlier_hook_value():
    a = admin()

    def hook(kind, key, value):
        return "bad value" if value == "x" else ""

    assert a.force_hooked("f1", "arm_left", "CONDITION", "fill_fail", "ok", 0.0, hook).accepted
    assert not a.force_hooked("f2", "arm_left", "CONDITION", "fill_fail", "x", 0.0, hook).accepted
    assert a.forced_values == {"fill_fail": "ok"}


def test_set_pause_is_judged_once_per_request_id():
    a = admin()
    calls = []

    def apply(paused):
        calls.append(paused)
        return pode_admin.Reply(True)

    assert a.set_pause("p1", True, 0.0, apply).accepted
    assert a.set_pause("p1", False, 1.0, apply).accepted
    assert calls == [True] and a.last_request_id == "p1"


def test_a_reboot_brings_back_the_boot_values():
    a = admin()
    a.lock = False
    a.force("f1", "arm_left", "CONDITION", "battery", "20", 0.0)
    a.reboot("boot-2")
    assert (a.boot_id, a.lock, a.mode, a.forced, a.last_request_id) == (
        "boot-2", True, "UNKNOWN", [], "")
    a.mode = "TEST"
    assert a.force("f1", "arm_left", "CLEAR", "", "", 1.0).accepted, "request ids forgotten"
