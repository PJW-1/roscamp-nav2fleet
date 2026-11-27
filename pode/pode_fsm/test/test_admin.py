"""로봇 노드의 관리 명령 판단: 켜질 때 값(IR-72), 같은 번호 한 번만(IR-20 · IR-21)."""
from pode_fsm.admin import (
    BATTERY_LOW,
    MODE_MISMATCH,
    NOT_FOUND,
    NOT_STOPPED,
    NOT_TEST_MODE,
    REMEMBER_COUNT,
    REMEMBER_S,
    UNKNOWN_TARGET,
    AdminState,
    Reply,
)


def fresh():
    return AdminState("pode1", "boot-1")


def test_a_fresh_robot_is_locked_in_unknown_mode_with_nothing_forced():
    a = fresh()
    assert (a.boot_id, a.lock, a.mode, a.forced, a.last_request_id) == (
        "boot-1", True, "UNKNOWN", [], "")


def test_the_lock_changes_only_on_a_stopped_robot():
    a = fresh()
    assert a.set_lock("r1", "pode1", False, stopped=False, now=0.0) == Reply(
        False, NOT_STOPPED, "the robot has a goal")
    assert a.lock is True and a.last_request_id == ""
    assert a.set_lock("r2", "pode1", False, stopped=True, now=1.0).accepted
    assert (a.lock, a.last_request_id) == (False, "r2")
    assert a.set_lock("r3", "pode1", True, stopped=False, now=2.0).reason == NOT_STOPPED
    assert a.lock is False
    assert a.set_lock("r4", "pode1", True, stopped=True, now=3.0).accepted
    assert a.lock is True


def test_asking_for_the_lock_it_already_has_is_accepted_while_moving():
    a = fresh()
    assert a.set_lock("r1", "pode1", True, stopped=False, now=0.0).accepted


def test_a_command_for_another_robot_is_refused():
    a = fresh()
    assert a.set_lock("r1", "pode2", False, stopped=True, now=0.0).reason == UNKNOWN_TARGET
    assert a.charge_end("r2", "arm_left", 90.0, now=0.0).reason == UNKNOWN_TARGET
    assert a.lock is True


def test_the_same_request_id_is_applied_once_and_answered_the_same():
    a = fresh()
    first = a.set_lock("r1", "pode1", False, stopped=True, now=0.0)
    assert a.set_lock("r9", "pode1", True, stopped=True, now=1.0).accepted
    again = a.set_lock("r1", "pode1", False, stopped=True, now=2.0)
    assert again == first and a.lock is True, "a repeat must not open the lock again"


def test_a_refusal_is_remembered_too():
    a = fresh()
    first = a.set_lock("r1", "pode1", False, stopped=False, now=0.0)
    assert a.set_lock("r1", "pode1", False, stopped=True, now=1.0) == first
    assert a.lock is True


def test_request_ids_are_forgotten_after_sixty_seconds():
    a = fresh()
    a.set_lock("r1", "pode1", False, stopped=True, now=0.0)
    a.set_lock("r2", "pode1", True, stopped=True, now=1.0)
    assert a.set_lock("r1", "pode1", False, stopped=True, now=1.0 + REMEMBER_S).accepted
    assert a.lock is False, "r1 was forgotten, so it is a new request"


def test_only_the_last_thirty_two_request_ids_are_kept():
    a = fresh()
    a.set_mode("old", "TEST", now=0.0)
    for i in range(REMEMBER_COUNT):
        a.set_mode(f"r{i}", "NORMAL", now=1.0)
    assert a.mode == "NORMAL"
    assert a.set_mode("old", "TEST", now=2.0).accepted and a.mode == "TEST"


def test_the_mode_is_a_target_value_and_unknown_values_are_refused():
    a = fresh()
    assert a.set_mode("r1", "TEST", now=0.0).accepted and a.mode == "TEST"
    assert a.set_mode("r2", "TEST", now=0.0).accepted and a.mode == "TEST"
    bad = a.set_mode("r3", "UNKNOWN", now=0.0)
    assert (bad.accepted, bad.reason, a.mode) == (False, MODE_MISMATCH, "TEST")


def test_charge_end_needs_test_mode_and_thirty_five_percent():
    a = fresh()
    assert a.charge_end("r1", "pode1", 90.0, now=0.0).reason == NOT_TEST_MODE
    a.set_mode("r2", "TEST", now=0.0)
    assert a.charge_end("r3", "pode1", 34.9, now=0.0).reason == BATTERY_LOW
    assert a.charge_end("r4", "pode1", -1.0, now=0.0).reason == BATTERY_LOW
    assert a.charge_end("r5", "pode1", 35.0, now=0.0).accepted


def test_an_empty_request_id_is_not_remembered():
    a = fresh()
    a.set_lock("", "pode1", False, stopped=True, now=0.0)
    assert a.last_request_id == ""
    a.set_lock("", "pode1", True, stopped=True, now=1.0)
    assert a.lock is True


def test_force_needs_test_mode_and_remembers_key_and_value():
    a = fresh()
    assert a.force("f1", "pode1", "CONDITION", "battery", "20", now=0.0).reason == NOT_TEST_MODE
    assert a.forced == []
    a.set_mode("m1", "TEST", now=0.0)
    assert a.force("f2", "pode1", "CONDITION", "battery", "20", now=0.0) == Reply(True)
    assert a.force("f3", "pode1", "STATE", "PAUSED", "1", now=0.0).accepted
    assert a.forced == ["battery", "PAUSED"]
    assert a.forced_values == {"battery": "20", "PAUSED": "1"}
    # 같은 key는 값만 바꾼다
    assert a.force("f4", "pode1", "CONDITION", "battery", "10", now=0.0).accepted
    assert a.forced_values == {"battery": "10", "PAUSED": "1"}
    assert a.last_request_id == "f4"


def test_force_clear_drops_one_key_or_all_with_an_empty_key():
    a = fresh()
    a.set_mode("m1", "TEST", now=0.0)
    a.force("f1", "pode1", "CONDITION", "battery", "20", now=0.0)
    a.force("f2", "pode1", "STATE", "PAUSED", "1", now=0.0)
    assert a.force("f3", "pode1", "CLEAR", "battery", "", now=0.0).accepted
    assert a.forced_values == {"PAUSED": "1"}
    assert a.force("f4", "pode1", "CLEAR", "nothing", "", now=0.0).accepted
    assert a.force("f5", "pode1", "CLEAR", "", "", now=0.0).accepted
    assert a.forced == [] and a.forced_values == {}


def test_force_refuses_another_robot_and_an_unknown_kind():
    a = fresh()
    a.set_mode("m1", "TEST", now=0.0)
    assert a.force("f1", "pode2", "CONDITION", "x", "1", now=0.0).reason == UNKNOWN_TARGET
    assert a.force("f2", "pode1", "TOGGLE", "x", "1", now=0.0).reason == NOT_FOUND
    assert a.forced == []


def test_force_with_the_same_request_id_is_applied_once():
    a = fresh()
    a.set_mode("m1", "TEST", now=0.0)
    a.force("same", "pode1", "CONDITION", "battery", "20", now=0.0)
    a.force("f2", "pode1", "CLEAR", "", "", now=1.0)
    assert a.force("same", "pode1", "CONDITION", "battery", "20", now=2.0).accepted
    assert a.forced == []
