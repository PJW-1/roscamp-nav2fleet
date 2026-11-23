"""로봇 노드의 관리 명령 판단: 켜질 때 값(IR-72), 같은 번호 한 번만(IR-20 · IR-21)."""
from pode_fsm.admin import (
    BATTERY_LOW,
    MODE_MISMATCH,
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


def test_the_lock_opens_only_on_a_stopped_robot_and_closes_again():
    a = fresh()
    assert a.set_lock("r1", "pode1", False, stopped=False, now=0.0) == Reply(
        False, NOT_STOPPED, "the robot has a goal")
    assert a.lock is True and a.last_request_id == ""
    assert a.set_lock("r2", "pode1", False, stopped=True, now=1.0).accepted
    assert (a.lock, a.last_request_id) == (False, "r2")
    assert a.set_lock("r3", "pode1", True, stopped=False, now=2.0).accepted
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
