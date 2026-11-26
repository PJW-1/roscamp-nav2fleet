"""락이 풀린 Pode는 새 운반 건을 받지 않고 다시 투입도 LOCK_RELEASED로 거절한다 (W-34)."""
from core_helpers import cmds, codes
from test_manager import O1, drive, make


def ticks(core, n=40):
    out = []
    for _ in range(n):
        out += core.tick()
    return out


def test_a_released_robot_gets_no_new_trip():
    core, _ = make({"r1": "P-1"}, [O1])
    core.set_lock_released("r1", True, "set_lock id-1")
    out = ticks(core)
    assert not core.robot_view("r1")["order"]
    assert not cmds(out, "FillCmd")
    assert "ROBOT_LOCK_RELEASED" in codes(out)


def test_it_takes_trips_again_when_locked():
    core, _ = make({"r1": "P-1"}, [O1])
    core.set_lock_released("r1", True)
    ticks(core)
    core.set_lock_released("r1", False)
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert "ROBOT_LOCKED" in codes(out)


def test_the_same_lock_state_twice_gives_one_event():
    core, _ = make({"r1": "P-1"})
    for released in (True, True, False, False):
        core.set_lock_released("r1", released)
    out = core.tick()
    assert codes(out).count("ROBOT_LOCK_RELEASED") == 1
    assert codes(out).count("ROBOT_LOCKED") == 1


def test_lock_and_mode_blocks_are_separate():
    core, _ = make({"r1": "P-1"}, [O1])
    core.set_lock_released("r1", True)
    core.set_mode_blocked("r1", True, "x")
    core.set_lock_released("r1", False)
    ticks(core)
    assert not core.robot_view("r1")["order"]


def test_reinstate_of_a_released_robot_is_lock_released():
    core, _ = make({"r1": "P-1", "r2": "P-2"})
    core.confirm_clear("r2", core.zones.held_by("r2"), "lifted")
    core.set_lock_released("r2", True)
    assert core.reinstate("r2", "EMPTY", "back") is False
    assert core.last_reinstate_code == "LOCK_RELEASED"
    core.set_lock_released("r2", False)
    assert core.reinstate("r2", "EMPTY", "back") is True
    assert core.last_reinstate_code == ""


def test_reinstate_refusals_carry_a_contract_code():
    core, _ = make({"r1": "P-1", "r2": "P-2"})
    assert not core.reinstate("pode9", "EMPTY")
    assert core.last_reinstate_code == "UNKNOWN_TARGET"
    assert not core.reinstate("r1", "EMPTY")
    assert core.last_reinstate_code == "NOT_READY"
    core.confirm_clear("r2", core.zones.held_by("r2"), "lifted")
    assert not core.reinstate("r2", "FULL")
    assert core.last_reinstate_code == "ORDER_STATE"


def test_confirm_clear_refusals_carry_a_contract_code():
    core, _ = make({"r1": "P-1", "r2": "P-2"})
    assert core.confirm_clear("pode9", []) is None
    assert core.last_clear_code == "UNKNOWN_TARGET"
    assert core.confirm_clear("r1", ["NOPE"]) is None
    assert core.last_clear_code == "NOT_FOUND"
    assert core.confirm_clear("r1", []) is None  # 쥔 구역이 있는데 비움
    assert core.last_clear_code == "NOT_READY"
    assert core.confirm_clear("r1", core.zones.held_by("r1")) is not None
    assert core.last_clear_code == ""


def test_an_unknown_robot_is_rejected():
    core, _ = make({"r1": "P-1"})
    core.set_lock_released("nobody", True)
    assert "INPUT_REJECTED" in codes(core.tick())
