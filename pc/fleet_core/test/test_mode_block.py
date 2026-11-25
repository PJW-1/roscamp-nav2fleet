"""운영 모드가 다른 Pode는 새 운반 건을 받지 않고, 모드가 맞으면 다시 받는다(IR-73). 하던 일은 그대로 한다."""
from core_helpers import cmds, codes
from test_manager import O1, drive, make


def ticks(core, n=40):
    out = []
    for _ in range(n):
        out += core.tick()
    return out


def test_a_robot_in_another_mode_gets_no_new_trip():
    core, _ = make({"r1": "P-1"}, [O1])
    core.set_mode_blocked("r1", True, "reports NORMAL, system TEST")
    out = ticks(core)
    assert not core.robot_view("r1")["order"]
    assert not cmds(out, "FillCmd")
    assert "ROBOT_MODE_MISMATCH" in codes(out)


def test_it_takes_trips_again_when_the_mode_matches():
    core, _ = make({"r1": "P-1"}, [O1])
    core.set_mode_blocked("r1", True, "reports NORMAL, system TEST")
    ticks(core)
    core.set_mode_blocked("r1", False)
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert "ROBOT_MODE_MATCHED" in codes(out)
    assert cmds(out, "FillCmd")[0].items == {"A": 1}


def test_another_robot_takes_the_trip_meanwhile():
    core, _ = make({"r1": "P-1", "r2": "P-2"}, [O1])
    core.set_mode_blocked("r1", True, "reports NORMAL, system TEST")
    drive(core, ("r1", "r2"), lambda: core.robot_view("r2")["order"] == "O-001")
    assert not core.robot_view("r1")["order"]


def test_the_same_answer_twice_gives_one_event():
    core, _ = make({"r1": "P-1"})
    core.set_mode_blocked("r1", True, "x")
    core.set_mode_blocked("r1", True, "x")
    core.set_mode_blocked("r1", False)
    core.set_mode_blocked("r1", False)
    out = core.tick()
    assert codes(out).count("ROBOT_MODE_MISMATCH") == 1
    assert codes(out).count("ROBOT_MODE_MATCHED") == 1


def test_an_unknown_robot_is_rejected():
    core, _ = make({"r1": "P-1"})
    core.set_mode_blocked("nobody", True, "x")
    assert "INPUT_REJECTED" in codes(core.tick())
