"""관제 반응 시간 상한(추가 지시서 2의 3절): BLOCKED 뒤 첫 사건 1 s, 규칙 7의 다음 단계는 + 5 s."""
from fake_robots.world.reaction import REACT_FIRST, REACT_NEXT, ReactionWatch
from fake_robots.world.robot import SimRobot
from fleet_core.common.commands import CancelCmd, EventOut
from fleet_core.common.params import load_params
from world_helpers import nav, place, quiet_world

PARAMS = load_params()  # blocked_mark_s 10


class _Run:
    def __init__(self, goal_id, blocked=True):
        self.goal_id, self.blocked = goal_id, blocked


def _robot(goal_id=7, blocked=True):
    r = SimRobot("pinky1", 0.5, 0.5)
    r.run = _Run(goal_id, blocked)
    return r


def _watch():
    out = []
    return ReactionWatch(PARAMS, out), out


def _ev(code, robot="pinky1"):
    return EventOut("INFO", code, "", robot)


def test_a_blocked_report_wants_a_fleet_event_about_that_robot_within_one_second():
    w, out = _watch()
    robots = {"pinky1": _robot()}
    w.on_report(1.0, "pinky1", "BLOCKED")
    w.check(1.9, robots)
    w.on_event(1.9, _ev("ZONE_WAIT", "pinky2"), None)  # 다른 로봇의 사건은 반응이 아니다
    w.check(2.0, robots)
    assert out == []
    w.check(2.1, robots)
    assert [v.code for v in out] == [REACT_FIRST]
    w.check(2.2, robots)
    assert len(out) == 1  # 막힘 하나에 한 번


def test_an_answer_in_time_or_a_short_block_is_fine():
    w, out = _watch()
    robots = {"pinky1": _robot()}
    w.on_report(1.0, "pinky1", "BLOCKED")
    w.on_event(1.5, _ev("BLOCKED_WAIT"), 7)
    w.check(3.0, robots)
    w.on_report(3.0, "pinky1", "MOVING")
    w.on_report(4.0, "pinky1", "BLOCKED")
    w.on_report(4.5, "pinky1", "MOVING")
    w.check(6.0, robots)
    assert out == []


def test_blocked_wait_wants_the_next_stage_within_blocked_mark_s_plus_five():
    w, out = _watch()
    robots = {"pinky1": _robot()}
    w.on_report(1.0, "pinky1", "BLOCKED")
    w.on_event(1.1, _ev("BLOCKED_WAIT"), 7)
    w.check(16.1, robots)
    assert out == []
    w.check(16.2, robots)
    assert [v.code for v in out] == [REACT_NEXT]


def test_the_next_stage_is_an_event_or_the_goal_withdrawn_or_the_robot_moving_on():
    for answer in ("event", "cancel", "moved", "new goal"):
        w, out = _watch()
        robots = {"pinky1": _robot()}
        w.on_report(1.0, "pinky1", "BLOCKED")
        w.on_event(1.1, _ev("BLOCKED_WAIT"), 7)
        if answer == "event":
            w.on_event(11.1, _ev("ZONE_MARKED_BLOCKED"), 7)
        elif answer == "cancel":
            w.on_command(11.1, CancelCmd("pinky1", 7))
        elif answer == "moved":
            robots["pinky1"].run.blocked = False
        else:
            robots["pinky1"].run = _Run(8)
        w.check(30.0, robots)
        assert out == [], answer


def test_a_robot_out_of_contact_is_not_judged():
    w, out = _watch()
    r = _robot()
    r.comm_up = False
    w.on_report(1.0, "pinky1", "BLOCKED")
    w.on_event(1.1, _ev("BLOCKED_WAIT"), 7)
    w.check(30.0, {"pinky1": r})
    assert out == []


def test_a_blocked_report_after_a_comm_loss_starts_a_new_deadline():
    w, out = _watch()
    r = _robot()
    w.on_report(1.0, "pinky1", "BLOCKED")
    r.comm_up = False
    w.check(1.5, {"pinky1": r})
    r.comm_up = True
    w.on_report(5.0, "pinky1", "BLOCKED")  # 끊긴 뒤 처음 닿은 보고도 BLOCKED
    w.check(6.1, {"pinky1": r})
    assert [v.code for v in out] == [REACT_FIRST]


def test_cancelling_an_older_goal_is_not_the_next_stage():
    w, out = _watch()
    robots = {"pinky1": _robot(goal_id=7)}
    w.on_report(1.0, "pinky1", "BLOCKED")
    w.on_event(1.1, _ev("BLOCKED_WAIT"), 7)
    w.on_command(5.0, CancelCmd("pinky1", 6))
    w.check(16.2, robots)
    assert [v.code for v in out] == [REACT_NEXT]


def test_the_world_records_a_silent_fleet_as_a_reaction_violation():
    """관제 대역은 아무 사건도 내지 않는다: 막힌 로봇의 BLOCKED에 답이 없다."""
    world = quiet_world(robots={"pinky1": "N_W", "pinky2": "P-2"}, avoid_mode="none")
    place(world, "pinky2", 1.0, 0.65)
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.run(15.0)
    assert [v.code for v in world.violations if v.code.startswith("I9")] == [REACT_FIRST]
