"""로봇단 교착 처리(추가 지시서 3의 7-4, 기준(사용자))의 mode nav_goal 쪽. ROS 없이 돈다.

진전 없이 block_report_s가 지나면 RECOVERING(wait)으로 local_wait_s 기다린다(NavigateToPose는 그대로
돈다. BT의 복구는 지도 지우기와 기다리기뿐이라 움직이지 않는다). 앞을 막은 것이 관제가 아는 다른
로봇이고 내가 낮은 쪽이면 NavigateToPose를 멈추고, 선 뒤 지나온 길 안에서 곧게 물러나고, 같은
goal을 다시 보낸다. 아니면 BLOCKED(goal은 그대로). 선 때부터 blocked_after_s 안에 BLOCKED다.
"""
import math

import pytest
from pinky_fsm.backup import BackUp, CancelBackUp
from pinky_fsm.core import BLOCKED, MOVING, RECOVERING, FsmParams, LimitSpeed, PinkyFsm
from pinky_fsm.local_yield import BACKUP, RETRY, WAIT, Peer
from pinky_fsm.nav_goal import MODE_NAV_GOAL, CancelNavGoal, SendNavGoal

START = (0.35, 0.30, 0.0)
STOP = (0.95, 0.30, 0.0)  # 동쪽으로 0.60 m 와서 섰다
TARGET = (1.58, 0.30, 0.0)
PEER = (1.15, 0.30)  # 앞 0.20 m
STILL = (0.0, 0.0)
DRIVING = (0.12, 0.0)


def no_obstacle(_points):
    return False


def ready(name="pinky1", **params):
    fsm = PinkyFsm(FsmParams(**params), robot_id=name)
    fsm.tick(0.0, START, STILL, no_obstacle)
    return fsm


def of(cmds, kind):
    return [c for c in cmds if isinstance(c, kind)]


def kinds(cmds):
    return [type(c).__name__ for c in cmds if not isinstance(c, LimitSpeed)]


def tick(fsm, t, pose, speed=STILL):
    fsm.heartbeat(t)
    return fsm.tick(t, pose, speed, no_obstacle)


def drive_and_stand(fsm, priority=-1, until=1.0):
    """0 s에 goal을 받고 5 s 동안 동쪽으로 0.60 m 가서 선다. 선 뒤 until초까지 tick한다."""
    cmds = fsm.move("g1", 0.0, "N_E", TARGET, [], START, mode=MODE_NAV_GOAL, priority=priority)
    [send] = of(cmds, SendNavGoal)
    for k in range(51):
        t = k * 0.1
        x = START[0] + 0.6 * t / 5.0
        cmds += tick(fsm, t, (x, 0.30, 0.0), DRIVING)
    t = 5.1
    while t <= 5.0 + until + 1e-9:
        cmds += tick(fsm, t, STOP)
        t = round(t + 0.1, 6)
    return send, cmds


def test_standing_still_first_waits_as_recovering_and_keeps_navigate_to_pose():
    fsm = ready()
    _, cmds = drive_and_stand(fsm, until=1.1)
    assert fsm.state() == RECOVERING and fsm.local_yield_step() == WAIT
    assert kinds(cmds) == ["SendNavGoal"]  # 멈추지도 다시 보내지도 않는다


def turn_in_place(fsm, t0, t1, yaw, rate=1.0):
    states, t = [], t0
    while t <= t1 + 1e-9:
        tick(fsm, t, (*START[:2], yaw(t)), (0.0, rate))
        states.append((round(t, 1), fsm.state()))
        t = round(t + 0.1, 6)
    return states


def facing_west(fsm):
    return fsm.move("g1", 0.0, "N_E", (START[0] - 0.6, 0.30, math.pi), [], START,
                    mode=MODE_NAV_GOAL)


def test_turning_in_place_to_face_the_goal_is_not_a_stand():
    """Drive tuning 2026-11-14: Nav2 first turns to the path (a half turn takes about 3 s at
    1 rad/s). In Gazebo v7 a third of the waits began in such a turn and some ended BLOCKED."""
    fsm = ready()
    facing_west(fsm)
    states = turn_in_place(fsm, 0.0, 3.1, lambda t: min(t, math.pi))
    assert {s for _, s in states} == {MOVING}, states
    for k in range(10):
        tick(fsm, 3.2 + k * 0.1, (START[0] - 0.012 * k, 0.30, math.pi), DRIVING)
    assert fsm.state() == MOVING


def test_turning_delays_the_wait_at_most_nav_turn_window_s():
    """Spinning on the spot counts only for nav_turn_window_s (4 s) after the robot last moved:
    the wait starts by 5 s, so the fleet (stall_s 12 s) hears RECOVERING first, and with no robot
    ahead it is BLOCKED when the 2 s wait ends. The BLOCKED clock itself (blocked_after_s from
    when the robot last moved) never counts turns."""
    fsm = ready()
    facing_west(fsm)
    states = dict(turn_in_place(fsm, 0.0, 7.3, lambda t: t))
    assert states[4.8] == MOVING and states[5.2] == RECOVERING, states
    assert states[6.8] == RECOVERING and states[7.3] == BLOCKED, states


def test_a_turn_during_the_wait_does_not_end_it():
    fsm = ready()
    drive_and_stand(fsm, until=1.5)
    assert fsm.state() == RECOVERING
    t = 6.6
    for k in range(6):
        tick(fsm, t + k * 0.1, (*STOP[:2], 0.1 * (k + 1)), (0.0, 1.0))
    assert fsm.state() == RECOVERING and fsm.local_yield_step() == WAIT
    for k in range(12):
        tick(fsm, 7.2 + k * 0.1, (*STOP[:2], 0.6))
    assert fsm.state() == BLOCKED, "the wait still ends at its time"


def test_small_back_and_forth_turns_add_up():
    fsm = ready()
    facing_west(fsm)
    states = turn_in_place(fsm, 0.0, 3.0, lambda t: 0.15 if round(t * 10) % 2 else -0.15)
    assert {s for _, s in states} == {MOVING}, states


def test_turning_counts_only_when_asked():
    fsm = ready(nav_turn_progress_rad=0.0)
    facing_west(fsm)
    states = dict(turn_in_place(fsm, 0.0, 1.5, lambda t: t))
    assert states[1.5] == RECOVERING, states


def test_after_the_wait_with_no_robot_ahead_it_is_blocked_and_the_goal_stays():
    fsm = ready()
    _, cmds = drive_and_stand(fsm, until=3.2)
    assert fsm.state() == BLOCKED and fsm.is_running("g1")
    assert not of(cmds, BackUp) and not of(cmds, CancelNavGoal)


def test_moving_on_during_the_wait_ends_it():
    fsm = ready()
    drive_and_stand(fsm, until=1.5)
    tick(fsm, 6.6, (1.02, 0.30, 0.0), DRIVING)
    assert fsm.state() == MOVING and fsm.local_yield_step() == ""


def test_the_lower_robot_stops_backs_up_along_its_trail_and_sends_the_goal_again():
    fsm = ready()
    fsm.see_peers(5.5, [Peer("pinky2", *PEER, 3)])
    send, cmds = drive_and_stand(fsm, priority=0, until=3.2)
    assert of(cmds, CancelNavGoal) and not of(cmds, BackUp)
    assert fsm.local_yield_step() == BACKUP
    # NavigateToPose가 취소로 끝나고 바퀴가 서야 물러난다
    fsm.nav_goal_result(send.seq, "canceled", 8.3, STOP)
    [b] = of(tick(fsm, 8.4, STOP), BackUp)
    assert b.distance_m == pytest.approx(0.10)
    back = (0.85, 0.30, 0.0)
    cmds = fsm.backup_result(b.seq, "succeeded", 10.0, back)
    [again] = of(cmds, SendNavGoal)
    assert again.seq > send.seq and again.pose == TARGET
    assert fsm.state() == RECOVERING and fsm.local_yield_step() == RETRY
    for k in range(11):  # 되돌아가 또 섰다
        tick(fsm, 10.1 + k * 0.1, STOP)
    assert fsm.state() == BLOCKED and fsm.is_running("g1")


def test_the_higher_robot_does_not_back_up():
    fsm = ready()
    fsm.see_peers(5.5, [Peer("pinky2", *PEER, 0)])
    _, cmds = drive_and_stand(fsm, priority=3, until=3.2)
    assert not of(cmds, CancelNavGoal) and fsm.state() == BLOCKED


def test_with_nothing_driven_yet_there_is_nothing_to_back_along():
    fsm = ready()
    fsm.see_peers(0.5, [Peer("pinky2", 0.55, 0.30, 3)])
    fsm.move("g1", 0.0, "N_E", TARGET, [], START, mode=MODE_NAV_GOAL, priority=0)
    cmds = []
    for k in range(1, 34):
        cmds += tick(fsm, k * 0.1, START)
    assert not of(cmds, CancelNavGoal) and fsm.state() == BLOCKED


def test_blocked_comes_within_blocked_after_s_while_the_backup_does_not_answer():
    fsm = ready()
    fsm.see_peers(5.5, [Peer("pinky2", *PEER, 3)])
    send, _ = drive_and_stand(fsm, priority=0, until=3.2)
    fsm.nav_goal_result(send.seq, "canceled", 8.3, STOP)
    assert of(tick(fsm, 8.4, STOP), BackUp)
    # 선 때는 5.0 s다(마지막으로 진전을 본 때)
    assert not of(tick(fsm, 14.9, STOP), CancelBackUp)
    cmds = tick(fsm, 15.0, STOP)
    assert of(cmds, CancelBackUp) and of(cmds, SendNavGoal)
    assert fsm.state() == BLOCKED and fsm.is_running("g1")


def test_with_the_switch_off_nav_goal_is_as_before():
    fsm = ready(local_yield=False)
    _, cmds = drive_and_stand(fsm, until=4.0)
    assert fsm.state() == MOVING and kinds(cmds) == ["SendNavGoal"]
    assert math.isclose(FsmParams().blocked_after_s, 10.0)
