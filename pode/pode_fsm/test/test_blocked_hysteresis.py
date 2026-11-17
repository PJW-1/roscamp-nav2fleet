"""BLOCKED는 분명히 나아갔을 때만 풀린다(2026-11-17). ROS 없이 돈다.

좁은 곳에서 Nav2가 다시 계획하며 몇 cm 꿈틀대거나(nav_goal), 라이다 잡음으로 물체가 1초쯤
안 보이면(follow_path) BLOCKED가 풀렸다가 다시 켜져 MOVING, RECOVERING, BLOCKED가 번갈아 보고됐다.
이제 BLOCKED는 막힌 자리에서 unblock_m 벗어나거나(두 모드) 물체가 unblock_clear_s 동안 계속
안 보여야(follow_path) 풀린다. 물체가 block_clear_s 안 보이면 기다리던 경로는 그대로 바로 다시
보낸다(주행은 늦추지 않는다).
"""
import math

from pode_fsm.core import BLOCKED, IDLE, MOVING, FsmParams, LimitSpeed, PodeFsm, SendPath
from pode_fsm.nav_goal import MODE_NAV_GOAL

STILL = (0.0, 0.0)


def no_obstacle(_points):
    return False


def obstacle(_points):
    return True


def tick(fsm, t, pose, seen=no_obstacle, speed=STILL):
    fsm.heartbeat(t)
    return [c for c in fsm.tick(t, pose, speed, seen) if not isinstance(c, LimitSpeed)]


def steps(t0, t1, dt=0.1):
    k = 0
    while (t := round(t0 + k * dt, 6)) <= t1 + 1e-9:
        yield t
        k += 1


# ---- nav_goal: 동쪽으로 0.60 m 와서 서고, 기다린 뒤 BLOCKED ----
START = (0.35, 0.30, 0.0)
STOP = (0.95, 0.30, 0.0)
TARGET = (1.58, 0.30, 0.0)
DRIVING_NAV = (0.12, 0.0)


def nav_blocked():
    fsm = PodeFsm(FsmParams(), robot_id="pode1")
    fsm.tick(0.0, START, STILL, no_obstacle)
    fsm.move("g1", 0.0, "N_E", TARGET, [], START, mode=MODE_NAV_GOAL)
    for t in steps(0.0, 5.0):
        tick(fsm, t, (START[0] + 0.6 * t / 5.0, 0.30, 0.0), speed=DRIVING_NAV)
    for t in steps(5.1, 8.2):
        tick(fsm, t, STOP)
    assert fsm.state() == BLOCKED
    return fsm


def east(dx):
    return (STOP[0] + dx, STOP[1], 0.0)


def test_nav_goal_wiggling_a_few_cm_keeps_blocked():
    fsm = nav_blocked()
    states = []
    # Nav2가 다시 계획하며 앞뒤로 꿈틀댄다(가장 멀리 0.10 m)
    wiggle = [0.06, 0.10, 0.04, 0.0, 0.07, 0.02, 0.09, 0.0]
    t = 8.3
    for _ in range(3):
        for dx in wiggle:
            for _ in range(5):
                tick(fsm, t, east(dx), speed=DRIVING_NAV)
                states.append((t, fsm.state()))
                t = round(t + 0.1, 6)
    assert all(s == BLOCKED for _, s in states), [x for x in states if x[1] != BLOCKED][:3]
    assert fsm.is_running("g1")


def test_nav_goal_real_progress_clears_blocked():
    fsm = nav_blocked()
    tick(fsm, 8.3, east(0.10), speed=DRIVING_NAV)
    assert fsm.state() == BLOCKED
    tick(fsm, 8.6, east(0.16), speed=DRIVING_NAV)
    assert fsm.state() == MOVING


def test_nav_goal_unblock_m_is_a_parameter():
    fsm = PodeFsm(FsmParams(unblock_m=0.05), robot_id="pode1")
    fsm.tick(0.0, START, STILL, no_obstacle)
    fsm.move("g1", 0.0, "N_E", TARGET, [], START, mode=MODE_NAV_GOAL)
    for t in steps(0.0, 5.0):
        tick(fsm, t, (START[0] + 0.6 * t / 5.0, 0.30, 0.0), speed=DRIVING_NAV)
    for t in steps(5.1, 8.2):
        tick(fsm, t, STOP)
    assert fsm.state() == BLOCKED
    tick(fsm, 8.3, east(0.06), speed=DRIVING_NAV)
    assert fsm.state() == MOVING


# ---- follow_path: 물체 앞에 서서 기다린 뒤 BLOCKED(4.1 s) ----
P3, P4, P5 = (0.35, 1.55), (0.35, 1.95), (0.80, 2.05)
NORTH = math.pi / 2
STUCK = (0.35, 1.93, NORTH)
DRIVING = (0.2, 0.0)


def path_blocked():
    fsm = PodeFsm(FsmParams(), robot_id="pode1")
    fsm.tick(0.0, (*P3, NORTH), STILL, no_obstacle)
    yaw = math.atan2(P5[1] - P4[1], P5[0] - P4[0])
    fsm.move("g1", 0.0, "P-5", (*P5, yaw), [P3, P4, P5], (*P3, NORTH))
    tick(fsm, 1.0, STUCK, obstacle)
    tick(fsm, 2.05, STUCK, obstacle)
    tick(fsm, 4.1, STUCK, obstacle)
    assert fsm.state() == BLOCKED
    return fsm


def along_p4_p5(d):
    ux, uy = P5[0] - P4[0], P5[1] - P4[1]
    n = math.hypot(ux, uy)
    return (P4[0] + ux / n * d, P4[1] + uy / n * d, math.atan2(uy, ux))


def test_follow_path_a_one_second_gap_in_the_obstacle_keeps_blocked_but_resends():
    fsm = path_blocked()
    states, cmds = [], []
    for t in steps(4.2, 5.4):  # 라이다 잡음: 1.2 s 동안 안 보인다
        cmds += tick(fsm, t, STUCK, no_obstacle)
        states.append((t, fsm.state()))
    for t in steps(5.5, 7.0):
        cmds += tick(fsm, t, STUCK, obstacle)
        states.append((t, fsm.state()))
    assert all(s == BLOCKED for _, s in states), [x for x in states if x[1] != BLOCKED][:3]
    assert [type(c) for c in cmds] == [SendPath]  # 기다리던 경로는 1초 뒤 바로 다시 보낸다


def test_follow_path_obstacle_unseen_for_unblock_clear_s_clears_blocked():
    fsm = path_blocked()
    for t in steps(4.2, 7.1):
        tick(fsm, t, STUCK, no_obstacle)
    assert fsm.state() == BLOCKED
    tick(fsm, 7.25, STUCK, no_obstacle)  # 4.2 s부터 3 s
    assert fsm.state() == MOVING


def test_follow_path_real_progress_clears_blocked_before_unblock_clear_s():
    fsm = path_blocked()
    for t in steps(4.2, 5.3):
        tick(fsm, t, STUCK, no_obstacle)
    assert fsm.state() == BLOCKED
    tick(fsm, 5.6, along_p4_p5(0.10), no_obstacle, DRIVING)
    assert fsm.state() == BLOCKED  # 막힌 자리에서 0.11 m
    tick(fsm, 5.9, along_p4_p5(0.17), no_obstacle, DRIVING)
    assert fsm.state() == MOVING


def test_follow_path_after_cancel_blocked_stays_until_unblock_clear_s():
    fsm = path_blocked()
    fsm.cancel("g1", 4.2)
    tick(fsm, 4.3, STUCK, no_obstacle)
    tick(fsm, 5.5, STUCK, no_obstacle)
    assert fsm.state() == BLOCKED
    tick(fsm, 7.35, STUCK, no_obstacle)
    assert fsm.state() == IDLE
