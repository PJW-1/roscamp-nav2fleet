"""MoveTo mode nav_goal (contract 0.11-draft proposal): the decision part, without ROS.

The robot sends NavigateToPose to target_pose and Nav2 plans and recovers by itself. pinky_fsm
reports BLOCKED after blocked_after_s without progress, keeps the goal running, and hands over
between Nav2 servers (bt_navigator, controller, behavior_server) only after the old motion ended.
"""
import math

import pytest
from pinky_fsm.backup import BackUp, CancelBackUp
from pinky_fsm.core import (
    BLOCKED,
    CANCELED,
    HEARTBEAT_LOST,
    IDLE,
    MOVING,
    NAV_ABORTED,
    TIMEOUT,
    CancelNav,
    Finish,
    FsmParams,
    LimitSpeed,
    PinkyFsm,
    SendPath,
)
from pinky_fsm.nav_goal import MODE_FOLLOW_PATH, MODE_NAV_GOAL, CancelNavGoal, SendNavGoal

HERE = (0.35, 0.30, 0.0)
TARGET = (1.58, 0.65, math.pi / 2)
STILL = (0.0, 0.0)
DRIVING = (0.12, 0.0)


def no_obstacle(_points):
    return False


def obstacle(_points):
    return True


def ready(t=0.0, pose=HERE, **params):
    # 이 파일은 로봇단 교착 처리(7-4) 전의 nav_goal을 본다(스위치 끔, 0.12의 blocked_after_s 5 s).
    # 켠 것은 test_yield_nav.py
    params.setdefault("local_yield", False)
    params.setdefault("blocked_after_s", 5.0)
    fsm = PinkyFsm(FsmParams(**params))
    fsm.tick(t, pose, STILL, no_obstacle)
    return fsm


def nav(fsm, gid, t, pose=HERE, target=TARGET, path=(), target_id="N_E"):
    return fsm.move(gid, t, target_id, target, list(path), pose, mode=MODE_NAV_GOAL)


def of(cmds, kind):
    return [c for c in cmds if isinstance(c, kind)]


def kinds(cmds):
    return [type(c).__name__ for c in cmds if not isinstance(c, LimitSpeed)]


def run(fsm, t0, t1, pose=HERE, speed=STILL, step=0.1, seen=no_obstacle):
    """tick from t0 to t1 with the heartbeat alive; returns every command."""
    cmds, k = [], 0
    while (t := t0 + k * step) <= t1 + 1e-9:
        fsm.heartbeat(t)
        cmds += fsm.tick(t, pose(t) if callable(pose) else pose, speed, seen)
        k += 1
    return cmds


# ---- start, result, feedback -------------------------------------------------------------


def test_a_nav_goal_sends_navigate_to_pose_to_the_target():
    fsm = ready()
    cmds = nav(fsm, "g1", 1.0)
    [send] = of(cmds, SendNavGoal)
    assert kinds(cmds) == ["SendNavGoal"]
    assert send.pose == pytest.approx(TARGET) and send.seq >= 1
    assert fsm.state() == MOVING and fsm.current_goal() == "N_E"


def test_a_nav_goal_ignores_the_path_and_never_checks_it_is_connected():
    """The fleet manager leaves path empty in nav_goal; a stray path is not followed."""
    fsm = ready()
    cmds = nav(fsm, "g1", 1.0, path=[(1.9, 0.9), (1.8, 0.9)])
    assert kinds(cmds) == ["SendNavGoal"]


@pytest.mark.parametrize("mode", ["", MODE_FOLLOW_PATH])
def test_an_empty_or_follow_path_mode_is_todays_follow_path(mode):
    fsm = ready()
    cmds = fsm.move("g1", 1.0, "P", (0.8, 0.3, 0.0), [], HERE, mode=mode)
    [send] = of(cmds, SendPath)
    assert send.poses[0][:2] == pytest.approx(HERE[:2]) and not of(cmds, SendNavGoal)


def test_an_unknown_mode_fails_without_driving():
    fsm = ready()
    cmds = fsm.move("g1", 1.0, "P", TARGET, [], HERE, mode="teleport")
    assert cmds == [Finish("g1", False, NAV_ABORTED)]
    assert fsm.state() == IDLE


def test_success_close_to_the_target_finishes_the_goal():
    fsm = ready()
    [send] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    cmds = fsm.nav_goal_result(send.seq, "succeeded", 9.0, (1.55, 0.63, 1.5))
    assert Finish("g1", True, "") in cmds
    assert fsm.state() == IDLE and fsm.current_goal() == ""


def test_a_success_far_from_the_target_sends_the_goal_again_then_gives_up():
    fsm = ready(far_success_limit=1)
    [send] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    [again] = of(fsm.nav_goal_result(send.seq, "succeeded", 9.0, (1.2, 0.6, 0.0)), SendNavGoal)
    assert again.seq > send.seq and again.pose == pytest.approx(TARGET)
    cmds = fsm.nav_goal_result(again.seq, "succeeded", 12.0, (1.2, 0.6, 0.0))
    assert Finish("g1", False, NAV_ABORTED) in cmds


@pytest.mark.parametrize("outcome", ["aborted", "rejected", "canceled"])
def test_a_nav2_failure_ends_the_goal_with_nav_aborted(outcome):
    fsm = ready()
    [send] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    cmds = fsm.nav_goal_result(send.seq, outcome, 5.0, HERE)
    assert Finish("g1", False, NAV_ABORTED) in cmds
    assert fsm.state() == IDLE


def test_the_result_of_a_replaced_nav_goal_is_ignored():
    fsm = ready()
    [first] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    nav(fsm, "g2", 2.0)
    assert fsm.nav_goal_result(first.seq, "aborted", 2.1, HERE) == []
    assert fsm.is_running("g2")


def test_distance_remaining_comes_from_nav2_feedback():
    fsm = ready()
    [send] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    straight = math.dist(HERE[:2], TARGET[:2])
    assert fsm.distance_remaining(HERE) == pytest.approx(straight), "before any feedback"
    fsm.nav_goal_feedback(send.seq, 1.9)
    assert fsm.distance_remaining(HERE) == pytest.approx(1.9)
    fsm.nav_goal_feedback(send.seq - 1, 0.1)  # an older goal's feedback
    assert fsm.distance_remaining(HERE) == pytest.approx(1.9)


# ---- stop: cancel, heartbeat, pose --------------------------------------------------------


def test_cancel_stops_navigate_to_pose():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    assert fsm.cancel("g1", 2.0) == [Finish("g1", False, CANCELED), CancelNavGoal()]
    assert fsm.state() == IDLE


def test_heartbeat_loss_stops_navigate_to_pose():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    t = 1.0 + FsmParams().heartbeat_timeout_s
    cmds = fsm.tick(t, HERE, DRIVING, no_obstacle)
    assert kinds(cmds) == ["Finish", "CancelNavGoal"]
    assert cmds[0] == Finish("g1", False, HEARTBEAT_LOST)


def test_losing_the_pose_stops_navigate_to_pose():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    fsm.heartbeat(1.5)
    cmds = fsm.tick(1.5, None, DRIVING, no_obstacle)
    assert kinds(cmds) == ["Finish", "CancelNavGoal"]


def test_a_finished_nav_goal_is_not_cancelled_again():
    fsm = ready()
    [send] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    fsm.nav_goal_result(send.seq, "aborted", 2.0, HERE)
    nav(fsm, "g2", 3.0)
    assert fsm.cancel("g2", 3.5) == [Finish("g2", False, CANCELED), CancelNavGoal()]
    assert of(fsm.cancel("g2", 3.6), CancelNavGoal) == []


# ---- BLOCKED (no progress for blocked_after_s) --------------------------------------------


def test_no_progress_for_blocked_after_s_reports_blocked_and_keeps_the_goal():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    cmds = run(fsm, 1.1, 5.9)
    assert fsm.state() == MOVING
    cmds += run(fsm, 6.0, 8.0)
    assert fsm.state() == BLOCKED and fsm.is_running("g1")
    assert not of(cmds, Finish) and not of(cmds, CancelNavGoal) and not of(cmds, SendNavGoal)


def test_standing_with_an_obstacle_ahead_never_starts_the_follow_path_self_recovery():
    """User decision 2026-11-12: Nav2's behavior tree recovers in nav_goal, not pinky_fsm."""
    fsm = ready()
    nav(fsm, "g1", 1.0)
    cmds = run(fsm, 1.1, 20.0, seen=obstacle)
    assert not of(cmds, BackUp) and fsm.recovery_attempts_used == 0
    assert fsm.state() == BLOCKED


def test_moving_on_clears_blocked():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    run(fsm, 1.1, 7.0)
    assert fsm.state() == BLOCKED
    run(fsm, 7.1, 7.1, pose=(HERE[0] + 0.03, *HERE[1:]))
    assert fsm.state() == BLOCKED, "3 cm is not progress"
    run(fsm, 7.2, 7.2, pose=(HERE[0] + 0.06, *HERE[1:]))
    assert fsm.state() == MOVING


def test_slow_steady_driving_is_progress():
    """0.05 m every 4 s never stands still for blocked_after_s."""
    fsm = ready()
    nav(fsm, "g1", 1.0)
    run(fsm, 1.1, 30.0, pose=lambda t: (HERE[0] + 0.0125 * (t - 1.0), *HERE[1:]))
    assert fsm.state() == MOVING


def test_turning_in_place_is_not_progress():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    run(fsm, 1.1, 7.0, pose=lambda t: (*HERE[:2], t), speed=(0.0, 1.0))
    assert fsm.state() == BLOCKED


def test_blocked_after_s_is_a_parameter():
    fsm = ready(blocked_after_s=2.0)
    nav(fsm, "g1", 1.0)
    run(fsm, 1.1, 3.1)
    assert fsm.state() == BLOCKED


def test_blocked_ends_with_the_goal():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    run(fsm, 1.1, 7.0)
    fsm.cancel("g1", 7.1)
    assert fsm.state() == IDLE


# ---- hand-over between Nav2 servers ------------------------------------------------------


def test_a_new_nav_goal_replaces_a_running_one_without_stopping():
    """bt_navigator preempts its goal like the controller does for FollowPath (guide 2.1)."""
    fsm = ready()
    nav(fsm, "g1", 1.0)
    cmds = nav(fsm, "g2", 2.0, target=(1.78, 0.55, 0.0))
    assert kinds(cmds) == ["Finish", "SendNavGoal"]
    assert cmds[0] == Finish("g1", False, CANCELED)


def test_a_follow_path_move_waits_until_the_nav_goal_has_ended_and_the_robot_stands():
    fsm = ready()
    [ng] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    cmds = fsm.move("g2", 2.0, "J_E", (1.58, 0.89, math.pi / 2), [(1.58, 0.65), (1.58, 0.89)],
                    (1.58, 0.65, math.pi / 2))
    assert kinds(cmds) == ["Finish", "CancelNavGoal"]
    assert cmds[0] == Finish("g1", False, CANCELED)
    assert fsm.state() == MOVING and fsm.current_goal() == "J_E"
    pose = (1.58, 0.65, math.pi / 2)
    assert not of(run(fsm, 2.1, 2.3, pose=pose, speed=DRIVING), SendPath), "still rolling"
    assert not of(run(fsm, 2.4, 2.6, pose=pose), SendPath), "no NavigateToPose result yet"
    fsm.nav_goal_result(ng.seq, "canceled", 2.65, pose)
    [send] = of(run(fsm, 2.7, 2.7, pose=pose), SendPath)
    assert send.poses[-1][:2] == pytest.approx((1.58, 0.89))
    assert fsm.is_running("g2")


def test_a_waiting_follow_path_move_does_not_start_a_self_recovery():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    fsm.move("g2", 2.0, "P", (0.8, 0.3, 0.0), [], HERE)
    cmds = run(fsm, 2.1, 2.9, seen=obstacle)
    assert not of(cmds, BackUp) and fsm.state() == MOVING


def test_a_waiting_follow_path_move_times_out_if_the_nav_goal_never_ends():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    fsm.move("g2", 2.0, "P", (0.8, 0.3, 0.0), [], HERE)
    cmds = run(fsm, 2.1, 2.0 + FsmParams().retreat_stop_wait_s + 0.1)
    assert Finish("g2", False, TIMEOUT) in cmds and not of(cmds, SendPath)


def test_an_accepted_cancel_lets_the_move_go_after_the_result_grace():
    fsm = ready()
    [ng] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    fsm.move("g2", 2.0, "P", (0.8, 0.3, 0.0), [], HERE)
    fsm.nav_goal_cancel_accepted(ng.seq, 2.1)
    grace = FsmParams().nav_result_grace_s
    assert not of(run(fsm, 2.2, 2.0 + grace), SendPath)
    assert of(run(fsm, 2.2 + grace, 2.2 + grace), SendPath)


def test_a_nav_goal_waits_until_a_running_follow_path_has_ended():
    fsm = ready()
    [fp] = of(fsm.move("g1", 1.0, "P", (0.8, 0.3, 0.0), [], HERE), SendPath)
    cmds = nav(fsm, "g2", 2.0)
    assert kinds(cmds) == ["Finish", "CancelNav"]
    assert not of(run(fsm, 2.1, 2.3), SendNavGoal)
    fsm.nav_result(fp.seq, "canceled", 2.35, HERE, no_obstacle)
    [send] = of(run(fsm, 2.4, 2.4), SendNavGoal)
    assert send.pose == pytest.approx(TARGET) and fsm.is_running("g2")


def test_a_nav_goal_waits_until_a_self_recovery_backup_has_ended():
    fsm = ready(release_distance_m=0.0)
    [fp] = of(fsm.move("g1", 1.0, "P", (1.0, 0.3, 0.0), [], HERE), SendPath)
    on = (HERE[0] + 0.2, *HERE[1:])
    [b] = of(run(fsm, 1.1, 3.0, pose=on, seen=obstacle), BackUp)
    fsm.nav_result(fp.seq, "canceled", 3.05, on, obstacle)  # the self-recovery stopped it
    cmds = nav(fsm, "g2", 3.1, pose=on)
    assert CancelBackUp() in cmds and not of(cmds, SendNavGoal)
    assert not of(run(fsm, 3.2, 3.4, pose=on), SendNavGoal)
    fsm.backup_result(b.seq, "canceled", 3.45, on)
    assert of(run(fsm, 3.5, 3.5, pose=on), SendNavGoal)


def test_a_retreat_cancels_the_nav_goal_and_backs_up_after_it_ended():
    fsm = ready()
    [ng] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    cmds = fsm.retreat("r1", 2.0, 0.2, 0.1, pose=HERE)
    assert kinds(cmds) == ["Finish", "CancelNavGoal"]
    assert not of(run(fsm, 2.1, 2.3), BackUp), "no NavigateToPose result yet"
    fsm.nav_goal_result(ng.seq, "canceled", 2.35, HERE)
    [b] = of(run(fsm, 2.4, 2.4), BackUp)
    assert (b.distance_m, b.speed_mps) == (0.2, 0.1)


def test_a_dock_cancels_the_nav_goal():
    fsm = ready()
    nav(fsm, "g1", 1.0)
    cmds = fsm.dock("d1", 2.0, "WORK")
    assert cmds[:2] == [Finish("g1", False, CANCELED), CancelNavGoal()]


def test_a_lost_nav_goal_result_ends_the_goal_but_keeps_waiting_for_the_cancel():
    """The goal may still run when only its result was lost: a later motion waits for the
    cancel to be accepted, as for a FollowPath whose result was lost."""
    fsm = ready()
    [ng] = of(nav(fsm, "g1", 1.0), SendNavGoal)
    cmds = fsm.nav_goal_result(ng.seq, "aborted", 2.0, HERE, lost=True)
    assert Finish("g1", False, NAV_ABORTED) in cmds
    fsm.retreat("r1", 2.1, 0.2, 0.1, pose=HERE)
    assert not of(run(fsm, 2.2, 2.6), BackUp)
    fsm.nav_goal_cancel_accepted(ng.seq, 2.6)
    assert of(run(fsm, 2.7 + FsmParams().nav_result_grace_s, 2.7 + FsmParams().nav_result_grace_s),
              BackUp)


def test_a_nav_goal_lifts_a_lock_boundary_speed_limit():
    fsm = ready()
    fsm.move("g1", 1.0, "P", (0.5, 0.3, 0.0), [], HERE, final_goal=False)
    assert any(c.mps > 0 for c in of(run(fsm, 1.1, 1.1), LimitSpeed))
    nav(fsm, "g2", 2.0)
    assert LimitSpeed(0.0) in run(fsm, 2.1, 2.1)


def test_cancel_nav_is_still_only_for_follow_path():
    """CancelNav (FollowPath) and CancelNavGoal (NavigateToPose) stay apart."""
    fsm = ready()
    nav(fsm, "g1", 1.0)
    assert not of(fsm.cancel("g1", 2.0), CancelNav)
