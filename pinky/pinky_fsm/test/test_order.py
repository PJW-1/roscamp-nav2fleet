"""order model (contract 0.11-draft proposal, order addendum 3 and 6-1): the decision part, no ROS.

A MoveTo with an order_id carries base segments (released, the robot may drive them) and horizon
segments (not released, for display and a pre-check only). Updates arrive as new goals and are
stitched onto the queue under four conditions. The robot never drives past the end of base.
"""
import math

import pytest
from pinky_fsm.core import (
    CANCELED,
    HEARTBEAT_LOST,
    IDLE,
    MOVING,
    PATH_NOT_CONNECTED,
    CancelNav,
    Finish,
    FsmParams,
    LimitSpeed,
    PinkyFsm,
    SendPath,
)
from pinky_fsm.nav_goal import CancelNavGoal, SendNavGoal
from pinky_fsm.order_fsm import (
    HORIZON_INVALID,
    ORDER_UPDATE_REJECTED,
    SUPERSEDED,
    OrderFsm,
)
from pinky_fsm.order_queue import MODE_FOLLOW_PATH, MODE_NAV_GOAL, MODE_RETREAT, OrderGoal, Seg

HERE = (0.3, 0.3, 0.0)
STILL = (0.0, 0.0)
DRIVING = (0.12, 0.0)
FP, THROUGH, REVERSE = "FollowPath", "FollowPathThrough", "FollowPathReverse"


def no_obstacle(_points):
    return False


def line(seq, x0, x1, y=0.3, ctrl=FP, node=None, mode=MODE_FOLLOW_PATH):
    yaw = 0.0 if x1 >= x0 else math.pi
    return Seg(seq, mode, ctrl, ((x0, y, yaw), (x1, y, yaw)), None, node or f"N{seq}")


def goal_seg(seq, x, y, node=None):
    return Seg(seq, MODE_NAV_GOAL, "", (), (x, y, 0.0), node or f"N{seq}")


S1, S2, S3 = line(1, 0.3, 0.8), line(2, 0.8, 1.3), line(3, 1.3, 1.8)


def ready(t=0.0, pose=HERE, check=None, **params):
    fsm = OrderFsm(FsmParams(**params), horizon_check=check)
    fsm.tick(t, pose, STILL, no_obstacle)
    return fsm


def order(fsm, gid, t, base, horizon=(), update=0, stitch="", oid="o1", pose=HERE):
    return fsm.move(gid, t, "", (0.0, 0.0, 0.0), [], pose,
                    order=OrderGoal(oid, update, stitch, tuple(base), tuple(horizon)))


def of(cmds, kind):
    return [c for c in cmds if isinstance(c, kind)]


def kinds(cmds):
    return [type(c).__name__ for c in cmds if not isinstance(c, LimitSpeed)]


def finishes(cmds):
    return of(cmds, Finish)


def arrive(fsm, send, t, pose):
    """The FollowPath ends with success where the robot stands (the latest one: near the end
    pinky_fsm sends the rest again, R13)."""
    fsm.heartbeat(t)
    again = of(fsm.tick(t, pose, STILL, no_obstacle), SendPath)
    seq = again[-1].seq if again else send.seq
    return fsm.nav_result(seq, "succeeded", t, pose, no_obstacle)


# ---- empty order_id ---------------------------------------------------------------------


def test_an_empty_order_id_is_todays_move_to():
    plain, ordered = ready(), ready()
    a = plain.move("g1", 1.0, "P", (0.8, 0.3, 0.0), [], HERE)
    b = ordered.move("g1", 1.0, "P", (0.8, 0.3, 0.0), [], HERE, order=OrderGoal(""))
    assert a == b and ordered.order_feedback() == (0, False, "")


# ---- base end: the invariant -----------------------------------------------------------


def test_the_robot_stops_at_the_end_of_base_although_a_horizon_exists():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, S3]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((0.8, 0.3))
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert not of(cmds, SendPath) and not of(cmds, SendNavGoal)
    assert not finishes(cmds)  # the goal stays open, waiting for an update
    assert fsm.order_feedback() == (1, True, "")
    assert fsm.is_running("g1") and fsm.state() == MOVING and fsm.current_goal() == "N1"


def test_the_order_succeeds_at_the_end_of_base_when_the_horizon_is_empty():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1]), SendPath)
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert Finish("g1", True, "", 1) in cmds
    assert fsm.state() == IDLE and not fsm.is_running("g1")


def test_the_last_done_seq_id_follows_the_robot_through_a_merged_run():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, S2], [S3]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((1.3, 0.3))  # one FollowPath for both
    fsm.heartbeat(3.0)
    fsm.tick(3.0, (0.6, 0.3, 0.0), DRIVING, no_obstacle)
    assert fsm.order_feedback()[0] == 0
    fsm.heartbeat(4.0)
    fsm.tick(4.0, (0.9, 0.3, 0.0), DRIVING, no_obstacle)
    assert fsm.order_feedback()[0] == 1
    arrive(fsm, send, 8.0, (1.3, 0.3, 0.0))
    assert fsm.order_feedback() == (2, True, "")


# ---- stitching ---------------------------------------------------------------------------


def test_an_update_that_fits_supersedes_the_old_goal_and_the_robot_goes_on():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, S3]), SendPath)
    arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    cmds = order(fsm, "g2", 6.0, [S2], [S3], update=1, stitch="N1", pose=(0.8, 0.3, 0.0))
    assert Finish("g1", False, SUPERSEDED, 1) in cmds
    [nxt] = of(cmds, SendPath)
    assert nxt.poses[0][:2] == pytest.approx((0.8, 0.3))
    assert nxt.poses[-1][:2] == pytest.approx((1.3, 0.3))
    assert fsm.is_running("g2") and not fsm.is_running("g1")
    assert fsm.order_feedback() == (1, False, "")


@pytest.mark.parametrize("change, why", [
    (dict(oid="o2"), "order_id"),
    (dict(update=2), "order_update_id"),
    (dict(stitch="N9"), "stitch_node_id"),
    (dict(base=[line(3, 0.8, 1.3)]), "seq_id"),
])
def test_each_stitch_condition_refuses_only_the_new_goal(change, why):
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2]), SendPath)
    args = dict(base=[S2], horizon=[S3], update=1, stitch="N1", oid="o1") | change
    cmds = order(fsm, "g2", 2.0, args.pop("base"), **args)
    [done] = finishes(cmds)
    assert done.goal_id == "g2" and not done.success
    assert done.reason.startswith(f"{ORDER_UPDATE_REJECTED}: ") and why in done.reason
    assert kinds(cmds) == ["Finish"]  # no stop, nothing sent: the queue goes on
    assert fsm.is_running("g1") and not fsm.is_running("g2")
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert fsm.order_feedback() == (1, True, "")  # the old base still ends where it did


def test_an_update_without_a_running_order_is_refused():
    fsm = ready()
    [done] = finishes(order(fsm, "g2", 1.0, [S2], update=1, stitch="N1"))
    assert done.reason.startswith(f"{ORDER_UPDATE_REJECTED}: ") and not done.success


# ---- horizon -----------------------------------------------------------------------------


def test_the_horizon_is_replaced_as_a_whole():
    fsm = ready()
    order(fsm, "g1", 1.0, [S1], [S2, S3])
    other = line(2, 0.8, 1.3, y=0.9)
    cmds = order(fsm, "g2", 2.0, [], [other], update=1, stitch="N1")
    assert Finish("g1", False, SUPERSEDED, 0) in cmds
    assert not of(cmds, SendPath) and not of(cmds, CancelNav)  # base is unchanged
    base, horizon = fsm.order_view()
    assert [s.seq_id for s in base] == [1] and horizon == [other]


def test_a_bad_horizon_only_warns():
    def check(seg):
        return "keepout" if seg.seq_id == 3 else ""

    fsm = ready(check=check)
    cmds = order(fsm, "g1", 1.0, [S1], [S2, S3])
    assert of(cmds, SendPath) and not finishes(cmds)
    assert fsm.order_feedback() == (0, False, f"{HORIZON_INVALID}: seq 3: keepout")
    order(fsm, "g2", 2.0, [], [S2], update=1, stitch="N1")
    assert fsm.order_feedback()[2] == ""


# ---- hand-over between segments ---------------------------------------------------------


def test_same_controller_and_touching_ends_join_into_the_running_follow_path():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, S3]), SendPath)
    fsm.heartbeat(2.0)
    fsm.tick(2.0, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    cmds = order(fsm, "g2", 2.5, [S2], [S3], update=1, stitch="N1", pose=(0.55, 0.3, 0.0))
    [more] = of(cmds, SendPath)
    assert not of(cmds, CancelNav)  # FollowPath preempts with the longer path, no stop
    assert more.seq > send.seq and more.poses[-1][:2] == pytest.approx((1.3, 0.3))
    assert fsm.current_goal() == "N2"


def test_a_different_controller_stops_and_starts_the_next_segment():
    fsm = ready()
    s2 = line(2, 0.8, 1.3, ctrl=THROUGH)
    [send] = of(order(fsm, "g1", 1.0, [S1, s2]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((0.8, 0.3))  # not joined
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    [nxt] = of(cmds, SendPath)
    assert nxt.final_goal is False and nxt.poses[-1][:2] == pytest.approx((1.3, 0.3))
    assert fsm.order_feedback()[0] == 1


def test_a_mode_change_stops_before_the_nav_goal():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, goal_seg(2, 1.3, 0.6)]), SendPath)
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert of(cmds, CancelNav) and not of(cmds, SendNavGoal)
    # the FollowPath replaced near the end (R13) has no result yet: wait for it
    assert not of(fsm.tick(5.1, (0.8, 0.3, 0.0), STILL, no_obstacle), SendNavGoal)
    fsm.nav_result(send.seq, "canceled", 5.2, (0.8, 0.3, 0.0), no_obstacle)
    fsm.heartbeat(5.3)
    [goal] = of(fsm.tick(5.3, (0.8, 0.3, 0.0), STILL, no_obstacle), SendNavGoal)
    assert goal.pose == pytest.approx((1.3, 0.6, 0.0)) and fsm.order_feedback()[0] == 1


def test_a_nav_goal_follows_another_without_stopping_at_the_middle_point():
    fsm = ready()
    g1, g2 = goal_seg(1, 0.8, 0.6), goal_seg(2, 1.3, 0.6)
    [first] = of(order(fsm, "g1", 1.0, [g1, g2]), SendNavGoal)
    fsm.heartbeat(3.0)
    cmds = fsm.tick(3.0, (0.6, 0.55, 0.0), DRIVING, no_obstacle)
    assert not of(cmds, SendNavGoal)
    fsm.heartbeat(4.0)
    cmds = fsm.tick(4.0, (0.7, 0.6, 0.0), DRIVING, no_obstacle)  # 0.10 m from the middle point
    [second] = of(cmds, SendNavGoal)
    assert not of(cmds, CancelNavGoal)  # bt_navigator takes the new goal in place
    assert second.seq > first.seq and second.pose == pytest.approx((1.3, 0.6, 0.0))
    assert fsm.order_feedback()[0] == 1 and not finishes(cmds)


def test_a_retreat_segment_follows_its_path_backwards_with_the_reverse_controller():
    fsm = ready(pose=(0.8, 0.3, 0.0))
    back = line(1, 0.8, 0.5, ctrl=REVERSE, mode=MODE_RETREAT)
    cmds = order(fsm, "g1", 1.0, [back], pose=(0.8, 0.3, 0.0))
    [send] = of(cmds, SendPath)
    assert send.reverse and send.poses[-1][:2] == pytest.approx((0.5, 0.3))
    cmds = fsm.nav_result(send.seq, "succeeded", 9.0, (0.5, 0.3, 0.0), no_obstacle)
    assert Finish("g1", True, "", 1) in cmds


# ---- failures, cancel ---------------------------------------------------------------------


def test_a_failing_segment_ends_the_order_with_its_reason():
    fsm = ready()
    far = line(1, 1.0, 1.5)  # does not start at the robot
    cmds = order(fsm, "g1", 1.0, [far], [S3])
    assert finishes(cmds) == [Finish("g1", False, PATH_NOT_CONNECTED, 0)]
    assert not fsm.is_running("g1")


def test_cancelling_the_order_stops_the_robot():
    fsm = ready()
    order(fsm, "g1", 1.0, [S1], [S2])
    cmds = fsm.cancel("g1", 2.0)
    assert Finish("g1", False, CANCELED, 0) in cmds and of(cmds, CancelNav)
    assert fsm.state() == IDLE


def test_a_plain_move_to_replaces_a_running_order():
    fsm = ready()
    order(fsm, "g1", 1.0, [S1], [S2])
    cmds = fsm.move("g2", 2.0, "P", (0.8, 0.5, 0.0), [], HERE)
    assert Finish("g1", False, CANCELED, 0) in cmds and of(cmds, SendPath)
    assert len(finishes(cmds)) == 1 and fsm.is_running("g2")


# ---- comm loss ----------------------------------------------------------------------------


def test_comm_loss_drives_to_the_end_of_base_then_stops():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, line(2, 0.8, 1.3, ctrl=THROUGH)], [S3]), SendPath)
    cmds = []
    for k in range(40):  # no heartbeat after 1.0 s
        cmds += fsm.tick(1.1 + 0.1 * k, (0.4 + 0.01 * k, 0.3, 0.0), DRIVING, no_obstacle)
    assert not finishes(cmds) and not of(cmds, CancelNav)
    send = (of(cmds, SendPath) or [send])[-1]  # sent again near the end (R13)
    cmds = fsm.nav_result(send.seq, "succeeded", 6.0, (0.8, 0.3, 0.0), no_obstacle)
    [nxt] = of(cmds, SendPath)  # the second base segment still runs
    cmds = fsm.nav_result(nxt.seq, "succeeded", 9.0, (1.3, 0.3, 0.0), no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 2) in cmds
    assert fsm.state() == IDLE


def test_comm_loss_while_waiting_at_the_end_of_base_ends_the_goal():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2]), SendPath)
    arrive(fsm, send, 2.0, (0.8, 0.3, 0.0))
    cmds = fsm.tick(3.6, (0.8, 0.3, 0.0), STILL, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 1) in cmds


def test_without_an_order_comm_loss_still_stops_at_once():
    fsm = ready()
    fsm.move("g1", 1.0, "P", (1.8, 0.3, 0.0), [], HERE)
    cmds = fsm.tick(2.6, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST) in cmds and of(cmds, CancelNav)


def test_an_order_stretched_while_nav2_stands_forgets_the_path_sent_before():
    """The way ahead is then watched on the stretched line, not on the old, shorter path."""
    fsm = ready(recovery_attempts=0)  # 자가 복구 없이 물체 앞에서 기다린다
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2]), SendPath)
    fsm.nav_result(send.seq, "aborted", 2.0, (0.5, 0.3, 0.0), lambda _points: True)  # 물체 앞
    assert fsm._active.sent is not None and not fsm._active.nav_running
    cmds = order(fsm, "g2", 2.5, [S2], [S3], update=1, stitch="N1", pose=(0.5, 0.3, 0.0))
    assert not of(cmds, SendPath) and fsm._active.sent is None


def test_the_order_fsm_is_a_pinky_fsm():
    assert issubclass(OrderFsm, PinkyFsm)


# ---- Codex review 2026-11-12 ---------------------------------------------------------------


def test_starting_the_next_segment_is_not_a_heartbeat():
    """Codex P1: a segment start is not a goal from the fleet manager."""
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, line(2, 0.8, 1.3, ctrl=THROUGH)], [S3]), SendPath)
    cmds = fsm.nav_result(send.seq, "succeeded", 6.0, (0.8, 0.3, 0.0), no_obstacle)
    [nxt] = of(cmds, SendPath)
    cmds = fsm.nav_result(nxt.seq, "succeeded", 6.5, (1.3, 0.3, 0.0), no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 2) in cmds


def test_comm_loss_stops_a_nav_goal_that_makes_no_progress():
    """Codex P1: without the fleet manager the robot drives on only while it gets somewhere."""
    fsm = ready()
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, 1.3, 0.6)], [S3]), SendNavGoal)
    cmds = []
    for k in range(80):  # no heartbeat after 1.0 s, the robot stands
        cmds += fsm.tick(1.1 + 0.1 * k, HERE, STILL, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 0) in cmds and of(cmds, CancelNavGoal)


def test_a_new_order_must_start_at_seq_id_one():
    """Codex P2: 0 means "nothing done" in last_done_seq_id."""
    fsm = ready()
    [done] = finishes(order(fsm, "g1", 1.0, [line(0, 0.3, 0.8)]))
    assert (done.goal_id, done.success) == ("g1", False)


def test_a_joined_run_counts_the_segment_end_on_the_path_it_sent():
    """Codex P2: ends up to 2 cm apart are joined; the bound is where the sent path has the end
    (the next segment's first point is dropped)."""
    fsm = ready()
    s2 = Seg(2, MODE_FOLLOW_PATH, FP, ((0.8, 0.315, 0.0), (1.3, 0.315, 0.0)), None, "N2")
    s3 = Seg(3, MODE_FOLLOW_PATH, FP, ((1.3, 0.33, 0.0), (1.8, 0.33, 0.0)), None, "N3")
    order(fsm, "g1", 1.0, [S1, s2, s3])
    fsm.heartbeat(2.0)
    fsm.tick(2.0, (0.9, 0.3, 0.0), DRIVING, no_obstacle)
    fsm.heartbeat(3.0)
    fsm.tick(3.0, (1.299, 0.315, 0.0), DRIVING, no_obstacle)
    assert fsm.order_feedback()[0] == 1
    fsm.heartbeat(3.5)
    fsm.tick(3.5, (1.31, 0.33, 0.0), DRIVING, no_obstacle)
    assert fsm.order_feedback()[0] == 2


def test_a_late_map_is_used_for_the_running_horizon():
    """Codex P2: the map or mask may come after the order."""
    seen = {"map": False}

    def check(seg):
        return "outside the map" if seen["map"] and seg.seq_id == 2 else ""

    fsm = ready(check=check)
    order(fsm, "g1", 1.0, [S1], [S2])
    assert fsm.order_feedback()[2] == ""
    seen["map"] = True
    fsm.recheck_horizon()
    assert fsm.order_feedback()[2] == f"{HORIZON_INVALID}: seq 2: outside the map"
