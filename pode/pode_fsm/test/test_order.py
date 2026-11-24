"""order 모델(계약 0.11-draft)의 판단 부분(ROS 없음). 기본 구간은 따라가고 horizon 구간은 표시와 사전 검사용이다."""
import math

import pytest
from pode_fsm.core import (
    CANCELED,
    HEARTBEAT_LOST,
    IDLE,
    MOVING,
    PATH_NOT_CONNECTED,
    CancelNav,
    Finish,
    FsmParams,
    LimitSpeed,
    PodeFsm,
    SendPath,
)
from pode_fsm.nav_goal import CancelNavGoal, SendNavGoal
from pode_fsm.results import NAV_UNAVAILABLE
from pode_fsm.order_fsm import (
    HORIZON_INVALID,
    ORDER_UPDATE_REJECTED,
    SUPERSEDED,
    OrderFsm,
)
from pode_fsm.order_queue import MODE_FOLLOW_PATH, MODE_NAV_GOAL, MODE_RETREAT, OrderGoal, Seg

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
    """마지막 FollowPath가 로봇이 선 자리에서 성공으로 끝난다."""
    fsm.heartbeat(t)
    again = of(fsm.tick(t, pose, STILL, no_obstacle), SendPath)
    seq = again[-1].seq if again else send.seq
    return fsm.nav_result(seq, "succeeded", t, pose, no_obstacle)


# ---- 빈 order_id ---------------------------------------------------------------------


def test_an_empty_order_id_is_todays_move_to():
    plain, ordered = ready(), ready()
    a = plain.move("g1", 1.0, "P", (0.8, 0.3, 0.0), [], HERE)
    b = ordered.move("g1", 1.0, "P", (0.8, 0.3, 0.0), [], HERE, order=OrderGoal(""))
    assert a == b and ordered.order_feedback() == (0, False, "")


# ---- 기본 구간 끝: 불변식 ------------------------------------------------------------


def test_the_robot_stops_at_the_end_of_base_although_a_horizon_exists():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, S3]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((0.8, 0.3))
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert not of(cmds, SendPath) and not of(cmds, SendNavGoal)
    assert not finishes(cmds)  # goal은 열어 두고 갱신을 기다린다
    assert fsm.order_feedback() == (1, True, "")
    assert fsm.is_running("g1") and fsm.state() == MOVING and fsm.current_goal() == "N1"


def test_an_order_waiting_at_the_end_of_base_still_has_a_goal():
    """W-34: 잠금은 멈춘 로봇을 기다리고, 기다리는 order도 goal로 남는다."""
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, S3]), SendPath)
    arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert fsm.order_feedback()[1] is True
    assert fsm.has_goal()


def test_the_order_succeeds_at_the_end_of_base_when_the_horizon_is_empty():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1]), SendPath)
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert Finish("g1", True, "", 1) in cmds
    assert fsm.state() == IDLE and not fsm.is_running("g1")
    assert not fsm.has_goal()


def test_the_last_done_seq_id_follows_the_robot_through_a_merged_run():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, S2], [S3]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((1.3, 0.3))  # 둘을 한 FollowPath로
    fsm.heartbeat(3.0)
    fsm.tick(3.0, (0.6, 0.3, 0.0), DRIVING, no_obstacle)
    assert fsm.order_feedback()[0] == 0
    fsm.heartbeat(4.0)
    fsm.tick(4.0, (0.9, 0.3, 0.0), DRIVING, no_obstacle)
    assert fsm.order_feedback()[0] == 1
    arrive(fsm, send, 8.0, (1.3, 0.3, 0.0))
    assert fsm.order_feedback() == (2, True, "")


# ---- 이어 붙이기 -------------------------------------------------------------------


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
    assert kinds(cmds) == ["Finish"]  # 멈춤도 전송도 없이 대기열이 계속된다
    assert fsm.is_running("g1") and not fsm.is_running("g2")
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert fsm.order_feedback() == (1, True, "")  # 옛 기본 구간 끝은 그대로


def test_an_update_without_a_running_order_is_refused():
    fsm = ready()
    [done] = finishes(order(fsm, "g2", 1.0, [S2], update=1, stitch="N1"))
    assert done.reason.startswith(f"{ORDER_UPDATE_REJECTED}: ") and not done.success


# ---- horizon -----------------------------------------------------------------------


def test_the_horizon_is_replaced_as_a_whole():
    fsm = ready()
    order(fsm, "g1", 1.0, [S1], [S2, S3])
    other = line(2, 0.8, 1.3, y=0.9)
    cmds = order(fsm, "g2", 2.0, [], [other], update=1, stitch="N1")
    assert Finish("g1", False, SUPERSEDED, 0) in cmds
    assert not of(cmds, SendPath) and not of(cmds, CancelNav)  # 기본 구간은 그대로
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


# ---- 구간 넘겨받기 ------------------------------------------------------------------


def test_same_controller_and_touching_ends_join_into_the_running_follow_path():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, S3]), SendPath)
    fsm.heartbeat(2.0)
    fsm.tick(2.0, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    cmds = order(fsm, "g2", 2.5, [S2], [S3], update=1, stitch="N1", pose=(0.55, 0.3, 0.0))
    [more] = of(cmds, SendPath)
    assert not of(cmds, CancelNav)  # 더 긴 경로로 FollowPath가 선점해 멈춤이 없다
    assert more.seq > send.seq and more.poses[-1][:2] == pytest.approx((1.3, 0.3))
    assert fsm.current_goal() == "N2"


def test_a_different_controller_stops_and_starts_the_next_segment():
    fsm = ready()
    s2 = line(2, 0.8, 1.3, ctrl=THROUGH)
    [send] = of(order(fsm, "g1", 1.0, [S1, s2]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((0.8, 0.3))  # 잇지 않았다
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    [nxt] = of(cmds, SendPath)
    assert nxt.final_goal is False and nxt.poses[-1][:2] == pytest.approx((1.3, 0.3))
    assert fsm.order_feedback()[0] == 1


def test_a_mode_change_stops_before_the_nav_goal():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, goal_seg(2, 1.3, 0.6)]), SendPath)
    cmds = arrive(fsm, send, 5.0, (0.8, 0.3, 0.0))
    assert of(cmds, CancelNav) and not of(cmds, SendNavGoal)
    # 끝 근처에서 교체한 FollowPath는 아직 결과가 없으니 기다린다
    assert not of(fsm.tick(5.1, (0.8, 0.3, 0.0), STILL, no_obstacle), SendNavGoal)
    fsm.nav_result(send.seq, "canceled", 5.2, (0.8, 0.3, 0.0), no_obstacle)
    fsm.heartbeat(5.3)
    [goal] = of(fsm.tick(5.3, (0.8, 0.3, 0.0), STILL, no_obstacle), SendNavGoal)
    assert goal.pose == pytest.approx((1.3, 0.6, 0.0)) and fsm.order_feedback()[0] == 1


def test_a_fixed_line_before_a_nav_goal_ends_without_turning_to_its_end_heading():
    """다음 조각이 Nav2 goal이면 고정 선을 끝 방향 없이(free_heading) 보낸다."""
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, goal_seg(2, 1.3, 0.6)]), SendPath)
    assert send.free_heading is True
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, S2, goal_seg(3, 1.8, 0.6)]), SendPath)
    assert send.poses[-1][:2] == pytest.approx((1.3, 0.3)) and send.free_heading is True


def test_a_nav_goal_waiting_in_the_horizon_also_frees_the_end_heading():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [goal_seg(2, 1.3, 0.6)]), SendPath)
    assert send.free_heading is True


@pytest.mark.parametrize("base, horizon", [
    ([S1], []),
    ([S1], [S2]),
    ([S1, line(2, 0.8, 1.3, ctrl=THROUGH)], [goal_seg(3, 1.8, 0.6)]),
    ([S1, line(2, 0.8, 0.5, ctrl=REVERSE, mode=MODE_RETREAT)], []),
])
def test_a_fixed_line_keeps_its_end_heading_when_no_nav_goal_comes_next(base, horizon):
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, base, horizon), SendPath)
    assert send.poses[-1][:2] == pytest.approx((0.8, 0.3)) and send.free_heading is False


def test_a_line_joined_on_the_way_takes_the_end_heading_rule_of_its_new_end():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2, goal_seg(3, 1.8, 0.6)]), SendPath)
    assert send.free_heading is False
    fsm.heartbeat(2.0)
    fsm.tick(2.0, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    cmds = order(fsm, "g2", 2.5, [S2], [goal_seg(3, 1.8, 0.6)], update=1, stitch="N1",
                 pose=(0.55, 0.3, 0.0))
    [more] = of(cmds, SendPath)
    assert more.poses[-1][:2] == pytest.approx((1.3, 0.3)) and more.free_heading is True


def test_an_update_that_changes_what_comes_next_sends_the_line_again_with_the_new_rule():
    """달리는 선에 안 이어지는 갱신(기본 구간에 Nav2 goal 추가, horizon 교체)도 끝 방향 규칙이 맞게 적용된다."""
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1]), SendPath)
    assert send.free_heading is False
    fsm.heartbeat(2.0)
    fsm.tick(2.0, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    cmds = order(fsm, "g2", 2.5, [goal_seg(2, 1.3, 0.6)], update=1, stitch="N1",
                 pose=(0.55, 0.3, 0.0))
    [again] = of(cmds, SendPath)
    assert not of(cmds, CancelNav) and again.seq > send.seq
    assert again.poses[-1][:2] == pytest.approx((0.8, 0.3)) and again.free_heading is True

    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [goal_seg(2, 1.3, 0.6)]), SendPath)
    assert send.free_heading is True
    fsm.heartbeat(2.0)
    fsm.tick(2.0, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    cmds = order(fsm, "g2", 2.5, [], [S2], update=1, stitch="N1", pose=(0.55, 0.3, 0.0))
    [again] = of(cmds, SendPath)
    assert not of(cmds, CancelNav) and again.free_heading is False
    # 같은 종류의 다음 구간은 아무것도 바꾸지 않는다(두 번째 전송 없음)
    cmds = order(fsm, "g3", 3.0, [], [S2, S3], update=2, stitch="N1", pose=(0.6, 0.3, 0.0))
    assert not of(cmds, SendPath) and not of(cmds, CancelNav)


def test_a_plain_move_to_keeps_its_end_heading():
    fsm = ready()
    [send] = of(fsm.move("g1", 1.0, "N1", (0.8, 0.3, 0.0), [(0.3, 0.3), (0.8, 0.3)], HERE),
                SendPath)
    assert send.free_heading is False


def test_a_nav_goal_follows_another_without_stopping_at_the_middle_point():
    fsm = ready()
    g1, g2 = goal_seg(1, 0.8, 0.6), goal_seg(2, 1.3, 0.6)
    [first] = of(order(fsm, "g1", 1.0, [g1, g2]), SendNavGoal)
    fsm.heartbeat(3.0)
    cmds = fsm.tick(3.0, (0.6, 0.55, 0.0), DRIVING, no_obstacle)
    assert not of(cmds, SendNavGoal)
    fsm.heartbeat(4.0)
    cmds = fsm.tick(4.0, (0.7, 0.6, 0.0), DRIVING, no_obstacle)  # 가운데 점에서 0.10 m
    [second] = of(cmds, SendNavGoal)
    assert not of(cmds, CancelNavGoal)  # bt_navigator가 새 goal을 그 자리에서 받는다
    assert second.seq > first.seq and second.pose == pytest.approx((1.3, 0.6, 0.0))
    assert fsm.order_feedback()[0] == 1 and not finishes(cmds)


def test_a_nav_goal_hands_over_to_the_next_fixed_line_before_it_stops():
    """Nav2 goal 다음 조각이 고정 선이면 끝 pass_m 안에서 넘긴다(끝까지 기지 않는다)."""
    fsm = ready()
    nav, fixed = goal_seg(1, 0.8, 0.3), line(2, 0.8, 1.3)
    [first] = of(order(fsm, "g1", 1.0, [nav, fixed]), SendNavGoal)
    fsm.heartbeat(3.0)
    cmds = fsm.tick(3.0, (0.6, 0.3, 0.0), DRIVING, no_obstacle)  # 끝에서 0.20 m
    assert not of(cmds, CancelNavGoal) and fsm.order_feedback()[0] == 0
    fsm.heartbeat(4.0)
    cmds = fsm.tick(4.0, (0.7, 0.3, 0.0), DRIVING, no_obstacle)  # 0.10 m: 넘긴다
    assert of(cmds, CancelNavGoal) and not finishes(cmds)
    assert fsm.order_feedback()[0] == 1 and fsm.current_goal() == "N2"
    fsm.nav_goal_result(first.seq, "canceled", 4.2, (0.72, 0.3, 0.0))
    fsm.heartbeat(4.3)
    [send] = of(fsm.tick(4.3, (0.72, 0.3, 0.0), STILL, no_obstacle), SendPath)
    assert send.poses[0][:2] == pytest.approx((0.72, 0.3), abs=0.03)
    assert send.poses[-1][:2] == pytest.approx((1.3, 0.3))
    assert fsm.is_running("g1")


def test_a_nav_goal_keeps_going_when_the_next_fixed_line_starts_too_far_to_join():
    """고정 선에 이을 수 없는 거리(connect_tolerance_m 밖)에서는 넘기지 않는다."""
    fsm = ready(connect_tolerance_m=0.05)
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, 0.8, 0.3), line(2, 0.8, 1.3)]), SendNavGoal)
    fsm.heartbeat(4.0)
    cmds = fsm.tick(4.0, (0.7, 0.3, 0.0), DRIVING, no_obstacle)
    assert not of(cmds, CancelNavGoal) and fsm.order_feedback()[0] == 0


def test_a_nav_goal_keeps_going_when_the_next_line_itself_is_out_of_reach():
    """다음 선의 시작이 떨어져 있으면(0.156 m) 미리 넘기지 않는다(넘기면 PATH_NOT_CONNECTED)."""
    fsm = ready()
    off_line = line(2, 0.8, 1.3, y=0.42)
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, 0.8, 0.3), off_line]), SendNavGoal)
    fsm.heartbeat(4.0)
    cmds = fsm.tick(4.0, (0.7, 0.3, 0.0), DRIVING, no_obstacle)
    assert not of(cmds, CancelNavGoal) and not finishes(cmds)
    assert fsm.order_feedback()[0] == 0 and fsm.is_running("g1")


def test_a_robot_that_slid_off_the_line_while_stopping_ends_the_order():
    """넘긴 뒤 선에서 connect_tolerance_m 밖으로 밀리면 서고 다시 재어 PATH_NOT_CONNECTED로 끝낸다."""
    fsm = ready()
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, 0.8, 0.3), line(2, 0.8, 1.3)]),
                 SendNavGoal)
    fsm.heartbeat(4.0)
    assert of(fsm.tick(4.0, (0.7, 0.3, 0.0), DRIVING, no_obstacle), CancelNavGoal)
    slid = (0.75, 0.5, 0.0)
    fsm.nav_goal_result(first.seq, "canceled", 4.2, slid)
    fsm.heartbeat(4.3)
    cmds = fsm.tick(4.3, slid, STILL, no_obstacle)
    assert not of(cmds, SendPath)
    assert finishes(cmds) == [Finish("g1", False, PATH_NOT_CONNECTED, 1)]
    assert not fsm.is_running("g1")


def test_an_order_before_nav2_is_ready_ends_as_unavailable():
    fsm = OrderFsm(FsmParams())
    fsm.set_nav_ready(False)
    fsm.tick(0.0, HERE, STILL, no_obstacle)
    cmds = order(fsm, "g1", 0.1, [S1, S2])
    assert finishes(cmds) == [Finish("g1", False, NAV_UNAVAILABLE)]
    assert not of(cmds, SendPath) and not fsm.is_running("g1")


def test_a_nav_goal_the_server_was_not_there_for_ends_the_order_as_unavailable():
    fsm = ready()
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, 0.8, 0.3)]), SendNavGoal)
    cmds = fsm.nav_goal_result(first.seq, "unavailable", 1.2, HERE)
    assert [(f.goal_id, f.success, f.reason) for f in finishes(cmds)] == [("g1", False, NAV_UNAVAILABLE)]


def test_pass_distance_leaves_room_below_the_join_tolerance():
    """먼저 넘기는 거리는 이음 허용치보다 작다(넘긴 뒤 서는 동안 위치가 고쳐져도 이어 붙게)."""
    from pode_fsm.order_fsm import OrderParams
    assert OrderParams().pass_m == pytest.approx(0.12)
    assert FsmParams().connect_tolerance_m - OrderParams().pass_m >= 0.03


def test_a_pose_correction_while_stopping_after_the_pass_still_joins_the_line():
    """Gazebo 재현: Q2 nav_goal 다음이 Q2에서 시작하는 고정 선. 추정 위치 0.144 m에서 넘기고 선 뒤
    AMCL이 2.2 cm 뒤로 고치면(0.163 m) 이음 허용치 0.15를 넘어 PATH_NOT_CONNECTED가 났다."""
    q2, q1 = (1.66, 0.15), (1.849, 0.26)
    yaw = math.atan2(q1[1] - q2[1], q1[0] - q2[0])
    nxt = Seg(2, MODE_FOLLOW_PATH, FP, ((*q2, yaw), (*q1, yaw)), None, "Q1")
    fsm = ready(pose=(1.40, 0.17, 0.3))
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, *q2, node="Q2"), nxt],
                       pose=(1.40, 0.17, 0.3)), SendNavGoal)
    fsm.heartbeat(3.0)
    # 기록의 넘긴 자리(Q2에서 0.144 m): 아직 넘기지 않는다
    cmds = fsm.tick(3.0, (1.5213, 0.1869, 0.31), DRIVING, no_obstacle)
    assert math.dist((1.5213, 0.1869), q2) == pytest.approx(0.144, abs=0.001)
    assert not of(cmds, CancelNavGoal) and fsm.order_feedback()[0] == 0
    # 0.12 m 안에 들면 넘긴다
    fsm.heartbeat(3.5)
    cmds = fsm.tick(3.5, (1.545, 0.18, 0.3), DRIVING, no_obstacle)
    assert of(cmds, CancelNavGoal) and fsm.order_feedback()[0] == 1
    # 서는 동안 추정 위치가 2.2 cm 뒤로 고쳐져도 이어 붙는다
    back = (1.523, 0.18, 0.24)
    fsm.nav_goal_result(first.seq, "canceled", 3.7, back)
    fsm.heartbeat(3.8)
    cmds = fsm.tick(3.8, back, STILL, no_obstacle)
    assert not finishes(cmds)
    [send] = of(cmds, SendPath)
    assert send.poses[-1][:2] == pytest.approx(q1)
    assert fsm.is_running("g1")


def test_the_old_pass_distance_broke_the_same_hand_over():
    """넘김 거리를 이음 허용치와 같은 0.15로 두면 0.144 m에서 넘기고, 0.163 m로 고쳐진 뒤 PATH_NOT_CONNECTED."""
    from pode_fsm.order_fsm import OrderParams
    q2, q1 = (1.66, 0.15), (1.849, 0.26)
    yaw = math.atan2(q1[1] - q2[1], q1[0] - q2[0])
    nxt = Seg(2, MODE_FOLLOW_PATH, FP, ((*q2, yaw), (*q1, yaw)), None, "Q1")
    fsm = OrderFsm(FsmParams(), order_params=OrderParams(pass_m=0.15))
    fsm.tick(0.0, (1.40, 0.17, 0.3), STILL, no_obstacle)
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, *q2, node="Q2"), nxt],
                       pose=(1.40, 0.17, 0.3)), SendNavGoal)
    fsm.heartbeat(3.0)
    assert of(fsm.tick(3.0, (1.5213, 0.1869, 0.31), DRIVING, no_obstacle), CancelNavGoal)
    back = (1.4990, 0.1800, 0.24)
    fsm.nav_goal_result(first.seq, "canceled", 3.2, back)
    fsm.heartbeat(3.3)
    cmds = fsm.tick(3.3, back, STILL, no_obstacle)
    assert finishes(cmds) == [Finish("g1", False, PATH_NOT_CONNECTED, 1)]


def test_a_retreat_segment_follows_its_path_backwards_with_the_reverse_controller():
    fsm = ready(pose=(0.8, 0.3, 0.0))
    back = line(1, 0.8, 0.5, ctrl=REVERSE, mode=MODE_RETREAT)
    cmds = order(fsm, "g1", 1.0, [back], pose=(0.8, 0.3, 0.0))
    [send] = of(cmds, SendPath)
    assert send.reverse and send.poses[-1][:2] == pytest.approx((0.5, 0.3))
    cmds = fsm.nav_result(send.seq, "succeeded", 9.0, (0.5, 0.3, 0.0), no_obstacle)
    assert Finish("g1", True, "", 1) in cmds


# ---- 실패, 취소 ---------------------------------------------------------------------


def test_a_failing_segment_ends_the_order_with_its_reason():
    fsm = ready()
    far = line(1, 1.0, 1.5)  # 로봇에서 시작하지 않는다
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


# ---- 통신 끊김 ----------------------------------------------------------------------


def test_comm_loss_drives_to_the_end_of_base_then_stops():
    fsm = ready(comm_loss_policy="base_end")
    [send] = of(order(fsm, "g1", 1.0, [S1, line(2, 0.8, 1.3, ctrl=THROUGH)], [S3]), SendPath)
    cmds = []
    for k in range(40):  # 1.0 s 뒤 heartbeat 없음
        cmds += fsm.tick(1.1 + 0.1 * k, (0.4 + 0.01 * k, 0.3, 0.0), DRIVING, no_obstacle)
    assert not finishes(cmds) and not of(cmds, CancelNav)
    send = (of(cmds, SendPath) or [send])[-1]  # 끝 근처에서 다시 보냄
    cmds = fsm.nav_result(send.seq, "succeeded", 6.0, (0.8, 0.3, 0.0), no_obstacle)
    [nxt] = of(cmds, SendPath)  # 두 번째 기본 구간이 계속 달린다
    cmds = fsm.nav_result(nxt.seq, "succeeded", 9.0, (1.3, 0.3, 0.0), no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 2) in cmds
    assert fsm.state() == IDLE


def test_comm_loss_while_waiting_at_the_end_of_base_ends_the_goal():
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2]), SendPath)
    arrive(fsm, send, 2.0, (0.8, 0.3, 0.0))
    cmds = fsm.tick(3.6, (0.8, 0.3, 0.0), STILL, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 1) in cmds


def test_comm_loss_stops_the_order_at_once_by_default():
    """comm_loss_policy stop: 주문 없는 goal처럼 끝나고 last_done_seq_id는 끝낸 마지막 구간이다."""
    fsm = ready()
    [send] = of(order(fsm, "g1", 1.0, [S1, line(2, 0.8, 1.3, ctrl=THROUGH)], [S3]), SendPath)
    cmds = fsm.nav_result(send.seq, "succeeded", 1.2, (0.8, 0.3, 0.0), no_obstacle)
    assert not finishes(cmds)  # 첫 구간이 끝나고 둘째가 시작한다
    cmds = fsm.tick(2.6, (0.9, 0.3, 0.0), DRIVING, no_obstacle)  # heartbeat 없이 1.5 s
    assert Finish("g1", False, HEARTBEAT_LOST, 1) in cmds and of(cmds, CancelNav)
    assert fsm.state() == IDLE


def test_comm_loss_during_a_retreat_segment_ends_the_order_with_heartbeat_lost():
    fsm = ready(pose=(0.8, 0.3, 0.0))
    back = line(1, 0.8, 0.5, ctrl=REVERSE, mode=MODE_RETREAT)
    order(fsm, "g1", 1.0, [back], [S3], pose=(0.8, 0.3, 0.0))
    cmds = fsm.tick(2.6, (0.7, 0.3, 0.0), DRIVING, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 0) in cmds


def test_comm_loss_counts_the_segment_the_robot_has_just_finished():
    fsm = ready()
    order(fsm, "g1", 1.0, [S1, S2], [S3])  # 합친 FollowPath 하나
    cmds = fsm.tick(2.6, (0.9, 0.3, 0.0), DRIVING, no_obstacle)  # S1 끝을 지났다
    assert Finish("g1", False, HEARTBEAT_LOST, 1) in cmds


def test_comm_loss_stops_a_waiting_order_under_both_policies():
    for policy in ("stop", "base_end"):
        fsm = ready(comm_loss_policy=policy)
        [send] = of(order(fsm, "g1", 1.0, [S1], [S2]), SendPath)
        arrive(fsm, send, 2.0, (0.8, 0.3, 0.0))
        cmds = fsm.tick(3.6, (0.8, 0.3, 0.0), STILL, no_obstacle)
        assert Finish("g1", False, HEARTBEAT_LOST, 1) in cmds


@pytest.mark.parametrize("policy", ["", "Stop", "base-end", "go_on"])
def test_an_unknown_comm_loss_policy_is_refused_at_start_up(policy):
    with pytest.raises(ValueError, match="comm_loss_policy"):
        FsmParams(comm_loss_policy=policy)


def test_without_an_order_comm_loss_still_stops_at_once():
    fsm = ready()
    fsm.move("g1", 1.0, "P", (1.8, 0.3, 0.0), [], HERE)
    cmds = fsm.tick(2.6, (0.5, 0.3, 0.0), DRIVING, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST) in cmds and of(cmds, CancelNav)


def test_an_order_stretched_while_nav2_stands_forgets_the_path_sent_before():
    """앞길은 늘어난 선에서 감시하고 옛 짧은 경로에서 보지 않는다."""
    fsm = ready(recovery_attempts=0)  # 자가 복구 없이 물체 앞에서 기다린다
    [send] = of(order(fsm, "g1", 1.0, [S1], [S2]), SendPath)
    fsm.nav_result(send.seq, "aborted", 2.0, (0.5, 0.3, 0.0), lambda _points: True)  # 물체 앞
    assert fsm._active.sent is not None and not fsm._active.nav_running
    cmds = order(fsm, "g2", 2.5, [S2], [S3], update=1, stitch="N1", pose=(0.5, 0.3, 0.0))
    assert not of(cmds, SendPath) and fsm._active.sent is None
    seen = []
    fsm.heartbeat(3.0)
    fsm.tick(3.0, (0.5, 0.3, 0.0), STILL, lambda points: seen.extend(points) or True)
    assert max(x for x, _ in seen) > 0.9, "the way ahead goes on past the old end at 0.8"


def test_comm_loss_of_a_plain_goal_does_not_rename_the_next_orders_failure():
    fsm = ready()
    fsm.move("g1", 1.0, "P", (1.8, 0.3, 0.0), [], HERE)
    assert Finish("g1", False, HEARTBEAT_LOST) in fsm.tick(2.6, HERE, STILL, no_obstacle)
    far = line(1, 1.0, 1.5)  # 로봇에서 시작하지 않는다
    cmds = order(fsm, "g2", 2.7, [far], [S3])
    assert finishes(cmds) == [Finish("g2", False, PATH_NOT_CONNECTED, 0)]


def test_a_tick_follows_a_pose_far_ahead_by_one_look_ahead_only():
    """한 tick에 두 번 추적하지 않는다."""
    fsm = ready(track_ahead_m=0.3)
    order(fsm, "g1", 1.0, [line(1, 0.3, 2.3), line(2, 2.3, 2.8)], [S3])
    fsm.tick(1.1, (2.0, 0.3, 0.0), DRIVING, no_obstacle)
    assert fsm._active.s == pytest.approx(0.3)


def test_the_order_fsm_is_a_pode_fsm():
    assert issubclass(OrderFsm, PodeFsm)


# ---- order 경계 경우 -----------------------------------------------------------------


def test_starting_the_next_segment_is_not_a_heartbeat():
    """구간 시작은 관제의 goal이 아니다."""
    fsm = ready(comm_loss_policy="base_end")
    [send] = of(order(fsm, "g1", 1.0, [S1, line(2, 0.8, 1.3, ctrl=THROUGH)], [S3]), SendPath)
    cmds = fsm.nav_result(send.seq, "succeeded", 6.0, (0.8, 0.3, 0.0), no_obstacle)
    [nxt] = of(cmds, SendPath)
    cmds = fsm.nav_result(nxt.seq, "succeeded", 6.5, (1.3, 0.3, 0.0), no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 2) in cmds


def test_comm_loss_stops_a_nav_goal_that_makes_no_progress():
    """관제가 없으면 로봇은 어딘가에 가는 동안만 계속 간다."""
    fsm = ready()
    [first] = of(order(fsm, "g1", 1.0, [goal_seg(1, 1.3, 0.6)], [S3]), SendNavGoal)
    cmds = []
    for k in range(80):  # 1.0 s 뒤 heartbeat 없고 로봇이 선다
        cmds += fsm.tick(1.1 + 0.1 * k, HERE, STILL, no_obstacle)
    assert Finish("g1", False, HEARTBEAT_LOST, 0) in cmds and of(cmds, CancelNavGoal)


def test_a_new_order_must_start_at_seq_id_one():
    """last_done_seq_id 0은 끝낸 것이 없다는 뜻이다."""
    fsm = ready()
    [done] = finishes(order(fsm, "g1", 1.0, [line(0, 0.3, 0.8)]))
    assert (done.goal_id, done.success) == ("g1", False)


def test_a_joined_run_counts_the_segment_end_on_the_path_it_sent():
    """끝이 2 cm 안이면 이어 붙이고, 보낸 경로의 끝을 기준으로 본다(다음 구간 첫 점은 뺀다)."""
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
    """지도나 마스크가 order보다 늦게 와도 된다."""
    seen = {"map": False}

    def check(seg):
        return "outside the map" if seen["map"] and seg.seq_id == 2 else ""

    fsm = ready(check=check)
    order(fsm, "g1", 1.0, [S1], [S2])
    assert fsm.order_feedback()[2] == ""
    seen["map"] = True
    fsm.recheck_horizon()
    assert fsm.order_feedback()[2] == f"{HORIZON_INVALID}: seq 2: outside the map"
