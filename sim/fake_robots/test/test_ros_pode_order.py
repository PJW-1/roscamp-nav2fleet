"""가짜 Pode의 order goal(base, horizon, update 이어 붙이기)을 실제 액션으로 시험한다."""
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces.msg")

from conftest import wait_until  # noqa: E402
from fleet_interfaces.action import MoveTo  # noqa: E402
from fleet_interfaces.msg import Segment  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from test_ros_pode import SITE, result_of, start  # noqa: E402

X0, Y0 = SITE.node_pose("P-1")
STEP_M = 0.3
NODES = {1: "a", 2: "b", 3: "c"}  # seq_id -> 끝 노드
SLOW = {"pode1": {"start": "P-1", "speed_mps": 0.1, "pose_noise_m": 0.0, "stop_error_m": 0.0}}
FAST = {"pode1": {"start": "P-1", "speed_mps": 1.0, "pose_noise_m": 0.0, "stop_error_m": 0.0}}


def end_of(seq):
    return (X0 + STEP_M * seq, Y0)


def _stamped(x, y):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.w = 1.0
    return p


def seg(seq, mode=Segment.MODE_FOLLOW_PATH):
    """x 방향 곧은 길의 seq번째 조각(0.3 m). 조각 1은 P-1에서 시작한다."""
    msg = Segment(seq_id=seq, mode=mode, controller_id="FollowPath", end_node_id=NODES[seq])
    msg.path.header.frame_id = "map"
    msg.path.poses = [_stamped(*end_of(seq - 1)), _stamped(*end_of(seq))]
    return msg


def order(p, base, horizon=(), update=0, stitch="", order_id="pode1-1"):
    goal = MoveTo.Goal(order_id=order_id, order_update_id=update, stitch_node_id=stitch,
                       base=[seg(s) for s in base], horizon=[seg(s) for s in horizon])
    if base:  # 관제처럼 옛 칸에 base 끝을 넣는다
        goal.target_id = NODES[base[-1]]
        goal.target_pose.x, goal.target_pose.y = end_of(base[-1])
    feedback = []
    fut = p.move.send_goal_async(goal, feedback_callback=lambda f: feedback.append(f.feedback))
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    return fut.result(), feedback


def here(p):
    s = p.states[-1][1]
    return (s.pose.x, s.pose.y)


def test_base_segments_run_in_order_with_feedback(harness, tmp_path):
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    handle, feedback = order(p, [1, 2])
    res = result_of(handle, 20.0).result
    assert (res.success, res.reason, res.last_done_seq_id) == (True, "", 2)
    done = [f.last_done_seq_id for f in feedback]
    assert done == sorted(done) and 1 in done
    assert not any(f.waiting_at_base_end for f in feedback)
    assert wait_until(lambda: here(p) == pytest.approx(end_of(2), abs=1e-3), 2.0)


def test_waits_at_the_base_end_then_an_update_is_stitched(harness, tmp_path):
    p = start(harness, tmp_path, robots=FAST)
    assert wait_until(lambda: p.states, 5.0)
    first, feedback = order(p, [1], horizon=[2])
    assert wait_until(lambda: any(f.waiting_at_base_end and f.last_done_seq_id == 1
                                  for f in feedback), 10.0)
    assert here(p) == pytest.approx(end_of(1), abs=1e-3)
    second, _ = order(p, [2], update=1, stitch="a")
    old = result_of(first).result
    assert (old.success, old.reason, old.last_done_seq_id) == (
        False, MoveTo.Result.REASON_SUPERSEDED, 1)
    new = result_of(second).result
    assert (new.success, new.last_done_seq_id) == (True, 2)
    assert wait_until(lambda: here(p) == pytest.approx(end_of(2), abs=1e-3), 2.0)


def test_an_update_while_driving_supersedes_the_old_goal_and_keeps_going(harness, tmp_path):
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    first, _ = order(p, [1], horizon=[2, 3])
    second, feedback = order(p, [2], horizon=[3], update=1, stitch="a")
    old = result_of(first).result
    assert (old.success, old.reason, old.last_done_seq_id) == (
        False, MoveTo.Result.REASON_SUPERSEDED, 0)
    third, _ = order(p, [3], update=2, stitch="b")
    assert result_of(second).result.reason == MoveTo.Result.REASON_SUPERSEDED
    new = result_of(third, 30.0).result
    assert (new.success, new.last_done_seq_id) == (True, 3)
    assert wait_until(lambda: here(p) == pytest.approx(end_of(3), abs=1e-3), 2.0)


def test_a_rejected_update_ends_only_the_new_goal(harness, tmp_path):
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    first, feedback = order(p, [1], horizon=[2])
    bad, _ = order(p, [2], update=1, stitch="zzz")
    res = result_of(bad).result
    assert res.success is False
    assert res.reason.startswith(MoveTo.Result.REASON_ORDER_UPDATE_REJECTED)
    wrong_id, _ = order(p, [2], update=1, stitch="a", order_id="other")
    assert result_of(wrong_id).result.reason.startswith(
        MoveTo.Result.REASON_ORDER_UPDATE_REJECTED)
    time.sleep(0.5)
    assert not first.get_result_async().done()  # 이전 order는 그대로 달린다
    good, _ = order(p, [2], update=1, stitch="a")
    assert result_of(first, 20.0).result.reason == MoveTo.Result.REASON_SUPERSEDED
    assert result_of(good, 20.0).result.success is True


def test_an_update_without_a_running_order_is_rejected(harness, tmp_path):
    p = start(harness, tmp_path, robots=FAST)
    assert wait_until(lambda: p.states, 5.0)
    handle, _ = order(p, [2], update=1, stitch="a")
    res = result_of(handle).result
    assert res.success is False
    assert res.reason.startswith(MoveTo.Result.REASON_ORDER_UPDATE_REJECTED)


def test_cancel_ends_the_order(harness, tmp_path):
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    handle, _ = order(p, [1, 2])
    time.sleep(0.3)
    cancel = handle.cancel_goal_async()
    assert wait_until(cancel.done, 5.0)
    res = result_of(handle).result
    assert (res.success, res.reason) == (False, MoveTo.Result.REASON_CANCELED)
    stop = here(p)
    time.sleep(0.5)
    assert here(p) == pytest.approx(stop, abs=1e-3)


def test_a_new_order_replaces_a_running_one_as_canceled(harness, tmp_path):
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    first, _ = order(p, [1, 2])
    second, _ = order(p, [1], order_id="pode1-2")
    assert result_of(first).result.reason == MoveTo.Result.REASON_CANCELED
    assert result_of(second, 20.0).result.success is True


def test_a_retreat_segment_backs_along_its_path(harness, tmp_path):
    p = start(harness, tmp_path, robots=FAST)
    assert wait_until(lambda: p.states, 5.0)
    back = Segment(seq_id=1, mode=Segment.MODE_RETREAT, controller_id="FollowPathReverse",
                   end_node_id="r")
    back.path.poses = [_stamped(X0, Y0), _stamped(X0 - 0.08, Y0)]
    goal = MoveTo.Goal(order_id="pode1-1", base=[back], target_id="r")
    fut = p.move.send_goal_async(goal)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    res = result_of(fut.result(), 20.0).result
    assert (res.success, res.last_done_seq_id) == (True, 1)
    assert wait_until(lambda: here(p) == pytest.approx((X0 - 0.08, Y0), abs=1e-3), 2.0)


def test_a_move_without_an_order_is_unchanged(harness, tmp_path):
    p = start(harness, tmp_path, robots=FAST)
    assert wait_until(lambda: p.states, 5.0)
    handle = p.goal("O_Q")
    res = result_of(handle).result
    assert (res.success, res.reason, res.last_done_seq_id) == (True, "", 0)
    assert all(f.feedback.last_done_seq_id == 0 and not f.feedback.waiting_at_base_end
               for f in p.feedback)


def send(p, goal):
    fut = p.move.send_goal_async(goal)
    assert wait_until(fut.done, 5.0) and fut.result().accepted
    return fut.result()


def test_a_new_order_with_bad_segments_is_nav_aborted(harness, tmp_path):
    """실물 order_fsm처럼 첫 seq가 1이 아니거나, seq가 건너뛰거나, mode를 모르면 받지 않는다."""
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    odd = seg(2)
    odd.mode = "fly"
    for base in ([seg(2)], [seg(1), seg(3)], [seg(1), odd]):
        res = result_of(send(p, MoveTo.Goal(order_id="pode1-1", base=base))).result
        assert (res.success, res.reason) == (False, MoveTo.Result.REASON_NAV_ABORTED)
    assert here(p) == pytest.approx((X0, Y0), abs=1e-3)


def test_an_update_with_a_seq_gap_inside_its_base_is_rejected(harness, tmp_path):
    p = start(harness, tmp_path, robots=SLOW)
    assert wait_until(lambda: p.states, 5.0)
    first, _ = order(p, [1], horizon=[2, 3])
    bad = send(p, MoveTo.Goal(order_id="pode1-1", order_update_id=1, stitch_node_id="a",
                              base=[seg(2), seg(2)]))
    res = result_of(bad).result
    assert res.reason.startswith(MoveTo.Result.REASON_ORDER_UPDATE_REJECTED)
    time.sleep(0.3)
    assert not first.get_result_async().done()


def test_a_stitched_path_that_starts_away_from_the_robot_is_not_followed(harness, tmp_path):
    """앞 조각 끝에 잇지 않은 path는 새로 시작하는 이동처럼 연결 검사를 받는다."""
    p = start(harness, tmp_path, robots=FAST)
    assert wait_until(lambda: p.states, 5.0)
    first, feedback = order(p, [1], horizon=[2])
    assert wait_until(lambda: any(f.waiting_at_base_end for f in feedback), 10.0)
    away = seg(2)
    away.path.poses = [_stamped(X0 + 0.3, Y0 + 0.4), _stamped(X0 + 0.6, Y0 + 0.4)]
    second = send(p, MoveTo.Goal(order_id="pode1-1", order_update_id=1, stitch_node_id="a",
                                 base=[away]))
    res = result_of(second).result
    assert (res.success, res.reason, res.last_done_seq_id) == (
        False, MoveTo.Result.REASON_PATH_NOT_CONNECTED, 1)
    assert here(p) == pytest.approx(end_of(1), abs=1e-3)


def test_an_obstacle_that_shows_up_while_waiting_holds_the_next_segment_from_its_start(
        harness, tmp_path):
    """base 끝에 서 있는 동안 나타난 앞 물체는 goal이 없던 로봇과 같이 다음에 달릴 때 그 자리에서
    막는다(blocked_lane 시나리오). 서 있을 때 물체 자리를 잡으면 다음 조각을 그만큼 달려 버린다."""
    from fleet_interfaces.msg import RobotState
    p = start(harness, tmp_path, robots=FAST, node={"recovery_attempts": 0}, faults=[
        {"at_s": 1.5, "robot": "pode1", "type": "blocked", "duration_s": 60.0}])
    assert wait_until(lambda: p.states, 5.0)
    first, feedback = order(p, [1], horizon=[2])
    assert wait_until(lambda: any(f.waiting_at_base_end for f in feedback), 10.0)
    time.sleep(2.0)  # 물체가 나타난 뒤
    second, _ = order(p, [2], update=1, stitch="a")
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 8.0)
    assert here(p) == pytest.approx(end_of(1), abs=1e-3)
    assert not second.get_result_async().done()


def test_an_obstacle_held_at_the_end_of_one_segment_holds_the_next_from_its_start(
        harness, tmp_path):
    """물체 앞에서 이미 끝에 있던 조각이 바로 끝나도, 그 물체 자리는 다음 조각의 시작이다
    (예전 goal 끝의 recovery.restart와 같이). 지난 조각의 거리를 다음 조각에 쓰면 그만큼 간다."""
    from fleet_interfaces.msg import RobotState
    p = start(harness, tmp_path, robots=FAST, node={"recovery_attempts": 0}, faults=[
        {"at_s": 0.0, "robot": "pode1", "type": "blocked", "duration_s": 60.0}])
    assert wait_until(lambda: p.states, 5.0)
    here_seg = seg(1)
    here_seg.path.poses = [_stamped(X0 - 0.01, Y0), _stamped(X0, Y0)]  # 이미 끝에 있다
    nxt = seg(2)
    nxt.path.poses = [_stamped(X0, Y0), _stamped(X0 + 0.3, Y0)]
    handle = send(p, MoveTo.Goal(order_id="pode1-1", base=[here_seg, nxt]))
    assert wait_until(lambda: p.states[-1][1].state == RobotState.STATE_BLOCKED, 8.0)
    assert here(p) == pytest.approx((X0, Y0), abs=1e-3)
    assert not handle.get_result_async().done()


def test_after_waiting_at_the_base_end_the_next_path_is_checked_again(harness, tmp_path):
    """실물은 base 끝에서 기다린 뒤 받은 조각을 새 줄로 시작해 연결을 다시 잰다.
    기다리는 동안 보고 자세가 옆으로 밀리면(drift) 앞 조각에 이어진 path라도 따르지 않는다."""
    p = start(harness, tmp_path, robots=FAST, faults=[
        {"at_s": 2.0, "robot": "pode1", "type": "drift", "duration_s": 100.0, "offset_m": 0.5}])
    assert wait_until(lambda: p.states, 5.0)
    first, feedback = order(p, [1], horizon=[2])
    assert wait_until(lambda: any(f.waiting_at_base_end for f in feedback), 10.0)
    time.sleep(3.0)  # drift가 시작된 뒤
    second, _ = order(p, [2], update=1, stitch="a")
    assert result_of(first).result.reason == MoveTo.Result.REASON_SUPERSEDED
    res = result_of(second).result
    assert (res.success, res.reason, res.last_done_seq_id) == (
        False, MoveTo.Result.REASON_PATH_NOT_CONNECTED, 1)
