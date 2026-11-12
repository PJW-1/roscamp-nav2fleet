"""로봇단 교착 처리(추가 지시서 3의 7-4, 기준(사용자))가 core에서 도는 순서. ROS 없이 돈다.

고정 경로(follow_path): 물체 앞에 block_report_s 서면 RECOVERING(local_yield wait)으로
local_wait_s 기다린다. 앞을 막은 것이 관제가 아는 다른 로봇이고 내가 낮은 쪽이면 지나온 길 안에서
local_backup_m까지 물러나고(backup) 지도를 비운 뒤 다시 간다(retry). 그래도 막히면 BLOCKED.
선 때부터 blocked_after_s 안에 BLOCKED가 된다.
"""
import math

import pytest
from pinky_fsm.backup import BackUp, CancelBackUp
from pinky_fsm.core import (
    BLOCKED,
    MOVING,
    RECOVERING,
    CancelNav,
    ClearCostmap,
    FsmParams,
    PinkyFsm,
    SendPath,
)
from pinky_fsm.local_yield import BACKUP, RETRY, WAIT, Peer
from pinky_fsm.order_fsm import OrderFsm
from pinky_fsm.order_queue import MODE_FOLLOW_PATH, OrderGoal, Seg

P3, P4, P5 = (0.35, 1.55), (0.35, 1.95), (0.80, 2.05)
NORTH = math.pi / 2
STILL = (0.0, 0.0)
DRIVING = (0.2, 0.0)
# P3(경로 시작)에서 0.38 m. 지난 꼭짓점에서 0.38 - 0.23 = 0.15 m까지 물러날 수 있다(자가 복구 규칙)
STUCK = (0.35, 1.93, NORTH)
PEER_AHEAD = (0.40, 2.00)  # 가던 길 앞 0.5 m 안


def no_obstacle(_points):
    return False


def obstacle(_points):
    return True


def ready(name="pinky1", t=0.0, pose=(*P3, NORTH), **params):
    fsm = PinkyFsm(FsmParams(**params), robot_id=name)
    fsm.tick(t, pose, STILL, no_obstacle)
    return fsm


def move(fsm, t, priority=-1, pose=(*P3, NORTH)):
    yaw = math.atan2(P5[1] - P4[1], P5[0] - P4[0])
    return fsm.move("g1", t, "P-5", (*P5, yaw), [P3, P4, P5], pose, priority=priority)


def tick(fsm, t, pose, seen=obstacle, speed=STILL):
    fsm.heartbeat(t)
    return fsm.tick(t, pose, speed, seen)


def stand(fsm, t0, pose, seen=obstacle):
    """t0부터 1초 넘게 서 있는다. 마지막 tick의 명령을 돌려준다."""
    tick(fsm, t0, pose, seen)
    return tick(fsm, t0 + 1.05, pose, seen)


def kinds(cmds):
    return [type(c).__name__ for c in cmds]


def backups(cmds):
    return [c for c in cmds if isinstance(c, BackUp)]


def test_standing_at_an_obstacle_first_waits_as_recovering_without_backing_up():
    fsm = ready()
    move(fsm, 0.0)
    cmds = stand(fsm, 1.0, STUCK)
    assert cmds == [CancelNav()]  # 선 채로 기다린다. 다른 Nav2 움직임은 없다
    assert fsm.state() == RECOVERING and fsm.local_yield_step() == WAIT
    assert tick(fsm, 3.9, STUCK) == []
    assert fsm.state() == RECOVERING


def test_an_object_that_stays_is_reported_blocked_after_the_wait_with_no_backup():
    fsm = ready()
    move(fsm, 0.0)
    stand(fsm, 1.0, STUCK)
    cmds = tick(fsm, 4.1, STUCK)  # 2.05 + 2 s
    assert backups(cmds) == [] and ClearCostmap() not in cmds
    assert fsm.state() == BLOCKED and fsm.local_yield_step() == ""


def test_an_obstacle_that_leaves_during_the_wait_is_followed_again_without_backing_up():
    fsm = ready()
    move(fsm, 0.0)
    stand(fsm, 1.0, STUCK)
    assert tick(fsm, 2.5, STUCK, no_obstacle) == []
    cmds = tick(fsm, 3.6, STUCK, no_obstacle)
    assert kinds(cmds) == ["SendPath"]
    assert fsm.state() == MOVING and fsm.local_yield_step() == ""


def test_the_lower_robot_ahead_backs_up_along_the_driven_line_then_follows_again():
    fsm = ready()
    move(fsm, 0.0, priority=0)
    fsm.see_peers(1.5, [Peer("pinky2", *PEER_AHEAD, 2)])
    stand(fsm, 1.0, STUCK)
    cmds = tick(fsm, 4.1, STUCK)
    [b] = backups(cmds)
    assert b.distance_m == pytest.approx(0.10)  # local_backup_m
    assert fsm.state() == RECOVERING and fsm.local_yield_step() == BACKUP
    behind = (0.35, 1.83, NORTH)
    cmds = fsm.backup_result(b.seq, "succeeded", 6.0, behind)
    assert kinds(cmds) == ["ClearCostmap", "SendPath"]
    assert fsm.local_yield_step() == RETRY
    stand(fsm, 6.5, behind)  # 다시 막혔다
    assert fsm.state() == BLOCKED


def test_the_higher_robot_ahead_does_not_back_up():
    fsm = ready()
    move(fsm, 0.0, priority=2)
    fsm.see_peers(1.5, [Peer("pinky2", *PEER_AHEAD, 0)])
    stand(fsm, 1.0, STUCK)
    assert backups(tick(fsm, 4.1, STUCK)) == []
    assert fsm.state() == BLOCKED


@pytest.mark.parametrize("mine_in_row, backs", [(0, True), (3, False)])
def test_my_own_row_in_the_same_message_decides_over_my_goal_priority(mine_in_row, backs):
    # 두 로봇이 같은 /fleet/robots 한 장의 두 값을 비교하게 한다(priority가 바뀌는 사이에 둘 다
    # 물러나지 않게). 그 줄이 없을 때만 MoveTo.priority를 쓴다
    fsm = ready()
    move(fsm, 0.0, priority=2)
    fsm.see_peers(1.5, [Peer("pinky1", *STUCK[:2], mine_in_row), Peer("pinky2", *PEER_AHEAD, 1)])
    stand(fsm, 1.0, STUCK)
    assert bool(backups(tick(fsm, 4.1, STUCK))) is backs


@pytest.mark.parametrize("me, other, backs", [("pinky1", "pinky2", True),
                                              ("pinky3", "pinky2", False)])
def test_without_priorities_the_name_order_decides(me, other, backs):
    fsm = ready(name=me)
    move(fsm, 0.0)
    fsm.see_peers(1.5, [Peer(other, *PEER_AHEAD)])
    stand(fsm, 1.0, STUCK)
    assert bool(backups(tick(fsm, 4.1, STUCK))) is backs


def test_peer_positions_older_than_the_timeout_are_not_used():
    fsm = ready()
    move(fsm, 0.0, priority=0)
    fsm.see_peers(0.0, [Peer("pinky2", *PEER_AHEAD, 2)])
    stand(fsm, 1.0, STUCK)
    assert backups(tick(fsm, 4.1, STUCK)) == []  # peers_timeout_s 3 s
    assert fsm.state() == BLOCKED


def test_at_the_start_of_a_goal_nothing_is_driven_so_nothing_is_backed():
    fsm = ready()
    move(fsm, 0.0, priority=0)
    fsm.see_peers(1.5, [Peer("pinky2", 0.35, 1.75, 2)])
    start = (*P3, NORTH)
    stand(fsm, 1.0, start)
    assert backups(tick(fsm, 4.1, start)) == []
    assert fsm.state() == BLOCKED


def test_blocked_comes_within_blocked_after_s_even_while_backing_up():
    fsm = ready()
    move(fsm, 0.0, priority=0)
    fsm.see_peers(1.5, [Peer("pinky2", *PEER_AHEAD, 2)])
    stand(fsm, 1.0, STUCK)
    fsm.see_peers(4.0, [Peer("pinky2", *PEER_AHEAD, 2)])
    assert backups(tick(fsm, 4.1, STUCK))
    assert tick(fsm, 10.9, STUCK) == []  # 선 때(1.0)부터 10 s 전: BackUp 결과를 기다린다
    cmds = tick(fsm, 11.0, STUCK)
    assert CancelBackUp() in cmds and fsm.state() == BLOCKED


def test_the_yield_is_used_once_per_spot_then_the_robot_reports_blocked():
    fsm = ready()
    move(fsm, 0.0)
    stand(fsm, 1.0, STUCK)
    tick(fsm, 2.5, STUCK, no_obstacle)
    tick(fsm, 3.6, STUCK, no_obstacle)  # 비켜서 다시 갔다
    stand(fsm, 4.0, STUCK)  # 같은 곳에서 또 막혔다
    assert fsm.state() == BLOCKED


def test_with_the_switch_off_the_old_self_recovery_backs_up_at_once():
    fsm = ready(local_yield=False)
    move(fsm, 0.0)
    cmds = stand(fsm, 1.0, STUCK)
    assert kinds(cmds) == ["CancelNav", "BackUp"]
    assert fsm.state() == RECOVERING and fsm.local_yield_step() == ""


def test_the_resend_after_waiting_starts_from_where_the_robot_stands():
    fsm = ready()
    move(fsm, 0.0)
    stand(fsm, 1.0, STUCK)
    tick(fsm, 2.5, STUCK, no_obstacle)
    [send] = [c for c in tick(fsm, 3.6, STUCK, no_obstacle) if isinstance(c, SendPath)]
    assert math.dist(send.poses[0][:2], STUCK[:2]) < 0.03  # 꺾이는 곳 호(2-1) 위의 가장 가까운 점


# ---- order 모델: 합친 경로도 지금 goal이다. priority는 order goal의 것이다 ----------------------


def order_fsm(name="pinky1"):
    fsm = OrderFsm(FsmParams(), robot_id=name)
    fsm.tick(0.0, (0.3, 0.3, 0.0), STILL, no_obstacle)
    return fsm


def order(fsm, t, base, priority, update=0, stitch="", pose=(0.3, 0.3, 0.0)):
    return fsm.move("o%d" % update, t, "", (0.0, 0.0, 0.0), [], pose,
                    order=OrderGoal("o1", update, stitch, tuple(base), ()), priority=priority)


def line(seq, x0, x1):
    return Seg(seq, MODE_FOLLOW_PATH, "FollowPath", ((x0, 0.3, 0.0), (x1, 0.3, 0.0)), None,
               f"N{seq}")


@pytest.mark.parametrize("mine, backs", [(0, True), (5, False)])
def test_an_order_run_backs_up_by_the_priority_of_its_order_goal(mine, backs):
    fsm = order_fsm()
    order(fsm, 0.0, [line(1, 0.3, 0.8)], mine)
    stuck = (0.7, 0.3, 0.0)  # 0.4 m 왔다. 0.4 - 0.23 = 0.17 m까지 된다
    fsm.see_peers(1.5, [Peer("pinky2", 0.85, 0.3, 2)])
    stand(fsm, 1.0, stuck)
    found = backups(tick(fsm, 4.1, stuck))
    assert bool(found) is backs
    if backs:
        assert found[0].distance_m == pytest.approx(0.10)


def test_a_merged_order_path_keeps_room_behind_its_last_node():
    fsm = order_fsm()
    order(fsm, 0.0, [line(1, 0.3, 0.5)], 0)
    tick(fsm, 0.5, (0.45, 0.3, 0.0), no_obstacle, DRIVING)
    order(fsm, 0.6, [line(2, 0.5, 1.0)], 0, update=1, stitch="N1",
          pose=(0.47, 0.3, 0.0))  # 달리는 FollowPath에 이어 붙였다
    stuck = (0.65, 0.3, 0.0)  # 합친 경로 시작에서 0.35 m, 지난 꼭짓점 N1(0.5)에서 0.15 m
    fsm.see_peers(1.5, [Peer("pinky2", 0.85, 0.3, 2)])
    stand(fsm, 1.0, stuck)
    assert backups(tick(fsm, 4.1, stuck)) == []  # 지난 꼭짓점에서 0.23 m를 남겨야 한다
    assert fsm.state() == BLOCKED
