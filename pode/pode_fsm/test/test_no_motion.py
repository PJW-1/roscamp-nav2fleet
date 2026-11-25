"""동작 없음 대역(no_motion.py)과 FSM 도킹 대기. ROS 없이 돈다."""
import math

from pode_fsm.core import FsmParams, PodeFsm
from pode_fsm.no_motion import (
    BACKUP,
    FOLLOW,
    GOAL,
    BackUpStub,
    FollowStub,
    NavGoalStub,
    NoMotion,
    NoMotionParams,
    dock_phase,
    dock_warning,
)
from pode_fsm.results import CANCELED, DOCKED, DOCKING, HEARTBEAT_LOST, IDLE, Finish

P = NoMotionParams(move_s=2.0, retreat_s=1.0, dock_s=3.0)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_a_path_succeeds_after_the_wait_and_the_pose_jumps_to_its_end():
    nm, clock = NoMotion((0.0, 0.0, 0.0), P), Clock()
    assert FollowStub(nm, clock).send(1, [(0.0, 0.0, 0.0), (0.5, 0.2, 1.0)], True, False)
    clock.t += 1.9
    assert nm.step(clock.t) == [] and nm.pose == (0.0, 0.0, 0.0)
    clock.t += 0.1
    [job] = nm.step(clock.t)
    assert (job.kind, job.key, job.canceled) == (FOLLOW, 1, False)
    assert nm.pose == (0.5, 0.2, 1.0)


def test_a_reverse_path_uses_the_retreat_wait():
    nm, clock = NoMotion((0.0, 0.0, 0.0), P), Clock()
    FollowStub(nm, clock).send(1, [(0.0, 0.0, 0.0), (-0.2, 0.0, 0.0)], True, True)
    [job] = nm.step(clock.t + 1.0)
    assert not job.canceled and nm.pose == (-0.2, 0.0, 0.0)


def test_a_resent_path_to_the_same_end_keeps_the_first_wait():
    """FSM이 끝 가까이에서 남은 경로를 다시 보내도 기다림이 늘지 않는다."""
    nm, clock = NoMotion((0.0, 0.0, 0.0), P), Clock()
    follow = FollowStub(nm, clock)
    follow.send(1, [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)], True, False)
    clock.t += 1.5
    follow.send(2, [(0.05, 0.0, 0.0), (0.1, 0.0, 0.0)], True, False)
    [old] = nm.step(clock.t)
    assert (old.key, old.canceled) == (1, True)
    [new] = nm.step(clock.t + 0.5)
    assert (new.key, new.canceled) == (2, False)


def test_a_path_to_another_end_waits_again():
    nm, clock = NoMotion((0.0, 0.0, 0.0), P), Clock()
    follow = FollowStub(nm, clock)
    follow.send(1, [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)], True, False)
    clock.t += 1.5
    follow.send(2, [(0.0, 0.0, 0.0), (0.0, 0.3, 0.0)], True, False)
    assert [j.key for j in nm.step(clock.t + 0.5)] == [1]
    assert [j.key for j in nm.step(clock.t + 2.0)] == [2]


def test_cancel_reports_a_canceled_job_at_once_and_does_not_move():
    nm, clock = NoMotion((0.0, 0.0, 0.0), P), Clock()
    NavGoalStub(nm, clock).send(4, (1.0, 1.0, 0.0))
    nm.cancel(GOAL)
    [job] = nm.step(clock.t)
    assert (job.kind, job.key, job.canceled) == (GOAL, 4, True)
    assert nm.pose == (0.0, 0.0, 0.0)
    assert nm.step(clock.t + 10.0) == []


def test_cancel_touches_only_its_kind():
    nm, clock = NoMotion((0.0, 0.0, 0.0), P), Clock()
    NavGoalStub(nm, clock).send(1, (1.0, 1.0, 0.0))
    BackUpStub(nm, clock).send(1, 0.1, 0.05, 5.0)
    nm.cancel(BACKUP)
    assert nm.busy(GOAL) and not nm.busy(BACKUP)


def test_backup_goes_straight_back_along_the_heading():
    nm, clock = NoMotion((1.0, 1.0, math.pi / 2), P), Clock()
    BackUpStub(nm, clock).send(1, 0.2, 0.05, 5.0)
    nm.step(clock.t + 1.0)
    assert math.dist(nm.pose[:2], (1.0, 0.8)) < 1e-9
    assert nm.pose[2] == math.pi / 2


def test_dock_phases_match_the_pc_dock_stand_in():
    assert [dock_phase(t, 3.0) for t in (0.0, 1.4, 1.5, 2.9, 3.0)] == [
        "APPROACH", "APPROACH", "ALIGN", "ALIGN", "DONE"]


def test_fsm_dock_wait_reports_docking_then_docked():
    fsm = PodeFsm(FsmParams(heartbeat_timeout_s=0.0))
    fsm.tick(0.0, (0.0, 0.0, 0.0), (0.0, 0.0), lambda _p: False)
    assert fsm.dock_wait("g", 0.0, "WORK") == []
    assert (fsm.state(), fsm.current_goal()) == (DOCKING, "WORK")
    assert fsm.dock_done("other") == []
    assert fsm.dock_done("g") == [Finish("g", True, "")]
    assert (fsm.state(), fsm.current_goal()) == (DOCKED, "")


def test_fsm_dock_wait_can_be_canceled_and_stops_on_heartbeat_loss():
    fsm = PodeFsm(FsmParams(heartbeat_timeout_s=1.5))
    fsm.heartbeat(0.0)
    fsm.tick(0.0, (0.0, 0.0, 0.0), (0.0, 0.0), lambda _p: False)
    fsm.dock_wait("g", 0.0, "WORK")
    assert fsm.cancel("g", 0.5) == [Finish("g", False, CANCELED)]
    assert fsm.dock_done("g") == [] and fsm.state() == IDLE
    fsm.dock_wait("h", 1.0, "WORK")
    out = fsm.tick(2.6, (0.0, 0.0, 0.0), (0.0, 0.0), lambda _p: False)
    assert Finish("h", False, HEARTBEAT_LOST) in out
    assert fsm.state() == IDLE


# ---- 도킹 이름: 거리는 보지 않고, 모르는 이름이면 경고만(2026-11-24 결정) ----
def test_default_dock_ids_are_the_fleet_station_names():
    assert NoMotionParams().dock_ids == ("WORK", "OUTBOUND")


def test_known_dock_ids_give_no_warning_and_others_do():
    assert dock_warning("WORK", P.dock_ids) is None
    assert dock_warning("OUTBOUND", P.dock_ids) is None
    for bad in ("NOPE", "", "work"):
        msg = dock_warning(bad, P.dock_ids)
        assert msg is not None and repr(bad) in msg and "WORK" in msg


def test_the_stand_ins_are_always_ready_so_the_fsm_leaves_localizing():
    nm, clock = NoMotion((0.12, 0.10, math.pi / 2), P), Clock()
    follow, goals = FollowStub(nm, clock), NavGoalStub(nm, clock)
    assert follow.ready() and goals.ready() and BackUpStub(nm, clock).ready()
    fsm = PodeFsm(FsmParams(heartbeat_timeout_s=0.0))
    fsm.set_nav_ready(follow.ready() and goals.ready())
    fsm.tick(0.0, (0.12, 0.10, math.pi / 2), (0.0, 0.0), lambda _p: False)
    assert fsm.state() == IDLE
