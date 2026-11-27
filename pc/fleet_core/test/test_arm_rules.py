"""팔 관제 합의 (2026-11-27) 중 팔 작업 층(ArmJobs) 몫: 작업대 막힘, 쥔 채 락 해제, 상태 끊김,
팔 재시작과 [복구], 담기 진행. 주문 쪽이 받는 보고까지만 본다."""
from core_helpers import cmds, codes
from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import (
    RESUME_REFILL,
    RESUME_SET_PAUSE_OFF,
    ArmDeclined,
    ArmFillCmd,
    ArmFillPlaced,
    ArmFillProgress,
    ArmFillResumed,
    ArmStationBlocked,
)
from fleet_core.common.params import load_params
from stations import STATIONS

ASK = ArmFillCmd("r1", "WORK", "O-004", {"A": 2, "B": 2})


def arm():
    now = [0.0]
    jobs = ArmJobs(STATIONS, load_params(), ["r1", "r2"], lambda: now[0])
    for a in ("arm_left", "arm_right"):
        jobs.on_arm_state(a, "IDLE", "", False, "b1", True)
    jobs.take_output()
    return jobs, now


def filling():
    jobs, now = arm()
    jobs.on_command(ASK)
    goal = cmds(jobs.take_output(), "FillCmd")[0]
    return jobs, now, goal


def faults(out):
    return cmds(out, "ArmFault")


def blocks(out):
    return [(b.station_id, bool(b.reason)) for b in out if isinstance(b, ArmStationBlocked)]


# ---- ④ 한 팔이라도 ERROR면 그 작업대에 새 목표를 보내지 않음 ----
def test_an_arm_in_error_blocks_its_workcell_until_it_leaves_error():
    jobs, _ = arm()
    jobs.on_arm_state("arm_right", "ERROR", "X", False, "b1", True)
    assert blocks(jobs.take_output()) == [("WORK", True)]
    jobs.on_command(ASK)
    assert jobs.take_output() == [ArmDeclined("r1")]
    jobs.on_arm_state("arm_right", "IDLE", "", False, "b1", True)
    assert blocks(jobs.take_output()) == [("WORK", False)]
    jobs.on_command(ASK)
    assert cmds(jobs.take_output(), "FillCmd")


def test_an_arm_error_during_a_fill_gives_it_up_as_an_arm_fault():
    jobs, _, _ = filling()
    jobs.on_arm_state("arm_left", "ERROR", "X", False, "b1", True)
    out = jobs.take_output()
    assert [f.robot for f in faults(out)] == ["r1"] and "ARM_ERROR" in codes(out)


# ---- ⑤ 쥔 채 락 해제: 목표는 이어 가고, 끝날 때 보류 ----
def test_a_holding_lock_release_keeps_the_goal_and_holds_it_when_it_ends():
    jobs, _, goal = filling()
    jobs.on_arm_state("arm_left", "PICKING", "", True, "b1", True)
    jobs.on_arm_state("arm_left", "PAUSED", "LOCK_RELEASED", True, "b1", False)
    out = jobs.take_output()
    assert not cmds(out, "CancelCmd") and not faults(out)
    assert "ARM_LOCK_RELEASED" in codes(out)
    jobs.on_fill_result(goal.goal_id, True, {"A": 2, "B": 2}, {})
    out = jobs.take_output()
    assert cmds(out, "ArmFillResult")[0].held
    assert [f.robot for f in faults(out)] == ["r1"]


def test_a_lock_release_seen_after_the_gripper_opened_counts_within_the_same_goal():
    jobs, _, goal = filling()
    jobs.on_arm_state("arm_left", "PICKING", "", True, "b1", True)
    jobs.on_arm_state("arm_left", "PAUSED", "LOCK_RELEASED", False, "b1", False)
    jobs.take_output()
    jobs.on_fill_result(goal.goal_id, True, {"A": 2, "B": 2}, {})
    assert faults(jobs.take_output())


def test_holding_seen_before_the_goal_does_not_count():
    jobs, _ = arm()
    jobs.on_arm_state("arm_left", "PLACING", "", True, "b1", True)
    jobs.on_command(ASK)
    goal = cmds(jobs.take_output(), "FillCmd")[0]
    jobs.on_arm_state("arm_left", "PAUSED", "LOCK_RELEASED", False, "b1", False)
    jobs.on_fill_result(goal.goal_id, True, {"A": 2, "B": 2}, {})
    assert not faults(jobs.take_output())


def test_a_lock_release_on_an_empty_arm_changes_nothing():
    jobs, _, goal = filling()
    jobs.on_arm_state("arm_left", "PAUSED", "LOCK_RELEASED", False, "b1", False)
    jobs.on_fill_result(goal.goal_id, True, {"A": 2, "B": 2}, {})
    assert not faults(jobs.take_output())


# ---- ⑥ 상태가 끊기면 상태 모름: 배차를 막되 담기는 보류하지 않음 ----
def test_a_lost_arm_state_blocks_the_workcell_without_holding_the_fill():
    jobs, _, _ = filling()
    jobs.on_arm_state_lost("arm_left")
    out = jobs.take_output()
    assert blocks(out) == [("WORK", True)] and not faults(out)
    assert "ARM_STATE_LOST" in codes(out)
    jobs.on_arm_state("arm_left", "PICKING", "", False, "b1", True)
    out = jobs.take_output()
    assert blocks(out) == [("WORK", False)] and "ARM_STATE_BACK" in codes(out)


# ---- ⑪ 재시작([초기화])과 [복구] ----
def restarted(holding=False):
    jobs, now, goal = filling()
    jobs.on_fill_feedback(goal.goal_id, "arm_left", "PLACING", 2)
    if holding:
        jobs.on_arm_state("arm_left", "PICKING", "", True, "b1", True)
    jobs.take_output()
    jobs.on_arm_state("arm_left", "IDLE", "", False, "b2", True)
    return jobs, now, goal, jobs.take_output()


def test_a_restart_counts_the_last_feedback_and_waits_for_resume():
    _, _, _, out = restarted()
    assert ArmFillPlaced("r1", {"A": 2}) in out
    assert not faults(out) and "ARM_RESTARTED" in codes(out)


def test_resume_after_a_restart_asks_for_the_rest():
    jobs, _, _, _ = restarted()
    assert jobs.on_resume_arm("arm_right") == (RESUME_REFILL, "")
    assert ArmFillResumed("r1") in jobs.take_output()
    jobs.on_command(ArmFillCmd("r1", "WORK", "O-004", {"B": 2}))
    assert cmds(jobs.take_output(), "FillCmd")[0].items == {"B": 2}


def test_no_new_goal_goes_out_before_resume():
    jobs, _, _, _ = restarted()
    jobs.on_command(ArmFillCmd("r1", "WORK", "O-004", {"B": 2}))
    assert jobs.take_output() == [ArmDeclined("r1")]


def test_a_late_result_of_the_dropped_goal_is_ignored():
    jobs, _, goal, _ = restarted()
    jobs.on_fill_result(goal.goal_id, True, {"A": 2, "B": 2}, {})
    out = jobs.take_output()
    assert not cmds(out, "ArmFillResult") and not cmds(out, "ArmLate")
    assert "ARM_LATE_RESULT" in codes(out)


def test_a_restart_while_holding_holds_without_resume():
    jobs, _, _, out = restarted(holding=True)
    placed = [p for p in out if isinstance(p, ArmFillPlaced)]
    assert placed == [ArmFillPlaced("r1", {"A": 2}, held=True)]
    assert [f.robot for f in faults(out)] == ["r1"]
    assert jobs.on_resume_arm("arm_left") == ("", "ORDER_STATE")


def test_resume_on_a_live_goal_unpauses_both_arms():
    jobs, _, _ = filling()
    assert jobs.on_resume_arm("arm_left") == (RESUME_SET_PAUSE_OFF, "")


def test_resume_refusals():
    jobs, _ = arm()
    assert jobs.on_resume_arm("arm_nowhere") == ("", "NOT_FOUND")
    assert jobs.on_resume_arm("arm_left") == ("", "ORDER_STATE")


# ---- ⑫ 담기 진행: 담은 수는 마지막 feedback ----
def test_feedback_reports_the_placed_items_in_goal_order_and_the_last_count_wins():
    jobs, _, goal = filling()
    jobs.on_fill_feedback(goal.goal_id, "arm_left", "PLACING", 3)
    assert jobs.take_output() == [ArmFillProgress("r1", {"A": 2, "B": 1})]
    jobs.on_fill_feedback(goal.goal_id, "arm_right", "RETURNING", 2)
    assert jobs.take_output() == [ArmFillProgress("r1", {"A": 2})]
    assert jobs.arm_task("arm_right") == ("RETURNING", "O-004")
    jobs.on_fill_feedback(goal.goal_id + 9, "arm_left", "PLACING", 4)
    assert not jobs.take_output()
