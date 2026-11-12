"""goal은 실패했거나 열린 채 멈췄는데 로봇은 그 goal의 끝 지점에 서 있는 경우.

Gazebo 3대 경기에서 나왔다: 출고 도크에서 물러난 로봇이 끝에 닿았지만 후진 goal이 끝나지 않았고,
다시 보낸 후진은 "남은 길이 없다"로 바로 실패해서 로봇이 격리됐다.
"""
from fleet_core.common.commands import CancelCmd, MoveCmd, RetreatCmd
from zone_harness import Harness


def backing_out(silent_end=True):
    """pinky1이 작업 도크에서 X_W로 물러나는 세계. silent_end면 끝에 닿아도 결과가 오지 않는다."""
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    world.call("OUTBOUND", "pinky1")
    world.run(0.5)
    answer = world.traffic.on_retreat_result
    if silent_end:
        world.traffic.on_retreat_result = lambda *args: None
    return world, world.sent(RetreatCmd, "pinky1")[0], answer


def test_a_retreat_left_open_with_the_robot_at_its_end_is_taken_as_arrived():
    world, retreat, _ = backing_out()
    assert world.run_until(lambda: "GOAL_STALLED" in world.codes("pinky1"), 30.0)
    codes = world.codes("pinky1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "RETREAT_DONE" in codes
    assert "GOAL_RETRY" not in codes, "standing at the end is not a failed attempt"
    assert [c.goal_id for c in world.sent(CancelCmd, "pinky1")] == [retreat.goal_id], \
        "the open goal is cancelled"
    assert world.view("pinky1")["node"] == "X_W"
    assert world.run_until(lambda: world.sent(MoveCmd, "pinky1"), 10.0), "it goes on from X_W"
    assert len(world.sent(RetreatCmd, "pinky1")) == 1, "the retreat is not sent again"
    assert not world.problems


def test_a_retreat_answered_as_failed_with_the_robot_at_its_end_is_taken_as_arrived():
    world, retreat, answer = backing_out()
    assert world.run_until(lambda: world.bots["pinky1"].cmd is None, 20.0), "it reached X_W"
    world.step()
    answer("pinky1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pinky1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "RETREAT_DONE" in codes
    assert "GOAL_RETRY" not in codes
    assert world.view("pinky1")["node"] == "X_W"
    assert not world.sent(CancelCmd, "pinky1"), "the goal has ended, nothing to cancel"


def test_a_retreat_that_fails_short_of_its_end_is_still_a_failed_attempt():
    world, retreat, answer = backing_out(silent_end=False)
    answer("pinky1", retreat.goal_id, False, "FAILED")  # 아직 도크 앞이다
    world.step()
    codes = world.codes("pinky1")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes
    assert world.view("pinky1")["node"] == "W"


def test_a_move_left_open_short_of_its_end_is_still_a_stall():
    world = Harness()
    world.step()
    world.call("WORK", "pinky2")
    world.run(1.0)
    world.bots["pinky2"].cmd = None  # 로봇이 가다가 섰고 결과도 오지 않는다
    assert world.run_until(lambda: "GOAL_STALLED" in world.codes("pinky2"), 30.0)
    codes = world.codes("pinky2")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes


def test_a_move_answered_as_failed_with_the_robot_at_its_end_is_taken_as_arrived():
    world = Harness()
    world.step()
    world.call("WORK", "pinky2")
    world.step()
    first = world.sent(MoveCmd, "pinky2")[0]
    answer, world.traffic.on_move_result = world.traffic.on_move_result, lambda *args: None
    assert world.run_until(lambda: world.bots["pinky2"].cmd is None, 60.0), "it reached N_W"
    world.step()
    answer("pinky2", first.goal_id, False, "NAV_ABORTED")
    codes = world.codes("pinky2")
    world.step()
    assert "GOAL_ENDED_AT_TARGET" in world.codes("pinky2") and "GOAL_RETRY" not in codes
    assert world.view("pinky2")["node"] == first.target_id


def test_the_next_goal_waits_until_the_cancel_of_the_open_goal_has_reached_the_robot():
    world, retreat, _ = backing_out()
    assert world.run_until(lambda: "GOAL_ENDED_AT_TARGET" in world.codes("pinky1"), 30.0)
    world.run(world.params.arrival_settle_s - 0.2)
    assert not world.sent(MoveCmd, "pinky1"), "the cancel may still be on its way"
    assert world.run_until(lambda: world.sent(MoveCmd, "pinky1"), 2.0)


# ---- 위치를 믿을 수 없으면 닿았다고 보지 않는다 ----

def at_the_end_unanswered():
    world, retreat, answer = backing_out()
    assert world.run_until(lambda: world.bots["pinky1"].cmd is None, 20.0), "it reached X_W"
    world.step()
    return world, retreat, answer


def test_a_failure_heard_after_the_reports_stopped_is_not_taken_as_arrived():
    world, retreat, answer = at_the_end_unanswered()
    world.silent.add("pinky1")
    world.run(world.params.arrival_settle_s + 0.5)  # 아직 COMM_LOST는 아니다
    answer("pinky1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pinky1")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes


def test_a_failure_from_a_robot_that_has_lost_its_position_is_not_taken_as_arrived():
    world, retreat, answer = at_the_end_unanswered()
    bot = world.bots["pinky1"]
    world.traffic.on_robot_state("pinky1", bot.x, bot.y, bot.yaw, "LOCALIZING")
    answer("pinky1", retreat.goal_id, False, "FAILED")
    codes = [e.code for e in world.traffic.take_output() if hasattr(e, "code")]
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes


def test_a_robot_that_stopped_itself_without_the_heartbeat_keeps_its_own_handling():
    world, retreat, answer = at_the_end_unanswered()
    answer("pinky1", retreat.goal_id, False, "HEARTBEAT_LOST")
    world.step()
    codes = world.codes("pinky1")
    assert "GOAL_ENDED_AT_TARGET" not in codes and "GOAL_RETRY" not in codes
