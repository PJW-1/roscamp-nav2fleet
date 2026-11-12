"""구역 교통 층의 실패한 명령: 이동 실패와 다시 보내기, 거절, 못 보냄, 잃음, heartbeat 끊김,
성공 뒤의 도착 판정, 그리고 이름으로 부른 배차의 거절.

로봇은 움직이지 않는다(speed 0). 결과는 시험이 손으로 넣는다.
"""
import pytest
from fleet_core.common.commands import DispatchResult, MoveCmd, RobotIsolated
from fleet_core.traffic.mission import HEARTBEAT_LOST, NOT_AT_TARGET
from zone_harness import Harness


def sent_once():
    """pinky2가 작업 도크로 불려 첫 Nav2 goal(N_W)을 받았다. 로봇은 그 자리에 선다."""
    world = Harness(speed=0.0)
    world.step()
    call = world.call("WORK", "pinky2")
    world.step()
    assert [c.target_id for c in world.sent(MoveCmd, "pinky2")] == ["N_W"]
    return world, call


def last_goal(world, robot="pinky2"):
    return world.sent(MoveCmd, robot)[-1].goal_id


def moves(world, robot="pinky2"):
    return len(world.sent(MoveCmd, robot))


def isolations(world):
    return [c for c in world.commands if isinstance(c, RobotIsolated)]


# ---- 이동 실패와 다시 보내기 ----

@pytest.mark.parametrize("fail", ["result", "rejected"])
def test_a_failed_move_is_sent_again_after_the_wait_and_isolates_after_the_limit(fail):
    world, call = sent_once()
    p = world.params
    for attempt in range(1, p.move_retry_limit + 2):
        if fail == "result":
            world.traffic.on_move_result("pinky2", last_goal(world), False, "ABORTED")
        else:
            world.traffic.on_goal_rejected("pinky2", last_goal(world))
        if attempt > p.move_retry_limit:
            break
        before = moves(world)
        world.run(p.move_retry_wait_s - 0.3)
        assert moves(world) == before, "nothing new before the retry wait is over"
        assert world.view("pinky2")["attempts"] == attempt
        world.run(0.5)
        assert moves(world) == before + 1, "the same goal goes out again"
        assert world.sent(MoveCmd, "pinky2")[-1].target_id == "N_W"
    world.step()
    reason = "ABORTED" if fail == "result" else "REJECTED"
    assert world.codes("pinky2").count("GOAL_RETRY") == p.move_retry_limit
    assert world.codes("pinky2")[-2:] == ["GOAL_FAILED", "ROBOT_ISOLATED"]
    assert (world.view("pinky2")["state"], world.result_of(call).reason) == ("ERROR", reason)
    assert not world.result_of(call).success


def test_a_result_for_another_goal_is_ignored():
    world, _ = sent_once()
    world.traffic.on_move_result("pinky2", last_goal(world) + 1, False, "ABORTED")
    world.traffic.on_goal_rejected("pinky2", last_goal(world) + 1)
    world.traffic.on_move_result("pinky1", last_goal(world), False, "ABORTED")
    world.step()
    assert "GOAL_RETRY" not in world.codes()
    assert world.view("pinky2")["goal_id"] == last_goal(world)


# ---- 시도로 세지 않는 실패 ----

def test_a_goal_that_could_not_be_sent_goes_again_after_the_wait_and_is_not_an_attempt():
    world, call = sent_once()
    p = world.params
    for _ in range(p.move_retry_limit + 3):
        world.traffic.on_goal_not_sent("pinky2", last_goal(world))
        before = moves(world)
        world.run(p.move_retry_wait_s - 0.3)
        assert moves(world) == before
        world.run(0.5)
        assert moves(world) == before + 1
    assert world.codes("pinky2").count("GOAL_NOT_SENT") == 1, "warned once"
    view = world.view("pinky2")
    assert (view["state"], view["attempts"]) == ("TO_WORK", 0)
    assert world.result_of(call) is None and not isolations(world)


def test_a_lost_goal_is_sent_again_at_once():
    world, _ = sent_once()
    lost = last_goal(world)
    world.traffic.on_goal_lost("pinky2", lost)
    assert world.view("pinky2")["goal_id"] is None
    world.step()
    assert moves(world) == 2 and last_goal(world) != lost
    assert world.view("pinky2")["attempts"] == 0
    assert "GOAL_RETRY" not in world.codes()


def test_a_move_the_robot_ended_on_losing_the_heartbeat_waits_and_is_not_an_attempt():
    world, call = sent_once()
    p = world.params
    for _ in range(p.move_retry_limit + 3):
        world.traffic.on_move_result("pinky2", last_goal(world), False, HEARTBEAT_LOST)
        before = moves(world)
        world.run(p.move_retry_wait_s - 0.3)
        assert moves(world) == before, "it waits like a retry"
        world.run(0.5)
        assert moves(world) == before + 1
    codes = world.codes("pinky2")
    assert "GOAL_RETRY" not in codes and "GOAL_FAILED" not in codes
    assert world.view("pinky2")["attempts"] == 0
    assert world.result_of(call) is None and not isolations(world)


# ---- 성공 뒤의 도착 판정 ----

def test_a_success_reported_away_from_the_target_waits_for_the_pose_then_counts_as_failed():
    world, _ = sent_once()
    goal = last_goal(world)
    world.traffic.on_move_result("pinky2", goal, True, "")  # 로봇은 아직 P-2에 서 있다
    world.run(world.params.arrival_settle_s - 0.3)
    view = world.view("pinky2")
    assert (view["goal_id"], view["node"]) == (goal, "P-2"), "it waits for a report at N_W"
    world.run(0.5)
    assert world.view("pinky2")["goal_id"] is None
    retry = [e.detail for e in world.events if e.code == "GOAL_RETRY"]
    assert retry == [f"{NOT_AT_TARGET}, attempt 1"]


def test_a_success_whose_pose_arrives_late_ends_the_segment():
    world, _ = sent_once()
    goal = last_goal(world)
    world.traffic.on_move_result("pinky2", goal, True, "")
    world.step()
    bot = world.bots["pinky2"]
    bot.cmd, bot.points = None, []
    bot.x, bot.y = world.site.node_pose("N_W")  # 늦게 온 보고가 N_W에 있다
    world.step()
    world.step()
    assert world.view("pinky2")["node"] == "N_W"
    assert [(c.mode, c.target_id) for c in world.sent(MoveCmd, "pinky2")] == [
        ("nav_goal", "N_W"), ("follow_path", "W")]
    assert "GOAL_RETRY" not in world.codes()


def test_a_success_with_the_pose_already_at_the_target_ends_the_segment_at_once():
    world, _ = sent_once()
    goal = last_goal(world)
    world.traffic.on_robot_state("pinky2", *world.site.node_pose("N_W"), 0.0, "MOVING")
    world.traffic.on_move_result("pinky2", goal, True, "")  # 자세는 이미 닿았다
    assert world.view("pinky2")["node"] == "N_W", "a pose at the target ends it at once"


# ---- 이름으로 부른 배차의 거절 ----

def test_a_named_dispatch_is_refused_for_an_unknown_an_unavailable_or_a_busy_robot():
    world = Harness()
    world.step()
    world.traffic.on_robot_fault("pinky3", "MOTOR")
    first = world.call("WORK", "pinky2")
    for robot in ("ghost", "pinky3", "pinky2"):
        world.call("OUTBOUND", robot)
    world.step()
    refused = [(r.robot_id, r.reason) for r in world.results if not r.success]
    assert refused == [("ghost", "NO_SUCH_ROBOT"), ("pinky3", "NOT_AVAILABLE"),
                       ("pinky2", "BUSY")]
    assert world.view("pinky2")["destination"] == "WORK", "the first call stands"
    assert world.result_of(first) is None


def test_a_silent_robot_is_not_available_for_a_named_dispatch():
    world = Harness()
    world.step()
    world.silent.add("pinky1")
    world.run(world.params.comm_timeout_s + 0.5)
    call = world.call("WORK", "pinky1")
    world.step()
    assert world.result_of(call) == DispatchResult(call, False, "pinky1", "NOT_AVAILABLE")


def test_an_open_call_is_only_for_the_fill_station():
    world = Harness()
    with pytest.raises(ValueError):
        world.call("OUTBOUND")


def test_a_dispatch_to_the_station_it_is_docked_at_is_answered_at_once():
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    first = world.call("WORK", "pinky1")
    assert world.run_until(lambda: world.result_of(first) is not None, 30.0)
    docked = len(world.commands)
    world.traffic.on_dispatch(4242, "WORK", "pinky1")  # 도크에 선 채로 또 부른다
    out = world.traffic.take_output()
    assert DispatchResult(4242, True, "pinky1") in out
    assert len(world.commands) == docked, "no new command"
