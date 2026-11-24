"""IR-62: goal 결과를 잃으면 결과 모름. 구역을 쥔 채 상태 보고로 한 번만 정하고, 못 정하면 격리한다."""
import dataclasses
import math

import pytest

from fleet_core.common.commands import CancelCmd, DockCmd, MoveCmd, MoveOrderCmd, RobotIsolated
from test_zone_move_orders import orders, waiting_at_the_queue_spot
from zone_harness import Harness


def sent_once(speed=0.0):
    """pode2가 작업 도크로 불려 첫 Nav2 goal(N_W)을 받았다. speed 0이면 로봇은 그 자리에 선다."""
    world = Harness(speed=speed)
    world.step()
    call = world.call("WORK", "pode2")
    world.step()
    assert [leg.target_id for leg in world.moves("pode2")] == ["N_W"]
    return world, call


def last_goal(world, robot="pode2"):
    return world.sent((MoveCmd, MoveOrderCmd), robot)[-1].goal_id


def moves(world, robot="pode2"):
    return len(world.moves(robot))


def isolations(world, reason=""):
    return [c for c in world.commands
            if isinstance(c, RobotIsolated) and (not reason or c.reason == reason)]


def lose(world, goal_id, robot="pode2"):
    world.traffic.on_goal_lost(robot, goal_id)
    return goal_id


def stop_quietly(world, robot="pode2", at=None):
    """로봇이 goal을 끝내고 서지만 결과는 관제에 오지 않는다. at이면 그 노드에 선다."""
    bot = world.bots[robot]
    if at is not None:
        bot.x, bot.y = world.site.node_pose(at)
    bot.cmd, bot.run, bot.points = None, None, []


def reports(world, robot, state):
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def test_a_lost_result_keeps_the_goal_and_its_zones_and_times_out_into_an_isolation():
    world, call = sent_once()
    p = world.params
    held = world.traffic.grants.held_by("pode2")
    lost = lose(world, last_goal(world))
    assert world.view("pode2")["goal_id"] == lost, "not forgotten at once"
    world.step()
    assert world.codes("pode2")[-1] == "ROBOT_TIMEOUT"
    world.run(p.robot_timeout_settle_s - 0.5)
    codes = world.codes("pode2")
    assert moves(world) == 1, "no new goal while the result is unknown"
    assert world.view("pode2")["goal_id"] == lost
    assert world.traffic.grants.held_by("pode2") == held, "its zones stay held"
    assert "GOAL_STALLED" not in codes and "GOAL_RETRY" not in codes, "nothing else settles it"
    assert not isolations(world)
    world.run(1.0)
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]
    assert "ROBOT_ISOLATED" in world.codes("pode2")
    assert world.result_of(call) is not None and not world.result_of(call).success


def test_a_lost_result_is_done_once_the_robot_reports_idle_at_the_end():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.run(3.0)
    assert moves(world) == 1, "MOVING settles nothing"
    stop_quietly(world, at="N_W")
    world.step()
    view = world.view("pode2")
    assert view["node"] == "N_W" and view["attempts"] == 0
    assert view["goal_id"] != lost and moves(world) == 2, "the next leg goes out"
    assert "GOAL_ENDED_AT_TARGET" in world.codes("pode2")
    world.run(world.params.robot_timeout_settle_s)
    assert not isolations(world, "ROBOT_TIMEOUT"), "settled once"


def test_a_lost_result_is_a_failed_attempt_once_the_robot_reports_idle_elsewhere():
    world, call = sent_once()
    p = world.params
    lost = lose(world, last_goal(world))
    stop_quietly(world)
    world.step()
    view = world.view("pode2")
    assert view["goal_id"] is None and view["attempts"] == 1
    assert world.codes("pode2")[-1] == "GOAL_RETRY"
    world.run(p.move_retry_wait_s + 0.5)
    assert moves(world) == 2 and last_goal(world) != lost, "the retry flow sends it again"
    world.run(p.robot_timeout_settle_s)
    assert world.codes("pode2").count("ROBOT_TIMEOUT") == 1
    assert not isolations(world, "ROBOT_TIMEOUT"), "settled once"


def test_a_late_result_inside_the_settle_time_settles_it_like_any_result():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.run(5.0)
    world.traffic.on_move_result("pode2", lost, False, "NAV_ABORTED")
    assert world.view("pode2")["goal_id"] is None and world.view("pode2")["attempts"] == 1
    world.run(world.params.robot_timeout_settle_s)
    assert not isolations(world, "ROBOT_TIMEOUT")


def lost_dock(world):
    """pode2가 작업 도크에서 도킹하다 결과를 잃었다. 로봇은 도킹을 끝내고 선다."""
    assert world.run_until(lambda: world.sent(DockCmd, "pode2"), 300.0)
    dock = world.sent(DockCmd, "pode2")[-1]
    reports(world, "pode2", "DOCKING")
    lose(world, dock.goal_id)
    world.bots["pode2"].dock = None
    world.run(2.0)
    return dock


def stand_by_the_dock(world, dx=0.0, dy=0.0, dyaw_deg=0.0, robot="pode2", dock="W"):
    spec, bot = world.site.nodes[dock], world.bots[robot]
    bot.x, bot.y, bot.yaw = spec.x + dx, spec.y + dy, spec.yaw + math.radians(dyaw_deg)


@pytest.mark.parametrize("state", ["IDLE", "DOCKED"])
def test_a_lost_dock_result_is_docked_when_the_robot_reports_from_the_dock_pose(state):
    world, call = sent_once(speed=0.12)
    dock = lost_dock(world)
    stand_by_the_dock(world, 0.03, -0.02, 5.0)
    reports(world, "pode2", state)
    world.step()
    result = world.result_of(call)
    assert result is not None and result.success and len(world.sent(DockCmd, "pode2")) == 1
    assert (round(result.error_x_cm, 3), round(result.error_y_cm, 3)) == (3.0, -2.0)
    assert round(result.error_yaw_deg, 3) == 5.0, "the error is the reported pose off the dock"
    assert dock.goal_id in [c.goal_id for c in world.sent(CancelCmd, "pode2")]
    assert "DOCKED" in world.codes("pode2") and not isolations(world)


@pytest.mark.parametrize(("dx", "dyaw_deg"), [(0.2, 0.0), (0.0, 30.0)])
def test_a_lost_dock_result_docks_again_when_the_robot_reports_off_the_dock_pose(dx, dyaw_deg):
    world, call = sent_once(speed=0.12)
    dock = lost_dock(world)
    stand_by_the_dock(world, dx, 0.0, dyaw_deg)
    reports(world, "pode2", "IDLE")
    world.step()
    assert world.result_of(call) is None and world.codes("pode2")[-1] == "GOAL_RETRY"
    assert dock.goal_id in [c.goal_id for c in world.sent(CancelCmd, "pode2")]
    world.__dict__.pop("_state")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    result = world.result_of(call)
    assert result.success and len(world.sent(DockCmd, "pode2")) == 2
    assert result.error_x_cm == 0.4, "the error measured by the second dock"


def test_a_dock_without_a_heading_on_the_map_cannot_be_settled_by_the_pose():
    world, call = sent_once(speed=0.12)
    world.site.nodes["W"] = dataclasses.replace(world.site.nodes["W"], yaw=None)
    lost_dock(world)
    bot, spec = world.bots["pode2"], world.site.nodes["W"]
    bot.x, bot.y = spec.x, spec.y
    reports(world, "pode2", "IDLE")
    world.step()
    assert world.result_of(call) is None and world.codes("pode2")[-1] == "GOAL_RETRY"


def test_a_dock_settled_just_before_the_deadline_is_judged_at_the_report_time(monkeypatch):
    world, call = sent_once(speed=0.12)
    lost_dock(world)
    stand_by_the_dock(world)
    deadline = lost_state(world) + world.params.robot_timeout_settle_s
    reads = iter([deadline - 0.001])
    monkeypatch.setattr(world.traffic, "_clock", lambda: next(reads, deadline + 0.001))
    bot = world.bots["pode2"]
    world.traffic.on_robot_state("pode2", bot.x, bot.y, bot.yaw, "IDLE")
    monkeypatch.undo()
    world.now = deadline + 0.001
    world.step()
    assert world.result_of(call) is not None and world.result_of(call).success
    assert not isolations(world)


def test_the_dock_pose_tolerance_comes_from_the_parameters():
    world, call = sent_once(speed=0.12)
    world.traffic.params = dataclasses.replace(world.params, lost_dock_reach_m=0.25,
                                               lost_dock_yaw_deg=40.0)
    lost_dock(world)
    stand_by_the_dock(world, 0.2, 0.0, 30.0)
    reports(world, "pode2", "IDLE")
    world.step()
    assert world.result_of(call) is not None and world.result_of(call).success


# ---- 결과 모름 동안 다른 길이 goal을 건드리지 않는다, 정하는 시각의 경계 ----

def lost_state(world, robot="pode2"):
    return world.traffic._robots[robot].result_lost_at


def test_settling_on_idle_cancels_the_lost_goal_so_the_node_can_let_it_go():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    stop_quietly(world)
    world.step()
    assert lost in [c.goal_id for c in world.sent(CancelCmd, "pode2")]


def test_a_robot_that_falls_silent_while_unknown_still_times_out_into_an_isolation():
    world, call = sent_once()
    lose(world, last_goal(world))
    world.silent.add("pode2")
    world.run(world.params.robot_timeout_settle_s + 0.5)
    assert "COMM_LOST" in world.codes("pode2")
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]


def test_blocked_reports_while_unknown_mark_nothing_and_cancel_nothing():
    world, call = sent_once()
    held = world.traffic.grants.held_by("pode2")
    lost = lose(world, last_goal(world))
    reports(world, "pode2", "BLOCKED")
    world.run(world.params.robot_timeout_settle_s - 1.0)
    assert world.view("pode2")["goal_id"] == lost
    assert world.traffic.grants.held_by("pode2") == held
    assert lost not in [c.goal_id for c in world.sent(CancelCmd, "pode2")]
    world.run(2.0)
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]


def test_a_cancelled_call_does_not_drop_a_goal_whose_result_is_unknown():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.traffic.on_dispatch_cancel(call)
    world.run(2.0)
    assert world.view("pode2")["goal_id"] == lost and moves(world) == 1
    assert lost not in [c.goal_id for c in world.sent(CancelCmd, "pode2")]


def test_a_late_success_result_ends_the_unknown_even_short_of_the_end():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.step()
    world.traffic.on_move_result("pode2", lost, True, "")
    assert lost_state(world) is None, "the late answer settled it"
    world.run(world.params.robot_timeout_settle_s + 1.0)
    assert not isolations(world, "ROBOT_TIMEOUT")


def test_an_idle_report_heard_after_the_settle_time_is_too_late():
    world, call = sent_once()
    lose(world, last_goal(world))
    world.now += world.params.robot_timeout_settle_s  # tick이 밀려 그 사이 보고가 없었다
    stop_quietly(world)
    world.step()
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]
    assert "GOAL_RETRY" not in world.codes("pode2")


def test_an_idle_report_heard_in_time_settles_even_if_a_later_report_comes_before_the_tick():
    world, call = sent_once()
    lost_at = world.now
    lose(world, last_goal(world))
    world.now = lost_at + world.params.robot_timeout_settle_s - 0.05
    stop_quietly(world)
    world.report()  # 기한 안에 들은 IDLE
    world.now = lost_at + world.params.robot_timeout_settle_s + 0.05
    world.step()  # 기한 뒤의 보고가 덮어쓴 다음 tick
    assert not isolations(world, "ROBOT_TIMEOUT")
    assert "GOAL_RETRY" in world.codes("pode2")


def test_a_silent_robot_that_comes_back_after_the_settle_time_is_still_isolated():
    world, call = sent_once()
    lost_at = world.now
    lose(world, last_goal(world))
    world.silent.add("pode2")
    world.run(world.params.comm_timeout_s + 1.0)
    assert "COMM_LOST" in world.codes("pode2")
    world.now = lost_at + world.params.robot_timeout_settle_s + 0.5
    world.silent.discard("pode2")
    world.report()  # 기한이 지난 뒤 tick보다 먼저 온 보고
    world.step()
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]


def test_a_result_after_the_settle_time_comes_too_late_even_before_the_tick():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.now += world.params.robot_timeout_settle_s + 0.5
    world.traffic.on_move_result("pode2", lost, True, "")
    world.step()
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]


def test_an_idle_report_at_the_end_from_a_robot_that_was_silent_is_no_failed_attempt():
    """통신이 끊겼던 로봇은 도착을 믿지 못하므로 결과 모름이 아니라 통신 복구가 goal을 끝낸다."""
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.silent.add("pode2")
    world.run(world.params.comm_timeout_s + 1.0)
    assert "COMM_LOST" in world.codes("pode2") and lost_state(world) is not None
    stop_quietly(world, at="N_W")
    world.silent.discard("pode2")
    world.report()
    assert world.view("pode2")["attempts"] == 0, "not counted as a failed attempt"
    assert lost_state(world) is None, "the recovery ended the goal"
    world.step()
    assert lost in [c.goal_id for c in world.sent(CancelCmd, "pode2")]
    assert not isolations(world)


def test_a_localizing_report_after_the_settle_time_isolates_at_once():
    world, call = sent_once()
    lose(world, last_goal(world))
    world.now += world.params.robot_timeout_settle_s + 0.5
    reports(world, "pode2", "LOCALIZING")
    world.report()
    assert lost_state(world) is None, "settled by the report, not the next tick"
    world.step()
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]


def test_an_old_order_goal_whose_result_was_lost_is_unknown_once_a_failed_update_restores_it():
    """갱신 중 이전 goal의 결과를 잃고 갱신이 실패하면, 되살린 goal은 결과가 오지 않는 결과 모름이다."""
    world, call = waiting_at_the_queue_spot()
    first = orders(world, "pode1")[-1]
    world.hold_answers = True
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: len(orders(world, "pode1")) == 2, 30.0)
    update = orders(world, "pode1")[-1]
    lose(world, first.goal_id, "pode1")
    assert lost_state(world, "pode1") is None, "the fleet still waits for the update goal"
    world.traffic.on_goal_not_sent("pode1", update.goal_id)
    assert world.view("pode1")["goal_id"] == first.goal_id
    assert lost_state(world, "pode1") is not None, "the restored goal has no result to wait for"
    stop_quietly(world, "pode1")
    world.report()
    assert lost_state(world, "pode1") is None, "settled by the IDLE report"
    world.step()
    assert first.goal_id in [c.goal_id for c in world.sent(CancelCmd, "pode1")]


def test_an_isolation_by_a_late_report_keeps_the_pose_of_that_report():
    world, call = sent_once()
    lose(world, last_goal(world))
    world.now += world.params.robot_timeout_settle_s + 0.5
    bot = world.bots["pode2"]
    bot.x += 0.3
    world.report()
    assert world.traffic._robots["pode2"].health.fault_xy == (bot.x, bot.y)
    world.step()
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]
