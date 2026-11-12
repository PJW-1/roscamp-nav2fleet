"""구역 교통 층의 실패한 명령: 이동 실패와 다시 보내기, 거절, 못 보냄, 잃음, heartbeat 끊김,
성공 뒤의 도착 판정, 그리고 이름으로 부른 배차의 거절.

로봇은 움직이지 않는다(speed 0). 결과는 시험이 손으로 넣는다.
"""
import math

import pytest
from fleet_core.common.commands import DispatchResult, DockCmd, MoveCmd, RetreatCmd, RobotIsolated
from fleet_core.fleet_map import RETREAT, SHARED
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


# ---- 고정 경로에 붙지 못한 로봇 ----

def beside_the_node():
    """pinky2가 N_W에서 0.148 m 떨어져 서 있다. 관제의 허용치(0.15 m) 안이라 N_W에 있다고 본다."""
    world = Harness(starts={"pinky1": "P-1", "pinky2": "N_W", "pinky3": "P-3"}, speed=0.0)
    bot = world.bots["pinky2"]
    bot.x, bot.y = bot.x - 0.03, bot.y - 0.145
    world.report()
    call = world.call("WORK", "pinky2")
    world.step()
    [first] = world.sent(MoveCmd, "pinky2")
    assert (first.mode, first.target_id) == ("follow_path", "W"), "it is sent along the fixed line"
    return world, call


def test_a_robot_that_cannot_join_the_fixed_line_is_first_sent_to_its_start_by_nav_goal():
    world, call = beside_the_node()
    world.traffic.on_move_result("pinky2", last_goal(world), False, "PATH_NOT_CONNECTED")
    world.run(world.params.move_retry_wait_s + 0.3)
    again = world.sent(MoveCmd, "pinky2")[-1]
    assert (again.mode, again.target_id) == ("nav_goal", "N_W"), "not the same line once more"
    bot = world.bots["pinky2"]
    bot.x, bot.y = world.site.node_pose("N_W")
    world.report()
    world.traffic.on_move_result("pinky2", again.goal_id, True, "")
    world.run(2.0)
    line = world.sent(MoveCmd, "pinky2")[-1]
    assert (line.mode, line.target_id) == ("follow_path", "W"), "from the node it takes the line"
    assert world.codes("pinky2").count("GOAL_RETRY") == 1 and not isolations(world)
    assert world.result_of(call) is None
    assert world.view("pinky2")["attempts"] == 1, "kept until the line itself works"
    reach(world, "W", line.goal_id)
    assert world.view("pinky2")["attempts"] == 0


def reach(world, node, goal_id, robot="pinky2"):
    """로봇을 그 지점에 옮겨 놓고 goal의 성공을 넣는다."""
    bot = world.bots[robot]
    bot.x, bot.y = world.site.node_pose(node)
    world.report()
    world.traffic.on_move_result(robot, goal_id, True, "")
    world.run(world.params.arrival_settle_s + 0.5)


def test_a_line_that_works_after_the_robot_moved_by_itself_clears_the_count():
    """되돌아가는 goal 없이 로봇이 지점에 닿았어도, 그 뒤 경로가 되면 실패 횟수는 지워진다."""
    world, _ = beside_the_node()
    traffic, robot = world.traffic, world.traffic._robots["pinky2"]
    traffic.on_move_result("pinky2", last_goal(world), False, "PATH_NOT_CONNECTED")
    bot = world.bots["pinky2"]
    bot.x, bot.y = world.site.node_pose("N_W")
    world.report()
    world.run(world.params.move_retry_wait_s + 0.3)
    line = world.sent(MoveCmd, "pinky2")[-1]
    assert line.mode == "follow_path" and robot.attempts == 1
    reach(world, "W", line.goal_id)
    assert robot.attempts == 0, "an unrelated later failure starts from zero"


def test_joining_that_keeps_failing_although_the_nav_goal_succeeds_ends_in_isolation():
    """되돌아가는 Nav2 goal이 움직임 없이 성공해도 실패 횟수는 그 경로가 될 때까지 남는다."""
    world, call = beside_the_node()
    p = world.params
    for failure in range(1, p.move_retry_limit + 2):
        line = world.sent(MoveCmd, "pinky2")[-1]
        assert line.mode == "follow_path"
        world.traffic.on_move_result("pinky2", line.goal_id, False, "PATH_NOT_CONNECTED")
        if failure > p.move_retry_limit:
            break
        world.run(p.move_retry_wait_s + 0.3)
        back = world.sent(MoveCmd, "pinky2")[-1]
        assert back.mode == "nav_goal"
        world.traffic.on_move_result("pinky2", back.goal_id, True, "")  # 로봇은 그 자리 그대로다
        world.run(2.0)
        assert world.view("pinky2")["attempts"] == failure, "the count is kept"
    world.step()
    assert world.codes("pinky2")[-2:] == ["GOAL_FAILED", "ROBOT_ISOLATED"]
    assert world.result_of(call).reason == "PATH_NOT_CONNECTED"


def test_a_robot_that_reached_the_node_after_joining_failed_is_on_the_node_again():
    world, _ = beside_the_node()
    traffic, robot = world.traffic, world.traffic._robots["pinky2"]
    assert traffic._on_node(robot)
    traffic.on_move_result("pinky2", last_goal(world), False, "PATH_NOT_CONNECTED")
    assert not traffic._on_node(robot), "where joining failed it does not count as on the node"
    bot = world.bots["pinky2"]
    bot.x, bot.y = world.site.node_pose("N_W")  # goal은 끝나지 않았지만 몸은 지점에 닿았다
    world.report()
    assert traffic._on_node(robot), "a report right at the node is believed"


def test_a_robot_that_could_not_join_stays_off_the_node_while_its_report_wanders():
    """재시작 시험에서 나왔다: 서 있는 로봇의 보고가 0.06 m 흔들리자 관제가 다시 지점 위로 보고
    같은 고정 경로를 또 보냈다. 로봇은 또 거절했고, 세 번째에 격리됐다."""
    world, call = beside_the_node()
    world.traffic.on_move_result("pinky2", last_goal(world), False, "PATH_NOT_CONNECTED")
    bot = world.bots["pinky2"]
    bot.x += 0.06  # N_W에서 0.145 m: 여전히 관제의 허용치 안이다
    assert math.dist((bot.x, bot.y), world.site.node_pose("N_W")) < 0.15
    world.run(world.params.move_retry_wait_s + 0.3)
    again = world.sent(MoveCmd, "pinky2")[-1]
    assert (again.mode, again.target_id) == ("nav_goal", "N_W"), "not the same line once more"
    assert world.codes("pinky2").count("GOAL_RETRY") == 1 and world.result_of(call) is None


def test_a_robot_that_could_not_join_and_now_stands_in_the_junction_gets_the_line_again():
    """넓은 구역 밖에서는 Nav2 goal로 지점에 돌아갈 수 없다. 거기서도 좁은 기준을 쓰면 지점
    위도 아니고 길도 못 잡는 로봇이 된다: 갈림목 안의 로봇은 전처럼 0.15 m로 본다."""
    world, call = beside_the_node()
    world.traffic.on_move_result("pinky2", last_goal(world), False, "PATH_NOT_CONNECTED")
    bot = world.bots["pinky2"]
    x, y = world.site.node_pose("N_W")
    bot.x, bot.y = x, y + 0.145  # 손으로 옮겼다: N_W에서 X_W로 가는 선 위, 갈림목 안이다
    assert world.site.zones[world.site.zone_at(bot.x, bot.y)].kind != SHARED
    world.run(world.params.move_retry_wait_s + 0.3)
    sent = world.sent(MoveCmd, "pinky2")
    assert len(sent) == 2, "it is not left without a way"
    assert (sent[-1].mode, sent[-1].target_id) == ("follow_path", "W")
    assert world.result_of(call) is None


def test_a_robot_that_cannot_join_a_line_outside_a_shared_zone_is_still_retried_and_isolated():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "X_W", "pinky3": "P-3"}, speed=0.0)
    world.step()
    world.call("WORK", "pinky2")
    world.step()
    p = world.params
    for _ in range(p.move_retry_limit + 1):
        cmd = world.sent(MoveCmd, "pinky2")[-1]
        assert cmd.mode == "follow_path"
        world.traffic.on_move_result("pinky2", cmd.goal_id, False, "PATH_NOT_CONNECTED")
        world.run(p.move_retry_wait_s + 0.3)
    # 갈림목 안에서는 Nav2 goal로 돌아갈 수 없다: 전처럼 다시 보내다 격리한다
    assert world.view("pinky2")["state"] == "ERROR"
    assert world.codes("pinky2")[-2:] == ["GOAL_FAILED", "ROBOT_ISOLATED"]


# ---- 도킹 실패와 다시 보내기 ----

def at_the_dock():
    """움직이는 pinky2가 작업 도크 앞에 닿아 DockCmd를 받았다. 도킹은 실패로 답하게 해 둔다."""
    world = Harness()
    world.dock_fails.add("pinky2")
    world.step()
    call = world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.sent(DockCmd, "pinky2"), 120.0)
    return world, call


def docks(world, robot="pinky2"):
    return len(world.sent(DockCmd, robot))


@pytest.mark.parametrize("fail", ["result", "rejected"])
def test_a_failed_docking_is_sent_again_only_after_the_wait(fail):
    world, _ = at_the_dock()
    if fail == "rejected":
        world.traffic.on_goal_rejected("pinky2", world.sent(DockCmd, "pinky2")[-1].goal_id)
        world.step()
    else:
        assert world.run_until(lambda: "GOAL_RETRY" in world.codes("pinky2"), 30.0)
    assert (world.view("pinky2")["attempts"], docks(world)) == (1, 1)
    world.run(world.params.move_retry_wait_s - 0.3)
    assert docks(world) == 1, "nothing new before the retry wait is over"
    assert not world.sent(RetreatCmd, "pinky2")
    world.run(0.5)
    if fail == "rejected":
        assert docks(world) == 2, "a docking the robot did not take goes out again as it is"
    else:
        assert docks(world) == 1 and world.sent(RetreatCmd, "pinky2"), "it backs out first"


def test_a_failed_docking_backs_out_and_comes_in_again_before_the_next_try():
    """The reported pose may be off (B-G5). Driving the fixed line again lets the robot find its
    place before it docks at the same spot."""
    world, _ = at_the_dock()
    assert world.run_until(lambda: docks(world) == 2, 60.0)
    codes = world.codes("pinky2")
    first = codes.index("GOAL_RETRY")
    assert codes[first:first + 5] == ["GOAL_RETRY", "ZONE_GRANTED", "DOCK_REAPPROACH",
                                      "RETREAT_START", "RETREAT_DONE"], codes[first:first + 8]
    assert "FOLLOW_SENT" in codes[first:]
    assert not world.problems
    assert world.run_until(lambda: world.codes("pinky2").count("GOAL_RETRY") == 2, 60.0)
    retries = [e.detail for e in world.events if e.code == "GOAL_RETRY"]
    assert retries == ["DOCK_FAILED, attempt 1", "DOCK_FAILED, attempt 2"], "each one counts"


def way_out(world, robot="pinky2"):
    """The link a robot at its dock backs out on."""
    site = world.traffic.site
    dock = site.stations[world.traffic._robots[robot].dest].dock
    return next(link for link in site.links_from(dock) if link.via == RETREAT)


def test_a_reapproach_whose_way_is_taken_waits_and_goes_when_it_is_free():
    world, _ = at_the_dock()
    assert world.run_until(lambda: "GOAL_RETRY" in world.codes("pinky2"), 30.0)
    spot, grants = way_out(world).b, world.traffic.grants
    assert grants.take("ghost", (), (spot,))
    world.run(world.params.move_retry_wait_s + 1.0)
    assert docks(world) == 1 and not world.sent(RetreatCmd, "pinky2"), "it waits at the dock"
    assert world.traffic.status_view("pinky2")["wait_robots"] == ("ghost",), "the wait is on record"
    grants.release_node("ghost", spot)
    world.run(1.0)
    assert world.sent(RetreatCmd, "pinky2") and docks(world) == 1


def test_a_reapproach_that_cannot_start_in_time_docks_where_it_stands():
    """Whoever holds the way out may never leave. The robot does not wait at the dock for ever."""
    world, _ = at_the_dock()
    assert world.run_until(lambda: "GOAL_RETRY" in world.codes("pinky2"), 30.0)
    assert world.traffic.grants.take("ghost", (), (way_out(world).b,))
    world.run(world.params.move_retry_wait_s + world.params.redock_wait_s - 0.5)
    assert docks(world) == 1
    world.run(1.0)
    assert docks(world) == 2 and not world.sent(RetreatCmd, "pinky2")


def test_a_reapproaching_robot_keeps_its_dock_turn():
    world, _ = at_the_dock()
    world.call("WORK", "pinky3")
    assert world.run_until(lambda: "DOCK_REAPPROACH" in world.codes("pinky2"), 30.0)
    assert world.traffic._robots["pinky2"].trip.turn
    assert world.run_until(lambda: docks(world) == 2, 60.0)
    assert not docks(world, "pinky3"), "the other robot does not get the dock in between"


def test_a_rejected_docking_and_failed_ones_count_together():
    world, _ = at_the_dock()
    world.traffic.on_goal_rejected("pinky2", world.sent(DockCmd, "pinky2")[-1].goal_id)
    assert world.run_until(lambda: isolations(world), 120.0)
    told = [e.detail for e in world.events if e.code in ("GOAL_RETRY", "GOAL_FAILED")]
    assert told == ["REJECTED, attempt 1", "DOCK_FAILED, attempt 2",
                    "DOCK_FAILED after 3 attempts"]


def test_a_rejected_docking_after_a_reapproach_counts_too():
    world, _ = at_the_dock()
    assert world.run_until(lambda: docks(world) == 2, 60.0)
    world.traffic.on_goal_rejected("pinky2", world.sent(DockCmd, "pinky2")[-1].goal_id)
    world.step()
    told = [e.detail for e in world.events if e.code == "GOAL_RETRY"]
    assert told == ["DOCK_FAILED, attempt 1", "REJECTED, attempt 2"]


def test_docking_that_fails_once_and_then_succeeds_ends_the_dispatch_well():
    world, call = at_the_dock()
    assert world.run_until(lambda: "GOAL_RETRY" in world.codes("pinky2"), 30.0)
    world.dock_fails.clear()
    assert world.run_until(lambda: world.result_of(call), 60.0)
    assert world.result_of(call).success and "GOAL_FAILED" not in world.codes()
    assert (world.view("pinky2")["state"], world.view("pinky2")["attempts"]) == ("AT_STATION", 0)


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
