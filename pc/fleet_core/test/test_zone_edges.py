"""구역 교통 층의 가장자리 경우: 취소와 놓는 순서, 달리는 중의 목적 바뀜, 복구 중 다시 끊김,
후진 결과, 교착 회피 다시 하기, 후진 중 막힘."""
import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, RetreatCmd
from fleet_core.traffic.mission import MOVE
from fleet_core.traffic.zone.lane_policy import Want
from fleet_core.traffic.zone.robots import Goal
from zone_harness import Harness


def reported(starts):
    world = Harness(starts=starts)
    world.report()
    return world


def targets(world, robot):
    return [(c.mode, c.target_id) for c in world.sent(MoveCmd, robot)]


def test_zones_ahead_are_given_back_only_after_reports_confirm_the_robot_is_clear():
    world = Harness()
    world.step()
    call = world.call("WORK", "pinky2")
    world.run(1.0)
    assert world.view("pinky2")["held"] == ["HALL", "J_W", "WORK_DOCK"]
    world.traffic.on_dispatch_cancel(call)
    world.step()
    assert world.sent(CancelCmd, "pinky2"), "the nav goal is cancelled"
    assert world.view("pinky2")["held"] == ["HALL", "J_W", "WORK_DOCK"], \
        "the cancel may not have reached the robot yet"
    world.run(world.params.arrival_settle_s - 0.2)
    assert world.view("pinky2")["held"] == ["HALL", "J_W", "WORK_DOCK"], "still settling"
    world.run(1.0)
    assert world.view("pinky2")["held"] == ["HALL"]
    assert not world.problems


def test_a_purpose_changed_and_changed_back_still_arrives_at_the_dock():
    world = Harness()
    world.step()
    first = world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.view("pinky2")["target"] == "W", 60.0)
    world.run(0.5)  # 도크로 들어가는 고정 경로 위다
    world.traffic.on_dispatch_cancel(first)
    world.step()
    second = world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.result_of(second), 60.0)
    assert world.result_of(second).success
    assert targets(world, "pinky2") == [("nav_goal", "N_W"), ("follow_path", "W")]
    assert "DOCK_QUEUED" not in world.codes("pinky2") and not world.problems


def test_a_robot_carried_into_the_yard_takes_the_yard_and_gives_the_dock_back():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.traffic.on_robot_state("pinky1", 1.0, 0.40, 0.0, "IDLE")  # 관제가 돌기 전에 들어 옮겼다
    samples = world.params.outside_zone_samples + world.params.release_confirm_samples + 2
    for _ in range(samples):
        world.traffic.on_robot_state("pinky1", 1.0, 0.40, 0.0, "IDLE")
    codes = [e.code for e in world.traffic.take_output()]
    assert codes == ["OUTSIDE_ZONE", "OUTSIDE_ZONE"], "seen, then it takes the zone it stands in"
    view = world.view("pinky1")
    assert view["held"] == ["HALL"] and view["nodes"] == []


def test_a_purpose_changed_during_a_fixed_stretch_switches_at_the_next_node():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    call = world.call("OUTBOUND", "pinky1")
    world.run(2.0)
    assert world.view("pinky1")["held"] == ["WORK_DOCK", "J_W", "EXPRESS", "J_E", "OUT_DOCK"]
    world.traffic.on_dispatch_cancel(call)
    world.step()
    assert not world.sent(CancelCmd), "it finishes backing out of the dock"
    assert world.run_until(lambda: world.phase("pinky1") == "PARKED", 120.0)
    assert targets(world, "pinky1") == [("follow_path", "N_W"), ("nav_goal", "P-1")]
    assert world.view("pinky1")["held"] == ["HALL"]
    assert world.traffic.grants.holders("EXPRESS") == ()
    assert not world.problems


def test_a_robot_that_goes_silent_again_while_recovering_gets_no_command():
    world = Harness()
    world.step()
    world.call("WORK")
    world.run(2.0)
    world.silent.add("pinky2")
    world.run(world.params.comm_timeout_s + 1.0)
    assert world.view("pinky2")["state"] == "COMM_LOST"
    sent = len(world.sent(MoveCmd, "pinky2"))
    world.silent.discard("pinky2")
    world.step()  # 보고 한 번
    assert world.view("pinky2")["state"] == "RECOVERING"
    world.silent.add("pinky2")
    world.run(world.params.recover_check_s + 1.0)
    assert world.view("pinky2")["state"] != "TO_WORK", "nothing new was heard from it"
    assert len(world.sent(MoveCmd, "pinky2")) == sent
    world.run(world.params.comm_timeout_s)
    assert world.view("pinky2")["state"] == "COMM_LOST"
    assert world.codes("pinky2").count("COMM_LOST") == 2


def test_a_retreat_success_is_checked_against_the_reported_pose():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.call("OUTBOUND", "pinky1")
    world.run(1.0)
    goal_id = world.sent(RetreatCmd, "pinky1")[0].goal_id
    world.traffic.on_retreat_result("pinky1", goal_id, True, "")  # 아직 5 cm밖에 못 물러났다
    world.step()
    view = world.view("pinky1")
    assert view["node"] == "W" and view["goal_id"] == goal_id, "it is not at X_W yet"
    assert not world.sent(MoveCmd, "pinky1")


@pytest.mark.parametrize("busy", ["pinky1", "pinky2"])
def test_when_one_robot_of_a_cycle_cannot_give_way_the_other_does(busy):
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-3"})
    world.step()
    traffic = world.traffic
    for name, node in (("pinky1", "P-3"), ("pinky2", "P-1")):
        traffic._wants[name] = Want((), (node,), {}, 0.0)
    traffic._robots[busy].goal = Goal(MOVE, 9000, 0, "P-2", "nav_goal")
    world.run(world.params.deadlock_grace_s + 0.5)
    free = "pinky2" if busy == "pinky1" else "pinky1"
    assert targets(world, free) == [("nav_goal", "P-2")]


def test_a_deadlock_evasion_that_could_not_start_is_tried_again():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-3"})
    world.step()
    traffic = world.traffic
    for name, node in (("pinky1", "P-3"), ("pinky2", "P-1")):
        traffic._wants[name] = Want((), (node,), {}, 0.0)
        traffic._robots[name].goal = Goal(MOVE, 9000, 0, node, "nav_goal")  # 달리는 중이다
    world.run(world.params.deadlock_grace_s + 0.5)
    assert world.codes().count("DEADLOCK_SEEN") == 1, "seen, nothing could be done yet"
    for name in ("pinky1", "pinky2"):
        traffic._robots[name].goal = None
    world.step()
    assert world.codes().count("DEADLOCK_SEEN") == 2, "now one of them gives way"
    assert targets(world, "pinky1") + targets(world, "pinky2") == [("nav_goal", "P-2")]


def test_a_robot_blocked_while_backing_out_waits_and_then_fails():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.call("OUTBOUND", "pinky1")
    world.obstacles.append((0.34, 0.89, 0.08))
    limit = world.params.blocked_timeout_s
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pinky1"), 30.0)
    world.run(limit - 5.0)
    assert world.view("pinky1")["state"] == "TO_OUTBOUND"
    world.run(10.0)
    view = world.view("pinky1")
    assert (view["state"], view["fault"]) == ("ERROR", "ERROR")
    assert "ZONE_MARKED_BLOCKED" not in world.codes("pinky1"), "there is no other way out"


# ---- 세계 시험이 찾은 것(B1, B2, B3) ----

def test_a_nav_goal_keeps_every_zone_it_may_cross_until_it_ends():
    """Nav2는 잡아 준 통로 안에서 어느 길로든 돌아올 수 있다. 지나온 넓은 구역도 goal이 끝날
    때까지 쥔다."""
    world = Harness("d", starts={"pinky1": "W", "pinky2": "P-1", "pinky3": "P-3"})
    world.report()
    world.call("WORK", "pinky1")  # 도크에 서 있다. 곧바로 도킹하고 도크를 쥔다
    call = world.call("WORK", "pinky3")  # 도크가 찼다. 서쪽 마당의 대기 자리로 건너온다
    world.step()
    bot = world.bots["pinky3"]
    assert world.run_until(lambda: 0.9 < bot.x < 1.0, 60.0)
    assert world.site.zone_at(bot.x, bot.y) in ("CORR_M", "CORR_S"), "it crosses by a corridor"
    assert world.sent(MoveCmd, "pinky3")[-1].mode == "nav_goal"
    assert "HALL_E" in world.view("pinky3")["held"], "the yard behind it is not given back"
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(call) is not None, 200.0)
    assert world.result_of(call).success and not world.problems
    assert "HALL_E" not in world.view("pinky3")["held"], "given back after the goal ended"


def test_a_robot_that_nav2_took_into_another_yard_zone_takes_that_zone():
    world = Harness("d")
    world.report()
    world.call("WORK", "pinky1")
    world.step()
    assert world.sent(MoveCmd, "pinky1")[-1].mode == "nav_goal"
    assert "CORR_S" not in world.view("pinky1")["held"]
    world.traffic.take_output()
    for _ in range(world.params.outside_zone_samples + 1):
        world.traffic.on_robot_state("pinky1", 0.75, 0.20, 0.0, "MOVING")
    codes = [e.code for e in world.traffic.take_output()]
    assert codes == ["OUTSIDE_ZONE", "OUTSIDE_ZONE"], "seen, then it takes the zone it is in"
    assert "CORR_S" in world.view("pinky1")["held"]
    assert world.view("pinky1")["goal_id"] is not None, "the goal goes on"


def test_the_status_names_the_nodes_ahead_that_are_granted_and_those_that_are_not():
    world = Harness()
    world.step()
    world.call("WORK")
    world.step()
    status, view = world.traffic.status_view("pinky2"), world.view("pinky2")
    assert status["base"] == tuple(view["base"]) and status["base"][-1] == "W"
    assert status["horizon"] == ()
    assert world.traffic.status_view("pinky1")["base"] == (), "a standing robot has no way ahead"


def test_the_status_of_an_unknown_robot_is_empty():
    world = Harness()
    assert world.traffic.status_view("ghost") == {}


def test_an_isolated_robot_gives_back_the_zones_it_never_reached():
    world = Harness()
    world.step()
    world.call("WORK", "pinky1")
    world.run(1.0)
    assert world.view("pinky1")["held"] == ["HALL", "J_W", "WORK_DOCK"]
    world.traffic.on_robot_fault("pinky1", "MOTOR")
    world.run(2.0)
    assert world.view("pinky1")["held"] == ["HALL"], "it stands in the yard"
    returned = [e.detail for e in world.events if e.code == "ZONES_RETURNED"]
    assert returned == ["gives back ['J_W', 'WORK_DOCK'] it did not reach, keeps ['HALL']"]
    other = world.call("WORK")
    assert world.run_until(lambda: world.result_of(other) is not None, 60.0)
    assert world.result_of(other).success, "the others still serve the work dock"


def test_a_robot_isolated_inside_the_junction_keeps_the_junction():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.call("OUTBOUND", "pinky1")
    bot = world.bots["pinky1"]
    assert world.run_until(lambda: bot.x > 0.36, 30.0)  # 후진으로 갈림목 안에 들어섰다
    world.traffic.on_robot_fault("pinky1", "MOTOR")
    world.run(2.0)
    held = world.view("pinky1")["held"]
    assert "J_W" in held, "it stands there"
    assert "J_E" not in held and "OUT_DOCK" not in held, "the far end of the trip goes back"
    assert set(held) <= {"WORK_DOCK", "J_W", "EXPRESS"}, "at most what it stands next to"


def test_a_silent_robot_gives_nothing_back():
    world = Harness()
    world.step()
    world.call("WORK", "pinky1")
    world.run(1.0)
    world.silent.add("pinky1")
    world.run(10.0)
    assert world.view("pinky1")["comm"] == "COMM_LOST"
    assert world.view("pinky1")["held"] == ["HALL", "J_W", "WORK_DOCK"]


def test_a_full_yard_zone_is_taken_as_soon_as_there_is_room():
    world = Harness("d")
    world.report()
    world.call("WORK", "pinky1")
    world.step()
    grants = world.traffic.grants
    assert grants.force("pinky2", ["CORR_S"], [])  # 통로는 한 대만 든다
    world.traffic.take_output()
    for _ in range(world.params.outside_zone_samples + 3):
        world.traffic.on_robot_state("pinky1", 0.75, 0.20, 0.0, "MOVING")
    assert "CORR_S" not in world.view("pinky1")["held"], "no room yet"
    grants.release_zone("pinky2", "CORR_S")
    world.traffic.on_robot_state("pinky1", 0.75, 0.20, 0.0, "MOVING")
    assert "CORR_S" in world.view("pinky1")["held"]
    codes = [e.code for e in world.traffic.take_output()]
    assert codes == ["OUTSIDE_ZONE", "OUTSIDE_ZONE"], "noted once, then taken"


def test_the_reports_that_let_go_of_a_zone_must_follow_each_other():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    assert world.traffic.grants.force("pinky1", ["HALL"], [])
    yard, elsewhere = (1.0, 0.40), (1.0, 0.89)  # 마당 안, 쥐지 않은 급행 안
    for _ in range(world.params.release_confirm_samples - 1):
        world.traffic.on_robot_state("pinky1", *yard, 0.0, "IDLE")
    world.traffic.on_robot_state("pinky1", *elsewhere, 0.0, "IDLE")  # 위치가 튀었다
    world.traffic.on_robot_state("pinky1", *yard, 0.0, "IDLE")
    assert "WORK_DOCK" in world.view("pinky1")["held"], "the count starts again after the jump"
    for _ in range(world.params.release_confirm_samples):
        world.traffic.on_robot_state("pinky1", *yard, 0.0, "IDLE")
    assert "WORK_DOCK" not in world.view("pinky1")["held"]


def test_a_report_without_a_position_restarts_the_count_too():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    assert world.traffic.grants.force("pinky1", ["HALL"], [])
    for _ in range(world.params.release_confirm_samples - 1):
        world.traffic.on_robot_state("pinky1", 1.0, 0.40, 0.0, "IDLE")
    world.traffic.on_robot_state("pinky1", 1.0, 0.40, 0.0, "LOCALIZING")
    world.traffic.on_robot_state("pinky1", 1.0, 0.40, 0.0, "IDLE")
    assert "WORK_DOCK" in world.view("pinky1")["held"]


def isolated_in_the_yard(reason="MOTOR"):
    world = Harness()
    world.step()
    world.call("WORK", "pinky1")
    world.run(1.0)
    world.traffic.on_robot_fault("pinky1", reason)
    world.step()
    world.now += 2.0  # 취소가 닿을 시간은 지났다
    return world, world.bots["pinky1"]


@pytest.mark.parametrize("state", ["MOVING", "BLOCKED", "RECOVERING", "LOCALIZING"])
def test_an_isolated_robot_that_still_reports_a_goal_gives_nothing_back(state):
    world, bot = isolated_in_the_yard()
    for _ in range(6):
        world.traffic.on_robot_state("pinky1", bot.x, bot.y, 0.0, state)
    assert world.view("pinky1")["held"] == ["HALL", "J_W", "WORK_DOCK"], "the cancel has not landed"
    for _ in range(6):
        world.traffic.on_robot_state("pinky1", bot.x, bot.y, 0.0, "IDLE")
    assert world.view("pinky1")["held"] == ["HALL"]


def test_an_isolated_robot_whose_report_moved_gives_nothing_back():
    world, bot = isolated_in_the_yard()
    for _ in range(6):
        world.traffic.on_robot_state("pinky1", bot.x + 0.30, bot.y, 0.0, "IDLE")
    assert world.view("pinky1")["held"] == ["HALL", "J_W", "WORK_DOCK"], "it is not where it failed"


@pytest.mark.parametrize("later", ["", "MOTOR"])
def test_a_robot_isolated_for_not_being_where_it_says_gives_nothing_back(later):
    world, bot = isolated_in_the_yard("NOT_AT_TARGET")
    if later:
        world.traffic.on_robot_fault("pinky1", later)  # 다른 고장이 뒤따라도 위치는 못 믿는다
    for _ in range(6):
        world.traffic.on_robot_state("pinky1", bot.x, bot.y, 0.0, "IDLE")
    assert world.view("pinky1")["held"] == ["HALL", "J_W", "WORK_DOCK"], "its position is in doubt"


def test_the_start_up_wait_counts_from_the_first_tick_so_a_record_plays_back():
    """재생은 시계가 0일 때 코어를 만들고 기록의 시각으로 건너뛴다. 만든 시각에서 세면 첫 tick에
    ROBOT_NOT_REPORTING이 나와 기록과 달라진다(world의 B4)."""
    world = Harness()
    world.silent = set(world.bots)
    world.now = 1_000_000.0  # 코어를 만든 뒤에 시계가 크게 뛴다
    world.step()
    assert "ROBOT_NOT_REPORTING" not in world.codes()
    world.run(world.params.startup_wait_s + 1.0)
    assert world.codes().count("ROBOT_NOT_REPORTING") == 1
