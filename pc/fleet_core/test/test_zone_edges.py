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
