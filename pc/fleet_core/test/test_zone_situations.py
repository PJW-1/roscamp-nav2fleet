"""구역 교통 층의 이상 상황: 도크 맞바꿈, 막힘, 통신 끊김, 치움과 다시 투입, 멈춤, 위치 튐."""
from fleet_core.common.commands import CancelCmd, MoveCmd, RetreatCmd
from fleet_core.traffic.mission import RUN, STOP
from zone_harness import Harness


def reported(starts):
    """로봇들이 지도에 올랐고 관제는 아직 한 번도 돌지 않았다."""
    world = Harness(starts=starts)
    world.report()
    return world


def from_work_to_outbound():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    return world, world.call("OUTBOUND", "pinky1")


def targets(world, robot):
    return [(c.mode, c.target_id) for c in world.sent(MoveCmd, robot)]


# ---- 도크 맞바꿈 ----

def test_two_robots_swapping_docks_pass_through_the_wait_spots():
    world = reported({"pinky1": "W", "pinky2": "O", "pinky3": "P-3"})
    east, west = world.call("OUTBOUND", "pinky1"), world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.result_of(east) and world.result_of(west), 120.0)
    assert world.result_of(east).success and world.result_of(west).success
    # W에서는 마당의 N_W까지 물러난 뒤 대기 자리로 간다(2-1: 갈림목 안에서 돌지 않는다)
    assert targets(world, "pinky1") == [
        ("nav_goal", "O_Q"), ("nav_goal", "N_E"), ("follow_path", "O")]
    assert world.codes().count("DOCK_QUEUED") == 2
    assert "DEADLOCK_SEEN" not in world.codes() and not world.problems


# ---- 막힘 ----

def test_blocked_inside_the_express_it_backs_out_and_takes_the_yard():
    world, call = from_work_to_outbound()
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success
    seen = [code for code in world.codes("pinky1") if code in (
        "BLOCKED_WAIT", "ZONE_MARKED_BLOCKED", "REROUTE", "RETREAT_START", "RETREAT_DONE")]
    assert seen == ["RETREAT_START", "RETREAT_DONE", "BLOCKED_WAIT", "ZONE_MARKED_BLOCKED",
                    "REROUTE", "RETREAT_START", "RETREAT_DONE"]
    back = [point[:2] for point in world.sent(RetreatCmd, "pinky1")[1].path]
    site = world.site
    assert back[-1] == site.node_pose("N_W"), "through the junction it entered by, to the yard"
    assert site.node_pose("X_W") in back
    assert targets(world, "pinky1") == [
        ("follow_path", "O"), ("nav_goal", "N_E"), ("follow_path", "O")]
    assert not world.problems


def test_a_short_block_is_only_waited_out():
    world, call = from_work_to_outbound()
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pinky1"), 60.0)
    world.run(8.0)  # 사람이 잠깐 지나간다
    world.obstacles.clear()
    assert world.run_until(lambda: world.result_of(call), 120.0)
    codes = world.codes("pinky1")
    assert "ZONE_MARKED_BLOCKED" not in codes and "REROUTE" not in codes
    assert len(world.sent(RetreatCmd, "pinky1")) == 1, "only the way out of the dock"
    assert not world.sent(CancelCmd)


def test_blocked_in_the_yard_it_goes_by_another_node():
    world = Harness(starts={"pinky1": "P-3"})
    world.step()
    call = world.call("WORK")
    world.run(1.0)
    assert world.view("pinky1")["target"] == "N_W"
    world.obstacles.append((1.1, 0.42, 0.12))
    assert world.run_until(lambda: world.result_of(call), 200.0)
    assert world.result_of(call).success
    codes = world.codes("pinky1")
    assert "ZONE_MARKED_BLOCKED" in codes and "REROUTE" in codes
    assert not world.sent(RetreatCmd), "a nav goal is cancelled, nothing backs up"
    assert not world.problems


def test_with_the_dock_entry_blocked_it_comes_round_through_the_express():
    world = Harness(starts={"pinky1": "P-2"})
    world.step()
    call = world.call("WORK")
    world.obstacles.append((0.35, 0.74, 0.05))
    assert world.run_until(lambda: world.result_of(call), 200.0)
    assert world.result_of(call).success
    assert targets(world, "pinky1") == [
        ("nav_goal", "N_W"), ("follow_path", "W"), ("nav_goal", "N_E"), ("follow_path", "W")]
    assert world.traffic.lanes.last == {"EXPRESS": "X_E"}
    assert not world.problems


def test_with_no_other_way_it_waits_at_a_wait_spot_until_the_mark_expires():
    world = Harness(starts={"pinky1": "P-2"})
    world.step()
    call = world.call("WORK")
    world.obstacles += [(0.35, 0.74, 0.05), (1.0, 0.89, 0.15)]
    assert world.run_until(lambda: world.codes("pinky1").count("ZONE_MARKED_BLOCKED") == 2, 200.0)
    assert world.run_until(lambda: world.view("pinky1")["node"] == "W_Q"
                           and world.view("pinky1")["goal_id"] is None, 100.0)
    assert world.view("pinky1")["held"] == ["HALL"], "it does not stand inside the express"
    world.obstacles.clear()
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success
    assert not [e for e in world.events if e.level == "ERROR"] and not world.problems


# ---- 통신 끊김 ----

def test_a_silent_robot_keeps_its_zones_and_goes_on_when_it_is_back():
    world = Harness()
    world.step()
    call = world.call("WORK")
    world.run(2.0)
    held = world.view("pinky2")["held"]
    world.silent.add("pinky2")
    world.run(world.params.comm_timeout_s + 1.0)
    view = world.view("pinky2")
    assert (view["state"], view["held"]) == ("COMM_LOST", held)
    assert world.view("pinky1")["state"] == "TO_WORK", "another robot answers the call"
    assert world.view("pinky1")["node"] in ("P-1", "W_Q") and world.result_of(call) is None
    world.silent.discard("pinky2")
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).robot_id == "pinky2"
    codes = world.codes("pinky2")
    assert codes.index("COMM_LOST") < codes.index("COMM_RESTORED")
    assert not world.problems


# ---- 운영자가 치움, 다시 투입 ----

def test_a_removed_robot_gives_its_zones_back_and_another_takes_the_call():
    world = Harness()
    world.step()
    call = world.call("WORK")
    world.run(3.0)
    world.traffic.on_robot_fault("pinky2", "MOTOR")
    world.run(1.0)
    view = world.view("pinky2")
    assert view["state"] == "ERROR" and view["held"] == ["HALL", "J_W", "WORK_DOCK"]
    assert world.sent(CancelCmd, "pinky2")
    assert world.traffic.confirm_clear("pinky2", [], "moved by hand") is None, "zones not listed"
    assert world.traffic.confirm_clear("pinky2", view["held"], "moved by hand") == view["held"]
    world.silent.add("pinky2")  # 손으로 들어냈다
    assert world.view("pinky2")["state"] == "OUT_OF_SERVICE"
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).robot_id == "pinky1"
    assert not world.problems


def test_a_reinstated_robot_is_placed_by_its_next_report():
    world = Harness()
    world.step()
    world.traffic.on_robot_fault("pinky3", "MOTOR")
    world.step()
    assert world.traffic.reinstate_refusal("pinky3") == "pinky3 is ERROR, not OUT_OF_SERVICE"
    assert world.traffic.confirm_clear("pinky3", ["HALL"], "checked") == ["HALL"]
    assert world.traffic.reinstate("pinky3")
    assert world.view("pinky3")["on_map"] is False
    world.step()
    view = world.view("pinky3")
    assert (view["state"], view["node"], view["held"]) == ("PARKED", "P-3", ["HALL"])


# ---- 멈춤 ----

def test_stop_cancels_the_goals_and_run_resumes_them():
    world = Harness()
    world.step()
    call = world.call("WORK")
    world.run(2.0)
    world.traffic.on_run_mode(STOP)
    world.step()
    assert [c.robot for c in world.sent(CancelCmd)] == ["pinky2"]
    sent = len(world.commands)
    world.run(5.0)
    assert len(world.commands) == sent and world.result_of(call) is None
    assert world.view("pinky2")["held"] == ["HALL", "J_W", "WORK_DOCK"], "its zones stay"
    world.traffic.on_run_mode(RUN)
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success and not world.problems


# ---- 도킹 실패, 위치 튐 ----

def test_docking_that_keeps_failing_ends_in_error():
    world = Harness()
    world.dock_fails.add("pinky2")
    world.step()
    call = world.call("WORK")
    assert world.run_until(lambda: world.view("pinky2")["state"] == "ERROR", 200.0)
    codes = world.codes("pinky2")
    assert codes.count("GOAL_RETRY") == world.params.move_retry_limit
    assert codes[-2:] == ["GOAL_FAILED", "ROBOT_ISOLATED"]
    assert world.result_of(call) is None, "the open call stays for another robot"


def test_a_pose_that_jumps_out_of_its_zones_is_recorded_and_releases_nothing():
    world = Harness()
    world.step()
    for _ in range(world.params.outside_zone_samples + 1):
        world.traffic.on_robot_state("pinky3", 1.0, 0.89, 0.0, "IDLE")
    events = world.traffic.take_output()
    assert [e.code for e in events] == ["OUTSIDE_ZONE"]
    view = world.view("pinky3")
    assert view["held"] == ["HALL"] and view["nodes"] == ["P-3"] and view["state"] == "PARKED"
