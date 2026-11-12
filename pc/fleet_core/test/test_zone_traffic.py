"""구역 교통 층의 평소 흐름: 지도에 오르기, 호출, 도크, 급행, 주차, 도크 줄."""
from fleet_core.common.commands import DockCmd, MoveCmd, RetreatCmd
from zone_harness import Harness


def placed(world):
    world.step()
    return world


def test_robots_are_placed_on_the_nodes_they_report_from():
    world = placed(Harness())
    for robot, node in (("pinky1", "P-1"), ("pinky2", "P-2"), ("pinky3", "P-3")):
        view = world.view(robot)
        assert (view["state"], view["node"], view["held"], view["nodes"]) == (
            "PARKED", node, ["HALL"], [node])
    assert world.traffic.everyone_placed
    assert world.traffic.grants.holders("HALL") == ("pinky1", "pinky2", "pinky3")
    assert not world.commands, "nobody moves without a call"


def test_an_open_call_brings_the_nearest_robot_to_the_work_dock():
    world = placed(Harness())
    call = world.call("WORK")
    assert world.run_until(lambda: world.result_of(call) is not None)
    result = world.result_of(call)
    assert (result.success, result.robot_id) == (True, "pinky2"), "P-2 is nearest to the dock"
    moves = world.sent(MoveCmd, "pinky2")
    assert [(c.mode, c.target_id) for c in moves] == [("nav_goal", "N_W"), ("follow_path", "W")]
    assert [(x, y) for x, y, _ in moves[1].path] == [(0.40, 0.65), (0.40, 0.89), (0.18, 0.89)]
    assert [c.dock_id for c in world.sent(DockCmd)] == ["WORK"]
    codes = world.codes("pinky2")
    assert [c for c in codes if c in ("ZONE_GRANTED", "NAV_GOAL_SENT", "FOLLOW_SENT", "DOCKED")] \
        == ["ZONE_GRANTED", "NAV_GOAL_SENT", "FOLLOW_SENT", "DOCKED"]
    world.run(3.0)
    view = world.view("pinky2")
    assert (view["state"], view["held"], view["nodes"]) == ("AT_STATION", ["WORK_DOCK"], ["W"])
    assert not world.problems


def test_the_dock_stretch_is_taken_whole_before_the_robot_leaves_the_parking_spot():
    world = placed(Harness())
    world.call("WORK")
    world.step()
    view = world.view("pinky2")
    assert view["held"] == ["HALL", "J_W", "WORK_DOCK"]
    assert view["nodes"] == ["P-2", "N_W", "X_W", "W"]
    assert view["route"] == ["N_W", "X_W", "W"] == view["base"] and view["horizon"] == []


def test_work_to_outbound_runs_the_express_and_backs_out_of_the_dock_first():
    world = placed(Harness())
    call = world.call("WORK")
    assert world.run_until(lambda: world.result_of(call) is not None)
    out = world.call("OUTBOUND", "pinky2")
    world.step()
    # 도킹 바로 뒤라 다가오며 지난 HALL을 아직 쥐고 있다: 도크에서 놓는 것은 보고 세 번 뒤다
    assert world.view("pinky2")["held"] == [
        "HALL", "J_W", "WORK_DOCK", "EXPRESS", "J_E", "OUT_DOCK"]
    assert world.traffic.grants.lane_direction("EXPRESS") == "X_W"
    assert world.run_until(lambda: world.result_of(out) is not None)
    assert world.result_of(out).success
    retreat = world.sent(RetreatCmd, "pinky2")
    assert len(retreat) == 1 and [(x, y) for x, y, _ in retreat[0].path] == [
        (0.18, 0.89), (0.40, 0.89)]
    express = world.sent(MoveCmd, "pinky2")[-1]
    assert [(x, y) for x, y, _ in express.path] == [(0.40, 0.89), (1.58, 0.89), (1.80, 0.89)]
    codes = world.codes("pinky2")
    assert "LANE_DIRECTION_SET" in codes and "RETREAT_START" in codes and "RETREAT_DONE" in codes
    world.run(3.0)
    assert world.view("pinky2")["held"] == ["OUT_DOCK"], "everything behind it is given back"
    assert world.traffic.grants.lane_direction("EXPRESS") == ""
    assert not world.problems


def test_a_released_robot_goes_to_a_free_parking_spot():
    world = placed(Harness())
    call = world.call("WORK")
    assert world.run_until(lambda: world.result_of(call) is not None)
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.phase("pinky2") == "PARKED")
    view = world.view("pinky2")
    assert view["node"] == "P-2" and view["held"] == ["HALL"], "the nearest free spot"
    assert not world.problems


def test_a_second_robot_waits_at_the_wait_spot_while_the_dock_is_busy():
    world = placed(Harness())
    first, second = world.call("WORK"), world.call("WORK")
    assert world.run_until(lambda: world.result_of(first) is not None)
    assert world.run_until(lambda: "DOCK_QUEUED" in world.codes("pinky1"))
    assert world.run_until(lambda: world.view("pinky1")["node"] == "W_Q"
                           and world.view("pinky1")["goal_id"] is None)
    world.run(5.0)
    assert world.view("pinky1")["node"] == "W_Q" and world.result_of(second) is None
    assert "WORK_DOCK" not in world.view("pinky1")["held"], "it never stands in the dock zones"
    world.call("OUTBOUND", "pinky2")
    assert world.run_until(lambda: world.result_of(second) is not None)
    assert world.result_of(second).robot_id == "pinky1"
    assert not world.problems


def test_three_robots_serve_calls_one_after_another_without_a_fault():
    world = placed(Harness())
    calls = [world.call("WORK") for _ in range(3)]
    served = set()

    def serve():
        for result in world.results:
            if result.goal_id in calls and result.success and result.robot_id not in served:
                served.add(result.robot_id)
                world.call("OUTBOUND", result.robot_id)
        for robot in world.bots:
            view = world.view(robot)
            if view["state"] == "AT_STATION" and view["destination"] == "OUTBOUND":
                world.traffic.on_release(robot)
        return len(served) == 3 and all(world.phase(r) == "PARKED" for r in world.bots)

    assert world.run_until(serve, timeout_s=600.0)
    assert not world.problems
    assert not [e for e in world.events if e.level == "ERROR"]
    assert "DEADLOCK_SEEN" not in world.codes()
