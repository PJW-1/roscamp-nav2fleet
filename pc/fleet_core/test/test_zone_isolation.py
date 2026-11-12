"""구역 교통 층에서 로봇을 세우는 일: 지도에 못 올림(PLACEMENT_CONFLICT), 고장으로 세움(한 번만),
세운 로봇이 쥔 대기 자리, 말이 없던 동안 지난 지점 따라잡기."""
from fleet_core.common.commands import CancelCmd, DispatchResult, DockCmd, MoveCmd, RobotIsolated
from zone_harness import Harness


def reported(starts, **kwargs):
    """로봇들이 지도에 올랐고 관제는 아직 한 번도 돌지 않았다."""
    world = Harness(starts=starts, **kwargs)
    world.report()
    return world


def targets(world, robot):
    return [(c.mode, c.target_id) for c in world.sent(MoveCmd, robot)]


def isolations(world, robot):
    return [c for c in world.commands if isinstance(c, RobotIsolated) and c.robot == robot]


# ---- 지도에 올리기 ----

def test_a_robot_reporting_from_a_taken_node_is_refused_once_and_placed_where_it_is_free():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-1", "pinky3": "P-3"})
    for _ in range(3):
        world.step()
    view = world.view("pinky2")
    assert (view["on_map"], view["state"], view["held"]) == (False, "ERROR", [])
    conflicts = [e for e in world.events if e.code == "PLACEMENT_CONFLICT"]
    assert [(e.level, e.robot) for e in conflicts] == [("ERROR", "pinky2")], "said once"
    assert world.traffic.status_view("pinky2")["fault_reason"] == "PLACEMENT_CONFLICT"
    world.bots["pinky2"].x, world.bots["pinky2"].y = world.site.node_pose("P-2")  # 손으로 옮겼다
    world.step()
    view = world.view("pinky2")
    assert (view["on_map"], view["state"], view["nodes"]) == (True, "PARKED", ["P-2"])
    assert world.traffic.status_view("pinky2")["fault_reason"] == ""


def test_while_one_robot_cannot_be_placed_nobody_is_sent_anywhere():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-1", "pinky3": "P-3"})
    world.step()
    call = world.call("WORK")
    world.run(world.params.startup_wait_s + 1.0)
    assert not world.sent(MoveCmd) and world.result_of(call) is None
    assert world.codes().count("ROBOT_NOT_REPORTING") == 1
    world.bots["pinky2"].x, world.bots["pinky2"].y = world.site.node_pose("P-2")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).success


def test_a_robot_inside_a_fixed_stretch_but_on_no_node_is_not_placed():
    world = Harness(starts={"pinky1": "P-1"})
    world.bots["pinky1"].x, world.bots["pinky1"].y = 1.0, 0.89  # 급행 한가운데
    world.step()
    detail = [e.detail for e in world.events if e.code == "PLACEMENT_CONFLICT"]
    assert detail == ["no node near (1.00, 0.89)"]
    assert world.traffic.grants.holders("EXPRESS") == ()


def test_a_robot_in_the_yard_away_from_any_node_is_placed_in_the_yard():
    world = Harness(starts={"pinky1": "P-1"})
    world.bots["pinky1"].x, world.bots["pinky1"].y = 1.0, 0.40
    world.report()
    view = world.view("pinky1")
    assert (view["on_map"], view["state"], view["held"], view["nodes"]) == (
        True, "IDLE", ["HALL"], [])
    assert "PLACEMENT_CONFLICT" not in world.codes()


# ---- 고장으로 세움 ----

def test_a_fault_isolates_once_answers_the_dispatch_and_cancels_the_goal():
    world = Harness(speed=0.0)
    world.step()
    call = world.call("WORK", "pinky2")
    world.step()
    goal = world.view("pinky2")["goal_id"]
    world.traffic.on_robot_fault("pinky2", "MOTOR")
    world.step()
    assert world.result_of(call) == DispatchResult(call, False, "pinky2", "MOTOR")
    assert CancelCmd("pinky2", goal) in world.commands
    assert isolations(world, "pinky2") == [RobotIsolated("pinky2", "MOTOR")]
    world.bots["pinky2"].x += 0.30  # 밀려났다
    world.traffic.on_robot_fault("pinky2", "BUMPER")
    world.run(1.0)
    assert len(isolations(world, "pinky2")) == 1, "a second fault does not isolate again"
    assert world.codes("pinky2").count("ROBOT_ISOLATED") == 1
    assert world.traffic.status_view("pinky2")["fault_reason"] == "BUMPER", "the cause is the last"
    assert world.sent(CancelCmd, "pinky2") == [CancelCmd("pinky2", goal)]
    assert world.traffic._robots["pinky2"].fault_xy == world.site.node_pose("P-2"), \
        "where it stood when it first failed"


def test_an_isolated_robot_keeps_its_zones_and_takes_no_call():
    world = Harness()
    world.step()
    world.call("WORK", "pinky2")
    world.run(1.0)
    held = world.view("pinky2")["held"]
    world.silent.add("pinky2")  # 고장 난 로봇이 말도 없다: 아무것도 돌려주지 않는다
    world.traffic.on_robot_fault("pinky2", "MOTOR")
    world.call("WORK")
    world.run(5.0)
    view = world.view("pinky2")
    assert (view["state"], view["held"], view["called"]) == ("ERROR", held, False)
    assert world.view("pinky1")["called"] or world.view("pinky3")["called"]
    out = world.call("OUTBOUND", "pinky1")
    assert world.run_until(lambda: world.result_of(out) is not None, 200.0)
    assert world.result_of(out).success, "the far dock is still served"
    assert world.view("pinky2")["held"] == held
    again = world.call("OUTBOUND", "pinky2")
    world.step()
    assert world.result_of(again).reason == "NOT_AVAILABLE"


# ---- 세운 로봇이 쥔 대기 자리 ----

def test_a_robot_isolated_on_the_wait_spot_keeps_it_and_the_next_robot_waits_parked():
    world = reported({"pinky1": "W", "pinky2": "P-2", "pinky3": "W_Q"})
    world.traffic.on_robot_fault("pinky3", "MOTOR")
    first = world.call("WORK", "pinky1")
    assert world.run_until(lambda: world.result_of(first) is not None, 30.0)
    second = world.call("WORK", "pinky2")
    world.run(10.0)
    assert ("nav_goal", "W_Q") not in targets(world, "pinky2"), "the wait spot is taken"
    queued = [e.detail for e in world.events if e.code == "DOCK_QUEUED" and e.robot == "pinky2"]
    assert queued == ["WORK is busy, waits at P-2"]
    assert world.view("pinky3")["nodes"] == ["W_Q"]
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(second) is not None, 200.0)
    assert world.result_of(second).success
    assert ("nav_goal", "W_Q") not in targets(world, "pinky2")
    assert world.view("pinky3")["nodes"] == ["W_Q"] and not world.problems


# ---- 말이 없던 동안 지난 지점 ----

def on_the_dock_path_then_silent():
    """pinky1이 N_W에서 W로 가는 고정 경로(N_W, X_W, W)를 받고 말이 끊긴다."""
    world = reported({"pinky1": "N_W"})
    world.call("WORK", "pinky1")
    world.step()
    assert targets(world, "pinky1") == [("follow_path", "W")]
    world.silent.add("pinky1")
    world.run(world.params.comm_timeout_s + 0.5)
    assert world.view("pinky1")["state"] == "COMM_LOST"
    return world, world.bots["pinky1"]


def back_at(world, bot, node):
    """말이 없던 동안 node까지 갔다가 다시 보고한다."""
    bot.cmd, bot.points = None, []
    bot.x, bot.y = world.site.node_pose(node)
    world.silent.discard("pinky1")
    world.step()
    assert world.view("pinky1")["state"] == "RECOVERING"
    world.run(world.params.recover_check_s + 0.2)


def test_a_robot_back_from_silence_halfway_resumes_from_the_node_it_passed():
    world, bot = on_the_dock_path_then_silent()
    back_at(world, bot, "X_W")
    assert world.view("pinky1")["node"] == "X_W"
    assert targets(world, "pinky1") == [("follow_path", "W"), ("follow_path", "W")]
    resent = world.sent(MoveCmd, "pinky1")[-1]
    assert resent.path[0][:2] == world.site.node_pose("X_W"), "it starts where it is"
    assert len(resent.path) == 2, "only the link still ahead"


def test_a_robot_back_from_silence_at_the_dock_docks_without_driving_again():
    world, bot = on_the_dock_path_then_silent()
    back_at(world, bot, "W")
    assert world.view("pinky1")["node"] == "W"
    assert len(world.sent(MoveCmd, "pinky1")) == 1
    assert world.sent(DockCmd, "pinky1")
