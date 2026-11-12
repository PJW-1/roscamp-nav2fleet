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


def beside_a_taken_node(gap_m: float, hold_m: float = 0.05):
    """pinky1이 대기 자리 W_Q를 쥐었고, pinky2의 첫 보고는 W_Q에서 남쪽으로 gap_m이다.

    남쪽이면 W_Q가 가장 가까운 지점이다(N_W와 P-1은 0.25 m 넘게 떨어진다). pinky1은 W_Q에서
    북쪽으로 hold_m에 서 있다.
    """
    world = Harness(starts={"pinky1": "W_Q", "pinky2": "P-2", "pinky3": "P-3"})
    x, y = world.site.node_pose("W_Q")[:2]
    world.bots["pinky1"].y = y + hold_m
    world.bots["pinky2"].x, world.bots["pinky2"].y = x, y - gap_m
    assert world.site.nearest_node(x, y - gap_m, radius=0.15) == "W_Q"
    world.step()
    return world


def test_a_robot_beside_a_taken_node_in_the_yard_is_placed_in_the_yard():
    """재시작 시험에서 나왔다: 새 관제가 받은 첫 보고 하나가 W_Q에서 0.15 m 안이었고(실제 약
    0.2 m), 그 자리는 먼저 오른 로봇이 쥐고 있었다. 지점에는 한 대만 서지만 그 옆의 넓은 구역에는
    설 수 있다."""
    world = beside_a_taken_node(0.13)
    view = world.view("pinky2")
    assert view["on_map"] and "HALL" in view["held"] and "W_Q" not in view["nodes"]
    assert view["node"] != "W_Q", "W_Q is pinky1's; it carries the name of a free node"
    assert "PLACEMENT_CONFLICT" not in world.codes()
    assert "W_Q" in world.view("pinky1")["nodes"], "the node stays with pinky1"
    world.run(1.0)
    first = world.sent(MoveCmd, "pinky2")[0]
    assert not first.path, "it stands on no node, so it leaves by a Nav2 goal"
    assert not world.problems


def test_a_robot_reporting_right_on_a_taken_node_is_still_refused():
    world = beside_a_taken_node(0.06)
    assert world.view("pinky2")["on_map"] is False
    assert world.codes("pinky2") == ["PLACEMENT_CONFLICT"]


def test_two_robots_reporting_the_same_spot_beside_a_node_are_still_a_conflict():
    """지점에서 떨어진 자리라도 두 보고가 한 몸 안에서 겹치면 둘 중 하나는 틀린 보고다."""
    world = beside_a_taken_node(0.10, hold_m=-0.10)
    assert world.view("pinky1")["nodes"][0] == "W_Q"
    assert world.view("pinky2")["on_map"] is False
    assert world.codes("pinky2") == ["PLACEMENT_CONFLICT"]


def test_a_robot_beside_a_taken_node_is_not_put_on_a_free_node_of_another_zone():
    """N_W 옆인데 급행 입구 X_W도 0.15 m 안인 자리다. 선 곳은 마당이므로 마당을 쥔다: X_W에
    올리면 갈림목만 쥐고, 실제로 서 있는 마당은 쥐지 않는다."""
    world = Harness(starts={"pinky1": "N_W", "pinky2": "P-2", "pinky3": "P-3"})
    x, y = world.site.node_pose("N_W")[:2]
    world.bots["pinky1"].y = y - 0.05
    world.bots["pinky2"].x, world.bots["pinky2"].y = x, y + 0.11
    others = [name for name in world.site.nodes if name != "N_W"]
    assert world.site.nearest_node(x, y + 0.11, others, radius=0.15) == "X_W"
    assert world.site.zone_at(x, y + 0.11) == "HALL"
    world.report()
    view = world.view("pinky2")
    assert (view["on_map"], view["held"], view["nodes"]) == (True, ["HALL"], [])
    assert view["node"] not in ("N_W", "X_W")


def test_two_robots_reporting_the_same_spot_in_the_yard_are_a_conflict():
    """지점이 없는 자리에서도 같다: 두 보고가 한 몸 안에서 겹치면 나중 것을 올리지 않는다."""
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"})
    for name in ("pinky1", "pinky2"):
        world.bots[name].x, world.bots[name].y = 1.0, 0.40
    world.step()
    assert world.view("pinky1")["on_map"] and world.view("pinky2")["on_map"] is False
    assert world.codes("pinky2") == ["PLACEMENT_CONFLICT"]
    world.bots["pinky2"].y -= 0.25  # 보고가 바로잡혔다
    world.step()
    assert world.view("pinky2")["on_map"]
    assert world.traffic.status_view("pinky2")["fault_reason"] == ""


def test_a_third_robot_reporting_where_the_one_beside_the_node_stands_is_a_conflict():
    """지점을 쥔 로봇과는 떨어져 있어도, 그 옆에 지점 없이 오른 로봇과 겹치면 올리지 않는다."""
    world = Harness(starts={"pinky1": "W_Q", "pinky2": "P-2", "pinky3": "P-3"})
    x, y = world.site.node_pose("W_Q")[:2]
    world.bots["pinky1"].y = y + 0.05
    for name in ("pinky2", "pinky3"):
        world.bots[name].x, world.bots[name].y = x, y - 0.13
    world.step()
    assert world.view("pinky2")["on_map"] and world.view("pinky3")["on_map"] is False
    assert world.codes("pinky3") == ["PLACEMENT_CONFLICT"]


def test_a_robot_beside_a_taken_node_is_not_on_it_when_the_zone_has_no_free_node():
    """구역의 지점을 남들이 모두 쥐고 있으면 붙일 이름은 남의 지점뿐이다. 이름은 그 지점이어도
    그 위에 선 로봇으로 다루지 않는다: 주차 자리 옆에 섰다고 주차한 것이 아니다."""
    world = Harness(layout="d", starts={"pinky1": "N_E", "pinky2": "P-3", "pinky3": "P-1"})
    world.silent.add("pinky3")
    world.report()
    assert world.traffic.grants.force("pinky1", ["HALL_E"], ["O_Q"])  # 대기 자리로 가려고 잡았다
    x, y = world.site.node_pose("P-3")[:2]
    world.bots["pinky2"].x = x + 0.05
    world.bots["pinky3"].x, world.bots["pinky3"].y = x - 0.12, y
    assert world.site.nearest_node(x - 0.12, y, radius=0.15) == "P-3"
    world.silent.clear()
    world.step()
    view = world.view("pinky3")
    assert view["on_map"] and view["held"] == ["HALL_E"] and view["nodes"] == []
    assert view["state"] != "PARKED" and "PLACEMENT_CONFLICT" not in world.codes()
    world.run(1.0)
    assert world.view("pinky3")["state"] == "TO_PARK", "P-3 is pinky2's; it looks for a free one"
    assert world.view("pinky2")["nodes"] == ["P-3"]


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
