"""FleetManager는 두 코어를 잇는다: 내부 호출은 안에 머물고 옛 출력은 그대로 나온다.

D2 배치와 구역 교통 층(ZoneTraffic)에서 돈다. 로봇 흉내는 drive()가 한다: 받은 이동과
물러나기는 곧바로 끝 점에 닿아 성공하고, 도킹도 곧바로 성공한다.
"""
from core_helpers import cmds
from fleet_core.common.commands import (
    CancelDispatchCmd,
    DispatchCmd,
    DispatchFeedback,
    DispatchResult,
    DockCmd,
    MoveCmd,
    MoveOrderCmd,
    OrderRecord,
    ReleaseCmd,
    RetreatCmd,
    RobotFaultCmd,
    RobotIsolated,
)
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.manager import FleetManager

INTERNAL = (DispatchCmd, CancelDispatchCmd, ReleaseCmd, RobotFaultCmd, DispatchFeedback,
            DispatchResult, RobotIsolated)
O1 = OrderRecord("O-001", {"A": 1})
O2 = OrderRecord("O-002", {"B": 1})
SITE = load_fleet_map("d2")


def make(nodes, orders=()):
    now = [0.0]
    core = FleetManager(SITE, load_params(), list(nodes), lambda: now[0])
    for robot, node in nodes.items():
        report(core, robot, node)
    core.on_order_board("RUN", list(orders))
    return core, now


def report_at(core, robot, x, y, state="IDLE"):
    for _ in range(core.params.release_confirm_samples):
        core.on_robot_state(robot, x, y, 0.0, state)


def report(core, robot, node, state="IDLE"):
    report_at(core, robot, *core.site.node_pose(node), state)


def finish(core, robots, out):
    """out 가운데 robots에 온 이동, 물러나기, 도킹을 곧바로 성공시킨다. order는 base 끝까지 곧바로
    가서, horizon이 남으면 서서 update를 기다리고 없으면 끝난다. update는 이전 goal을 밀어낸다."""
    runs = core.__dict__.setdefault("_test_order_runs", {})  # 로봇 -> (goal id, 끝낸 seq)
    for c in out:
        robot = getattr(c, "robot", None)
        if robot not in robots:
            continue
        if isinstance(c, MoveOrderCmd):
            seq = 0
            if c.update_id and robot in runs:
                old, seq = runs[robot]
                core.on_move_result(robot, old, False, "SUPERSEDED", seq)
            if c.base:
                report_at(core, robot, c.base[-1].x, c.base[-1].y)
                seq = c.base[-1].seq_id
            runs[robot] = (c.goal_id, seq)
            if c.horizon:
                core.on_move_feedback(robot, c.goal_id, seq, True)
            else:
                del runs[robot]
                core.on_move_result(robot, c.goal_id, True, "", seq)
        elif isinstance(c, MoveCmd):
            report_at(core, robot, c.x, c.y)
            core.on_move_result(robot, c.goal_id, True, "")
        elif isinstance(c, RetreatCmd):
            x, y = c.path[-1][:2]
            report_at(core, robot, x, y)
            core.on_retreat_result(robot, c.goal_id, True, "")
        elif isinstance(c, DockCmd):
            core.on_dock_result(robot, c.goal_id, True)


def drive(core, robots, until, first=(), limit=300):
    """관제를 돌리며 robots(이름 하나 또는 여럿)에 온 일을 finish로 끝낸다. first는 이미
    받아 둔 출력이다. 다른 로봇은 선다."""
    robots = (robots,) if isinstance(robots, str) else tuple(robots)
    finish(core, robots, first)
    seen = []
    for _ in range(limit):
        out = core.tick()
        seen += out
        finish(core, robots, out)
        if until():
            return seen + core.tick()
    raise AssertionError(f"not reached: {[core.robot_view(r) for r in robots]}")


def test_inside_calls_never_reach_the_ros_side():
    core, _ = make({"r1": "P-1"}, [O1, O2])
    core.on_order_board("RUN", [O1, O2])
    out = core.tick()
    out += drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001", first=out)
    assert out, "the walk produced commands"
    assert not [c for c in out if isinstance(c, INTERNAL)]


def test_an_order_goes_from_the_dock_to_the_fill():
    core, _ = make({"r1": "P-1"}, [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert not [c for c in out if isinstance(c, INTERNAL)]
    assert cmds(out, "FillCmd")[0].items == {"A": 1}
    assert core.robot_view("r1")["phase"] == "FILLING"
    assert core.robot_view("r1")["state"] == "FILLING"


def test_the_view_shows_the_old_station_phases_not_the_traffic_one():
    core, _ = make({"r1": "P-1"}, [O1])
    drive(core, "r1", lambda: core.traffic.robot_view("r1")["phase"] == "AT_STATION")
    assert core.robot_view("r1")["phase"] == "FILLING"
    assert [a.task_type for a in core.assignments_view()] == ["FILL"]


def test_goal_ids_of_moves_and_arm_jobs_share_one_sequence():
    core, _ = make({"r1": "P-1"}, [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    ids = [c.goal_id for c in out
           if type(c).__name__ in ("MoveCmd", "MoveOrderCmd", "RetreatCmd", "DockCmd", "FillCmd")]
    assert ids == sorted(ids) and len(ids) == len(set(ids))


def test_confirm_clear_reaches_both_cores():
    core, _ = make({"r1": "P-1"}, [O1])
    drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    released = core.confirm_clear("r1", core.zones.held_by("r1"), "lifted")
    assert released
    assert core.robot_view("r1")["state"] == "OUT_OF_SERVICE"
    assert core.robot_view("r1")["order"] is None
    assert core.confirm_clear("nobody", [], "typo") is None


def test_a_refused_confirm_clear_keeps_the_order_on_its_robot():
    core, _ = make({"r1": "P-1"}, [O1])
    drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    assert core.confirm_clear("r1", ["NOPE"], "typo") is None
    assert core.confirm_clear("r1", [], "forgot the list") is None
    assert core.robot_view("r1")["order"] == "O-001"
    assert core.robot_view("r1")["state"] != "OUT_OF_SERVICE"


def test_clearing_a_robot_in_two_calls_puts_its_order_back_once():
    core, _ = make({"r1": "P-1"}, [O1])
    drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    held = list(core.zones.held_by("r1"))
    assert held and "OUT_DOCK" not in held
    core.confirm_clear("r1", ["OUT_DOCK"], "a zone it does not hold")
    first = [c.code for c in core.tick() if hasattr(c, "code")]
    core.confirm_clear("r1", held, "the one it does")
    second = [c.code for c in core.tick() if hasattr(c, "code")]
    assert first.count("ORDER_REQUEUED") == 1
    assert "ORDER_REQUEUED" not in second
    assert list(core.zones.held_by("r1")) == []


def test_a_robot_faulted_by_the_arm_stops_being_available_to_traffic():
    core, now = make({"r1": "P-1"}, [O1])
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001")
    goal = cmds(out, "FillCmd")[0].goal_id
    for _ in range(core.params.move_retry_limit + 1):
        core.on_fill_result(goal, False, {}, {}, "GRIPPER")
        now[0] += core.params.arm_retry_wait_s
        report(core, "r1", "W", "DOCKED")
        fills = cmds(core.tick(), "FillCmd")
        if fills:
            goal = fills[0].goal_id
    assert core.robot_view("r1")["state"] == "ERROR"
    assert not core.traffic.available("r1")


def _filled_and_leaving(robot="r1"):
    """실제 결과로 채워지고 나가는 중인 바구니. 화이트박스 없음."""
    core, now = make({robot: "P-1"}, [O1])
    out = drive(core, robot, lambda: core.robot_view(robot)["order"] == "O-001")
    core.on_fill_result(cmds(out, "FillCmd")[0].goal_id, True, {"A": 1}, {}, "")
    report(core, robot, "W", "DOCKED")
    leaving = core.tick()
    return core, now, leaving


def test_a_real_fill_opens_the_trip_out_by_name():
    core, _, leaving = _filled_and_leaving()
    assert [c for c in leaving if isinstance(c, (MoveCmd, MoveOrderCmd, RetreatCmd))
            and c.robot == "r1"]
    assert core.robot_view("r1")["basket"] == "FULL"
    assert core.robot_view("r1")["destination"] == "OUTBOUND"


def test_recovery_after_a_real_fill_still_heads_to_outbound():
    """test_fleet_unit은 legacy_core로 같은 상황을 강제하고, 여기서는 실제 입력을 쓴다."""
    core, now, _ = _filled_and_leaving()
    now[0] += core.params.comm_timeout_s + 0.1
    core.tick()
    assert core.robot_view("r1")["state"] == "COMM_LOST"
    now[0] += 1.0
    report(core, "r1", "W")
    core.tick()
    now[0] += core.params.recover_check_s
    report(core, "r1", "W")
    core.tick()
    assert core.robot_view("r1")["state"] == "TO_OUTBOUND"
    assert core.robot_view("r1")["destination"] == "OUTBOUND"


def test_a_robot_at_a_station_waits_for_nobody():
    """스테이션에 선 로봇은 구역을 기다리지 않는다(대기 순환에 낄 수 없다)."""
    core, _ = make({"r1": "P-1", "r2": "P-2"}, [O1])
    pair = ("r1", "r2")
    def docked():
        return [r for r in pair if core.traffic.robot_view(r)["phase"] == "AT_STATION"]
    drive(core, pair, docked)
    robot = docked()[0]
    assert (status(core, robot).wait_zone, status(core, robot).wait_robots) == ("", ())


# ---- /fleet/robots ----------------------------------------------------------------------


def status(core, robot):
    return {view.robot_id: view for view in core.robots_view()}[robot]


def test_the_status_says_what_the_pinky_reported_and_where_it_is():
    core, _ = make({"r1": "P-1"})
    x, y = core.site.node_pose("P-1")
    core.on_robot_state("r1", x, y, 0.5, "WAITING", 87.5)
    view = status(core, "r1")
    assert (view.fsm_state, view.x, view.y, view.yaw, view.battery) == ("WAITING", x, y, 0.5, 87.5)
    assert (view.fault, view.fault_reason) == ("NONE", "")
    assert (view.wait_zone, view.wait_robots) == ("", ())


def test_a_robot_nobody_has_heard_from_still_has_a_status():
    core = FleetManager(SITE, load_params(), ["r1"], lambda: 0.0)
    view = status(core, "r1")
    assert (view.fsm_state, view.task_type, view.order_id, view.fault) == ("", "", "", "NONE")


def test_comm_loss_shows_in_the_status_and_goes_away():
    core, now = make({"r1": "P-1"})
    now[0] = core.params.comm_timeout_s + 0.1
    core.tick()
    assert (status(core, "r1").fault, status(core, "r1").fault_reason) == ("COMM_LOST", "NO_STATE")
    report(core, "r1", "P-1")
    assert status(core, "r1").fault == "RECOVERING"
    now[0] += core.params.recover_check_s + 0.1
    report(core, "r1", "P-1")
    core.tick()
    assert (status(core, "r1").fault, status(core, "r1").fault_reason) == ("NONE", "")


def test_an_error_shows_its_cause():
    core, _ = make({"r1": "P-1"})
    core.traffic.on_robot_fault("r1", "NAV_ABORTED")
    assert (status(core, "r1").fault, status(core, "r1").fault_reason) == ("ERROR", "NAV_ABORTED")


def test_a_cleared_robot_is_out_of_service_with_no_task():
    core, _ = make({"r1": "P-1"})
    core.confirm_clear("r1", core.zones.held_by("r1"), "lifted off")
    view = status(core, "r1")
    assert (view.fault, view.fault_reason, view.task_type) == ("OUT_OF_SERVICE", "lifted off", "")


def test_a_waiting_robot_shows_the_zone_and_who_is_in_the_way():
    """a가 작업 도크 앞목(X_W, 구역 J_W)에서 고장 나 섰다. 불린 b는 J_W를 a 때문에 기다린다.
    (도크에 선 로봇 뒤의 차례는 구역 대기가 아니라 DOCK_QUEUED다.)"""
    core, _ = make({"a": "X_W", "b": "P-2"})
    core.traffic.on_robot_fault("a", "NAV_ABORTED")
    core.on_order_board("RUN", [O1])
    core.tick()
    core.tick()
    view = status(core, "b")
    assert view.wait_zone == "J_W" and view.wait_robots == ("a",)
    assert status(core, "a").wait_zone == ""
    core.confirm_clear("a", core.zones.held_by("a"), "lifted off")
    core.tick()
    core.tick()
    assert (status(core, "b").wait_zone, status(core, "b").wait_robots) == ("", ())
    assert "J_W" in core.zones.held_by("b")


def test_sr17_every_robot_state_can_be_told_apart():
    """대기, 이동, 도킹, 담기 대기, 출고 대기, 오류 + 원인. 표는 worklog에 있다."""
    core, _ = make({"r1": "P-1", "r2": "P-3"})
    assert status(core, "r1").task_type == "WAIT"
    core.on_order_board("RUN", [O1])
    first = core.tick()
    assert status(core, "r1").task_type == "TO_WORK"
    out = drive(core, "r1", lambda: core.robot_view("r1")["order"] == "O-001", first=first)
    out = first + out
    view = status(core, "r1")
    assert (view.task_type, view.order_id) == ("FILL", "O-001")
    core.on_fill_result(cmds(out, "FillCmd")[0].goal_id, True, {"A": 1}, {}, "")
    report(core, "r1", "W", "DOCKED")
    first = core.tick()
    assert status(core, "r1").task_type == "TO_OUTBOUND"
    drive(core, "r1", lambda: core.robot_view("r1")["phase"] == "SWAPPING", first=first)
    assert status(core, "r1").task_type == "SWAP"
    x, y = core.site.node_pose("P-3")
    core.on_robot_state("r2", x, y, 0.0, "DOCKING")
    assert status(core, "r2").fsm_state == "DOCKING"
    core.traffic.on_robot_fault("r2", "DOCK_FAILED")
    assert (status(core, "r2").fault, status(core, "r2").fault_reason) == ("ERROR", "DOCK_FAILED")
