"""직접 짠 서버와 실제 그래프로 노드의 배선(이름, 타입, goal, 취소)을 시험한다.

가짜 로봇 없이 래퍼의 배선만 본다. 그래프에 참여하므로 로봇 네트워크와 분리된 도메인에서만 돈다.
"""
import dataclasses
import os
import subprocess
import sys
import threading
import time
from functools import partial
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.commands import EventOut, ReportOrderCmd, RetreatCmd  # noqa: E402
from fleet_core.common.contract import to_ros_qos  # noqa: E402
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_interfaces.action import Dock, FillBasket, MoveTo, Retreat, SwapBasket  # noqa: E402
from fleet_interfaces.msg import (  # noqa: E402
    ArmState,
    AssignmentArray,
    FleetEvent,
    Order,
    OrderBoard,
    OrderItem,
    RobotState,
    RobotStatusArray,
    SystemMode,
    ZoneStateArray,
)
from fleet_interfaces.srv import (  # noqa: E402
    ConfirmClear,
    Reinstate,
    ReportOrder,
    SetMode,
    TripCommand,
)
from fleet_manager.fleet_manager_node import FleetManagerNode, make_executor  # noqa: E402
from fleet_manager.goal_tracker import GoalTracker, ReportOutbox  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.task import Future  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

REPLAY = Path(__file__).parents[3] / "tools" / "replay.py"

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")


def items(counts):
    return [OrderItem(kind=k, count=c) for k, c in counts.items()]


class World(Node):
    """시키는 대로 즉시 하는 로봇 하나, 팔 서버 둘, central."""

    def __init__(self, contract, site, node, robot="pode1", robot_servers=True, arms=True):
        super().__init__("wiring_world")
        self.robot = robot
        group = ReentrantCallbackGroup()
        self.site = site
        self.pose = site.node_pose(node)
        self.mode = ""
        self.fills = []
        self.swaps = []
        self.reports = []
        self.assignments = []
        self.hold_fill = threading.Event()  # 켜져 있는 동안 fill은 취소될 때까지 돈다
        self.canceled_fills = []
        self.fill_cancel_reason = "CANCELED"
        self.report_accepted = True
        self.report_delay_s = 0.0
        self.refuse_fill_cancel = False
        self.fill_cancel_asks = 0
        name = contract.name
        self.events = []
        self.create_subscription(
            FleetEvent, name("events"), self.events.append, to_ros_qos(contract.qos("events")))
        self.board = self.create_publisher(
            OrderBoard, name("order_board"), to_ros_qos(contract.qos("order_board")))
        self.state = self.create_publisher(
            RobotState, name("robot_state", pode=robot),
            to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(
            AssignmentArray, name("assignments"),
            lambda m: self.assignments.append(list(m.assignments)),
            to_ros_qos(contract.qos("assignments")))
        self.zones = []
        self.create_subscription(
            ZoneStateArray, name("zones"), self.zones.append, to_ros_qos(contract.qos("zones")))
        self.status = []
        self.create_subscription(
            RobotStatusArray, name("robots"), self.status.append,
            to_ros_qos(contract.qos("robots")))
        self.beats = []
        self.create_subscription(
            Header, name("heartbeat"), lambda _m: self.beats.append(time.monotonic()),
            to_ros_qos(contract.qos("heartbeat")))
        self.create_timer(0.02, self._publish_state, callback_group=group)
        # 작업대 두 팔도 실물처럼 상태를 낸다 (3 s 없으면 상태 모름으로 작업대가 막힘)
        self.arm_mode = ""
        self.arm_states = {"arm_left", "arm_right"} if arms else set()
        self._arm_pubs = {arm: self.create_publisher(ArmState, name("arm_state", arm=arm),
                                                     to_ros_qos(contract.qos("arm_state")))
                          for arm in ("arm_left", "arm_right")}
        self.create_timer(0.1, self._publish_arms, callback_group=group)
        self.create_service(ReportOrder, name("report_order"), self._on_report,
                            callback_group=group)
        self._servers = [
            ActionServer(self, FillBasket, name("fill_basket"), self._on_fill,
                         callback_group=group, cancel_callback=self._on_fill_cancel),
            ActionServer(self, SwapBasket, name("swap_basket", arm="arm_ship"), self._on_swap,
                         callback_group=group),
        ]
        self._contract, self._group = contract, group
        if robot_servers:
            self.serve_robot()

    def serve_robot(self):
        name, group = self._contract.name, self._group
        self._servers += [
            ActionServer(self, MoveTo, name("move_to", pode=self.robot), self._on_move,
                         callback_group=group),
            ActionServer(self, Dock, name("dock", pode=self.robot), self._on_dock,
                         callback_group=group),
            ActionServer(self, Retreat, name("retreat", pode=self.robot), self._on_retreat,
                         callback_group=group),
        ]

    def _publish_state(self):
        msg = RobotState(robot_id=self.robot, state=RobotState.STATE_IDLE, mode=self.mode,
                         boot_id="w1")
        msg.pose.x, msg.pose.y = self.pose
        self.state.publish(msg)

    def _publish_arms(self):
        for arm, pub in self._arm_pubs.items():
            if arm in self.arm_states:
                pub.publish(ArmState(arm_id=arm, state="IDLE", lock=True, boot_id="b-1",
                                     mode=self.arm_mode))

    def _on_move(self, handle):
        self.pose = (handle.request.target_pose.x, handle.request.target_pose.y)
        handle.succeed()
        return MoveTo.Result(success=True)

    def _on_retreat(self, handle):
        if handle.request.path:
            end = handle.request.path[-1].position
            self.pose = (end.x, end.y)
        handle.succeed()
        return Retreat.Result(success=True)

    def _on_dock(self, handle):
        handle.succeed()
        return Dock.Result(success=True)

    def _on_fill(self, handle):
        wanted = {i.kind: i.count for i in handle.request.items}
        self.fills.append((handle.request.order_id, handle.request.robot_id,
                           handle.request.station_id, wanted))
        while self.hold_fill.is_set():
            if handle.is_cancel_requested:
                first = sorted(wanted)[0]
                handle.canceled()
                self.canceled_fills.append(handle.request.order_id)
                return FillBasket.Result(success=False, placed=items({first: 1}),
                                         reason=self.fill_cancel_reason)
            time.sleep(0.01)
        handle.succeed()
        return FillBasket.Result(success=True, placed=items(wanted))

    def _on_fill_cancel(self, _handle):
        self.fill_cancel_asks += 1
        return CancelResponse.REJECT if self.refuse_fill_cancel else CancelResponse.ACCEPT

    def _on_swap(self, handle):
        self.swaps.append((handle.request.order_id, handle.request.take_full,
                           handle.request.put_empty))
        handle.succeed()
        return SwapBasket.Result(success=True, took_full=handle.request.take_full,
                                 put_empty_done=True)

    def _on_report(self, request, response):
        self.reports.append(request)
        time.sleep(self.report_delay_s)
        response.accepted = self.report_accepted
        return response


@pytest.fixture
def graph(request):
    # indirect param {"arms": False}: 작업대 팔 상태를 처음부터 내지 않는다
    opts = getattr(request, "param", {})
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", "tick_hz:=50.0"])
    node = FleetManagerNode()
    world = World(node.contract, load_fleet_map("d2"), "P-1", arms=opts.get("arms", True))
    world.fms = node
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield world
    world.hold_fill.clear()  # 서버 스레드를 멈춘다
    executor.shutdown()
    thread.join(timeout=5)
    world.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def publish(world, mode, counts=None):
    orders = [Order(order_id="O-001", items=items(counts))] if counts else []
    world.board.publish(OrderBoard(run_mode=mode, orders=orders))


def wait_for(predicate, timeout_s=40.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_an_order_goes_through_fill_and_swap_to_its_report(graph):
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.reports), f"fills {graph.fills}, swaps {graph.swaps}"
    report = graph.reports[0]
    assert (report.order_id, report.robot_id, report.status) == ("O-001", "pode1", "SHIPPED")
    assert {i.kind: i.count for i in report.placed} == {"A": 1, "C": 2}
    assert graph.fills == [("O-001", "pode1", "WORK", {"A": 1, "C": 2})]
    assert graph.swaps == [("O-001", True, True)]
    seen = [(a.order_id, a.station_id, a.task_type) for array in graph.assignments for a in array]
    assert ("O-001", "WORK", "FILL") in seen
    assert ("O-001", "OUTBOUND", "SWAP") in seen


def test_the_status_of_every_robot_comes_out_about_five_times_a_second(graph):
    """팀 명세 F-11: /fleet/robots는 5 Hz 안팎."""
    graph.status.clear()
    assert wait_for(lambda: len(graph.status) >= 11), f"got {len(graph.status)}"
    first, last = graph.status[0], graph.status[-1]
    span = (last.stamp.sec - first.stamp.sec) + (last.stamp.nanosec - first.stamp.nanosec) / 1e9
    assert 0.14 <= span / (len(graph.status) - 1) <= 0.26
    robot = last.robots[0]
    assert (robot.robot_id, robot.fsm_state, robot.fault) == ("pode1", "IDLE", "NONE")
    assert (robot.x, robot.y) != (0.0, 0.0)


@pytest.mark.parametrize("graph", [{"arms": False}], indirect=True)
def test_the_status_lists_the_three_arms_after_the_podes(graph):
    # arm_left는 이 시험이 직접 내고, arm_right는 상태 없음을 본다
    pub = graph.create_publisher(ArmState, graph.fms.contract.name("arm_state", arm="arm_left"),
                                 to_ros_qos(graph.fms.contract.qos("arm_state")))
    graph.create_timer(0.1, lambda: pub.publish(ArmState(arm_id="arm_left", state="PICKING",
                                                         lock=True, boot_id="b-7")))

    def left_seen():
        rows = graph.status[-1].robots if graph.status else []
        return any(r.robot_id == "arm_left" and r.fsm_state == "PICKING" for r in rows)
    assert wait_for(left_seen, 10.0)
    rows = {r.robot_id: r for r in graph.status[-1].robots}
    assert list(rows) == ["pode1", "arm_left", "arm_right", "arm_ship"]
    assert (rows["pode1"].kind, rows["arm_left"].kind) == ("POD", "ARM")
    assert (rows["arm_left"].fault, rows["arm_left"].lock, rows["arm_left"].boot_id) == (
        "NONE", True, "b-7")
    assert rows["arm_right"].fault == "COMM_LOST"


@pytest.mark.parametrize("bad", ["0.0", "-5.0", ".inf"])
def test_a_robots_rate_that_is_not_positive_is_refused(bad):
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", f"robots_hz:={bad}"])
    try:
        with pytest.raises(ValueError, match="robots_hz"):
            FleetManagerNode()
    finally:
        rclpy.shutdown()


def test_a_new_fms_stays_silent_until_any_goal_of_an_old_one_must_have_stopped(graph):
    """먼저 읽고 나서 아직 조용한지 확인한다."""
    fms = graph.fms
    while True:
        on_map, beats = fms.core.traffic.robot_view("pode1")["on_map"], list(graph.beats)
        if not fms.silent():
            break
        assert not on_map, "took a report during the silence"
        assert not beats, "a heartbeat during the silence"
        time.sleep(0.02)
    assert wait_for(lambda: fms.core.traffic.robot_view("pode1")["on_map"], 10.0)


def test_a_short_stall_s_is_warned_once_when_the_params_are_read(tmp_path, monkeypatch):
    """stall_s가 로봇 BLOCKED 보고보다 짧으면 경고만 남기고 뜬다."""
    import yaml
    from fleet_core.common.params import DEFAULT_PARAMS
    data = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    path = tmp_path / "short_stall.yaml"
    path.write_text(yaml.safe_dump({**data, "stall_s": 6.0}), encoding="utf-8")
    logged = []

    class Recorder:
        def get_child(self, _name):
            return self

        def __getattr__(self, level):
            return lambda text, **_: logged.append((level, text))

    monkeypatch.setattr(FleetManagerNode, "get_logger", lambda self: Recorder())
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", f"params_file:={path}"])
    try:
        node = FleetManagerNode()
        try:
            assert node.core.params.stall_s == 6.0
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    warned = [text for level, text in logged if level == "warning" and "stall_s" in text]
    assert len(warned) == 1, logged


def test_a_robot_counts_only_once_its_goals_can_reach_it():
    """새 FMS는 전원이 로봇으로 인정되기 전에는 goal, 잠금, heartbeat를 보내지 않는다."""
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode3]", "-p", "tick_hz:=50.0"])
    node = FleetManagerNode()
    world = World(node.contract, load_fleet_map("d2"), "P-1", robot="pode3", robot_servers=False)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert wait_for(lambda: not node.silent(), 20.0)
        time.sleep(1.0)
        assert not node.core.traffic.robot_view("pode3")["on_map"]
        assert not world.zones
        assert not world.beats
        world.serve_robot()
        assert wait_for(lambda: node.core.traffic.robot_view("pode3")["on_map"], 20.0)
        assert wait_for(lambda: world.zones, 5.0)
        assert wait_for(lambda: world.beats, 5.0)
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        world.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_a_record_of_the_running_node_plays_back_to_the_same_commands(tmp_path):
    """main()과 같은 실행기(make_executor)로 노드를 돌린다."""
    record = tmp_path / "fms.jsonl"
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", "tick_hz:=50.0"])
    node = FleetManagerNode(record=record)
    world = World(node.contract, load_fleet_map("d2"), "P-1")
    executors = [make_executor(), MultiThreadedExecutor(num_threads=4)]
    executors[0].add_node(node)
    executors[1].add_node(world)
    threads = [threading.Thread(target=e.spin, daemon=True) for e in executors]
    for thread in threads:
        thread.start()
    try:
        publish(world, "RUN", {"A": 1, "C": 2})
        assert wait_for(lambda: world.reports), f"fills {world.fills}, swaps {world.swaps}"
    finally:
        for executor, thread in zip(executors, threads, strict=True):
            executor.shutdown()
            thread.join(timeout=5)
        world.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
    played = subprocess.run([sys.executable, str(REPLAY), str(record)], capture_output=True,
                            text=True, timeout=120)
    assert played.returncode == 0, played.stdout + played.stderr
    assert "same as recorded" in played.stdout
    assert '"out": "ReportOrderCmd"' in record.read_text(encoding="utf-8")


def test_a_zone_that_does_not_exist_is_refused_and_the_node_lives_on(graph):
    """콜백 예외 하나로 노드 프로세스가 끝나지 않는다."""
    fms = graph.fms
    client = graph.create_client(ConfirmClear, fms.contract.name("confirm_clear"))
    assert client.wait_for_service(timeout_sec=10.0)
    assert wait_for(lambda: fms.core.traffic.robot_view("pode1")["on_map"]), "never on the map"
    future = client.call_async(ConfirmClear.Request(
        robot_id="pode1", zone_ids=["NOPE"], note="typo"))
    assert wait_for(future.done, 10.0), "no reply: the callback raised"
    assert not future.result().accepted
    assert fms.core.traffic.robot_view("pode1")["fault"] == ""


def test_a_bad_clear_gives_a_reason_and_input_rejected_and_the_ticks_go_on(graph):
    """잘못된 치움은 거부 이유와 INPUT_REJECTED를 주고 그 뒤에도 타이머와 주문이 돈다."""
    fms = graph.fms
    client = graph.create_client(ConfirmClear, fms.contract.name("confirm_clear"))
    assert client.wait_for_service(timeout_sec=10.0)
    assert wait_for(lambda: fms.core.traffic.robot_view("pode1")["on_map"]), "never on the map"
    graph.events.clear()
    future = client.call_async(ConfirmClear.Request(
        robot_id="pode1", zone_ids=["NOPE"], note="typo"))
    assert wait_for(future.done, 10.0), "no reply: the callback raised"
    result = future.result()
    assert not result.accepted and result.reason == "NOT_FOUND" and "NOPE" in result.detail
    assert wait_for(lambda: any(e.code == "INPUT_REJECTED" for e in graph.events), 10.0)
    rejected = [e for e in graph.events if e.code == "INPUT_REJECTED"]
    assert len(rejected) == 1 and rejected[0].level == "WARN"
    assert not any(e.code == "CLEAR_REFUSED" for e in graph.events)
    # 노드가 살아 있다
    graph.status.clear()
    assert wait_for(lambda: len(graph.status) >= 2, 10.0), "no /fleet/robots after the call"
    second = client.call_async(ConfirmClear.Request(robot_id="pode9", zone_ids=[], note="x"))
    assert wait_for(second.done, 10.0)
    assert not second.result().accepted and second.result().reason == "UNKNOWN_TARGET"
    assert "pode9" in second.result().detail
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.reports), "the node stopped working orders"


def test_reinstate_takes_a_cleared_robot_back_and_refuses_a_bad_call_with_a_reason(graph):
    """거부는 이유와 INPUT_REJECTED, 받아들이면 ROBOT_REINSTATED."""
    fms = graph.fms
    clear = graph.create_client(ConfirmClear, fms.contract.name("confirm_clear"))
    back = graph.create_client(Reinstate, fms.contract.name("reinstate"))
    assert clear.wait_for_service(timeout_sec=10.0) and back.wait_for_service(timeout_sec=10.0)
    assert wait_for(lambda: fms.core.traffic.robot_view("pode1")["on_map"]), "never on the map"
    graph.events.clear()
    early = back.call_async(Reinstate.Request(robot_id="pode1", basket="EMPTY", note="x"))
    assert wait_for(early.done, 10.0), "no reply: the callback raised"
    assert not early.result().accepted and early.result().reason == "NOT_READY"
    assert "OUT_OF_SERVICE" in early.result().detail
    cleared = clear.call_async(ConfirmClear.Request(
        robot_id="pode1", zone_ids=fms.core.zones.held_by("pode1"), note="lifted"))
    assert wait_for(cleared.done, 10.0) and cleared.result().accepted
    wrong = back.call_async(Reinstate.Request(robot_id="pode1", basket="FULL", note="x"))
    assert wait_for(wrong.done, 10.0)
    assert not wrong.result().accepted and wrong.result().reason == "ORDER_STATE"
    assert "basket" in wrong.result().detail
    good = back.call_async(Reinstate.Request(
        robot_id="pode1", basket=Reinstate.Request.BASKET_EMPTY, note="back"))
    assert wait_for(good.done, 10.0)
    assert good.result().accepted and good.result().reason == ""
    assert wait_for(lambda: any(e.code == "ROBOT_REINSTATED" for e in graph.events), 10.0)
    rejected = [e for e in graph.events if e.code == "INPUT_REJECTED"]
    assert len(rejected) == 2 and all(e.level == "WARN" for e in rejected)
    assert wait_for(lambda: fms.core.traffic.robot_view("pode1")["on_map"]), "not back on"


def test_force_ship_refuses_an_order_it_does_not_know_with_the_reason(graph):
    """강제 출고 창구(TripCommand). 모르는 주문은 NOT_FOUND로 거절한다."""
    fms = graph.fms
    client = graph.create_client(TripCommand, fms.contract.name("force_ship"))
    assert client.wait_for_service(timeout_sec=10.0)
    reply = client.call_async(TripCommand.Request(request_id="r-9", trip_id="O-NOPE"))
    assert wait_for(reply.done, 10.0), "no reply: the callback raised"
    assert not reply.result().accepted
    assert reply.result().reason == "NOT_FOUND"
    assert reply.result().request_id == "r-9"


@pytest.mark.parametrize("answer", [(True, ""), (False, "NOT_READY"), (False, "ORDER_STATE")])
def test_force_ship_passes_the_order_to_the_core_and_returns_its_answer(graph, monkeypatch,
                                                                         answer):
    fms = graph.fms
    asked = []

    def force_ship(order_id):
        asked.append(order_id)
        return answer

    monkeypatch.setattr(fms.core, "force_ship", force_ship)
    client = graph.create_client(TripCommand, fms.contract.name("force_ship"))
    assert client.wait_for_service(timeout_sec=10.0)
    reply = client.call_async(TripCommand.Request(trip_id="O-007"))
    assert wait_for(reply.done, 10.0), "no reply: the callback raised"
    assert asked == ["O-007"]
    assert (reply.result().accepted, reply.result().reason) == answer


def test_an_event_is_logged_at_its_own_level(graph, monkeypatch):
    """rclpy가 한 호출 위치의 두 번째 심각도를 거부하므로 실제 로거를 먼저 부른다."""
    fms = graph.fms
    events = [EventOut(level, f"CODE_{level}", "detail") for level in ("INFO", "WARN", "ERROR")]
    for event in events * 2:
        fms._log_event(event)

    logged = []

    class Recorder:
        def info(self, text):
            logged.append(("info", text))

        def warning(self, text):
            logged.append(("warning", text))

        def error(self, text):
            logged.append(("error", text))

    monkeypatch.setattr(fms, "get_logger", Recorder)
    for event in events:
        fms._log_event(event)
    assert [level for level, _ in logged] == ["info", "warning", "error"]
    assert all(event.code in text for event, (_, text) in zip(events, logged, strict=True))


def test_a_robot_goal_with_no_server_is_not_a_rejection_and_an_arm_goal_still_is():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]"])
    node = FleetManagerNode()
    try:
        node.core = MagicMock()
        client = MagicMock()
        client.server_is_ready.return_value = False
        node._send(client, None, ("pode1", 7), node._move_done, node._robot_rejected)
        node._send(client, None, ("pode1", 8), node._dock_done, node._robot_rejected)
        assert [c.args for c in node.core.on_goal_not_sent.call_args_list] == [
            ("pode1", 7), ("pode1", 8)]
        node.core.on_goal_rejected.assert_not_called()
        node._send(client, None, ("workcell", 9), node._fill_done, node._fill_rejected)
        node.core.on_fill_result.assert_called_once_with(9, False, {}, {}, "REJECTED")
        node._send(client, None, ("arm_ship", 10), node._swap_done, node._swap_rejected)
        node.core.on_swap_result.assert_called_once_with(10, False, False, False, "REJECTED")
        assert node.core.on_goal_not_sent.call_count == 2
        client.send_goal_async.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_retreat_command_goes_to_the_retreat_action_and_its_result_back_to_the_core():
    """RetreatCmd는 /{pode}/retreat goal이 되고 결과는 on_retreat_result로 간다."""
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]"])
    node = FleetManagerNode()
    try:
        node.core = MagicMock()
        client = MagicMock()
        node._retreat["pode1"] = client
        path = ((1.0, 2.0, 0.0), (1.0, 1.6, 0.0))
        node._dispatch(RetreatCmd("pode1", 7, 0.4, 0.05, path), 1.0)
        (goal,), _ = client.send_goal_async.call_args
        assert isinstance(goal, Retreat.Goal)
        assert (goal.distance_m, goal.speed_mps) == (pytest.approx(0.4), pytest.approx(0.05))
        assert [(p.position.x, p.position.y) for p in goal.path] == [(1.0, 2.0), (1.0, 1.6)]
        node._retreat_done(("pode1", 7),
                           Retreat.Result(success=False, reason="OBSTACLE_BEHIND"))
        node.core.on_retreat_result.assert_called_once_with("pode1", 7, False,
                                                            "OBSTACLE_BEHIND")
        client.server_is_ready.return_value = False
        node._dispatch(RetreatCmd("pode1", 8, 0.15, 0.08), 2.0)
        node.core.on_goal_not_sent.assert_called_once_with("pode1", 8)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_retreat_result_with_a_core_that_has_no_handler_yet_is_logged_and_dropped():
    """on_retreat_result가 없어도 노드는 죽지 않는다."""
    warnings = []
    node = SimpleNamespace(core=SimpleNamespace(),
                           get_logger=lambda: SimpleNamespace(warning=warnings.append))
    FleetManagerNode._retreat_done(node, ("pode1", 7), Retreat.Result(success=True))
    assert len(warnings) == 1 and "on_retreat_result" in warnings[0]


def test_a_lost_result_of_a_robot_goal_is_no_rejection():
    """로봇 goal의 결과를 잃으면 거절로 넘기지 않는다.

    취소한 goal은 취소를 남기고, 취소하지 않은 goal은 코어에 결과 모름(on_goal_lost, IR-62)으로 넘긴다.
    팔 goal은 그대로 거절이다.
    """
    def broken():
        raise RuntimeError("result lost")

    def robot_rejected(key):
        rejected.append(key)

    def arm_rejected(key):
        rejected.append(key)

    key = ("pode1", 7)
    for canceled, on_rejected, want_rejected, want_lost in (
            (True, robot_rejected, [], []),
            (False, robot_rejected, [], [key]),
            (True, arm_rejected, [key], [])):
        goals = GoalTracker()
        goals.sent(key)
        goals.accepted(key, SimpleNamespace(cancel_goal_async=lambda: None))
        if canceled:
            goals.cancel(key, 0.0)
        lost = []
        node = SimpleNamespace(_goals=goals, _robot_rejected=robot_rejected,
                               core=SimpleNamespace(on_goal_lost=lambda *k: lost.append(k)),
                               get_logger=lambda: SimpleNamespace(error=lambda msg: None))
        rejected = []
        FleetManagerNode._goal_result(node, key, lambda k, r: None, on_rejected,
                                      SimpleNamespace(result=broken))
        assert (rejected, lost) == (want_rejected, want_lost)
        assert goals.canceling(key) == (canceled and on_rejected is robot_rejected)
        if want_lost:  # 코어가 나중에 내는 취소는 아직 로봇에 닿는다
            asked = []
            goals._handles[key] = SimpleNamespace(cancel_goal_async=lambda: asked.append(1))
            goals.cancel(key, 1.0)
            assert asked == [1]


def test_a_dropped_cancel_of_a_robot_goal_is_told_to_the_core_as_lost():
    """결과를 잃은 취소를 그만둘 때 코어에 알린다."""
    key = ("pode1", 7)
    goals = GoalTracker(rand=lambda: 0.5)  # 지연 배수 1.0
    goals.sent(key)
    goals.accepted(key, SimpleNamespace(cancel_goal_async=lambda: None))
    goals.cancel(key, 0.0)
    goals.lost(key)
    params = SimpleNamespace(cancel_retry_s=1.0, cancel_warn_after=2, robot_timeout_settle_s=30.0)
    events = []
    node = SimpleNamespace(_goals=goals, core=MagicMock(params=params),
                           _dispatch=lambda command, now: events.append(command.code))
    for now in (1.0, 2.0):
        FleetManagerNode._retry_cancels(node, now)
    assert events == ["CANCEL_UNCONFIRMED"]
    node.core.on_goal_lost.assert_not_called()
    FleetManagerNode._retry_cancels(node, 3.0)
    node.core.on_goal_lost.assert_called_once_with("pode1", 7)
    FleetManagerNode._retry_cancels(node, 4.0)
    assert node.core.on_goal_lost.call_count == 1


def test_the_heartbeat_comes_twice_a_second(graph):
    assert wait_for(lambda: len(graph.beats) >= 5, 20.0), graph.beats
    gaps = [b - a for a, b in zip(graph.beats, graph.beats[1:], strict=False)]
    assert 0.3 <= sum(gaps) / len(gaps) <= 0.8, gaps


def test_the_heartbeat_goes_out_with_the_reliable_profile_of_the_contract(graph):
    """heartbeat QoS는 계약에서 읽는다."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    topic, me = graph.fms.contract.name("heartbeat"), graph.fms.get_name()

    def mine():
        return [i for i in graph.get_publishers_info_by_topic(topic) if i.node_name == me]

    assert wait_for(mine, 10.0), "the node has no heartbeat publisher"
    qos = mine()[0].qos_profile
    assert (qos.reliability, qos.durability) == (ReliabilityPolicy.RELIABLE,
                                                 DurabilityPolicy.VOLATILE)
    # depth는 발행자 객체에서 본다
    depths = [p.qos_profile.depth for p in graph.fms.publishers if p.topic_name == topic]
    assert depths == [1]


def test_stop_cancels_the_fill_on_its_server_and_run_sends_what_remains(graph):
    graph.hold_fill.set()
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.fills), "the fill never reached the server"
    publish(graph, "STOP", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.canceled_fills), "the cancel never reached the server"
    graph.hold_fill.clear()
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.reports), f"fills {graph.fills}"
    assert graph.fills[1][3] == {"C": 2}, "A was placed before the stop"
    assert graph.reports[0].status == "SHIPPED"


def test_a_stop_answered_with_the_status_alone_is_no_failed_attempt(graph):
    """서버가 canceled()를 부르고 reason을 비워 둬도 goal 상태로 충분하다."""
    graph.hold_fill.set()
    graph.fill_cancel_reason = ""
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.fills), "the fill never reached the server"
    publish(graph, "STOP", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.canceled_fills), "the cancel never reached the server"
    orchestrator = graph.fms.core.orchestrator
    assert wait_for(lambda: orchestrator.robot_view("pode1")["placed"]), "no result came in"
    assert graph.fms.core.arm.attempts("pode1") == 0


def events_of(world, code):
    return [e for e in world.events if e.code == code]


def test_a_refused_cancel_is_sent_again_and_warned_about_once_until_the_result_comes(graph):
    # 경고 주기 수는 이 시험의 대상이 아니다. 노드가 읽는 값만 줄여 10 s 안에 경고를 본다
    fms = graph.fms
    fms.core.params = dataclasses.replace(fms.core.params, cancel_warn_after=7)
    graph.hold_fill.set()
    graph.refuse_fill_cancel = True
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.fills), "the fill never reached the server"
    publish(graph, "STOP", {"A": 1, "C": 2})
    assert wait_for(lambda: events_of(graph, "CANCEL_UNCONFIRMED"), 10.0), graph.fill_cancel_asks
    assert graph.fill_cancel_asks >= 3
    event = events_of(graph, "CANCEL_UNCONFIRMED")[0]
    assert (event.level, event.source) == ("WARN", "workcell")
    assert "goal" in event.detail
    asks = graph.fill_cancel_asks
    assert wait_for(lambda: graph.fill_cancel_asks > asks, 5.0), "it stopped trying"
    graph.hold_fill.clear()
    assert wait_for(lambda: not graph.fms._goals._cancels, 5.0), "the result ended nothing"
    asks = graph.fill_cancel_asks
    time.sleep(1.5)
    assert graph.fill_cancel_asks == asks, "the result came, so nobody has to ask any more"
    assert len(events_of(graph, "CANCEL_UNCONFIRMED")) == 1


def test_a_report_with_no_reply_yet_is_sent_again_and_ends_with_the_reply(graph):
    """central이 매번 재전송보다 늦게(응답 한도 3 s 안) 답해도 보고는 끝난다(IR-41)."""
    graph.report_delay_s = 2.5  # report_retry_s × 1.2보다 길고 응답 한도 3 s보다 짧다
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: len(graph.reports) >= 2), f"reports {graph.reports}"
    assert {r.order_id for r in graph.reports} == {"O-001"}
    assert wait_for(lambda: not graph.fms._reports._items, 10.0), "the reply ended nothing"
    sent = len(graph.reports)
    time.sleep(2.5)
    assert len(graph.reports) == sent, "the reply came, so nothing goes out again"
    assert not events_of(graph, "ORDER_REPORT_REFUSED")


def test_an_accepted_report_is_sent_once(graph):
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: graph.reports), f"fills {graph.fills}, swaps {graph.swaps}"
    assert wait_for(lambda: not graph.fms._reports._items, 5.0), "the reply ended nothing"
    time.sleep(2.5)
    assert len(graph.reports) == 1


def test_a_refused_report_is_dropped_with_one_error_naming_the_order(graph):
    graph.report_accepted = False
    publish(graph, "RUN", {"A": 1, "C": 2})
    assert wait_for(lambda: events_of(graph, "ORDER_REPORT_REFUSED")), f"{graph.reports}"
    event = events_of(graph, "ORDER_REPORT_REFUSED")[0]
    assert event.level == "ERROR" and "O-001" in event.detail
    assert not graph.fms._reports._items
    time.sleep(2.5)
    assert len(graph.reports) == 1, "a refusal is an answer, so it does not go out again"
    assert len(events_of(graph, "ORDER_REPORT_REFUSED")) == 1


def test_a_failed_reply_keeps_the_report_for_sending_again():
    report = SimpleNamespace(order_id="O-001")
    outbox = ReportOutbox(rand=lambda: 0.5)
    outbox.add(report)
    [(report_id, _)] = outbox.due(0.0, True, 2.0, 4)
    future = Future()
    future.set_exception(RuntimeError("central went away"))
    node = SimpleNamespace(_reports=outbox, get_logger=lambda: SimpleNamespace(warning=print))
    FleetManagerNode._report_answered(node, report_id, report, future)
    assert outbox.due(2.0, True, 2.0, 4) == [(report_id, report)]


class _ReportClient:
    """call_async가 답 없는 future를 내고 지운 요청을 적는다."""

    def __init__(self):
        self.sent = []
        self.removed = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.sent.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def report_node(limit=4):
    outbox = ReportOutbox(rand=lambda: 0.5)
    events = []
    node = SimpleNamespace(
        _reports=outbox, _report_client=_ReportClient(),
        core=SimpleNamespace(params=SimpleNamespace(report_retry_s=2.0,
                                                    report_max_in_flight=limit)),
        get_logger=lambda: SimpleNamespace(warning=print),
        _now=lambda: 0.0, _dispatch=lambda command, now: events.append(command.code),
        _post=lambda handler, *args: handler(*args))  # 받은 편지함을 바로 비운 것과 같다
    node._report_answered = partial(FleetManagerNode._report_answered, node)
    return node, events


def report_cmd(order_id):
    return ReportOrderCmd(order_id, "pode1", "SHIPPED", {"A": 1}, {}, "")


def answer(future, accepted):
    future.set_result(ReportOrder.Response(accepted=accepted))


def test_a_resent_report_removes_only_requests_past_the_reply_limit():
    """IR-41: 응답 한도(3 s)를 넘긴 요청만 클라이언트에서 지워 대기 표가 늘지 않는다."""
    node, _ = report_node()
    node._reports.add(report_cmd("O-001"))
    client = node._report_client
    FleetManagerNode._flush_reports(node, 0.0)
    FleetManagerNode._flush_reports(node, 2.0)
    assert client.removed == [], "the first request is still inside the reply limit"
    FleetManagerNode._flush_reports(node, 4.0)
    assert client.removed == client.sent[:1]
    answer(client.sent[1], True)  # 한도 안에 남은 앞선 요청의 답이 끝낸다
    assert not node._reports._items
    assert client.removed == [client.sent[0], client.sent[2]], "the other waiting request goes"


def test_a_request_past_the_reply_limit_is_dropped_without_a_resend():
    node, _ = report_node()
    node.core.params.report_retry_s = 10.0
    node._reports.add(report_cmd("O-001"))
    client = node._report_client
    FleetManagerNode._flush_reports(node, 0.0)
    FleetManagerNode._flush_reports(node, 2.9)
    assert client.removed == []
    FleetManagerNode._flush_reports(node, 3.0)
    assert client.removed == client.sent[:1] and len(client.sent) == 1


def test_a_late_answer_to_a_done_report_is_harmless():
    node, events = report_node()
    node._reports.add(report_cmd("O-001"))
    client = node._report_client
    FleetManagerNode._flush_reports(node, 0.0)
    FleetManagerNode._flush_reports(node, 2.0)
    answer(client.sent[1], True)
    answer(client.sent[0], False)  # 지웠어도 늦게 끝난 future
    assert events == [] and not node._reports._items
    assert client.removed == client.sent[:1]
    FleetManagerNode._flush_reports(node, 10.0)
    assert len(client.sent) == 2


def test_the_node_keeps_reports_over_the_limit_for_a_later_flush():
    node, _ = report_node(limit=1)
    for order in ("O-001", "O-002"):
        node._reports.add(report_cmd(order))
    client = node._report_client
    FleetManagerNode._flush_reports(node, 0.0)
    assert len(client.sent) == 1
    answer(client.sent[0], True)
    FleetManagerNode._flush_reports(node, 0.1)
    assert len(client.sent) == 2


# ---- 운영 모드 (IR-73) ----
def serve_set_mode(world, accept=True):
    """pode1의 set_mode 서버. 받아들이면 그 모드를 상태에 싣는다."""
    asked = []

    def on_set_mode(request, response):
        asked.append(request.mode)
        if accept:
            world.mode = request.mode
        response.accepted, response.request_id = accept, request.request_id
        response.reason = "" if accept else "MODE_MISMATCH"
        return response

    contract = world.fms.contract
    world.create_service(SetMode, contract.name("robot_set_mode", robot="pode1"), on_set_mode,
                         callback_group=world._group)
    return asked


def send_system_mode(world, mode):
    contract = world.fms.contract
    pub = world.create_publisher(SystemMode, contract.name("system_mode"),
                                 to_ros_qos(contract.qos("system_mode")))
    pub.publish(SystemMode(mode=mode))
    return pub


def arms_report_mode(world, mode):
    """작업대 두 팔이 그 모드를 보고한다 (담기는 팔이 모드를 보고한 뒤 나간다)."""
    world.arm_mode = mode


def event_codes(world):
    return [e.code for e in list(world.events)]


def test_nothing_is_asked_before_the_system_mode_comes(graph):
    asked = serve_set_mode(graph)
    publish(graph, "RUN", {"A": 1})
    assert wait_for(lambda: graph.fills, 20.0), "the robot still takes trips without a mode"
    assert asked == []


def test_the_system_mode_is_sent_to_the_robot_and_it_keeps_working(graph):
    asked = serve_set_mode(graph)
    arms_report_mode(graph, "TEST")
    send_system_mode(graph, "TEST")
    assert wait_for(lambda: asked == ["TEST"], 10.0), asked
    assert wait_for(lambda: graph.mode == "TEST", 5.0)
    publish(graph, "RUN", {"A": 1})
    assert wait_for(lambda: graph.fills, 20.0)
    assert "ROBOT_MODE_MISMATCH" not in event_codes(graph)


def test_a_robot_that_refuses_the_mode_gets_no_new_trip(graph):
    asked = serve_set_mode(graph, accept=False)
    send_system_mode(graph, "TEST")
    assert wait_for(lambda: "ROBOT_MODE_MISMATCH" in event_codes(graph), 15.0), asked
    assert len(asked) >= 2
    publish(graph, "RUN", {"A": 1})
    time.sleep(3.0)
    assert graph.fills == []
