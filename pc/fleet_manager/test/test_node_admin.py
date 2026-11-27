"""관리 명령 서버를 실제 그래프로 시험한다: 곧바로 답, 같은 번호, 로봇 부르기, ReportCommand.

로봇 쪽 서버는 직접 짠다 (받음 · 거절 · 늦은 답). 로봇 네트워크와 분리된 도메인에서만 돈다.
"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.contract import to_ros_qos  # noqa: E402
from fleet_interfaces.action import Dock, MoveTo  # noqa: E402
from fleet_interfaces.msg import ArmState, FleetEvent, RobotState, SystemMode  # noqa: E402
from fleet_interfaces.srv import (  # noqa: E402
    ConfirmClear,
    ForceInput,
    Reinstate,
    RobotCommand,
    SetLock,
    SetPause,
    TripCommand,
)
from fleet_core.common.commands import RESUME_REFILL, RESUME_SET_PAUSE_OFF  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")

FAST = ["-p", "robot_call_try_s:=0.3", "-p", "robot_call_gap_s:=0.1", "-p",
        "robot_call_tries:=3", "-p", "robot_call_confirm_s:=3.0"]


class Robots(Node):
    """pode1과 arm_left: 상태를 내고 관리 서버로 답한다. answer: accept · reject · late."""

    def __init__(self, contract, pose):
        super().__init__("admin_robots")
        group = ReentrantCallbackGroup()
        self.contract, self.group = contract, group
        self.answer = "accept"
        self.answer_of = {}  # 명령마다 다른 답 (없으면 answer)
        self.paused = []  # set_pause로 받은 (팔, paused)
        self.late_s = 1.6
        self.pod_state = RobotState.STATE_IDLE
        self.lock = True
        self.reported_lock = None  # None이면 lock을 그대로 싣는다
        self.arm = ArmState(arm_id="arm_left", boot_id="a1", state=ArmState.STATE_IDLE,
                            lock=True)
        self.asked = []
        self.events = []
        name, qos = contract.name, lambda key: to_ros_qos(contract.qos(key))
        self.create_subscription(FleetEvent, name("events"), self.events.append, qos("events"))
        self._pod_pub = self.create_publisher(RobotState, name("robot_state", pode="pode1"),
                                              qos("robot_state"))
        self._arm_pub = self.create_publisher(ArmState, name("arm_state", arm="arm_left"),
                                              qos("arm_state"))
        self._mode_pub = self.create_publisher(SystemMode, name("system_mode"),
                                               qos("system_mode"))
        self.pose = pose
        # goal이 닿는 로봇이어야 지도에 오른다. 일은 주지 않으니 goal은 오지 않는다
        self._actions = [ActionServer(self, kind, name(key, pode="pode1"),
                                      lambda handle, kind=kind: kind.Result(),
                                      callback_group=group)
                         for kind, key in ((MoveTo, "move_to"), (Dock, "dock"))]
        self.create_timer(0.02, self._publish, callback_group=group)
        self.create_service(SetLock, name("robot_set_lock", robot="pode1"),
                            self._serve("set_lock"), callback_group=group)
        self.create_service(SetLock, name("robot_set_lock", robot="arm_left"),
                            self._serve("arm_set_lock"), callback_group=group)
        self.create_service(RobotCommand, name("pode_charge_end", pode="pode1"),
                            self._serve("charge_end"), callback_group=group)
        self.create_service(ForceInput, name("robot_force", robot="pode1"),
                            self._serve("force"), callback_group=group)
        self.create_service(RobotCommand, name("arm_fault_resolved", arm="arm_left"),
                            self._serve("fault_resolved"), callback_group=group)
        for arm in ("arm_left", "arm_right"):
            self.create_service(SetPause, name("arm_set_pause", arm=arm),
                                self._serve(f"set_pause {arm}"), callback_group=group)

    def mode(self, mode):
        self._mode_pub.publish(SystemMode(mode=mode))

    def _publish(self):
        lock = self.lock if self.reported_lock is None else self.reported_lock
        msg = RobotState(robot_id="pode1", state=self.pod_state, boot_id="b1", lock=lock)
        msg.pose.x, msg.pose.y = self.pose
        self._pod_pub.publish(msg)
        self._arm_pub.publish(self.arm)

    def _serve(self, what):
        def serve(request, response):
            self.asked.append((what, request.request_id))
            if what.startswith("set_pause"):
                self.paused.append((what.split()[1], request.paused))
            answer = self.answer_of.get(what, self.answer)
            if answer == "late":
                time.sleep(self.late_s)
            accept = answer != "reject"
            if accept and what == "set_lock":
                self.lock = request.locked
            response.accepted, response.request_id = accept, request.request_id
            response.reason = "" if accept else "NOT_STOPPED"
            response.detail = "" if accept else "robot says no"
            return response
        return serve


@pytest.fixture
def graph():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", "tick_hz:=50.0", *FAST])
    fms = FleetManagerNode()
    robots = Robots(fms.contract, fms.site.node_pose("P-1"))
    robots.fms = fms
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(fms)
    executor.add_node(robots)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert wait_for(lambda: fms.core.traffic.robot_view("pode1")["on_map"]), "never on the map"
    yield robots
    executor.shutdown()
    thread.join(timeout=5)
    robots.destroy_node()
    fms.destroy_node()
    rclpy.shutdown()


def wait_for(predicate, timeout_s=20.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def call(graph, srv, service, **fields):
    client = graph.create_client(srv, graph.fms.contract.name(service),
                                 callback_group=graph.group)
    assert client.wait_for_service(timeout_sec=10.0)
    future = client.call_async(srv.Request(**fields))
    assert wait_for(future.done, 10.0), "no reply"
    return future.result()


def reports(graph, request_id):
    return [r for r in list(graph.fms.command_reports) if r[0] == request_id]


def results(graph, request_id):
    return [r[3] for r in reports(graph, request_id)]


def released(graph):
    return graph.fms.core.traffic.lock_released("pode1")


# ---- set_lock ----
def test_a_release_is_answered_at_once_sent_once_and_reported_accepted(graph):
    reply = call(graph, SetLock, "set_lock", request_id="L-1", robot_id="pode1", locked=False)
    assert (reply.accepted, reply.reason, reply.request_id) == (True, "", "L-1")
    assert released(graph)  # 받자마자 배정에서 뺀다
    assert wait_for(lambda: results(graph, "L-1") == ["ACCEPTED"], 5.0), reports(graph, "L-1")
    assert reports(graph, "L-1")[0][1:3] == ("set_lock", "pode1")
    again = call(graph, SetLock, "set_lock", request_id="L-1", robot_id="pode1", locked=False)
    assert again.accepted
    time.sleep(0.5)
    assert graph.asked.count(("set_lock", "L-1")) == 1
    assert results(graph, "L-1") == ["ACCEPTED"]
    assert wait_for(lambda: "ROBOT_LOCK_RELEASED" in [e.code for e in graph.events], 5.0)
    back = call(graph, SetLock, "set_lock", request_id="L-2", robot_id="pode1", locked=True)
    assert back.accepted
    assert wait_for(lambda: not released(graph), 5.0)


def test_a_moving_robot_is_not_stopped_and_nothing_is_sent(graph):
    graph.pod_state = RobotState.STATE_MOVING
    assert wait_for(lambda: graph.fms._pode_states["pode1"].state == "MOVING", 5.0)
    reply = call(graph, SetLock, "set_lock", request_id="L-3", robot_id="pode1", locked=False)
    assert (reply.accepted, reply.reason) == (False, "NOT_STOPPED")
    assert "MOVING" in reply.detail
    assert reports(graph, "L-3") == [("L-3", "set_lock", "pode1", "REJECTED", "NOT_STOPPED",
                                      reply.detail)]
    assert not released(graph) and graph.asked == []


def test_an_unknown_robot_is_unknown_target(graph):
    reply = call(graph, SetLock, "set_lock", request_id="L-4", robot_id="pode9", locked=False)
    assert (reply.accepted, reply.reason) == (False, "UNKNOWN_TARGET")


def test_an_arm_holding_an_item_needs_confirm_drop(graph):
    graph.arm.holding = True
    assert wait_for(lambda: graph.fms._arm_states.get("arm_left", (None,))[0] is not None
                    and graph.fms._arm_states["arm_left"][0].holding, 5.0)
    reply = call(graph, SetLock, "set_lock", request_id="A-1", robot_id="arm_left",
                 locked=False)
    assert reply.reason == "HOLDING_ITEM"
    reply = call(graph, SetLock, "set_lock", request_id="A-2", robot_id="arm_left",
                 locked=False, confirm_drop=True)
    assert reply.accepted
    assert wait_for(lambda: results(graph, "A-2") == ["ACCEPTED"], 5.0)


def test_a_robot_refusal_is_rejected_and_the_release_is_undone(graph):
    graph.answer = "reject"
    reply = call(graph, SetLock, "set_lock", request_id="L-5", robot_id="pode1", locked=False)
    assert reply.accepted  # fleet 판단은 받음
    assert wait_for(lambda: results(graph, "L-5") == ["REJECTED"], 5.0)
    r = reports(graph, "L-5")[0]
    assert r[4] == "ROBOT_REJECTED" and r[5] == "NOT_STOPPED robot says no"
    assert wait_for(lambda: not released(graph), 5.0)


def test_a_late_answer_turns_unconfirmed_into_accepted(graph):
    graph.answer = "late"
    call(graph, SetLock, "set_lock", request_id="L-6", robot_id="pode1", locked=False)
    graph.reported_lock = True  # 관측으로 먼저 정하지 않게 상태는 잠김으로 둔다
    assert wait_for(lambda: results(graph, "L-6") == ["UNCONFIRMED", "ACCEPTED"], 6.0), \
        reports(graph, "L-6")
    assert reports(graph, "L-6")[0][4] == "ROBOT_TIMEOUT"
    assert released(graph)


def test_an_observed_lock_confirms_an_unconfirmed_release(graph):
    graph.answer = "late"
    graph.late_s = 5.0
    graph.reported_lock = True
    call(graph, SetLock, "set_lock", request_id="L-7", robot_id="pode1", locked=False)
    assert wait_for(lambda: results(graph, "L-7") == ["UNCONFIRMED"], 4.0), reports(graph, "L-7")
    graph.reported_lock = False
    assert wait_for(lambda: results(graph, "L-7") == ["UNCONFIRMED", "ACCEPTED"], 2.0), \
        reports(graph, "L-7")
    assert "robot state" in reports(graph, "L-7")[-1][5]


def test_no_result_in_time_is_timeout_with_late_command_result(graph):
    graph.answer = "late"
    graph.late_s = 5.0
    graph.reported_lock = True
    call(graph, SetLock, "set_lock", request_id="L-8", robot_id="pode1", locked=False)
    assert wait_for(lambda: results(graph, "L-8") == ["UNCONFIRMED", "TIMEOUT"], 6.0), \
        reports(graph, "L-8")
    assert wait_for(lambda: "LATE_COMMAND_RESULT" in [e.code for e in graph.events], 3.0)
    assert released(graph)  # 결과 모름: 풀렸을 수 있어 빼 둔 채 둔다


# ---- 시험 모드 명령 ----
def test_charge_end_and_force_input_need_the_test_mode(graph):
    reply = call(graph, RobotCommand, "charge_end", request_id="C-1", robot_id="pode1")
    assert (reply.accepted, reply.reason) == (False, "NOT_TEST_MODE")
    reply = call(graph, ForceInput, "force_input", request_id="F-1", robot_id="pode1",
                 kind="CONDITION", key="battery", value="20")
    assert reply.reason == "NOT_TEST_MODE"
    graph.mode("TEST")
    assert wait_for(lambda: graph.fms._mode_sync.system_mode == "TEST", 5.0)
    assert call(graph, RobotCommand, "charge_end", request_id="C-2", robot_id="pode1").accepted
    assert call(graph, ForceInput, "force_input", request_id="F-2", robot_id="pode1",
                kind="CONDITION", key="battery", value="20").accepted
    assert wait_for(lambda: results(graph, "C-2") == ["ACCEPTED"]
                    and results(graph, "F-2") == ["ACCEPTED"], 5.0)
    assert ("charge_end", "C-2") in graph.asked and ("force", "F-2") in graph.asked
    assert reports(graph, "F-2")[0][1] == "force_input"


def test_an_accepted_charge_end_ends_the_charge_stay_in_the_core(graph):
    ended = []
    graph.fms.core.end_charge = lambda robot, detail="": ended.append(robot)
    graph.mode("TEST")
    assert wait_for(lambda: graph.fms._mode_sync.system_mode == "TEST", 5.0)
    assert call(graph, RobotCommand, "charge_end", request_id="C-9", robot_id="pode1").accepted
    assert wait_for(lambda: results(graph, "C-9") == ["ACCEPTED"], 5.0)
    assert wait_for(lambda: ended == ["pode1"], 5.0), ended


def test_fault_resolved_goes_to_a_phody_arm_only(graph):
    reply = call(graph, RobotCommand, "fault_resolved", request_id="R-1", robot_id="pode1")
    assert reply.reason == "UNKNOWN_TARGET"
    reply = call(graph, RobotCommand, "fault_resolved", request_id="R-2", robot_id="arm_left")
    assert reply.accepted
    assert wait_for(lambda: results(graph, "R-2") == ["ACCEPTED"], 5.0)


def test_fault_resolved_unconfirmed_is_confirmed_when_the_arm_leaves_error(graph):
    graph.arm.state = ArmState.STATE_ERROR
    assert wait_for(lambda: graph.fms._in_error("arm_left"), 5.0)
    graph.answer = "late"
    graph.late_s = 5.0
    call(graph, RobotCommand, "fault_resolved", request_id="R-3", robot_id="arm_left")
    assert wait_for(lambda: results(graph, "R-3") == ["UNCONFIRMED"], 4.0)
    graph.arm.state = ArmState.STATE_IDLE
    assert wait_for(lambda: results(graph, "R-3") == ["UNCONFIRMED", "ACCEPTED"], 2.0)


# ---- 로봇에 보내지 않는 명령 (W-32) ----
def test_confirm_clear_and_reinstate_report_once_with_codes(graph):
    fms = graph.fms
    bad = call(graph, ConfirmClear, "confirm_clear", request_id="X-1", robot_id="pode1",
               zone_ids=["NOPE"])
    assert (bad.accepted, bad.reason) == (False, "NOT_FOUND") and "NOPE" in bad.detail
    assert reports(graph, "X-1") == [("X-1", "confirm_clear", "pode1", "REJECTED", "NOT_FOUND",
                                      bad.detail)]
    zones = list(fms.core.zones.held_by("pode1"))
    ok = call(graph, ConfirmClear, "confirm_clear", request_id="X-2", robot_id="pode1",
              zone_ids=zones)
    assert ok.accepted and list(ok.released_zones) == zones
    again = call(graph, ConfirmClear, "confirm_clear", request_id="X-2", robot_id="pode1",
                 zone_ids=zones)
    assert again.accepted and list(again.released_zones) == zones
    assert results(graph, "X-2") == ["ACCEPTED"]
    call(graph, SetLock, "set_lock", request_id="X-3", robot_id="pode1", locked=False)
    back = call(graph, Reinstate, "reinstate", request_id="X-4", robot_id="pode1",
                basket="EMPTY")
    assert (back.accepted, back.reason) == (False, "LOCK_RELEASED")
    assert results(graph, "X-4") == ["REJECTED"]


def test_trip_commands_report_once_and_unknown_ones_are_not_implemented(graph):
    reply = call(graph, TripCommand, "resume_fill", request_id="T-1", trip_id="O-1")
    assert (reply.accepted, reply.reason) == (False, "NOT_IMPLEMENTED")
    assert reports(graph, "T-1") == [("T-1", "resume_fill", "O-1", "REJECTED",
                                      "NOT_IMPLEMENTED", "주문 쪽 미구현")]
    reply = call(graph, TripCommand, "force_ship", request_id="T-2", trip_id="O-NOPE")
    assert (reply.accepted, reply.reason) == (False, "NOT_FOUND")
    assert results(graph, "T-2") == ["REJECTED"]
    for key in ("force_partial", "return_to_shelf", "resume_held", "requeue_trip"):
        assert call(graph, TripCommand, key, request_id=f"T-{key}", trip_id="O-1").reason == \
            "NOT_IMPLEMENTED"


def test_fault_resolved_on_an_arm_not_in_error_is_not_confirmed_by_its_state(graph):
    graph.answer = "late"
    graph.late_s = 5.0
    call(graph, RobotCommand, "fault_resolved", request_id="R-4", robot_id="arm_left")
    assert wait_for(lambda: results(graph, "R-4") == ["UNCONFIRMED", "TIMEOUT"], 6.0), \
        reports(graph, "R-4")


def test_an_empty_request_id_gets_its_own_id_and_each_call_is_sent(graph):
    first = call(graph, SetLock, "set_lock", request_id="", robot_id="pode1", locked=True)
    # 같은 로봇의 set_lock은 하나씩 받는다. 첫 결과가 나온 뒤 보낸다
    assert wait_for(lambda: results(graph, first.request_id)[-1:] == ["ACCEPTED"], 5.0), \
        reports(graph, first.request_id)
    second = call(graph, SetLock, "set_lock", request_id="", robot_id="pode1", locked=True)
    assert first.accepted and second.accepted
    assert first.request_id and second.request_id and first.request_id != second.request_id
    assert wait_for(lambda: len([a for a in graph.asked if a[0] == "set_lock"]) == 2, 5.0)


def test_a_second_set_lock_while_one_is_pending_is_not_ready(graph):
    graph.answer = "late"
    graph.late_s = 1.0
    call(graph, SetLock, "set_lock", request_id="O-1", robot_id="pode1", locked=False)
    second = call(graph, SetLock, "set_lock", request_id="O-2", robot_id="pode1", locked=True)
    assert (second.accepted, second.reason) == (False, "NOT_READY") and "O-1" in second.detail
    assert wait_for(lambda: results(graph, "O-1")[-1:] == ["ACCEPTED"], 5.0), \
        reports(graph, "O-1")
    third = call(graph, SetLock, "set_lock", request_id="O-3", robot_id="pode1", locked=True)
    assert third.accepted


def test_fault_resolved_on_an_arm_that_stays_in_error_ends_in_timeout(graph):
    graph.arm.state = ArmState.STATE_ERROR
    assert wait_for(lambda: graph.fms._in_error("arm_left"), 5.0)
    graph.answer = "late"
    graph.late_s = 5.0
    call(graph, RobotCommand, "fault_resolved", request_id="R-5", robot_id="arm_left")
    assert wait_for(lambda: results(graph, "R-5") == ["UNCONFIRMED", "TIMEOUT"], 6.0), \
        reports(graph, "R-5")
    assert reports(graph, "R-5")[-1][4] == "ROBOT_TIMEOUT"



# ---- [장애 처리 완료]: 보낼 때 ERROR였던 팔은 받았다는 답도 ERROR를 벗어나야 확정 ----
def blocked_by_left_error(graph):
    return any("arm_left ERROR" in why for why in graph.fms.core.arm._station_block.values())


def arm_left_in_error(graph):
    graph.arm.state = ArmState.STATE_ERROR
    assert wait_for(lambda: graph.fms._in_error("arm_left") and blocked_by_left_error(graph),
                    5.0)


def test_fault_resolved_accepted_while_the_arm_stays_in_error_is_not_confirmed(graph):
    arm_left_in_error(graph)
    call(graph, RobotCommand, "fault_resolved", request_id="R-6", robot_id="arm_left")
    time.sleep(1.5)
    assert results(graph, "R-6") == [] and blocked_by_left_error(graph)
    assert wait_for(lambda: results(graph, "R-6") == ["TIMEOUT"], 3.0), reports(graph, "R-6")
    assert reports(graph, "R-6")[-1][4] == "ROBOT_TIMEOUT"
    assert blocked_by_left_error(graph)


def test_fault_resolved_accepted_is_confirmed_when_the_arm_leaves_error(graph):
    arm_left_in_error(graph)
    call(graph, RobotCommand, "fault_resolved", request_id="R-7", robot_id="arm_left")
    time.sleep(0.5)
    assert results(graph, "R-7") == []
    graph.arm.state = ArmState.STATE_IDLE
    assert wait_for(lambda: results(graph, "R-7") == ["ACCEPTED"], 2.0), reports(graph, "R-7")
    assert wait_for(lambda: not blocked_by_left_error(graph), 2.0)


# ---- 관리자 [복구] (팔 관제 합의 ⑧) ----
def resume_answer(graph, answer):
    graph.fms.core.on_resume_arm = lambda target: answer


def test_a_resume_the_core_refuses_is_rejected_with_its_code_and_nothing_is_sent(graph):
    resume_answer(graph, ("", "NOT_PAUSED"))
    reply = call(graph, RobotCommand, "resume_arm", request_id="U-1", robot_id="workcell")
    assert (reply.accepted, reply.reason, reply.request_id) == (False, "NOT_PAUSED", "U-1")
    assert reports(graph, "U-1") == [("U-1", "resume_arm", "workcell", "REJECTED",
                                      "NOT_PAUSED", reply.detail)]
    time.sleep(0.3)
    assert graph.paused == []


def test_a_resume_with_set_pause_off_goes_to_both_arms_and_is_reported_once(graph):
    resume_answer(graph, (RESUME_SET_PAUSE_OFF, ""))
    reply = call(graph, RobotCommand, "resume_arm", request_id="U-2", robot_id="arm_left")
    assert (reply.accepted, reply.reason) == (True, "")
    assert wait_for(lambda: results(graph, "U-2") == ["ACCEPTED"], 5.0), reports(graph, "U-2")
    assert reports(graph, "U-2")[0][1:3] == ("resume_arm", "arm_left")
    assert sorted(graph.paused) == [("arm_left", False), ("arm_right", False)]
    assert ("set_pause arm_left", "U-2") in graph.asked
    again = call(graph, RobotCommand, "resume_arm", request_id="U-2", robot_id="arm_left")
    assert again.accepted
    time.sleep(0.3)
    assert len(graph.paused) == 2 and results(graph, "U-2") == ["ACCEPTED"]


def test_a_resume_with_refill_is_accepted_at_once_without_set_pause(graph):
    resume_answer(graph, (RESUME_REFILL, ""))
    reply = call(graph, RobotCommand, "resume_arm", request_id="U-3", robot_id="workcell")
    assert reply.accepted
    assert results(graph, "U-3") == ["ACCEPTED"]
    time.sleep(0.3)
    assert graph.paused == []


def test_one_arm_refusing_set_pause_makes_the_resume_rejected(graph):
    resume_answer(graph, (RESUME_SET_PAUSE_OFF, ""))
    graph.answer_of["set_pause arm_right"] = "reject"
    call(graph, RobotCommand, "resume_arm", request_id="U-4", robot_id="workcell")
    assert wait_for(lambda: results(graph, "U-4") == ["REJECTED"], 5.0), reports(graph, "U-4")
    r = reports(graph, "U-4")[0]
    assert r[4] == "ROBOT_REJECTED" and r[5].startswith("arm_right:"), r


def test_an_unanswered_set_pause_is_confirmed_when_the_paused_arm_leaves_paused(graph):
    graph.arm.state = ArmState.STATE_PAUSED
    assert wait_for(lambda: graph.fms._paused("arm_left"), 5.0)
    resume_answer(graph, (RESUME_SET_PAUSE_OFF, ""))
    graph.answer_of["set_pause arm_left"] = "late"
    graph.late_s = 5.0
    call(graph, RobotCommand, "resume_arm", request_id="U-5", robot_id="workcell")
    assert wait_for(lambda: results(graph, "U-5") == ["UNCONFIRMED"], 4.0), reports(graph, "U-5")
    assert reports(graph, "U-5")[0][5].startswith("arm_left:")
    # 앞 [복구]가 끝나기 전 다음 [복구]는 받지 않는다
    busy = call(graph, RobotCommand, "resume_arm", request_id="U-6", robot_id="workcell")
    assert (busy.accepted, busy.reason) == (False, "NOT_READY")
    graph.arm.state = ArmState.STATE_IDLE
    assert wait_for(lambda: results(graph, "U-5") == ["UNCONFIRMED", "ACCEPTED"], 2.0), \
        reports(graph, "U-5")


def test_an_unanswered_set_pause_on_an_arm_not_paused_ends_in_timeout(graph):
    resume_answer(graph, (RESUME_SET_PAUSE_OFF, ""))
    graph.answer_of["set_pause arm_left"] = "late"
    graph.late_s = 5.0
    call(graph, RobotCommand, "resume_arm", request_id="U-7", robot_id="workcell")
    assert wait_for(lambda: results(graph, "U-7") == ["UNCONFIRMED", "TIMEOUT"], 6.0), \
        reports(graph, "U-7")
    r = reports(graph, "U-7")[-1]
    assert r[4] == "ROBOT_TIMEOUT" and r[5].startswith("arm_left:"), r
