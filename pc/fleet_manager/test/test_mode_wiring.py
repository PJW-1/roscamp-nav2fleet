"""담기 목표를 작업대 팔의 모드 보고에 묶는 것(IR-72, PI-20)을 실제 노드로 본다. 노드를 돌리지 않고
보내기만 가로챈다. 로봇 네트워크와 분리된 도메인에서만 돈다."""
import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.commands import CancelCmd, FillCmd  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")


# ---- 담기 목표: 작업대 팔이 시스템 모드를 보고하기 전에는 보내지 않는다 (IR-72, PI-20) ----
FILL = FillCmd(goal_id=7, robot="pode1", station_id="WORK", order_id="O-1", items={"choco_bar": 1})


def node_with_spied_sends():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]"])
    fms = FleetManagerNode()
    sent, results, given_up = [], [], set()
    fms._send = lambda client, goal, key, *args, **kwargs: sent.append(key)
    fms.core.arm.holds = lambda goal_id: goal_id not in given_up
    fms.core.on_fill_result = (
        lambda goal_id, success, placed, missing, reason="", **kwargs:
        results.append((goal_id, success, reason)))

    def stop():
        fms.destroy_node()
        rclpy.shutdown()

    return fms, sent, results, given_up, stop


def arms_report(fms, left, right):
    now = time.monotonic()
    fms._mode_sync.on_state("arm_left", left, "b-left", now)
    fms._mode_sync.on_state("arm_right", right, "b-right", now)


def test_a_fill_waits_until_both_arms_report_the_system_mode():
    fms, sent, _, _, stop = node_with_spied_sends()
    try:
        fms._mode_sync.on_system_mode("TEST")
        arms_report(fms, "TEST", "UNKNOWN")
        fms._dispatch(FILL, time.monotonic())
        fms._release_fills()
        assert sent == []
        arms_report(fms, "TEST", "TEST")
        fms._release_fills()
        fms._release_fills()
        assert sent == [("workcell", 7)]
    finally:
        stop()


def test_a_fill_goes_at_once_while_the_system_mode_is_unknown():
    fms, sent, _, _, stop = node_with_spied_sends()
    try:
        arms_report(fms, "UNKNOWN", "UNKNOWN")
        fms._dispatch(FILL, time.monotonic())
        assert sent == [("workcell", 7)]
    finally:
        stop()


def test_a_held_fill_that_is_canceled_ends_as_canceled_and_is_never_sent():
    fms, sent, results, _, stop = node_with_spied_sends()
    try:
        fms._mode_sync.on_system_mode("TEST")
        arms_report(fms, "UNKNOWN", "UNKNOWN")
        fms._dispatch(FILL, time.monotonic())
        fms._dispatch(CancelCmd("workcell", 7), time.monotonic())
        arms_report(fms, "TEST", "TEST")
        fms._release_fills()
        assert sent == [] and results == [(7, False, "CANCELED")]
    finally:
        stop()


def test_a_held_fill_the_core_gave_up_on_is_dropped_and_never_sent():
    fms, sent, results, given_up, stop = node_with_spied_sends()
    try:
        fms._mode_sync.on_system_mode("TEST")
        arms_report(fms, "UNKNOWN", "UNKNOWN")
        fms._dispatch(FILL, time.monotonic())
        given_up.add(7)  # 팔 ERROR로 코어가 작업을 포기함 (취소 명령 없음)
        arms_report(fms, "TEST", "TEST")
        fms._release_fills()
        assert sent == [] and results == [] and fms._held_fills == []
    finally:
        stop()
