"""팔 관제 합의의 노드 몫: 팔 상태 입력(boot_id · lock), 상태 끊김, 담기 진행, 상태판 팔 줄, 확인 한도."""
import os
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fleet_core.common.commands import FillCmd  # noqa: E402
from fleet_core.fleet_map import FILL  # noqa: E402
from fleet_interfaces.action import FillBasket  # noqa: E402
from fleet_interfaces.msg import ArmState, RobotStatus  # noqa: E402
from fleet_manager import adapters  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402
from fleet_manager.robot_calls import CallParams  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")


class Spy:
    """코어를 감싸 부른 메서드와 인자를 남긴다."""

    def __init__(self, core):
        self._core, self.calls = core, []

    def __getattr__(self, name):
        value = getattr(self._core, name)
        if not callable(value):
            return value

        def call(*args):
            self.calls.append((name, args))
            return value(*args)
        return call

    def args(self, name):
        return [a for n, a in self.calls if n == name]


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]"])
    made = FleetManagerNode()
    made._silent_until = 0.0
    made.core = Spy(made.core)
    yield made
    made.destroy_node()
    rclpy.shutdown()


def arm(state="IDLE", **fields):
    return ArmState(arm_id="arm_left", state=state, mode="UNKNOWN", **fields)


def feed(node, msg, name="arm_left"):
    node._post(node._on_arm_state, name, msg)
    node._drain()


def row_of(node, name):
    msg = adapters.robots_to_msg(node.core.robots_view(), 1.0, node._pode_states,
                                 node._arm_rows())
    return next(r for r in msg.robots if r.robot_id == name)


# ---- 팔 상태: boot_id · lock도 넣고 바뀔 때만 ----
def test_boot_id_and_lock_reach_the_core_and_count_as_a_change(node):
    for msg in (arm(boot_id="b1", lock=True), arm(boot_id="b1", lock=True),
                arm(boot_id="b1", lock=False), arm(boot_id="b2", lock=False),
                arm(boot_id="b2", lock=False, holding=True)):
        feed(node, msg)
    assert node.core.args("on_arm_state") == [
        ("arm_left", "IDLE", "", False, "b1", True),
        ("arm_left", "IDLE", "", False, "b1", False),
        ("arm_left", "IDLE", "", False, "b2", False),
        ("arm_left", "IDLE", "", True, "b2", False)]


# ---- 상태 끊김: comm_timeout_s 동안 없으면 한 번, 다음 상태는 다시 넣는다 ----
def test_a_quiet_arm_is_told_lost_once_and_its_next_state_is_fed_again(node):
    limit = node.core.params.comm_timeout_s
    feed(node, arm(boot_id="b1"))
    node._check_arms_lost()
    assert node.core.args("on_arm_state_lost") == []
    msg, _ = node._arm_states["arm_left"]
    node._arm_states["arm_left"] = (msg, time.monotonic() - limit - 0.1)
    node._check_arms_lost()
    node._check_arms_lost()
    assert node.core.args("on_arm_state_lost") == [("arm_left",)]
    assert row_of(node, "arm_left").fault == RobotStatus.FAULT_COMM_LOST
    feed(node, arm(boot_id="b1"))  # 끊기기 전과 같은 상태도 다시 넣는다
    assert len(node.core.args("on_arm_state")) == 2
    assert row_of(node, "arm_left").fault == RobotStatus.FAULT_NONE
    node._arm_states["arm_left"] = (msg, time.monotonic() - limit - 0.1)
    node._check_arms_lost()
    assert node.core.args("on_arm_state_lost") == [("arm_left",), ("arm_left",)]


def test_a_workcell_arm_never_heard_from_is_told_lost_once_after_the_start(node):
    node._check_arms_lost()
    assert node.core.args("on_arm_state_lost") == [], "not before comm_timeout_s from the start"
    node._started_at = time.monotonic() - node.core.params.comm_timeout_s - 0.1
    node._check_arms_lost()
    node._check_arms_lost()
    assert sorted(node.core.args("on_arm_state_lost")) == [("arm_left",), ("arm_right",)]
    feed(node, arm(boot_id="b1"))  # 첫 상태는 그대로 넣는다
    assert node.core.args("on_arm_state") == [("arm_left", "IDLE", "", False, "b1", False)]
    assert "arm_left" not in node._arm_lost and "arm_right" in node._arm_lost


def test_the_tick_checks_for_quiet_arms(node):
    feed(node, arm(boot_id="b1"))
    msg, _ = node._arm_states["arm_left"]
    node._arm_states["arm_left"] = (msg, time.monotonic() - node.core.params.comm_timeout_s - 1)
    node._on_tick()
    assert node.core.args("on_arm_state_lost") == [("arm_left",)]


# ---- 담기 진행: feedback마다 코어로 ----
def test_every_fill_feedback_reaches_the_core(node):
    station = next(s for s in node.site.stations.values() if s.action == FILL)
    client = MagicMock()
    node._fill[station.id] = client
    node._send_fill(FillCmd(7, "pode1", station.id, "O-1", {"apple": 2}))
    callback = client.send_goal_async.call_args.kwargs["feedback_callback"]
    for phase, placed in (("PICKING", 0), ("PLACING", 1), ("PLACING", 1)):
        callback(SimpleNamespace(feedback=FillBasket.Feedback(
            arm_id="arm_left", phase=phase, placed_total=placed)))
    assert node.core.args("on_fill_feedback") == [], "the callback only queues"
    node._drain()
    assert node.core.args("on_fill_feedback") == [
        (7, "arm_left", "PICKING", 0), (7, "arm_left", "PLACING", 1),
        (7, "arm_left", "PLACING", 1)]


# ---- 상태판 팔 줄: fsm_state는 팔 상태, task_type · order_id는 코어의 진행 ----
def test_the_arm_row_takes_the_phase_and_trip_from_the_core(node):
    feed(node, arm("PLACING", order_id="O-ARM"))
    node.core._core.arm_task = lambda name: (
        ("PLACING", "T-9") if name == "arm_left" else ("", ""))
    row = row_of(node, "arm_left")
    assert (row.kind, row.fsm_state, row.task_type, row.order_id) == (
        "ARM", "PLACING", "PLACING", "T-9")
    node.core._core.arm_task = lambda name: ("", "")
    row = row_of(node, "arm_left")
    assert (row.task_type, row.order_id) == ("", ""), "the order comes from the core only"
    assert row_of(node, "arm_right").fault == RobotStatus.FAULT_COMM_LOST


# ---- [장애 처리 완료]: 확인 한도 30초 ----
def test_fault_resolved_is_confirmed_within_thirty_seconds_by_default(node):
    assert CallParams().confirm_s == 30.0
    assert node._robot_calls.params.confirm_s == 30.0


# ---- 틱 검사 전에 한도를 넘겨 들어온 상태도 끊김을 먼저 알린다 ----
def test_a_first_state_past_the_limit_is_told_lost_before_it_is_fed(node):
    node._started_at = time.monotonic() - node.core.params.comm_timeout_s - 0.05
    feed(node, arm(boot_id="b1"))
    names = [n for n, _ in node.core.calls if n in ("on_arm_state_lost", "on_arm_state")]
    assert names == ["on_arm_state_lost", "on_arm_state"]
    assert node.core.args("on_arm_state_lost") == [("arm_left",)]
    assert "arm_left" not in node._arm_lost


def test_a_state_after_a_gap_past_the_limit_is_told_lost_and_fed_again(node):
    limit = node.core.params.comm_timeout_s
    feed(node, arm(boot_id="b1"))
    msg, _ = node._arm_states["arm_left"]
    node._arm_states["arm_left"] = (msg, time.monotonic() - limit - 0.05)
    feed(node, arm(boot_id="b1"))
    assert node.core.args("on_arm_state_lost") == [("arm_left",)]
    assert len(node.core.args("on_arm_state")) == 2
    feed(node, arm(boot_id="b1"))  # 한도 안에 온 다음 상태는 끊김이 아니다
    assert node.core.args("on_arm_state_lost") == [("arm_left",)]
    assert len(node.core.args("on_arm_state")) == 2


# ---- 끊김은 도착 시각으로 센다 (코어가 받은 편지함에서 늦게 꺼내도) ----
def test_the_arrival_time_counts_not_the_time_the_core_takes_it(node):
    limit = node.core.params.comm_timeout_s
    t0 = time.monotonic() - 3 * limit
    node._started_at = t0
    node._post(node._on_arm_state, "arm_left", arm(boot_id="b1"), t0 + 0.1)
    node._post(node._on_arm_state, "arm_left", arm(boot_id="b1", holding=True), t0 + 0.3)
    node._drain()
    assert node.core.args("on_arm_state_lost") == [], "0.2 s apart on arrival"
    node._post(node._on_arm_state, "arm_left", arm(boot_id="b1"), t0 + 0.4 + limit)
    node._drain()
    assert node.core.args("on_arm_state_lost") == [("arm_left",)]


def test_the_subscription_stamps_the_arrival_time(node):
    node._post_arm_state("arm_left", arm(boot_id="b1"))
    time.sleep(0.3)
    node._drain()
    assert node._arm_states["arm_left"][1] < time.monotonic() - 0.25
