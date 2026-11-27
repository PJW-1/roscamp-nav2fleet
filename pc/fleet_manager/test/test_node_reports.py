"""가짜 브리지와 실제 그래프로 브리지 보고 배선을 본다: Hello 전 보류, epoch·n, 재전송, 사건·명령 보고.

그래프에 참여하므로 로봇 네트워크와 분리된 도메인에서만 돈다.
"""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("fleet_interfaces")

from fake_robots.fake_bridge_node import FakeBridgeNode  # noqa: E402
from fleet_core.common.commands import EventOut, ReportOrderCmd, ReportTripCmd  # noqa: E402
from fleet_interfaces.msg import OpenTrip, OrderItem  # noqa: E402
from fleet_manager.fleet_manager_node import FleetManagerNode  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_DOMAIN_ID", "") in ("", "6")
    or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST",
    reason="set ROS_DOMAIN_ID (not 6) and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST first")


class Graph:
    def __init__(self):
        rclpy.init(args=["--ros-args", "-p", "robots:=[pode1]", "-p", "tick_hz:=50.0"])
        self.fms = FleetManagerNode()
        self.bridge = None
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.fms)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def start_bridge(self, open_trips=()):
        self.bridge = FakeBridgeNode()
        self.bridge.open_trips = list(open_trips)
        self.executor.add_node(self.bridge)
        return self.bridge

    def dispatch(self, command):
        # 코어 그룹에서 돌도록 받은 편지함으로 넣는다
        self.fms._post(self.fms._dispatch, command, 1.0)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5)
        if self.bridge is not None:
            self.bridge.destroy_node()
        self.fms.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def graph():
    g = Graph()
    yield g
    g.close()


def wait_for(predicate, timeout_s=15.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_hello_comes_first_then_open_trips_with_its_epoch_and_they_are_kept(graph):
    trip = OpenTrip(trip_id="T-9", order_id="O-9", status="FILLING", robot_id="pode1",
                    items=[OrderItem(kind="A", count=1)])
    bridge = graph.start_bridge([trip])
    assert wait_for(lambda: bridge.open_trip_asks), "ListOpenTrips was never called"
    [hello] = bridge.hellos
    assert hello.node == "fleet" and hello.boot_id == graph.fms._bridge.boot_id
    assert bridge.open_trip_asks == [graph.fms._bridge.epoch] == [1]
    assert wait_for(lambda: graph.fms._bridge.open_trips is not None)
    [kept] = graph.fms._bridge.open_trips
    assert (kept["trip_id"], kept["status"], kept["items"]) == ("T-9", "FILLING", {"A": 1})


def test_reports_wait_for_the_bridge_and_then_go_with_the_epoch_in_order(graph):
    graph.dispatch(ReportTripCmd("T-1", "pode1", "ASSIGNED", {}, {}))
    graph.dispatch(EventOut("WARN", "CANCEL_UNCONFIRMED", "goal 3", "pode1"))
    graph.dispatch(EventOut("INFO", "ORDER_BOUND", "T-1", "pode1"))
    assert wait_for(lambda: graph.fms._bridge.waiting() == 2, 5.0)
    time.sleep(1.0)
    assert graph.fms._bridge.waiting() == 2, "nothing goes before Hello"
    bridge = graph.start_bridge()
    assert wait_for(lambda: bridge.trips and bridge.events)
    [trip] = bridge.trips
    assert (trip.trip_id, trip.status, trip.report_epoch) == ("T-1", "ASSIGNED", 1)
    [event] = bridge.events
    assert (event.level, event.code, event.robot_id, event.source) == (
        "WARN", "CANCEL_UNCONFIRMED", "pode1", "pode1")
    assert len(event.event_id) == 36
    graph.dispatch(ReportTripCmd("T-1", "pode1", "TRANSPORTING", {"A": 1}, {}))
    graph.dispatch(ReportOrderCmd("T-1", "pode1", "CANCELED", {}, {"A": 1}, "nothing"))
    assert wait_for(lambda: len(bridge.trips) == 3 or (
        len(bridge.trips) == 2 and bridge.trips[-1].status == "CANCELLED"))
    last = bridge.trips[-1]
    assert (last.status, last.reason) == ("CANCELLED", "NOTHING_TO_FILL")
    asks = [n for trip_id, _, n in bridge.trip_asks if trip_id == "T-1"]
    assert asks == sorted(asks) and len(set(asks)) == len(asks), f"n must grow, got {asks}"
    assert "LATE_REPORT" not in bridge.trip_results
    assert len(bridge.events) == 1, "INFO stays on /central/events only"


def test_a_failed_trip_report_is_sent_again_with_the_same_n(graph):
    bridge = graph.start_bridge()
    bridge.trip_fail = 2
    assert wait_for(lambda: graph.fms._bridge.epoch is not None)
    graph.dispatch(ReportTripCmd("T-2", "pode1", "FILLING", {}, {}))
    started = time.monotonic()
    assert wait_for(lambda: bridge.trips, 10.0), "the report never got through"
    took = time.monotonic() - started
    assert 0.8 * 3.0 <= took <= 1.2 * 3.0 + 1.0, f"1 s then 2 s backoff, took {took:.1f} s"
    assert bridge.trip_results == ["APPLIED"]
    assert len(bridge.trip_asks) == 3 and len(set(bridge.trip_asks)) == 1, bridge.trip_asks
    assert wait_for(lambda: graph.fms._bridge.waiting() == 0, 2.0)


def test_a_command_result_goes_to_the_bridge(graph):
    bridge = graph.start_bridge()
    graph.fms._post(graph.fms.report_command, "r-1", "set_lock", "pode1", "ACCEPTED")
    assert wait_for(lambda: bridge.commands)
    [cmd] = bridge.commands
    assert (cmd.request_id, cmd.command, cmd.target, cmd.result, cmd.reason) == (
        "r-1", "set_lock", "pode1", "ACCEPTED", "")


def test_a_refused_command_result_is_not_sent_again(graph):
    # 명세 4.3: 같은 request_id의 끝 상태는 한 번만 받는다. false 답은 이미 받았다는 뜻이다
    bridge = graph.start_bridge()
    graph.fms._post(graph.fms.report_command, "r-2", "set_lock", "pode1", "ACCEPTED")
    assert wait_for(lambda: len(bridge.command_asks) == 1)
    graph.fms._post(graph.fms.report_command, "r-2", "set_lock", "pode1", "ACCEPTED")
    assert wait_for(lambda: len(bridge.command_asks) == 2)
    time.sleep(3.0)  # 첫 재전송 간격(1 s)의 세 배
    assert len(bridge.command_asks) == 2, "a refused duplicate must not be retried"
    assert bridge.command_refused == ["r-2"]
    assert graph.fms._bridge.waiting() == 0
