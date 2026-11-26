"""FakeBridge: 브리지 대역. 시험과 골라 쓰는 실행에서 Hello와 Report*를 받기만 한다."""
from __future__ import annotations

import threading

import rclpy
from fleet_interfaces.srv import (
    CheckStock,
    Hello,
    ListOpenTrips,
    ReportCommand,
    ReportEvent,
    ReportTrip,
)
from rclpy.node import Node

from fake_robots.common import load_contract_param


class FakeBridgeNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("fake_bridge", parameter_overrides=parameter_overrides, **kwargs)
        name = load_contract_param(self).name
        self._lock = threading.Lock()
        self._epochs: dict[str, int] = {}  # boot_id -> epoch. 같은 boot_id는 같은 epoch
        self._last_n: dict[str, tuple[int, int]] = {}  # trip_id -> 반영한 (epoch, n)
        self.hellos: list = []
        self.trips: list = []  # 반영한 ReportTrip 요청, 도착 순
        self.trip_results: list = []
        self.trip_asks: list = []  # 받은 모든 ReportTrip의 (trip_id, epoch, n), 실패로 답한 것 포함
        self.events: list = []
        self.commands: list = []
        self.command_asks: list = []  # 받은 모든 ReportCommand, 거절한 것 포함
        self.command_refused: list = []  # 끝 상태가 이미 있어 false로 답한 request_id
        self._command_final: set = set()
        self.open_trip_asks: list = []
        self.open_trips: list = []  # ListOpenTrips 답. 기본은 비어 있다
        self.trip_fail = 0  # 이 수만큼 ReportTrip에 accepted=false로 답한다
        self.create_service(Hello, name("hello"), self._hello)
        self.create_service(ListOpenTrips, name("list_open_trips"), self._list_open_trips)
        self.create_service(ReportTrip, name("report_trip"), self._report_trip)
        self.create_service(ReportEvent, name("report_event"), self._report_event)
        self.create_service(ReportCommand, name("report_command"), self._report_command)
        self.create_service(CheckStock, name("check_stock"), self._check_stock)

    def _hello(self, request, response):
        with self._lock:
            self.hellos.append(request)
            epoch = self._epochs.setdefault(request.boot_id, len(self._epochs) + 1)
        response.accepted, response.epoch = True, epoch
        return response

    def _list_open_trips(self, request, response):
        with self._lock:
            self.open_trip_asks.append(request.epoch)
            response.accepted, response.trips = True, list(self.open_trips)
        return response

    def _report_trip(self, request, response):
        key = (request.report_epoch, request.report_n)
        with self._lock:
            self.trip_asks.append((request.trip_id, *key))
            if self.trip_fail > 0:
                self.trip_fail -= 1
                response.accepted, response.reason = False, "DB write failed"
                return response
            last = self._last_n.get(request.trip_id)
            if last == key:
                result = "DUPLICATE"
            elif last is not None and key < last:
                result = "LATE_REPORT"
            else:
                result = "APPLIED"
                self._last_n[request.trip_id] = key
                self.trips.append(request)
            self.trip_results.append(result)
        response.accepted, response.result = True, result
        return response

    def _report_event(self, request, response):
        with self._lock:
            self.events.append(request)
        response.accepted = True
        return response

    def _report_command(self, request, response):
        # 같은 request_id는 UNCONFIRMED 뒤 끝 상태 한 번만 받는다(명세 4.3)
        with self._lock:
            self.command_asks.append(request)
            final = request.result != ReportCommand.Request.RESULT_UNCONFIRMED
            if final and request.request_id in self._command_final:
                self.command_refused.append(request.request_id)
                response.accepted = False
                return response
            if final:
                self._command_final.add(request.request_id)
            self.commands.append(request)
        response.accepted = True
        return response

    def _check_stock(self, request, response):
        # 재고는 늘 넉넉하다
        response.accepted, response.available = True, list(request.items)
        return response


def main() -> None:
    rclpy.init()
    node = FakeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
