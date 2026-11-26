# ---- 브리지 보고: Hello로 epoch 받기, ReportTrip · ReportEvent · ReportCommand 보내기 ----
from __future__ import annotations

import time
import uuid
from functools import partial

from fleet_interfaces.srv import (
    Hello,
    ListOpenTrips,
    ReportCommand,
    ReportEvent,
    ReportTrip,
)

from fleet_manager import adapters
from fleet_manager.goal_tracker import BridgeOutbox, Handshake

WARN_LEVELS = ("WARN", "ERROR")  # IR-12: 이 등급 이상은 ReportEvent로도 보낸다


class BridgeReports:
    """노드의 코어 그룹에서만 부른다. 답은 post로 받은 편지함에 넣어 코어 그룹에서 처리한다."""

    def __init__(self, node, params, robots, post, open_trips_handler=None):
        self._node = node
        self._post = post
        self._robots = set(robots)
        self._open_trips_handler = open_trips_handler
        self.open_trips = None  # 주문 층이 받지 않으면 여기 둔다
        self.boot_id = str(uuid.uuid4())
        name = node.contract.name
        # IR-30: 클라이언트는 코어 그룹이 아닌 받은 편지함 그룹에 둔다(답은 큐에 넣기만 함)
        group = node._inbox_group
        self._hello = node.create_client(Hello, name("hello"), callback_group=group)
        self._list = node.create_client(ListOpenTrips, name("list_open_trips"),
                                        callback_group=group)
        self._clients = {
            "trip": node.create_client(ReportTrip, name("report_trip"), callback_group=group),
            "event": node.create_client(ReportEvent, name("report_event"), callback_group=group),
            "command": node.create_client(ReportCommand, name("report_command"),
                                          callback_group=group),
        }
        self._handshake = Handshake(params.bridge_retry_first_s, params.bridge_retry_max_s,
                                    params.bridge_reply_limit_s)
        self._hs_future = None
        self._outbox = BridgeOutbox(params.bridge_retry_first_s, params.bridge_retry_max_s,
                                    params.bridge_reply_limit_s, params.bridge_max_in_flight)
        self._futures = {}  # token -> (client, future)

    @property
    def epoch(self):
        return self._outbox.epoch

    def _stamp(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    # ---- 보고 넣기 ----
    def trip(self, cmd) -> None:
        # ReportTripCmd(진행)와 ReportOrderCmd(끝) 모두 운반 건 보고 하나로 간다
        request = adapters.trip_report_request(cmd, self._stamp())
        self._outbox.put(("trip", request.trip_id), request)

    def event(self, event) -> None:
        if event.level not in WARN_LEVELS:
            return
        event_id = str(uuid.uuid4())
        request = adapters.report_event_request(event, event_id, self._stamp(), self._robots)
        self._outbox.put(("event", event_id), request)

    def command(self, request_id, command, target, result, reason="", detail="") -> None:
        request = adapters.report_command_request(request_id, command, target, result, reason,
                                                  detail, self._stamp())
        self._outbox.put(("command", request_id), request)

    # ---- 매 틱: Hello 순서 진행, 응답 한도 넘긴 요청 버리기, 보낼 것 보내기 ----
    def flush(self, now=None) -> None:
        now = time.monotonic() if now is None else now
        self._step_handshake(now)
        for token in self._outbox.expire(now):
            client, future = self._futures.pop(token)
            client.remove_pending_request(future)
        for token, request, epoch, n in self._outbox.due(now):
            kind = self._kind(request)
            client = self._clients[kind]
            if kind == "trip":
                request.report_epoch, request.report_n = epoch, n
            future = client.call_async(request)
            self._futures[token] = (client, future)
            future.add_done_callback(partial(self._post, self._answered, token, request))

    def waiting(self) -> int:
        return self._outbox.waiting()

    @staticmethod
    def _kind(request) -> str:
        if isinstance(request, ReportTrip.Request):
            return "trip"
        return "event" if isinstance(request, ReportEvent.Request) else "command"

    def _answered(self, token, request, future) -> None:
        if self._futures.pop(token, None) is None:
            return  # 응답 한도로 이미 버린 요청
        now = time.monotonic()
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"bridge report got no reply: {exc}")
            self._outbox.answered(token, False, now)
            return
        if isinstance(request, ReportTrip.Request):
            delivered, late = adapters.trip_report_delivered(response)
            if late:
                self._node.get_logger().warning(
                    f"bridge took trip {request.trip_id} {request.status} "
                    f"({request.report_epoch}, {request.report_n}) as LATE_REPORT")
        elif isinstance(request, ReportCommand.Request):
            # 같은 request_id의 끝 상태는 한 번만 받는다(명세 4.3). false도 다시 보내지 않는다
            delivered = True
            if not response.accepted:
                self._node.get_logger().warning(
                    f"bridge refused command result {request.request_id} {request.command} "
                    f"{request.result}: it already has a final result")
        else:
            delivered = bool(response.accepted)
        self._outbox.answered(token, delivered, now)

    # ---- Hello → ListOpenTrips (IR-22), 포기하지 않는다 (IR-40) ----
    def _step_handshake(self, now: float) -> None:
        hs = self._handshake
        if hs.expired(now):
            client = self._hello if hs.epoch is None else self._list
            client.remove_pending_request(self._hs_future)
            self._hs_future = None
        step = hs.due(now)
        if step == "hello" and self._hello.service_is_ready():
            future = self._hello.call_async(adapters.hello_request(self.boot_id))
        elif step == "list" and self._list.service_is_ready():
            future = self._list.call_async(ListOpenTrips.Request(epoch=hs.epoch))
        else:
            return
        hs.sent(now)
        self._hs_future = future
        future.add_done_callback(partial(self._post, self._hs_answered, step))

    def _hs_answered(self, step, future) -> None:
        if future is not self._hs_future:
            return  # 응답 한도로 이미 버린 요청
        self._hs_future = None
        hs = self._handshake
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"bridge {step} got no reply: {exc}")
            hs.failed(time.monotonic())
            return
        if not response.accepted:
            self._node.get_logger().warning(f"bridge refused {step}: {response.reason}")
            hs.failed(time.monotonic())
            return
        if step == "hello":
            hs.hello_ok(response.epoch)
            self._outbox.set_epoch(response.epoch)
            self._node.get_logger().info(f"bridge gave epoch {response.epoch}, "
                                         f"{self._outbox.waiting()} reports waiting")
            return
        hs.list_ok()
        trips = adapters.open_trips(response)
        if self._open_trips_handler is not None:
            self._open_trips_handler(trips)
            return
        # TODO(orders): 주문 층에 on_open_trips가 생기면 거기로 넘긴다. 모르는 상태는 짐작하지 않는다
        self.open_trips = trips
        self._node.get_logger().info(
            f"bridge has {len(trips)} open trips; the order layer has no on_open_trips, kept")
