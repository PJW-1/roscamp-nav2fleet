"""FakeCentral: 중앙 시스템 대역. 재고 기록과 주문을 가진다."""
from __future__ import annotations

import threading
from dataclasses import dataclass

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.msg import FleetEvent, Order, OrderBoard, OrderItem
from fleet_interfaces.srv import ReportOrder, TripCommand
from rclpy.node import Node

from fake_robots.common import (
    ScenarioClock,
    load_contract_param,
    load_scenario_param,
    load_site_param,
)

SOURCE = "central"
FINAL_EVENTS = {
    ReportOrder.Request.STATUS_SHIPPED: ("INFO", "ORDER_SHIPPED"),
    ReportOrder.Request.STATUS_PARTIAL: ("WARN", "ORDER_PARTIAL"),
    ReportOrder.Request.STATUS_CANCELED: ("WARN", "ORDER_CANCELED"),
}
HELD = ReportOrder.Request.STATUS_HELD  # 끝이 아니다. 관리자의 강제 출고나 취소를 기다린다
ARM_FAULT = ReportOrder.Request.HOLD_ARM_FAULT


@dataclass
class _Entry:
    order_id: str
    items: dict
    created_at: object


def _text(items) -> str:
    return ",".join(f"{i.kind}:{i.count}" for i in items) or "-"


class FakeCentralNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("fake_central", parameter_overrides=parameter_overrides, **kwargs)
        contract = load_contract_param(self)
        site = load_site_param(self)
        scenario = load_scenario_param(self, site)
        self._sc = ScenarioClock(self)
        self._lock = threading.Lock()
        self._mode = OrderBoard.MODE_RUN
        self._record = dict(scenario.inventory)
        self._waiting_release = list(scenario.orders)  # 등록 순서
        self._board: list[_Entry] = []  # 오래된 순
        self._final: set[str] = set()
        self._held: dict[str, str] = {}  # 보류 중인 주문 -> hold_reason
        # 강제 출고를 보낸 주문. 거절되면 뺀다(최종 보고가 응답보다 먼저 올 수 있어 보낼 때 넣는다)
        self._force_sent: set[str] = set()
        self._force_after_s = scenario.force_ship_after_s
        self._force_due: dict[str, float] = {}  # 주문 -> 부를 시각(시나리오 초). 아직 안 부른 것
        self._forced: set[str] = set()

        self._pub = self.create_publisher(
            OrderBoard, contract.name("order_board"), to_ros_qos(contract.qos("order_board")))
        self._events = self.create_publisher(
            FleetEvent, contract.name("events"), to_ros_qos(contract.qos("events")))
        self.create_service(ReportOrder, contract.name("report_order"), self._report)
        self._force_cli = self.create_client(TripCommand, contract.name("force_ship"))
        self.create_timer(0.1, self._release_due)
        self.create_timer(0.2, self._force_due_orders)
        self.create_timer(1.0, self._publish)
        self._release_due()
        self._publish()

    def inventory(self) -> dict:
        with self._lock:
            return dict(self._record)

    def held_reasons(self) -> dict:
        with self._lock:
            return dict(self._held)

    def _event(self, level: str, code: str, detail: str) -> None:
        self._events.publish(FleetEvent(level=level, source=SOURCE, code=code, detail=detail,
                                        stamp=self.get_clock().now().to_msg()))

    def _add(self, order_id: str, items: dict) -> None:
        self._board.append(_Entry(order_id, items, self.get_clock().now().to_msg()))

    def _release_due(self) -> None:
        """시나리오 주문은 접수 검사를 건너뛴다. 기록으로 못 채우는 주문도 시험 대상이다."""
        t = self._sc.now()
        released = []
        with self._lock:
            while self._waiting_release and self._waiting_release[0].at_s <= t:
                order = self._waiting_release.pop(0)
                self._add(order.id, dict(order.items))
                released.append(order.id)
        for order_id in released:
            self._event("INFO", "ORDER_ACCEPTED", order_id)
        if released:
            self._publish()

    def _settle_stock(self, request, forced: bool) -> list[tuple[str, str, str]]:
        """놓은 만큼 빼고 없던 만큼 바로잡는다. 잠금을 잡은 채 호출한다.

        forced: 강제 출고 뒤의 최종. missing은 고장 난 팔이 못 놓은 것이라 재고로 보지 않는다.
        """
        out = []
        for item in request.placed:
            if item.kind in self._record:
                self._record[item.kind] = max(0, self._record[item.kind] - item.count)
        for item in () if forced else request.missing:
            left = self._record.get(item.kind, 0)
            text = f"{request.order_id} {item.kind} missing {item.count}, record {left}"
            out.append(("WARN", "STOCK_MISMATCH" if left > 0 else "OUT_OF_STOCK", text))
            if item.kind in self._record:
                self._record[item.kind] = 0  # 팔이 하나도 못 찾았으니 재고는 0
        return out

    def _force_due_orders(self) -> None:
        """보류 시각에서 force_ship_after_s가 지난 주문마다 /fleet/force_ship을 한 번 부른다."""
        with self._lock:
            now = self._sc.now()
            due = [o for o, t in self._force_due.items() if t <= now]
        if not due or not self._force_cli.service_is_ready():
            return  # 서버가 뜰 때까지 다음 틱에 다시
        for order_id in due:
            with self._lock:
                self._force_due.pop(order_id, None)
                self._forced.add(order_id)
                self._force_sent.add(order_id)
            future = self._force_cli.call_async(TripCommand.Request(trip_id=order_id))
            future.add_done_callback(lambda f, o=order_id: self._force_answer(o, f))

    def _force_answer(self, order_id: str, future) -> None:
        result = future.result() if future.exception() is None else None
        if result is None or not result.accepted:
            with self._lock:
                self._force_sent.discard(order_id)
        if result is None:
            self._event("WARN", "FORCE_SHIP", f"{order_id} accepted=False reason=NO_ANSWER")
            return
        level = "INFO" if result.accepted else "WARN"
        self._event(level, "FORCE_SHIP",
                    f"{order_id} accepted={result.accepted} reason={result.reason}".rstrip())

    def _on_held(self, request, response):
        """보류는 주문을 판에 두고 사유만 적는다. 같은 보류가 또 와도 바뀌는 것이 없다."""
        with self._lock:
            if not any(e.order_id == request.order_id for e in self._board):
                response.accepted = False
                return response
            response.accepted = True
            if self._held.get(request.order_id) == request.hold_reason:
                return response
            self._held[request.order_id] = request.hold_reason
            if (self._force_after_s is not None and request.hold_reason == ARM_FAULT
                    and request.order_id not in self._forced):
                self._force_due.setdefault(request.order_id,
                                           self._sc.now() + self._force_after_s)
        self._event("WARN", "ORDER_HELD",
                    f"{request.order_id} robot={request.robot_id} "
                    f"hold_reason={request.hold_reason} placed={_text(request.placed)} "
                    f"missing={_text(request.missing)} {request.detail}".rstrip())
        return response

    def _report(self, request, response):
        if request.status == HELD:
            return self._on_held(request, response)
        final = FINAL_EVENTS.get(request.status)
        with self._lock:
            entry = next((e for e in self._board if e.order_id == request.order_id), None)
            if final is None or (entry is None and request.order_id not in self._final):
                response.accepted = False
                return response
            response.accepted = True
            if entry is None:  # 같은 보고가 또 와도 바뀌는 것이 없다
                return response
            self._board.remove(entry)
            self._final.add(request.order_id)
            self._held.pop(request.order_id, None)
            forced = request.order_id in self._force_sent
            self._force_due.pop(request.order_id, None)
            events = self._settle_stock(request, forced)
        for level, code, text in events:
            self._event(level, code, text)
        level, code = final
        self._event(level, code, f"{request.order_id} robot={request.robot_id} "
                                 f"placed={_text(request.placed)} "
                                 f"missing={_text(request.missing)} {request.detail}".rstrip())
        self._publish()
        return response

    def _publish(self) -> None:
        with self._lock:
            msg = OrderBoard(
                run_mode=self._mode,
                orders=[Order(order_id=e.order_id, created_at=e.created_at,
                              items=[OrderItem(kind=k, count=c) for k, c in e.items.items()])
                        for e in self._board],
                stamp=self.get_clock().now().to_msg())
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
