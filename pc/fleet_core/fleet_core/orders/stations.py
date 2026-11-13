# ---- 스테이션 작업: 채우기, 교환, 출고 배차 ----
from __future__ import annotations

from fleet_core.common.commands import (
    FILL,
    HELD,
    SWAP,
    ArmFillCmd,
    ArmResetCmd,
    ArmSwapCmd,
    DispatchCmd,
    ReleaseCmd,
)

from .records import EMPTY, FILLING, FULL, _BoundOrder, _Robot, _subtract


# ---- 주문 묶기와 채우기 마무리 ----
class Stations:
    def _start_filling(self, r: _Robot) -> None:
        pending = self._pending()
        if not pending:
            self._out.append(ReleaseCmd(r.name))
            return
        record = pending[0]
        r.order = _BoundOrder(record.order_id, dict(record.items))
        self._bind_seq += 1
        r.bound = self._bind_seq
        r.last_order_id = record.order_id
        r.stage = FILLING
        self._event("INFO", "ORDER_BOUND", record.order_id, r.name)

    def _finish_fill(self, r: _Robot) -> None:
        order = r.order
        self._out.append(ArmResetCmd(r.name))
        order.missing = _subtract(order.items, order.placed)
        if not order.placed:
            self._report(r, HELD, f"nothing could be placed, missing {order.missing}")
            self._leave_station(r)
        elif order.missing and not self.params.ship_partial:
            detail = f"{order.order_id} partial not allowed, basket holds {order.placed}"
            r.basket = FULL
            r.stage = ""
            self._report(r, HELD, detail)
            self._fail(r, detail)
            self._event("ERROR", "FILL_FAILED", detail, r.name)
        else:
            r.basket = FULL
            r.stage = ""
            self._send_to_outbound(r)

    # ---- 출고 스테이션 배차와 재시도 ----
    def _send_to_outbound(self, r: _Robot) -> None:
        r.outbound_goal = self._next_dispatch_id()
        self._out.append(DispatchCmd(r.outbound_goal, self._outbound, r.name))

    def _retry_outbound(self, r: _Robot) -> None:
        if self._clock() < r.dispatch_retry_at:
            return
        r.dispatch_retry_at = None
        detail = (f"dispatch to {self._outbound} failed {r.dispatch_failures} times: "
                  f"{r.dispatch_reason}")
        if r.dispatch_failures > self.params.dispatch_retry_limit:
            self._fail(r, detail)
            self._event("ERROR", "DISPATCH_FAILED", detail, r.name)
            return
        self._event("WARN", "DISPATCH_RETRY", f"{detail}, sending it again", r.name)
        self._send_to_outbound(r)

    @staticmethod
    def _forget_dispatch_failures(r: _Robot) -> None:
        r.dispatch_failures = 0
        r.dispatch_retry_at = None
        r.dispatch_reason = ""

    # ---- 채우기 단계 진행 ----
    def _step_filling(self, r: _Robot) -> None:
        if r.asked:
            return
        order = r.order
        if sum(order.items.values()) > self.params.basket_capacity:
            self._report(r, HELD, f"{sum(order.items.values())} items exceed the basket "
                                  f"capacity of {self.params.basket_capacity}")
            self._leave_station(r)
            return
        remaining = _subtract(order.items, order.placed, order.missing)
        if not remaining:
            self._finish_fill(r)
            return
        r.asked = FILL
        self._out.append(ArmFillCmd(r.name, self._work, order.order_id, remaining))

    # ---- 교환 단계 진행 ----
    def _step_swapping(self, r: _Robot) -> None:
        if r.asked:
            return
        if r.basket == EMPTY:
            self._leave_station(r)
            return
        r.asked = SWAP
        self._out.append(ArmSwapCmd(r.name, self._outbound, r.last_order_id,
                                    take_full=r.basket == FULL, put_empty=True))
