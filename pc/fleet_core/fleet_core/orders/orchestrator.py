# ---- OrderOrchestrator: 주문판을 받아 로봇에 주문을 배정하고 진행 ----
from __future__ import annotations

from fleet_core.common.commands import (
    FILL,
    HELD,
    NO_ROBOT,
    PARTIAL,
    SHIPPED,
    SWAP,
    TRIP_FILLING,
    TRIP_FILLING_PAUSE,
    ArmDeclined,
    ArmFailedCmd,
    ArmFault,
    ArmFillPaused,
    ArmFillPlaced,
    ArmFillProgress,
    ArmFillResumed,
    ArmFillResult,
    ArmLate,
    ArmLost,
    ArmResetCmd,
    ArmStationBlocked,
    ArmSwapResult,
    CancelDispatchCmd,
    DispatchCmd,
    EventOut,
    ReleaseCmd,
    ReportOrderCmd,
    ReportTripCmd,
    RobotFaultCmd,
)

from .faults import Faults
from .records import (
    EMPTY,
    FILLING,
    FULL,
    NONE,
    SWAPPING,
    LostJob,
    RobotRecord,
    merge_items,
    subtract_items,
)
from .stations import Stations

# ---- OrderOrchestrator 생성 ----
STOP = "STOP"
PAUSE = "PAUSE"


class OrderOrchestrator(Faults, Stations):
    def __init__(self, stations, params, robots, clock):
        self.params = params
        self.shipped = 0
        self._stations = stations
        self._clock = clock
        self._dispatch_seq = 0
        self._bind_seq = 0
        self._robots = {name: RobotRecord(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._board: list = []
        self._reported: set[str] = set()  # 최종 보고한 주문
        self._held: dict[str, tuple[str, str]] = {}  # 보류 주문: id -> (로봇, 사유)
        self._over_capacity: set[str] = set()  # 용량 초과를 알린 주문
        self._open_calls: list[int] = []
        self._work = next(s.id for s in stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in stations.values() if s.action == SWAP)
        self._lost_jobs: dict[int, LostJob] = {}
        self._work_blocked = ""  # 작업대 팔이 ERROR · 상태 모름이면 배차 요청을 거둠

    # ---- 주문판, 배차, 팔 보고 입력 처리 ----
    def on_order_board(self, run_mode, orders) -> None:
        self._run_mode = run_mode
        self._board = list(orders)
        ids = {o.order_id for o in self._board}
        self._reported &= ids
        self._over_capacity &= ids
        self._held = {k: v for k, v in self._held.items() if k in ids}
        for r in self._robots.values():
            if (r.held or r.paused) and r.order and r.order.order_id not in ids:
                r.order = None
        self._warn_over_capacity()
        self._sync_calls()

    def on_dispatch_feedback(self, goal_id, robot_id, phase) -> None:
        if phase == NO_ROBOT:
            ids = [o.order_id for o in self._pending()]
            self._event("WARN", "NO_ROBOT_AVAILABLE", f"{ids} wait and no robot can take them")

    def on_dispatch_result(self, goal_id, success, robot_id, reason="") -> None:
        if goal_id in self._open_calls:
            self._open_calls.remove(goal_id)
            if success and robot_id in self._robots:
                self._start_filling(self._robots[robot_id])
            self._sync_calls()
            return
        r = self._robots.get(robot_id)
        if r is None:
            return
        if r.outbound_goal == goal_id:
            r.outbound_goal = None
            if success:
                r.stage = SWAPPING
                r.dispatch_failures = 0
            else:
                r.dispatch_failures += 1
                r.dispatch_retry_at = self._clock() + self.params.dispatch_retry_wait_s
                r.dispatch_reason = reason

    def on_arm_report(self, report) -> None:
        if isinstance(report, ArmStationBlocked):
            if report.station_id == self._work:
                self._work_blocked = report.reason
                self._sync_calls()
            return
        if isinstance(report, ArmLate):
            if report.kind == FILL:
                self._late_fill(report.goal_id, report.placed, report.uncertain)
            else:
                self._late_swap(report.goal_id, report.took_full, report.put_empty_done)
            return
        r = self._robots.get(report.robot)
        if r is None:
            return
        if isinstance(report, ArmDeclined):
            r.asked = ""
        elif isinstance(report, ArmFillResult):
            self._on_fill_result(r, report.success, report.placed, report.uncertain,
                                 report.held)
        elif isinstance(report, ArmSwapResult):
            self._on_swap_result(r, report.took_full, report.put_empty_done)
        elif isinstance(report, ArmLost):
            self._on_arm_lost(r, report)
        elif isinstance(report, ArmFault):
            self._on_arm_fault(r, report)
        elif isinstance(report, ArmFillPaused):
            self._pause_fill(r, report)
        elif isinstance(report, ArmFillProgress):
            self._report_progress(r, report.placed)
        elif isinstance(report, ArmFillPlaced):
            self._on_fill_placed(r, report.placed, report.held)
        elif isinstance(report, ArmFillResumed):
            r.asked, r.arm_restarted = "", False

    # ---- 팔 결과를 주문에 반영하고 결과를 잃은 작업 기록 ----
    def _on_fill_result(self, r: RobotRecord, success, placed, uncertain=None,
                        held=False) -> None:
        r.asked = ""
        merge_items(r.order.placed, placed)
        if r.isolated:
            self._requeue_isolated(r, uncertain, held)
            return
        if success:
            r.order.missing = subtract_items(r.order.items, r.order.placed)
            return
        self._out.append(ArmFailedCmd(r.name))

    # ---- 담기 일시 중지: 다시 보내지 않고 관리자를 기다린다 (팔 관제 합의 ①) ----
    def _pause_fill(self, r: RobotRecord, paused: ArmFillPaused) -> None:
        r.asked = ""
        if r.isolated:
            self._decide_if_isolated(r)
            return
        if not r.order:
            return
        r.paused = paused.reason
        order = r.order
        left = subtract_items(order.items, order.placed)
        self._out.append(ReportTripCmd(order.order_id, r.name, TRIP_FILLING_PAUSE,
                                       dict(order.placed), left, paused.reason))

    # ---- 재시작으로 사라진 목표: 담은 것을 더하고 [복구]까지 담기 요청 중으로 둠 (팔 관제 합의 ⑪) ----
    def _on_fill_placed(self, r: RobotRecord, placed: dict[str, int], held=False) -> None:
        r.arm_restarted = True
        if r.order:
            merge_items(r.order.placed, placed)
        if r.isolated:
            r.asked = ""
            self._requeue_isolated(r, held=held)

    # ---- 담는 동안 담은 수 진행 보고 (팔 관제 합의 ⑫) ----
    def _report_progress(self, r: RobotRecord, placed: dict[str, int]) -> None:
        if not r.order or r.held or r.paused:
            return
        order = r.order
        now = dict(order.placed)
        merge_items(now, placed)
        self._out.append(ReportTripCmd(order.order_id, r.name, TRIP_FILLING, now,
                                       subtract_items(order.items, now)))

    def _on_arm_lost(self, r: RobotRecord, lost: ArmLost) -> None:
        if lost.cleared:
            order_id, bound = lost.order_id, lost.bound
        else:
            r.asked = ""
            order_id, bound = self.order_of(r.name), r.bound
        self._lost_jobs[lost.goal_id] = LostJob(r.name, lost.server, lost.kind, order_id,
                                                 cleared=lost.cleared, bound=bound,
                                                 isolated=lost.isolated)

    def _on_swap_result(self, r: RobotRecord, took_full, put_empty_done) -> None:
        r.asked = ""
        if took_full and r.basket == FULL:
            r.basket = NONE
            self._report_shipped(r)
        if r.isolated:
            self._requeue_isolated(r)
            return
        if put_empty_done and r.basket == NONE:
            r.basket = EMPTY
            self._out.append(ArmResetCmd(r.name))
            self._leave_station(r)
            return
        self._out.append(ArmFailedCmd(r.name))

    # ---- 로봇 한 대의 단계 진행 ----
    def step(self, robot) -> None:
        r = self._robots.get(robot)
        if r is None or r.held or r.paused:
            return
        # 서비스 정지: 담기를 시작한 운반 건만 끝까지 (팔 관제 합의 ⑦)
        if self._run_mode == PAUSE and not self.carries(robot):
            return
        if r.stage == FILLING:
            self._step_filling(r)
        elif r.stage == SWAPPING:
            self._step_swapping(r)
        elif r.isolated or r.outbound_goal is not None:
            return
        elif r.dispatch_retry_at is not None:
            self._retry_outbound(r)
        elif r.basket == NONE and not r.order:
            self._send_to_outbound(r)

    # ---- 출력과 로봇 조회 ----
    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    @property
    def service_paused(self) -> bool:
        return self._run_mode == PAUSE

    # 담기를 시작한 운반을 맡고 있는지 (서비스 정지에도 출고 · 교환까지 이어 감)
    def carries(self, robot) -> bool:
        r = self._robots.get(robot)
        return r is not None and ((r.order is not None and r.order.started)
                                  or r.basket == FULL or r.stage == SWAPPING)

    def order_of(self, robot) -> str:
        r = self._robots.get(robot)
        return r.order.order_id if r and r.order else ""

    def robot_view(self, robot) -> dict:
        r = self._robots.get(robot)
        if r is None:
            return {}
        return {
            "stage": r.stage,
            "basket": r.basket,
            "order": r.order.order_id if r.order else None,
            "placed": dict(r.order.placed) if r.order else {},
        }

    # ---- 이벤트, 고장, 스테이션 떠남 출력 ----
    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    def _fail(self, r: RobotRecord, reason: str) -> None:
        self._out.append(RobotFaultCmd(r.name, reason))

    def _leave_station(self, r: RobotRecord) -> None:
        r.stage = ""
        self._out.append(ReleaseCmd(r.name))

    # ---- 대기 주문 계산과 배차 요청 맞추기 ----
    def _pending(self) -> list:
        skip = ({r.order.order_id for r in self._robots.values() if r.order}
                | self._reported | self._held.keys())
        return [o for o in self._board if o.order_id not in skip
                and sum(o.items.values()) <= self.params.basket_capacity]

    def _warn_over_capacity(self) -> None:
        capacity = self.params.basket_capacity
        for o in self._board:
            count = sum(o.items.values())
            if count <= capacity:
                self._over_capacity.discard(o.order_id)
            elif o.order_id not in self._over_capacity:
                self._over_capacity.add(o.order_id)
                self._event("WARN", "ORDER_OVER_CAPACITY",
                            f"{o.order_id}: {count} items exceed the basket capacity of {capacity}")

    # 작업대가 막혔거나 서비스 정지면 대기 주문 배차 요청을 모두 거둔다 (팔 관제 합의 ④ · ⑥ · ⑦)
    def _sync_calls(self) -> None:
        stop = self._work_blocked or self._run_mode == PAUSE
        wanted = 0 if stop else len(self._pending())
        while len(self._open_calls) < wanted:
            goal_id = self._next_dispatch_id()
            self._open_calls.append(goal_id)
            self._out.append(DispatchCmd(goal_id, self._work))
        while len(self._open_calls) > wanted:
            self._out.append(CancelDispatchCmd(self._open_calls.pop()))

    def _next_dispatch_id(self) -> int:
        self._dispatch_seq += 1
        return self._dispatch_seq

    # ---- 주문 보고 만들기 ----
    def _report(self, r: RobotRecord, status: str, detail: str = "") -> None:
        order, r.order = r.order, None
        r.held = r.paused = ""
        self._reported.add(order.order_id)
        self._held.pop(order.order_id, None)
        self._out.append(ReportOrderCmd(order.order_id, r.name, status, dict(order.placed),
                                        dict(order.missing), detail))

    def _hold(self, r: RobotRecord, reason: str, detail: str) -> None:
        order = r.order
        order.missing = subtract_items(order.items, order.placed)
        self._held[order.order_id] = (r.name, reason)
        self._out.append(ReportOrderCmd(order.order_id, r.name, HELD, dict(order.placed),
                                        dict(order.missing), detail, reason))

    def _report_shipped(self, r: RobotRecord) -> None:
        self.shipped += 1
        self._report(r, PARTIAL if r.order.missing else SHIPPED)
