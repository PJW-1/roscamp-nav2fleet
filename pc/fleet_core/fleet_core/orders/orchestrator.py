# ---- OrderOrchestrator: 주문판을 받아 로봇에 주문을 배정하고 진행 ----
from __future__ import annotations

from fleet_core.common.commands import (
    FILL,
    NO_ROBOT,
    PARTIAL,
    SHIPPED,
    SWAP,
    ArmDeclined,
    ArmFailedCmd,
    ArmFillResult,
    ArmLate,
    ArmLost,
    ArmResetCmd,
    ArmSwapResult,
    CancelDispatchCmd,
    DispatchCmd,
    EventOut,
    ReleaseCmd,
    ReportOrderCmd,
    RobotFaultCmd,
)

from .faults import Faults
from .records import (
    EMPTY,
    FILLING,
    FULL,
    NONE,
    SWAPPING,
    _LostJob,
    _merge,
    _Robot,
    _subtract,
)
from .stations import Stations

# ---- OrderOrchestrator 생성 ----
STOP = "STOP"


class OrderOrchestrator(Faults, Stations):
    def __init__(self, stations, params, robots, clock):
        self.params = params
        self.shipped = 0
        self._stations = stations
        self._clock = clock
        self._dispatch_seq = 0
        self._bind_seq = 0
        self._robots = {name: _Robot(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._board: list = []
        self._reported: set[str] = set()
        self._open_calls: list[int] = []
        self._work = next(s.id for s in stations.values() if s.action == FILL)
        self._outbound = next(s.id for s in stations.values() if s.action == SWAP)
        self._lost_jobs: dict[int, _LostJob] = {}

    # ---- 주문판, 배차, 팔 보고 입력 처리 ----
    def on_order_board(self, run_mode, orders) -> None:
        self._run_mode = run_mode
        self._board = list(orders)
        self._reported &= {o.order_id for o in self._board}
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
        if isinstance(report, ArmLate):
            if report.kind == FILL:
                self._late_fill(report.goal_id, report.placed)
            else:
                self._late_swap(report.goal_id, report.took_full, report.put_empty_done)
            return
        r = self._robots.get(report.robot)
        if r is None:
            return
        if isinstance(report, ArmDeclined):
            r.asked = ""
        elif isinstance(report, ArmFillResult):
            self._on_fill_result(r, report.success, report.placed)
        elif isinstance(report, ArmSwapResult):
            self._on_swap_result(r, report.took_full, report.put_empty_done)
        elif isinstance(report, ArmLost):
            self._on_arm_lost(r, report)

    # ---- 팔 결과를 주문에 반영하고 결과를 잃은 작업 기록 ----
    def _on_fill_result(self, r: _Robot, success, placed) -> None:
        r.asked = ""
        _merge(r.order.placed, placed)
        if success:
            r.order.missing = _subtract(r.order.items, r.order.placed)
            return
        self._out.append(ArmFailedCmd(r.name))

    def _on_arm_lost(self, r: _Robot, lost: ArmLost) -> None:
        if lost.cleared:
            order_id, bound = lost.order_id, lost.bound
        else:
            r.asked = ""
            order_id, bound = self.order_of(r.name), r.bound
        self._lost_jobs[lost.goal_id] = _LostJob(r.name, lost.server, lost.kind, order_id,
                                                 cleared=lost.cleared, bound=bound,
                                                 isolated=lost.isolated)

    def _on_swap_result(self, r: _Robot, took_full, put_empty_done) -> None:
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
        if r is None:
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

    def station_job(self, robot) -> tuple[str, int] | None:
        r = self._robots.get(robot)
        if r is None or r.stage not in (FILLING, SWAPPING):
            return None
        if r.stage == SWAPPING:
            return SWAP, 1
        left = _subtract(r.order.items, r.order.placed, r.order.missing) if r.order else {}
        return FILL, sum(left.values())

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

    def _fail(self, r: _Robot, reason: str) -> None:
        self._out.append(RobotFaultCmd(r.name, reason))

    def _leave_station(self, r: _Robot) -> None:
        r.stage = ""
        self._out.append(ReleaseCmd(r.name))

    # ---- 대기 주문 계산과 배차 요청 맞추기 ----
    def _pending(self) -> list:
        bound = {r.order.order_id for r in self._robots.values() if r.order}
        return [o for o in self._board
                if o.order_id not in bound and o.order_id not in self._reported]

    def _sync_calls(self) -> None:
        wanted = len(self._pending())
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
    def _report(self, r: _Robot, status: str, detail: str = "") -> None:
        order, r.order = r.order, None
        self._reported.add(order.order_id)
        self._out.append(ReportOrderCmd(order.order_id, r.name, status, dict(order.placed),
                                        dict(order.missing), detail))

    def _report_shipped(self, r: _Robot) -> None:
        self.shipped += 1
        self._report(r, PARTIAL if r.order.missing else SHIPPED)
