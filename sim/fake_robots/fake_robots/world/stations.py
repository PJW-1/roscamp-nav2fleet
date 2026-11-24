"""가짜 중앙 서버(주문판)와 팔(채우기, 교환). SimWorld(fleet_core/test/sim_world.py)에서 옮겼다.

틀린 명령은 예외 대신 violation(글)으로 알린다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.common.commands import (
    HELD,
    HOLD_ROBOT_FAULT,
    TRIP_STATUSES,
    FillCmd,
    OrderRecord,
    ReportOrderCmd,
    ReportTripCmd,
)


def six_orders(at_s=0.0, gap_s=0.0, count=6):
    return [{"at_s": at_s + i * gap_s, "id": f"O-{i + 1:03d}",
             "items": {"AB"[(i + 1) % 2]: 1, "CD"[(i + 1) % 2]: 1}} for i in range(count)]


class Central:
    """주문을 시각에 맞춰 판에 올리고, 최종 보고가 오면 내린다.

    ROBOT_FAULT 보류는 central이 손으로 닫는다(v2.4). 세계 안에서는 아무도 닫지 않으므로
    판에 남긴 채 끝난 것으로 본다(closed_by_hand). ARM_FAULT 보류는 최종 보고를 기다린다.
    """

    def __init__(self, orders, violation):
        self.future = sorted(orders, key=lambda o: o["at_s"])
        self.total = len(self.future)
        self.registered: list[OrderRecord] = []
        self.reports: list[ReportOrderCmd] = []  # 최종 보고만(SHIPPED, PARTIAL, CANCELED)
        self.held: dict[str, ReportOrderCmd] = {}  # 지금 보류 중인 주문(최종 보고가 오면 지움)
        self.hold_reports: list[ReportOrderCmd] = []  # 받은 보류 보고 전부, 도착 순
        self._new_reports: list[ReportOrderCmd] = []
        self.trip_reports: list[tuple[float, ReportTripCmd]] = []  # 진행 단계 보고, 도착 순
        self._violation = violation
        self.last_report_t: float | None = None

    def board(self, t: float) -> list[OrderRecord]:
        finals = {r.order_id for r in self._new_reports}
        self._new_reports = []
        self.registered = [o for o in self.registered if o.order_id not in finals]
        while self.future and self.future[0]["at_s"] <= t:
            order = self.future.pop(0)
            self.registered.append(OrderRecord(order["id"], dict(order["items"])))
        return list(self.registered)

    def take_trip_report(self, t: float, cmd: ReportTripCmd) -> None:
        """진행 단계 보고. 실제로는 브리지가 받는다. 판에 남고 끝이 아니다.

        단계 순서는 보지 않는다(IR-25). 대기열로 되돌린 주문은 다시 ASSIGNED가 된다.
        """
        if cmd.status not in TRIP_STATUSES:
            self._violation(f"{cmd.trip_id} trip report with status {cmd.status}")
            return
        if cmd.trip_id in {r.order_id for r in self.reports}:
            self._violation(f"{cmd.trip_id} {cmd.status} after its final report")
            return
        if cmd.trip_id not in {o.order_id for o in self.registered}:
            self._violation(f"{cmd.trip_id} {cmd.status} reported but not on the board")
            return
        hold = self.held.get(cmd.trip_id)
        if hold is not None and hold.hold_reason == HOLD_ROBOT_FAULT:
            self._violation(f"{cmd.trip_id} {cmd.status} after a ROBOT_FAULT hold")
            return
        self.trip_reports.append((t, cmd))

    def take_report(self, t: float, cmd: ReportOrderCmd) -> None:
        if cmd.order_id in {r.order_id for r in self.reports}:
            self._violation(f"{cmd.order_id} reported twice")
            return
        if cmd.order_id not in {o.order_id for o in self.registered}:
            self._violation(f"{cmd.order_id} reported but not on the board")
            return
        hold = self.held.get(cmd.order_id)
        if hold is not None and hold.hold_reason == HOLD_ROBOT_FAULT:
            # ROBOT_FAULT 보류 주문은 관제가 더 진행하지 않는다(central이 닫는다)
            self._violation(f"{cmd.order_id} {cmd.status} after a ROBOT_FAULT hold")
            return
        if cmd.status == HELD:  # 보류는 끝이 아니다. 판에 남고 최종 보고를 한 번 더 받는다
            # 사유가 바뀌는 것(ARM_FAULT 뒤 담은 채 격리)은 된다. 같은 사유의 반복은 위반이다
            if any(r.order_id == cmd.order_id and r.hold_reason == cmd.hold_reason
                   for r in self.hold_reports):
                self._violation(f"{cmd.order_id} held twice for {cmd.hold_reason}")
                return
            self.held[cmd.order_id] = cmd
            self.hold_reports.append(cmd)
            self.last_report_t = t
            return
        self.held.pop(cmd.order_id, None)
        self.reports.append(cmd)
        self._new_reports.append(cmd)
        self.last_report_t = t

    @property
    def finished(self) -> int:
        return len(self.reports)

    @property
    def closed_by_hand(self) -> list[str]:
        """central이 손으로 닫을 주문(ROBOT_FAULT 보류), 보류 순."""
        return [o for o, r in self.held.items() if r.hold_reason == HOLD_ROBOT_FAULT]

    def all_final(self) -> bool:
        waiting = set(self.closed_by_hand)
        return (not self.future and not self._new_reports
                and all(o.order_id in waiting for o in self.registered))


@dataclass
class ArmJob:
    cmd: object
    next_at: float
    left: list = field(default_factory=list)  # 채우기: 아직 놓을 품목
    placed: dict = field(default_factory=dict)
    missing: dict = field(default_factory=dict)
    steps: list = field(default_factory=list)  # 교환: 남은 take, put


class Arms:
    def __init__(self, site, stock, s_per_item, swap_step_s, violation):
        self.site = site
        self.stock = dict(stock)
        self.s_per_item, self.swap_step_s = s_per_item, swap_step_s
        self.jobs: dict[str, ArmJob] = {}
        self.shipped: dict[str, int] = {}
        self._violation = violation

    def _station_of(self, cmd):
        return self.site.stations.get(cmd.station_id)

    def start_fill(self, t, cmd, robot, slow) -> None:
        station = self._station_of(cmd)
        if station is None or station.action != "fill" or station.server in self.jobs:
            self._violation(f"{cmd} at a wrong or busy station")
            return
        if robot.docked_at != cmd.station_id:
            self._violation(f"{cmd} but {cmd.robot} is docked at {robot.docked_at!r}")
        self.jobs[station.server] = ArmJob(
            cmd, t + self.s_per_item * slow,
            left=[k for k, v in sorted(cmd.items.items()) for _ in range(v)])

    def start_swap(self, t, cmd, robot, slow) -> None:
        station = self._station_of(cmd)
        if station is None or station.action != "swap" or cmd.arm in self.jobs:
            self._violation(f"{cmd} at a wrong or busy station")
            return
        if robot.docked_at != cmd.station_id:
            self._violation(f"{cmd} but {cmd.robot} is docked at {robot.docked_at!r}")
        self.jobs[cmd.arm] = ArmJob(cmd, t + self.swap_step_s * slow,
                                    steps=["take", "put"] if cmd.take_full else ["put"])

    def cancel(self, server, goal_id) -> ArmJob | None:
        job = self.jobs.get(server)
        if job is None or job.cmd.goal_id != goal_id:
            return None
        del self.jobs[server]
        return job

    def run(self, t, robots, slow_of, manager) -> None:
        for server, job in list(self.jobs.items()):
            if t < job.next_at:
                continue
            robot = robots[job.cmd.robot]
            slow = slow_of(job.cmd.station_id)
            if isinstance(job.cmd, FillCmd):
                self._fill(t, server, job, robot, slow, manager)
            else:
                self._swap(t, server, job, robot, slow, manager)

    def _fill(self, t, server, job, robot, slow, manager) -> None:
        if job.left:
            kind = job.left.pop(0)
            if self.stock.get(kind, 0) > 0:
                self.stock[kind] -= 1
                robot.basket[kind] = robot.basket.get(kind, 0) + 1
                job.placed[kind] = job.placed.get(kind, 0) + 1
            else:
                job.missing[kind] = job.missing.get(kind, 0) + 1
            job.next_at = t + self.s_per_item * slow
            return
        del self.jobs[server]
        manager.on_fill_result(job.cmd.goal_id, True, dict(job.placed), dict(job.missing))

    def _swap(self, t, server, job, robot, slow, manager) -> None:
        if job.steps and job.steps[0] == "take":
            job.steps.pop(0)
            for kind, count in robot.basket.items():
                self.shipped[kind] = self.shipped.get(kind, 0) + count
            robot.basket, robot.has_basket = {}, False
            job.next_at = t + self.swap_step_s * slow
            return
        took = job.cmd.take_full
        if job.steps:
            job.steps.pop(0)
            robot.has_basket = True
        del self.jobs[server]
        manager.on_swap_result(job.cmd.goal_id, True, took, True)
