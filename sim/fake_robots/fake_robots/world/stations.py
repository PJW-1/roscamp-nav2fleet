"""가짜 중앙 서버(주문판)와 팔(채우기, 교환). SimWorld(fleet_core/test/sim_world.py)에서 옮겼다.

틀린 명령은 예외 대신 violation(글)으로 알린다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.common.commands import HELD, FillCmd, OrderRecord, ReportOrderCmd


def six_orders(at_s=0.0, gap_s=0.0, count=6):
    return [{"at_s": at_s + i * gap_s, "id": f"O-{i + 1:03d}",
             "items": {"AB"[(i + 1) % 2]: 1, "CD"[(i + 1) % 2]: 1}} for i in range(count)]


class Central:
    """주문을 시각에 맞춰 판에 올리고, 보고가 오면 내린다."""

    def __init__(self, orders, violation):
        self.future = sorted(orders, key=lambda o: o["at_s"])
        self.total = len(self.future)
        self.registered: list[OrderRecord] = []
        self.reports: list[ReportOrderCmd] = []
        self._new_reports: list[ReportOrderCmd] = []
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

    def take_report(self, t: float, cmd: ReportOrderCmd) -> None:
        if cmd.order_id in {r.order_id for r in self.reports}:
            self._violation(f"{cmd.order_id} reported twice")
            return
        if cmd.order_id not in {o.order_id for o in self.registered}:
            self._violation(f"{cmd.order_id} reported but not on the board")
            return
        self.reports.append(cmd)
        self._new_reports.append(cmd)
        self.last_report_t = t

    @property
    def finished(self) -> int:
        return sum(1 for r in self.reports if r.status != HELD)

    def all_final(self) -> bool:
        return not self.future and not self.registered and not self._new_reports


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
