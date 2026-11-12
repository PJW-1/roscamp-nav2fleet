"""로봇에서 관제로 가는 것: 결과(위치 보고 앞이나 뒤, 씨드로)와 늦게 닿는 위치 보고."""
from __future__ import annotations

from fleet_core.common.commands import FillCmd

from .obstacle_view import seen_ahead
from .runs import DOCK, HEARTBEAT_LOST, RETREAT, DockRun

DOCK_ERROR = (0.4, 0.3, 0.5)  # 성공한 도킹의 오차(cm, cm, deg)


class _Delay:
    """넣은 보고가 delay 뒤에 순서대로 나온다. 시각은 가짜 시계의 걸음에 맞춰 반올림한다."""

    def __init__(self, delay: float):
        self.delay = delay
        self._items: list = []

    def push(self, t: float, item) -> None:
        self._items.append((round(t + self.delay, 6), item))

    def due(self, t: float) -> list:
        out = [item for at, item in self._items if at <= t]
        self._items = [(at, item) for at, item in self._items if at > t]
        return out

    def clear(self) -> None:
        self._items = []


class Link:
    """ZoneWorld에 섞는다."""

    def _setup_link(self) -> None:
        self.queued: list = []  # 다음 걸음 처음에 낼 결과 (로봇, Result)
        self.queued_arm: list = []
        self._held: list = []  # 끊긴 채 스스로 선 로봇의 결과. 이어지면 낸다
        self.queued_checks: list = []  # 다음 걸음에 낼 CheckClear 답 (로봇, 번호, 답)
        self._early: list = []
        self._late: list = []
        self._lines = {name: _Delay(r.profile.report_delay_s) for name, r in self.robots.items()}

    def _deliver(self, name, result) -> None:
        r = self.robots[name]
        if not r.comm_up and not r.removed and result.reason == HEARTBEAT_LOST:
            self._held.append((name, result))  # 액션 결과는 이어질 때까지 로봇이 쥔다
            return
        if not r.comm_up or r.removed:
            return  # 끊긴 동안의 결과는 잃는다
        self.results.append((self.t, name, result))
        m = self.manager
        if result.kind == RETREAT:
            m.on_retreat_result(name, result.goal_id, result.success, result.reason)
        elif result.kind == DOCK:
            errors = DOCK_ERROR if result.success else (5.0, 5.0, 5.0)
            m.on_dock_result(name, result.goal_id, result.success, *errors)
        elif result.last_done_seq is None:
            m.on_move_result(name, result.goal_id, result.success, result.reason)
        else:
            m.on_move_result(name, result.goal_id, result.success, result.reason,
                             result.last_done_seq)

    def _feedback(self, name, run) -> None:
        """order goal의 피드백: 끝낸 조각, base 끝에 서 있음, goal이 바뀌면 알린다.
        끊긴 동안에는 못 알리고, 이어지면 그때의 것을 알린다."""
        r = self.robots[name]
        now = (run.done_seq, run.waiting, run.goal_id)
        if run.told == now or not r.comm_up or r.removed:
            return
        run.told = now
        self.manager.on_move_feedback(name, run.goal_id, run.done_seq, run.waiting, "")

    def _deliver_queued(self) -> None:
        held, self._held = self._held, []
        for name, result in held:
            self._deliver(name, result)
        queued, self.queued = self.queued, []
        for name, result in queued:
            self._deliver(name, result)
        checks, self.queued_checks = self.queued_checks, []
        for name, check_id, answer in checks:
            if self._reachable(name):
                self.manager.on_clear_check(name, check_id, answer)
        arms, self.queued_arm = self.queued_arm, []
        for job, reason in arms:
            if isinstance(job.cmd, FillCmd):
                self.manager.on_fill_result(job.cmd.goal_id, False, dict(job.placed), {}, reason)
            else:
                self.manager.on_swap_result(job.cmd.goal_id, False, False, False, reason)

    def _finished(self, name, result) -> None:
        """이번 걸음에 끝난 goal. 같은 걸음의 위치 보고 앞이나 뒤로 보낸다."""
        early = self.rng.random() < self.imperfect.result_early_p
        (self._early if early else self._late).append((name, result))

    def _flush(self, early: bool) -> None:
        box = self._early if early else self._late
        items = list(box)
        box.clear()
        for name, result in items:
            self._deliver(name, result)

    def _state_of(self, r) -> str:
        if self.t < r.localizing_until:
            return "LOCALIZING"
        run = r.run
        if run is None:
            return "IDLE"
        if run.blocked:
            return "BLOCKED"
        return "DOCKING" if isinstance(run, DockRun) else "MOVING"

    def _report(self) -> None:
        for name, r in self.robots.items():
            line = self._lines[name]
            if r.removed:
                line.clear()
                continue
            if r.comm_up:
                line.push(self.t, self._sample(r))
            for report in line.due(self.t):
                if r.comm_up:
                    self.reports_sent += 1
                    self.manager.on_robot_state(name, *report)
                    self.reaction.on_report(self.t, name, report[3])

    def _sample(self, r):
        state = self._state_of(r)
        if state == "LOCALIZING":
            return (0.0, 0.0, 0.0, state, self._battery(r))
        sigma, jump = self.schedule.noise(r.name, self.t)
        sigma = max(sigma, self.imperfect.pose_noise_m)
        x = r.x + jump + (self.rng.gauss(0.0, sigma) if sigma else 0.0)
        y = r.y + (self.rng.gauss(0.0, sigma) if sigma else 0.0)
        report = (x, y, r.yaw, state, self._battery(r))
        seen = (seen_ahead(r.x, r.y, r.yaw, self.discs(r), self.params.obstacle_look_m)
                if state == "BLOCKED" else None)
        if seen is None:
            return report
        return (*report, "", *seen, "unknown")  # robot_id는 비우고 막은 것(계약 0.12-draft)

    def _battery(self, r) -> float:
        battery = self.batteries.get(r.name)
        if battery is None:
            return 0.0
        standing = r.run is None or isinstance(r.run, DockRun)
        return battery.percent(self.t, r.x, r.y, standing)
