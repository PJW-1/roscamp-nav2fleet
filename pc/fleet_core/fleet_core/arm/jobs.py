# ---- 팔 작업(채우기, 교환) 실행과 재시도, 시간 초과 처리 ----
from __future__ import annotations

from dataclasses import dataclass

from fleet_core.common.commands import (
    FILL,
    SWAP,
    ArmDeclined,
    ArmDropCmd,
    ArmFailedCmd,
    ArmFault,
    ArmFillCmd,
    ArmFillResult,
    ArmLate,
    ArmLost,
    ArmResetCmd,
    ArmStopCmd,
    ArmSwapCmd,
    ArmSwapResult,
    CancelCmd,
    EventOut,
    FillCmd,
    SwapCmd,
)

# ---- 작업 상수와 작업, 로봇 기록 ----
STOP = "STOP"
CANCELED = "CANCELED"
ARM_ERROR = "ERROR"

_CODES = {FILL: ("FILL_RETRY", "FILL_FAILED"), SWAP: ("SWAP_RETRY", "SWAP_FAILED")}


@dataclass
class _Job:
    server: str
    goal_id: int
    kind: str
    sent_at: float
    canceled_at: float | None = None
    timed_out: bool = False


@dataclass
class _Robot:
    name: str
    job: _Job | None = None
    retry_at: float = 0.0
    attempts: int = 0
    last: tuple[_Job, str] | None = None


# ---- ArmJobs 생성과 실행 모드 처리 ----
class ArmJobs:
    def __init__(self, stations, params, robots, clock, next_goal_id=None):
        self.params = params
        self._stations = stations
        self._clock = clock
        self._next_goal_id = next_goal_id or self._own_goal_id
        self._goal_seq = 0
        self._robots = {name: _Robot(name) for name in robots}
        self._out: list = []
        self._run_mode: str | None = None
        self._cleared_jobs: dict[str, tuple[int, float]] = {}
        # 팔 이름 -> 그 팔이 일하는 스테이션 서버, ERROR를 보고 있는 팔
        self._server_of = {arm: s.server for s in stations.values() for arm in s.arms}
        self._arm_errors: set[str] = set()

    # TODO(Q-05) PAUSE 때 담기 목표 안에서 멈추는 신호 미정(W-42), 지금은 STOP만 취소
    def on_run_mode(self, run_mode) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for r in self._robots.values():
                if r.job and r.job.canceled_at is None:
                    self._cancel(r)
        self._run_mode = run_mode

    # ---- 팔 결과 보고 받기 ----
    def on_fill_result(self, goal_id, success, placed, missing, reason="",
                       placed_confirmed=None, uncertain=None) -> None:
        self._forget_cleared_job(goal_id)
        r = self._unexpired_owner(goal_id, FILL)
        if r is None:
            self._out.append(ArmLate(goal_id, FILL, placed=placed))
            return
        r.last, r.job = (r.job, reason), None
        # 확인된 수를 안 주면 placed가 확인된 수다(0.6 호환)
        confirmed = placed if placed_confirmed is None else placed_confirmed
        self._out.append(ArmFillResult(r.name, success, placed, reason, dict(missing),
                                       dict(confirmed), dict(uncertain or {})))

    def on_swap_result(self, goal_id, success, took_full, put_empty_done, reason="") -> None:
        self._forget_cleared_job(goal_id)
        r = self._unexpired_owner(goal_id, SWAP)
        if r is None:
            self._out.append(ArmLate(goal_id, SWAP, took_full=took_full,
                                     put_empty_done=put_empty_done))
            return
        r.last, r.job = (r.job, reason), None
        self._out.append(ArmSwapResult(r.name, took_full, put_empty_done))

    # ---- 팔 상태: ERROR로 바뀔 때만 한 번 반응한다 ----
    def on_arm_state(self, arm, state, detail="", holding=False) -> None:
        server = self._server_of.get(arm)
        if server is None:
            return
        if state != ARM_ERROR:
            self._arm_errors.discard(arm)
            return
        if arm in self._arm_errors:
            return
        self._arm_errors.add(arm)
        held = " holding an item" if holding else ""
        r = next((r for r in self._robots.values() if r.job and r.job.server == server), None)
        if r is None:
            # TODO(Q-03) 팔 고장 동안 새 배차를 막을지 미정, 지금은 사건만 낸다
            self._event("ERROR", "ARM_ERROR", f"{arm} ERROR {detail}{held}, no job running",
                        arm)
            return
        job, r.job = r.job, None
        reason = f"{arm} ERROR {detail} during {server} goal {job.goal_id}"
        # 결과를 잃은 작업으로 남겨, 나중에 온 결과는 늦은 결과로 한 번만 처리된다
        self._out.append(ArmLost(r.name, job.goal_id, job.server, job.kind))
        self._out.append(ArmFault(r.name, job.kind, job.server, reason))
        self._event("ERROR", "ARM_ERROR", f"{reason}{held}, {r.name} job given up", arm)

    # ---- 시간 초과 점검 ----
    def check_timeouts(self, now: float) -> None:
        for r in self._robots.values():
            job = r.job
            if not job:
                continue
            limit = (self.params.fill_timeout_s if job.kind == FILL
                     else self.params.swap_timeout_s)
            settle = self.params.robot_timeout_settle_s
            if job.canceled_at is None and now - job.sent_at > limit:
                # IR-62: 실패가 아니라 결과 모름. 작업과 로봇은 쥔 채 취소하고 결과를 기다린다
                self._cancel(r)
                job.timed_out = True
                self._event("WARN", "ROBOT_TIMEOUT",
                            f"{job.server} goal {job.goal_id} has no result after {limit:.0f} s: "
                            f"result unknown, canceled, settles within {settle:.0f} s", r.name)
                continue
            if self._expired(job, now):
                self._give_up(r, job)

    # ---- 취소한 작업의 기다림: 시간 초과는 robot_timeout_settle_s, 정지 취소는 arm_cancel_wait_s ----
    def _wait_of(self, job: _Job) -> float:
        p = self.params
        return p.robot_timeout_settle_s if job.timed_out else p.arm_cancel_wait_s

    def _expired(self, job: _Job, now: float) -> bool:
        return job.canceled_at is not None and now - job.canceled_at >= self._wait_of(job)

    def _give_up(self, r: _Robot, job: _Job) -> None:
        r.job = None
        self._out.append(ArmLost(r.name, job.goal_id, job.server, job.kind))
        reason = f"{job.server} goal {job.goal_id} never reported"
        self._out.append(ArmFault(r.name, job.kind, job.server, reason))
        self._event("ERROR", "ARM_NO_RESULT", f"{reason} within {self._wait_of(job):.0f} s",
                    r.name)

    # ---- 결과의 주인: 기다림이 이미 끝난 작업이면 먼저 포기하고, 결과는 늦은 것이 된다 ----
    def _unexpired_owner(self, goal_id: int, kind: str) -> _Robot | None:
        r = self._job_owner(goal_id, kind)
        if r is not None and self._expired(r.job, self._clock()):
            self._give_up(r, r.job)
            return None
        return r

    # ---- 팔 명령 처리 ----
    def on_command(self, command) -> None:
        r = self._robots.get(command.robot)
        if r is None:
            return
        match command:
            case ArmFillCmd():
                self._start(r, FILL, command)
            case ArmSwapCmd():
                self._start(r, SWAP, command)
            case ArmStopCmd():
                if r.job and r.job.canceled_at is None:
                    self._cancel(r)
            case ArmDropCmd():
                self._drop(r, command)
            case ArmFailedCmd():
                self._attempt_failed(r)
            case ArmResetCmd():
                if command.forget:
                    self._robots[r.name] = _Robot(r.name)
                else:
                    r.attempts = 0

    # ---- 출력과 조회 ----
    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def sent_at(self, robot) -> float | None:
        r = self._robots.get(robot)
        return r.job.sent_at if r and r.job else None

    def attempts(self, robot) -> int:
        r = self._robots.get(robot)
        return r.attempts if r else 0

    def _own_goal_id(self) -> int:
        self._goal_seq += 1
        return self._goal_seq

    def _event(self, level, code, detail, robot="") -> None:
        self._out.append(EventOut(level, code, detail, robot))

    # ---- 작업 시작과 목표 허용 판단 ----
    def _start(self, r: _Robot, kind: str, command) -> None:
        server = self._stations[command.station_id].server
        if r.job or not self._goal_allowed(r, server):
            self._out.append(ArmDeclined(r.name))
            return
        goal_id = self._next_goal_id()
        r.job, r.last = _Job(server, goal_id, kind, self._clock()), None
        if kind == FILL:
            self._out.append(FillCmd(goal_id, r.name, command.station_id, command.order_id,
                                     command.items))
        else:
            self._out.append(SwapCmd(server, goal_id, r.name, command.station_id,
                                     command.order_id, take_full=command.take_full,
                                     put_empty=command.put_empty))

    def _goal_allowed(self, r: _Robot, server: str) -> bool:
        now = self._clock()
        cleared = self._cleared_jobs.get(server)
        if cleared and now >= cleared[1]:
            del self._cleared_jobs[server]
            cleared = None
        return not cleared and now >= r.retry_at

    def _job_owner(self, goal_id: int, kind: str) -> _Robot | None:
        for r in self._robots.values():
            if r.job and r.job.goal_id == goal_id and r.job.kind == kind:
                return r
        return None

    def _forget_cleared_job(self, goal_id: int) -> None:
        for server, (cleared_goal, _) in list(self._cleared_jobs.items()):
            if cleared_goal == goal_id:
                del self._cleared_jobs[server]

    # ---- 작업 취소와 버리기 ----
    def _cancel(self, r: _Robot) -> None:
        r.job.canceled_at = self._clock()
        self._out.append(CancelCmd(r.job.server, r.job.goal_id))

    def _drop(self, r: _Robot, command: ArmDropCmd) -> None:
        job = r.job
        if job is None:
            return
        self._out.append(CancelCmd(job.server, job.goal_id))
        self._cleared_jobs[job.server] = (
            job.goal_id, self._clock() + self.params.arm_cancel_wait_s)
        self._out.append(ArmLost(r.name, job.goal_id, job.server, job.kind, cleared=True,
                                 isolated=command.isolated, order_id=command.order_id,
                                 bound=command.bound))
        r.job = None

    # ---- 실패한 시도의 재시도 또는 팔 고장 보고 ----
    def _attempt_failed(self, r: _Robot) -> None:
        if r.last is None:
            return
        (job, reason), r.last = r.last, None
        if reason == CANCELED and job.canceled_at is not None and not job.timed_out:
            return
        retry_code, failed_code = _CODES[job.kind]
        r.attempts += 1
        r.retry_at = self._clock() + self.params.arm_retry_wait_s
        if r.attempts > self.params.move_retry_limit:
            detail = f"{job.server}: {reason}"
            self._out.append(ArmFault(r.name, job.kind, job.server, detail))
            self._event("ERROR", failed_code, detail, r.name)
        else:
            self._event("WARN", retry_code,
                        f"{job.server} attempt {r.attempts} failed: {reason}", r.name)
