# ---- 팔 작업(채우기, 교환) 실행과 재시도, 시간 초과 처리 ----
from __future__ import annotations

from dataclasses import dataclass, field

from fleet_core.arm.fill_outcome import HOLD, PAUSE, RETRY, next_after_fill
from fleet_core.common.commands import (
    FILL,
    RESUME_REFILL,
    RESUME_SET_PAUSE_OFF,
    SWAP,
    ArmDeclined,
    ArmDropCmd,
    ArmFailedCmd,
    ArmFault,
    ArmFillCmd,
    ArmFillPaused,
    ArmFillPlaced,
    ArmFillProgress,
    ArmFillResumed,
    ArmFillResult,
    ArmLate,
    ArmLost,
    ArmResetCmd,
    ArmStationBlocked,
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
    order_id: str = ""
    items: dict[str, int] = field(default_factory=dict)
    placed_total: int = 0  # 마지막 feedback의 담은 수
    phases: dict[str, str] = field(default_factory=dict)  # 팔 -> 마지막 진행 단계
    unlocked_holding: str = ""  # 쥔 채 락이 풀린 팔: 목표가 끝나면 보류


# 목표 items 앞에서부터 n개
def first_items(items: dict[str, int], n: int) -> dict[str, int]:
    out = {}
    for item, count in items.items():
        take = min(count, n)
        if take <= 0:
            break
        out[item] = take
        n -= take
    return out


@dataclass
class _Robot:
    name: str
    job: _Job | None = None
    retry_at: float = 0.0
    attempts: int = 0
    last: tuple[_Job, str] | None = None
    blocked: str = ""  # 담기 일시 중지 · 보류 뒤 주문 쪽이 초기화할 때까지 새 목표를 막는 사유


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
        # 상태가 끊긴 팔, 팔별 boot_id · 쥐고 있음, 작업대 막힘 사유, 재시작으로 버린 목표
        self._arm_unknown: set[str] = set()
        self._boot: dict[str, str] = {}
        self._holding: dict[str, bool] = {}
        self._held_goal: dict[str, int] = {}  # 팔 -> holding=true를 마지막으로 본 담기 목표
        self._station_block: dict[str, str] = {}
        self._dropped_goals: set[int] = set()
        # 팔 재시작으로 목표가 사라진 작업대 서버 -> 그 담기를 하던 로봇 ([복구] 대기)
        self._resume_wait: dict[str, str] = {}

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
        if goal_id in self._dropped_goals:
            self._dropped_goals.discard(goal_id)
            self._event("WARN", "ARM_LATE_RESULT",
                        f"goal {goal_id} reported placed {placed} after the arm restarted: "
                        "dropped, the trip goes on from the last feedback")
            return
        self._forget_cleared_job(goal_id)
        r = self._unexpired_owner(goal_id, FILL)
        if r is None:
            self._out.append(ArmLate(goal_id, FILL, placed=placed,
                                     uncertain=dict(uncertain) if uncertain else None))
            return
        job = r.job
        r.last, r.job = (job, reason), None
        # 확인된 수를 안 주면 placed가 확인된 수다(0.6 호환)
        confirmed = placed if placed_confirmed is None else placed_confirmed
        self._out.append(ArmFillResult(r.name, success, placed, reason, dict(missing),
                                       dict(confirmed), dict(uncertain or {}),
                                       held=bool(job.unlocked_holding)))
        uncertain = dict(uncertain or {})
        if not success or job.unlocked_holding or any(c > 0 for c in uncertain.values()):
            self._judge_fill(r, job, reason, uncertain)

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
    # ---- 팔 상태 끊김: 상태 모름, 그 작업대 배차를 막음 (팔 관제 합의 ⑥, 담기는 그대로) ----
    def on_arm_state_lost(self, arm) -> None:
        if arm not in self._server_of or arm in self._arm_unknown:
            return
        self._arm_unknown.add(arm)
        self._event("WARN", "ARM_STATE_LOST",
                    f"{arm} state unknown: no state within the comm timeout, dispatch blocked", arm)
        self._update_stations()

    # ---- 담기 진행: 담은 수가 늘면 진행 보고 (팔 관제 합의 ⑫) ----
    def on_fill_feedback(self, goal_id, arm, phase, placed_total) -> None:
        r = self._job_owner(goal_id, FILL)
        if r is None:
            return
        job = r.job
        if phase:
            job.phases[arm] = phase
        if placed_total != job.placed_total:
            job.placed_total = placed_total
            self._out.append(ArmFillProgress(r.name, first_items(job.items, placed_total)))

    # ---- 관리자 [복구] (팔 관제 합의 ⑧ · ⑪) ----
    def on_resume_arm(self, target) -> tuple[str, str]:
        servers = set(self._server_of.values())
        server = target if target in servers else self._server_of.get(target)
        if server is None:
            return "", "NOT_FOUND"
        robot = self._resume_wait.pop(server, None)
        if robot is not None:
            # 재시작으로 사라진 목표: 주문 쪽이 남은 상품으로 새 목표를 다시 요청한다
            self._out.append(ArmFillResumed(robot))
            self._event("INFO", "FILL_RESUMED",
                        f"{server} lost its goal in the restart: sending the rest as a new goal",
                        robot)
            return RESUME_REFILL, ""
        r = self._fill_owner(server)
        if r is None:
            return "", "ORDER_STATE"
        self._event("INFO", "FILL_RESUMED",
                    f"{server} goal {r.job.goal_id}: set_pause(false) to both arms", r.name)
        return RESUME_SET_PAUSE_OFF, ""

    # ---- 상태판 팔 줄: (진행 단계, 담는 운반 건) ----
    def arm_task(self, arm) -> tuple[str, str]:
        server = self._server_of.get(arm)
        r = next((r for r in self._robots.values() if r.job and r.job.server == server), None)
        if r is None:
            return "", ""
        return r.job.phases.get(arm, ""), r.job.order_id

    def on_arm_state(self, arm, state, detail="", holding=False, boot_id="", lock=True) -> None:
        server = self._server_of.get(arm)
        if server is None:
            return
        if arm in self._arm_unknown:
            self._arm_unknown.discard(arm)
            self._event("INFO", "ARM_STATE_BACK", f"{arm} state back: {state}", arm)
        before, held_before = self._boot.get(arm, ""), self._holding.get(arm, False)
        if boot_id:
            self._boot[arm] = boot_id
        self._holding[arm] = holding
        # 직전 상태의 holding은 같은 담기 목표 안에서 본 것만 쓴다
        owner = self._fill_owner(server)
        goal = owner.job.goal_id if owner is not None else None
        held_in_goal = held_before and goal is not None and self._held_goal.get(arm) == goal
        if holding and goal is not None:
            self._held_goal[arm] = goal
        if before and boot_id and boot_id != before:
            self._on_arm_restart(arm, server, held_before)
        if state == ARM_ERROR:
            self._on_arm_error(arm, server, detail, holding)
        else:
            self._arm_errors.discard(arm)
            # 그리퍼를 연 뒤의 상태만 받아도 같은 목표의 직전 상태에서 쥐고 있었으면 같다
            if not lock and (holding or held_in_goal):
                self._on_lock_released_holding(arm, server)
        self._update_stations()

    # ---- ERROR는 복구할 수 없는 고장: 하던 작업을 포기하고 보류 ----
    def _on_arm_error(self, arm, server, detail, holding) -> None:
        if arm in self._arm_errors:
            return
        self._arm_errors.add(arm)
        held = " holding an item" if holding else ""
        r = next((r for r in self._robots.values() if r.job and r.job.server == server), None)
        waiting = self._resume_wait.pop(server, None)
        if waiting is not None:
            # [복구]를 기다리던 담기: 담은 수는 이미 주문에 있다
            reason = f"{arm} ERROR {detail} while {server} waits for [복구]"
            self._out.append(ArmFault(waiting, FILL, server, reason))
            self._event("ERROR", "ARM_ERROR", f"{reason}{held}, {waiting} held", arm)
            return
        if r is None:
            self._event("ERROR", "ARM_ERROR",
                        f"{arm} ERROR {detail}{held}, no job running, dispatch blocked", arm)
            return
        job, r.job = r.job, None
        reason = f"{arm} ERROR {detail} during {server} goal {job.goal_id}"
        # 결과를 잃은 작업으로 남겨, 나중에 온 결과는 늦은 결과로 한 번만 처리된다
        self._out.append(ArmLost(r.name, job.goal_id, job.server, job.kind))
        self._out.append(ArmFault(r.name, job.kind, job.server, reason))
        self._event("ERROR", "ARM_ERROR", f"{reason}{held}, {r.name} job given up", arm)

    # ---- 담기 중 쥔 채 락 해제: 담기는 PAUSED로 [복구]를 기다리고, 목표가 끝나면 보류 (팔 관제 합의 ⑤) ----
    def _on_lock_released_holding(self, arm, server) -> None:
        r = self._fill_owner(server)
        if r is None or r.job.unlocked_holding:
            return  # 취소를 기다리는 중이어도 표시해 결과가 오면 보류한다
        r.job.unlocked_holding = arm
        self._event("WARN", "ARM_LOCK_RELEASED",
                    f"{arm} lock released holding an item during {server} goal {r.job.goal_id}: "
                    "the fill waits for [복구], held when the goal ends", arm)

    # ---- 팔 재시작([초기화]): 담기는 [복구] 대기, 쥐고 있었으면 보류 (팔 관제 합의 ⑪) ----
    def _on_arm_restart(self, arm, server, held_before) -> None:
        # 재시작한 팔이나 같은 작업대의 다른 팔이 쥐고 있던 상품은 행방을 모른다
        held = held_before or any(self._holding.get(a, False)
                                  for a in self._arms_of(server) if a != arm)
        waiting = self._resume_wait.get(server)
        if waiting is not None:
            if held:
                del self._resume_wait[server]
                reason = f"{arm} restarted holding an item while {server} waits for [복구]"
                self._out.append(ArmFault(waiting, FILL, server, reason))
                self._event("ERROR", "ARM_RESTARTED", f"{reason}, item unaccounted, held", arm)
            return
        r = self._fill_owner(server)
        if r is None or r.job.canceled_at is not None:
            self._event("INFO", "ARM_RESTARTED", f"{arm} restarted, no fill running", arm)
            return
        job, r.job, r.last = r.job, None, None
        held = held or bool(job.unlocked_holding)
        placed = first_items(job.items, job.placed_total)
        self._dropped_goals.add(job.goal_id)
        # 담은 수는 바로 주문에 더한다(치움 · 격리 · 고장에도 빠지지 않게)
        self._out.append(ArmFillPlaced(r.name, placed, held=held))
        if held:
            reason = (f"{arm} restarted holding an item during {server} goal {job.goal_id}, "
                      f"placed {placed}")
            self._out.append(ArmFault(r.name, FILL, job.server, reason))
            self._event("ERROR", "ARM_RESTARTED", f"{reason}, item unaccounted, held", arm)
            return
        self._resume_wait[server] = r.name
        self._event("WARN", "ARM_RESTARTED",
                    f"{arm} restarted during {server} goal {job.goal_id}, placed {placed}: "
                    "waits for [복구]", arm)

    def _arms_of(self, server) -> tuple[str, ...]:
        return tuple(a for a, s in self._server_of.items() if s == server)

    def _fill_owner(self, server) -> _Robot | None:
        return next((r for r in self._robots.values()
                     if r.job and r.job.kind == FILL and r.job.server == server), None)

    # ---- 팔이 ERROR이거나 상태 모름이면 그 작업대 배차 · 새 목표를 막음 (팔 관제 합의 ④ · ⑥) ----
    def _block_of(self, station) -> str:
        why = [f"{a} ERROR" for a in station.arms if a in self._arm_errors]
        why += [f"{a} state unknown" for a in station.arms if a in self._arm_unknown]
        return ", ".join(why)

    def _update_stations(self) -> None:
        for s in self._stations.values():
            if s.action != FILL:
                continue
            why = self._block_of(s)
            if self._station_block.get(s.id, "") != why:
                self._station_block[s.id] = why
                self._out.append(ArmStationBlocked(s.id, why))

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
                self._forget_resume(r.name)
            case ArmFailedCmd():
                self._attempt_failed(r)
            case ArmResetCmd():
                self._forget_resume(r.name)
                if command.forget:
                    self._robots[r.name] = _Robot(r.name)
                else:
                    r.attempts, r.blocked = 0, ""

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
        r.job, r.last = _Job(server, goal_id, kind, self._clock(), order_id=command.order_id,
                             items=dict(command.items) if kind == FILL else {}), None
        if kind == FILL:
            self._out.append(FillCmd(goal_id, r.name, command.station_id, command.order_id,
                                     command.items))
        else:
            self._out.append(SwapCmd(server, goal_id, r.name, command.station_id,
                                     command.order_id, take_full=command.take_full,
                                     put_empty=command.put_empty))

    # 그 로봇이 맡던 [복구] 대기를 지움 (치움 · 격리 · 다시 투입)
    def _forget_resume(self, robot) -> None:
        for server, owner in list(self._resume_wait.items()):
            if owner == robot:
                del self._resume_wait[server]

    def _goal_allowed(self, r: _Robot, server: str) -> bool:
        if server in self._resume_wait:
            return False
        if any(self._station_block.get(s.id) for s in self._stations.values()
               if s.server == server):
            return False
        now = self._clock()
        cleared = self._cleared_jobs.get(server)
        if cleared and now >= cleared[1]:
            del self._cleared_jobs[server]
            cleared = None
        return not cleared and now >= r.retry_at and not r.blocked

    def holds(self, goal_id: int) -> bool:
        """이 담기 목표를 아직 맡고 있는지. 포기하거나 잊은 목표는 보내지 않는다."""
        return self._job_owner(goal_id, FILL) is not None

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

    # ---- 담기 실패의 다음 처리 (팔 관제 합의 ①): 재시도는 NOT_READY만 ----
    def _judge_fill(self, r: _Robot, job: _Job, reason: str, uncertain: dict) -> None:
        # 쥔 채 락이 풀렸으면 uncertain이 비어 있어도 보류한다(안전장치)
        held = any(count > 0 for count in uncertain.values()) or bool(job.unlocked_holding)
        if reason == CANCELED and job.canceled_at is not None and not job.timed_out and not held:
            return  # 관제가 멈춘 목표
        then = HOLD if job.unlocked_holding else next_after_fill(reason, uncertain, job.timed_out)
        if then == RETRY:
            return  # 주문 쪽 ArmFailedCmd로 센다
        r.last, r.blocked = None, reason or then
        why = "result unknown after the timeout" if job.timed_out else reason or "no reason"
        detail = f"{job.server}: {why}"
        if then == PAUSE:
            self._out.append(ArmFillPaused(r.name, job.server, reason, detail))
            self._event("WARN", "FILL_PAUSED", f"{detail}, waits for the operator", r.name)
            return
        if held:
            detail += f", uncertain {uncertain}"
        if job.unlocked_holding:
            detail += f", {job.unlocked_holding} lock released holding an item"
        self._out.append(ArmFault(r.name, FILL, job.server, detail))
        self._event("ERROR", "FILL_FAILED", f"{detail}, held without a retry", r.name)

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
