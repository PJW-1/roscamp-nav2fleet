"""Nav2 goal(NavigateToPose) 흉내 (지시서 6-1, 계약 5절).

로봇이 넓은 구역 안에서 스스로 길을 짠다(planner.py). 충돌 검사를 켠 Nav2처럼 서 있는 로봇이나
물체는 자리가 있으면 돌아가고, 바로 앞에 무엇이 있으면 선다. blocked_after_s 동안 못 나아가면
BLOCKED를 보고하고, 길이 아예 없으면 NAV_ABORTED로 끝난다. 끝에서 목표 방향으로 돈다.
Nav2의 기본 복구(Spin, BackUp)는 몸으로 흉내 내지 않는다(서서 다시 계획만 한다).
"""
from __future__ import annotations

import math

from .geom import heading
from .planner import plan
from .runs import LOOK_M, Run, Turning, blocked_ahead, end_offset, jittered, move_along

REPLAN_S = 1.0  # 막혔을 때 다시 계획하는 주기(NavigateToPose 기본 BT 1 Hz)
PLAN_GAP_M = 0.03  # 계획할 때 다른 로봇과 물체를 이만큼 더 키운다(격자 반 칸 + 충돌 검사 틈)
PROGRESS_M = 0.02  # 이만큼 움직이면 나아간 것으로 본다
ABORT_AFTER_S = 0.5  # 지도 위에 길이 없을 때 이만큼 뒤 NAV_ABORTED
GIVE_UP_S = 60.0  # 이만큼 못 나아가면 NAV_ABORTED(Nav2 복구 여섯 번쯤)


class NavGoalRun(Run):
    def __init__(self, goal_id, target, yaw, speed, blocked_after_s):
        super().__init__(goal_id)
        self.target, self.yaw, self.speed = tuple(target), yaw, speed
        self.blocked_after_s = blocked_after_s
        self.way: list | None = None
        self.no_way_at: float | None = None  # 정적 지도에 길이 없다고 본 시각
        self.planned_at = -math.inf
        self.mark = None
        self.mark_t = 0.0

    def _start(self, ctx, robot) -> None:
        self.started = True
        self.target = end_offset(ctx.rng, ctx.imperfect.arrival_error_m, self.target)
        self.mark, self.mark_t = robot.pose, ctx.t
        if plan(ctx.space, ctx.space.static, robot.pose, self.target) is None:
            self.no_way_at = ctx.t
            return
        self._replan(ctx, robot)

    def _replan(self, ctx, robot) -> None:
        self.planned_at = ctx.t
        discs = [(x, y, ctx.radius + r + PLAN_GAP_M) for x, y, r in ctx.discs(robot)]
        # 목표 위의 것은 빼고 계획한다(목표 위에 선 로봇은 비킬 것으로 본다). 빼도 충돌 검사가
        # 바로 앞에서 세운다. 발밑까지 닿는 것은 로봇 바로 앞까지만 키운다(출발 칸을 비운다)
        discs = [(x, y, min(reach, math.dist((x, y), robot.pose) - 0.5 * ctx.space.res))
                 for x, y, reach in discs if math.dist((x, y), self.target) >= reach]
        cells = ctx.space.with_discs(discs)
        self.way = plan(ctx.space, cells, robot.pose, self.target)

    def _progress(self, ctx, robot) -> None:
        if math.dist(self.mark, robot.pose) >= PROGRESS_M:
            self.mark, self.mark_t, self.blocked = robot.pose, ctx.t, False
        elif ctx.t - self.mark_t >= self.blocked_after_s - 1e-9:
            self.blocked = True

    def step(self, ctx, robot):
        if not self.started:
            self._start(ctx, robot)
        if self.no_way_at is not None:
            return (self.done(False, "NAV_ABORTED")
                    if ctx.t - self.no_way_at >= ABORT_AFTER_S - 1e-9 else None)
        result = self._drive(ctx, robot)
        if result is None and ctx.t - self.mark_t >= GIVE_UP_S:
            return self.done(False, "NAV_ABORTED")
        return result

    def _drive(self, ctx, robot):
        if self.way is not None and len(self.way) > 1:
            step = jittered(ctx, self.speed) * ctx.dt
            if blocked_ahead(ctx, robot, self.way, LOOK_M + step):
                if ctx.t - self.planned_at >= REPLAN_S - 1e-9:
                    self._replan(ctx, robot)
                self._progress(ctx, robot)
                return None
            robot.yaw = heading(self.way[0], self.way[1], robot.yaw)
            self.way, _ = move_along(ctx, robot, self.way, step)
            self._progress(ctx, robot)
            return None
        if self.way is None:  # 지금은 길이 없다(다른 로봇, 물체): 서서 다시 계획한다
            if ctx.t - self.planned_at >= REPLAN_S - 1e-9:
                self._replan(ctx, robot)
            self._progress(ctx, robot)
            return None
        self.mark_t, self.blocked = ctx.t, False  # 끝에서 도는 것도 나아가는 것이다
        if Turning.toward(ctx, robot, self.yaw):
            return self.done(True)
        return None
