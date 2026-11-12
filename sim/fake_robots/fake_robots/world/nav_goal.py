"""Nav2 goal(NavigateToPose) 흉내 (지시서 6-1, 계약 5절).

로봇이 넓은 구역 안에서 스스로 길을 짠다(planner.py). 충돌 검사를 켠 Nav2처럼 서 있는 로봇이나
물체는 자리가 있으면 돌아가고, 바로 앞에 무엇이 있으면 선다. blocked_after_s 동안 못 나아가면
BLOCKED를 보고하고, 길이 아예 없으면 NAV_ABORTED로 끝난다. 끝에서 목표 방향으로 돈다.
Nav2의 기본 복구(Spin, BackUp)는 몸으로 흉내 내지 않는다(서서 다시 계획만 한다).
돌아가는 정도는 회피 흉내 모드(avoid.py: none, perfect, mixed)가 정한다.
"""
from __future__ import annotations

import math

from .avoid import Avoidance
from .geom import heading
from .planner import plan
from .runs import LOOK_M, Run, Turning, blocked_ahead, end_offset, jittered, move_along

REPLAN_S = 1.0  # 막혔을 때 다시 계획하는 주기(NavigateToPose 기본 BT 1 Hz)
PLAN_GAP_M = 0.03  # 계획할 때 다른 로봇과 물체를 이만큼 더 키운다(격자 반 칸 + 충돌 검사 틈)
PROGRESS_M = 0.02  # 이만큼 움직이면 나아간 것으로 본다
# 이만큼 못 나아가면 BLOCKED. 핑키의 blocked_after_s(계약 0.13-draft)와 같아야 한다
# (fleet_gz_sim의 test_timing_agreement.py가 본다)
BLOCKED_AFTER_S = 10.0
ABORT_AFTER_S = 0.5  # 지도 위에 길이 없을 때 이만큼 뒤 NAV_ABORTED
GIVE_UP_S = 60.0  # 이만큼 못 나아가면 NAV_ABORTED(Nav2 복구 여섯 번쯤)


def plan_around(space, radius, start, target, others):
    """다른 로봇과 물체(others: (x, y, 반지름))를 피한 길, 없으면 None.

    목표 위의 것은 빼고 계획한다(목표 위에 선 로봇은 비킬 것으로 본다). 빼도 충돌 검사가
    바로 앞에서 세운다. 발밑까지 닿는 것은 로봇 바로 앞까지만 키운다(출발 칸을 비운다).
    """
    discs = [(x, y, radius + r + PLAN_GAP_M) for x, y, r in others]
    discs = [(x, y, min(reach, math.dist((x, y), start) - 0.5 * space.res))
             for x, y, reach in discs if math.dist((x, y), target) >= reach]
    return plan(space, space.with_discs(discs), start, target)


class NavGoalRun(Run):
    def __init__(self, goal_id, target, yaw, speed, blocked_after_s, avoid=None):
        super().__init__(goal_id)
        self.avoid = avoid or Avoidance()
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
        static = plan(ctx.space, ctx.space.static, robot.pose, self.target)
        if static is None:
            self.no_way_at = ctx.t
            return
        if self.avoid.plans_around:
            self._replan(ctx, robot)
        else:  # none, mixed: 정적 지도 위 길로 떠난다
            self.way = self.avoid.stray(ctx.rng, static)

    def _replan(self, ctx, robot) -> None:
        self.planned_at = ctx.t
        self.way = plan_around(ctx.space, ctx.radius, robot.pose, self.target, ctx.discs(robot))

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
                self._stand(ctx, robot)
                return None
            robot.yaw = heading(self.way[0], self.way[1], robot.yaw)
            self.way, moved = move_along(ctx, robot, self.way, step)
            if moved:
                self.avoid.moved()
            self._progress(ctx, robot)
            return None
        if self.way is None:  # 지금은 길이 없다(다른 로봇, 물체): 서서 다시 계획한다
            self._stand(ctx, robot)
            return None
        self.mark_t, self.blocked = ctx.t, False  # 끝에서 도는 것도 나아가는 것이다
        if Turning.toward(ctx, robot, self.yaw):
            return self.done(True)
        return None

    def _stand(self, ctx, robot) -> None:
        """막혀 섰다. 회피 모드가 허락하면 REPLAN_S마다 돌아가는 길을 다시 짠다."""
        if (self.avoid.may_detour(ctx.rng, ctx.t)
                and ctx.t - self.planned_at >= REPLAN_S - 1e-9):
            self._replan(ctx, robot)
        self._progress(ctx, robot)
