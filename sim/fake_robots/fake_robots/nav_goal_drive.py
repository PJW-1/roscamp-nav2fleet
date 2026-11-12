"""ROS 가짜 핑키의 nav_goal 모드(MoveTo mode, 계약 5절). ROS를 쓰지 않는다.

world.NavGoalRun과 같은 규칙을 Motion 위에서 쓴다: 넓은 구역 안에서 다른 로봇을 피해 길을
짜고(1 Hz로 다시), 바로 앞에 로봇이 있으면 서고, blocked_after_s 동안 못 나아가면 BLOCKED,
지도 위에 길이 아예 없으면 곧 NAV_ABORTED, GIVE_UP_S 동안 못 나아가도 NAV_ABORTED.
노드는 매 틱 update()를 부르고, clear_ahead()가 참일 때만 몬다.
"""
from __future__ import annotations

import math

from .ahead import nothing_ahead
from .world.nav_goal import ABORT_AFTER_S, GIVE_UP_S, PROGRESS_M, REPLAN_S, plan_around
from .world.planner import plan

BLOCKED_AFTER_S = 5.0  # 계약 5절: nav_goal 모드는 이만큼 못 나아가면 BLOCKED


class NavGoalDrive:
    def __init__(self, space, radius, target, t, here, blocked_after_s=BLOCKED_AFTER_S):
        self.space, self.radius = space, radius
        self.target = tuple(target)
        self.blocked_after_s = blocked_after_s
        self.mark, self.mark_t = tuple(here), t
        self.planned_at = -math.inf
        self.has_way = False
        self.no_way_at = t if plan(space, space.static, here, self.target) is None else None

    def update(self, t, motion, others) -> str | None:
        """'NAV_ABORTED'면 목표를 끝낸다. 길이 없거나 서 있으면 REPLAN_S마다 다시 짠다."""
        if self.no_way_at is not None:
            return "NAV_ABORTED" if t - self.no_way_at >= ABORT_AFTER_S - 1e-9 else None
        here = (motion.x, motion.y)
        if math.dist(here, self.mark) >= PROGRESS_M:
            self.mark, self.mark_t = here, t
        if t - self.mark_t >= GIVE_UP_S - 1e-9:
            return "NAV_ABORTED"
        stalled = not self.has_way or t - self.mark_t >= REPLAN_S - 1e-9
        if stalled and t - self.planned_at >= REPLAN_S - 1e-9:
            self.planned_at = t
            way = plan_around(self.space, self.radius, here, self.target, others)
            self.has_way = way is not None
            if self.has_way:
                motion.set_target(*self.target, via=way[1:-1])
            else:
                motion.clear_target()
        return None

    def clear_ahead(self, motion, step, others) -> bool:
        """바로 앞에 몸이 닿을 로봇이 없다(충돌 검사를 켠 Nav2)."""
        return nothing_ahead(motion, self.radius, step, others)

    def blocked(self, t) -> bool:
        return t - self.mark_t >= self.blocked_after_s - 1e-9

    def arrived(self, motion) -> bool:
        return self.has_way and not motion.moving
