# ---- 목표를 받고도 출발하지 않는 로봇 감시 믹스인 ----
from __future__ import annotations

import math

from fleet_core.traffic.mission import BLOCKED, DOCK, RUN, SELF_RECOVERING

START_MOVE_M = 0.02


class NoStart:
    # ---- 목표별 출발 위치 기록과 제한 시간 내 이동 여부 판정 ----
    def _watch_start(self, m, now: float) -> None:
        p, goal = self.params, m.goal
        waiting = m.move_order is not None and m.move_order.waiting
        if (p.start_limit_s <= 0 or not m.available or self._run_mode != RUN or goal is None
                or goal.kind == DOCK or m.arrival_by is not None or waiting):
            m.start_goal = 0
            return
        here = (m.x, m.y)
        if m.start_goal != goal.id:
            m.start_goal, m.start_xy, m.start_since = goal.id, here, now
            m.start_done = (goal.end in self.site.nodes
                            and self._gap(m, goal.end) <= START_MOVE_M)
            return
        if m.start_done:
            return
        if m.robot_state in (BLOCKED, SELF_RECOVERING) or math.dist(m.start_xy, here) > (
                START_MOVE_M):
            m.start_done = True
        elif now - m.start_since >= p.start_limit_s:
            m.start_done = True
            self._event("WARN", "NO_START", f"{goal.kind} goal {goal.id} to {goal.end}: moved "
                        f"less than {START_MOVE_M} m in {p.start_limit_s:.0f} s", m.name)
