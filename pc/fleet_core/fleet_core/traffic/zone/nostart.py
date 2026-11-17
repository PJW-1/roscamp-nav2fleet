# ---- 목표를 받고도 출발하지 않는 로봇 감시 믹스인 ----
from __future__ import annotations

import math

from fleet_core.traffic.mission import BLOCKED, DOCK, RUN, SELF_RECOVERING

START_MOVE_M = 0.02


class NoStart:
    # ---- 목표별 출발 위치 기록과 제한 시간 내 이동 여부 판정 ----
    def _watch_start(self, m, now: float) -> None:
        p, goal, ep = self.params, m.goal, m.start
        waiting = m.move_order is not None and m.move_order.waiting
        if (p.start_limit_s <= 0 or not m.available or self._run_mode != RUN or goal is None
                or goal.kind == DOCK or m.arrival_by is not None or waiting):
            ep.clear()
            return
        here = (m.x, m.y)
        if ep.key != goal.id:
            ep.key, ep.xy, ep.since = goal.id, here, now
            ep.noted = (goal.end in self.site.nodes
                        and self._gap(m, goal.end) <= START_MOVE_M)
            return
        if ep.noted:
            return
        if m.robot_state in (BLOCKED, SELF_RECOVERING) or math.dist(ep.xy, here) > (
                START_MOVE_M):
            ep.noted = True
        elif now - ep.since >= p.start_limit_s:
            ep.noted = True
            self._event("WARN", "NO_START", f"{goal.kind} goal {goal.id} to {goal.end}: moved "
                        f"less than {START_MOVE_M} m in {p.start_limit_s:.0f} s", m.name)
