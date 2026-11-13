# ---- 조용히 멈춘 로봇과 할 일 없는 로봇 감시 믹스인 ----
from __future__ import annotations

import math

from fleet_core.traffic.mission import AT_STATION, DOCKING, RUN, TO_OUTBOUND, TO_PARK, TO_WORK

SILENT_STALL = "SILENT_STALL"
GOING = (TO_WORK, TO_OUTBOUND, TO_PARK)


class Silent:
    # ---- 이벤트가 나면 무진척 기록 초기화 ----
    def _event(self, level: str, code: str, detail: str, robot: str = "") -> None:
        super()._event(level, code, detail, robot)
        m = self._robots.get(robot)
        if m is not None and code != SILENT_STALL:
            m.silent_from = None

    # ---- 관제가 기다리는 것이 없는 상태인지 판정 ----
    def _unattended(self, m, now: float) -> bool:
        return (m.available and self._run_mode == RUN and m.phase != AT_STATION
                and (m.dispatch is not None or m.phase in GOING or m.phase == DOCKING)
                and m.wait_since is None and now >= m.retry_at and now >= m.yield_until)

    # ---- 목표 없이 이동 단계에 머문 로봇 경고 ----
    def _watch_idle(self, m, now: float) -> None:
        p = self.params
        idle = (p.idle_watch_s > 0 and m.goal is None and m.phase in GOING
                and self._unattended(m, now))
        if not idle:
            m.idle_since = None
        elif m.idle_since is None:
            m.idle_since = now
        elif now - m.idle_since >= p.idle_watch_s:
            m.idle_since = now
            self._event("WARN", "ROBOT_IDLE", f"{m.phase} at {m.node}: no goal and nothing to "
                        "wait for", m.name)

    # ---- 진척도 이벤트도 없는 로봇 경고 ----
    def _watch_silent(self, m, now: float) -> None:
        p = self.params
        if p.silent_limit_s <= 0 or not self._unattended(m, now):
            m.silent_from = None
            return
        here = (m.x, m.y)
        if m.silent_from is None or math.dist(m.silent_from, here) >= p.stall_move_m:
            m.silent_from, m.silent_since, m.silent_noted = here, now, False
            return
        if not m.silent_noted and now - m.silent_since >= p.silent_limit_s:
            m.silent_noted = True
            self._event("WARN", SILENT_STALL, f"{m.phase} at {m.node}: no progress and no fleet "
                        f"event for {p.silent_limit_s:.0f} s", m.name)
