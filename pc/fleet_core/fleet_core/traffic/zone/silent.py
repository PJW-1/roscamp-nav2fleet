# ---- 조용히 멈춘 로봇과 할 일 없는 로봇 감시 ----
from __future__ import annotations

import math

from fleet_core.traffic.mission import AT_STATION, DOCKING, TO_OUTBOUND, TO_PARK, TO_WORK

SILENT_STALL = "SILENT_STALL"
GOING = (TO_WORK, TO_OUTBOUND, TO_PARK)


class SilentWatch:
    def __init__(self, params, robots, emit, emit_once):
        self.params = params
        self._robots = robots
        self.emit = emit
        self.emit_once = emit_once

    # ---- 이벤트가 나면 무진척 기록 초기화 ----
    def note(self, robot: str, code: str) -> None:
        m = self._robots.get(robot)
        if m is not None and code != SILENT_STALL:
            m.watch.silent.clear()

    # ---- 관제가 기다리는 것이 없는 상태인지 판정 ----
    @staticmethod
    def _unattended(m, now: float, running: bool) -> bool:
        return (m.available and running and m.phase != AT_STATION
                and (m.dispatch is not None or m.phase in GOING or m.phase == DOCKING)
                and m.watch.wait.since is None and now >= m.retry_at and now >= m.avoid.yield_until)

    # ---- 목표 없이 이동 단계에 머문 로봇 경고 ----
    def watch_idle(self, m, now: float, running: bool) -> None:
        p, ep = self.params, m.watch.idle
        idle = (p.idle_watch_s > 0 and m.goal is None and m.phase in GOING
                and self._unattended(m, now, running))
        if not idle:
            ep.clear()
        elif ep.since is None:
            ep.since = now
        elif now - ep.since >= p.idle_watch_s:
            ep.since = now
            self.emit("WARN", "ROBOT_IDLE", f"{m.phase} at {m.node}: no goal and nothing to "
                      "wait for", m.name)

    # ---- 진척도 이벤트도 없는 로봇 경고 ----
    def watch_silent(self, m, now: float, running: bool) -> None:
        p, ep = self.params, m.watch.silent
        if p.silent_limit_s <= 0 or not self._unattended(m, now, running):
            ep.clear()
            return
        here = (m.x, m.y)
        if ep.xy is None or math.dist(ep.xy, here) >= p.stall_move_m:
            ep.xy, ep.since, ep.noted = here, now, False
            return
        if now - ep.since >= p.silent_limit_s:
            self.emit_once(ep, "WARN", SILENT_STALL, f"{m.phase} at {m.node}: no progress "
                           f"and no fleet event for {p.silent_limit_s:.0f} s", m.name)
