"""말 없는 로봇 감시: 할 일 없이 섬(ROBOT_IDLE)과 조용히 멈춤(SILENT_STALL, 추가 지시서 2의 3절).

갈 곳이 있는 로봇: 배차를 쥐었거나(스테이션에서 일하는 중은 빼고) 단계가 TO_WORK, TO_OUTBOUND,
TO_PARK, DOCKING이다. 이 로봇이 관제가 알고 세워 둔 것이 아닌데 보고 위치가 stall_move_m 넘게
나아가지 않고, 관제도 이 로봇에 대한 사건을 하나도 내지 않은 채 silent_limit_s가 지나면
SILENT_STALL을 한 번 낸다. 움직이거나, 이 로봇에 대한 사건이 나오거나, 아래의 "아는 멈춤"이 되면
다시 잰다. 관제가 그 로봇을 두고 아무 말이 없다는 것은 관제의 결함이다.

관제가 알고 세워 둔 것(시계를 다시 잰다):

| 경우 | 왜 |
|---|---|
| 지도에 없음, 고장(ERROR, OUT_OF_SERVICE), 통신 끊김과 복구 | 다른 감시와 운영자가 본다 |
| 관제를 세움(STOP) | 운영자가 세웠다 |
| 주차나 IDLE이고 배차가 없음 | 할 일이 없다 |
| 스테이션에서 일하는 중(AT_STATION) | 팔 쪽 시간 상한이 본다 |
| 찬 도크나 구역을 기다림(wait_since) | DOCK_QUEUED, ZONE_WAIT로 말했다. 오래되면 ZONE_WAIT 경고 |
| 다시 내기 전 기다림(retry_at), 비켜 섬(yield_until) | 몇 초 뒤 관제가 스스로 다시 움직인다 |

BLOCKED나 RECOVERING을 보고하는 로봇은 빼지 않는다: 막힘 처리(blocked.py)가 정한 시간 안에
말해야 한다. ROBOT_IDLE(goal도 기다림도 없이 섬)도 이 로봇에 대한 사건이므로 그 감시가 켜져
있으면 goal 없이 선 로봇은 여기 걸리지 않는다. 둘 다 같은 "갈 곳이 있고 아는 멈춤이 아니다"를 쓴다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.mission import AT_STATION, DOCKING, RUN, TO_OUTBOUND, TO_PARK, TO_WORK

SILENT_STALL = "SILENT_STALL"
GOING = (TO_WORK, TO_OUTBOUND, TO_PARK)


class Silent:
    def _event(self, level: str, code: str, detail: str, robot: str = "") -> None:
        super()._event(level, code, detail, robot)
        m = self._robots.get(robot)
        if m is not None and code != SILENT_STALL:
            m.silent_from = None  # 관제가 이 로봇에 대해 말했다: 다시 잰다

    def _unattended(self, m, now: float) -> bool:
        """갈 곳이 있는데 관제가 알고 세워 둔 것이 아니다."""
        return (m.available and self._run_mode == RUN and m.phase != AT_STATION
                and (m.dispatch is not None or m.phase in GOING or m.phase == DOCKING)
                and m.wait_since is None and now >= m.retry_at and now >= m.yield_until)

    def _watch_idle(self, m, now: float) -> None:
        """갈 곳이 있는데 goal도 없고 기다리는 것도 없이 서 있으면 알린다."""
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
