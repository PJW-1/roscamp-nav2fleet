# ---- 도킹 실패 후 다시 접근 믹스인 ----
from __future__ import annotations

from fleet_core.fleet_map import FOLLOW_PATH, RETREAT

from .routes import Route


# ---- 도크로 들어가는 FOLLOW_PATH 경로 찾기 ----
def way_in(site, start: str, dock: str) -> list | None:
    direct = site.edge(start, dock)
    if direct is not None:
        return [direct] if direct.mode == FOLLOW_PATH else None
    for first in site.edges_from(start):
        second = site.edge(first.b, dock)
        if first.mode == FOLLOW_PATH and second is not None and second.mode == FOLLOW_PATH:
            return [first, second]
    return None


class Redock:
    # ---- 도킹 실패 횟수 기록 ----
    def _dock_failed(self, m, reason: str = "DOCK_FAILED") -> None:
        m.dock_tries += 1
        m.attempts = m.dock_tries - 1
        if reason == "DOCK_FAILED":
            m.watch.redock.key, m.watch.redock.since = True, None
        self._goal_failed(m, reason)

    # ---- 물러났다가 도크로 다시 들어오는 route 만들기 ----
    def _reapproach(self, m) -> bool:
        now = self._clock()
        dock = self.site.stations[m.dest].dock
        out = next((edge for edge in self.site.edges_from(dock) if edge.mode == RETREAT), None)
        back = way_in(self.site, out.b, dock) if out is not None else None
        ep = m.watch.redock
        if ep.since is None:
            ep.since = now
        edges = [out, *back] if back is not None else []
        marked = any(self._blocked.get((edge.a, edge.b), 0.0) > now for edge in edges)
        if (not edges or marked or m.node != dock
                or now - ep.since >= self.params.redock_wait_s):
            ep.clear()
            self._stop_wanting(m)
            return False
        route = Route(dock, edges)
        want = self._take(m, route)
        if want is not None:
            self._wants[m.name] = want
            self._note_wait(m, want)
            return True
        ep.clear()
        m.route, m.phase = route, self._leg_of(m.dest)
        self._event("INFO", "DOCK_REAPPROACH", f"backs to {out.b} and comes in to {dock} again",
                    m.name)
        return True
