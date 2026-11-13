# ---- 도킹 실패 후 다시 접근 믹스인 ----
from __future__ import annotations

from fleet_core.fleet_map import FOLLOW_PATH, RETREAT

from .trips import Trip


# ---- 도크로 들어가는 FOLLOW_PATH 경로 찾기 ----
def way_in(site, start: str, dock: str) -> list | None:
    direct = site.link(start, dock)
    if direct is not None:
        return [direct] if direct.via == FOLLOW_PATH else None
    for first in site.links_from(start):
        second = site.link(first.b, dock)
        if first.via == FOLLOW_PATH and second is not None and second.via == FOLLOW_PATH:
            return [first, second]
    return None


class Redock:
    # ---- 도킹 실패 횟수 기록 ----
    def _dock_failed(self, m, reason: str = "DOCK_FAILED") -> None:
        m.dock_tries += 1
        m.attempts = m.dock_tries - 1
        if reason == "DOCK_FAILED":
            m.redock, m.redock_since = True, None
        self._goal_failed(m, reason)

    # ---- 물러났다가 도크로 다시 들어오는 트립 만들기 ----
    def _reapproach(self, m) -> bool:
        now = self._clock()
        dock = self.site.stations[m.dest].dock
        out = next((link for link in self.site.links_from(dock) if link.via == RETREAT), None)
        back = way_in(self.site, out.b, dock) if out is not None else None
        if m.redock_since is None:
            m.redock_since = now
        links = [out, *back] if back is not None else []
        marked = any(self._blocked.get((link.a, link.b), 0.0) > now for link in links)
        if (not links or marked or m.node != dock
                or now - m.redock_since >= self.params.redock_wait_s):
            m.redock = False
            self._stop_wanting(m)
            return False
        trip = Trip(dock, links)
        want = self._take(m, trip)
        if want is not None:
            self._wants[m.name] = want
            self._note_wait(m, want)
            return True
        m.redock = False
        m.trip, m.phase = trip, self._leg_of(m.dest)
        self._event("INFO", "DOCK_REAPPROACH", f"backs to {out.b} and comes in to {dock} again",
                    m.name)
        return True
