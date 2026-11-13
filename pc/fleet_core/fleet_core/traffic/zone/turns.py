from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK
from fleet_core.traffic.mission import TO_OUTBOUND, TO_WORK


class Turns:
    def _candidates(self, m):
        site = self.site
        if m.phase in (TO_WORK, TO_OUTBOUND):
            station = site.stations[m.dest]
            if self._is_next(m, station.id) and self._dock_free(m, station.dock):
                yield station.dock, False
                if self._may_stay(m):
                    return
            elif m.node in station.queue and self._on_node(m):
                self._stay(m, station)
                return
            for spot in station.queue:
                if self.grants.node_holder(spot) in ("", m.name):
                    yield spot, True
            if self._may_stay(m):
                self._stay(m, station)
                return
        cost = self._cost(m)
        free = [spot for spot in site.parking if self.grants.node_holder(spot) in ("", m.name)]

        def price(spot) -> float:
            links = self._way(m, spot)
            return math.inf if links is None else sum(cost(link) for link in links)
        for spot in sorted(free, key=lambda spot: (price(spot), spot)):
            yield spot, False

    def _stay(self, m, station) -> None:
        if m.wait_since is None:
            m.wait_since = self._clock()
        m.wait_for = (self.site.nodes[station.dock].zone,)
        if not m.queued:
            m.queued = True
            self._event("INFO", "DOCK_QUEUED", f"{station.id} is busy, waits at {m.node}", m.name)

    def _may_stay(self, m) -> bool:
        return (self._on_node(m) and self.site.can_wait(m.node)
                and self.site.nodes[m.node].kind != AT_DOCK and not m.retreat_to)

    def _is_next(self, m, station_id: str) -> bool:
        return all(self._waits_for(other, m) for other in self._ahead(m, station_id))

    def _ahead(self, m, station_id: str) -> list:
        dock = self.site.stations[station_id].dock

        def rank(r) -> tuple:
            trip = r.trip
            left = trip is not None and trip.turn and not trip.stale and trip.goal == dock
            return (not (left or r.dock_tries), r.asked_at or 0.0, r.asked_gap, r.name)

        mine = rank(m)
        return [other for other in self._robots.values()
                if (other is not m and other.available and other.dest == station_id
                    and other.phase in (TO_WORK, TO_OUTBOUND) and rank(other) < mine)]

    def _waits_for(self, other, m) -> bool:
        return (other.goal is None and m.goal is None and self._on_node(m)
                and m.node in other.wait_for and self.grants.node_holder(m.node) == m.name)

    def _take_turn(self, m, trip) -> None:
        if m.phase not in (TO_WORK, TO_OUTBOUND) or trip.goal != self.site.stations[m.dest].dock:
            return
        for other in self._ahead(m, m.dest):
            self._event("INFO", "DOCK_TURN_TAKEN",
                        f"goes before {other.name}, which waits for {m.node}", m.name)
        trip.turn = True

    def _dock_free(self, m, dock: str) -> bool:
        zone = self.site.nodes[dock].zone
        return (all(name == m.name for name in self.grants.holders(zone))
                and self.grants.node_holder(dock) in ("", m.name))
