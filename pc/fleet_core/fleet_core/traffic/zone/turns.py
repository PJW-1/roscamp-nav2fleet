# ---- 도크 대기 줄과 차례, 갈 곳 후보 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK
from fleet_core.traffic.mission import TO_OUTBOUND, TO_WORK


# ---- 스테이션 도크, 대기 지점, 주차 지점 후보 내기 ----
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
                step = self._line_step(m, station)
                if step is None:
                    self._stay(m, station)
                    return
                yield step, True
                return
            for spot in self._open_spots(m, station):
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

    # ---- 줄(line) 스테이션: 빈 앞자리로 한 칸 당기기, 새 로봇은 맨 뒤 로봇 바로 뒤로 ----
    def _line_step(self, m, station):
        if not station.line:
            return None
        at = station.queue.index(m.node)
        if at == 0 or self.grants.node_holder(station.queue[at - 1]) not in ("", m.name):
            return None
        return station.queue[at - 1]

    def _open_spots(self, m, station) -> list:
        mine = ("", m.name)
        if not station.line:
            return [spot for spot in station.queue if self.grants.node_holder(spot) in mine]
        taken = [i for i, spot in enumerate(station.queue)
                 if self.grants.node_holder(spot) not in mine]
        return list(station.queue[max(taken) + 1 if taken else 0:])

    # ---- 대기 지점에서 기다리기와 머물 수 있는지 판단 ----
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

    # ---- 같은 스테이션 앞선 로봇 순위와 다음 차례 판정 ----
    def _is_next(self, m, station_id: str) -> bool:
        return all(self._waits_for(other, m) for other in self._ahead(m, station_id))

    def _ahead(self, m, station_id: str) -> list:
        station = self.site.stations[station_id]
        dock = station.dock

        def spot(r) -> int:
            # 줄 스테이션에서는 선 자리 순서(앞이 0), 줄에 안 선 로봇은 그 뒤
            if station.line and r.node in station.queue:
                return station.queue.index(r.node)
            return len(station.queue) if station.line else 0

        def rank(r) -> tuple:
            trip = r.trip
            left = trip is not None and trip.turn and not trip.stale and trip.goal == dock
            return (not (left or r.dock_tries), spot(r), r.asked_at or 0.0, r.asked_gap, r.name)

        mine = rank(m)
        return [other for other in self._robots.values()
                if (other is not m and other.available and other.dest == station_id
                    and other.phase in (TO_WORK, TO_OUTBOUND) and rank(other) < mine)]

    def _waits_for(self, other, m) -> bool:
        return (other.goal is None and m.goal is None and self._on_node(m)
                and m.node in other.wait_for and self.grants.node_holder(m.node) == m.name)

    # ---- 도크 차례 가져가기와 도크 빈 여부 확인 ----
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
