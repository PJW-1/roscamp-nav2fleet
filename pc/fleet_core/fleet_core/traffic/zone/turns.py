"""어디로 갈지와 도크 차례(지시서 2-2의 5번).

- 도크가 비어 있고 먼저 온 차례이면 도크로, 아니면 그 스테이션의 대기 자리로, 그것도 없고
  지금 자리에 서 있으면 안 되면 주차 자리로 간다.
- 차례는 그 스테이션을 먼저 바란 순서다. 이미 도크로 가는 길을 잡고 떠난 로봇이 앞이고, 먼저 온
  로봇이 내가 선 지점이 비기를 기다리며 서 있으면 내가 먼저 간다.
"""
from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK
from fleet_core.traffic.mission import TO_OUTBOUND, TO_WORK


class Turns:
    def _candidates(self, m):
        """(목표 지점, 대기 자리인가)를 좋은 순서로."""
        site = self.site
        if m.phase in (TO_WORK, TO_OUTBOUND):
            station = site.stations[m.dest]
            if self._is_next(m, station.id) and self._dock_free(m, station.dock):
                yield station.dock, False
                if self._may_stay(m):
                    return  # 도크는 비었고 가는 길만 못 잡았다. 여기서 기다린다
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
        """도크가 찼거나 차례가 아니어서 지금 자리(대기 자리나 주차)에서 기다린다."""
        if m.wait_since is None:
            m.wait_since = self._clock()
        m.wait_for = (self.site.nodes[station.dock].zone,)
        if not m.queued:
            m.queued = True
            self._event("INFO", "DOCK_QUEUED", f"{station.id} is busy, waits at {m.node}", m.name)

    def _may_stay(self, m) -> bool:
        """지금 자리에 서서 기다려도 되는가: 대기 자리나 주차에 서 있을 때만이다."""
        return (self._on_node(m) and self.site.can_wait(m.node)
                and self.site.nodes[m.node].kind != AT_DOCK and not m.retreat_to)

    def _is_next(self, m, station_id: str) -> bool:
        """그 스테이션을 바라는 로봇 가운데 먼저 온 차례인가. 같은 때 왔으면 가까운 쪽이다.

        먼저 온 로봇이 내가 선 지점이 비기를 기다리며 서 있으면 세지 않는다: 내가 떠나야 그 로봇이
        움직이므로 내가 먼저 간다.
        """
        return all(self._waits_for(other, m) for other in self._ahead(m, station_id))

    def _ahead(self, m, station_id: str) -> list:
        """그 스테이션을 나보다 먼저 바란 로봇들과, 이미 그 도크로 가는 길을 잡고 떠난 로봇들."""
        mine = (m.asked_at or 0.0, m.asked_gap, m.name)
        dock = self.site.stations[station_id].dock
        return [other for other in self._robots.values()
                if (other is not m and other.available and other.dest == station_id
                    and other.phase in (TO_WORK, TO_OUTBOUND)
                    and ((other.asked_at or 0.0, other.asked_gap, other.name) < mine
                         or (other.trip is not None and other.trip.goal == dock)))]

    def _waits_for(self, other, m) -> bool:
        """other가 m이 서서 쥔 지점이 비기를 기다리며 서 있는가."""
        return (other.goal is None and m.goal is None and self._on_node(m)
                and m.node in other.wait_for and self.grants.node_holder(m.node) == m.name)

    def _note_turn(self, m, goal: str) -> None:
        """떠나는 곳이 도크이고 먼저 온 로봇을 앞질렀으면 알린다."""
        if m.phase not in (TO_WORK, TO_OUTBOUND) or goal != self.site.stations[m.dest].dock:
            return
        for other in self._ahead(m, m.dest):
            self._event("INFO", "DOCK_TURN_TAKEN",
                        f"goes before {other.name}, which waits for {m.node}", m.name)

    def _dock_free(self, m, dock: str) -> bool:
        zone = self.site.nodes[dock].zone
        return (all(name == m.name for name in self.grants.holders(zone))
                and self.grants.node_holder(dock) in ("", m.name))
