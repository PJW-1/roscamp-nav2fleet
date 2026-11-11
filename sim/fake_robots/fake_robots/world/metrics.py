"""결과 표(지시서 6-2)의 숫자: 끝난 주문, 끝 시각, 로봇마다 기다린 시간, 로봇 사이 최소 거리,
교착(DEADLOCK_SEEN), 구역 밖(OUTSIDE_ZONE), Retreat 수, 급행 사용과 방향 바뀜."""
from __future__ import annotations

import itertools
import math

from fleet_core.common.commands import RetreatCmd
from fleet_core.fleet_map import LANE


class Metrics:
    def __init__(self, site, robots):
        self.site = site
        self.wait_s = {name: 0.0 for name in robots}
        self.min_gap = math.inf
        self.lane_uses = 0
        self.direction_changes = 0
        self.retreats = 0
        self.end_t: float | None = None
        self.event_counts: dict[str, int] = {}
        self._lanes = [z.id for z in site.zones.values() if z.kind == LANE]
        self._in_lane: dict[str, str] = {}
        self._last_direction: dict[str, str] = {}

    def on_output(self, item) -> None:
        if isinstance(item, RetreatCmd):
            self.retreats += 1

    def on_event(self, event) -> None:
        self.event_counts[event.code] = self.event_counts.get(event.code, 0) + 1

    def _waiting(self, grants, robots, views) -> list[str]:
        """구역이나 지점을 기다리는 로봇. grants.waiting()이 없으면 goal 없이 서 있는데
        갈 길(route)이 남은 정상 로봇으로 본다."""
        waiting = getattr(grants, "waiting", None)
        if waiting is not None:
            return list(waiting())
        out = []
        for name, r in robots.items():
            view = views.get(name) or {}
            if r.run is None and view.get("route") and not view.get("fault") \
                    and not view.get("comm"):
                out.append(name)
        return out

    def step(self, t, dt, grants, robots, finished: bool, views=None) -> None:
        for a, b in itertools.combinations(robots.values(), 2):
            self.min_gap = min(self.min_gap, a.gap(b))
        for name, r in robots.items():
            lane = next((z for z in self._lanes
                         if self.site.distance_to_zone(r.x, r.y, z) <= 0.0), "")
            if lane and self._in_lane.get(name) != lane:
                self.lane_uses += 1
            self._in_lane[name] = lane
        if grants is not None:
            for name in self._waiting(grants, robots, views or {}):
                if name in self.wait_s:
                    self.wait_s[name] += dt
            for zone in self._lanes:
                way = grants.lane_direction(zone)
                last = self._last_direction.get(zone)
                if way and last and way != last:
                    self.direction_changes += 1
                if way:
                    self._last_direction[zone] = way
        if finished and self.end_t is None:
            self.end_t = t

    def count(self, code: str) -> int:
        return self.event_counts.get(code, 0)
