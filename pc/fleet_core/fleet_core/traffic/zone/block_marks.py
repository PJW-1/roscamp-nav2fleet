from __future__ import annotations

import math
from dataclasses import dataclass, field

from fleet_core.fleet_map import NAV_GOAL, SHARED
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import OUT_OF_SERVICE

from .base import HERE


@dataclass
class Mark:
    id: int
    x: float
    y: float
    by: str
    since: float
    until: float
    retry_s: float
    keys: list = field(default_factory=list)
    clears: int = 0
    asked_at: float = 0.0
    check: int = 0
    asked: str = ""
    asked_last: dict = field(default_factory=dict)


def polyline_gap(point, points) -> float:
    if len(points) == 1:
        return math.dist(point, points[0])
    return min(point_segment_distance(point, a, b) for a, b in zip(points, points[1:],
                                                                    strict=False))


def keys_at(site, params, point) -> list:
    reach = params.footprint_radius_m + params.release_margin_m
    zone = site.zone_at(*point)
    keys = []
    for link in site.links():
        if link.via != NAV_GOAL:
            hit = polyline_gap(point, link.points) <= reach
        elif zone is None or zone not in link.zones:
            hit = False
        elif not site.nodes_in(zone):
            hit = True
        else:
            hit = point_segment_distance(point, link.points[0], link.points[-1]) <= reach
        if hit:
            keys.append((link.a, link.b))
    return keys


class BlockMarks:
    def _obstacle_point(self, m) -> tuple | None:
        if not self.params.blocked_by_position or m.obstacle is None:
            return None
        return m.obstacle[:2]

    def _robot_at(self, m, point):
        near = [other for other in self._robots.values()
                if other is not m and other.placed and other.fault != OUT_OF_SERVICE
                and math.dist(point, (other.x, other.y)) <= self.params.near_m]
        return min(near, key=lambda o: (math.dist(point, (o.x, o.y)), o.name), default=None)

    def _point_keys(self, m, point, link) -> list:
        keys = keys_at(self.site, self.params, point)
        if not keys:
            return keys
        if not link.a:
            keys.append((HERE + m.name, link.b))
        zone = self.site.zone_at(m.x, m.y)
        if zone is not None and self.site.zones[zone].kind == SHARED:
            reach = self.params.footprint_radius_m + self.params.release_margin_m
            here = (m.x, m.y)
            for node in self._reach(zone):
                key, end = (HERE + m.name, node), self.site.node_pose(node)
                toward = ((point[0] - here[0]) * (end[0] - here[0])
                          + (point[1] - here[1]) * (end[1] - here[1])) > 0.0
                if (key not in keys and toward
                        and point_segment_distance(point, here, end) <= reach):
                    keys.append(key)
        return keys

    def _mark_point(self, m, point, keys, now: float) -> Mark:
        p = self.params
        mark = next((mk for mk in self._marks.values()
                     if math.dist(point, (mk.x, mk.y)) <= p.near_m), None)
        if mark is None:
            retry = p.blocked_retry_s
            old = next((item for item in self._mark_memory
                        if math.dist(point, item[:2]) <= p.near_m), None)
            if old is not None:
                self._mark_memory.remove(old)
                retry = min(2.0 * old[2], max(p.blocked_retry_max_s, p.blocked_retry_s))
            mark = Mark(self._mark_seq(), point[0], point[1], m.name, now, now + retry, retry,
                        asked_at=now)
            self._marks[mark.id] = mark
        else:
            mark.until, mark.clears = now + mark.retry_s, 0
            mark.check, mark.asked = 0, ""
        for key in keys:
            self._hold_key(mark, key)
        shown = ", ".join(f"{a or 'here'}->{b}" for a, b in keys)
        self._event("WARN", "ZONE_MARKED_BLOCKED",
                    f"{shown} at ({point[0]:.2f}, {point[1]:.2f}) in "
                    f"{self.site.zone_at(*point)}, tried again in {mark.retry_s:.0f} s", m.name)
        return mark

    def _hold_key(self, mark, key) -> None:
        if key not in mark.keys:
            mark.keys.append(key)
        self._blocked[key] = max(self._blocked.get(key, 0.0), mark.until)

    def _drop_mark(self, mark) -> None:
        del self._marks[mark.id]
        for key in mark.keys:
            rest = [mk.until for mk in self._marks.values() if key in mk.keys]
            if rest:
                self._blocked[key] = max(rest)
            else:
                self._blocked.pop(key, None)

    def _expire_marks(self, now: float) -> None:
        keep = self.params.blocked_retry_max_s
        self._mark_memory = [item for item in self._mark_memory if now - item[3] <= keep]
        for mark in [mk for mk in self._marks.values() if mk.until <= now]:
            self._drop_mark(mark)
            self._mark_memory.append((mark.x, mark.y, mark.retry_s, now))

    def _yielded_lately(self, m, now: float) -> bool:
        return m.robot_yield_at is not None and now - m.robot_yield_at <= (
            self.params.blocked_retry_s)
