# ---- 막힘 표시 책: 만들기, 만료, 해제 확인 ----
from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import count, pairwise

from fleet_core.common.commands import BLOCKED_AT, CLEAR, NO_ANSWER, CheckClearCmd
from fleet_core.fleet_map import NAV_GOAL
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import LOCALIZING


# ---- 막힘 표시 기록 ----
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


# ---- 점 근처를 지나는 링크 키 찾기 ----
def polyline_gap(point, points) -> float:
    if len(points) == 1:
        return math.dist(point, points[0])
    return min(point_segment_distance(point, a, b) for a, b in pairwise(points))


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



# ---- 막힘 표시와 해제 확인 기록 ----
class MarkBook:
    def __init__(self, params, blocked, marks, robots, clock):
        self.params = params
        self._blocked = blocked
        self._marks = marks
        self._robots = robots
        self._clock = clock
        self._memory: list = []
        self._seq = count(1).__next__

    # ---- 막힘 표시 만들거나 갱신 ----
    def mark(self, by: str, point, keys, now: float) -> Mark:
        p = self.params
        mark = next((mk for mk in self._marks.values()
                     if math.dist(point, (mk.x, mk.y)) <= p.near_m), None)
        if mark is None:
            retry = p.blocked_retry_s
            old = next((item for item in self._memory
                        if math.dist(point, item[:2]) <= p.near_m), None)
            if old is not None:
                self._memory.remove(old)
                retry = min(2.0 * old[2], max(p.blocked_retry_max_s, p.blocked_retry_s))
            mark = Mark(self._seq(), point[0], point[1], by, now, now + retry, retry,
                        asked_at=now)
            self._marks[mark.id] = mark
        else:
            mark.until, mark.clears = now + mark.retry_s, 0
            mark.check, mark.asked = 0, ""
        for key in keys:
            self.hold(mark, key)
        return mark

    # ---- 링크 막기, 표시 지우기와 만료 ----
    def hold(self, mark, key) -> None:
        if key not in mark.keys:
            mark.keys.append(key)
        self._blocked[key] = max(self._blocked.get(key, 0.0), mark.until)

    def _drop(self, mark) -> None:
        del self._marks[mark.id]
        for key in mark.keys:
            rest = [mk.until for mk in self._marks.values() if key in mk.keys]
            if rest:
                self._blocked[key] = max(rest)
            else:
                self._blocked.pop(key, None)

    def expire(self, now: float) -> None:
        keep = self.params.blocked_retry_max_s
        self._memory = [item for item in self._memory if now - item[3] <= keep]
        for mark in [mk for mk in self._marks.values() if mk.until <= now]:
            self._drop(mark)
            self._memory.append((mark.x, mark.y, mark.retry_s, now))

    # ---- 막힘 표시 근처 로봇에게 확인 요청 만들기 ----
    def ask(self, now: float) -> list:
        p, asks = self.params, []
        for mark in self._marks.values():
            if now - mark.asked_at < p.check_interval_s:
                continue
            if mark.check:
                mark.clears = 0
            near = [m for m in self._robots.values()
                    if m.available and m.name != mark.by and m.report.robot_state != LOCALIZING
                    and math.dist((m.x, m.y), (mark.x, mark.y)) <= p.check_range_m]
            if not near:
                mark.check, mark.asked = 0, ""
                continue
            m = min(near, key=lambda r: (mark.asked_last.get(r.name, -math.inf),
                                         math.dist((r.x, r.y), (mark.x, mark.y)), r.name))
            mark.check, mark.asked, mark.asked_at = self._seq(), m.name, now
            mark.asked_last[m.name] = now
            asks.append(CheckClearCmd(m.name, mark.check, mark.x, mark.y, p.check_radius_m))
        return asks

    # ---- 확인 응답으로 표시를 지우거나 다시 막기 ----
    def answer(self, robot, check_id, answer) -> tuple | None:
        mark = next((mk for mk in self._marks.values()
                     if mk.check and mk.check == check_id and mk.asked == robot), None)
        if mark is None:
            return None
        mark.check = 0
        if answer == NO_ANSWER:
            mark.clears = 0
            return None
        now, p = self._clock(), self.params
        where = f"({mark.x:.2f}, {mark.y:.2f})"
        if answer == CLEAR:
            mark.clears += 1
            if mark.clears >= p.clear_confirm:
                self._drop(mark)
                self._memory = [item for item in self._memory
                                if math.dist(item[:2], (mark.x, mark.y)) > p.near_m]
                return ("INFO", "BLOCK_CLEARED", f"{where} clear by {mark.clears} checks, "
                        f"marked {now - mark.since:.0f} s ago")
        elif answer == BLOCKED_AT:
            mark.clears = 0
            mark.until = now + mark.retry_s
            for key in mark.keys:
                self.hold(mark, key)
            return ("INFO", "BLOCK_CONFIRMED",
                    f"{where} still blocked, tried again in {mark.retry_s:.0f} s")
        else:
            mark.clears = 0
        return None
