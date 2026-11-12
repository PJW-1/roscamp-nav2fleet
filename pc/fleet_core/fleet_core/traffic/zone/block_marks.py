"""막힘 표시를 위치 단위로(추가 지시서 3의 4-2와 4-3의 5번, 기준(사용자)).

로봇이 BLOCKED와 함께 장애물 위치를 보고하면(obstacle) 그 점이 든 연결만 막힘으로 표시한다.

- 고정 경로(FollowPath, 후진): 선이 그 점에서 발자국 반지름 + release_margin_m 안을 지난다.
- Nav2 goal: 그 점이 지나는 구역 안이고, 그 구역이 지점 없는 통로이거나(통로 전체가 막힌다),
  지점 있는 넓은 구역이면 두 끝을 잇는 곧은 선이 위와 같은 거리 안을 지난다(마당 하나가 통째로
  막히지 않게).

그 점 곁(near_m 안)에 관제가 아는 다른 로봇이 있으면 표시하지 않고 비키게 한다(HALL_YIELD).
위치가 없거나 든 연결이 없으면 blocked.py가 예전처럼 가던 연결을 표시한다.

표시(Mark)는 점과, 그 점 때문에 막은 연결들(_blocked의 열쇠)을 가진다. 지나가는 로봇이 확인하면
지우거나 새로 고친다(clear_checks.py). 아무도 확인하지 않으면 지우기 시간(retry_s) 뒤 지우고 길을
다시 열어 본다. 지운 뒤 같은 점에서 다시 막히면 지우기 시간을 두 배로 늘린다(blocked_retry_max_s
까지).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from fleet_core.fleet_map import LANE, NAV_GOAL, RETREAT, SHARED
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import OUT_OF_SERVICE

from .base import HERE


@dataclass
class Mark:
    id: int
    x: float
    y: float
    by: str  # 보고한 로봇
    since: float  # 처음 표시한 시각
    until: float  # 지울 시각
    retry_s: float  # 지우기 시간(다시 막히면 두 배)
    keys: list = field(default_factory=list)  # 막은 연결 (a, b)
    clears: int = 0  # 이어진 clear 답의 수
    asked_at: float = 0.0  # 마지막으로 물은 시각(처음에는 표시한 시각)
    check: int = 0  # 답을 기다리는 질문 번호. 없으면 0
    asked: str = ""  # 그 질문을 받은 로봇


def polyline_gap(point, points) -> float:
    """점과 꺾은선 사이의 거리."""
    if len(points) == 1:
        return math.dist(point, points[0])
    return min(point_segment_distance(point, a, b) for a, b in zip(points, points[1:],
                                                                    strict=False))


def keys_at(site, params, point) -> list:
    """그 점이 든 지도의 연결들 (a, b). 순서는 지도의 연결 순서다."""
    reach = params.footprint_radius_m + params.release_margin_m
    zone = site.zone_at(*point)
    keys = []
    for link in site.links():
        if link.via != NAV_GOAL:
            hit = polyline_gap(point, link.points) <= reach
        elif zone is None or zone not in link.zones:
            hit = False
        elif not site.nodes_in(zone):
            hit = True  # 지점 없는 통로: 어디로 지나든 그 점 곁을 지난다
        else:
            hit = point_segment_distance(point, link.points[0], link.points[-1]) <= reach
        if hit:
            keys.append((link.a, link.b))
    return keys


KEEPOUT, CORRIDOR, DESTINATION, IN_ZONE = "keepout", "corridor", "destination", "in_zone"


def detour_kind(site, old_goal: str, old_zones, trip) -> str:
    """막힌 뒤 새로 잡은 길이 어떤 우회인가(추가 지시서 3의 4-4). Nav2가 모르는 선택지부터 본다.

    destination: 목적지를 바꿨다(다른 대기 자리, 주차 자리). keepout: keepout 너머의 길로 바꿨다
    (마당이 막혀 급행으로, 급행이 막혀 마당으로: 지나는 한 줄 구역이 달라졌다). corridor: 지점 없는
    다른 통로로 간다(D, C). in_zone: 같은 넓은 구역 안에서 다른 지점을 거쳐 갈 뿐이다.
    """
    if trip.goal != old_goal:
        return DESTINATION
    # 들어온 입구까지 물러나는 후진(지도에 없는 연결)은 우회가 아니다
    new = {zone for link in trip.links if link.a or link.via != RETREAT for zone in link.zones}
    old = set(old_zones)

    def of(zones, test):
        return {zone for zone in zones if test(site.zones[zone])}

    def lane(spec):
        return spec.kind == LANE

    def corridor(spec):
        return spec.kind == SHARED and not site.nodes_in(spec.id)

    if of(new, lane) != of(old, lane):
        return KEEPOUT
    if of(new, corridor) - of(old, corridor):
        return CORRIDOR
    return IN_ZONE


class BlockMarks:
    def _remember_way(self, m) -> None:
        """막혀 길을 버리기 전에 가던 목적지와 지나던 구역을 적어 둔다(우회 종류를 셀 때 쓴다)."""
        trip = m.trip
        m.detour = (trip.goal, frozenset(z for link in trip.links[trip.done:] for z in link.zones))

    def _note_detour(self, m, trip) -> None:
        """막힌 뒤 처음 잡은 길이면 그 우회의 종류를 DETOUR로 남긴다."""
        if m.detour is None:
            return
        goal, zones = m.detour
        m.detour = None
        self._event("INFO", "DETOUR", f"{detour_kind(self.site, goal, zones, trip)}: "
                    f"{'->'.join(trip.route())}", m.name)

    def _obstacle_point(self, m) -> tuple | None:
        """m이 보고한 장애물 위치. 모르거나 위치로 표시하지 않게 꺼 두었으면 None."""
        if not self.params.blocked_by_position or m.obstacle is None:
            return None
        return m.obstacle[:2]

    def _robot_at(self, m, point):
        """그 점 곁(near_m 안)에 서 있는 다른 로봇. 없으면 None."""
        near = [other for other in self._robots.values()
                if other is not m and other.placed and other.fault != OUT_OF_SERVICE
                and math.dist(point, (other.x, other.y)) <= self.params.near_m]
        return min(near, key=lambda o: (math.dist(point, (o.x, o.y)), o.name), default=None)

    def _point_keys(self, m, point, link) -> list:
        """그 점 때문에 막을 연결들. 넓은 구역 한가운데에서 막혔으면 선 자리에서 가던 지점으로
        곧장 가는 길(예전 규칙 7과 같다)과, 선 자리에서 곧장 가면 그 점 곁을 지나는 지점들로 가는
        길도 막는다(통로 안에서 막힌 로봇이 그 점 너머로 가지 않게)."""
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
                    keys.append(key)  # 그 점을 지나 그 지점으로 간다. 지나온 쪽은 열어 둔다
        return keys

    def _mark_point(self, m, point, keys, now: float) -> Mark:
        """점과 연결들을 막힘으로 표시하고 ZONE_MARKED_BLOCKED를 낸다. 가까운 표시가 살아 있으면
        그 표시에 더한다(같은 것에 또 막혔다)."""
        p = self.params
        mark = next((mk for mk in self._marks.values()
                     if math.dist(point, (mk.x, mk.y)) <= p.near_m), None)
        if mark is None:
            retry = p.blocked_retry_s
            old = next((item for item in self._mark_memory
                        if math.dist(point, item[:2]) <= p.near_m), None)
            if old is not None:  # 지운 뒤 보내 본 로봇이 같은 점에서 다시 막혔다
                self._mark_memory.remove(old)
                retry = min(2.0 * old[2], max(p.blocked_retry_max_s, p.blocked_retry_s))
            mark = Mark(self._mark_seq(), point[0], point[1], m.name, now, now + retry, retry,
                        asked_at=now)
            self._marks[mark.id] = mark
        else:
            mark.until, mark.clears = now + mark.retry_s, 0
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
        """표시를 지운다. 다른 표시도 막은 연결은 그 표시의 시각까지 남긴다."""
        del self._marks[mark.id]
        for key in mark.keys:
            rest = [mk.until for mk in self._marks.values() if key in mk.keys]
            if rest:
                self._blocked[key] = max(rest)
            else:
                self._blocked.pop(key, None)

    def _expire_marks(self, now: float) -> None:
        """아무도 확인하지 않은 표시를 지운다. 같은 점에서 다시 막히면 시간을 늘리려고 남긴다."""
        keep = self.params.blocked_retry_max_s
        self._mark_memory = [item for item in self._mark_memory if now - item[3] <= keep]
        for mark in [mk for mk in self._marks.values() if mk.until <= now]:
            self._drop_mark(mark)
            self._mark_memory.append((mark.x, mark.y, mark.retry_s, now))

    def _yielded_lately(self, m, now: float) -> bool:
        """blocked_retry_s 안에 다른 로봇 때문에 이미 표시 없이 비켰다."""
        return m.robot_yield_at is not None and now - m.robot_yield_at <= (
            self.params.blocked_retry_s)

    def _yield_to_robot(self, m, other, now: float) -> None:
        """막은 것이 다른 로봇이다. 막힘으로 표시하지 않고 비킨다(HALL_YIELD).

        고정 구간 안이면 m이 들어온 입구로 물러난다(뒤의 로봇이 먼저 나간 뒤다: blocked.py가
        이미 보았다). 넓은 구역이면 goal 없이 선 그 로봇을 주차 자리로 보내고, 그럴 수 없으면 m이
        비켜 선다(backoff.py의 _stand_aside).
        """
        m.blocked_since = None
        m.robot_yield_at = now
        link = m.trip.links[m.trip.done]
        zone = self.site.zone_at(m.x, m.y)
        if link.a and zone is not None and self.site.zones[zone].kind != SHARED:
            self._cancel_goal(m)
            m.retreat_to = link.a
            self._drop_trip(m)
            self._event("INFO", "HALL_YIELD", f"backs out to {link.a} for {other.name}, "
                        "nothing marked", m.name)
            return
        if (other.available and other.goal is None and self._in_yard(other)
                and self._to_parking(other, "INFO", "HALL_YIELD",
                                     f"out of the way of {m.name}, goes to", clear_of=m)):
            return
        self._cancel_goal(m)
        self._stand_aside(m, other, now)
