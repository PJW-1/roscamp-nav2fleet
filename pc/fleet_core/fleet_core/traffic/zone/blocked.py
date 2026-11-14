# ---- BLOCKED 로봇 처리 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL, RETREAT, SHARED
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import (
    BLOCKED,
    DOCK,
    SELF_RECOVERING,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)

from .base import HERE
from .robots import BACKOFF
from .trips import segment_end
from .yard_retreat import yard_retreat


# ---- 막힌 상태와 시작 시각 기록 ----
class Blocked:
    def _watch_blocked(self, m, now: float) -> None:
        self._note_self_recovery(m, now)
        p = self.params
        long_recovery = (m.self_recovery_since is not None
                         and now - m.self_recovery_since > p.self_recovery_limit_s)
        stuck = m.robot_state == BLOCKED or long_recovery
        if not stuck or m.goal is None or m.goal.kind in (DOCK, BACKOFF):
            if not stuck:
                m.blocked_since = None
                m.blocked_noted = m.rear_noted = False
            return
        if m.blocked_since is None or m.blocked_goal != m.goal.id:
            m.blocked_since, m.blocked_goal = now, m.goal.id
            m.rear_noted = False
        if not m.blocked_noted:
            m.blocked_noted = True
            self._event("INFO", "BLOCKED_WAIT", f"stands before something on the way to "
                        f"{m.goal.end}", m.name)
        # ---- 기다린 시간에 따라 양보, 막힘 표시, 실패 결정 ----
        waited = now - m.blocked_since
        backing = self._via(m) == RETREAT
        if not backing and waited >= p.blocked_mark_s:
            behind = self._behind_on_the_way_back(m)
            point = self._obstacle_point(m) if not behind else None
            other = self._robot_at(m, point) if point is not None else None
            if (other is not None and not self._yielded_lately(m, now)
                    and self._yield_to_robot(m, other, now)):
                m.rear_noted = False
            elif not behind:
                m.rear_noted = False
                self._mark_blocked(m, now, None if other is not None else point)
            elif waited >= p.blocked_timeout_s:
                self._event("ERROR", "GOAL_FAILED",
                            f"blocked for {waited:.0f} s with {behind} behind it", m.name)
                self._fail(m, BLOCKED)
            elif not m.rear_noted:
                m.rear_noted = True
                self._event("INFO", "RETREAT_WAIT", f"{behind} is behind it, backs out after it",
                            m.name)
        elif backing and waited >= p.blocked_timeout_s:
            self._event("ERROR", "GOAL_FAILED", f"blocked for {waited:.0f} s while backing out",
                        m.name)
            self._fail(m, BLOCKED)

    # ---- 되돌아갈 길 뒤에 있는 로봇 확인 ----
    def _behind_on_the_way_back(self, m) -> str:
        site = self.site
        link = m.trip.links[m.trip.done]
        zone = site.zone_at(m.x, m.y)
        if not link.a or zone is None or site.zones[zone].kind == SHARED:
            return ""
        target, via = yard_retreat(site, link)
        if via:
            return self._behind_on_the_bent_way(m, target, via)
        here, back = (m.x, m.y), site.node_pose(link.a)
        way = (back[0] - here[0], back[1] - here[1])
        for other in self._robots.values():
            if other is m or not other.placed:
                continue
            spot = (other.x, other.y)
            toward = (spot[0] - here[0]) * way[0] + (spot[1] - here[1]) * way[1] > 0.0
            if toward and point_segment_distance(spot, here, back) < self.params.retreat_clear_m:
                return other.name
        return ""

    # ---- 막힘 표시와 다른 길로 다시 가기 ----
    def _link_on(self, m):
        trip = m.trip
        first = trip.done
        if trip.links[first].via != NAV_GOAL:
            return trip.links[first]
        end = segment_end(self.site, trip.links, first, max(m.goal.links, first + 1))
        return min(trip.links[first:end], key=lambda link: point_segment_distance(
            (m.x, m.y), link.points[0], link.points[-1]))

    def _mark_blocked(self, m, now: float, point=None) -> None:
        p, site = self.params, self.site
        link = self._link_on(m)
        keys = self._point_keys(m, point, link) if point is not None else []
        zone = site.zone_at(m.x, m.y)
        if keys:
            mark = self._mark_point(m, point, keys, now)

            def hold(key):
                self._hold_key(mark, key)
        else:
            until = now + p.blocked_retry_s

            def hold(key):
                self._blocked[key] = until
            if link.a:
                hold((link.a, link.b))
                if site.link(link.b, link.a) is not None:
                    hold((link.b, link.a))
            else:
                hold((HERE + m.name, link.b))
            self._event("WARN", "ZONE_MARKED_BLOCKED",
                        f"{link.a or 'here'}->{link.b} in {zone}, tried again in "
                        f"{p.blocked_retry_s:.0f} s", m.name)
        if keys:
            self._remember_way(m)
        self._cancel_goal(m)
        m.blocked_since = None
        if link.a and zone is not None and site.zones[zone].kind != SHARED:
            self._set_retreat(m, link, now)
            self._event("INFO", "REROUTE", f"backs out to {m.retreat_to}, then another way",
                        m.name)
        else:
            self._event("INFO", "REROUTE", f"another way to {m.trip.goal}", m.name)
            if not self._make_way(m, link.b):
                hold((HERE + m.name, link.b))
        self._drop_trip(m)

    # ---- 길을 막고 선 로봇 주차 지점으로 보내기 ----
    def _make_way(self, m, node: str) -> bool:
        site, here = self.site, (m.x, m.y)
        end = site.node_pose(node)

        def in_the_way(other) -> bool:
            return (other is not m and other.available and other.goal is None
                    and other.phase in (TO_WORK, TO_OUTBOUND, TO_PARK)
                    and self._in_yard(other)
                    and point_segment_distance((other.x, other.y), here, end)
                    <= self.params.near_m)

        standing = [other for other in self._robots.values() if in_the_way(other)]
        for other in sorted(standing, key=lambda o: (math.dist(here, (o.x, o.y)), o.name)):
            if self._to_parking(other, "INFO", "HALL_YIELD",
                                f"out of the way of {m.name}, goes to", clear_of=m):
                return True
        return False

    # ---- 로봇단 양보 기록과 막힘 만료 ----
    def _note_local_yield(self, m, now: float) -> None:
        if m.local_yield:
            if m.local_yield_since is None:
                m.local_yield_since = now
            if m.local_yield != m.local_yield_noted:
                m.local_yield_noted = m.local_yield
                self._event("INFO", "LOCAL_YIELD", m.local_yield, m.name)
            return
        self._event("INFO", "LOCAL_YIELD", f"ends {m.robot_state or '?'} after "
                    f"{now - m.local_yield_since:.0f} s", m.name)
        m.local_yield_since, m.local_yield_noted = None, ""

    def _expire_blocked(self, now: float) -> None:
        for key in [key for key, until in self._blocked.items() if until <= now]:
            del self._blocked[key]

    # ---- 자가 복구 시작과 끝 기록 ----
    def _note_self_recovery(self, m, now: float) -> None:
        if m.local_yield or m.local_yield_since is not None:
            self._note_local_yield(m, now)
            return
        if m.robot_state == SELF_RECOVERING:
            if m.self_recovery_since is None:
                m.self_recovery_since = now
                self._event("INFO", "SELF_RECOVERY", "backs up by itself", m.name)
        elif m.self_recovery_since is not None:
            self._event("INFO", "SELF_RECOVERY_END",
                        f"after {now - m.self_recovery_since:.0f} s", m.name)
            m.self_recovery_since = None
