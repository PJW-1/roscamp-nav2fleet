"""막힘(지시서 2-2의 7번): 기다리고, 안 풀리면 그 연결을 막힘으로 표시하고 다른 길을 고른다.

넓은 구역 안이면 goal을 거두고 다른 지점을 거쳐 간다. 한 줄 구역이나 도크 구역 안이면 지나온
지점까지 후진(Retreat)으로 물러난 뒤 다른 길로 간다. 넓은 구역에서 막힌 로봇의 길에 다른 로봇이
goal 없이 서 있으면 그 로봇을 주차 자리로 보낸다.
"""
from __future__ import annotations

import math

from fleet_core.fleet_map import RETREAT, SHARED
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


class Blocked:
    def _watch_blocked(self, m, now: float) -> None:
        self._note_self_recovery(m, now)
        p = self.params
        long_recovery = (m.self_recovery_since is not None
                         and now - m.self_recovery_since > p.self_recovery_limit_s)
        stuck = m.robot_state == BLOCKED or long_recovery
        if not stuck or m.goal is None or m.goal.kind == DOCK:
            if not stuck:
                m.blocked_since = None
                m.blocked_noted = False
            return
        if m.blocked_since is None or m.blocked_goal != m.goal.id:
            m.blocked_since, m.blocked_goal = now, m.goal.id  # 새 goal의 막힘은 새로 잰다
        if not m.blocked_noted:
            m.blocked_noted = True
            self._event("INFO", "BLOCKED_WAIT", f"stands before something on the way to "
                        f"{m.goal.end}", m.name)
        waited = now - m.blocked_since
        backing = self._via(m) == RETREAT
        if not backing and waited >= p.blocked_mark_s:
            self._mark_blocked(m, now)
        elif backing and waited >= p.blocked_timeout_s:
            # 물러나는 길이 막혔다. 다른 길이 없으니 운영자에게 넘긴다
            self._event("ERROR", "GOAL_FAILED", f"blocked for {waited:.0f} s while backing out",
                        m.name)
            self._fail(m, BLOCKED)

    def _mark_blocked(self, m, now: float) -> None:
        """지금 달리던 연결을 막힘으로 표시하고 다른 길을 찾게 한다."""
        p, site = self.params, self.site
        link = m.trip.links[m.trip.done]
        until = now + p.blocked_retry_s
        if link.a:
            self._blocked[(link.a, link.b)] = until
            if site.link(link.b, link.a) is not None:
                self._blocked[(link.b, link.a)] = until
        else:  # 넓은 구역 한가운데: 여기서 그 지점으로 곧장 가지 않고 다른 지점을 거친다
            self._blocked[(HERE + m.name, link.b)] = until
        zone = site.zone_at(m.x, m.y)
        self._event("WARN", "ZONE_MARKED_BLOCKED",
                    f"{link.a or 'here'}->{link.b} in {zone}, tried again in "
                    f"{p.blocked_retry_s:.0f} s", m.name)
        self._cancel_goal(m)
        m.blocked_since = None
        if link.a and zone is not None and site.zones[zone].kind != SHARED:
            m.retreat_to = link.a  # 고정 구간 안: 지나온 지점까지 물러난다
            self._event("INFO", "REROUTE", f"backs out to {link.a}, then another way", m.name)
        else:
            self._event("INFO", "REROUTE", f"another way to {m.trip.goal}", m.name)
            self._make_way(m, link.b)
        self._drop_trip(m)

    def _make_way(self, m, node: str) -> None:
        """넓은 구역에서 막힌 로봇의 길 가까이에 goal 없이 서 있는 로봇 하나를 주차 자리로 보낸다.

        차례를 기다리며 선 로봇은 구역 대기 그래프에 없다. 그래서 허락을 쥔 로봇의 길을 몸으로
        막아도 대기 순환으로 보이지 않는다(Gazebo 3대 경기). 주차한 로봇은 보내지 않는다.
        """
        site, here = self.site, (m.x, m.y)
        end = site.node_pose(node)

        def in_the_way(other) -> bool:
            zone = site.zone_at(other.x, other.y)
            return (other is not m and other.available and other.goal is None
                    and other.phase in (TO_WORK, TO_OUTBOUND, TO_PARK)
                    and zone is not None and site.zones[zone].kind == SHARED
                    and point_segment_distance((other.x, other.y), here, end)
                    <= self.params.near_m)

        standing = [other for other in self._robots.values() if in_the_way(other)]
        for other in sorted(standing, key=lambda o: (math.dist(here, (o.x, o.y)), o.name)):
            if self._to_parking(other, "INFO", "HALL_YIELD",
                                f"out of the way of {m.name}, goes to", clear_of=m):
                return

    def _expire_blocked(self, now: float) -> None:
        for key in [key for key, until in self._blocked.items() if until <= now]:
            del self._blocked[key]

    def _note_self_recovery(self, m, now: float) -> None:
        """로봇이 스스로 짧게 물러났다 다시 가는 동안(RECOVERING)을 사건으로 남긴다."""
        if m.robot_state == SELF_RECOVERING:
            if m.self_recovery_since is None:
                m.self_recovery_since = now
                self._event("INFO", "SELF_RECOVERY", "backs up by itself", m.name)
        elif m.self_recovery_since is not None:
            self._event("INFO", "SELF_RECOVERY_END",
                        f"after {now - m.self_recovery_since:.0f} s", m.name)
            m.self_recovery_since = None
