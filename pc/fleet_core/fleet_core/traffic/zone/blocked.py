"""막힘(지시서 2-2의 7번): 기다리고, 안 풀리면 그 연결을 막힘으로 표시하고 다른 길을 고른다.

넓은 구역 안이면 goal을 거두고 다른 지점을 거쳐 간다. 한 줄 구역이나 도크 구역 안이면 지나온
지점까지 후진(Retreat)으로 물러난 뒤 다른 길로 간다. 물러날 길 위에 다른 로봇이 있으면 그 로봇이
먼저 나갈 때까지 미룬다(추가 지시서 2의 4절: 맨 뒤 로봇부터 물러난다). 넓은 구역에서 막힌 로봇의
길에 다른 로봇이 goal 없이 서 있으면 그 로봇을 주차 자리로 보낸다.
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
from .block_marks import yard_retreat
from .robots import BACKOFF


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
            m.blocked_since, m.blocked_goal = now, m.goal.id  # 새 goal의 막힘은 새로 잰다
            m.rear_noted = False
        if not m.blocked_noted:
            m.blocked_noted = True
            self._event("INFO", "BLOCKED_WAIT", f"stands before something on the way to "
                        f"{m.goal.end}", m.name)
        waited = now - m.blocked_since
        backing = self._via(m) == RETREAT
        if not backing and waited >= p.blocked_mark_s:
            behind = self._behind_on_the_way_back(m)
            point = self._obstacle_point(m) if not behind else None
            other = self._robot_at(m, point) if point is not None else None
            if other is not None and not self._yielded_lately(m, now):
                m.rear_noted = False  # 다른 로봇이 막았다: 표시하지 않고 비킨다(block_marks.py)
                self._yield_to_robot(m, other, now)
            elif not behind:
                m.rear_noted = False
                # 비켜도 또 로봇에 막혔으면 예전처럼 가던 연결을 표시한다(서로 마주 선 두 대)
                self._mark_blocked(m, now, None if other is not None else point)
            elif waited >= p.blocked_timeout_s:
                # 앞은 막혔고 뒤의 로봇은 끝내 나가지 않았다. 갈 데가 없으니 운영자에게 넘긴다
                self._event("ERROR", "GOAL_FAILED",
                            f"blocked for {waited:.0f} s with {behind} behind it", m.name)
                self._fail(m, BLOCKED)
            elif not m.rear_noted:  # 맨 뒤 로봇부터 물러난다. 보고마다 다시 본다
                m.rear_noted = True
                self._event("INFO", "RETREAT_WAIT", f"{behind} is behind it, backs out after it",
                            m.name)
        elif backing and waited >= p.blocked_timeout_s:
            # 물러나는 길이 막혔다. 다른 길이 없으니 운영자에게 넘긴다
            self._event("ERROR", "GOAL_FAILED", f"blocked for {waited:.0f} s while backing out",
                        m.name)
            self._fail(m, BLOCKED)

    def _behind_on_the_way_back(self, m) -> str:
        """고정 구간 안에서 물러날 길(지나온 지점까지) 위에 있는 다른 로봇의 이름. 없으면 빈 문자열.

        넓은 구역 안이면 물러나지 않으므로 늘 빈 문자열이다. 가는 쪽에 선 로봇은 아무리 가까워도
        세지 않는다: 그런 로봇까지 세면 가까이 선 두 대가 서로를 기다린다.
        """
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

    def _mark_blocked(self, m, now: float, point=None) -> None:
        """막힘으로 표시하고 다른 길을 찾게 한다. 장애물 위치(point)가 있으면 그 점이 든 연결을,
        없으면 지금 달리던 연결을 표시한다(block_marks.py)."""
        p, site = self.params, self.site
        link = m.trip.links[m.trip.done]
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
            else:  # 넓은 구역 한가운데: 여기서 그 지점으로 곧장 가지 않고 다른 지점을 거친다
                hold((HERE + m.name, link.b))
            self._event("WARN", "ZONE_MARKED_BLOCKED",
                        f"{link.a or 'here'}->{link.b} in {zone}, tried again in "
                        f"{p.blocked_retry_s:.0f} s", m.name)
        if keys:  # 위치로 표시했으면 다음에 잡은 길로 우회 종류를 센다(DETOUR)
            self._remember_way(m)
        self._cancel_goal(m)
        m.blocked_since = None
        if link.a and zone is not None and site.zones[zone].kind != SHARED:
            # 고정 구간 안: 지나온 지점까지, 그 지점이 갈림목이면 마당 지점까지 물러난다
            self._set_retreat(m, link, now)
            self._event("INFO", "REROUTE", f"backs out to {m.retreat_to}, then another way",
                        m.name)
        else:
            self._event("INFO", "REROUTE", f"another way to {m.trip.goal}", m.name)
            if not self._make_way(m, link.b):
                # 비켜 줄 로봇이 없다: 선 자리에서 그 지점으로 곧장 가면 같은 것에 또 막힌다
                hold((HERE + m.name, link.b))
        self._drop_trip(m)

    def _make_way(self, m, node: str) -> bool:
        """넓은 구역에서 막힌 로봇의 길 가까이에 goal 없이 서 있는 로봇 하나를 주차 자리로 보낸다.

        차례를 기다리며 선 로봇은 구역 대기 그래프에 없다. 그래서 허락을 쥔 로봇의 길을 몸으로
        막아도 대기 순환으로 보이지 않는다(Gazebo 3대 경기). 주차한 로봇은 보내지 않는다.
        보낸 로봇이 있으면 True.
        """
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

    def _note_self_recovery(self, m, now: float) -> None:
        """로봇이 스스로 짧게 물러났다 다시 가는 동안(RECOVERING)을 사건으로 남긴다.
        로봇단 교착 처리(local_yield가 있음, 추가 지시서 3의 7-4)는 단계가 바뀔 때마다와 끝날 때
        LOCAL_YIELD로 남긴다. 어느 쪽이든 움직임은 정하지 않는다(BLOCKED 뒤에 정한다)."""
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
