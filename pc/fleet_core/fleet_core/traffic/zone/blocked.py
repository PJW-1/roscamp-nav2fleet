# ---- BLOCKED 로봇 처리 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL, RETREAT, SHARED
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import (
    BLOCKED,
    DOCK,
    OUT_OF_SERVICE,
    SELF_RECOVERING,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)

from .base import HERE
from .block_marks import keys_at
from .routes import segment_end
from .yard_retreat import yard_retreat


# ---- 막힌 상태와 시작 시각 기록 ----
class Blocked:
    def _watch_blocked(self, m, now: float) -> None:
        ep = m.watch.blocked
        self._note_self_recovery(m, now)
        p = self.params
        long_recovery = (ep.self_recovery_since is not None
                         and now - ep.self_recovery_since > p.self_recovery_limit_s)
        stuck = m.report.robot_state == BLOCKED or long_recovery
        if not stuck or m.goal is None or m.goal.kind == DOCK:
            if not stuck:
                self._blocked_gap(m, now)
            return
        if ep.out and self._gap_over(m, now):
            self._end_blocked(m)  # 새 일화
        ep.clear_since = None
        if ep.since is None or ep.key != m.goal.id:
            if ep.key != m.goal.id:
                ep.noted = False  # goal이 바뀌면 새 일화
            ep.since, ep.key = now, m.goal.id
            ep.xy, ep.out, ep.count = (m.x, m.y), False, 0
            ep.rear_noted = False
        elif ep.out:
            ep.out = False
            ep.count += 1  # 짧은 틈 뒤 다시 막히면 시계를 이어 센다
        self._event_once(ep, "INFO", "BLOCKED_WAIT", f"stands before something on the way "
                         f"to {m.goal.end}", m.name)
        # ---- 기다린 시간에 따라 양보, 막힘 표시, 실패 결정 ----
        waited = now - ep.since
        backing = self._mode(m) == RETREAT
        if not backing and waited >= p.blocked_mark_s:
            behind = self._behind_on_the_way_back(m)
            point = self._obstacle_point(m) if not behind else None
            other = self._robot_at(m, point) if point is not None else None
            if (other is not None and not self._yielded_lately(m, now)
                    and self._yield_to_robot(m, other, now)):
                ep.rear_noted = False
            elif not behind:
                ep.rear_noted = False
                self._mark_blocked(m, now, None if other is not None else point)
            elif waited >= p.blocked_timeout_s:
                self._event("ERROR", "GOAL_FAILED",
                            f"blocked for {waited:.0f} s with {behind} behind it"
                            f"{self._gaps_note(m)}", m.name)
                self._fail(m, BLOCKED)
            elif not ep.rear_noted:
                ep.rear_noted = True
                self._event("INFO", "RETREAT_WAIT", f"{behind} is behind it, backs out after it",
                            m.name)
        elif backing and waited >= p.blocked_timeout_s:
            self._event("ERROR", "GOAL_FAILED", f"blocked for {waited:.0f} s while backing out"
                        f"{self._gaps_note(m)}", m.name)
            self._fail(m, BLOCKED)

    # ---- 막힘 아닌 보고: 일화를 끝낼지 이어 갈지 ----
    def _blocked_gap(self, m, now: float) -> None:
        ep = m.watch.blocked
        if ep.since is None and not ep.noted:
            return
        if m.goal is None or m.goal.id != ep.key or self._gap_over(m, now):
            self._end_blocked(m)
            return
        ep.out = True
        if m.report.robot_state == SELF_RECOVERING:
            ep.clear_since = None  # 자가 복구는 막힘의 일부라 끝내는 시간에 넣지 않는다
        elif ep.clear_since is None:
            ep.clear_since = now

    # ---- 일화 끝: 시작점에서 나아감, 막힘 아닌 보고가 오래 이어짐 ----
    def _gap_over(self, m, now: float) -> bool:
        ep = m.watch.blocked
        p = self.params
        moved = (ep.xy is not None
                 and math.dist(ep.xy, (m.x, m.y)) > p.blocked_reset_m)
        return moved or (ep.clear_since is not None
                         and now - ep.clear_since >= p.blocked_reset_s)

    @staticmethod
    def _end_blocked(m) -> None:
        ep = m.watch.blocked
        ep.since, ep.xy, ep.clear_since = None, None, None
        ep.noted = ep.rear_noted = ep.out = False
        ep.count = 0

    @staticmethod
    def _gaps_note(m) -> str:
        gaps = m.watch.blocked.count
        return f", after {gaps} short gaps" if gaps else ""

    # ---- 되돌아갈 길 뒤에 있는 로봇 확인 ----
    def _behind_on_the_way_back(self, m) -> str:
        site = self.site
        edge = m.route.edges[m.route.done]
        zone = site.zone_at(m.x, m.y)
        if not edge.a or zone is None or site.zones[zone].kind == SHARED:
            return ""
        target, via = yard_retreat(site, edge)
        if via:
            return self._behind_on_the_bent_way(m, target, via)
        return self._behind_along(m, ((m.x, m.y), site.node_pose(edge.a)))

    # ---- 장애물 위치와 그 자리의 로봇 찾기 ----
    def _obstacle_point(self, m) -> tuple | None:
        if not self.params.blocked_by_position or m.report.obstacle is None:
            return None
        return m.report.obstacle[:2]

    def _robot_at(self, m, point):
        near = [other for other in self._robots.values()
                if other is not m and other.placed and other.health.fault != OUT_OF_SERVICE
                and math.dist(point, (other.x, other.y)) <= self.params.near_m]
        return min(near, key=lambda o: (math.dist(point, (o.x, o.y)), o.name), default=None)

    # ---- 장애물 점으로 막을 edge 키 모으기 ----
    def _point_keys(self, m, point, edge) -> list:
        keys = keys_at(self.site, self.params, point)
        if not keys:
            return keys
        if not edge.a:
            keys.append((HERE + m.name, edge.b))
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

    # ---- 막힘 표시 만들거나 갱신하고 알리기 ----
    def _mark_point(self, m, point, keys, now: float):
        mark = self._mark_book.mark(m.name, point, keys, now)
        shown = ", ".join(f"{a or 'here'}->{b}" for a, b in keys)
        self._event("WARN", "ZONE_MARKED_BLOCKED",
                    f"{shown} at ({point[0]:.2f}, {point[1]:.2f}) in "
                    f"{self.site.zone_at(*point)}, tried again in {mark.retry_s:.0f} s"
                    f"{self._gaps_note(m)}", m.name)
        return mark

    # ---- 최근에 로봇에게 양보했는지 ----
    def _yielded_lately(self, m, now: float) -> bool:
        return m.avoid.robot_yield_at is not None and now - m.avoid.robot_yield_at <= (
            self.params.blocked_retry_s)

    # ---- 막힘 표시와 다른 길로 다시 가기 ----
    def _edge_on(self, m):
        route = m.route
        first = route.done
        if route.edges[first].mode != NAV_GOAL:
            return route.edges[first]
        end = segment_end(self.site, route.edges, first, max(m.goal.edges, first + 1))
        return min(route.edges[first:end], key=lambda edge: point_segment_distance(
            (m.x, m.y), edge.points[0], edge.points[-1]))

    def _mark_blocked(self, m, now: float, point=None) -> None:
        p, site = self.params, self.site
        edge = self._edge_on(m)
        keys = self._point_keys(m, point, edge) if point is not None else []
        zone = site.zone_at(m.x, m.y)
        if keys:
            mark = self._mark_point(m, point, keys, now)

            def hold(key):
                self._mark_book.hold(mark, key)
        else:
            until = now + p.blocked_retry_s

            def hold(key):
                self._blocked[key] = until
            if edge.a:
                hold((edge.a, edge.b))
                if site.edge(edge.b, edge.a) is not None:
                    hold((edge.b, edge.a))
            else:
                hold((HERE + m.name, edge.b))
            self._event("WARN", "ZONE_MARKED_BLOCKED",
                        f"{edge.a or 'here'}->{edge.b} in {zone}, tried again in "
                        f"{p.blocked_retry_s:.0f} s{self._gaps_note(m)}", m.name)
        if keys:
            self._detours.remember(m)
        self._cancel_goal(m)
        m.watch.blocked.since = None
        if edge.a and zone is not None and site.zones[zone].kind != SHARED:
            self._set_retreat(m, edge, now)
            self._event("INFO", "REROUTE", f"backs out to {m.avoid.retreat_to}, then another way",
                        m.name)
        else:
            self._event("INFO", "REROUTE", f"another way to {m.route.goal}", m.name)
            if not self._make_way(m, edge.b):
                hold((HERE + m.name, edge.b))
        self._drop_route(m)

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
        ep = m.watch.local_episode
        if m.report.local_yield:
            if ep.since is None:
                ep.since = now
            if m.report.local_yield != ep.noted:
                ep.noted = m.report.local_yield
                self._event("INFO", "LOCAL_YIELD", m.report.local_yield, m.name)
            return
        self._event("INFO", "LOCAL_YIELD", f"ends {m.report.robot_state or '?'} after "
                    f"{now - ep.since:.0f} s", m.name)
        ep.clear()

    def _expire_blocked(self, now: float) -> None:
        for key in [key for key, until in self._blocked.items() if until <= now]:
            del self._blocked[key]

    # ---- 자가 복구 시작과 끝 기록 ----
    def _note_self_recovery(self, m, now: float) -> None:
        if m.report.local_yield or m.watch.local_episode.since is not None:
            self._note_local_yield(m, now)
            return
        if m.report.robot_state == SELF_RECOVERING:
            if m.watch.blocked.self_recovery_since is None:
                m.watch.blocked.self_recovery_since = now
                self._event("INFO", "SELF_RECOVERY", "backs up by itself", m.name)
        elif m.watch.blocked.self_recovery_since is not None:
            self._event("INFO", "SELF_RECOVERY_END",
                        f"after {now - m.watch.blocked.self_recovery_since:.0f} s", m.name)
            m.watch.blocked.self_recovery_since = None
