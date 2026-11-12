"""감시: 제자리 멈춤, 구역 밖(기록만), 넓은 구역에서 서로 막음, 대기 순환(보험), 할 일 없이 섬."""
from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL, SHARED
from fleet_core.traffic.deadlock import choose_retreat, find_cycle
from fleet_core.traffic.mission import (
    BLOCKED,
    DOCK,
    MOVE,
    RUN,
    SELF_RECOVERING,
    STALLED,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)

from .trips import Trip


class Watch:
    def _watch_stall(self, m, now: float) -> None:
        """goal이 열린 채 제자리이면 거두고 실패한 시도로 센다. 끝 지점에 서 있으면 닿은 것이다."""
        p = self.params
        if (m.goal is None or self._run_mode != RUN or m.arrival_by is not None
                or m.robot_state in (BLOCKED, SELF_RECOVERING)):
            m.stall_from = None
            return
        here = (m.x, m.y)
        if m.stall_from is None or math.dist(m.stall_from, here) >= p.stall_move_m:
            m.stall_from, m.stall_since = here, now
            return
        limit = p.dock_stall_s if m.goal.kind == DOCK else p.stall_s
        if now - m.stall_since < limit:
            return
        self._event("WARN", "GOAL_STALLED",
                    f"{m.goal.kind} goal {m.goal.id} open, no movement for {limit:.0f} s", m.name)
        m.stall_from = None
        goal, at_end = m.goal, self._stands_at_end(m)
        self._cancel_goal(m)
        if at_end:  # 끝 지점에 서 있다: 열린 goal만 거두고 닿은 것으로 본다
            m.goal = goal
            self._arrived_anyway(m, STALLED)
            m.retry_at = now + p.arrival_settle_s  # 취소가 닿은 뒤에 다음 goal을 낸다
        else:
            self._goal_failed(m, STALLED)

    def _watch_outside(self, m) -> None:
        """보고 위치가 쥔 구역 밖에 이어서 있으면 사건으로 남긴다. 멈추지는 않는다."""
        p = self.params
        held = self.grants.held_by(m.name)
        gap = min((self.site.distance_to_zone(m.x, m.y, zone) for zone in held), default=0.0)
        if gap <= p.outside_zone_m:
            m.outside_n = 0
            m.outside_noted = False
            return
        m.outside_n += 1
        if m.outside_n < p.outside_zone_samples:
            return
        if not m.outside_noted:
            m.outside_noted = True
            self._event("WARN", "OUTSIDE_ZONE", f"at ({m.x:.2f}, {m.y:.2f}), {gap:.2f} m outside "
                        f"{list(held)} for {m.outside_n} reports", m.name)
        self._take_where_it_stands(m)  # 자리가 없었으면 보고마다 다시 본다

    def _take_where_it_stands(self, m) -> None:
        """쥔 구역 밖의 넓은 구역에 있는 로봇에게 그 구역을 쥐게 한다. 자리가 없으면 기록뿐이다.

        서 있는 로봇은 그래야 뒤에 남은 구역과 지점이 놓인다(progress._release_behind). Nav2
        goal로 달리는 로봇은 Nav2가 잡아 준 통로 밖으로 돌아간 것이다: 장부를 몸이 있는 곳에
        맞춘다. 고정 구간을 달리는 중이거나 넓은 구역이 아니면 손대지 않는다: 운영자가 본다.
        """
        zone = self.site.zone_at(m.x, m.y)
        if zone is None or self.site.zones[zone].kind != SHARED:
            return
        if m.goal is not None and self._via(m) != NAV_GOAL:
            return
        if self.grants.force(m.name, [zone], []):
            if m.goal is None:
                inside = [name for name, spec in self.site.nodes.items() if spec.zone == zone]
                m.node = self.site.nearest_node(m.x, m.y, inside) or m.node
            self._event("WARN", "OUTSIDE_ZONE", f"takes {zone} where it stands", m.name)

    def _watch_near(self, now: float) -> None:
        """넓은 구역에서 두 대가 가까이서 못 나아가면 우선순위 낮은 쪽을 잠시 세운다."""
        p = self.params
        moving = [m for m in self._robots.values()
                  if m.available and m.goal is not None and m.goal.kind == MOVE
                  and m.goal.via == NAV_GOAL]
        seen = set()
        for index, a in enumerate(moving):
            for b in moving[index + 1:]:
                spots = ((a.x, a.y), (b.x, b.y))
                if math.dist(*spots) > p.near_m:
                    continue
                key = (a.name, b.name)
                seen.add(key)
                since, before = self._near.get(key, (now, spots))
                if any(math.dist(old, new) >= p.stall_move_m
                       for old, new in zip(before, spots, strict=True)):
                    since, before = now, spots  # 누군가 나아갔다
                self._near[key] = (since, before)
                if now - since >= p.near_stuck_s:
                    low, other = sorted((a, b), key=lambda m: (self._priority(m), m.name))
                    self._cancel_goal(low)
                    low.yield_until = now + p.near_stuck_s
                    self._event("INFO", "HALL_YIELD", f"stops {p.near_stuck_s:.0f} s for "
                                f"{other.name}", low.name)
                    del self._near[key]
        for key in [key for key in self._near if key not in seen]:
            del self._near[key]

    def _watch_deadlock(self, now: float) -> frozenset:
        """구역 대기 그래프의 순환. 들어가기 규칙이 맞으면 없어야 하므로 보이면 사건으로 남긴다."""
        graph = {}
        for name, want in self._wants.items():
            if not self._robots[name].available:
                continue
            blockers = self.grants.blockers(name, want.zones, want.nodes, want.lanes)
            robots = {robot for group in blockers.values() for robot in group}
            if robots:
                graph[name] = robots
        cycle = tuple(find_cycle(graph) or ())
        if not cycle:
            self._cycle = ()
        elif set(cycle) != set(self._cycle):
            self._cycle, self._cycle_since, self._cycle_acted = cycle, now, False
            self._event("WARN", "DEADLOCK_SEEN", f"wait cycle {' -> '.join(cycle)}")
        elif not self._cycle_acted and now - self._cycle_since >= self.params.deadlock_grace_s:
            rest = list(cycle)
            while rest and not self._cycle_acted:  # 비킬 수 있는 로봇이 나올 때까지. 없으면 다음에
                victim = choose_retreat(rest, lambda name: self._priority(self._robots[name]))
                self._cycle_acted = self._evade(self._robots[victim])
                rest.remove(victim)
        return frozenset()

    def _evade(self, m) -> bool:
        """대기 순환을 풀려고 이 로봇을 빈 주차 자리로 보낸다. 보냈으면 True."""
        return self._to_parking(m, "WARN", "DEADLOCK_SEEN", f"{m.name} gives way to")

    def _to_parking(self, m, level: str, code: str, why: str, clear_of=None) -> bool:
        """goal이 없는 로봇을 가까운 빈 주차 자리로 보낸다. 목적은 그대로다. 보냈으면 True.

        clear_of가 있으면 그 로봇에 더 가까운 자리는 고르지 않는다: 그 앞을 다시 지나게 된다.
        """
        if m.goal is not None:
            return False
        free = [spot for spot in self.site.parking
                if self.grants.node_holder(spot) in ("", m.name)
                and (clear_of is None or self._gap(m, spot) < self._gap(clear_of, spot))]
        for spot in sorted(free, key=lambda spot: (self._gap(m, spot), spot)):
            links = self._way(m, spot)
            if not links:
                continue
            trip = Trip(spot, links)
            if self._take(m, trip) is None:
                m.trip = trip
                self._event(level, code, f"{why} {spot}", m.name)
                self._send(m)
                return True
        return False

    def _watch_idle(self, m, now: float) -> None:
        """갈 곳이 있는데 goal도 없고 기다리는 것도 없이 서 있으면 알린다."""
        p = self.params
        idle = (p.idle_watch_s > 0 and self._run_mode == RUN and m.goal is None
                and m.phase in (TO_WORK, TO_OUTBOUND, TO_PARK) and m.wait_since is None
                and now >= m.retry_at and now >= m.yield_until)
        if not idle:
            m.idle_since = None
        elif m.idle_since is None:
            m.idle_since = now
        elif now - m.idle_since >= p.idle_watch_s:
            m.idle_since = now
            self._event("WARN", "ROBOT_IDLE", f"{m.phase} at {m.node}: no goal and nothing to "
                        "wait for", m.name)
