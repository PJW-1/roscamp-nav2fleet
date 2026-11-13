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
)

from .robots import BACKOFF
from .trips import Trip


class Watch:
    def _watch_stall(self, m, now: float) -> None:
        p = self.params
        waiting = m.move_order is not None and m.move_order.waiting
        rests = m.robot_state in (BLOCKED, SELF_RECOVERING)
        if (m.goal is None or self._run_mode != RUN or m.arrival_by is not None or waiting
                or (rests and m.goal.kind not in (DOCK, BACKOFF))):
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
        if goal.kind == BACKOFF:
            self._backoff_ended(m, STALLED)
        elif at_end:
            m.goal = goal
            self._arrived_anyway(m, STALLED)
            m.retry_at = now + p.arrival_settle_s
        else:
            self._goal_failed(m, STALLED)

    def _watch_outside(self, m) -> None:
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
        self._take_where_it_stands(m)

    def _take_where_it_stands(self, m) -> None:
        zone = self.site.zone_at(m.x, m.y)
        if zone is None or self.site.zones[zone].kind != SHARED:
            return
        if m.goal is not None and self._via(m) != NAV_GOAL:
            return
        if self.grants.force(m.name, [zone], []):
            if m.goal is None:
                inside = self.site.nodes_in(zone)
                m.node = self.site.nearest_node(m.x, m.y, inside) or m.node
            self._event("WARN", "OUTSIDE_ZONE", f"takes {zone} where it stands", m.name)

    def _watch_near(self, now: float) -> None:
        p = self.params

        def roams(m) -> bool:
            return (m.available and m.goal is not None and m.goal.kind == MOVE
                    and self._via(m) == NAV_GOAL)

        moving = [m for m in self._robots.values() if roams(m)]
        seen = set()
        for index, a in enumerate(moving):
            for b in moving[index + 1:]:
                spots = ((a.x, a.y), (b.x, b.y))
                if math.dist(*spots) > p.near_m or not (roams(a) and roams(b)):
                    continue
                key = (a.name, b.name)
                seen.add(key)
                since, before = self._near.get(key, (now, spots))
                if any(math.dist(old, new) >= p.stall_move_m
                       for old, new in zip(before, spots, strict=True)):
                    since, before = now, spots
                self._near[key] = (since, before)
                if now - since >= p.near_stuck_s:
                    low, other = sorted((a, b), key=lambda m: (self._priority(m), m.name))
                    if self._awaits_blocked(low, now - since):
                        continue
                    self._cancel_goal(low)
                    self._stand_aside(low, other, now)
                    del self._near[key]
        for key in [key for key in self._near if key not in seen]:
            del self._near[key]

    def _watch_deadlock(self, now: float) -> frozenset:
        graph = {}
        for name, want in self._wants.items():
            if not self._robots[name].available or self._robots[name].redock:
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
            while rest and not self._cycle_acted:
                victim = choose_retreat(rest, lambda name: self._priority(self._robots[name]))
                self._cycle_acted = self._evade(self._robots[victim])
                rest.remove(victim)
        return frozenset()

    def _evade(self, m) -> bool:
        return self._to_parking(m, "WARN", "DEADLOCK_SEEN", f"{m.name} gives way to")

    def _to_parking(self, m, level: str, code: str, why: str, clear_of=None) -> bool:
        if m.goal is not None:
            return False
        waited = self.grants.waiting().get(m.name)
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
        self.grants.note_waiting(m.name, waited)
        return False
