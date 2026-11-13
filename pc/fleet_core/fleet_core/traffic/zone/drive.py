from __future__ import annotations

from fleet_core.common.commands import DockCmd
from fleet_core.fleet_map import NAV_GOAL, RETREAT
from fleet_core.traffic.mission import (
    DOCK,
    DOCKING,
    MOVE,
    PARKED,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)

from .robots import ORDER, Goal
from .trips import move_command, retreat_command, segment_end


class Drive:
    def step(self, robot) -> None:
        m = self._robots.get(robot)
        if m is None or not m.available:
            return
        for _ in range(4):
            before = (m.phase, m.dest)
            self._step_phase(m)
            if (m.phase, m.dest) == before:
                break

    def _step_phase(self, m) -> None:
        now = self._clock()
        if m.goal is not None:
            if m.goal.via == ORDER:
                self._extend_order(m)
            return
        if now < m.retry_at or now < m.yield_until:
            return
        if m.phase in (TO_WORK, TO_OUTBOUND, TO_PARK):
            self._drive(m)
        elif m.phase == DOCKING:
            if m.redock and self._reapproach(m):
                return
            goal_id = self._next_goal_id()
            m.goal = Goal(DOCK, goal_id)
            m.stall_from = None
            self._out.append(DockCmd(m.name, goal_id, m.dest))

    def _drive(self, m) -> None:
        trip = m.trip
        if trip is not None:
            if trip.stale and not trip.finished and self._switch(m):
                return
            if trip.sent < trip.granted:
                self._send(m)
                return
            if not trip.finished and self._take(m, trip) is None:
                self._send(m)
                return
            self._drop_trip(m)
        if self._at_purpose(m):
            self._arrive(m)
            return
        trip = self._choose(m)
        if trip is not None:
            m.trip = trip
            self._send(m)

    def _switch(self, m) -> bool:
        fresh = self._choose(m)
        if fresh is None:
            self._stop_wanting(m)
            return False
        m.trip = fresh
        self._send(m)
        return True

    def _at_purpose(self, m) -> bool:
        if not self._on_node(m) or m.retreat_to:
            return False
        if m.phase == TO_PARK:
            return m.node in self.site.parking
        return m.node == self.site.stations[m.dest].dock

    def _arrive(self, m) -> None:
        self._stop_wanting(m)
        if m.phase == TO_PARK:
            m.phase = PARKED
        elif m.phase == TO_OUTBOUND or m.dispatch or self._open_calls:
            m.phase = DOCKING
        else:
            self._head_for(m, TO_PARK, None)

    def _send(self, m) -> None:
        if self.params.use_order_model:
            self._send_order(m)
            return
        trip = m.trip
        start = trip.sent
        end = segment_end(trip.links, start, trip.granted)
        links = trip.links[start:end]
        goal_id = self._next_goal_id()
        via, target = links[0].via, links[-1].b
        if via == RETREAT:
            self._out.append(retreat_command(self.params, m.name, goal_id, links, m.x, m.y))
            self._event("INFO", "RETREAT_START", f"backs to {target}", m.name)
        else:
            after = trip.links[end] if end < len(trip.links) else None
            self._out.append(move_command(self.site, m.name, goal_id, links, m.x, m.y, after,
                                          self._goal_priority(m)))
            code = "NAV_GOAL_SENT" if via == NAV_GOAL else "FOLLOW_SENT"
            self._event("INFO", code, f"to {target}", m.name)
        m.goal = Goal(RETREAT if via == RETREAT else MOVE, goal_id, end, target, via)
        trip.sent = end
        m.stall_from = None
