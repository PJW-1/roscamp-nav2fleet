"""로봇 한 대의 한 걸음: 단계에 맞게 길을 고르고, 구간을 잡고, 조각 하나를 명령으로 낸다.

order 모델이 꺼져 있으면(use_order_model=false) 조각 하나를 보내고 결과를 기다린 뒤 다음을 보낸다.
"""
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

from .robots import Goal
from .trips import move_command, retreat_command, segment_end


class Drive:
    def step(self, robot) -> None:
        m = self._robots.get(robot)
        if m is None or not m.available:
            return
        for _ in range(4):  # 단계가 바뀌면 이어서 본다
            before = (m.phase, m.dest)
            self._step_phase(m)
            if (m.phase, m.dest) == before:
                break

    def _step_phase(self, m) -> None:
        now = self._clock()
        if m.goal is not None or now < m.retry_at or now < m.yield_until:
            return
        if m.phase in (TO_WORK, TO_OUTBOUND, TO_PARK):
            self._drive(m)
        elif m.phase == DOCKING:
            goal_id = self._next_goal_id()
            m.goal = Goal(DOCK, goal_id)
            m.stall_from = None
            self._out.append(DockCmd(m.name, goal_id, m.dest))

    def _drive(self, m) -> None:
        trip = m.trip
        if trip is not None:
            if trip.sent < trip.granted:
                self._send(m)
                return
            if not trip.finished and self._take(m, trip) is None:
                self._send(m)  # 같은 길의 다음 구간을 잡았다
                return
            self._drop_trip(m)  # 다 왔거나 다음 구간을 못 잡았다. 다시 고른다
        if self._at_purpose(m):
            self._arrive(m)
            return
        trip = self._choose(m)
        if trip is not None:
            m.trip = trip
            self._send(m)

    def _at_purpose(self, m) -> bool:
        """이 단계의 목적지에 서 있는가."""
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
            m.phase = DOCKING  # 채우기 도크에 먼저 닿은 로봇이 열린 호출을 받는다
        else:  # 불렸다가 호출이 사라졌다. 도크를 비운다
            self._head_for(m, TO_PARK, None)

    def _send(self, m) -> None:
        """잡은 구간의 다음 조각을 명령 하나로 낸다."""
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
            self._out.append(move_command(self.site, m.name, goal_id, links, m.x, m.y, after))
            code = "NAV_GOAL_SENT" if via == NAV_GOAL else "FOLLOW_SENT"
            self._event("INFO", code, f"to {target}", m.name)
        m.goal = Goal(RETREAT if via == RETREAT else MOVE, goal_id, end, target, via)
        trip.sent = end
        m.stall_from = None
