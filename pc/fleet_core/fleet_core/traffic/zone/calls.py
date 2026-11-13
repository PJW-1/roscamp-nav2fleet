from __future__ import annotations

import math

from fleet_core.common.commands import NO_ROBOT, DispatchFeedback, DispatchResult
from fleet_core.fleet_map import FILL, NAV_GOAL, route
from fleet_core.traffic.mission import (
    AT_STATION,
    DOCK,
    IDLE,
    MOVE,
    OUT_OF_SERVICE,
    PARKED,
    STOP,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)
from fleet_core.traffic.selector import RobotInfo

from .robots import BACKOFF, Dispatch


class Calls:
    def on_run_mode(self, run_mode) -> None:
        if run_mode == STOP and self._run_mode != STOP:
            for m in self._robots.values():
                if m.goal and not m.comm:
                    self._cancel_goal(m)
        if run_mode != self._run_mode:
            for m in self._robots.values():
                m.stall_from = None
        self._run_mode = run_mode

    def on_dispatch(self, goal_id, station_id, robot_id="") -> None:
        if not robot_id:
            if station_id != self._work:
                raise ValueError(f"only the fill station takes open calls, not {station_id}")
            self._open_calls.append(goal_id)
            return
        m = self._robots.get(robot_id)
        if m is None:
            self._out.append(DispatchResult(goal_id, False, robot_id, "NO_SUCH_ROBOT"))
        elif not self.available(robot_id):
            self._out.append(DispatchResult(goal_id, False, robot_id, "NOT_AVAILABLE"))
        elif m.dispatch:
            self._out.append(DispatchResult(goal_id, False, robot_id, "BUSY"))
        else:
            m.dispatch = Dispatch(goal_id, station_id)
            if m.phase == AT_STATION and m.dest == station_id:
                self._finish_dispatch(m)
            else:
                m.hold = False
                self._head_for(m, self._leg_of(station_id), station_id)

    def on_dispatch_cancel(self, goal_id) -> None:
        if goal_id in self._open_calls:
            self._open_calls.remove(goal_id)
            return
        for m in self._robots.values():
            if m.dispatch and m.dispatch.goal_id == goal_id:
                m.dispatch = None
                self._out.append(DispatchResult(goal_id, False, m.name, "CANCELED"))
                if m.goal and m.goal.kind == DOCK:
                    self._cancel_goal(m)
                if m.available:
                    self._head_for(m, TO_PARK, None)
                return

    def on_release(self, robot) -> None:
        m = self._robots.get(robot)
        if m is not None:
            m.hold = False

    def on_robot_fault(self, robot, reason="") -> None:
        m = self._robots.get(robot)
        if m is not None:
            self._fail(m, reason)

    def tick_pick(self, stopped: frozenset = frozenset()) -> None:
        pool = [m for m in self._robots.values() if m.placed and m.fault != OUT_OF_SERVICE]
        self._warn_if_no_robot()
        chosen = self.selector.pick(len(self._open_calls),
                                    [self._robot_info(m, m.name in stopped) for m in pool])
        for m in pool:
            m.called = m.name in chosen
        for m in pool:
            if m.available:
                self._redirect(m)

    def _robot_info(self, m, stopped: bool) -> RobotInfo:
        node = m.trip.goal if m.trip is not None and m.phase == TO_PARK else m.node
        eligible = (self.available(m.name) and not m.hold and m.dispatch is None and not stopped
                    and (not m.charge or (m.called and self._committed(m))))
        return RobotInfo(m.name, node, m.called or self._committed(m), eligible)

    def _committed(self, m) -> bool:
        dock = self.site.stations[self._work].dock
        return m.phase == TO_WORK and self.site.nodes[dock].zone in self.grants.held_by(m.name)

    def _warn_if_no_robot(self) -> None:
        stalled = bool(self._open_calls) and not any(m.live for m in self._robots.values())
        if stalled and not self._no_robot_warned:
            self._out.append(DispatchFeedback(self._open_calls[0], "", NO_ROBOT))
        self._no_robot_warned = stalled

    def _redirect(self, m) -> None:
        if m.phase == AT_STATION:
            self._leave_station(m)
        elif m.phase in (PARKED, IDLE):
            if m.dispatch or (m.called and not m.charge) or m.phase == IDLE:
                self._next_work(m)
        elif m.phase not in (TO_WORK, TO_PARK) or m.dispatch or self._committed(m):
            return
        elif m.called and m.phase != TO_WORK:
            self._head_for(m, TO_WORK, self._work)
        elif not m.called and m.phase == TO_WORK:
            self._head_for(m, TO_PARK, None)

    def _leg_of(self, station_id: str) -> str:
        return TO_WORK if self.site.stations[station_id].action == FILL else TO_OUTBOUND

    def _head_for(self, m, phase: str, station_id) -> None:
        if (m.phase, m.dest) == (phase, station_id):
            return
        m.phase, m.dest = phase, station_id
        m.asked_at = self._clock() if station_id else None
        m.asked_gap = self._metres_to_dock(m, station_id) if station_id else 0.0
        m.attempts = m.dock_tries = 0
        m.redock = False
        self._stop_wanting(m)
        goal = m.goal
        if goal is None:
            self._drop_trip(m)
        elif goal.kind == MOVE and self._via(m) == NAV_GOAL:
            self._cancel_goal(m)
            self._drop_trip(m)
        elif goal.kind == BACKOFF:
            self._drop_trip(m)
        elif m.trip is not None:
            del m.trip.links[m.trip.granted:]
            m.trip.stale = True

    def _metres_to_dock(self, m, station_id: str) -> float:
        way = route(self.site, m.node, self.site.stations[station_id].dock)
        return math.inf if way is None else sum(link.length for link in way)

    def _finish_dispatch(self, m, error_x_cm=0.0, error_y_cm=0.0, error_yaw_deg=0.0) -> None:
        if m.dispatch:
            goal_id, m.dispatch = m.dispatch.goal_id, None
        elif m.dest == self._work and self._open_calls:
            goal_id = self._open_calls.pop(0)
        else:
            return
        m.hold = True
        self._out.append(DispatchResult(goal_id, True, m.name, "", error_x_cm, error_y_cm,
                                        error_yaw_deg))

    def _leave_station(self, m) -> None:
        if m.hold:
            return
        if m.called and m.dest == self._work and self._open_calls:
            self._finish_dispatch(m)
        else:
            self._next_work(m)

    def _next_work(self, m) -> None:
        if m.dispatch:
            self._head_for(m, self._leg_of(m.dispatch.station_id), m.dispatch.station_id)
        elif m.called:
            self._head_for(m, TO_WORK, self._work)
        elif m.node in self.site.parking and self._on_node(m):
            m.phase, m.dest = PARKED, None
        else:
            self._head_for(m, TO_PARK, None)
