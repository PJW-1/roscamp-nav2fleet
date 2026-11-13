from __future__ import annotations

import math

from fleet_core.common.commands import RetreatCmd
from fleet_core.fleet_map import RETREAT, SHARED
from fleet_core.fleet_map.geometry import (
    point_rect_distance,
    point_segment_distance,
    sample,
    segment_rect_distance,
    wall_gap,
)
from fleet_core.traffic.mission import BLOCKED

from .robots import BACKOFF, Goal

NO_ROOM = "no room behind"
WAY_STEP_M = 0.02


def backoff_spot(site, params, held, m, others, nodes=()) -> tuple[tuple | None, str]:
    p = params
    zone = site.zone_at(m.x, m.y)
    if zone is None or site.zones[zone].kind != SHARED or zone not in held:
        return None, "not in a yard it holds"
    way = (-math.cos(m.yaw), -math.sin(m.yaw))
    here = (m.x, m.y)
    end = (m.x + way[0] * p.yard_backoff_m, m.y + way[1] * p.yard_backoff_m)
    edge = p.yard_backoff_edge_m
    fixed = [z.id for z in site.zones.values() if z.kind != SHARED]
    if (any(site.zone_at(x, y) != zone for x, y in sample(here, end, WAY_STEP_M))
            or wall_gap(site.bounds(), *end) < edge
            or any(site.distance_to_zone(*end, z) < edge for z in fixed)
            or any(point_rect_distance(shelf, *end) < edge
                   or segment_rect_distance(here, end, shelf) <= 0.0 for shelf in site.shelves)):
        return None, NO_ROOM
    for other in others:
        nearer = (m.x - other.x) * way[0] + (m.y - other.y) * way[1] < 0.0
        gap = point_segment_distance((other.x, other.y), here, end)
        if nearer and gap < p.retreat_clear_m:
            return None, f"{other.name} behind"
    for node in nodes:
        if math.dist(end, site.node_pose(node)) < p.node_clear_m:
            return None, f"too close to {node}"
    return end, ""


class Backoff:
    def _awaits_blocked(self, m, stuck_s: float) -> bool:
        p = self.params
        return (p.yard_backoff and m.robot_state != BLOCKED
                and stuck_s < p.near_stuck_s + p.yard_backoff_wait_s)

    def _stand_aside(self, m, other, now: float) -> None:
        p = self.params
        again = (m.yielded is not None and now - m.yielded[2] <= p.blocked_retry_s
                 and math.dist(m.yielded[:2], (m.x, m.y)) < p.near_m)
        m.yielded = (m.x, m.y, now)
        if again and self._to_parking(m, "INFO", "HALL_YIELD",
                                      f"out of the way of {other.name}, goes to", clear_of=other):
            m.yielded = None
            return
        stops = f"stops {p.near_stuck_s:.0f} s for {other.name}"
        if p.yard_backoff and m.robot_state == BLOCKED:
            why = self._back_off(m)
            if not why:
                self._event("INFO", "HALL_YIELD", f"backs {p.yard_backoff_m:.2f} m, then {stops}",
                            m.name)
                return
            stops += f" (no back-off: {why})"
        m.yield_until = now + p.near_stuck_s
        self._event("INFO", "HALL_YIELD", stops, m.name)

    def _back_off(self, m) -> str:
        p = self.params
        others = [other for other in self._robots.values() if other is not m and other.placed]
        nodes = [node for node in self.site.nodes if self.grants.node_holder(node) != m.name]
        end, why = backoff_spot(self.site, p, self.grants.held_by(m.name), m, others, nodes)
        if end is None:
            return why
        goal_id = self._next_goal_id()
        path = ((m.x, m.y, m.yaw), (*end, m.yaw))
        self._out.append(RetreatCmd(m.name, goal_id, p.yard_backoff_m, p.retreat_speed_mps,
                                    path))
        m.goal = Goal(BACKOFF, goal_id, m.trip.done if m.trip is not None else 0, "", RETREAT)
        m.backoff_from = (m.x, m.y)
        m.stall_from = None
        return ""

    def _backoff_result(self, robot, goal_id, reason: str) -> bool:
        m = self._active(robot, goal_id, BACKOFF)
        if m is None:
            return False
        m.goal = None
        self._backoff_ended(m, reason)
        return True

    def _backoff_ended(self, m, reason: str) -> None:
        moved = math.dist(m.backoff_from, (m.x, m.y))
        m.backoff_from = None
        m.yield_until = self._clock() + self.params.near_stuck_s
        if reason:
            self._event("WARN", "HALL_BACKOFF_FAILED", f"{reason} after {moved:.2f} m", m.name)
        else:
            self._event("INFO", "HALL_BACKOFF_DONE", f"backed {moved:.2f} m", m.name)
