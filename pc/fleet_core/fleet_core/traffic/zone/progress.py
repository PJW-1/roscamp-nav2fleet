from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL
from fleet_core.traffic.mission import (
    AT_STATION,
    COMM_LOST,
    DOCK,
    ERROR,
    LOCALIZING,
    NOT_PLACEABLE,
    OUT_OF_SERVICE,
    SELF_RECOVERING,
)

from .trips import passed

NODE_KEY = "node:"


def release_gap(p) -> float:
    return p.zone_clear_m or p.footprint_radius_m + p.release_margin_m


class Progress:
    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, obstacle=None,
                       local_yield="") -> None:
        m = self._robots.get(robot)
        if m is None:
            return
        now = self._clock()
        m.last_seen = now
        m.robot_state = state
        m.obstacle = obstacle
        m.local_yield = local_yield if state == SELF_RECOVERING else ""
        self._note_battery(m, battery)
        if state == LOCALIZING:
            m.pose_at = None
            m.clear_n.clear()
            return
        if m.comm == COMM_LOST:
            m.pose_at = None
        self._watch_pose_jump(m, x, y, yaw, now)
        m.x, m.y, m.yaw = x, y, yaw
        m.heard_since_success = True
        if m.fault == OUT_OF_SERVICE:
            return
        if not m.placed:
            self._place(m)
        elif m.comm == COMM_LOST:
            self._begin_recovery(m)
        elif not m.fault and not m.comm:
            self._track(m)
            self._release_behind(m)
            self._watch_outside(m)
            self._watch_blocked(m, now)
        elif m.fault == ERROR and not m.comm and m.goal is None:
            self._return_unreached(m)

    def _track(self, m) -> None:
        trip, goal = m.trip, m.goal
        if trip is None or goal is None or goal.kind == DOCK:
            return
        while (trip.done < goal.links - 1 and trip.links[trip.done].via != NAV_GOAL
               and passed(trip.links[trip.done], m.x, m.y)):
            m.node = trip.links[trip.done].b
            trip.done += 1

    def _still_needed(self, m) -> tuple[set, set]:
        trip = m.trip
        zones, nodes = set(), set()
        if trip is None:
            return zones, nodes
        links = trip.links[trip.done:trip.granted]
        if links:
            crossed = links[0].zones
            here = self.site.zone_at(m.x, m.y)
            if links[0].via != NAV_GOAL and here in crossed:
                crossed = crossed[crossed.index(here):]
            zones.update(crossed)
            for link in links[1:]:
                zones.update(link.zones)
            nodes.update(link.b for link in links)
        return zones, nodes

    def _release_behind(self, m, keep_trip: bool = True) -> list[str]:
        p, site, grants = self.params, self.site, self.grants
        held = grants.held_by(m.name)
        if (self._clock() < m.settle_until
                or all(site.distance_to_zone(m.x, m.y, zone) > p.outside_zone_m
                       for zone in held)):
            m.clear_n.clear()
            return []
        zones, nodes = self._still_needed(m) if keep_trip else (set(), set())
        if m.phase == AT_STATION and m.node in site.nodes and not p.zone_clear_m:
            (x, y), gap = site.node_pose(m.node), p.footprint_radius_m + m.dock_error_m
        else:
            (x, y), gap = (m.x, m.y), release_gap(p)
        released = []
        for zone in held:
            clear = zone not in zones and site.distance_to_zone(x, y, zone) >= gap - 1e-9
            if self._counted(m, zone, clear) and grants.release_zone(m.name, zone):
                released.append(zone)
        for node in grants.nodes_of(m.name):
            clear = node not in nodes and self._gap(m, node) >= p.node_clear_m
            if self._counted(m, NODE_KEY + node, clear):
                grants.release_node(m.name, node)
        return released

    def _return_unreached(self, m) -> None:
        moved = m.fault_xy is None or math.dist(m.fault_xy, (m.x, m.y)) > (
            self.params.arrival_tolerance_m)
        if m.robot_state in NOT_PLACEABLE or moved or m.pose_doubt:
            m.clear_n.clear()
            return
        released = self._release_behind(m, keep_trip=False)
        if released:
            m.trip = None
            self._event("INFO", "ZONES_RETURNED", f"gives back {released} it did not reach, "
                        f"keeps {list(self.grants.held_by(m.name))}", m.name)

    def _counted(self, m, key: str, clear: bool) -> bool:
        if not clear:
            m.clear_n.pop(key, None)
            return False
        m.clear_n[key] = m.clear_n.get(key, 0) + 1
        if m.clear_n[key] < self.params.release_confirm_samples:
            return False
        del m.clear_n[key]
        return True

    def _note_battery(self, m, battery) -> None:
        m.battery = battery
        p = self.params
        if p.battery_low_pct <= 0 or battery <= 0:
            return
        if not m.charge and battery <= p.battery_low_pct:
            m.charge = True
            self._event("WARN", "ROBOT_BATTERY_LOW", f"{battery:.0f}%, takes no new call",
                        m.name)
        elif m.charge and battery >= p.battery_charged_pct:
            m.charge = False
            self._event("INFO", "ROBOT_CHARGED", f"{battery:.0f}%", m.name)
