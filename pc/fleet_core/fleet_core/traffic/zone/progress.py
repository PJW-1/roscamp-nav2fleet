"""로봇의 상태 보고: 지도에 올리기, 지난 지점 따라가기, 뒤에 남은 구역과 지점 놓기."""
from __future__ import annotations

from fleet_core.fleet_map import NAV_GOAL, SHARED
from fleet_core.traffic.mission import (
    COMM_LOST,
    DOCK,
    ERROR,
    IDLE,
    LOCALIZING,
    NOT_PLACEABLE,
    OUT_OF_SERVICE,
    PARKED,
    PLACEMENT_CONFLICT,
)

from .trips import passed

NODE_KEY = "node:"  # 놓기 셈(clear_n)에서 지점을 구역과 구별하는 머리말


class Progress:
    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0) -> None:
        m = self._robots.get(robot)
        if m is None:
            return
        now = self._clock()
        m.last_seen = now
        m.robot_state = state
        self._note_battery(m, battery)
        if state == LOCALIZING:
            return  # 위치를 아직 못 잡았다. 자세를 믿지 않는다
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

    def _place(self, m) -> None:
        """첫 보고(또는 다시 투입된 뒤의 보고)로 로봇을 지도에 올린다."""
        if m.robot_state in NOT_PLACEABLE:
            return
        site = self.site
        node = site.nearest_node(m.x, m.y, radius=self.params.arrival_tolerance_m)
        zone = site.nodes[node].zone if node else site.zone_at(m.x, m.y)
        if zone is None or (node is None and site.zones[zone].kind != SHARED):
            self._placement_refused(m, f"no node near ({m.x:.2f}, {m.y:.2f})")
            return
        if not self.grants.force(m.name, [zone], [node] if node else []):
            self._placement_refused(m, f"{zone} or {node} is taken")
            return
        if m.fault_reason == PLACEMENT_CONFLICT:
            m.fault = m.fault_reason = ""
        m.placed = True
        m.node = node or site.nearest_node(
            m.x, m.y, [name for name, spec in site.nodes.items() if spec.zone == zone]) or ""
        m.phase = PARKED if node in site.parking else IDLE
        self._joining.discard(m.name)

    def _placement_refused(self, m, why: str) -> None:
        if m.fault_reason != PLACEMENT_CONFLICT:
            m.fault, m.fault_reason = ERROR, PLACEMENT_CONFLICT
            self._event("ERROR", "PLACEMENT_CONFLICT", why, m.name)

    def _track(self, m) -> None:
        """고정 경로의 꺾은선을 달리는 동안 지난 지점을 따라간다. 끝 지점은 결과로만 닿는다."""
        trip, goal = m.trip, m.goal
        if trip is None or goal is None or goal.kind == DOCK:
            return
        while (trip.done < goal.links - 1 and trip.links[trip.done].via != NAV_GOAL
               and passed(trip.links[trip.done], m.x, m.y)):
            m.node = trip.links[trip.done].b
            trip.done += 1

    def _still_needed(self, m) -> tuple[set, set]:
        """가는 길에 아직 필요한 (구역, 지점). 지금 구역보다 뒤의 구역은 들어가지 않는다."""
        trip = m.trip
        zones, nodes = set(), set()
        if trip is None:
            return zones, nodes
        links = trip.links[trip.done:trip.granted]
        if links:
            crossed = links[0].zones
            here = self.site.zone_at(m.x, m.y)
            zones.update(crossed[crossed.index(here):] if here in crossed else crossed)
            for link in links[1:]:
                zones.update(link.zones)
            nodes.update(link.b for link in links)
        return zones, nodes

    def _release_behind(self, m) -> None:
        """뒤에 남은 구역과 지점을 놓는다: 벗어난 보고가 release_confirm_samples번 이어질 때."""
        p, site, grants = self.params, self.site, self.grants
        if self._clock() < m.settle_until:
            return  # 방금 goal을 거뒀다. 취소가 로봇에 닿기 전일 수 있다
        held = grants.held_by(m.name)
        if all(site.distance_to_zone(m.x, m.y, zone) > p.outside_zone_m for zone in held):
            return  # 보고 위치가 쥔 구역 밖이다(위치가 튀었다). 이 보고로는 아무것도 놓지 않는다
        zones, nodes = self._still_needed(m)
        for zone in held:
            clear = (zone not in zones
                     and site.distance_to_zone(m.x, m.y, zone) >= p.zone_clear_m - 1e-9)
            if self._counted(m, zone, clear):
                grants.release_zone(m.name, zone)
        for node in grants.nodes_of(m.name):
            clear = node not in nodes and self._gap(m, node) >= p.node_clear_m
            if self._counted(m, NODE_KEY + node, clear):
                grants.release_node(m.name, node)

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
            return  # 꺼 두었거나 모르는 값
        if not m.charge and battery <= p.battery_low_pct:
            m.charge = True
            self._event("WARN", "ROBOT_BATTERY_LOW", f"{battery:.0f}%, takes no new call",
                        m.name)
        elif m.charge and battery >= p.battery_charged_pct:
            m.charge = False
            self._event("INFO", "ROBOT_CHARGED", f"{battery:.0f}%", m.name)
