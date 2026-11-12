"""로봇의 상태 보고: 지난 지점 따라가기, 뒤에 남은 구역과 지점 놓기."""
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
)

from .trips import passed

NODE_KEY = "node:"  # 놓기 셈(clear_n)에서 지점을 구역과 구별하는 머리말


def release_gap(p) -> float:
    """뒤 구역을 놓을 보고 중심과 구역의 거리: 발자국 반지름 + 여유. 옛 기록은 zone_clear_m."""
    return p.zone_clear_m or p.footprint_radius_m + p.release_margin_m


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
            m.clear_n.clear()  # 벗어난 보고는 이어져야 한다
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
        elif m.fault == ERROR and not m.comm and m.goal is None:
            self._return_unreached(m)

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
            if links[0].via != NAV_GOAL and here in crossed:
                # 고정 경로는 지나온 구역으로 돌아가지 않는다. Nav2 goal은 잡아 준 구역 안에서
                # 어느 길로든 돌아올 수 있으므로 goal이 끝날 때까지 모두 쥔다
                crossed = crossed[crossed.index(here):]
            zones.update(crossed)
            for link in links[1:]:
                zones.update(link.zones)
            nodes.update(link.b for link in links)
        return zones, nodes

    def _release_behind(self, m, keep_trip: bool = True) -> list[str]:
        """뒤에 남은 구역과 지점을 놓는다: 벗어난 보고가 release_confirm_samples번 이어질 때.

        구역은 보고 중심이 release_gap 넘게 벗어나야 벗어난 것이다: 몸 전체가 나간 뒤다. 도킹을
        마친 로봇은 도킹 결과가 자리를 보증하므로 도크 지점에서 발자국 반지름 + 도킹이 보고한
        오차만 본다. 옛 기록(zone_clear_m)은 도킹한 로봇도 그때의 규칙으로 놓는다.

        keep_trip이 거짓이면 가던 길에 잡아 둔 것도 남기지 않는다. 놓은 구역을 돌려준다.
        """
        p, site, grants = self.params, self.site, self.grants
        held = grants.held_by(m.name)
        if (self._clock() < m.settle_until  # 방금 goal을 거뒀다. 취소가 닿기 전일 수 있다
                or all(site.distance_to_zone(m.x, m.y, zone) > p.outside_zone_m
                       for zone in held)):  # 보고 위치가 쥔 구역 밖이다(위치가 튀었다)
            m.clear_n.clear()  # 이 보고로는 아무것도 놓지 않는다. 벗어난 보고는 이어져야 한다
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
        """고장으로 세운 로봇이 계속 보고한다. 서 있는 자리 밖의 구역(가던 길에 잡아 둔 것)은
        돌려준다. 말이 끊긴 로봇의 것은 돌려주지 않는다: 어디 있는지 모른다.

        로봇이 goal 없이 서 있다고 보고하지 않거나(취소가 닿지 않았을 수 있다. 막혀 선 것도
        goal은 살아 있다), 선 자리가 고장 났을 때의 자리와 다르거나, 위치를 못 믿어 세운 적이
        있으면(NOT_AT_TARGET) 돌려주지 않는다.
        """
        moved = m.fault_xy is None or math.dist(m.fault_xy, (m.x, m.y)) > (
            self.params.arrival_tolerance_m)
        if m.robot_state in NOT_PLACEABLE or moved or m.pose_doubt:
            m.clear_n.clear()
            return
        released = self._release_behind(m, keep_trip=False)
        if released:
            m.trip = None  # 가던 길은 버렸다. 다시 투입되면 새로 고른다
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
            return  # 꺼 두었거나 모르는 값
        if not m.charge and battery <= p.battery_low_pct:
            m.charge = True
            self._event("WARN", "ROBOT_BATTERY_LOW", f"{battery:.0f}%, takes no new call",
                        m.name)
        elif m.charge and battery >= p.battery_charged_pct:
            m.charge = False
            self._event("INFO", "ROBOT_CHARGED", f"{battery:.0f}%", m.name)
