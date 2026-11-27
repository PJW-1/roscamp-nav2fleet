# ---- 로봇 상태 보고 처리와 구역 놓기 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL, RETREAT
from fleet_core.traffic.mission import (
    AT_STATION,
    COMM_LOST,
    DOCK,
    ERROR,
    LOCALIZING,
    NOT_PLACEABLE,
    OUT_OF_SERVICE,
    PARKED,
    SELF_RECOVERING,
)

from .routes import passed

# ---- 노드 키 접두사와 구역 놓는 거리 ----
NODE_KEY = "node:"


def release_gap(p) -> float:
    return p.zone_clear_m or p.footprint_radius_m + p.release_margin_m


# ---- 상태 보고 받아 배치, 복구, 추적으로 나누기 ----
class Progress:
    def on_robot_state(self, robot, x, y, yaw, state, battery=0.0, obstacle=None,
                       local_yield="") -> None:
        m = self._robots.get(robot)
        if m is None:
            return
        now = self._clock()
        m.report.last_seen = now
        m.report.robot_state = state
        m.report.obstacle = obstacle
        m.report.local_yield = local_yield if state == SELF_RECOVERING else ""
        self._note_battery(m, battery)
        if state == LOCALIZING:
            if m.result_lost_at is not None:
                self._past_unknown(m, now)  # 좌표를 믿지 않는 보고라도 기한은 본다
            self._note_localizing(m, now)
            m.health.pose_at = None
            m.clear_n.clear()
            return
        self._localized(m)
        if m.health.comm == COMM_LOST:
            m.health.pose_at = None
        self._pose_watch.watch(m, x, y, yaw, now)
        m.x, m.y, m.yaw = x, y, yaw
        m.heard_since_success = True
        lost = m.result_lost_at is not None and not self._past_unknown(m, now)  # 좌표 반영 뒤 기한을 본다
        if lost and m.health.comm != COMM_LOST:
            self._settle_lost(m, now, state)  # 들은 그 순간 정한다. 통신이 끊겼던 로봇은 복구가 끝낸다
        if m.health.fault == OUT_OF_SERVICE:
            return
        if not m.placed:
            self._place(m)
        elif m.health.comm == COMM_LOST:
            self._begin_recovery(m)
        elif not m.health.fault and not m.health.comm:
            self._track(m)
            if m.result_lost_at is None:  # 결과 모름이면 구역을 쥐고 막힘 처리도 하지 않는다
                self._release_behind(m)
            self._watch_outside(m)
            if m.result_lost_at is None:
                self._watch_blocked(m, now)
        elif m.health.fault == ERROR and not m.health.comm and m.goal is None:
            self._return_unreached(m)

    # ---- 지나간 edge만큼 route 진행 ----
    def _track(self, m) -> None:
        route, goal = m.route, m.goal
        if route is None or goal is None or goal.kind == DOCK:
            return
        while route.done < goal.edges - 1:
            edge = route.edges[route.done]
            if edge.mode == NAV_GOAL or not passed(edge, m.x, m.y):
                break
            # 물러나기는 노드 좌표로 잴 거리 안이면 끝냈다는 답을 기다린다
            if edge.mode == RETREAT and self._gap(m, edge.b) <= self.params.release_margin_m:
                break
            m.node = edge.b
            route.done += 1

    # ---- 아직 필요한 구역과 노드 계산 ----
    def _still_needed(self, m) -> tuple[set, set]:
        route = m.route
        zones, nodes = set(), set()
        if route is None:
            return zones, nodes
        edges = route.edges[route.done:route.granted]
        if edges:
            crossed = edges[0].zones
            here = self.site.zone_at(m.x, m.y)
            if edges[0].mode != NAV_GOAL and here in crossed:
                crossed = crossed[crossed.index(here):]
            zones.update(crossed)
            for edge in edges[1:]:
                zones.update(edge.zones)
            nodes.update(edge.b for edge in edges)
        return zones, nodes

    # ---- 지나온 구역과 노드 놓기 ----
    def _release_behind(self, m, keep_route: bool = True) -> list[str]:
        p, site, grants = self.params, self.site, self.grants
        held = grants.held_by(m.name)
        if (self._clock() < m.settle_until
                or all(site.distance_to_zone(m.x, m.y, zone) > p.outside_zone_m
                       for zone in held)):
            m.clear_n.clear()
            return []
        zones, nodes = self._still_needed(m) if keep_route else (set(), set())
        if m.phase == AT_STATION and m.node in site.nodes and not p.zone_clear_m:
            (x, y), gap = site.node_pose(m.node), p.footprint_radius_m + m.dock_error_m
        elif (m.node in site.nodes and not p.zone_clear_m
              and self._gap(m, m.node) <= p.release_margin_m):
            (x, y), gap = site.node_pose(m.node), release_gap(p)
        else:
            (x, y), gap = (m.x, m.y), release_gap(p)
        released = []
        for zone in held:
            clear = zone not in zones and site.distance_to_zone(x, y, zone) >= gap - 1e-9
            if self._counted(m, zone, clear) and grants.release_zone(m.name, zone):
                released.append(zone)
        parked = m.phase == PARKED and m.node in site.parking and self._on_node(m)
        for node in grants.nodes_of(m.name):
            clear = node not in nodes and ((parked and node != m.node and self._in_bay(m, node))
                                           or self._gap(m, node) >= p.node_clear_m)
            if self._counted(m, NODE_KEY + node, clear):
                grants.release_node(m.name, node)
        return released

    def _in_bay(self, m, node: str) -> bool:
        apart = math.dist(self.site.node_pose(m.node), self.site.node_pose(node))
        return 2 * self._gap(m, m.node) < apart

    # ---- 고장 로봇이 닿지 않은 구역 돌려주기와 연속 확인 횟수 세기 ----
    def _return_unreached(self, m) -> None:
        moved = m.health.fault_xy is None or math.dist(m.health.fault_xy, (m.x, m.y)) > (
            self.params.arrival_tolerance_m)
        if m.report.robot_state in NOT_PLACEABLE or moved or m.health.pose_doubt:
            m.clear_n.clear()
            return
        released = self._release_behind(m, keep_route=False)
        if released:
            m.route = None
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

    # ---- [충전 끝내기](시험): 배터리가 오르지 않아도 다시 일을 받는다 ----
    def end_charge(self, robot) -> bool:
        m = self._robots.get(robot)
        if m is None or not m.report.charge:
            return False
        m.report.charge = False
        self._event("INFO", "ROBOT_CHARGED", f"{m.report.battery:.0f}%, ended by charge_end", m.name)
        return True

    # ---- 배터리 부족과 충전 완료 기록 ----
    def _note_battery(self, m, battery) -> None:
        m.report.battery = battery
        p = self.params
        if p.battery_low_pct <= 0 or battery <= 0:
            return
        if not m.report.charge and battery <= p.battery_low_pct:
            m.report.charge = True
            self._event("WARN", "ROBOT_BATTERY_LOW", f"{battery:.0f}%, takes no new call",
                        m.name)
        elif m.report.charge and battery >= p.battery_charged_pct:
            m.report.charge = False
            self._event("INFO", "ROBOT_CHARGED", f"{battery:.0f}%", m.name)
