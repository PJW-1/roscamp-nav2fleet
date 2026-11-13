# ---- 시작 시 로봇을 지도에 배치 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import SHARED
from fleet_core.traffic.mission import (
    ERROR,
    IDLE,
    NOT_PLACEABLE,
    OUT_OF_SERVICE,
    PARKED,
    PLACEMENT_CONFLICT,
)


class Placement:
    # ---- 노드와 구역을 잡아 로봇 배치 ----
    def _place(self, m) -> None:
        if m.robot_state in NOT_PLACEABLE:
            return
        site = self.site
        node = self._node_under(m)
        zone = site.nodes[node].zone if node else site.zone_at(m.x, m.y)
        if zone is None or (node is None and site.zones[zone].kind != SHARED):
            self._placement_refused(m, f"no node near ({m.x:.2f}, {m.y:.2f})")
            return
        other = self._overlapped(m)
        if other:
            self._placement_refused(m, f"reports the spot where {other} stands")
            return
        if not self.grants.force(m.name, [zone], [node] if node else []):
            self._placement_refused(m, f"{zone} or {node} is taken")
            return
        if m.fault_reason == PLACEMENT_CONFLICT:
            m.fault = m.fault_reason = ""
        m.placed = True
        inside = site.nodes_in(zone)
        free = [name for name in inside if self._free(m, name)]
        m.node = (node or site.nearest_node(m.x, m.y, free)
                  or site.nearest_node(m.x, m.y, inside) or "")
        m.phase = PARKED if node in site.parking else IDLE
        self._joining.discard(m.name)

    # ---- 노드 빈 여부와 겹치는 로봇 찾기 ----
    def _free(self, m, node: str) -> bool:
        return self.grants.node_holder(node) in ("", m.name)

    def _overlapped(self, m) -> str:
        radius = self.params.footprint_radius_m
        for other in self._robots.values():
            if (other is not m and other.placed and other.fault != OUT_OF_SERVICE
                    and math.dist((m.x, m.y), (other.x, other.y)) < radius):
                return other.name
        return ""

    # ---- 로봇 아래 노드 찾기와 배치 거절 처리 ----
    def _node_under(self, m):
        site, tolerance = self.site, self.params.arrival_tolerance_m
        node = site.nearest_node(m.x, m.y, radius=tolerance)
        if node is None or self._free(m, node) or self._gap(m, node) <= tolerance / 2:
            return node
        here = site.zone_at(m.x, m.y)
        free = [name for name in site.nodes_in(here) if self._free(m, name)]
        return site.nearest_node(m.x, m.y, free, radius=tolerance)

    def _placement_refused(self, m, why: str) -> None:
        if m.fault_reason != PLACEMENT_CONFLICT:
            m.fault, m.fault_reason = ERROR, PLACEMENT_CONFLICT
            self._event("ERROR", "PLACEMENT_CONFLICT", why, m.name)
