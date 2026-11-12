"""로봇을 지도에 올리기: 선 지점 고르기, 겹친 보고, 거절."""
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
    def _place(self, m) -> None:
        """첫 보고(또는 다시 투입된 뒤의 보고)로 로봇을 지도에 올린다."""
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
        # 지점 없이 오른 로봇은 그 구역의 가까운 지점 이름을 달아 둔다. 빈 지점이 없으면 남이 쥔
        # 지점의 이름이다: 가까워도 그 위에 선 것으로 보지 않는다(_on_node)
        inside = site.nodes_in(zone)
        free = [name for name in inside if self._free(m, name)]
        m.node = (node or site.nearest_node(m.x, m.y, free)
                  or site.nearest_node(m.x, m.y, inside) or "")
        m.phase = PARKED if node in site.parking else IDLE
        self._joining.discard(m.name)

    def _free(self, m, node: str) -> bool:
        return self.grants.node_holder(node) in ("", m.name)

    def _overlapped(self, m) -> str:
        """보고 위치가 이미 오른 다른 로봇의 보고 위치와 한 몸 안(발자국 반지름)에서 겹치면 그
        로봇의 이름. 둘 중 하나는 틀린 보고이므로 나중 것을 올리지 않는다. 운영자가 치운 로봇의
        마지막 위치는 보지 않는다."""
        radius = self.params.footprint_radius_m
        for other in self._robots.values():
            if (other is not m and other.placed and other.fault != OUT_OF_SERVICE
                    and math.dist((m.x, m.y), (other.x, other.y)) < radius):
                return other.name
        return ""

    def _node_under(self, m):
        """보고 위치에서 도착 허용 거리 안의 가장 가까운 지점. 없으면 None.

        그 지점을 다른 로봇이 쥐고 있어도 이 로봇이 지점 바로 위(도착 허용 거리의 절반 안)가
        아니면 그 지점 옆에 선 것으로 본다: 선 구역의 다른 빈 지점이 허용 거리 안에 있으면 그
        지점이고, 없으면 지점 없이 넓은 구역이다. 지점에는 한 대만 서지만 그 옆에는 설 수 있고,
        보고 하나가 0.15 m 안으로 들어왔다고 자리가 겹친 것은 아니다. 다른 구역의 지점으로는
        옮기지 않는다: 그 구역만 쥐고, 서 있는 구역은 쥐지 않게 된다.
        """
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
