# ---- 우회 경로 종류 판정과 DETOUR 이벤트 ----
from __future__ import annotations

from fleet_core.fleet_map import LANE, RETREAT, SHARED

KEEPOUT, CORRIDOR, DESTINATION, IN_ZONE = "keepout", "corridor", "destination", "in_zone"


# ---- 이전 경로와 새 경로 비교해 우회 종류 판정 ----
def detour_kind(site, old_goal: str, old_zones, route) -> str:
    if route.goal != old_goal:
        return DESTINATION
    new = {zone for edge in route.edges if edge.a or edge.mode != RETREAT for zone in edge.zones}
    old = set(old_zones)

    def of(zones, test):
        return {zone for zone in zones if test(site.zones[zone])}

    def lane(spec):
        return spec.kind == LANE

    def corridor(spec):
        return spec.kind == SHARED and not site.nodes_in(spec.id)

    if of(new, lane) != of(old, lane):
        return KEEPOUT
    if of(new, corridor) - of(old, corridor):
        return CORRIDOR
    return IN_ZONE


# ---- 남은 경로 기억과 우회 알림 ----
class DetourNotes:
    def __init__(self, site, emit):
        self.site = site
        self.emit = emit

    def remember(self, m) -> None:
        route = m.route
        m.avoid.detour = (route.goal, frozenset(z for edge in route.edges[route.done:]
                                               for z in edge.zones))

    def note(self, m, route) -> None:
        if m.avoid.detour is None:
            return
        goal, zones = m.avoid.detour
        m.avoid.detour = None
        self.emit("INFO", "DETOUR", f"{detour_kind(self.site, goal, zones, route)}: "
                  f"{'->'.join(route.nodes_left())}", m.name)
