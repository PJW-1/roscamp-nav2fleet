# ---- 우회 경로 종류 판정과 DETOUR 이벤트 ----
from __future__ import annotations

from fleet_core.fleet_map import LANE, RETREAT, SHARED

KEEPOUT, CORRIDOR, DESTINATION, IN_ZONE = "keepout", "corridor", "destination", "in_zone"


# ---- 이전 경로와 새 경로 비교해 우회 종류 판정 ----
def detour_kind(site, old_goal: str, old_zones, trip) -> str:
    if trip.goal != old_goal:
        return DESTINATION
    new = {zone for link in trip.links if link.a or link.via != RETREAT for zone in link.zones}
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
        trip = m.trip
        m.avoid.detour = (trip.goal, frozenset(z for link in trip.links[trip.done:]
                                               for z in link.zones))

    def note(self, m, trip) -> None:
        if m.avoid.detour is None:
            return
        goal, zones = m.avoid.detour
        m.avoid.detour = None
        self.emit("INFO", "DETOUR", f"{detour_kind(self.site, goal, zones, trip)}: "
                  f"{'->'.join(trip.route())}", m.name)
