"""막힌 뒤의 우회 종류: 어떤 우회인지 가르고 DETOUR로 남긴다."""
from __future__ import annotations

from fleet_core.fleet_map import LANE, RETREAT, SHARED

KEEPOUT, CORRIDOR, DESTINATION, IN_ZONE = "keepout", "corridor", "destination", "in_zone"


def detour_kind(site, old_goal: str, old_zones, trip) -> str:
    """막힌 뒤 새로 잡은 길이 어떤 우회인가(추가 지시서 3의 4-4). Nav2가 모르는 선택지부터 본다.

    destination: 목적지를 바꿨다(다른 대기 자리, 주차 자리). keepout: keepout 너머의 길로 바꿨다
    (마당이 막혀 급행으로, 급행이 막혀 마당으로: 지나는 한 줄 구역이 달라졌다). corridor: 지점 없는
    다른 통로로 간다(D, C). in_zone: 같은 넓은 구역 안에서 다른 지점을 거쳐 갈 뿐이다.
    """
    if trip.goal != old_goal:
        return DESTINATION
    # 들어온 입구까지 물러나는 후진(지도에 없는 연결)은 우회가 아니다
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


class Detours:
    def _remember_way(self, m) -> None:
        """막혀 길을 버리기 전에 가던 목적지와 지나던 구역을 적어 둔다(우회 종류를 셀 때 쓴다)."""
        trip = m.trip
        m.detour = (trip.goal, frozenset(z for link in trip.links[trip.done:] for z in link.zones))

    def _note_detour(self, m, trip) -> None:
        """막힌 뒤 처음 잡은 길이면 그 우회의 종류를 DETOUR로 남긴다."""
        if m.detour is None:
            return
        goal, zones = m.detour
        m.detour = None
        self._event("INFO", "DETOUR", f"{detour_kind(self.site, goal, zones, trip)}: "
                    f"{'->'.join(trip.route())}", m.name)
