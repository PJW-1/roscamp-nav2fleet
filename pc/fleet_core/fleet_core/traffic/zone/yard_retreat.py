# ---- 물러나기와 다른 로봇에게 양보 믹스인 ----
from __future__ import annotations

from fleet_core.fleet_map import FOLLOW_PATH, PASS, SHARED
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import BLOCKED


# ---- PASS 노드에서 SHARED 구역 쪽 물러날 지점 찾기 ----
def yard_retreat(site, link) -> tuple[str, str]:
    a = link.a
    if not a or site.nodes[a].kind != PASS:
        return a, ""
    for down in site.links():
        if (down.a == a and down.via == FOLLOW_PATH and down.b != link.b
                and site.zones[site.nodes[down.b].zone].kind == SHARED):
            return down.b, a
    return a, ""


# ---- 뒤가 비기를 기다리기와 물러날 목표 정하기 ----
class YardRetreat:
    def _waits_behind(self, m) -> bool:
        now = self._clock()
        if m.behind_since is None:
            m.behind_since = now
            self._event("INFO", "RETREAT_WAIT", "something is behind it, backs out when it is "
                        "gone", m.name)
        if now - m.behind_since >= self.params.blocked_timeout_s:
            return False
        m.retry_at = now + self.params.move_retry_wait_s
        return True

    def _set_retreat(self, m, link, now: float) -> None:
        m.retreat_to, m.retreat_via = yard_retreat(self.site, link)
        m.retreat_since = now

    # ---- 꺾인 물러나기 길 뒤에 있는 로봇 확인 ----
    def _behind_on_the_bent_way(self, m, target: str, via: str) -> str:
        site = self.site
        here, corner, end = (m.x, m.y), site.node_pose(via), site.node_pose(target)
        way = (corner[0] - here[0], corner[1] - here[1])
        for other in self._robots.values():
            if other is m or not other.placed:
                continue
            spot = (other.x, other.y)
            toward = (spot[0] - here[0]) * way[0] + (spot[1] - here[1]) * way[1] > 0.0
            gap = min(point_segment_distance(spot, here, corner),
                      point_segment_distance(spot, corner, end))
            if toward and gap < self.params.retreat_clear_m:
                return other.name
        return ""

    def _retreat_held_up(self, m) -> bool:
        behind = (self._behind_on_the_bent_way(m, m.retreat_to, m.retreat_via)
                  if m.retreat_via else "")
        if behind and behind != m.retreat_behind:
            self._event("INFO", "RETREAT_WAIT", f"{behind} is behind it, backs out after it",
                        m.name)
        m.retreat_behind = behind
        return bool(behind)

    # ---- 물러나기를 너무 오래 못 하면 실패 처리 ----
    def _watch_retreat_wait(self, m, now: float) -> None:
        if (not m.retreat_to or m.goal is not None or m.retreat_since is None
                or now - m.retreat_since < self.params.blocked_timeout_s):
            return
        self._event("ERROR", "GOAL_FAILED", f"could not back out to {m.retreat_to} for "
                    f"{now - m.retreat_since:.0f} s", m.name)
        m.retreat_since = None
        self._fail(m, BLOCKED)

    # ---- 다른 로봇에게 길 양보 ----
    def _yield_to_robot(self, m, other, now: float) -> None:
        m.blocked_since = None
        m.robot_yield_at = now
        link = m.trip.links[m.trip.done]
        zone = self.site.zone_at(m.x, m.y)
        if link.a and zone is not None and self.site.zones[zone].kind != SHARED:
            self._cancel_goal(m)
            self._set_retreat(m, link, now)
            self._drop_trip(m)
            self._event("INFO", "HALL_YIELD", f"backs out to {m.retreat_to} for {other.name}, "
                        "nothing marked", m.name)
            return
        if (other.available and other.goal is None and self._in_yard(other)
                and self._to_parking(other, "INFO", "HALL_YIELD",
                                     f"out of the way of {m.name}, goes to", clear_of=m)):
            self._event("INFO", "HALL_YIELD", f"waits for {other.name} to clear the way, "
                        "nothing marked", m.name)
            return
        self._cancel_goal(m)
        self._stand_aside(m, other, now)
