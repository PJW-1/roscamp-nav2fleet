"""막혀 물러나기: 고정 구간에서 막힌 로봇이 물러날 곳, 뒤의 로봇 기다리기, 다른 로봇에게 비키기."""
from __future__ import annotations

from fleet_core.fleet_map import FOLLOW_PATH, PASS, SHARED
from fleet_core.fleet_map.geometry import point_segment_distance
from fleet_core.traffic.mission import BLOCKED


def yard_retreat(site, link) -> tuple[str, str]:
    """고정 구간에서 막혀 물러날 (지점, 꺾는 지점).

    지나온 지점(link.a)이 갈림목(pass)이고 거기서 넓은 구역의 지점으로 내려가는 고정 연결이 있으면
    갈림목에서 꺾어 그 지점까지 물러난다(2-1 지도: 갈림목은 도크에서 0.17 m라 도크에 로봇이 있으면
    거기에 설 수 없다). 이미 그 지점으로 내려가던 중이거나 그런 연결이 없으면 (link.a, "")이다.
    """
    a = link.a
    if not a or site.nodes[a].kind != PASS:
        return a, ""
    for down in site.links():
        if (down.a == a and down.via == FOLLOW_PATH and down.b != link.b
                and site.zones[site.nodes[down.b].zone].kind == SHARED):
            return down.b, a
    return a, ""


class YardRetreat:
    def _waits_behind(self, m) -> bool:
        """후진이 뒤의 무엇(OBSTACLE_BEHIND) 때문에 끝났다. blocked_timeout_s까지는 실패로 세지
        않고 move_retry_wait_s마다 다시 낸다(도크 앞에 잠깐 선 사람). 기다리면 True."""
        now = self._clock()
        if m.behind_since is None:
            m.behind_since = now
            self._event("INFO", "RETREAT_WAIT", "something is behind it, backs out when it is "
                        "gone", m.name)
        if now - m.behind_since >= self.params.blocked_timeout_s:
            return False  # 다 기다렸다. 빠져나갈 때까지 예전처럼 시도로 센다
        m.retry_at = now + self.params.move_retry_wait_s
        return True

    def _set_retreat(self, m, link, now: float) -> None:
        """고정 구간에서 막힌 m이 물러날 곳을 정한다(yard_retreat)."""
        m.retreat_to, m.retreat_via = yard_retreat(self.site, link)
        m.retreat_since = now

    def _behind_on_the_bent_way(self, m, target: str, via: str) -> str:
        """갈림목에서 꺾어 마당 지점까지 물러날 길 위의 다른 로봇. 없으면 빈 문자열.

        도크에 선 로봇도 센다. 2-1 지도에서 갈림목은 도크에서 0.17 m라 몸이 닿을 만큼 가깝다.
        그 로봇이 도크에서 나가는 길(갈림목, 마당)은 급행에서 기다리는 이 로봇이 쥐지 않으므로
        먼저 나간 뒤 물러난다. 끝내 나가지 않으면 blocked_timeout_s 뒤 운영자에게 넘긴다
        (blocked.py).
        """
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
        """갈림목에서 꺾어 물러날 길 위에 다른 로봇이 있어 아직 그 길을 잡지 않는다.

        표시한 뒤에 도크에 들어온 로봇도 본다. 잡으면 도크의 로봇은 갈림목을 못 지나 나가지 못하고
        물러나는 로봇은 갈림목에서 그 로봇에 닿아 둘이 서로 기다린다. 기다림은 _watch_retreat_wait가
        blocked_timeout_s로 끝낸다. 누가 막는지 바뀔 때만 RETREAT_WAIT를 낸다.
        """
        behind = (self._behind_on_the_bent_way(m, m.retreat_to, m.retreat_via)
                  if m.retreat_via else "")
        if behind and behind != m.retreat_behind:
            self._event("INFO", "RETREAT_WAIT", f"{behind} is behind it, backs out after it",
                        m.name)
        m.retreat_behind = behind
        return bool(behind)

    def _watch_retreat_wait(self, m, now: float) -> None:
        """물러날 길의 구역을 blocked_timeout_s 동안 못 잡았으면 운영자에게 넘긴다."""
        if (not m.retreat_to or m.goal is not None or m.retreat_since is None
                or now - m.retreat_since < self.params.blocked_timeout_s):
            return
        self._event("ERROR", "GOAL_FAILED", f"could not back out to {m.retreat_to} for "
                    f"{now - m.retreat_since:.0f} s", m.name)
        m.retreat_since = None
        self._fail(m, BLOCKED)

    def _yield_to_robot(self, m, other, now: float) -> None:
        """막은 것이 다른 로봇이다. 막힘으로 표시하지 않고 비킨다(HALL_YIELD).

        고정 구간 안이면 m이 들어온 입구로 물러난다(뒤의 로봇이 먼저 나간 뒤다: blocked.py가
        이미 보았다). 넓은 구역이면 goal 없이 선 그 로봇을 주차 자리로 보내고, 그럴 수 없으면 m이
        비켜 선다(backoff.py의 _stand_aside).
        """
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
            # 막힌 로봇의 다음 단계도 그 로봇 이름으로 남긴다. 그래도 또 막히면 예전처럼 표시한다
            self._event("INFO", "HALL_YIELD", f"waits for {other.name} to clear the way, "
                        "nothing marked", m.name)
            return
        self._cancel_goal(m)
        self._stand_aside(m, other, now)
