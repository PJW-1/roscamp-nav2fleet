"""어디로, 어느 길로 갈지 고르고 그 길의 다음 구간을 한꺼번에 잡는다(지시서 2-2의 1, 2, 5번).

- 길은 지점 그래프에서 가장 빠른 것: 길이 ÷ 속도 + 예상 대기.
- 도크가 비어 있고 먼저 온 차례이면 도크로, 아니면 그 스테이션의 대기 자리로, 그것도 없고
  지금 자리에 서 있으면 안 되면 주차 자리로 간다.
- 떠날 수 있는 길(다음에 설 수 있는 지점까지의 구역과 지점을 모두 잡은 길)만 돌려준다.
"""
from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK, LANE, SHARED, route
from fleet_core.traffic.mission import TO_OUTBOUND, TO_WORK

from .base import HERE
from .lane_policy import Want
from .trips import Trip, back_link, free_link, left_s, needs, speed, stretch_end

TRANSIT_WAIT_S = 6.0  # 찬 갈림목이나 좁은 통로가 비기를 기다리는 예상 시간(길 비용, 가정)


class Goals:
    # ---- 길 ----

    def _cost(self, m):
        """이 로봇에게 연결 하나가 드는 시간(s). 막힘으로 표시한 연결은 쓰지 않는다."""
        now, p, site, grants = self._clock(), self.params, self.site, self.grants

        def cost(link) -> float:
            if self._blocked.get((link.a, link.b), 0.0) > now:
                return math.inf
            seconds = link.length / speed(p, link.via)
            for zone in link.zones:
                others = [name for name in grants.holders(zone) if name != m.name]
                if not others or m.name in grants.holders(zone):
                    continue
                spec = site.zones[zone]
                if spec.kind == SHARED:
                    seconds += TRANSIT_WAIT_S if len(others) >= spec.capacity else 0.0
                elif spec.kind == LANE:
                    same = grants.lane_direction(zone) == link.a and len(others) < spec.capacity
                    seconds += p.lane_follow_gap_s if same else link.length / p.follow_speed_mps
                elif site.nodes[link.b].kind == AT_DOCK and site.nodes[link.b].zone == zone:
                    seconds += p.dock_wait_estimate_s
                else:
                    seconds += TRANSIT_WAIT_S
            return seconds
        return cost

    def _way(self, m, goal: str):
        """지금 자리에서 goal까지의 연결들. 이미 거기면 [], 길이 없으면 None."""
        site, cost = self.site, self._cost(m)
        if m.retreat_to:  # 막혀서 물러나는 중: 지나온 지점까지 후진한 뒤 거기서 간다
            back = self._back_link(m)
            rest = route(site, m.retreat_to, goal, cost)
            return None if rest is None or back is None else [back, *rest]
        if self._on_node(m):
            return route(site, m.node, goal, cost)
        zone = site.zone_at(m.x, m.y)
        if zone is None or site.zones[zone].kind != SHARED:
            return None  # 고정 구간 한가운데: 가던 길을 잇거나 물러난다(blocked.py)
        best, best_cost = None, math.inf
        now = self._clock()
        for name, node in site.nodes.items():
            if node.zone != zone or self._blocked.get((HERE + m.name, name), 0.0) > now:
                continue
            first = free_link(site, m.x, m.y, name)
            rest = route(site, name, goal, cost)
            if rest is None:
                continue
            total = (first.length / self.params.nav_speed_mps
                     + sum(cost(link) for link in rest))
            if total < best_cost:
                best, best_cost = [first, *rest], total
        return best

    def _back_link(self, m):
        """물러날 지점까지의 후진 연결. 지금 구역부터 그 지점의 구역까지를 지난다."""
        site = self.site
        here = site.zone_at(m.x, m.y)
        target = site.nodes[m.retreat_to].zone
        zones = [zone for zone in (here, target) if zone is not None]
        return back_link(site, m.x, m.y, m.retreat_to, dict.fromkeys(zones))

    # ---- 목적지 ----

    def _choose(self, m):
        """단계에 맞는 목적지로 지금 떠날 수 있는 길(Trip). 없으면 None이고 그 자리에서 기다린다."""
        first_want = None
        for goal, queued in self._candidates(m):
            links = self._way(m, goal)
            if not links:
                continue
            trip = Trip(goal, links)
            if queued and self.params.use_order_model:
                trip = self._through(m, trip)  # 대기 자리를 거쳐 도크까지. 도크 쪽은 horizon이다
            want = self._take(m, trip)
            if want is None:
                if queued and not m.queued:
                    m.queued = True
                    self._event("INFO", "DOCK_QUEUED", f"{m.dest} is busy, waits at {goal}",
                                m.name)
                return trip
            first_want = first_want or want
        if first_want is not None:
            self._wants[m.name] = first_want
            self._note_wait(m, first_want)
        elif self._blocked and m.wait_since is None:
            m.wait_since = self._clock()  # 막힘 표시 때문에 갈 길이 없다. 표시가 풀리기를 기다린다
            m.wait_for = ("BLOCKED",)
        return None

    def _candidates(self, m):
        """(목표 지점, 대기 자리인가)를 좋은 순서로."""
        site = self.site
        if m.phase in (TO_WORK, TO_OUTBOUND):
            station = site.stations[m.dest]
            if self._is_next(m, station.id) and self._dock_free(m, station.dock):
                yield station.dock, False
                if self._may_stay(m):
                    return  # 도크는 비었고 가는 길만 못 잡았다. 여기서 기다린다
            elif m.node in station.queue and self._on_node(m):
                self._stay(m, station)
                return
            for spot in station.queue:
                if self.grants.node_holder(spot) in ("", m.name):
                    yield spot, True
            if self._may_stay(m):
                self._stay(m, station)
                return
        cost = self._cost(m)
        free = [spot for spot in site.parking if self.grants.node_holder(spot) in ("", m.name)]

        def price(spot) -> float:
            links = self._way(m, spot)
            return math.inf if links is None else sum(cost(link) for link in links)
        for spot in sorted(free, key=lambda spot: (price(spot), spot)):
            yield spot, False

    def _stay(self, m, station) -> None:
        """도크가 찼거나 차례가 아니어서 지금 자리(대기 자리나 주차)에서 기다린다."""
        if m.wait_since is None:
            m.wait_since = self._clock()
        m.wait_for = (self.site.nodes[station.dock].zone,)
        if not m.queued:
            m.queued = True
            self._event("INFO", "DOCK_QUEUED", f"{station.id} is busy, waits at {m.node}", m.name)

    def _may_stay(self, m) -> bool:
        """지금 자리에 서서 기다려도 되는가: 대기 자리나 주차에 서 있을 때만이다."""
        return (self._on_node(m) and self.site.can_wait(m.node)
                and self.site.nodes[m.node].kind != AT_DOCK and not m.retreat_to)

    def _is_next(self, m, station_id: str) -> bool:
        """그 스테이션을 바라는 로봇 가운데 먼저 온 차례인가. 같은 때 왔으면 가까운 쪽이다."""
        mine = (m.asked_at or 0.0, m.asked_gap, m.name)
        for other in self._robots.values():
            if (other is not m and other.available and other.dest == station_id
                    and other.phase in (TO_WORK, TO_OUTBOUND)
                    and (other.asked_at or 0.0, other.asked_gap, other.name) < mine):
                return False
        return True

    def _dock_free(self, m, dock: str) -> bool:
        zone = self.site.nodes[dock].zone
        return (all(name == m.name for name in self.grants.holders(zone))
                and self.grants.node_holder(dock) in ("", m.name))

    # ---- 구간 잡기 ----

    def _take(self, m, trip):
        """trip의 다음 구간을 한꺼번에 잡는다. 잡았으면 None, 못 잡았으면 바라는 것(Want)."""
        site, grants, now = self.site, self.grants, self._clock()
        start = trip.granted
        end = stretch_end(site, trip.links, start)
        zones, nodes, lanes = needs(trip.links[start:end])
        eta = {}
        for lane in lanes:
            index = next(i for i in range(start, end) if trip.links[i].lane == lane)
            eta[lane] = left_s(self.params, trip.links[trip.done:index], m.x, m.y)
        since = now if m.wait_since is None else m.wait_since
        want = Want(tuple(zones), tuple(nodes), lanes, since, eta)
        entered = [lane for lane in lanes if m.name not in grants.holders(lane)]
        for lane in entered:
            if self.lanes.refusal(m.name, lane, want, now, self._rivals(m, lane, lanes[lane])):
                return want
        empty = [lane for lane in entered if not grants.holders(lane)]
        if not grants.take(m.name, zones, nodes, lanes):
            return want
        trip.granted = end
        self._stop_wanting(m)
        for lane in empty:
            self._lane_set(m, lane, want, now)
        self._event("INFO", "ZONE_GRANTED",
                    f"{'+'.join(zones)} to {trip.links[end - 1].b}", m.name)
        return None

    def _rivals(self, m, lane: str, direction: str) -> dict:
        """같은 한 줄 구역을 반대 방향으로 바라고, 그 구역만 비면 떠날 수 있는 로봇들."""
        rivals = {}
        for name, want in self._wants.items():
            if name == m.name or want.lanes.get(lane) in (None, direction):
                continue
            blocked = self.grants.blockers(name, want.zones, want.nodes, want.lanes)
            if set(blocked) <= {lane}:
                rivals[name] = want
        return rivals

    def _lane_set(self, m, lane: str, want: Want, now: float) -> None:
        direction = want.lanes[lane]
        changed = self.lanes.note(lane, direction)
        if changed and self.lanes.handed_over(want, now):
            self._event("INFO", "LANE_DIRECTION_HANDOVER",
                        f"{lane} from {direction} after waiting {now - want.since:.0f} s", m.name)
        self._event("INFO", "LANE_DIRECTION_SET",
                    f"{lane} from {direction}{' (changed)' if changed else ''}", m.name)

    def _note_wait(self, m, want: Want) -> None:
        """구간을 못 잡아 서 있다. 시작할 때 한 번, 오래되면 한 번 더 알린다."""
        now = self._clock()
        blockers = self.grants.blockers(m.name, want.zones, want.nodes, want.lanes)
        m.wait_for = tuple(blockers) or tuple(want.lanes)
        if m.wait_since is None:
            m.wait_since = now
            self._event("INFO", "ZONE_WAIT", f"waits for {'+'.join(m.wait_for)}", m.name)
        elif not m.wait_warned and now - m.wait_since >= self.params.zone_wait_warn_s:
            m.wait_warned = True
            self._event("WARN", "ZONE_WAIT",
                        f"has waited {now - m.wait_since:.0f} s for {'+'.join(m.wait_for)}",
                        m.name)

