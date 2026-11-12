"""어느 길로 갈지 고르고 그 길의 다음 구간을 한꺼번에 잡는다(지시서 2-2의 1, 2번).

- 길은 지점 그래프에서 가장 빠른 것: 길이 ÷ 속도 + 예상 대기.
- 목적지 후보와 도크 차례는 turns.py가 낸다.
- 떠날 수 있는 길(다음에 설 수 있는 지점까지의 구역과 지점을 모두 잡은 길)만 돌려준다.
"""
from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK, LANE, NAV_GOAL, SHARED, route
from fleet_core.traffic.mission import TO_OUTBOUND, TO_PARK, TO_WORK

from .base import HERE
from .lane_policy import Want
from .trips import Trip, back_link, free_link, left_s, needs, speed, stretch_end

TRANSIT_WAIT_S = 6.0  # 찬 갈림목이나 좁은 통로가 비기를 기다리는 예상 시간(길 비용, 가정)
STANDING_WAIT_S = 300.0  # 스스로 비키지 않는 로봇이 선 지점을 거치는 길에 치는 시간(길 비용, 가정)


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
                holders = grants.holders(zone)
                others = [name for name in holders if name != m.name]
                if not others or m.name in holders:
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
            return seconds + self._node_wait(m, link.b)
        return cost

    def _node_wait(self, m, node: str) -> float:
        """다른 로봇이 goal 없이 서서 쥐고 있는 지점을 거치려면 기다릴 예상 시간(s).

        비었거나, 제가 쥐었거나, 지나가는 로봇이 쥔 지점은 0이다. 차례를 기다리며 선 로봇의 지점은
        찬 도크만큼(dock_wait_estimate_s) 쳐서 다른 길이 있으면 그리로 간다. 할 일 없이 선 로봇과
        고장 난 로봇은 스스로 비키지 않으므로 STANDING_WAIT_S를 친다: 다른 길이 그만큼 더 걸려도
        그리로 가고, 다른 길이 없을 때만 그 지점을 기다린다(누구 때문에 서 있는지가 보인다).
        도크 지점은 도크 차례 규칙이 따로 본다.
        """
        holder = self._robots.get(self.grants.node_holder(node))
        if (holder is None or holder is m or holder.goal is not None
                or self.site.nodes[node].kind == AT_DOCK):
            return 0.0
        if not holder.available or holder.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK):
            return STANDING_WAIT_S
        return self.params.dock_wait_estimate_s

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
        for name, zones in self._reach(zone).items():
            if self._blocked.get((HERE + m.name, name), 0.0) > now:
                continue
            first = free_link(site, m.x, m.y, name, zones)
            rest = route(site, name, goal, cost)
            if rest is None:
                continue
            total = (first.length / self.params.nav_speed_mps + self._node_wait(m, name)
                     + sum(cost(link) for link in rest))
            if total < best_cost:
                best, best_cost = [first, *rest], total
        return best

    def _reach(self, zone: str) -> dict:
        """넓은 구역 안의 아무 자리에서 곧장 갈 수 있는 지점과 거기까지 지나는 구역.

        그 구역의 지점들이다. 지점이 없는 구역(통로)이면 그 구역을 지나는 Nav2 goal 연결의 끝
        지점들이고, 지나는 구역은 그 연결에서 지금 구역부터 끝까지다.
        """
        memo = self.__dict__.setdefault("_reach_memo", {})  # 지도는 바뀌지 않으니 구역마다 한 번
        if zone not in memo:
            site = self.site
            inside = {name: (zone,) for name, node in site.nodes.items() if node.zone == zone}
            if inside:
                memo[zone] = inside
            else:
                reach = {}
                for link in site.links():
                    if link.via == NAV_GOAL and zone in link.zones:
                        reach.setdefault(link.b, link.zones[link.zones.index(zone):])
                memo[zone] = reach
        return memo[zone]

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
                elif not queued:
                    self._take_turn(m, trip)
                return trip
            first_want = first_want or want
        if first_want is not None:
            self._wants[m.name] = first_want
            self._note_wait(m, first_want)
        elif self._blocked:
            if m.wait_since is None:  # 막힘 표시 때문에 갈 길이 없다. 표시가 풀리기를 기다린다
                m.wait_since = self._clock()
                m.wait_for = ("BLOCKED",)
        elif m.wait_for == ("BLOCKED",):
            # 표시는 풀렸는데도 길이 없다. 기다리는 것이 아니다: ROBOT_IDLE이 알리게 한다
            self._stop_wanting(m)
        return None

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

