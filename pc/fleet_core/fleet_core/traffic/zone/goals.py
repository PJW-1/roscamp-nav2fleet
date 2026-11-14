# ---- 경로 비용, 트립 고르기, 구역 잡기 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK, LANE, NAV_GOAL, SHARED, route
from fleet_core.traffic.mission import TO_OUTBOUND, TO_PARK, TO_WORK

from .base import HERE
from .lane_policy import Want
from .trips import Trip, back_link, free_link, left_s, needs, reach_end, speed, stretch_end

TRANSIT_WAIT_S = 6.0
STANDING_WAIT_S = 300.0


# ---- 링크 비용과 노드 대기 시간 추정 ----
class Goals:
    def _cost(self, m):
        now, p, site, grants = self._clock(), self.params, self.site, self.grants

        def cost(link) -> float:
            if self._blocked.get((link.a, link.b), 0.0) > now:
                return math.inf
            seconds = link.price_m / speed(p, link.via)
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
        holder = self._robots.get(self.grants.node_holder(node))
        if (holder is None or holder is m or holder.goal is not None
                or self.site.nodes[node].kind == AT_DOCK):
            return 0.0
        if not holder.available or holder.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK):
            return STANDING_WAIT_S
        return self.params.dock_wait_estimate_s

    # ---- 현재 위치에서 목적지까지 링크 찾기 ----
    def _way(self, m, goal: str):
        site, cost = self.site, self._cost(m)
        if m.retreat_to:
            if self._retreat_held_up(m):
                return None
            back = self._back_link(m)
            rest = route(site, m.retreat_to, goal, cost)
            return None if rest is None or back is None else [back, *rest]
        if self._on_node(m):
            return route(site, m.node, goal, cost)
        zone = site.zone_at(m.x, m.y)
        if zone is None or site.zones[zone].kind != SHARED:
            return None
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

    # ---- 구역에서 닿는 노드와 물러나기 링크 ----
    def _reach(self, zone: str) -> dict:
        memo = self.__dict__.setdefault("_reach_memo", {})
        if zone not in memo:
            site = self.site
            by_goal = {link.b for link in site.links() if link.via == NAV_GOAL}
            inside = {name: (zone,) for name in site.nodes_in(zone)
                      if name in by_goal or not by_goal}
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
        site = self.site
        here = site.zone_at(m.x, m.y)
        via = site.nodes[m.retreat_via].zone if m.retreat_via else None
        target = site.nodes[m.retreat_to].zone
        zones = [zone for zone in (here, via, target) if zone is not None]
        # 갈림 -> 마당 지점 고정 경로의 꺾는 점을 따른다(곧게 가면 다른 구역 모서리에 붙을 수 있다)
        down = site.link(m.retreat_via, m.retreat_to) if m.retreat_via else None
        bends = down.points[1:-1] if down is not None else ()
        return back_link(site, m.x, m.y, m.retreat_to, dict.fromkeys(zones), m.retreat_via,
                         bends)

    # ---- 후보 중 잡을 수 있는 트립 고르기 ----
    def _choose(self, m):
        first_want = None
        for goal, queued in self._candidates(m):
            links = self._way(m, goal)
            if not links:
                continue
            trip = Trip(goal, links)
            if queued and self.params.use_order_model:
                trip = self._through(m, trip)
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
        if first_want is None and m.retreat_to and not self._retreat_held_up(m):
            back = self._back_link(m)
            trip = None if back is None else Trip(m.retreat_to, [back])
            if trip is not None and self._take(m, trip) is None:
                return trip
        if first_want is not None:
            self._wants[m.name] = first_want
            self._note_wait(m, first_want)
        elif self._blocked:
            if m.wait_since is None:
                m.wait_since = self._clock()
                m.wait_for = ("BLOCKED",)
        elif m.wait_for == ("BLOCKED",):
            self._stop_wanting(m)
        return None

    # ---- 다음 구간의 구역, 노드, 차선 잡기 ----
    def _take(self, m, trip):
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
        far = reach_end(site, trip.links, start)
        if far > end and not grants.blockers(m.name, *needs(trip.links[start:far])):
            end = far
            zones, nodes, lanes = needs(trip.links[start:end])
        if not grants.take(m.name, zones, nodes, lanes):
            return want
        trip.granted = end
        self._stop_wanting(m)
        for lane in empty:
            self._lane_set(m, lane, want, now)
        self._event("INFO", "ZONE_GRANTED",
                    f"{'+'.join(zones)} to {trip.links[end - 1].b}", m.name)
        self._note_detour(m, trip)
        return None

    # ---- 차선 경쟁 로봇 찾기와 방향 기록 ----
    def _rivals(self, m, lane: str, direction: str) -> dict:
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

    # ---- 구역 대기 기록과 경고 ----
    def _note_wait(self, m, want: Want) -> None:
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
