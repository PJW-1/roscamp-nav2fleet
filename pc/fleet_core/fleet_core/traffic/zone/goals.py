# ---- 경로 비용, route 고르기, 구역 잡기 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import AT_DOCK, LANE, NAV_GOAL, SHARED, WAIT, Edge, shortest_path
from fleet_core.traffic.mission import TO_OUTBOUND, TO_PARK, TO_WORK

from .base import HERE
from .lane_policy import Want
from .routes import ARRIVE, Route, back_edge, free_edge, needs, segment_end, speed, stretch_end

TRANSIT_WAIT_S = 6.0
STANDING_WAIT_S = 300.0


# ---- edge 비용과 노드 대기 시간 추정 ----
class Goals:
    def _cost(self, m):
        now, p, site, grants = self._clock(), self.params, self.site, self.grants

        def cost(edge) -> float:
            if self._blocked.get((edge.a, edge.b), 0.0) > now:
                return math.inf
            seconds = edge.price_m / speed(p, edge.mode)
            for zone in edge.zones:
                holders = grants.holders(zone)
                others = [name for name in holders if name != m.name]
                if not others or m.name in holders:
                    continue
                spec = site.zones[zone]
                if spec.kind == SHARED:
                    seconds += TRANSIT_WAIT_S if len(others) >= spec.capacity else 0.0
                elif spec.kind == LANE:
                    same = grants.lane_direction(zone) == edge.a and len(others) < spec.capacity
                    seconds += p.lane_follow_gap_s if same else edge.length / p.follow_speed_mps
                elif site.nodes[edge.b].kind == AT_DOCK and site.nodes[edge.b].zone == zone:
                    seconds += p.dock_wait_estimate_s
                else:
                    seconds += TRANSIT_WAIT_S
            return seconds + self._node_wait(m, edge.b)
        return cost

    def _node_wait(self, m, node: str) -> float:
        holder = self._robots.get(self.grants.node_holder(node))
        if (holder is None or holder is m or holder.goal is not None
                or self.site.nodes[node].kind == AT_DOCK):
            return 0.0
        if not holder.available or holder.phase not in (TO_WORK, TO_OUTBOUND, TO_PARK):
            return STANDING_WAIT_S
        return self.params.dock_wait_estimate_s

    # ---- 현재 위치에서 목적지까지 edge 찾기 ----
    def _way(self, m, goal: str):
        site, cost = self.site, self._cost(m)
        if m.avoid.retreat_to:
            if self._retreat_held_up(m):
                return None
            back = self._back_edge(m)
            rest = self._path(m, m.avoid.retreat_to, goal, cost)
            return None if rest is None or back is None else [back, *rest]
        # 중간 지점 전용 노드(via)의 지도 이음은 나눈 goal의 둘째 조각뿐이라 선 자리에서 다시 짠다
        if self._on_node(m) and not site.nodes[m.node].via:
            return self._path(m, m.node, goal, cost)
        zone = site.zone_at(m.x, m.y)
        if zone is None or site.zones[zone].kind != SHARED:
            return None
        best, best_cost = None, math.inf
        now = self._clock()
        for name, zones in self._reach(zone).items():
            if self._blocked.get((HERE + m.name, name), 0.0) > now:
                continue
            rest = self._path(m, name, goal, cost)
            if rest is None:
                continue
            first = self._split(m, name, zones, cost, now)
            if first is None:
                first = [free_edge(site, m.x, m.y, name, zones)]
            total = (first[0].length / self.params.nav_speed_mps
                     + self._node_wait(m, first[0].b)
                     + sum(cost(edge) for edge in (*first[1:], *rest)))
            if total < best_cost:
                best, best_cost = [*first, *rest], total
        return best

    # ---- 나눈 goal(해결안 B): 두 길 값이 비슷한 출발 자리면 중간 지점을 거쳐 둘로 ----
    def _split(self, m, name: str, zones, cost, now: float):
        site = self.site
        via = site.split_via(name, m.x, m.y) if self.params.plan_split else ""
        # 첫 조각이 막혔거나 중간 지점을 남이 쥐고 있으면 곧장 간다
        if (not via or self._blocked.get((HERE + m.name, via), 0.0) > now
                or self.grants.node_holder(via) not in ("", m.name)):
            return None
        second = site.edge(via, name)
        if math.isinf(cost(second)):
            return None
        return [free_edge(site, m.x, m.y, via, zones), second]

    # ---- 노드에서 짠 길: 나눌 대상으로 끝나는 nav_goal 사슬(through로 합쳐짐)의 출발 노드가 사각형 안이면 ----
    def _path(self, m, start: str, goal: str, cost=None):
        cost = cost or self._cost(m)
        edges = shortest_path(self.site, start, goal, cost)
        if not edges or not self.params.plan_split:
            return edges
        site, now = self.site, self._clock()
        out, index = [], 0
        while index < len(edges):
            # 사슬: order가 goal 하나로 합칠 조각(_cut과 같은 자름)
            end = segment_end(site, edges, index, stretch_end(site, edges, index))
            chain = edges[index:end]
            out.extend(self._split_chain(m, chain, cost, now) if chain[0].mode == NAV_GOAL else chain)
            index = end
        return out

    def _split_chain(self, m, chain, cost, now: float) -> list:
        site, start, to = self.site, chain[0].a, chain[-1].b
        via = site.split_via(to, *site.node_pose(start)) if start else ""
        if (not via or via == start or self._blocked.get((start, via), 0.0) > now
                or self.grants.node_holder(via) not in ("", m.name)):
            return chain
        second = site.edge(via, to)
        if second is None or math.isinf(cost(second)):
            return chain
        a, b = site.nodes[start], site.nodes[via]
        points = ((a.x, a.y), (b.x, b.y))
        zones = tuple(dict.fromkeys(z for z in (a.zone, b.zone) if z is not None))
        return [Edge(start, via, NAV_GOAL, "", points, zones, math.dist(*points)), second]

    # ---- 구역에서 닿는 노드와 물러나기 edge ----
    def _reach(self, zone: str) -> dict:
        memo = self.__dict__.setdefault("_reach_memo", {})
        if zone not in memo:
            site = self.site
            by_goal = {edge.b for edge in site.edges() if edge.mode == NAV_GOAL}
            inside = {name: (zone,) for name in site.nodes_in(zone)
                      if (name in by_goal or not by_goal) and not site.nodes[name].via}
            if inside:
                memo[zone] = inside
            else:
                reach = {}
                for edge in site.edges():
                    if edge.mode == NAV_GOAL and zone in edge.zones and not site.nodes[edge.b].via:
                        reach.setdefault(edge.b, edge.zones[edge.zones.index(zone):])
                memo[zone] = reach
        return memo[zone]

    def _back_edge(self, m):
        site = self.site
        here = site.zone_at(m.x, m.y)
        via = site.nodes[m.avoid.retreat_via].zone if m.avoid.retreat_via else None
        target = site.nodes[m.avoid.retreat_to].zone
        zones = [zone for zone in (here, via, target) if zone is not None]
        # 갈림 -> 마당 지점 고정 경로의 꺾는 점을 따른다
        to, via_node = m.avoid.retreat_to, m.avoid.retreat_via
        down = site.edge(via_node, to) if via_node else None
        bends = down.points[1:-1] if down is not None else ()
        return back_edge(site, m.x, m.y, to, dict.fromkeys(zones), via_node, bends)

    # ---- 후보 중 잡을 수 있는 route 고르기 ----
    def _choose(self, m):
        route, first_want = self._grab(m, self._candidates(m))
        if route is not None:
            route.standby = (m.phase in (TO_WORK, TO_OUTBOUND)
                            and route.goal in self.site.parking)
            return route
        if first_want is None and m.avoid.retreat_to and not self._retreat_held_up(m):
            back = self._back_edge(m)
            route = None if back is None else Route(m.avoid.retreat_to, [back])
            if route is not None and self._take(m, route) is None:
                return route
        if first_want is not None:
            self._wants[m.name] = first_want
            self._note_wait(m, first_want)
        elif self._blocked:
            if m.watch.wait.since is None:
                m.watch.wait.since = self._clock()
                m.watch.wait.key = ("BLOCKED",)
        elif m.watch.wait.key == ("BLOCKED",):
            self._stop_wanting(m)
        return None

    def _grab(self, m, candidates, after: int = 0):
        first_want = None
        for goal, queued in candidates:
            edges = self._way_after(m, after, goal) if after else self._way(m, goal)
            if not edges:
                continue
            route = Route(goal, edges)
            if queued and self.params.use_order_model:
                route = self._through(m, route)
            if after:
                route.done, route.granted = m.route.done, after
            want = self._take(m, route)
            if want is None:
                if queued and not m.watch.wait.queued:
                    m.watch.wait.queued = True
                    self._event("INFO", "DOCK_QUEUED", f"{m.dest} is busy, waits at {goal}",
                                m.name)
                elif not queued:
                    self._take_turn(m, route)
                return route, None
            first_want = first_want or want
        return None, first_want

    # ---- 다음 구간의 구역, 노드, 차선 잡기 ----
    def _take(self, m, route):
        site, grants, now = self.site, self.grants, self._clock()
        start = route.granted
        end = stretch_end(site, route.edges, start)
        if not self.params.use_order_model and route.goal in site.docks:
            while end < len(route.edges) and self._lane_spot(route.edges[end - 1].b):
                end = stretch_end(site, route.edges, end)
        zones, nodes, lanes = needs(route.edges[start:end])
        eta = {}
        for lane in lanes:
            index = next(i for i in range(start, end) if route.edges[i].lane == lane)
            eta[lane] = self._left_s(m, route, index, ARRIVE)
        since = now if m.watch.wait.since is None else m.watch.wait.since
        want = Want(tuple(zones), tuple(nodes), lanes, since, eta)
        entered = [lane for lane in lanes if m.name not in grants.holders(lane)]
        for lane in entered:
            if self.lanes.refusal(m.name, lane, want, now, self._rivals(m, lane, lanes[lane])):
                return want
        empty = [lane for lane in entered if not grants.holders(lane)]
        if not grants.take(m.name, zones, nodes, lanes):
            return want
        route.granted = end
        self._stop_wanting(m)
        for lane in empty:
            self._lane_set(m, lane, want, now)
        self._event("INFO", "ZONE_GRANTED",
                    f"{'+'.join(zones)} to {route.edges[end - 1].b}", m.name)
        self._detours.note(m, route)
        return None

    def _lane_spot(self, node: str) -> bool:
        spec = self.site.nodes[node]
        return spec.kind == WAIT and self.site.zones[spec.zone].kind == LANE

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
        ep = m.watch.wait
        ep.key = tuple(blockers) or tuple(want.lanes)
        if ep.since is None:
            ep.since = now
            self._event("INFO", "ZONE_WAIT", f"waits for {'+'.join(ep.key)}", m.name)
        elif now - ep.since >= self.params.zone_wait_warn_s:
            self._event_once(ep, "WARN", "ZONE_WAIT",
                             f"has waited {now - ep.since:.0f} s for {'+'.join(ep.key)}", m.name)
