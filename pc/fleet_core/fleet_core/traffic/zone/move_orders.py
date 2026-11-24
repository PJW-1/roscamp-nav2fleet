# ---- order 모델 주문 만들기와 갱신 믹스인 ----
from __future__ import annotations

from fleet_core.common.commands import MoveOrderCmd, SegmentOut
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, SHARED
from fleet_core.traffic.mission import MOVE, TO_PARK

from .robots import ORDER, Goal, MoveOrder
from .routes import Route, move_command, retreat_command, segment_end, stretch_end

HORIZON_HOLD_S = 5.0
UPDATE_ANSWER_S = 2.0
CONTROLLER = {FOLLOW_PATH: "FollowPath", RETREAT: "FollowPathReverse", NAV_GOAL: ""}


# ---- route를 seq 조각으로 나누고 조각 명령 만들기 ----
class MoveOrders:
    def _cut(self, route, start: int, first_seq: int) -> list:
        cuts, seq, index = [], first_seq, start
        while index < len(route.edges):
            stop = stretch_end(self.site, route.edges, index)
            while index < stop:
                end = segment_end(self.site, route.edges, index, stop)
                cuts.append((seq, index, end))
                seq, index = seq + 1, end
        return cuts

    def _segment(self, m, route, cut, here: bool) -> SegmentOut:
        seq, first, end = cut
        edges = route.edges[first:end]
        mode, node = edges[0].mode, edges[-1].b
        x, y = (m.x, m.y) if here or not edges[0].a else self.site.node_pose(edges[0].a)
        zones = tuple(dict.fromkeys(zone for edge in edges for zone in edge.zones))
        if mode == RETREAT:
            path = retreat_command(self.params, m.name, 0, edges, x, y).path
            return SegmentOut(seq, mode, node, zones, path, *path[-1], CONTROLLER[mode])
        after = route.edges[end] if end < len(route.edges) else None
        cmd = move_command(self.site, m.name, 0, edges, x, y, after)
        return SegmentOut(seq, mode, node, zones, cmd.path, cmd.x, cmd.y, cmd.yaw, CONTROLLER[mode])

    # ---- base와 horizon 조각, 도크까지 이은 route ----
    def _fresh_base(self, m, route, here: bool = False, upto: int | None = None) -> tuple:
        mo = m.move_order
        upto = route.granted if upto is None else upto
        cuts = [cut for cut in mo.cuts[mo.given:] if cut[2] <= upto]
        mo.given += len(cuts)
        return tuple(self._segment(m, route, cut, here and index == 0)
                     for index, cut in enumerate(cuts))

    def _horizon(self, m, route) -> tuple:
        mo = m.move_order
        return tuple(self._segment(m, route, cut, False) for cut in mo.cuts[mo.given:])

    def _through(self, m, route):
        dock = self.site.stations[m.dest].dock
        rest = self._path(m, route.goal, dock)
        return Route(dock, [*route.edges, *rest]) if rest else route

    # ---- 첫 주문 보내기와 갱신 보내기 ----
    def _send_order(self, m) -> None:
        route = m.route
        goal_id = self._next_goal_id()
        mo = m.move_order = MoveOrder(f"{m.name}-{goal_id}", self._cut(route, route.done, 1),
                                      horizon_at=self._clock())
        upto = self._standby_end(m, route)
        base, horizon = self._fresh_base(m, route, True, upto), self._horizon(m, route)
        self._out.append(MoveOrderCmd(m.name, goal_id, mo.id, 0, "", base, horizon,
                                      self._goal_priority(m)))
        self._order_goal(m, goal_id)
        far = f", horizon to {horizon[-1].end_node}" if horizon else ""
        self._event("INFO", "ORDER_SENT", f"{mo.id}: base to {m.goal.end}{far}", m.name)
        if base[0].mode == RETREAT:
            self._event("INFO", "RETREAT_START", f"backs to {base[0].end_node}", m.name)

    def _order_goal(self, m, goal_id: int) -> None:
        route, mo = m.route, m.move_order
        end = mo.cuts[mo.given - 1][2] if mo.given else route.granted
        self._set_goal(m, Goal(MOVE, goal_id, end, route.edges[end - 1].b, ORDER))
        route.sent = end

    def _send_update(self, m, before: tuple, base: tuple) -> None:
        route, mo = m.route, m.move_order
        goal_id = self._next_goal_id()
        stitch = route.edges[before[1] - 1].b
        if self.order_fault == "stitch":
            stitch, self.order_fault = "?", ""
        mo.update += 1
        mo.before, mo.asked_at = (*before, mo.waiting), self._clock()
        if base:
            mo.waiting = mo.waited = False
        self._out.append(MoveOrderCmd(m.name, goal_id, mo.id, mo.update, stitch, base,
                                      self._horizon(m, route), self._goal_priority(m)))
        self._order_goal(m, goal_id)

    # ---- base 끝이 다가오면 다음 구간 잡아 갱신 ----
    def _extend_order(self, m) -> None:
        route, mo = m.route, m.move_order
        if route is None or mo is None:
            return
        if mo.before is not None:
            if self._clock() - mo.asked_at >= UPDATE_ANSWER_S:
                self._end_both(m, f"no answer in {UPDATE_ANSWER_S:.0f} s")
            return
        if mo.broken or route.stale:
            # 목적이 바뀐 route: nav_goal이면 바로, 고정 경로면 넓은 구역 노드에서 멈춘다
            mode = self._mode(m)
            at_yard_node = (mode != RETREAT and self._in_yard(m) and self._on_node(m)
                            and self._yard_node(m.node))
            if mo.waiting or (route.stale and (mode == NAV_GOAL or at_yard_node)):
                self._cancel_goal(m)
                if route.stale:
                    self._drop_route(m)
            return
        p = self.params
        end = m.goal.edges
        if end < route.granted:
            left = self._left_s(m, route, end)
            if mo.waiting or left <= p.standby_lookahead_s:
                self._send_update(m, (m.goal.id, end), self._fresh_base(m, route))
                self._event("INFO", "ORDER_UPDATED",
                            f"{mo.id} update {mo.update}: base to {m.goal.end}", m.name)
            return
        if route.granted >= len(route.edges) or (
                not mo.waiting
                and self._left_s(m, route, route.granted) > p.lookahead_s):
            return
        if self._refresh_horizon(m):
            return
        station = self._dock_ahead(m, route)
        if station is not None and not (self._is_next(m, station.id)
                                        and self._dock_free(m, station.dock)):
            self._stay(m, station)
            return
        before = (m.goal.id, route.granted)
        want = self._take(m, route)
        if want is not None and mo.waiting and m.phase == TO_PARK:
            # 주차 자리를 남이 잡았으면 다시 고른다
            self._cancel_goal(m)
            self._drop_route(m)
            return
        if want is not None:
            self._wants[m.name] = want
            self._note_wait(m, want)
            return
        if station is not None:
            route.turn = True
        self._send_update(m, before, self._fresh_base(m, route))
        self._event("INFO", "ORDER_UPDATED",
                    f"{mo.id} update {mo.update}: base to {m.goal.end}", m.name)

    def _yard_node(self, node: str) -> bool:
        zone = self.site.zones.get(self.site.nodes[node].zone)
        return zone is not None and zone.kind == SHARED

    # ---- 주차하러 가는 route: 첫 고정 경로까지만 base로 주고, 그 끝에 새 길을 잇기 ----
    def _standby_end(self, m, route) -> int | None:
        if not route.standby:
            return None
        mo, end = m.move_order, None
        for _, first, stop in mo.cuts[mo.given:]:
            if stop > route.granted or route.edges[first].mode != FOLLOW_PATH:
                break
            end = stop
        return end if end is not None and end < route.granted else None

    def _held_end(self, m) -> int:
        route, goal, mo = m.route, m.goal, m.move_order
        if (goal is None or goal.mode != ORDER or mo is None or mo.before is not None
                or mo.broken or route.stale or goal.edges >= route.granted):
            return 0
        return goal.edges

    def _way_after(self, m, end: int, goal: str):
        rest = self._path(m, m.route.edges[end - 1].b, goal)
        return [*m.route.edges[:end], *rest] if rest else None

    def _turn_order(self, m, fresh) -> None:
        mo, end = m.move_order, m.goal.edges
        before = (m.goal.id, end)
        m.route = fresh
        mo.cuts[mo.given:] = self._cut(fresh, end, mo.cuts[mo.given - 1][0] + 1)
        mo.horizon_at = self._clock()
        self._send_update(m, before, self._fresh_base(m, fresh))
        self._event("INFO", "ORDER_UPDATED", f"{mo.id} update {mo.update}: base to {m.goal.end}",
                    m.name)

    # ---- 앞쪽 도크 확인과 horizon 경로 다시 짜기 ----
    def _dock_ahead(self, m, route):
        end = stretch_end(self.site, route.edges, route.granted)
        station = self.site.stations.get(m.dest) if m.dest else None
        if station is not None and route.edges[end - 1].b == station.dock:
            return station
        return None

    def _refresh_horizon(self, m) -> bool:
        route, mo = m.route, m.move_order
        now = self._clock()
        if now - mo.horizon_at < HORIZON_HOLD_S:
            return False
        old = route.edges[route.granted:]
        fresh = self._path(m, route.edges[route.granted - 1].b, route.goal)
        if not fresh or [(k.a, k.b) for k in fresh] == [(k.a, k.b) for k in old]:
            return False
        station = self.site.stations.get(m.dest) if m.dest else None
        if station is not None and station.line:
            spots = [[k.b for k in edges if k.b in station.queue] for edges in (old, fresh)]
            if spots[0] and spots[0] != spots[1]:
                return False  # 줄 자리가 바뀌는 새 길은 쓰지 않는다
        route.edges[route.granted:] = fresh
        mo.cuts[mo.given:] = self._cut(route, route.granted, mo.cuts[mo.given - 1][0] + 1)
        mo.horizon_at = now
        self._send_update(m, (m.goal.id, route.granted), ())
        self._event("INFO", "HORIZON_CHANGED", f"{mo.id} update {mo.update}: now by "
                    f"{' '.join(k.b for k in fresh)}", m.name)
        return True
