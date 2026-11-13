# ---- order 모델 주문 만들기와 갱신 믹스인 ----
from __future__ import annotations

from fleet_core.common.commands import MoveOrderCmd, SegmentOut
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, route
from fleet_core.traffic.mission import MOVE

from .robots import ORDER, Goal, MoveOrder
from .trips import Trip, left_s, move_command, retreat_command, segment_end, stretch_end

HORIZON_HOLD_S = 5.0
UPDATE_ANSWER_S = 2.0
CONTROLLER = {FOLLOW_PATH: "FollowPath", RETREAT: "FollowPathReverse", NAV_GOAL: ""}


# ---- 트립을 seq 조각으로 나누고 조각 명령 만들기 ----
class MoveOrders:
    def _cut(self, trip, start: int, first_seq: int) -> list:
        cuts, seq, index = [], first_seq, start
        while index < len(trip.links):
            stop = stretch_end(self.site, trip.links, index)
            while index < stop:
                end = segment_end(trip.links, index, stop)
                cuts.append((seq, index, end))
                seq, index = seq + 1, end
        return cuts

    def _segment(self, m, trip, cut, here: bool) -> SegmentOut:
        seq, first, end = cut
        links = trip.links[first:end]
        via, node = links[0].via, links[-1].b
        x, y = (m.x, m.y) if here or not links[0].a else self.site.node_pose(links[0].a)
        zones = tuple(dict.fromkeys(zone for link in links for zone in link.zones))
        if via == RETREAT:
            path = retreat_command(self.params, m.name, 0, links, x, y).path
            return SegmentOut(seq, via, node, zones, path, *path[-1], CONTROLLER[via])
        after = trip.links[end] if end < len(trip.links) else None
        cmd = move_command(self.site, m.name, 0, links, x, y, after)
        return SegmentOut(seq, via, node, zones, cmd.path, cmd.x, cmd.y, cmd.yaw, CONTROLLER[via])

    # ---- base와 horizon 조각, 도크까지 이은 트립 ----
    def _fresh_base(self, m, trip, here: bool = False) -> tuple:
        mo = m.move_order
        cuts = [cut for cut in mo.cuts[mo.given:] if cut[2] <= trip.granted]
        mo.given += len(cuts)
        return tuple(self._segment(m, trip, cut, here and index == 0)
                     for index, cut in enumerate(cuts))

    def _horizon(self, m, trip) -> tuple:
        mo = m.move_order
        return tuple(self._segment(m, trip, cut, False) for cut in mo.cuts[mo.given:])

    def _through(self, m, trip):
        dock = self.site.stations[m.dest].dock
        rest = route(self.site, trip.goal, dock, self._cost(m))
        return Trip(dock, [*trip.links, *rest]) if rest else trip

    # ---- 첫 주문 보내기와 갱신 보내기 ----
    def _send_order(self, m) -> None:
        trip = m.trip
        goal_id = self._next_goal_id()
        mo = m.move_order = MoveOrder(f"{m.name}-{goal_id}", self._cut(trip, trip.done, 1),
                                      horizon_at=self._clock())
        base, horizon = self._fresh_base(m, trip, here=True), self._horizon(m, trip)
        self._out.append(MoveOrderCmd(m.name, goal_id, mo.id, 0, "", base, horizon,
                                      self._goal_priority(m)))
        self._order_goal(m, goal_id)
        far = f", horizon to {horizon[-1].end_node}" if horizon else ""
        self._event("INFO", "ORDER_SENT", f"{mo.id}: base to {m.goal.end}{far}", m.name)
        if base[0].mode == RETREAT:
            self._event("INFO", "RETREAT_START", f"backs to {base[0].end_node}", m.name)

    def _order_goal(self, m, goal_id: int) -> None:
        trip = m.trip
        m.goal = Goal(MOVE, goal_id, trip.granted, trip.links[trip.granted - 1].b, ORDER)
        trip.sent = trip.granted
        m.stall_from = None

    def _send_update(self, m, before: tuple, base: tuple) -> None:
        trip, mo = m.trip, m.move_order
        goal_id = self._next_goal_id()
        stitch = trip.links[before[1] - 1].b
        if self.order_fault == "stitch":
            stitch, self.order_fault = "?", ""
        mo.update += 1
        mo.before, mo.asked_at = (*before, mo.waiting), self._clock()
        if base:
            mo.waiting = mo.waited = False
        self._out.append(MoveOrderCmd(m.name, goal_id, mo.id, mo.update, stitch, base,
                                      self._horizon(m, trip), self._goal_priority(m)))
        self._order_goal(m, goal_id)

    # ---- base 끝이 다가오면 다음 구간 잡아 갱신 ----
    def _extend_order(self, m) -> None:
        trip, mo = m.trip, m.move_order
        if trip is None or mo is None:
            return
        if mo.before is not None:
            if self._clock() - mo.asked_at >= UPDATE_ANSWER_S:
                self._end_both(m, f"no answer in {UPDATE_ANSWER_S:.0f} s")
            return
        if mo.broken or trip.stale:
            if mo.waiting or (trip.stale and self._via(m) == NAV_GOAL):
                self._cancel_goal(m)
                if trip.stale:
                    self._drop_trip(m)
            return
        p = self.params
        if trip.granted >= len(trip.links) or (
                not mo.waiting
                and left_s(p, trip.links[trip.done:trip.granted], m.x, m.y) > p.lookahead_s):
            return
        if self._refresh_horizon(m):
            return
        station = self._dock_ahead(m, trip)
        if station is not None and not (self._is_next(m, station.id)
                                        and self._dock_free(m, station.dock)):
            self._stay(m, station)
            return
        before = (m.goal.id, trip.granted)
        want = self._take(m, trip)
        if want is not None:
            self._wants[m.name] = want
            self._note_wait(m, want)
            return
        if station is not None:
            trip.turn = True
        self._send_update(m, before, self._fresh_base(m, trip))
        self._event("INFO", "ORDER_UPDATED",
                    f"{mo.id} update {mo.update}: base to {m.goal.end}", m.name)

    # ---- 앞쪽 도크 확인과 horizon 경로 다시 짜기 ----
    def _dock_ahead(self, m, trip):
        end = stretch_end(self.site, trip.links, trip.granted)
        station = self.site.stations.get(m.dest) if m.dest else None
        if station is not None and trip.links[end - 1].b == station.dock:
            return station
        return None

    def _refresh_horizon(self, m) -> bool:
        trip, mo = m.trip, m.move_order
        now = self._clock()
        if now - mo.horizon_at < HORIZON_HOLD_S:
            return False
        fresh = route(self.site, trip.links[trip.granted - 1].b, trip.goal, self._cost(m))
        old = trip.links[trip.granted:]
        if not fresh or [(k.a, k.b) for k in fresh] == [(k.a, k.b) for k in old]:
            return False
        trip.links[trip.granted:] = fresh
        mo.cuts[mo.given:] = self._cut(trip, trip.granted, mo.cuts[mo.given - 1][0] + 1)
        mo.horizon_at = now
        self._send_update(m, (m.goal.id, trip.granted), ())
        self._event("INFO", "HORIZON_CHANGED", f"{mo.id} update {mo.update}: now by "
                    f"{' '.join(k.b for k in fresh)}", m.name)
        return True
