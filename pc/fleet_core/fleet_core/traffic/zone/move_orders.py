"""order 모델(use_order_model, order 추가지시서 4절): 가는 길 전체를 조각 목록으로 보낸다.

여기의 order는 VDA 5050의 order(이동 하나의 조각 목록)다. 주문과는 다른 것이다.

- 잡은 구역 안의 조각은 base(로봇이 달려도 된다), 목적지까지의 나머지는 horizon(참고만)이다.
- 로봇의 base 끝까지 남은 시간이 lookahead_s보다 짧아지면 다음 구간을 한꺼번에 잡아 보고, 잡으면
  새 조각만 담은 update를 보낸다. 못 잡으면 로봇은 base 끝에서 서고 관제는 계속 잡아 본다.
- 앞길이 바뀌면 base가 빈 update로 horizon만 바꾼다. 이미 준 base는 줄이지 않는다. 줄여야 하면
  이동을 거둔다(막힘, 목적 바뀜).
- update가 거절되면 로봇이 지금 base를 끝낸 뒤 그 자리에서 새 order로 잇는다.
"""
from __future__ import annotations

from fleet_core.common.commands import MoveOrderCmd, SegmentOut
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, route
from fleet_core.traffic.mission import MOVE

from .robots import ORDER, Goal, MoveOrder
from .trips import Trip, left_s, move_command, retreat_command, segment_end, stretch_end

HORIZON_HOLD_S = 5.0  # horizon을 바꾼 뒤 이만큼은 다시 바꾸지 않는다(가정)
COMM_BASE_MARGIN_S = 3.0  # 끊긴 로봇이 base 끝에 닿을 예상 시각에 더하는 여유
REJECTED = "ORDER_UPDATE_REJECTED"
SUPERSEDED = "SUPERSEDED"
CONTROLLER = {FOLLOW_PATH: "FollowPath", RETREAT: "FollowPathReverse", NAV_GOAL: ""}


class MoveOrders:
    # ---- 조각 ----

    def _cut(self, trip, start: int, first_seq: int) -> list:
        """trip.links[start:]를 조각 (번호, 첫 연결, 끝 연결의 다음)들로 자른다.

        한꺼번에 잡는 구간의 끝과 가는 방식이 바뀌는 곳에서 끊는다. base는 구간 단위로 늘어난다.
        """
        cuts, seq, index = [], first_seq, start
        while index < len(trip.links):
            stop = stretch_end(self.site, trip.links, index)
            while index < stop:
                end = segment_end(trip.links, index, stop)
                cuts.append((seq, index, end))
                seq, index = seq + 1, end
        return cuts

    def _segment(self, m, trip, cut, here: bool) -> SegmentOut:
        """조각 하나를 명령의 모양으로. here면 로봇이 선 자리에서, 아니면 앞 지점에서 시작한다."""
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

    def _fresh_base(self, m, trip, here: bool = False) -> tuple:
        """아직 주지 않은 조각 가운데 잡은 연결 안에 드는 것들. 준 것으로 센다."""
        mo = m.move_order
        cuts = [cut for cut in mo.cuts[mo.given:] if cut[2] <= trip.granted]
        mo.given += len(cuts)
        return tuple(self._segment(m, trip, cut, here and index == 0)
                     for index, cut in enumerate(cuts))

    def _horizon(self, m, trip) -> tuple:
        mo = m.move_order
        return tuple(self._segment(m, trip, cut, False) for cut in mo.cuts[mo.given:])

    def _through(self, m, trip):
        """대기 자리로 가는 길에 그 자리에서 도크까지를 잇는다. 도크 쪽은 horizon이 된다."""
        dock = self.site.stations[m.dest].dock
        rest = route(self.site, trip.goal, dock, self._cost(m))
        return Trip(dock, [*trip.links, *rest]) if rest else trip

    # ---- 보내기 ----

    def _send_order(self, m) -> None:
        """지금 자리에서 목적지까지를 order 하나로 보낸다."""
        trip = m.trip
        goal_id = self._next_goal_id()
        mo = m.move_order = MoveOrder(f"{m.name}-{goal_id}", self._cut(trip, trip.done, 1),
                                      horizon_at=self._clock())
        base, horizon = self._fresh_base(m, trip, here=True), self._horizon(m, trip)
        self._out.append(MoveOrderCmd(m.name, goal_id, mo.id, 0, "", base, horizon))
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
        """update를 새 goal로 보낸다. base가 비면 horizon만 바꾸는 update다."""
        trip, mo = m.trip, m.move_order
        goal_id = self._next_goal_id()
        stitch = trip.links[before[1] - 1].b
        if self.order_fault == "stitch":  # 시험용: 이음 지점을 일부러 틀리게 보낸다
            stitch, self.order_fault = "?", ""
        mo.update += 1
        mo.before = before
        self._out.append(MoveOrderCmd(m.name, goal_id, mo.id, mo.update, stitch, base,
                                      self._horizon(m, trip)))
        self._order_goal(m, goal_id)

    # ---- 늘리기 ----

    def _extend_order(self, m) -> None:
        """base 끝이 가까워졌거나 로봇이 거기 서 있으면 다음 구간을 잡아 update를 보낸다."""
        trip, mo = m.trip, m.move_order
        if trip is None or mo is None:
            return
        if mo.broken or trip.stale:
            # 거절된 order는 base 끝에 선 뒤에, 목적이 바뀐 order는 넓은 구역에 나오는 대로 거둔다.
            # 고정 구간 안에서는 거두지 않는다: 갈림목이나 한 줄 구역 안에 서지 않는다
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
        if want is not None:  # 한꺼번에 못 잡았다. 로봇은 base 끝에서 선다
            self._wants[m.name] = want
            self._note_wait(m, want)
            return
        mo.waiting = mo.waited = False
        self._send_update(m, before, self._fresh_base(m, trip))
        self._event("INFO", "ORDER_UPDATED",
                    f"{mo.id} update {mo.update}: base to {m.goal.end}", m.name)

    def _dock_ahead(self, m, trip):
        """다음 구간의 끝이 이 로봇이 가는 스테이션의 도크이면 그 스테이션."""
        end = stretch_end(self.site, trip.links, trip.granted)
        station = self.site.stations.get(m.dest) if m.dest else None
        if station is not None and trip.links[end - 1].b == station.dock:
            return station
        return None

    def _refresh_horizon(self, m) -> bool:
        """base 끝에서 목적지까지의 길이 달라졌으면 horizon만 바꿔 보낸다."""
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

    # ---- 로봇이 알려 온 것 ----

    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="") -> None:
        m = self._active(robot, goal_id, MOVE)
        if m is None or m.move_order is None:
            return
        mo = m.move_order
        self._order_progress(m, last_done_seq)
        mo.waiting = bool(waiting_at_base_end)
        if mo.waiting and not mo.waited:
            mo.waited = True
            self._event("INFO", "BASE_END_WAIT",
                        f"stands at {m.node}, the next stretch is not taken yet", m.name)
        if horizon_warning and not mo.warned:
            mo.warned = True
            self._event("WARN", "HORIZON_INVALID", horizon_warning, m.name)

    def _order_progress(self, m, seq: int) -> None:
        """로봇이 seq번 조각까지 끝냈다. 지난 지점을 따라간다."""
        trip, mo = m.trip, m.move_order
        for number, first, end in mo.cuts:
            if mo.done_seq < number <= seq:
                via = trip.links[first].via
                trip.done = max(trip.done, end)
                m.node = trip.links[end - 1].b
                self._event("INFO", "SEGMENT_DONE", f"{mo.id} seq {number} ({via}) to {m.node}",
                            m.name)
                if via == RETREAT:
                    m.retreat_to = ""
                    self._event("INFO", "RETREAT_DONE", f"stands at {m.node}", m.name)
        mo.done_seq = max(mo.done_seq, seq)

    def _order_result(self, m, success: bool, reason: str, last_done_seq: int) -> bool:
        """order goal의 결과. 여기서 다 다뤘으면 True, 여느 이동처럼 마저 다룰 것이면 False."""
        mo, trip = m.move_order, m.trip
        if reason.startswith(REJECTED):  # 이 update만 끝났다. 로봇은 이전 goal로 지금 base를 간다
            self._event("WARN", "ORDER_UPDATE_REJECTED", f"{mo.id} update {mo.update}: {reason}",
                        m.name)
            goal_id, links = mo.before
            m.goal = Goal(MOVE, goal_id, links, trip.links[links - 1].b, ORDER)
            trip.sent = links
            mo.broken = True
            return True
        if reason == SUPERSEDED:  # 실패가 아니다
            return True
        self._order_progress(m, last_done_seq)
        return False

    # ---- 통신이 끊긴 로봇 ----

    def _hold_base(self, m, now: float) -> str:
        """끊긴 로봇은 base 끝까지 간다. 그 예상 시각과 여유가 지날 때까지 구역을 풀지 않는다."""
        trip = m.trip
        if m.move_order is None or trip is None or m.goal is None:
            return ""
        left = left_s(self.params, trip.links[trip.done:trip.granted], m.x, m.y)
        m.base_until = now + left + COMM_BASE_MARGIN_S
        return f", drives on to {m.goal.end} (about {left:.0f} s)"
