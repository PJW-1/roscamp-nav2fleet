"""order 모델에서 로봇이 알려 온 것: 피드백, 결과, 붙지 못한 update, 통신이 끊긴 로봇의 base.

update를 보낸 뒤 로봇의 답이 오기 전에는(MoveOrder.before) 로봇에서 살아 있는 goal이 이전 것일
수도, 새것일 수도 있다. 이전 goal의 SUPERSEDED나 새 goal의 피드백이 오면 붙은 것이고, 거절이나
전송 실패가 오면 이전 goal로 돌아가 지금 base가 끝난 뒤 새 order로 잇는다.
"""
from __future__ import annotations

from fleet_core.fleet_map import RETREAT
from fleet_core.traffic.mission import MOVE

from .robots import ORDER, Goal
from .trips import left_s

REJECTED = "ORDER_UPDATE_REJECTED"
SUPERSEDED = "SUPERSEDED"


class OrderAnswers:

    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="") -> None:
        m = self._robots.get(robot)
        mo = m.move_order if m is not None else None
        if mo is None or m.goal is None:
            return
        if mo.before is not None and goal_id == mo.before[0]:  # 이전 goal이 아직 말한다
            self._order_progress(m, last_done_seq, mo.before[1])
            mo.before = (*mo.before[:2], bool(waiting_at_base_end))
            return
        if m.goal.id != goal_id:
            return
        mo.before = None  # 새 goal이 말한다: update가 붙었다
        self._order_progress(m, last_done_seq, m.goal.links)
        mo.waiting = bool(waiting_at_base_end)
        if mo.waiting and not mo.waited:
            mo.waited = True
            self._event("INFO", "BASE_END_WAIT",
                        f"stands at {m.node}, the next stretch is not taken yet", m.name)
        if horizon_warning and not mo.warned:
            mo.warned = True
            self._event("WARN", "HORIZON_INVALID", horizon_warning, m.name)

    def _order_progress(self, m, seq: int, links: int) -> None:
        """로봇이 seq번 조각까지 끝냈다. 지난 지점을 따라간다.

        links는 말하는 goal이 허락한 연결 수다. 그 밖의 조각 번호는 믿지 않는다.
        """
        trip, mo = m.trip, m.move_order
        last = max((number for number, _, end in mo.cuts if end <= links), default=0)
        if seq > last:  # 허락하지 않은 조각이다. 잘못된 보고로 보고 진행을 바꾸지 않는다
            if not mo.overrun:
                mo.overrun = True
                self._event("WARN", "SEGMENT_BEYOND_BASE",
                            f"{mo.id} reports seq {seq} done, the base ends at seq {last}", m.name)
            return
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
        mo = m.move_order
        if reason.startswith(REJECTED) and mo.before is not None:
            self._update_failed(m, reason)  # 이 update만 끝났다
            return True
        if reason == SUPERSEDED:  # 실패가 아니다
            return True
        mo.before = None
        self._order_progress(m, last_done_seq, m.goal.links)
        return False

    def _old_goal_result(self, m, goal_id: int, reason: str) -> bool:
        """답을 기다리는 update의 이전 goal이 끝났다. 여기서 다 다뤘으면 True."""
        mo = m.move_order
        if mo.before is None or goal_id != mo.before[0]:
            return False
        if reason == SUPERSEDED:  # 로봇이 update를 이어 붙였다
            mo.before = None
            return True
        self._restore(m)  # update가 붙기 전에 이전 goal이 끝났다. 그 goal의 결과로 다룬다
        return False

    def _update_lost(self, m, why: str) -> bool:
        """update의 goal이 로봇에 닿지 못했으면 이전 goal로 돌아가고 True."""
        if m.move_order is None or m.move_order.before is None:
            return False
        self._update_failed(m, why)
        return True

    def _update_unclear(self, m, why: str) -> bool:
        """update의 goal이 로봇에 붙었는지 알 수 없으면 두 goal을 모두 거두고 True."""
        if m.move_order is None or m.move_order.before is None:
            return False
        self._end_both(m, why)
        return True

    def _end_both(self, m, why: str) -> None:
        """이전 goal과 update의 goal을 모두 거둔다. 잠시 뒤 서 있는 자리에서 새 order로 잇는다."""
        mo = m.move_order
        self._event("WARN", "ORDER_UPDATE_REJECTED",
                    f"{mo.id} update {mo.update}: {why}, ends both goals", m.name)
        self._cancel_goal(m)
        m.retry_at = self._clock() + self.params.move_retry_wait_s

    def _update_failed(self, m, why: str) -> None:
        """update가 붙지 못했다. 로봇은 이전 goal로 지금 base를 간다. 끝나면 새 order로 잇는다."""
        mo = m.move_order
        self._event("WARN", "ORDER_UPDATE_REJECTED", f"{mo.id} update {mo.update}: {why}", m.name)
        self._restore(m)
        mo.broken = True

    def _restore(self, m) -> None:
        """이전 goal을 다시 지금 goal로 삼는다."""
        mo, trip = m.move_order, m.trip
        goal_id, links, waiting = mo.before
        m.goal = Goal(MOVE, goal_id, links, trip.links[links - 1].b, ORDER)
        trip.sent = links
        mo.before = None
        mo.waiting = waiting
        mo.waited = mo.waited or waiting

    # ---- 통신이 끊긴 로봇 ----

    def _hold_base(self, m, now: float) -> str:
        """끊긴 로봇은 base 끝까지 간다. 그 예상 시각과 여유가 지날 때까지 구역을 풀지 않는다."""
        trip = m.trip
        if m.move_order is None or trip is None or m.goal is None:
            return ""
        left = left_s(self.params, trip.links[trip.done:trip.granted], m.x, m.y)
        # 음수 여유는 0으로 본다: 닿을 예상 시각보다 먼저 풀지 않는다
        m.base_until = now + left + max(0.0, self.params.comm_base_hold_extra_s)
        return f", drives on to {m.goal.end} (about {left:.0f} s)"
