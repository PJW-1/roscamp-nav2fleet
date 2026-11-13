# ---- order 모델 피드백과 결과 처리 믹스인 ----
from __future__ import annotations

from fleet_core.fleet_map import RETREAT
from fleet_core.traffic.mission import MOVE

from .robots import ORDER, Goal
from .trips import left_s

REJECTED = "ORDER_UPDATE_REJECTED"
SUPERSEDED = "SUPERSEDED"


# ---- order 피드백으로 진행과 base 끝 대기 기록 ----
class OrderAnswers:

    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="") -> None:
        m = self._robots.get(robot)
        mo = m.move_order if m is not None else None
        if mo is None or m.goal is None:
            return
        if mo.before is not None and goal_id == mo.before[0]:
            self._order_progress(m, last_done_seq, mo.before[1])
            mo.before = (*mo.before[:2], bool(waiting_at_base_end))
            return
        if m.goal.id != goal_id:
            return
        mo.before = None
        self._order_progress(m, last_done_seq, m.goal.links)
        mo.waiting = bool(waiting_at_base_end)
        if mo.waiting and not mo.waited:
            mo.waited = True
            self._event("INFO", "BASE_END_WAIT",
                        f"stands at {m.node}, the next stretch is not taken yet", m.name)
        if horizon_warning and not mo.warned:
            mo.warned = True
            self._event("WARN", "HORIZON_INVALID", horizon_warning, m.name)

    # ---- 끝낸 seq까지 트립 진행 반영 ----
    def _order_progress(self, m, seq: int, links: int) -> None:
        trip, mo = m.trip, m.move_order
        last = max((number for number, _, end in mo.cuts if end <= links), default=0)
        if seq > last:
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

    # ---- order 결과와 이전 goal 결과 처리 ----
    def _order_result(self, m, success: bool, reason: str, last_done_seq: int) -> bool:
        mo = m.move_order
        if reason.startswith(REJECTED) and mo.before is not None:
            self._update_failed(m, reason)
            return True
        if reason == SUPERSEDED:
            return True
        mo.before = None
        self._order_progress(m, last_done_seq, m.goal.links)
        return False

    def _old_goal_result(self, m, goal_id: int, reason: str) -> bool:
        mo = m.move_order
        if mo.before is None or goal_id != mo.before[0]:
            return False
        if reason == SUPERSEDED:
            mo.before = None
            return True
        self._restore(m)
        return False

    # ---- order 갱신 실패 처리와 이전 goal 되살리기 ----
    def _update_lost(self, m, why: str) -> bool:
        if m.move_order is None or m.move_order.before is None:
            return False
        self._update_failed(m, why)
        return True

    def _update_unclear(self, m, why: str) -> bool:
        if m.move_order is None or m.move_order.before is None:
            return False
        self._end_both(m, why)
        return True

    def _end_both(self, m, why: str) -> None:
        mo = m.move_order
        self._event("WARN", "ORDER_UPDATE_REJECTED",
                    f"{mo.id} update {mo.update}: {why}, ends both goals", m.name)
        self._cancel_goal(m)
        m.retry_at = self._clock() + self.params.move_retry_wait_s

    def _update_failed(self, m, why: str) -> None:
        mo = m.move_order
        self._event("WARN", "ORDER_UPDATE_REJECTED", f"{mo.id} update {mo.update}: {why}", m.name)
        self._restore(m)
        mo.broken = True

    def _restore(self, m) -> None:
        mo, trip = m.move_order, m.trip
        goal_id, links, waiting = mo.before
        m.goal = Goal(MOVE, goal_id, links, trip.links[links - 1].b, ORDER)
        trip.sent = links
        mo.before = None
        mo.waiting = waiting
        mo.waited = mo.waited or waiting

    # ---- 통신 끊김 때 base 끝까지 갈 시간 잡기 ----
    def _hold_base(self, m, now: float) -> str:
        trip = m.trip
        if m.move_order is None or trip is None or m.goal is None:
            return ""
        left = left_s(self.params, trip.links[trip.done:trip.granted], m.x, m.y)
        m.base_until = now + left + max(0.0, self.params.comm_base_hold_extra_s)
        return f", drives on to {m.goal.end} (about {left:.0f} s)"
