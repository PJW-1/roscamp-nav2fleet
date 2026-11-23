# ---- order 모델 피드백과 결과 처리 믹스인 ----
from __future__ import annotations

import math

from fleet_core.fleet_map import NAV_GOAL, RETREAT
from fleet_core.traffic.mission import MOVE

from .robots import ORDER, Goal, LeftReport
from .routes import PASS, STOP, eta_speed, left_s, nav_end_s, nav_stretch, travel_s

REJECTED = "ORDER_UPDATE_REJECTED"
SUPERSEDED = "SUPERSEDED"
LEFT_STALE_S = 1.0
LEFT_JUMP_M = 0.10
LEFT_CALM_S = 2.0
LEFT_SHORT_M = 0.10


# ---- order 피드백으로 진행과 base 끝 대기 기록 ----
class OrderAnswers:

    def on_move_feedback(self, robot, goal_id, last_done_seq, waiting_at_base_end,
                         horizon_warning="", distance_remaining=None) -> None:
        m = self._robots.get(robot)
        mo = m.move_order if m is not None else None
        if mo is None or m.goal is None:
            return
        if mo.before is not None and goal_id == mo.before[0]:
            self._order_progress(m, last_done_seq, mo.before[1])
            mo.before = (*mo.before[:2], bool(waiting_at_base_end))
            self._note_left(mo, last_done_seq, waiting_at_base_end, distance_remaining)
            return
        if m.goal.id != goal_id:
            return
        mo.before = None
        self._order_progress(m, last_done_seq, m.goal.edges)
        self._note_left(mo, last_done_seq, waiting_at_base_end, distance_remaining)
        mo.waiting = bool(waiting_at_base_end)
        if mo.waiting and not mo.waited:
            mo.waited = True
            self._event("INFO", "BASE_END_WAIT",
                        f"stands at {m.node}, the next stretch is not taken yet", m.name)
        if horizon_warning and not mo.warned:
            mo.warned = True
            self._event("WARN", "HORIZON_INVALID", horizon_warning, m.name)

    # ---- 로봇이 보낸 지금 조각의 남은 거리 ----
    def _note_left(self, mo, done, waiting, distance) -> None:
        if waiting:
            mo.left = None
            return
        now, seq, old = self._clock(), int(done) + 1, mo.left
        if not isinstance(distance, (int, float)) or not 0.0 <= distance < math.inf:
            if old is not None:
                old.at = -math.inf
            return
        same = old is not None and old.seq == seq
        jumped_at = old.jumped_at if same else None
        if same and distance > old.m + LEFT_JUMP_M:
            jumped_at = now
        mo.left = LeftReport(seq, float(distance), now, jumped_at)

    def _reported_end(self, m, route) -> int | None:
        mo = m.move_order
        report = mo.left if mo is not None and route is m.route else None
        if report is None or report.seq != mo.done_seq + 1:
            return None
        cut = next((c for c in mo.cuts[:mo.given] if c[0] == report.seq), None)
        if cut is None or cut[1] != route.done or route.edges[cut[1]].mode != NAV_GOAL:
            return None
        now = self._clock()
        if now - report.at > LEFT_STALE_S:
            return None
        if report.jumped_at is not None and now - report.jumped_at < LEFT_CALM_S:
            return None
        end = self.site.node_pose(route.edges[cut[2] - 1].b)
        if report.m < math.dist((m.x, m.y), end) - LEFT_SHORT_M:
            return None
        return cut[2]

    def _left_s(self, m, route, end: int, ending: str = PASS) -> float:
        p, edges, done = self.params, route.edges, route.done
        stop = self._reported_end(m, route)
        nav_first = end > done and edges[done].mode == NAV_GOAL
        chain = done + nav_stretch(edges[done:end]) if nav_first else done
        end_turn = 0.0
        if ending == STOP and chain < end:
            ending = PASS
            if edges[end - 1].mode == NAV_GOAL:
                end_turn = nav_end_s(p, self.site, edges, end, m.x, m.y, m.yaw, STOP)
        adjust = (nav_end_s(p, self.site, edges, chain, m.x, m.y, m.yaw, ending)
                  if nav_first else 0.0)
        if adjust == 0.0:
            if stop is None or stop > end:
                return left_s(p, edges[done:end], m.x, m.y) + end_turn
            return (m.move_order.left.m / eta_speed(p, NAV_GOAL) + travel_s(p, edges[stop:end])
                    + end_turn)
        if stop is not None and stop <= chain:
            nav = m.move_order.left.m / eta_speed(p, NAV_GOAL) + travel_s(p, edges[stop:chain])
        else:
            nav = left_s(p, edges[done:chain], m.x, m.y)
        return max(0.0, nav + adjust) + travel_s(p, edges[chain:end]) + end_turn

    # ---- 끝낸 seq까지 route 진행 반영 ----
    def _order_progress(self, m, seq: int, edges: int) -> None:
        route, mo = m.route, m.move_order
        last = max((number for number, _, end in mo.cuts if end <= edges), default=0)
        if seq > last:
            if not mo.overrun:
                mo.overrun = True
                self._event("WARN", "SEGMENT_BEYOND_BASE",
                            f"{mo.id} reports seq {seq} done, the base ends at seq {last}", m.name)
            return
        for number, first, end in mo.cuts:
            if mo.done_seq < number <= seq:
                mode = route.edges[first].mode
                route.done = max(route.done, end)
                m.node = route.edges[end - 1].b
                m.avoid.behind_since = None
                self._event("INFO", "SEGMENT_DONE", f"{mo.id} seq {number} ({mode}) to {m.node}",
                            m.name)
                if mode == RETREAT:
                    m.avoid.retreat_to = ""
                    self._event("INFO", "RETREAT_DONE", f"stands at {m.node}", m.name)
        mo.done_seq = max(mo.done_seq, seq)

    # ---- order 결과와 이전 goal 결과 처리 ----
    def _order_result(self, m, reason: str, last_done_seq: int) -> bool:
        mo = m.move_order
        if reason.startswith(REJECTED) and mo.before is not None:
            self._update_failed(m, reason)
            return True
        if reason == SUPERSEDED:
            return True
        mo.before = None
        self._order_progress(m, last_done_seq, m.goal.edges)
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
        mo, route = m.move_order, m.route
        goal_id, edges, waiting = mo.before
        self._set_goal(m, Goal(MOVE, goal_id, edges, route.edges[edges - 1].b, ORDER),
                       reset_watch=False)
        route.sent = edges
        mo.before = None
        mo.waiting = waiting
        mo.waited = mo.waited or waiting

    # ---- 통신 끊김 때 base 끝까지 갈 시간 잡기 ----
    def _hold_base(self, m, now: float) -> str:
        route = m.route
        if m.move_order is None or route is None or m.goal is None:
            return ""
        left = self._left_s(m, route, route.granted, STOP)
        m.base_until = now + left + max(0.0, self.params.comm_base_hold_extra_s)
        return f", drives on to {m.goal.end} (about {left:.0f} s)"
