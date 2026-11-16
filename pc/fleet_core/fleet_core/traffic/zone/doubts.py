# ---- 위치 의심 때 구역 붙잡기 믹스인(suspect_hold = check) ----
from __future__ import annotations

from fleet_core.traffic.mission import NOT_AT_TARGET

SUSPECT_HELD = "ZONES_HELD_SUSPECT"
DOUBT_CLEARED = "DOUBT_CLEARED"


class Doubts:
    def _holding(self, m) -> bool:
        return bool(m.doubt)

    # ---- 의심 걸기: 쥔 구역을 그대로 쥐고 세던 놓기 횟수는 버린다 ----
    def _suspect(self, m, why: str) -> None:
        if m.doubt or not m.live or m.comm:
            return
        m.doubt, m.doubt_by = why, None
        m.clear_n.clear()
        held = list(self.grants.held_by(m.name))
        self._event("WARN", SUSPECT_HELD, f"{why}: keeps {held}", m.name)

    # ---- base 끝에서 기다릴 때 위치 확인: 맞으면 풀고, 늦으면 NOT_AT_TARGET ----
    def _judge_doubt(self, m, now: float) -> None:
        if self._gap(m, m.goal.end) <= self.params.arrival_tolerance_m:
            self._clear_doubt(m)
            return
        if m.doubt_by is None:
            m.doubt_by = now + self.params.arrival_settle_s
        elif now >= m.doubt_by:
            m.doubt_by = None
            self._cancel_goal(m)
            self._goal_failed(m, NOT_AT_TARGET)

    def _clear_doubt(self, m) -> None:
        if m.doubt:
            self._event("INFO", DOUBT_CLEARED, f"{m.doubt} ends at {m.node}", m.name)
        m.doubt, m.doubt_by = "", None

    # ---- 격리 때 의심을 pose_doubt로 넘김 ----
    def _doubt_to_fault(self, m) -> None:
        if m.doubt:
            m.pose_doubt = True
        m.doubt, m.doubt_by = "", None
