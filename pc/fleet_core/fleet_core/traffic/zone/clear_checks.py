# ---- 막힘 표시 해제 확인 요청 믹스인 ----
from __future__ import annotations

import math

from fleet_core.common.commands import BLOCKED_AT, CLEAR, NO_ANSWER, CheckClearCmd
from fleet_core.traffic.mission import LOCALIZING


class ClearChecks:
    # ---- 막힘 표시 근처 로봇에게 확인 요청 보내기 ----
    def _ask_clear(self, now: float) -> None:
        p = self.params
        for mark in self._marks.values():
            if now - mark.asked_at < p.check_interval_s:
                continue
            if mark.check:
                mark.clears = 0
            near = [m for m in self._robots.values()
                    if m.available and m.name != mark.by and m.robot_state != LOCALIZING
                    and math.dist((m.x, m.y), (mark.x, mark.y)) <= p.check_range_m]
            if not near:
                mark.check, mark.asked = 0, ""
                continue
            m = min(near, key=lambda r: (mark.asked_last.get(r.name, -math.inf),
                                         math.dist((r.x, r.y), (mark.x, mark.y)), r.name))
            mark.check, mark.asked, mark.asked_at = self._mark_seq(), m.name, now
            mark.asked_last[m.name] = now
            self._out.append(CheckClearCmd(m.name, mark.check, mark.x, mark.y,
                                           p.check_radius_m))

    # ---- 확인 응답으로 표시를 지우거나 다시 막기 ----
    def on_clear_check(self, robot, check_id, answer) -> None:
        mark = next((mk for mk in self._marks.values()
                     if mk.check and mk.check == check_id and mk.asked == robot), None)
        if mark is None:
            return
        mark.check = 0
        if answer == NO_ANSWER:
            mark.clears = 0
            return
        now, p = self._clock(), self.params
        where = f"({mark.x:.2f}, {mark.y:.2f})"
        if answer == CLEAR:
            mark.clears += 1
            if mark.clears >= p.clear_confirm:
                self._drop_mark(mark)
                self._mark_memory = [item for item in self._mark_memory
                                     if math.dist(item[:2], (mark.x, mark.y)) > p.near_m]
                self._event("INFO", "BLOCK_CLEARED",
                            f"{where} clear by {mark.clears} checks, marked "
                            f"{now - mark.since:.0f} s ago", robot)
        elif answer == BLOCKED_AT:
            mark.clears = 0
            mark.until = now + mark.retry_s
            for key in mark.keys:
                self._hold_key(mark, key)
            self._event("INFO", "BLOCK_CONFIRMED",
                        f"{where} still blocked, tried again in {mark.retry_s:.0f} s", robot)
        else:
            mark.clears = 0
