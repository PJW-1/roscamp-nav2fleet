"""지나가는 로봇이 확인하면 막힘 표시를 지운다(추가 지시서 3의 4-3, 기준(사용자)).

1. 표시의 점에서 check_range_m 안에 있는 로봇에게 CheckClear(x, y, check_radius_m)를 묻는다
   (CheckClearCmd. fleet_manager가 로봇 노드의 서비스로 옮긴다). 한 표시에 check_interval_s마다
   한 번을 넘게 묻지 않고, 답을 기다리는 동안에는 다시 묻지 않는다(답이 없으면 그 간격 뒤에).
   표시를 낸 로봇에게는 묻지 않는다: 지나가는 다른 로봇이 확인한다.
2. clear가 clear_confirm번 이어지면 표시를 지운다(BLOCK_CLEARED).
3. blocked이면 표시를 새로 고친다: 지우기 시계를 다시 잰다(BLOCK_CONFIRMED).
4. unknown은 아무것도 바꾸지 않는다.

아무도 지나가지 않으면 block_marks._expire_marks가 예전처럼 지우기 시간 뒤 지운다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import BLOCKED_AT, CLEAR, CheckClearCmd
from fleet_core.traffic.mission import LOCALIZING


class ClearChecks:
    def _ask_clear(self, now: float) -> None:
        p = self.params
        for mark in self._marks.values():
            if now - mark.asked_at < p.check_interval_s:
                continue
            near = [m for m in self._robots.values()
                    if m.available and m.name != mark.by and m.robot_state != LOCALIZING
                    and math.dist((m.x, m.y), (mark.x, mark.y)) <= p.check_range_m]
            if not near:
                mark.check, mark.asked = 0, ""  # 답이 오지 않았다. 다음에 지나가는 로봇에게 묻는다
                continue
            m = min(near, key=lambda r: (math.dist((r.x, r.y), (mark.x, mark.y)), r.name))
            mark.check, mark.asked, mark.asked_at = self._mark_seq(), m.name, now
            self._out.append(CheckClearCmd(m.name, mark.check, mark.x, mark.y,
                                           p.check_radius_m))

    def on_clear_check(self, robot, check_id, answer) -> None:
        mark = next((mk for mk in self._marks.values()
                     if mk.check and mk.check == check_id and mk.asked == robot), None)
        if mark is None:
            return  # 모르는 질문이거나, 표시가 이미 지워졌다
        mark.check = 0
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
