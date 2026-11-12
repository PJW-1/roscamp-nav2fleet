"""움직이기 시작하지 않음(NO_START, 추가 지시서 2의 6절 질문 2).

관제가 도킹이 아닌 goal(이동, 후진, 마당 물러나기)을 보낸 뒤 start_limit_s 안에 보고 위치가
START_MOVE_M 넘게 바뀌지 않았고 goal이 아직 열려 있으면 그 goal에 NO_START를 한 번 낸다. 원인
분류(tools/triage.py)가 "주행" 탓을 가리는 근거다. 알리기만 하고 goal은 그대로 둔다(제자리 멈춤은
watch.py가 stall_s 뒤에 거둔다).

재는 시작점은 관제가 그 goal을 처음 본 주기의 보고 위치다(보낸 다음 주기, 0.1 s 늦을 수 있다).
보지 않는 goal:

- 도킹: 천천히 맞추며 dock_stall_s가 본다.
- 끝 지점이 이미 START_MOVE_M 안: 그 자리에서 돌기만 한다.
- 시작 전에 BLOCKED나 RECOVERING을 보고함: 막힘으로 답했다(blocked.py, 원인 분류 질문 3).
- 성공을 받고 자세를 기다리는 중, 로봇이 쓸 수 없음(지도 밖, 고장, 통신), 관제를 세움: 다시
  쓸 수 있게 되면 그때부터 잰다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.mission import BLOCKED, DOCK, RUN, SELF_RECOVERING

START_MOVE_M = 0.02  # 이보다 더 움직여야 시작한 것이다(지시서 값)


class NoStart:
    def _watch_start(self, m, now: float) -> None:
        p, goal = self.params, m.goal
        if (p.start_limit_s <= 0 or not m.available or self._run_mode != RUN or goal is None
                or goal.kind == DOCK or m.arrival_by is not None):
            m.start_goal = 0
            return
        here = (m.x, m.y)
        if m.start_goal != goal.id:
            m.start_goal, m.start_xy, m.start_since = goal.id, here, now
            m.start_done = (goal.end in self.site.nodes
                            and self._gap(m, goal.end) <= START_MOVE_M)
            return
        if m.start_done:
            return
        if m.robot_state in (BLOCKED, SELF_RECOVERING) or math.dist(m.start_xy, here) > (
                START_MOVE_M):
            m.start_done = True
        elif now - m.start_since >= p.start_limit_s:
            m.start_done = True
            self._event("WARN", "NO_START", f"{goal.kind} goal {goal.id} to {goal.end}: moved "
                        f"less than {START_MOVE_M} m in {p.start_limit_s:.0f} s", m.name)
