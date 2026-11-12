"""관제 반응 시간 상한(추가 지시서 2의 3절). 위반은 불변식 위반 목록에 같이 모은다.

- 9번: 관제가 받은 첫 BLOCKED 보고부터 그 로봇에 대한 첫 관제 사건까지 FIRST_S 안.
- 10번: BLOCKED_WAIT 뒤 로봇이 같은 goal에서 계속 BLOCKED이면, blocked_mark_s + NEXT_GRACE_S 안에
  규칙 7의 다음 단계(NEXT_STAGE 사건, 또는 goal을 거두거나 바꿈)가 나와야 한다.
끊긴 로봇은 관제가 모르므로 보지 않는다. SILENT_STALL 사건은 판정(table.py)이 센다.
"""
from __future__ import annotations

from fleet_core.common.commands import CancelCmd, DockCmd, MoveCmd, RetreatCmd

from .invariants import Violation

FIRST_S = 1.0
NEXT_GRACE_S = 5.0
NEXT_STAGE = ("ZONE_MARKED_BLOCKED", "REROUTE", "RETREAT_START", "HALL_YIELD", "GOAL_FAILED",
              "ROBOT_ISOLATED")
REACT_FIRST, REACT_NEXT = "I9_REACTION_FIRST", "I10_REACTION_NEXT"


class ReactionWatch:
    def __init__(self, params, violations: list):
        self.mark_s = params.blocked_mark_s
        self.violations = violations
        self._first: dict[str, float] = {}  # 로봇 -> 답을 기다리는 첫 BLOCKED 보고 시각
        self._blocked: set[str] = set()  # 지금 BLOCKED로 보고된 로봇
        self._stage: dict[str, tuple[float, int]] = {}  # 로봇 -> (BLOCKED_WAIT 시각, goal)

    def on_report(self, t: float, robot: str, state: str) -> None:
        """관제에 닿은 보고 하나."""
        if state == "BLOCKED":
            if robot not in self._blocked:
                self._blocked.add(robot)
                self._first[robot] = t
            return
        self._blocked.discard(robot)
        self._first.pop(robot, None)
        self._stage.pop(robot, None)

    def on_event(self, t: float, event, goal_id) -> None:
        """관제 사건 하나. goal_id: 그때 그 로봇이 하던 goal(없으면 None)."""
        robot = event.robot
        self._first.pop(robot, None)
        if event.code == "BLOCKED_WAIT" and goal_id is not None:
            self._stage[robot] = (t, goal_id)
        elif event.code in NEXT_STAGE:
            self._stage.pop(robot, None)

    def on_command(self, t: float, cmd) -> None:
        if isinstance(cmd, (CancelCmd, MoveCmd, RetreatCmd, DockCmd)):
            self._stage.pop(cmd.robot, None)

    def check(self, t: float, robots) -> None:
        """robots: 이름 -> SimRobot(바닥 위만)."""
        for name, since in list(self._first.items()):
            r = robots.get(name)
            if r is None or not r.comm_up:
                del self._first[name]
            elif t - since > FIRST_S + 1e-9:
                del self._first[name]
                self._add(t, REACT_FIRST, name, f"{name} BLOCKED at {since:.1f}, no fleet event "
                          f"within {FIRST_S:.0f} s")
        for name, (since, goal) in list(self._stage.items()):
            r = robots.get(name)
            run = r.run if r is not None else None
            if (r is None or not r.comm_up or run is None or not run.blocked
                    or run.goal_id != goal):
                del self._stage[name]
            elif t - since > self.mark_s + NEXT_GRACE_S + 1e-9:
                del self._stage[name]
                self._add(t, REACT_NEXT, name, f"{name} BLOCKED_WAIT at {since:.1f} on goal "
                          f"{goal}, no next step within {self.mark_s + NEXT_GRACE_S:.0f} s")

    def _add(self, t, code, robot, detail) -> None:
        self.violations.append(Violation(t, code, detail, robot))
