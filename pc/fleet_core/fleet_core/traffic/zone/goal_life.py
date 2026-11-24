# ---- goal을 바꾸는 입구 믹스인 ----
from __future__ import annotations

from fleet_core.common.commands import CancelCmd


# ---- goal 정하기, 지우기, 취소와 멈춤 감시 초기화 ----
class GoalLife:
    def _set_goal(self, m, goal, *, reset_watch: bool = True) -> None:
        m.goal = goal
        if reset_watch:
            self._reset_stall(m)

    def _clear_goal(self, m) -> None:
        m.goal = None
        m.result_lost_at = None

    def _reset_stall(self, m) -> None:
        m.watch.stall.clear()

    def _cancel_goal(self, m) -> None:
        if m.goal is None:
            return
        self._out.append(CancelCmd(m.name, m.goal.id))
        if m.move_order is not None and m.move_order.before is not None:
            self._out.append(CancelCmd(m.name, m.move_order.before[0]))
        self._clear_goal(m)
        m.move_order = None
        m.arrival_by = None
        m.settle_until = self._clock() + self.params.arrival_settle_s
        if m.route is not None:
            m.route.sent = m.route.done
