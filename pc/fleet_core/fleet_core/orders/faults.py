# ---- 주문 코어의 고장 처리: 로봇 제거, 격리, 늦은 팔 결과 ----
from __future__ import annotations

from fleet_core.common.commands import FILL, SWAP, ArmDropCmd, ArmResetCmd, ArmStopCmd

from .records import EMPTY, FULL, NONE, _LostJob, _merge, _Robot


# ---- 로봇 치움, 다시 투입, 격리와 주문 되돌리기 ----
class Faults:
    def on_robot_removed(self, robot) -> None:
        r = self._robots.get(robot)
        if r is None:
            return
        r.isolated = ""
        if r.asked:
            self._drop_arm(r)
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}", robot)
            r.order = None
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._sync_calls()

    def on_robot_reinstated(self, robot, basket) -> None:
        if robot in self._robots:
            self._robots[robot] = _Robot(robot, basket=basket)
            self._out.append(ArmResetCmd(robot, forget=True))

    def on_robot_isolated(self, robot, reason) -> None:
        r = self._robots.get(robot)
        if r is None or self._result_still_due(r):
            return
        r.isolated = reason
        if r.asked == SWAP and r.order:
            self._out.append(ArmStopCmd(r.name))
            return
        if r.asked:
            self._drop_arm(r, isolated=True)
        self._requeue_isolated(r)

    def _requeue_isolated(self, r: _Robot) -> None:
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}, "
                        f"robot isolated: {r.isolated}", r.name)
            r.order = None
        r.basket = NONE
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._sync_calls()

    # ---- 팔 작업 버리기와 결과 대기 판단 ----
    def _drop_arm(self, r: _Robot, isolated: bool = False) -> None:
        self._out.append(ArmDropCmd(r.name, isolated, self.order_of(r.name), r.bound))
        r.asked = ""

    def _result_still_due(self, r: _Robot) -> bool:
        return any(lost.robot == r.name and not lost.cleared and self._still_bound(lost)
                   for lost in self._lost_jobs.values())

    # ---- 늦게 도착한 팔 결과 처리 ----
    def _late_fill(self, goal_id: int, placed: dict[str, int]) -> None:
        lost = self._take_lost_job(goal_id, FILL)
        if lost is None or (lost.cleared and not placed):
            return
        self._late_event(goal_id, lost, f"placed {placed}")
        if self._still_bound(lost):
            _merge(self._robots[lost.robot].order.placed, placed)

    def _late_swap(self, goal_id: int, took_full: bool, put_empty_done: bool) -> None:
        lost = self._take_lost_job(goal_id, SWAP)
        if lost is None or (lost.cleared and not (took_full or put_empty_done)):
            return
        self._late_event(goal_id, lost, f"took_full={took_full} put_empty_done={put_empty_done}")
        r = self._robots[lost.robot]
        if self._still_bound(lost) and took_full and r.basket == FULL:
            r.basket = EMPTY if put_empty_done else NONE
            self._report_shipped(r)

    def _take_lost_job(self, goal_id: int, kind: str) -> _LostJob | None:
        lost = self._lost_jobs.get(goal_id)
        if lost is None or lost.kind != kind:
            return None
        return self._lost_jobs.pop(goal_id)

    def _still_bound(self, lost: _LostJob) -> bool:
        return (bool(lost.order_id) and self.order_of(lost.robot) == lost.order_id
                and self._robots[lost.robot].bound == lost.bound)

    # ---- 늦은 결과 경고 이벤트 ----
    def _late_event(self, goal_id: int, lost: _LostJob, reported: str) -> None:
        if not lost.order_id:
            then = "its order had already been reported"
        elif self._still_bound(lost):
            then = f"{lost.order_id} is still bound and {lost.robot} stays in fault"
        else:
            then = (f"{lost.order_id} was requeued from {lost.robot}, so it may be filled "
                    "or shipped twice: check the basket")
        after = ("the isolation" if lost.isolated else "the clear" if lost.cleared
                 else "ARM_NO_RESULT")
        self._event("WARN", "ARM_LATE_RESULT",
                    f"{lost.server} goal {goal_id} reported {reported} after {after}, {then}",
                    lost.robot)
