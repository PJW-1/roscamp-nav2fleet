"""로봇을 빼거나 격리하거나 다시 넣을 때의 주문 처리, 그리고 놓친 팔 작업의 늦은 결과."""
from __future__ import annotations

from fleet_core.common.commands import FILL, SWAP, ArmDropCmd, ArmResetCmd, ArmStopCmd

from .records import EMPTY, FULL, NONE, _LostJob, _merge, _Robot


class Faults:
    # ---- removed and isolated robots, arms, late results -------------------------------------

    def on_robot_removed(self, robot) -> None:
        """운영자가 로봇을 뺐다. 아직 묶인 주문은 등록 순서 자리로 돌아간다.

        격리된 로봇은 대개 주문을 이미 되돌렸다. 예외 둘은 여기서 되돌린다: 교환 취소 결과를
        기다리던 주문(그 뒤 took_full이 와도 ARM_LATE_RESULT만 낸다), ARM_NO_RESULT로 쥔 주문.
        """
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
        """운영자가 로봇을 다시 넣었다. 바구니는 운영자가 본 값, 주문 없음, 재시도 0."""
        if robot in self._robots:
            self._robots[robot] = _Robot(robot, basket=basket)
            self._out.append(ArmResetCmd(robot, forget=True))

    def on_robot_isolated(self, robot, reason) -> None:
        """교통이 로봇을 격리했다(지시서 5.5). 주문은 사람 없이 대기로 돌아간다.

        자리는 등록 순서다. 묶을 때 늘 대기 맨 앞을 쓰므로, 되돌린 주문은 한 번도 묶이지 않은
        주문보다 앞서고 되돌린 주문끼리는 등록 순서를 지킨다.
        두 번 출고를 막는 예외 둘: 결과 없는 팔 작업(ARM_NO_RESULT)이 그 주문에 걸려 있으면
        confirm_clear까지, 교환 중이면 그 결과까지 주문을 둔다. 팔이 이미 바구니를 가져갔을 수 있다.
        """
        r = self._robots.get(robot)
        if r is None or self._result_still_due(r):
            return
        r.isolated = reason
        if r.asked == SWAP and r.order:
            self._out.append(ArmStopCmd(r.name))
            return  # the swap result or the ARM_NO_RESULT rule takes over
        if r.asked:
            self._drop_arm(r, isolated=True)  # 늦게 담긴 품목은 이 로봇 바구니에 남는다
        self._requeue_isolated(r)

    def _requeue_isolated(self, r: _Robot) -> None:
        if r.order:
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}, "
                        f"robot isolated: {r.isolated}", r.name)
            r.order = None
        r.basket = NONE  # 바구니 안 물건은 로봇과 함께 남는다. 사람이 치운다
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._sync_calls()

    def _drop_arm(self, r: _Robot, isolated: bool = False) -> None:
        """Drops the arm job on a clear or an isolation; the operator counts what the arm did there.

        The arm side cancels the goal and answers with ArmLost, which carries the order bound
        now: a late result that did something is then reported.
        """
        self._out.append(ArmDropCmd(r.name, isolated, self.order_of(r.name), r.bound))
        r.asked = ""

    def _result_still_due(self, r: _Robot) -> bool:
        """ARM_NO_RESULT로 포기한 작업이 지금 묶인 주문에 걸려 있다."""
        return any(lost.robot == r.name and not lost.cleared and self._still_bound(lost)
                   for lost in self._lost_jobs.values())

    def _late_fill(self, goal_id: int, placed: dict[str, int]) -> None:
        lost = self._take_lost_job(goal_id, FILL)
        if lost is None or (lost.cleared and not placed):
            return  # 취소된 작업이 한 일이 없으면 운영자가 셀 것도 없다
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
        """고장 중에는 해제나 격리만 주문을 떼어 간다. 해제 뒤 다시 묶인 같은 주문은 다른 일이다."""
        return (bool(lost.order_id) and self.order_of(lost.robot) == lost.order_id
                and self._robots[lost.robot].bound == lost.bound)

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
