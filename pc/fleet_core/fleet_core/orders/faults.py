# ---- 주문 코어의 고장 처리: 로봇 제거, 격리, 팔 고장, 강제 출고, 늦은 팔 결과 ----
from __future__ import annotations

from fleet_core.common.commands import (
    FILL,
    FORCE_NOT_HELD,
    FORCE_ROBOT_FAULT,
    FORCE_UNKNOWN_ORDER,
    HOLD_ARM_FAULT,
    HOLD_ROBOT_FAULT,
    SWAP,
    ArmDropCmd,
    ArmFault,
    ArmResetCmd,
    ArmStopCmd,
)

from .records import (
    EMPTY,
    FILLING,
    FULL,
    NONE,
    SWAPPING,
    LostJob,
    RobotRecord,
    merge_items,
    subtract_items,
)


# ---- 로봇 치움, 다시 투입, 격리와 주문 되돌리기 ----
class Faults:
    def on_robot_removed(self, robot) -> None:
        r = self._robots.get(robot)
        if r is None:
            return
        r.isolated = ""
        if r.asked:
            self._drop_arm(r)
        if r.order and r.order.placed:
            self._hold(r, HOLD_ROBOT_FAULT, "robot cleared")
            r.order = None
        elif r.order:
            self._held.pop(r.order.order_id, None)
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}", robot)
            r.order = None
        r.held = r.paused = ""
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._forget_arm_restart(r)
        self._sync_calls()

    def on_robot_reinstated(self, robot, basket) -> None:
        if robot in self._robots:
            self._robots[robot] = RobotRecord(robot, basket=basket)
            self._out.append(ArmResetCmd(robot, forget=True))

    def on_robot_isolated(self, robot, reason) -> None:
        r = self._robots.get(robot)
        if r is None:
            return
        r.isolated = reason
        # 답 없던 팔 결과를 기다린 뒤 판단한다
        if not r.asked and self._result_still_due(r):
            return
        if r.asked == SWAP and r.order:
            self._out.append(ArmStopCmd(r.name))
            return
        # 담는 중이면 팔 결과를 기다린다(재시작으로 목표가 사라졌으면 올 결과가 없다)
        if r.asked == FILL and not r.arm_restarted:
            return
        if r.asked:
            self._drop_arm(r, isolated=True)
        self._requeue_isolated(r)

    # 담은 것이나 쥐고 있던 상품(uncertain)이 있으면 보류, 없으면 대기로 되돌림
    def _requeue_isolated(self, r: RobotRecord, uncertain=None, held=False) -> None:
        unsure = {k: v for k, v in (uncertain or {}).items() if v > 0}
        # held: 쥐고 있던 상품의 행방을 모름 (쥔 채 락 해제 · 쥔 채 재시작)
        if r.order and (r.order.placed or unsure or held):
            extra = f", uncertain {unsure}" if unsure else ""
            extra += ", item unaccounted" if held else ""
            self._hold(r, HOLD_ROBOT_FAULT, f"robot isolated: {r.isolated}{extra}")
            r.order = None
        elif r.order:
            self._held.pop(r.order.order_id, None)
            self._event("INFO", "ORDER_REQUEUED",
                        f"{r.order.order_id} was bound, basket held {r.order.placed}, "
                        f"robot isolated: {r.isolated}", r.name)
            r.order = None
        r.held = r.paused = ""
        r.basket = NONE
        r.stage = ""
        r.outbound_goal = None
        self._forget_dispatch_failures(r)
        self._forget_arm_restart(r)
        self._sync_calls()

    # 팔 재시작으로 [복구]를 기다리던 로봇이면 팔 쪽 대기도 지운다
    def _forget_arm_restart(self, r: RobotRecord) -> None:
        if r.arm_restarted:
            r.arm_restarted = False
            self._out.append(ArmResetCmd(r.name))

    # ---- 팔 작업 버리기와 결과 대기 판단 ----
    def _drop_arm(self, r: RobotRecord, isolated: bool = False) -> None:
        self._out.append(ArmDropCmd(r.name, isolated, self.order_of(r.name), r.bound))
        r.asked = ""

    def _result_still_due(self, r: RobotRecord) -> bool:
        return any(lost.robot == r.name and not lost.cleared and self._still_bound(lost)
                   for lost in self._lost_jobs.values())

    # ---- 팔 고장: 주문을 보류하고 Pode는 작업대에 붙잡아 둠 ----
    def _on_arm_fault(self, r: RobotRecord, fault: ArmFault) -> None:
        r.asked = ""
        # 격리 중이면 늦은 결과나 치움이 판단한다
        if r.isolated:
            self._decide_if_isolated(r)
            return
        r.held = HOLD_ARM_FAULT
        if r.order:
            self._hold(r, HOLD_ARM_FAULT, fault.reason)

    # ---- 강제 출고: 팔 고장 보류만 받음 ----
    def on_force_ship(self, order_id) -> tuple[bool, str]:
        hold = self._held.get(order_id)
        if hold is None:
            known = (any(o.order_id == order_id for o in self._board)
                     or any(self.order_of(name) == order_id for name in self._robots))
            return False, FORCE_NOT_HELD if known else FORCE_UNKNOWN_ORDER
        robot, reason = hold
        if reason == HOLD_ROBOT_FAULT:
            return False, FORCE_ROBOT_FAULT
        r = self._robots[robot]
        if (r.isolated or not r.held or self.order_of(robot) != order_id
                or r.stage not in (FILLING, SWAPPING)):
            return False, FORCE_NOT_HELD
        del self._held[order_id]
        r.held = ""
        if r.stage == FILLING:
            self._finish_fill(r)
        else:
            self._out.append(ArmResetCmd(r.name))
        return True, ""

    # ---- 늦게 도착한 팔 결과 처리 ----
    def _late_fill(self, goal_id: int, placed: dict[str, int], uncertain=None) -> None:
        lost = self._take_lost_job(goal_id, FILL)
        unsure = {k: v for k, v in (uncertain or {}).items() if v > 0}
        if lost is None or (lost.cleared and not placed and not unsure):
            return
        self._late_event(goal_id, lost, f"placed {placed}" + (f", uncertain {unsure}"
                                                              if unsure else ""))
        if self._still_bound(lost):
            r = self._robots[lost.robot]
            merge_items(r.order.placed, placed)
            r.order.missing = subtract_items(r.order.items, r.order.placed)
            self._decide_if_isolated(r, unsure)

    def _late_swap(self, goal_id: int, took_full: bool, put_empty_done: bool) -> None:
        lost = self._take_lost_job(goal_id, SWAP)
        if lost is None or (lost.cleared and not (took_full or put_empty_done)):
            return
        self._late_event(goal_id, lost, f"took_full={took_full} put_empty_done={put_empty_done}")
        r = self._robots[lost.robot]
        if self._still_bound(lost):
            if took_full and r.basket == FULL:
                r.basket = EMPTY if put_empty_done else NONE
                self._report_shipped(r)
            self._decide_if_isolated(r)

    # 격리 중 기다리던 결과가 오면 판단
    def _decide_if_isolated(self, r: RobotRecord, uncertain=None) -> None:
        if r.isolated and not r.asked and not self._result_still_due(r):
            self._requeue_isolated(r, uncertain)

    def _take_lost_job(self, goal_id: int, kind: str) -> LostJob | None:
        lost = self._lost_jobs.get(goal_id)
        if lost is None or lost.kind != kind:
            return None
        return self._lost_jobs.pop(goal_id)

    def _still_bound(self, lost: LostJob) -> bool:
        return (bool(lost.order_id) and self.order_of(lost.robot) == lost.order_id
                and self._robots[lost.robot].bound == lost.bound)

    # ---- 늦은 결과 경고 이벤트 ----
    def _late_event(self, goal_id: int, lost: LostJob, reported: str) -> None:
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
