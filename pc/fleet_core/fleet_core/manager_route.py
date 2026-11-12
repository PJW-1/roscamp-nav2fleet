"""FleetManager의 내부 배달: 세 코어가 서로 보내는 명령을 받을 곳으로 넘긴다."""
from __future__ import annotations

from fleet_core.common.commands import (
    ARM_COMMANDS,
    ARM_REPORTS,
    CancelDispatchCmd,
    DispatchCmd,
    DispatchFeedback,
    DispatchResult,
    ReleaseCmd,
    RobotFaultCmd,
    RobotIsolated,
)


class ManagerRoute:
    def _step_robot(self, robot) -> None:
        """주문, 교통 순으로 돌고, 방금 묶인 주문을 위해 주문 쪽을 한 번 더 돈다."""
        self.orchestrator.step(robot)
        self._pump()
        self.traffic.step(robot)
        self._pump()
        self.orchestrator.step(robot)
        self._pump()

    def _pump(self) -> None:
        """세 코어 모두 보낼 명령이 없을 때까지 내부 명령을 넘긴다."""
        while True:
            items = (self.arm.take_output() + self.orchestrator.take_output()
                     + self.traffic.take_output())
            if not items:
                return
            for item in items:
                self._route(item)

    def _route(self, item) -> None:
        if isinstance(item, ARM_COMMANDS):
            # Orders and arm answer each other in place, before the next item: what comes out
            # keeps the order it had when one core did both.
            self._note("inside", item)
            self.arm.on_command(item)
            for answer in self.arm.take_output():
                self._route(answer)
        elif isinstance(item, ARM_REPORTS):
            self._note("inside", item)
            self.orchestrator.on_arm_report(item)
            for answer in self.orchestrator.take_output():
                self._route(answer)
        elif isinstance(item, DispatchCmd):
            self._note("inside", item)
            self.traffic.on_dispatch(item.goal_id, item.station_id, item.robot_id)
        elif isinstance(item, CancelDispatchCmd):
            self._note("inside", item)
            self.traffic.on_dispatch_cancel(item.goal_id)
        elif isinstance(item, ReleaseCmd):
            self._note("inside", item)
            self.traffic.on_release(item.robot)
        elif isinstance(item, RobotFaultCmd):
            self._note("inside", item)
            self.traffic.on_robot_fault(item.robot, item.reason)
        elif isinstance(item, RobotIsolated):
            self._note("inside", item)
            self.orchestrator.on_robot_isolated(item.robot, item.reason)
        elif isinstance(item, DispatchFeedback):
            self._note("inside", item)
            self.orchestrator.on_dispatch_feedback(item.goal_id, item.robot_id, item.phase)
        elif isinstance(item, DispatchResult):
            self._note("inside", item)
            self.orchestrator.on_dispatch_result(item.goal_id, item.success, item.robot_id,
                                                 item.reason)
        else:
            self._out.append(item)
