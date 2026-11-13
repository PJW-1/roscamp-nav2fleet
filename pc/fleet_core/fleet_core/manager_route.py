# ---- 코어 사이 명령과 보고를 전달하는 ManagerRoute ----
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


# ---- 로봇 한 대 진행과 코어 출력 모으기 ----
class ManagerRoute:
    def _step_robot(self, robot) -> None:
        self.orchestrator.step(robot)
        self._pump()
        self.traffic.step(robot)
        self._pump()
        self.orchestrator.step(robot)
        self._pump()

    def _pump(self) -> None:
        while True:
            items = (self.arm.take_output() + self.orchestrator.take_output()
                     + self.traffic.take_output())
            if not items:
                return
            for item in items:
                self._route(item)

    # ---- 출력 항목을 종류별로 알맞은 코어에 전달 ----
    def _route(self, item) -> None:
        if isinstance(item, ARM_COMMANDS):
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
