"""OrderOrchestrator와 ArmJobs를 FleetManager처럼 교통 없이 이은 것: 둘이 서로 하는 말은 바로 전하고 나머지는 take_output()으로 나온다."""
from fleet_core.arm.jobs import ArmJobs
from fleet_core.common.commands import ARM_COMMANDS, ARM_REPORTS
from fleet_core.orders.orchestrator import OrderOrchestrator

_ORDER_INPUTS = ("on_dispatch_feedback", "on_dispatch_result", "on_robot_removed",
                 "on_robot_reinstated", "on_robot_isolated", "on_force_ship", "step")


class OrdersAndArm:
    def __init__(self, stations, params, robots, clock):
        self.orders = OrderOrchestrator(stations, params, robots, clock)
        self.arm = ArmJobs(stations, params, robots, clock)
        self._out = []

    def __getattr__(self, name):
        found = getattr(self.orders, name)
        if name not in _ORDER_INPUTS:
            return found

        def call(*args, **kwargs):
            answer = found(*args, **kwargs)
            self._settle()
            return answer
        return call

    def on_order_board(self, run_mode, orders) -> None:
        self.arm.on_run_mode(run_mode)
        self.orders.on_order_board(run_mode, orders)
        self._settle()

    def on_fill_result(self, *args) -> None:
        self.arm.on_fill_result(*args)
        self._settle()

    def on_swap_result(self, *args) -> None:
        self.arm.on_swap_result(*args)
        self._settle()

    def on_arm_state(self, *args) -> None:
        self.arm.on_arm_state(*args)
        self._settle()

    def on_arm_state_lost(self, arm) -> None:
        self.arm.on_arm_state_lost(arm)
        self._settle()

    def on_fill_feedback(self, *args) -> None:
        self.arm.on_fill_feedback(*args)
        self._settle()

    def on_resume_arm(self, target) -> tuple[str, str]:
        answer = self.arm.on_resume_arm(target)
        self._settle()
        return answer

    def arm_task(self, arm) -> tuple[str, str]:
        return self.arm.arm_task(arm)

    def check_arm_timeouts(self, now) -> None:
        self.arm.check_timeouts(now)
        self._settle()

    def take_output(self) -> list:
        out, self._out = self._out, []
        return out

    def robot_view(self, robot) -> dict:
        view = self.orders.robot_view(robot)
        return {**view, "attempts": self.arm.attempts(robot)} if view else view

    def _settle(self) -> None:
        for item in self.arm.take_output() + self.orders.take_output():
            self._route(item)

    def _route(self, item) -> None:
        """FleetManager._route와 같다: 말을 받은 쪽이 다음 항목 전에 답한다."""
        if isinstance(item, ARM_COMMANDS):
            self.arm.on_command(item)
            for answer in self.arm.take_output():
                self._route(answer)
        elif isinstance(item, ARM_REPORTS):
            self.orders.on_arm_report(item)
            for answer in self.orders.take_output():
                self._route(answer)
        else:
            self._out.append(item)
