"""FakeArm: 출고 팔. 트레이의 찬 바구니를 내리고 빈 바구니를 올린다."""
from __future__ import annotations

import random
import threading
import time
import uuid

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import SwapBasket
from fleet_interfaces.msg import ArmState
from fleet_interfaces.srv import SetMode
from pode_fsm.admin import AdminState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fake_robots.common import (
    ScenarioClock,
    load_contract_param,
    load_scenario_param,
    load_site_param,
    node_name,
    station_with_action,
)
from fake_robots.motion import ArmFaults, abort_at_shutdown, finish_action

ERROR_HOLD_S = 2.0
REASON_ARM_FAULT = "ARM_FAULT"
REASON_PUT_EMPTY = "PUT_EMPTY_FAILED"
REASON_CANCELED = "CANCELED"
STATE_IDLE = "IDLE"
STATE_SWAPPING = "SWAPPING"
STATE_ERROR = "ERROR"
STATE_PERIOD_S = 0.2  # IR-34: 초당 5번
RESULT_FIRST_S = 0.1  # 실제 초. 고장 결과가 관제에 먼저 닿은 뒤 ERROR를 보인다


class FakeArmNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__(node_name("fake_arm", "arm_id", parameter_overrides),
                         parameter_overrides=parameter_overrides, **kwargs)
        self.declare_parameter("arm_id", "arm_ship")
        self.arm_id = self.get_parameter("arm_id").value
        contract = load_contract_param(self)
        site = load_site_param(self)
        scenario = load_scenario_param(self, site)
        if self.arm_id not in station_with_action(site, "swap").arms:
            raise ValueError(f"{self.arm_id} does not swap baskets, FakeWorkcell runs the others")
        self._cfg = scenario.arm
        self._faults = ArmFaults(scenario.faults)
        self._rng = random.Random(f"{scenario.seed}-{self.arm_id}")
        self._sc = ScenarioClock(self)
        self._lock = threading.Lock()
        self._alive = True
        self._busy = False
        self._state, self._error_until = STATE_IDLE, 0.0
        self._boot_id = str(uuid.uuid4())
        self._order_id = ""
        # pode_fsm과 같은 규칙의 운영 모드와 요청 번호 기억(IR-72)
        self._admin = AdminState(self.arm_id, self._boot_id)
        self.create_service(SetMode, contract.name("robot_set_mode", robot=self.arm_id),
                            self._on_set_mode)
        self._state_pub = self.create_publisher(
            ArmState, contract.name("arm_state", arm=self.arm_id),
            to_ros_qos(contract.qos("arm_state")))
        self.create_timer(STATE_PERIOD_S, self._publish_state)

        self._server = ActionServer(
            self, SwapBasket, contract.name("swap_basket", arm=self.arm_id),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(0.5, self._tick)

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        self._server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    def _on_goal(self, _request):
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _tick(self) -> None:
        if self._state == STATE_ERROR and self._sc.now() >= self._error_until:
            self._state = STATE_IDLE

    def _on_set_mode(self, request, response):
        with self._lock:
            reply = self._admin.set_mode(request.request_id, request.mode, time.monotonic())
        response.accepted = reply.accepted
        response.reason = reply.reason
        response.detail = reply.detail
        response.request_id = request.request_id
        return response

    def _show_error(self) -> None:
        self._error_until = self._sc.now() + ERROR_HOLD_S
        self._state = STATE_ERROR

    def _publish_state(self) -> None:
        # Phony는 락이 없어 lock은 늘 true, 모드는 set_mode 전까지 UNKNOWN(IR-72)
        state = self._state
        with self._lock:
            mode, last_id = self._admin.mode, self._admin.last_request_id
        self._state_pub.publish(ArmState(
            arm_id=self.arm_id, boot_id=self._boot_id, state=state,
            detail=REASON_ARM_FAULT if state == STATE_ERROR else "", lock=True,
            gripper=ArmState.GRIPPER_OPEN, order_id=self._order_id, mode=mode,
            last_request_id=last_id, stamp=self.get_clock().now().to_msg()))

    def _failure_step(self, steps: list[str]) -> tuple[str | None, str]:
        """이 목표에서 실패할 단계와 이유. swap_partial은 항상 빈 바구니 단계를 친다."""
        if "PUT_EMPTY" in steps and self._faults.take_swap_partial(self._sc.now()):
            return "PUT_EMPTY", REASON_PUT_EMPTY
        if steps and self._rng.random() < self._cfg.fail_rate:
            return self._rng.choice(steps), REASON_ARM_FAULT
        return None, ""

    def _execute(self, handle):
        req = handle.request
        self._order_id = req.order_id
        steps = (["TAKE_FULL"] if req.take_full else []) + (["PUT_EMPTY"] if req.put_empty else [])
        failing, fail_reason = self._failure_step(steps)
        done, reason = set(), ""
        step_s = self._cfg.swap_seconds / 2

        def keep_going():
            return self._running() and not handle.is_cancel_requested

        for step in steps:
            if not keep_going():
                reason = REASON_CANCELED
                break
            self._state = STATE_SWAPPING
            handle.publish_feedback(SwapBasket.Feedback(phase=step))
            if not self._sc.sleep_until(self._sc.now() + step_s, keep_going):
                reason = REASON_CANCELED
                break
            if step == failing:
                reason = fail_reason
                break
            done.add(step)
        self._state = STATE_IDLE
        success = not reason
        result = SwapBasket.Result(
            success=success, took_full="TAKE_FULL" in done, put_empty_done="PUT_EMPTY" in done,
            reason=reason)
        self._order_id = ""
        with self._lock:
            self._busy = False
        if self._running():
            finish_action(handle, success)
        else:
            abort_at_shutdown(handle)
        if failing is not None and reason == fail_reason:
            # 재시도할 고장으로 받도록 결과가 나간 뒤 ERROR를 보인다
            timer = threading.Timer(RESULT_FIRST_S, self._show_error)
            timer.daemon = True
            timer.start()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeArmNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # SIGINT가 메시지 수신 중에 떨어질 수 있다. 진짜 오류면 ROS가 아직 살아 있다
        if rclpy.ok():
            raise
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
