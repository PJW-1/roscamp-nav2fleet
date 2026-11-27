"""FakeWorkcell: arm_left, arm_right를 뒤에 둔 fill_basket 서버.

요청한 물건은 그 종류의 재고가 있는 팔에 간다. 두 팔이 동시에 집지만
잠금 하나로 바구니 공간(PLACING)에는 한 팔만 들어간다.
"""
from __future__ import annotations

import json
import random
import threading
import time
import uuid
from collections import Counter

import rclpy
from fleet_core.common.contract import to_ros_qos
from fleet_interfaces.action import FillBasket
from fleet_interfaces.msg import ArmState, OrderItem
from fleet_interfaces.srv import SetMode
from pode_fsm.admin import MODE_MISMATCH, AdminState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fake_robots.common import (
    STOCK_TOPIC,
    ScenarioClock,
    load_contract_param,
    load_scenario_param,
    load_site_param,
    station_with_action,
)
from fake_robots.motion import ArmFaults, abort_at_shutdown, finish_action

ERROR_HOLD_S = 2.0
REASON_ARM_FAULT = "ARM_FAULT"
REASON_CANCELED = "CANCELED"
LOCK_POLL_S = 0.01
STATE_IDLE = "IDLE"
STATE_PICKING = "PICKING"
STATE_PLACING = "PLACING"
STATE_ERROR = "ERROR"
STATE_PERIOD_S = 0.2  # IR-34: 초당 5번
RESULT_FIRST_S = 0.1  # 실제 초. 고장 결과가 관제에 먼저 닿은 뒤 ERROR를 보인다

STOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def order_items(counts: Counter) -> list[OrderItem]:
    return [OrderItem(kind=kind, count=count) for kind, count in sorted(counts.items()) if count]


class _Run:
    """팔 스레드가 처리하는 동안의 채움 목표 하나."""

    def __init__(self, handle):
        self.handle = handle
        self.placed: Counter = Counter()
        self.uncertain: Counter = Counter()  # 놓다가 끊겨 쥔 채 남은 상품
        self.errored: list[str] = []  # 이 목표에서 고장 난 팔
        self.fault = threading.Event()
        self.lock = threading.Lock()


class FakeWorkcellNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("fake_workcell", parameter_overrides=parameter_overrides, **kwargs)
        # true면 실물 Phody처럼 두 팔 모드가 UNKNOWN이거나 서로 다를 때 담기를 MODE_MISMATCH로 끝낸다.
        # 가짜 실행에는 /system/mode를 내는 브리지가 없어 기본은 false
        self.declare_parameter("mode_gate", False)
        self._mode_gate = bool(self.get_parameter("mode_gate").value)
        contract = load_contract_param(self)
        site = load_site_param(self)
        scenario = load_scenario_param(self, site)
        self._arms = list(station_with_action(site, "fill").arms)
        self._cfg = scenario.arm
        self._faults = ArmFaults(scenario.faults)
        self._rng = random.Random(f"{scenario.seed}-workcell")
        self._sc = ScenarioClock(self)
        self._stock = {arm: Counter(scenario.stock.get(arm, {})) for arm in self._arms}
        self._stock_lock = threading.Lock()
        self._busy_lock = threading.Lock()
        self._busy = False
        self._alive = True
        # 실제 서버도 지킬 규칙: 이 잠금을 쥔 쪽만 PLACING이 될 수 있다.
        self._basket_space = threading.Lock()
        self._state_lock = threading.Lock()
        self._state = dict.fromkeys(self._arms, STATE_IDLE)
        self._error_until = dict.fromkeys(self._arms, 0.0)
        self._cancel_at: float | None = None
        self._boot_id = str(uuid.uuid4())
        self._order_id = ""
        self._placed = dict.fromkeys(self._arms, 0)
        # 팔마다 pode_fsm과 같은 규칙의 운영 모드와 요청 번호 기억(IR-72)
        self._admin = {arm: AdminState(arm, self._boot_id) for arm in self._arms}
        self._admin_lock = threading.Lock()
        for arm in self._arms:
            self.create_service(SetMode, contract.name("robot_set_mode", robot=arm),
                                lambda req, res, a=arm: self._on_set_mode(a, req, res))
        self._state_pubs = {
            arm: self.create_publisher(ArmState, contract.name("arm_state", arm=arm),
                                       to_ros_qos(contract.qos("arm_state")))
            for arm in self._arms}
        self.create_timer(STATE_PERIOD_S, self._publish_states)

        self._stock_pubs = {
            arm: self.create_publisher(String, STOCK_TOPIC.format(arm=arm), STOCK_QOS)
            for arm in self._arms}
        self._server = ActionServer(
            self, FillBasket, contract.name("fill_basket"),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(0.5, self._tick)
        for arm in self._arms:
            self._publish_stock(arm)

    def stop(self) -> None:
        self._alive = False

    def destroy_node(self):
        self.stop()
        self._server.destroy()
        return super().destroy_node()

    def _running(self) -> bool:
        return self._alive and rclpy.ok()

    # ---- 상태와 재고 ----

    def _set_state(self, arm: str, state: str) -> None:
        with self._state_lock:
            self._state[arm] = state

    def _tick(self) -> None:
        t = self._sc.now()
        for arm in self._arms:
            if self._state[arm] == STATE_ERROR and t >= self._error_until[arm]:
                self._set_state(arm, STATE_IDLE)

    def _on_set_mode(self, arm, request, response):
        with self._admin_lock:
            reply = self._admin[arm].set_mode(request.request_id, request.mode,
                                              time.monotonic())
        response.accepted = reply.accepted
        response.reason = reply.reason
        response.detail = reply.detail
        response.request_id = request.request_id
        return response

    def _modes_match(self) -> bool:
        with self._admin_lock:
            modes = {self._admin[arm].mode for arm in self._arms}
        return len(modes) == 1 and "UNKNOWN" not in modes

    def _show_errors(self, arms: list[str]) -> None:
        for arm in arms:
            self._error_until[arm] = self._sc.now() + ERROR_HOLD_S
            self._set_state(arm, STATE_ERROR)

    def _publish_states(self) -> None:
        # 두 팔 모두 락 잠김, 모드는 set_mode 전까지 UNKNOWN(IR-72). 놓는 동안 상품을 쥐고 있다
        stamp = self.get_clock().now().to_msg()
        for arm in self._arms:
            state = self._state[arm]
            holding = state == STATE_PLACING
            with self._admin_lock:
                mode, last_id = self._admin[arm].mode, self._admin[arm].last_request_id
            self._state_pubs[arm].publish(ArmState(
                arm_id=arm, boot_id=self._boot_id, state=state,
                detail=REASON_ARM_FAULT if state == STATE_ERROR else "", lock=True,
                gripper=ArmState.GRIPPER_CLOSED if holding else ArmState.GRIPPER_OPEN,
                holding=holding, order_id=self._order_id, placed_total=self._placed[arm],
                mode=mode, last_request_id=last_id, stamp=stamp))

    def _publish_stock(self, arm: str) -> None:
        with self._stock_lock:
            data = json.dumps({k: c for k, c in self._stock[arm].items() if c > 0})
        self._stock_pubs[arm].publish(String(data=data))

    def _change_stock(self, arm: str, kind: str, delta: int) -> None:
        with self._stock_lock:
            self._stock[arm][kind] += delta
        self._publish_stock(arm)

    # ---- 계획 ----

    def _plan(self, items) -> tuple[dict[str, list[str]], Counter]:
        """요청한 물건을 그 종류가 가장 많은 팔에 준다. 나머지는 missing이다."""
        with self._stock_lock:
            left = {arm: Counter(self._stock[arm]) for arm in self._arms}
        units: dict[str, list[str]] = {arm: [] for arm in self._arms}
        missing: Counter = Counter()
        for item in items:
            for _ in range(max(item.count, 0)):
                arm = max(self._arms, key=lambda a: left[a][item.kind])
                if left[arm][item.kind] <= 0:
                    missing[item.kind] += 1
                    continue
                left[arm][item.kind] -= 1
                units[arm].append(item.kind)
        return units, missing

    def _plan_failures(self, units: dict[str, list[str]]) -> dict[str, int]:
        """이 목표에서 실패할 팔과, 그 팔의 몇 번째 물건 전인지."""
        t = self._sc.now()
        fail_at = {}
        for arm in self._arms:
            at = self._faults.take_arm_fail(arm, t, len(units[arm]))
            if at is not None:
                fail_at[arm] = at
        planned = [(arm, i) for arm in self._arms for i in range(len(units[arm]))]
        if planned and self._rng.random() < self._cfg.fail_rate:
            arm, i = self._rng.choice(planned)
            fail_at[arm] = min(i, fail_at.get(arm, i))
        return fail_at

    # ---- 목표 ----

    def _on_goal(self, _request):
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _handle):
        if self._cancel_at is None:
            self._cancel_at = self._sc.now()
        return CancelResponse.ACCEPT

    def _mode_mismatch(self, handle):
        requested = Counter()
        for item in handle.request.items:
            requested[item.kind] += max(item.count, 0)
        with self._busy_lock:
            self._busy = False
        if self._running():
            handle.abort()
        else:
            abort_at_shutdown(handle)
        return FillBasket.Result(success=False, placed=[], missing=order_items(requested),
                                 reason=MODE_MISMATCH, placed_confirmed=[], uncertain=[])

    def _execute(self, handle):
        if self._mode_gate and not self._modes_match():
            return self._mode_mismatch(handle)
        self._cancel_at = None
        self._order_id = handle.request.order_id
        self._placed = dict.fromkeys(self._arms, 0)
        units, missing = self._plan(handle.request.items)
        fail_at = self._plan_failures(units)
        run = _Run(handle)
        workers = [threading.Thread(target=self._work,
                                    args=(run, arm, units[arm], fail_at.get(arm)), daemon=True)
                   for arm in self._arms if units[arm]]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        if handle.is_cancel_requested and self._cancel_at is not None:
            # 실제 팔은 멈추는 데 시간이 걸리고 그동안 서버는 바쁘다.
            self._sc.sleep_until(self._cancel_at + self._cfg.cancel_seconds, self._running)

        planned = sum(len(u) for u in units.values())
        complete = sum(run.placed.values()) == planned
        success = complete and not run.fault.is_set()
        reason = ""
        if run.fault.is_set():
            reason = REASON_ARM_FAULT
        elif not complete:
            reason = REASON_CANCELED
        # `missing`은 서버가 시도를 마쳤을 때만 확정이다. 재시도하면 다시 계산한다.
        result = FillBasket.Result(
            success=success, placed=order_items(run.placed),
            missing=order_items(missing) if success else [], reason=reason,
            placed_confirmed=order_items(run.placed), uncertain=order_items(run.uncertain))
        self._order_id = ""
        with self._busy_lock:
            self._busy = False
        if self._running():
            finish_action(handle, success)
        else:
            abort_at_shutdown(handle)
        if run.errored:
            # 재시도할 고장으로 받도록 결과가 나간 뒤 ERROR를 보인다
            timer = threading.Timer(RESULT_FIRST_S, self._show_errors, (run.errored,))
            timer.daemon = True
            timer.start()
        return result

    def _go_on(self, run: _Run) -> bool:
        return (self._running() and not run.fault.is_set()
                and not run.handle.is_cancel_requested)

    def _work(self, run: _Run, arm: str, kinds: list[str], fail_at: int | None) -> None:
        half = self._cfg.seconds_per_item / 2
        for i, kind in enumerate(kinds):
            if not self._go_on(run):
                break
            if fail_at == i:
                run.fault.set()
                run.errored.append(arm)
                return
            self._set_state(arm, STATE_PICKING)
            if not self._sc.sleep_until(self._sc.now() + half, lambda: self._go_on(run)):
                break
            self._change_stock(arm, kind, -1)
            if not self._enter_basket_space(run):
                self._change_stock(arm, kind, +1)  # 바구니에 못 닿았으니 선반으로 되돌린다
                break
            try:
                self._set_state(arm, STATE_PLACING)
                placed = self._sc.sleep_until(self._sc.now() + half, self._running)
                self._set_state(arm, STATE_IDLE)
            finally:
                self._basket_space.release()
            if not placed:
                with run.lock:
                    run.uncertain[kind] += 1
                return
            with run.lock:
                run.placed[kind] += 1
                total = sum(run.placed.values())
            self._placed[arm] += 1
            run.handle.publish_feedback(FillBasket.Feedback(
                arm_id=arm, phase="PLACED", placed_total=total))
        self._set_state(arm, STATE_IDLE)

    def _enter_basket_space(self, run: _Run) -> bool:
        while self._go_on(run):
            if self._basket_space.acquire(timeout=LOCK_POLL_S):
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = FakeWorkcellNode()
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
