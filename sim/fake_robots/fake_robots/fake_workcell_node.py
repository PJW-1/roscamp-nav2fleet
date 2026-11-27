"""FakeWorkcell: arm_left, arm_right를 뒤에 둔 fill_basket 서버.

요청한 물건은 그 종류의 재고가 있는 팔에 간다. 두 팔이 동시에 집지만
잠금 하나로 바구니 공간(PLACING)에는 한 팔만 들어간다.

시험 훅은 팔의 /{arm}/force(시험 모드, kind CONDITION)로 건다. 키는 HOOK_* 상수.
"""
from __future__ import annotations

import json
import math
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
from pode_fsm.admin import MODE_MISMATCH, Reply
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fake_robots import fake_admin
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
_R = FillBasket.Result
_F = FillBasket.Feedback
REASON_ARM_FAULT = _R.REASON_ARM_FAULT
REASON_CANCELED = _R.REASON_CANCELED
# fill_fail 훅으로 낼 수 있는 결과 사유
FAIL_REASONS = (_R.REASON_PICK_FAILED, _R.REASON_RECOGNITION_ERROR, _R.REASON_OUT_OF_STOCK,
                _R.REASON_ARM_FAULT)
LOCK_POLL_S = 0.01
STATE_IDLE = ArmState.STATE_IDLE
STATE_PICKING = ArmState.STATE_PICKING
STATE_PLACING = ArmState.STATE_PLACING
STATE_PAUSED = ArmState.STATE_PAUSED
STATE_ERROR = ArmState.STATE_ERROR
STATE_PERIOD_S = 0.2  # IR-34: 초당 5번
RESULT_FIRST_S = 0.1  # 실제 초. 고장 결과가 관제에 먼저 닿은 뒤 ERROR를 보인다

# ---- 시험 훅 키 (/{arm}/force value) ----
HOOK_FILL_FAIL = "fill_fail"        # "<사유>[@k]": 이 팔이 k개 담은 뒤 그 사유로 목표 끝
HOOK_NOT_READY = "not_ready"        # "[N]": 다음 목표 N개(기본 1)를 NOT_READY로 바로 거절
HOOK_UNKNOWN = "unknown_items"      # "Z,Y": 이 상품 코드가 든 목표는 UNKNOWN_TARGET
HOOK_SELF_STOP = "self_stop"        # "[k]": 이 팔이 k개 담은 뒤 스스로 멈춤(PAUSED)
HOOK_RECOVER = "recover"            # [복구]: 처음 자세로, PAUSED 유지
HOOK_RESTART = "restart"            # [초기화]: "gap=1.0,holding=1,late_result=2.0"
HOOK_APP_UNLOCK = "app_unlock"      # Phody 앱 락 해제 버튼
HOOK_ERROR = "error"                # 복구할 수 없는 고장: fault_resolved까지 ERROR
HOOK_STATE_LOSS = "state_loss"      # "[초]": 이 팔 상태 발행을 그만큼 멈춤

STOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def order_items(counts: Counter) -> list[OrderItem]:
    return [OrderItem(kind=kind, count=count) for kind, count in sorted(counts.items()) if count]


def _count(value: str, default: int = 0) -> int:
    n = int(value) if value.strip() else default
    if n < 0:
        raise ValueError("negative")
    return n


def parse_fail(value: str) -> tuple[str, int]:
    """'사유[@k]' → (사유, k)."""
    reason, _, at = value.partition("@")
    if reason not in FAIL_REASONS:
        raise ValueError(f"reason must be one of {FAIL_REASONS}")
    return reason, _count(at)


def parse_restart(value: str, gap: float) -> tuple[float, bool, float | None]:
    """'gap=1.0,holding=1,late_result=2.0' → (gap, holding, late_result)."""
    opts = dict(part.split("=", 1) for part in value.split(",") if part.strip())
    unknown = set(opts) - {"gap", "holding", "late_result"}
    if unknown:
        raise ValueError(f"unknown option {sorted(unknown)}")
    gap = float(opts.get("gap", gap))
    late = float(opts["late_result"]) if "late_result" in opts else None
    if gap < 0 or (late is not None and late < 0):
        raise ValueError("negative")
    return gap, opts.get("holding", "0") in ("1", "true"), late


class _Run:
    """팔 스레드가 처리하는 동안의 채움 목표 하나."""

    def __init__(self, handle):
        self.handle = handle
        self.placed: Counter = Counter()
        self.uncertain: Counter = Counter()  # 쥔 채 끝났거나 떨어뜨린 상품
        self.let_go = 0  # 락 해제로 떨어뜨린 수. 다시 집지 않고 다음 상품으로 간다
        self.errored: dict[str, float | None] = {}  # 고장 난 팔 → ERROR 유지 초(None = 해결까지)
        self.reason = ""
        self.working: set[str] = set()
        self.fault = threading.Event()
        self.dropped = threading.Event()  # 재시작으로 버린 목표
        self.workers_done = threading.Event()
        self.restart_at = 0.0
        self.late_result: float | None = None
        self.lock = threading.Lock()

    def fail(self, reason: str, arm: str, hold: float | None, show: bool = True) -> None:
        # show=False: 이미 ERROR인 팔. 결과 뒤 ERROR를 다시 보이지 않는다
        with self.lock:
            if not self.fault.is_set():
                self.reason = reason
            if reason == REASON_ARM_FAULT and show:
                self.errored[arm] = hold
        self.fault.set()


class FakeWorkcellNode(Node):
    def __init__(self, parameter_overrides=None, **kwargs):
        super().__init__("fake_workcell", parameter_overrides=parameter_overrides, **kwargs)
        # true면 실물 Phody처럼 두 팔 모드가 UNKNOWN이거나 서로 다를 때 담기를 MODE_MISMATCH로 끝낸다.
        # 가짜 실행에는 /system/mode를 내는 브리지가 없어 기본은 false
        self.declare_parameter("mode_gate", False)
        self._mode_gate = bool(self.get_parameter("mode_gate").value)
        # 시험 훅 기본값: 모르는 상품 코드(쉼표), 재시작 · 상태 끊김 · 고장 해결 시간
        self.declare_parameter("unknown_items", "")
        self.declare_parameter("restart_gap_s", 1.0)
        self.declare_parameter("state_loss_s", 3.0)
        self.declare_parameter("fault_resolved_delay_s", 0.0)
        self._unknown = {k.strip() for k in str(self.get_parameter("unknown_items").value)
                         .split(",") if k.strip()}
        self._restart_gap = float(self.get_parameter("restart_gap_s").value)
        self._state_loss = float(self.get_parameter("state_loss_s").value)
        self._resolve_delay = float(self.get_parameter("fault_resolved_delay_s").value)
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
        # 받은 목표의 팔 작업이 끝나기 전이면 참 (_admin_lock 안). 느린 취소 대기는 포함하지 않는다
        self._moving = False
        self._alive = True
        # 실제 서버도 지킬 규칙: 이 잠금을 쥔 쪽만 PLACING이 될 수 있다.
        self._basket_space = threading.Lock()
        self._state_lock = threading.RLock()  # ERROR 세대 갱신과 상태 설정을 한 구간에서
        self._state = dict.fromkeys(self._arms, STATE_IDLE)
        self._error_until = dict.fromkeys(self._arms, 0.0)
        self._fault_gen = dict.fromkeys(self._arms, 0)  # ERROR를 보일 때마다 하나씩
        self._resolved_n = dict.fromkeys(self._arms, 0)  # fault_resolved로 ERROR를 푼 횟수
        self._paused = dict.fromkeys(self._arms, False)
        self._holding = dict.fromkeys(self._arms, False)
        self._dropped = dict.fromkeys(self._arms, False)  # 락 해제로 쥔 상품을 놓음
        self._silent_until = dict.fromkeys(self._arms, 0.0)
        self._rebooting = False
        self._run: _Run | None = None
        self._dropped_goals: set[bytes] = set()
        self._cancel_at: float | None = None
        self._boot_id = str(uuid.uuid4())
        self._order_id = ""
        self._placed_total = 0
        # 팔마다 pode_fsm과 같은 규칙의 락 · 운영 모드 · 요청 번호 기억(IR-72)
        self._admin = {arm: fake_admin.FakeAdmin(arm, self._boot_id) for arm in self._arms}
        self._admin_lock = threading.Lock()
        group = ReentrantCallbackGroup()
        for arm in self._arms:
            self.create_service(SetMode, contract.name("robot_set_mode", robot=arm),
                                lambda req, res, a=arm: self._on_set_mode(a, req, res))
            # 관리 명령: set_lock · force · fault_resolved · set_pause (상태 칸 lock · forced에 실린다)
            fake_admin.mount(
                self, contract, self._admin[arm], self._admin_lock, group,
                lambda a=arm: self._stopped(a),
                holding=lambda a=arm: self._held(a),
                on_fault_resolved=lambda a=arm: self._clear_error(a),
                on_force=lambda kind, key, value, a=arm: self._on_force(a, key, value),
                on_unlock=lambda a=arm: self._let_go_now(a),
                set_pause=lambda paused, a=arm: self._apply_pause(a, paused))
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

    def _shown(self, arm: str) -> str:
        state = self._state[arm]
        if state != STATE_ERROR and self._paused[arm]:
            return STATE_PAUSED
        return state

    def _held(self, arm: str) -> bool:
        return self._holding[arm] or self._state[arm] == STATE_PLACING

    def _tick(self) -> None:
        t = self._sc.now()
        for arm in self._arms:
            if self._state[arm] == STATE_ERROR and t >= self._error_until[arm]:
                self._set_state(arm, STATE_IDLE)

    def _stopped(self, arm: str) -> bool:
        # 목표를 받은 뒤 팔 작업이 끝나기 전에는 IDLE · ERROR여도 곧 움직인다 (_admin_lock 안).
        # 멈춘(PAUSED) 팔은 목표 중에도 멈춘 것이다
        return self._paused[arm] or (self._shown(arm) in fake_admin.ARM_STOPPED
                                     and not self._moving)

    def _moved(self) -> None:
        with self._admin_lock:
            self._moving = False

    def _clear_error(self, arm: str) -> None:
        with self._state_lock:
            self._resolved_n[arm] += 1  # 결과 뒤 아직 보이지 않은 고장은 보이지 않는다
            asked = self._fault_gen[arm]

        def clear():
            with self._state_lock:
                if self._fault_gen[arm] != asked:  # 요청 뒤 새로 난 고장
                    return
                self._error_until[arm] = 0.0
                if self._state[arm] == STATE_ERROR:
                    self._state[arm] = STATE_IDLE
        if self._resolve_delay <= 0:
            clear()
            return
        timer = threading.Timer(self._resolve_delay, clear)
        timer.daemon = True
        timer.start()

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

    def _show_errors(self, arms, hold: float | None = ERROR_HOLD_S,
                     boot_id: str | None = None, resolved: dict | None = None) -> None:
        # hold None이면 fault_resolved까지 ERROR. 그 사이 재시작했으면 보이지 않는다
        if boot_id is not None and boot_id != self._boot_id:
            return
        until = math.inf if hold is None else self._sc.now() + hold
        for arm in arms:
            with self._state_lock:
                if resolved is not None and self._resolved_n[arm] != resolved[arm]:
                    continue  # 고장 뒤 보이기 전에 fault_resolved가 왔다
                self._fault_gen[arm] += 1
                self._error_until[arm] = until
                self._set_state(arm, STATE_ERROR)

    def _publish_states(self) -> None:
        # 모드는 set_mode 전까지 UNKNOWN(IR-72). 재시작 · 상태 끊김 동안은 내지 않는다
        if self._rebooting:
            return
        stamp = self.get_clock().now().to_msg()
        now = self._sc.now()
        for arm in self._arms:
            if now < self._silent_until[arm]:
                continue
            state = self._shown(arm)
            holding = self._held(arm)
            with self._admin_lock:
                a = self._admin[arm]
                lock, mode, last_id, forced = a.lock, a.mode, a.last_request_id, a.forced
            self._state_pubs[arm].publish(ArmState(
                arm_id=arm, boot_id=self._boot_id, state=state,
                detail=REASON_ARM_FAULT if state == STATE_ERROR else "", lock=lock,
                gripper=ArmState.GRIPPER_CLOSED if holding else ArmState.GRIPPER_OPEN,
                holding=holding, order_id=self._order_id, placed_total=self._placed_total,
                mode=mode, last_request_id=last_id, forced=forced, stamp=stamp))

    def _publish_stock(self, arm: str) -> None:
        with self._stock_lock:
            data = json.dumps({k: c for k, c in self._stock[arm].items() if c > 0})
        self._stock_pubs[arm].publish(String(data=data))

    def _change_stock(self, arm: str, kind: str, delta: int) -> None:
        with self._stock_lock:
            self._stock[arm][kind] += delta
        self._publish_stock(arm)

    # ---- 시험 훅 (_admin_lock 안에서 불린다) ----

    def _on_force(self, arm: str, key: str, value: str) -> str:
        """훅 키면 값을 확인하고 바로 할 일은 한다. 거절 사유를 돌려준다(받으면 빈 값)."""
        forced = self._admin[arm].forced_values
        try:
            if key == HOOK_FILL_FAIL:
                parse_fail(value)
            elif key in (HOOK_SELF_STOP, HOOK_NOT_READY):
                _count(value)
            elif key == HOOK_UNKNOWN:
                if not value.strip():
                    return "give item codes"
            elif key == HOOK_RECOVER:
                if not self._paused[arm]:
                    return f"{arm} is not PAUSED"
                forced.pop(key, None)
                self.get_logger().info(f"{arm}: back to the start pose, still PAUSED")
            elif key == HOOK_RESTART:
                gap, holding, late = parse_restart(value, self._restart_gap)
                forced.pop(key, None)
                threading.Thread(target=self._restart, args=(gap, holding, late),
                                 daemon=True).start()
            elif key == HOOK_APP_UNLOCK:
                forced.pop(key, None)
                self._app_unlock(arm)
            elif key == HOOK_ERROR:
                forced.pop(key, None)
                run = self._run
                if run is not None and arm in run.working:
                    run.fail(REASON_ARM_FAULT, arm, None)
                else:
                    self._show_errors([arm], None)
            elif key == HOOK_STATE_LOSS:
                secs = float(value) if value.strip() else self._state_loss
                if secs < 0:
                    return "negative seconds"
                forced.pop(key, None)
                self._silent_until[arm] = self._sc.now() + secs
        except ValueError as e:
            return f"bad value for {key}: {e}"
        return ""

    def _app_unlock(self, arm: str) -> None:
        # see_center로 가고 쥐었으면 그리퍼를 연다. 모터 힘 유지. 담는 중이면 PAUSED
        self._admin[arm].lock = False
        run = self._run
        if run is not None and arm in run.working:
            self._paused[arm] = True
        self._let_go_now(arm)

    def _let_go_now(self, arm: str) -> None:
        with self._state_lock:
            if self._holding[arm] or self._state[arm] == STATE_PLACING:
                self._holding[arm] = False
                self._dropped[arm] = True

    def _take_drop(self, arm: str) -> bool:
        with self._state_lock:
            dropped, self._dropped[arm] = self._dropped[arm], False
        return dropped

    def _apply_pause(self, arm: str, paused: bool) -> Reply:
        if self._state[arm] == STATE_ERROR:
            return Reply(False, fake_admin.NOT_READY, f"{arm} is in ERROR")
        if not paused and not self._admin[arm].lock:
            return Reply(False, fake_admin.LOCK_RELEASED, f"{arm} lock released, lock it first")
        self._paused[arm] = paused
        return Reply(True)

    def _take_not_ready(self) -> bool:
        with self._admin_lock:
            for arm in self._arms:
                forced = self._admin[arm].forced_values
                if HOOK_NOT_READY in forced:
                    left = _count(forced[HOOK_NOT_READY], 1) - 1
                    if left > 0:
                        forced[HOOK_NOT_READY] = str(left)
                    else:
                        forced.pop(HOOK_NOT_READY)
                    return True
        return False

    def _unknown_kinds(self) -> set[str]:
        kinds = set(self._unknown)
        with self._admin_lock:
            for arm in self._arms:
                value = self._admin[arm].forced_values.get(HOOK_UNKNOWN, "")
                kinds |= {k.strip() for k in value.split(",") if k.strip()}
        return kinds

    def _take_item_hooks(self, arm: str, done: int) -> tuple[str, int] | None:
        """이 팔이 done개 담은 지금 걸린 self_stop · fill_fail을 쓴다. fill_fail이면 그 값."""
        with self._admin_lock:
            forced = self._admin[arm].forced_values
            if HOOK_SELF_STOP in forced and done >= _count(forced[HOOK_SELF_STOP]):
                forced.pop(HOOK_SELF_STOP)
                self._paused[arm] = True
                self.get_logger().info(f"{arm}: stopped by itself after {done} items")
            if HOOK_FILL_FAIL in forced:
                fail = parse_fail(forced[HOOK_FILL_FAIL])
                if done >= fail[1]:
                    forced.pop(HOOK_FILL_FAIL)
                    return fail
        return None

    # ---- 재시작 ----

    def _restart(self, gap: float, holding: bool, late: float | None) -> None:
        run = self._run
        if holding and run is not None:
            for arm in list(run.working):
                self._holding[arm] = True
            self._publish_states()
            time.sleep(STATE_PERIOD_S)
        self._rebooting = True
        start = self._sc.now()
        if run is not None:
            run.late_result = late
            run.restart_at = start
            self._dropped_goals.add(bytes(run.handle.goal_id.uuid))
            run.dropped.set()
            run.workers_done.wait(2.0)
        self._sc.sleep_until(start + gap, self._running)
        self._boot_id = str(uuid.uuid4())
        with self._admin_lock:
            for a in self._admin.values():
                a.reboot(self._boot_id)
            self._moving = False
        with self._state_lock:
            for arm in self._arms:
                self._state[arm] = STATE_IDLE
                self._error_until[arm] = 0.0
                self._paused[arm] = self._holding[arm] = self._dropped[arm] = False
                self._silent_until[arm] = 0.0
        self._order_id = ""
        self._placed_total = 0
        self._run = None
        self._cancel_at = None
        with self._busy_lock:
            self._busy = False
        self._rebooting = False
        self.get_logger().info(f"restarted, boot_id {self._boot_id}")

    def _dropped_result(self, handle, run: _Run):
        # 재시작으로 버린 목표는 결과를 내지 않는다. late_result면 그 시각에 옛 목표를 끝낸다
        if run.late_result is not None:
            self._sc.sleep_until(run.restart_at + run.late_result, self._running)
        else:
            while self._running():
                time.sleep(0.05)
        requested = Counter()
        for item in handle.request.items:
            requested[item.kind] += max(item.count, 0)
        result = FillBasket.Result(success=True, placed=order_items(requested), missing=[],
                                   reason="", placed_confirmed=order_items(requested),
                                   uncertain=[])
        if self._running():
            handle.succeed()
        else:
            abort_at_shutdown(handle)
        return result

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
        # IR-61: 락이 풀린 팔이 하나라도 있으면 받지 않는다. 락 해제와 같은 잠금 안에서 받는다
        with self._admin_lock:
            released = [arm for arm in self._arms if not self._admin[arm].lock]
            if not released:
                with self._busy_lock:
                    if self._busy:
                        return GoalResponse.REJECT
                    self._busy = True
                self._moving = True
        if released:
            self.get_logger().warn(f"fill_basket goal rejected: lock released ({released})")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, handle):
        # 재시작으로 버린 목표는 새 프로세스가 모른다
        if bytes(handle.goal_id.uuid) in self._dropped_goals:
            return CancelResponse.REJECT
        if self._cancel_at is None:
            self._cancel_at = self._sc.now()
        return CancelResponse.ACCEPT

    def _refusal(self, request) -> str:
        if self._mode_gate and not self._modes_match():
            return MODE_MISMATCH
        if self._take_not_ready():
            return _R.REASON_NOT_READY
        if {item.kind for item in request.items} & self._unknown_kinds():
            return _R.REASON_UNKNOWN_TARGET
        return ""

    def _refuse(self, handle, reason: str):
        # 하나도 집지 않고 바로 끝낸다
        requested = Counter()
        for item in handle.request.items:
            requested[item.kind] += max(item.count, 0)
        self._moved()
        with self._busy_lock:
            self._busy = False
        if self._running():
            handle.abort()
        else:
            abort_at_shutdown(handle)
        return FillBasket.Result(success=False, placed=[], missing=order_items(requested),
                                 reason=reason, placed_confirmed=[], uncertain=[])

    def _execute(self, handle):
        refusal = self._refusal(handle.request)
        if refusal:
            return self._refuse(handle, refusal)
        self._cancel_at = None
        self._order_id = handle.request.order_id
        self._placed_total = 0
        units, missing = self._plan(handle.request.items)
        fail_at = self._plan_failures(units)
        run = _Run(handle)
        self._run = run
        workers = [threading.Thread(target=self._work,
                                    args=(run, arm, units[arm], fail_at.get(arm)), daemon=True)
                   for arm in self._arms if units[arm]]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        run.workers_done.set()
        if run.dropped.is_set():
            return self._dropped_result(handle, run)
        self._moved()
        if handle.is_cancel_requested and self._cancel_at is not None:
            # 실제 팔은 멈추는 데 시간이 걸리고 그동안 서버는 바쁘다.
            self._sc.sleep_until(self._cancel_at + self._cfg.cancel_seconds, self._running)
            if run.dropped.is_set():  # 취소를 기다리는 동안 재시작
                return self._dropped_result(handle, run)

        planned = sum(len(u) for u in units.values())
        complete = sum(run.placed.values()) + run.let_go == planned
        success = complete and not run.fault.is_set()
        reason = ""
        if run.fault.is_set():
            reason = run.reason
        elif not complete:
            reason = REASON_CANCELED
        # `missing`은 서버가 시도를 마쳤을 때만 확정이다. 재시도하면 다시 계산한다.
        result = FillBasket.Result(
            success=success, placed=order_items(run.placed),
            missing=order_items(missing) if success else [], reason=reason,
            placed_confirmed=order_items(run.placed), uncertain=order_items(run.uncertain))
        self._order_id = ""
        self._run = None
        with self._state_lock:
            for arm in self._arms:
                self._paused[arm] = self._dropped[arm] = False
        with self._busy_lock:
            self._busy = False
        if self._running():
            finish_action(handle, success)
        else:
            abort_at_shutdown(handle)
        for hold in set(run.errored.values()):
            # 재시도할 고장으로 받도록 결과가 나간 뒤 ERROR를 보인다
            arms = [arm for arm, h in run.errored.items() if h == hold]
            timer = threading.Timer(RESULT_FIRST_S, self._show_errors,
                                    (arms, hold, self._boot_id, dict(self._resolved_n)))
            timer.daemon = True
            timer.start()
        return result

    def _alive_run(self, run: _Run) -> bool:
        return self._running() and not run.dropped.is_set()

    def _go_on(self, run: _Run) -> bool:
        return (self._alive_run(run) and not run.fault.is_set()
                and not run.handle.is_cancel_requested)

    def _feedback(self, run: _Run, arm: str, phase: str, total: int | None = None) -> None:
        # 두 팔의 placed_total이 거꾸로 가지 않게 run.lock 안에서 낸다(total을 주면 이미 안)
        if total is not None:
            self._publish_feedback(run, arm, phase, total)
            return
        with run.lock:
            self._publish_feedback(run, arm, phase, sum(run.placed.values()))

    def _publish_feedback(self, run: _Run, arm: str, phase: str, total: int) -> None:
        if not run.dropped.is_set():
            run.handle.publish_feedback(_F(arm_id=arm, phase=phase, placed_total=total))

    def _unpaused(self, run: _Run, arm: str) -> bool:
        """멈춘 동안 기다린다. 목표가 끝나면 False."""
        while self._paused[arm]:
            if not self._go_on(run):
                return False
            time.sleep(LOCK_POLL_S)
        return True

    def _wait(self, run: _Run, arm: str, seconds: float, cancel_ok: bool = True) -> bool:
        """seconds 동안 움직인다. 멈춘 동안은 시간이 가지 않는다. 다 움직이면 True."""
        def move_on():
            return self._go_on(run) if cancel_ok else self._alive_run(run)
        left = seconds
        while True:
            if self._dropped[arm] or not self._unpaused(run, arm):
                return False
            if left <= 0:
                return True
            start = self._sc.now()
            self._sc.sleep_until(start + left, lambda: move_on() and not self._paused[arm])
            if not move_on():
                return False
            left -= self._sc.now() - start

    def _let_go(self, run: _Run, arm: str, kind: str, put_back: bool) -> bool:
        """쥔 상품을 못 놓고 멈춘 뒤. 떨어뜨렸고 목표가 살아 있으면 다음 상품으로(True)."""
        self._holding[arm] = False
        if self._take_drop(arm):
            with run.lock:
                run.uncertain[kind] += 1
                run.let_go += 1
            return self._go_on(run)
        if put_back:
            self._change_stock(arm, kind, +1)  # 바구니에 못 닿았으니 선반으로 되돌린다
        else:
            with run.lock:
                run.uncertain[kind] += 1
        return False

    def _work(self, run: _Run, arm: str, kinds: list[str], fail_at: int | None) -> None:
        with run.lock:
            run.working.add(arm)
        try:
            self._work_items(run, arm, kinds, fail_at)
        finally:
            with run.lock:
                run.working.discard(arm)
            if not run.dropped.is_set():
                self._holding[arm] = False
                if self._state[arm] != STATE_ERROR:
                    self._set_state(arm, STATE_IDLE)

    def _work_items(self, run: _Run, arm: str, kinds: list[str], fail_at: int | None) -> None:
        # 상품마다 CAPTURING → PICKING → TO_BASKET → PLACING → RETURNING
        q = self._cfg.seconds_per_item / 4
        if kinds and self._error_until[arm] == math.inf:
            run.fail(REASON_ARM_FAULT, arm, None, show=False)  # fault_resolved 전
            return
        done = 0
        for i, kind in enumerate(kinds):
            if not self._go_on(run):
                return
            fail = self._take_item_hooks(arm, done)
            if fail is not None:
                run.fail(fail[0], arm, None)
                return
            if fail_at == i:
                # error_until_resolved면 실물처럼 fault_resolved까지 ERROR
                run.fail(REASON_ARM_FAULT, arm,
                         None if self._cfg.error_until_resolved else ERROR_HOLD_S)
                return
            if not self._unpaused(run, arm) or not self._go_on(run):
                return
            self._set_state(arm, STATE_PICKING)
            self._feedback(run, arm, _F.PHASE_CAPTURING)
            if not self._wait(run, arm, q):
                return
            self._feedback(run, arm, _F.PHASE_PICKING)
            if not self._wait(run, arm, q):
                return
            self._change_stock(arm, kind, -1)
            with self._state_lock:
                # 집는 순간 락이 이미 풀렸으면 바로 놓는다
                if self._admin[arm].lock:
                    self._holding[arm] = True
                else:
                    self._dropped[arm] = True
            self._feedback(run, arm, _F.PHASE_TO_BASKET)
            if not self._enter_basket_space(run, arm):
                if self._let_go(run, arm, kind, put_back=True):
                    continue
                return
            try:
                self._set_state(arm, STATE_PLACING)
                self._feedback(run, arm, _F.PHASE_PLACING)
                placed = self._wait(run, arm, q, cancel_ok=False)
                if placed:
                    self._holding[arm] = False
                self._set_state(arm, STATE_IDLE)
            finally:
                self._basket_space.release()
            if not placed:
                if self._let_go(run, arm, kind, put_back=False):
                    continue
                return
            with run.lock:
                run.placed[kind] += 1
                total = sum(run.placed.values())
                self._placed_total = total
                self._feedback(run, arm, _F.PHASE_RETURNING, total)
            done += 1
            if not self._wait(run, arm, q):
                return

    def _enter_basket_space(self, run: _Run, arm: str) -> bool:
        while self._go_on(run) and not self._dropped[arm]:
            if self._paused[arm]:
                time.sleep(LOCK_POLL_S)
            elif self._basket_space.acquire(timeout=LOCK_POLL_S):
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = FakeWorkcellNode()
    # 재시작 훅으로 버린 목표가 결과 없이 스레드 하나를 쥐고 있을 수 있다
    executor = MultiThreadedExecutor(num_threads=8)
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
