"""가짜 로봇 쪽 관리 명령 서버: set_lock · force · charge_end · fault_resolved · set_pause (IR-21 · IR-61).

pode_fsm.admin.AdminState(강제 입력 포함)에 팔 락과 장애 처리 완료를 더한다.
최근 32건의 request_id를 60초 기억하고 같은 답을 준다. mount()로 Pode든 팔이든 어느 노드에나 붙인다.
"""
from __future__ import annotations

import time
from collections.abc import Callable

from fleet_interfaces.msg import ArmState
from fleet_interfaces.srv import ForceInput, RobotCommand, SetLock, SetPause
from pode_fsm.admin import FORCE_CLEAR, NOT_FOUND, UNKNOWN_TARGET, AdminState, Reply

HOLDING_ITEM = "HOLDING_ITEM"
# 계약 request_rejected 코드: 고장 중인 팔 · 락이 풀린 팔
NOT_READY = "NOT_READY"
LOCK_RELEASED = "LOCK_RELEASED"
# 관제가 멈춘 팔로 보는 상태(fleet_manager ARM_STOPPED와 같다)
ARM_STOPPED = (ArmState.STATE_IDLE, ArmState.STATE_PAUSED, ArmState.STATE_ERROR)


class FakeAdmin(AdminState):
    def reboot(self, boot_id: str) -> None:
        """프로세스 재시작: 락 · 모드 · 강제 값 · 요청 기억이 켜질 때 값으로 돌아간다."""
        AdminState.__init__(self, self.robot_id, boot_id)

    def force_hooked(self, request_id, robot_id, kind, key, value, now,
                     hook: Callable | None = None) -> Reply:
        """강제 입력을 받은 뒤 hook(kind, key, value)이 거절 사유 문자열을 주면 앞 값으로 되돌리고 NOT_FOUND."""
        def judge() -> Reply:
            before = self.forced_values.get(key)
            reply = AdminState.force(self, "", robot_id, kind, key, value, now)
            if reply.accepted and hook is not None and kind != FORCE_CLEAR:
                err = hook(kind, key, value)
                if err:
                    if before is None:
                        self.forced_values.pop(key, None)
                    else:
                        self.forced_values[key] = before
                    return Reply(False, NOT_FOUND, err)
            return reply
        return self._once(request_id, now, judge)

    def set_pause(self, request_id, paused, now, apply: Callable[[bool], Reply]) -> Reply:
        """팔 멈춤 · 이어 하기. 판단은 노드의 apply가 한다."""
        return self._once(request_id, now, lambda: apply(bool(paused)))

    def set_lock_held(self, request_id, robot_id, locked, confirm_drop, stopped, holding,
                      now) -> Reply:
        """팔: 쥔 상품이 있으면 confirm_drop 없이는 풀지 않는다."""
        if not locked and holding and not confirm_drop and self.lock:
            return self._once(request_id, now, lambda: Reply(
                False, HOLDING_ITEM, "holding an item, confirm_drop first"))
        return self.set_lock(request_id, robot_id, locked, stopped, now)

    def fault_resolved(self, request_id, robot_id, now, clear: Callable | None = None) -> Reply:
        def judge() -> Reply:
            if robot_id != self.robot_id:
                return Reply(False, UNKNOWN_TARGET, f"this is {self.robot_id}")
            if clear is not None:
                clear()
            return Reply(True)
        return self._once(request_id, now, judge)


def _answer(response, request_id, reply: Reply):
    response.accepted, response.reason = reply.accepted, reply.reason
    response.request_id, response.detail = request_id, reply.detail
    return response


def mount(node, contract, admin: FakeAdmin, lock, group, stopped: Callable[[], bool],
          battery: Callable[[], float] | None = None, holding: Callable[[], bool] | None = None,
          on_fault_resolved: Callable[[], None] | None = None,
          on_force: Callable[[str, str, str], str] | None = None,
          on_unlock: Callable[[], None] | None = None,
          set_pause: Callable[[bool], Reply] | None = None) -> list:
    """admin.robot_id 이름으로 계약의 로봇 쪽 관리 서버를 연다. 만든 서비스를 돌려준다.

    콜백은 모두 lock 안에서 불린다. on_force는 거절 사유(빈 값이면 받음),
    on_unlock은 set_lock으로 락이 실제로 풀렸을 때 불린다.
    """
    rid = admin.robot_id

    def instances(key, field):
        entry = contract.entries[key]
        return entry.instances.get(field, contract.placeholders.get(field, []))

    def on_set_lock(request, response):
        with lock:
            was = admin.lock
            reply = admin.set_lock_held(request.request_id, request.robot_id, request.locked,
                                        request.confirm_drop, stopped(),
                                        bool(holding and holding()), time.monotonic())
            if on_unlock is not None and was and not admin.lock:
                on_unlock()
        return _answer(response, request.request_id, reply)

    def on_force_request(request, response):
        with lock:
            reply = admin.force_hooked(request.request_id, request.robot_id, request.kind,
                                       request.key, request.value, time.monotonic(), on_force)
        return _answer(response, request.request_id, reply)

    def on_set_pause(request, response):
        with lock:
            reply = admin.set_pause(request.request_id, request.paused, time.monotonic(),
                                    set_pause)
        return _answer(response, request.request_id, reply)

    def on_charge_end(request, response):
        with lock:
            reply = admin.charge_end(request.request_id, request.robot_id, float(battery()),
                                     time.monotonic())
        return _answer(response, request.request_id, reply)

    def on_fault(request, response):
        with lock:
            reply = admin.fault_resolved(request.request_id, request.robot_id, time.monotonic(),
                                         on_fault_resolved)
        return _answer(response, request.request_id, reply)

    services = []
    if rid in instances("robot_set_lock", "robot"):
        services.append(node.create_service(
            SetLock, contract.name("robot_set_lock", robot=rid), on_set_lock,
            callback_group=group))
    if rid in instances("robot_force", "robot"):
        services.append(node.create_service(
            ForceInput, contract.name("robot_force", robot=rid), on_force_request,
            callback_group=group))
    if battery is not None and rid in instances("pode_charge_end", "pode"):
        services.append(node.create_service(
            RobotCommand, contract.name("pode_charge_end", pode=rid), on_charge_end,
            callback_group=group))
    if rid in instances("arm_fault_resolved", "arm"):
        services.append(node.create_service(
            RobotCommand, contract.name("arm_fault_resolved", arm=rid), on_fault,
            callback_group=group))
    if set_pause is not None and rid in instances("arm_set_pause", "arm"):
        services.append(node.create_service(
            SetPause, contract.name("arm_set_pause", arm=rid), on_set_pause,
            callback_group=group))
    return services
