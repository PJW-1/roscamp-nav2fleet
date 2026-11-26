"""가짜 로봇 쪽 관리 명령 서버: set_lock · force · charge_end · fault_resolved (IR-21 · IR-61).

pode_fsm.admin.AdminState(강제 입력 포함)에 팔 락과 장애 처리 완료를 더한다.
최근 32건의 request_id를 60초 기억하고 같은 답을 준다. mount()로 Pode든 팔이든 어느 노드에나 붙인다.
"""
from __future__ import annotations

import time
from collections.abc import Callable

from fleet_interfaces.srv import ForceInput, RobotCommand, SetLock
from pode_fsm.admin import UNKNOWN_TARGET, AdminState, Reply

HOLDING_ITEM = "HOLDING_ITEM"


class FakeAdmin(AdminState):
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
          on_fault_resolved: Callable[[], None] | None = None) -> list:
    """admin.robot_id 이름으로 계약의 로봇 쪽 관리 서버를 연다. 만든 서비스를 돌려준다."""
    rid = admin.robot_id

    def instances(key, field):
        entry = contract.entries[key]
        return entry.instances.get(field, contract.placeholders.get(field, []))

    def on_set_lock(request, response):
        with lock:
            reply = admin.set_lock_held(request.request_id, request.robot_id, request.locked,
                                        request.confirm_drop, stopped(),
                                        bool(holding and holding()), time.monotonic())
        return _answer(response, request.request_id, reply)

    def on_force(request, response):
        with lock:
            reply = admin.force(request.request_id, request.robot_id, request.kind, request.key,
                                request.value, time.monotonic())
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
            ForceInput, contract.name("robot_force", robot=rid), on_force, callback_group=group))
    if battery is not None and rid in instances("pode_charge_end", "pode"):
        services.append(node.create_service(
            RobotCommand, contract.name("pode_charge_end", pode=rid), on_charge_end,
            callback_group=group))
    if rid in instances("arm_fault_resolved", "arm"):
        services.append(node.create_service(
            RobotCommand, contract.name("arm_fault_resolved", arm=rid), on_fault,
            callback_group=group))
    return services
