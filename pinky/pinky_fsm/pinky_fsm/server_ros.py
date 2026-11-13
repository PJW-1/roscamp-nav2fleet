from __future__ import annotations

import threading
import time
from collections.abc import Callable
from functools import partial

import rclpy
from fleet_interfaces.action import Dock, MoveTo, Retreat
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from pinky_fsm.order_ros import order_goal

FEEDBACK_PERIOD_S = 0.2
_RESULT = {"move": MoveTo.Result, "dock": Dock.Result, "retreat": Retreat.Result}


class _Slot:
    def __init__(self, handle):
        self.handle = handle
        self.done = threading.Event()
        self.success = False
        self.reason = ""
        self.last_done = 0


class GoalServers:
    def __init__(self, node, contract, robot_id: str, group, lock, fsm,
                 pose: Callable, run: Callable, alive: Callable[[], bool]):
        self._node, self._lock, self._fsm = node, lock, fsm
        self._pose, self._run, self._alive = pose, run, alive
        self._slots: dict[bytes, _Slot] = {}
        self._move_server = self._server(MoveTo, contract.name("move_to", pinky=robot_id),
                                         "move", group)
        self._dock_server = self._server(Dock, contract.name("dock", pinky=robot_id),
                                         "dock", group)
        self._retreat_server = self._server(
            Retreat, contract.name("retreat", pinky=robot_id), "retreat", group)

    def destroy(self) -> None:
        self._move_server.destroy()
        self._dock_server.destroy()
        self._retreat_server.destroy()

    def finish(self, goal_id, success: bool, reason: str, last_done: int = 0) -> None:
        slot = self._slots.get(goal_id)
        if slot is not None:
            slot.success, slot.reason = success, reason
            slot.last_done = last_done
            slot.done.set()

    def _server(self, action_type, name, kind, group):
        return ActionServer(
            self._node, action_type, name,
            execute_callback=partial(self._execute, kind=kind),
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            handle_accepted_callback=partial(self._accepted, kind=kind),
            callback_group=group)

    def _accepted(self, handle, kind: str) -> None:
        key = bytes(handle.goal_id.uuid)
        with self._lock:
            self._slots[key] = _Slot(handle)
            pose = self._pose()
            here = pose[:3] if pose else None
            now = time.monotonic()
            req = handle.request
            if kind == "move":
                t = req.target_pose
                path = [(p.position.x, p.position.y) for p in req.path]
                cmds = self._fsm.move(key, now, req.target_id, (t.x, t.y, t.theta), path, here,
                                      final_goal=req.final_goal, mode=req.mode,
                                      order=order_goal(req), priority=req.priority)
            elif kind == "retreat":
                path = [(p.position.x, p.position.y) for p in req.path]
                cmds = self._fsm.retreat(key, now, req.distance_m, req.speed_mps, path, here)
            else:
                cmds = self._fsm.dock(key, now, req.dock_id)
            self._run(cmds)
        handle.execute()

    def _execute(self, handle, kind: str):
        key = bytes(handle.goal_id.uuid)
        slot = self._slots[key]
        last_fb = 0.0
        while not slot.done.is_set():
            if not (self._alive() and rclpy.ok()):
                _abort_quietly(handle)
                return _RESULT[kind]()
            if handle.is_cancel_requested:
                with self._lock:
                    self._run(self._fsm.cancel(key, time.monotonic()))
            now = time.monotonic()
            if kind == "move" and now - last_fb >= FEEDBACK_PERIOD_S:
                last_fb = now
                with self._lock:
                    if self._fsm.is_running(key):
                        pose = self._pose()
                        left = self._fsm.distance_remaining(pose[:3] if pose else None)
                        done, waiting, warning = self._fsm.order_feedback()
                        handle.publish_feedback(MoveTo.Feedback(
                            distance_remaining=float(left), last_done_seq_id=done,
                            waiting_at_base_end=waiting, horizon_warning=warning))
            slot.done.wait(0.02)
        with self._lock:
            self._slots.pop(key, None)
        if slot.success:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        if kind == "move":
            return MoveTo.Result(success=slot.success, reason=slot.reason,
                                 last_done_seq_id=slot.last_done)
        if kind == "retreat":
            return Retreat.Result(success=slot.success, reason=slot.reason)
        if not slot.success:
            self._node.get_logger().warn(f"dock failed: {slot.reason}")
        return Dock.Result(success=slot.success)


def _abort_quietly(handle) -> None:
    try:
        handle.abort()
    except Exception:  # noqa: BLE001 - 종료 중에는 상태 발행이 실패할 수 있다
        pass
