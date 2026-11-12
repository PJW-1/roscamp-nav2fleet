"""가짜 핑키의 목표 받기와 끝내기: 수락 정책, 시작, 종료, 결과."""
from __future__ import annotations

import math
import time

from fleet_interfaces.action import Dock, MoveTo, Retreat
from fleet_interfaces.msg import RobotState
from rclpy.action import CancelResponse, GoalResponse

from fake_robots.motion import abort_at_shutdown, drifted, finish_action, noisy_pose, path_gap
from fake_robots.nav_goal_drive import NavGoalDrive
from fake_robots.pinky_goal import NAV_ABORT_DELAY_S, _Goal

FEEDBACK_PERIOD_S = 0.2
RETREAT_REASONS = (Retreat.Result.REASON_OBSTACLE_BEHIND, Retreat.Result.REASON_TIMEOUT,
                   Retreat.Result.REASON_CANCELED, Retreat.Result.REASON_FAILED)


class PinkyGoals:
    def _goal_policy(self):
        return GoalResponse.REJECT if self._unreachable() else GoalResponse.ACCEPT

    def _cancel_policy(self):
        return CancelResponse.REJECT if self._unreachable() else CancelResponse.ACCEPT

    def _end(self, goal: _Goal, success: bool, reason: str = "") -> None:
        if (goal.abort_at is not None and not goal.done.is_set()
                and reason != MoveTo.Result.REASON_NAV_ABORTED):
            self._faults.give_back_nav_abort(goal.abort_from)  # 대체·취소되면 다음 목표로
        if goal.kind == "retreat" and not success and reason not in RETREAT_REASONS:
            reason = Retreat.Result.REASON_FAILED  # 계약 0.9: Retreat reason은 이 넷뿐이다
        goal.finish(success, reason)
        if self._active is goal:
            self._rest = (RobotState.STATE_DOCKED if goal.kind == "dock" and success
                          else RobotState.STATE_IDLE)
            self._motion.clear_target()
            self._recovery.restart()  # 후진과 RECOVERING을 멈춘다(명세 7번)

    def _accepted(self, handle, kind: str) -> None:
        goal = _Goal(kind, handle)
        with self._lock:
            t = self._sc.now()
            if self._active is not None:
                self._end(self._active, False, MoveTo.Result.REASON_CANCELED)
            self._active = goal
            self._last_beat = time.monotonic()  # 목표가 왔으면 누군가 있다는 뜻
            goal.start_at = t
            if kind == "move":
                pose = handle.request.target_pose
                goal.target_id = handle.request.target_id
                path = [(p.position.x, p.position.y) for p in handle.request.path]
                if handle.request.mode == MoveTo.Goal.MODE_NAV_GOAL:
                    m = self._motion
                    m.clear_target()
                    target = noisy_pose(pose.x, pose.y, self._cfg.stop_error_m, self._rng)
                    goal.nav = NavGoalDrive(self._space, self._recovery.p.robot_radius_m,
                                            target, t, (m.x, m.y))
                    self._take_nav_abort(t, goal, math.dist((m.x, m.y), target))
                elif path and self._path_misses_me(t, path):
                    # 지시서 7.5: 따라가지 않고 실패한다. 이전 목표는 위에서 끝났으니 선다(R6)
                    self._end(goal, False, MoveTo.Result.REASON_PATH_NOT_CONNECTED)
                else:
                    # path: 첫 점은 로봇 자리라 건너뛰고, 끝 점은 target_pose다
                    self._motion.set_target(*noisy_pose(pose.x, pose.y, self._cfg.stop_error_m,
                                                        self._rng), via=path[1:-1],
                                            start=path[0] if path else None)
                    self._take_nav_abort(t, goal, self._motion.remaining())
            elif kind == "retreat":
                self._start_retreat(t, goal, handle.request)
            else:
                goal.target_id = handle.request.dock_id
                self._motion.clear_target()
                goal.end_at = t + self._dock_cfg.duration_s
            self._goals[bytes(handle.goal_id.uuid)] = goal
        handle.execute()

    def _take_nav_abort(self, t: float, goal: _Goal, remaining: float) -> None:
        if self._faults.take_nav_abort(t, goal.target_id):
            eta = remaining / self._cfg.speed_mps
            goal.abort_at = t + min(NAV_ABORT_DELAY_S, eta / 2)  # 도착 전에 중단
            goal.abort_from = self._faults.last_taken

    def _dock_phase(self, goal: _Goal) -> str:
        span = max(goal.end_at - goal.start_at, 1e-6)
        done = (self._sc.now() - goal.start_at) / span
        return "APPROACH" if done < 0.5 else "ALIGN" if done < 1.0 else "DONE"

    def _result(self, goal: _Goal, action_type):
        if action_type is MoveTo:
            return MoveTo.Result(success=goal.success, reason=goal.reason)
        if action_type is Retreat:
            return Retreat.Result(success=goal.success, reason=goal.reason)
        ex, ey, eyaw = goal.error
        return Dock.Result(success=goal.success, error_x_cm=ex, error_y_cm=ey,
                           error_yaw_deg=eyaw)

    def _unreachable(self) -> bool:
        t = self._sc.now()
        return self._faults.comms_down(t) or self._faults.stuck(t) or self._faults.removed(t)

    def _path_misses_me(self, t: float, path) -> bool:
        """받은 path가 로봇이 아는 제 자리에서 연결 허용 거리보다 먼가(지시서 7.5).

        로봇이 보고하는 자리로 잰다. drift는 들어가고 잡음은 빠진다. 실물 핑키도 제가 아는
        자리, 곧 보고하는 자리로 잰다. path가 비면(point 모드) 부르지 않는다.
        """
        x, y = drifted(self._motion.x, self._motion.y, self._motion.yaw, self._faults.drift(t))
        return path_gap(x, y, path) > self._connect_tolerance

    def _execute(self, handle, action_type):
        goal = self._goals[bytes(handle.goal_id.uuid)]
        last_fb, last_phase = 0.0, ""
        while not goal.done.is_set() and not handle.is_cancel_requested:
            if not self._running():
                abort_at_shutdown(handle)
                return action_type.Result()
            now = time.monotonic()
            if not self._faults.comms_down(self._sc.now()):
                if goal.kind == "move" and now - last_fb >= FEEDBACK_PERIOD_S:
                    last_fb = now
                    with self._lock:
                        remaining = self._motion.remaining()
                    handle.publish_feedback(MoveTo.Feedback(distance_remaining=remaining))
                elif goal.kind == "dock":
                    phase = self._dock_phase(goal)
                    if phase != last_phase:
                        last_phase = phase
                        handle.publish_feedback(Dock.Feedback(phase=phase))
            time.sleep(0.02)
        if handle.is_cancel_requested and not goal.done.is_set():
            with self._lock:
                self._end(goal, False, MoveTo.Result.REASON_CANCELED)
        while self._running() and self._faults.comms_down(self._sc.now()):
            time.sleep(0.02)  # 통신이 돌아올 때까지 결과를 쥐고 있는다
        if not self._running():
            abort_at_shutdown(handle)
            return action_type.Result()
        self._goals.pop(bytes(handle.goal_id.uuid), None)
        result = self._result(goal, action_type)
        finish_action(handle, goal.success)
        return result
