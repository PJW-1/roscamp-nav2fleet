"""가짜 핑키의 상태 보고: 진실 토픽과 지연을 거친 robot_state 발행."""
from __future__ import annotations

from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose2D

from fake_robots.motion import drifted, noisy_pose
from fake_robots.pinky_goal import _Goal


class PinkyReport:
    def _publish_state(self) -> None:
        with self._lock:
            t = self._sc.now()
            self._truth_pub.publish(RobotState(
                robot_id=self.robot_id,
                pose=Pose2D(x=self._motion.x, y=self._motion.y, theta=self._motion.yaw)))
            if self._unreachable():
                self._reports.clear()  # 못 나간 보고는 링크와 함께 사라진다
                return
            goal = self._active
            busy = goal is not None and not goal.done.is_set()
            battery = self._battery.percent(t, self._motion.x, self._motion.y,
                                            not self._motion.moving)
            if t < self._cfg.localizing_s:
                # 위치를 아직 못 잡았다. 자세는 아무 값이다.
                self._reports.push(t, RobotState(
                    robot_id=self.robot_id, pose=Pose2D(), battery=battery,
                    state=RobotState.STATE_LOCALIZING, stamp=self.get_clock().now().to_msg()))
            else:
                self._push_report(t, goal, busy, battery)
            due = self._reports.due(t)
        for msg in due:
            self._state_pub.publish(msg)

    def _push_report(self, t: float, goal: _Goal | None, busy: bool, battery: float) -> None:
        if busy and goal.kind == "move" and self._recovery.recovering:
            state = RobotState.STATE_RECOVERING  # 명세 8번: LOCALIZING 다음
        elif self._recovery.blocked or (busy and goal.blocked(t)):
            state = RobotState.STATE_BLOCKED
        elif busy:
            state = RobotState.STATE_DOCKING if goal.kind == "dock" else RobotState.STATE_MOVING
        else:
            state = self._rest
        x, y = noisy_pose(self._motion.x, self._motion.y, self._cfg.pose_noise_m, self._rng)
        x, y = drifted(x, y, self._motion.yaw, self._faults.drift(t))  # 진실 토픽은 그대로
        self._reports.push(t, RobotState(
            robot_id=self.robot_id, pose=Pose2D(x=x, y=y, theta=self._motion.yaw),
            battery=battery, state=state,
            current_goal=goal.target_id if busy else "",  # Pinky contract section 2
            stamp=self.get_clock().now().to_msg()))
