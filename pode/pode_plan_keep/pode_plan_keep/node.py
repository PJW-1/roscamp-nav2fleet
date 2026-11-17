# ---- 계획 요청과 planner 사이에서 길을 지키는 노드 ----
import copy
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from .route import (
    BLOCKED,
    KEEP,
    OFF,
    OTHER,
    SHORTER,
    SILENT,
    Rules,
    first_look,
    length,
    second_look,
)

# ---- 이름과 상수 ----
SERVER = "compute_path_keep"
PLANNER = "compute_path_to_pose"
VALID = "is_path_valid"
POLL_S = 0.005
PLANNER_WAIT_S = 1.0
CANCEL_WAIT_S = 1.0
UNKNOWN = getattr(ComputePathToPose.Result, "UNKNOWN", 200)
REASONS = {OFF: "the robot is off the kept route", BLOCKED: "the kept route is blocked",
           SILENT: "no answer from is_path_valid", SHORTER: "the new plan is clearly shorter"}


# ---- 노드 생성과 파라미터 ----
class PlanKeep(Node):
    def __init__(self, **kwargs):
        super().__init__("plan_keep", **kwargs)
        declare = self.declare_parameter
        self.keep = bool(declare("keep", True).value)
        self.rules = Rules(same_route_m=float(declare("same_route_m", 0.20).value),
                           off_route_m=float(declare("off_route_m", 0.30).value),
                           shorter_m=float(declare("shorter_m", 0.30).value),
                           shorter_ratio=float(declare("shorter_ratio", 0.2).value),
                           same_goal_m=float(declare("same_goal_m", 0.05).value))
        self.valid_timeout_s = float(declare("valid_timeout_s", 0.2).value)
        self._lock = threading.Lock()
        self._kept = None
        self._kept_goal = None
        clients = ReentrantCallbackGroup()
        self.planner = ActionClient(self, ComputePathToPose, PLANNER, callback_group=clients)
        self.valid = self.create_client(IsPathValid, VALID, callback_group=clients)
        self.server = ActionServer(self, ComputePathToPose, SERVER, self.execute,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"{SERVER} -> {PLANNER}, keep the route: {self.keep}")

    # ---- 요청 하나 처리 ----
    def execute(self, handle):
        if not self.planner.wait_for_server(timeout_sec=PLANNER_WAIT_S):
            self.get_logger().warn(f"{PLANNER} is not up")
            return self._abort(handle, ComputePathToPose.Result(error_code=UNKNOWN))
        sent = self._wait(self.planner.send_goal_async(handle.request), handle)
        if sent is None:
            handle.canceled()
            return ComputePathToPose.Result()
        if not sent.accepted:
            return self._abort(handle, ComputePathToPose.Result(error_code=UNKNOWN))
        got = self._wait(sent.get_result_async(), handle)
        if got is None:
            self._cancel_inner(sent)
            handle.canceled()
            return ComputePathToPose.Result()
        result = got.result
        if got.status != GoalStatus.STATUS_SUCCEEDED:
            return self._abort(handle, result)
        if self.keep:
            result.path = self._choose(handle.request.goal, result.path)
        handle.succeed()
        return result

    def _abort(self, handle, result):
        self._remember(None, None)
        handle.abort()
        return result

    def _wait(self, future, handle):
        while not future.done():
            if handle.is_cancel_requested or not rclpy.ok(context=self.context):
                return None
            time.sleep(POLL_S)
        return future.result()

    def _cancel_inner(self, sent):
        sent.cancel_goal_async()
        result = sent.get_result_async()
        end = time.monotonic() + CANCEL_WAIT_S
        while not result.done() and time.monotonic() < end:
            time.sleep(POLL_S)

    # ---- 길 고르기 ----
    def _choose(self, goal, new_path):
        new = _points(new_path)
        goal_xy = (goal.pose.position.x, goal.pose.position.y)
        with self._lock:
            kept_path, kept_goal = self._kept, self._kept_goal
        kept = _points(kept_path) if kept_path is not None else []
        look, start = first_look(kept, kept_goal, new, goal_xy, self.rules)
        if look not in (OFF, OTHER):
            self._remember(new_path, goal_xy)
            return new_path
        rest = kept[start:]
        verdict = OFF if look == OFF else second_look(rest, new, self._is_valid(kept_path, start),
                                                      self.rules)
        lengths = f"new plan {length(new):.2f} m"
        if verdict == KEEP:
            kept_path = self._restamped(kept_path, start)
            self._remember(kept_path, goal_xy)
            self.get_logger().info(f"kept the route: {lengths} by another way, "
                                   f"kept {length(rest):.2f} m")
            return kept_path
        self._remember(new_path, goal_xy)
        self.get_logger().info(f"changed the route: {lengths}, old {length(rest):.2f} m "
                               f"({REASONS[verdict]})")
        return new_path

    def _remember(self, path, goal_xy):
        with self._lock:
            self._kept, self._kept_goal = path, goal_xy

    def _is_valid(self, kept_path, start):
        if not self.valid.service_is_ready():
            return None
        request = IsPathValid.Request()
        request.path = Path(header=kept_path.header, poses=kept_path.poses[start:])
        future = self.valid.call_async(request)
        end = time.monotonic() + self.valid_timeout_s
        while not future.done() and time.monotonic() < end:
            time.sleep(POLL_S)
        if not future.done():
            self.valid.remove_pending_request(future)
            return None
        return bool(future.result().is_valid)

    def _restamped(self, kept_path, start):
        stamp = self.get_clock().now().to_msg()
        path = Path(poses=copy.deepcopy(kept_path.poses[start:]))
        path.header.frame_id = kept_path.header.frame_id
        path.header.stamp = stamp
        for pose in path.poses:
            pose.header.stamp = stamp
        return path


# ---- 경로 점 ----
def _points(path) -> list:
    return [(p.pose.position.x, p.pose.position.y) for p in path.poses]


# ---- 여러 로봇을 한 프로세스에 ----
def make_fleet(robots: list, keep: bool = True, use_sim_time: bool = False,
               context=None) -> list:
    return [PlanKeep(namespace=f"/{name}", context=context, use_global_arguments=False,
                     parameter_overrides=[Parameter("keep", value=keep),
                                          Parameter("use_sim_time", value=use_sim_time)])
            for name in robots]


# ---- 실행 ----
def main():
    rclpy.init()
    node = PlanKeep()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
