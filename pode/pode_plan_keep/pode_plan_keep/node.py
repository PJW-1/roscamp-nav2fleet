# ---- 계획 요청과 planner 사이에서 길을 지키는 노드 ----
import copy
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import route
from .route import (
    BLOCKED,
    KEEP,
    OFF,
    OPEN,
    OTHER,
    SHORTER,
    SILENT,
    Grid,
    Rules,
    check_route,
    first_look,
    length,
    second_look,
)

# ---- 이름과 상수 ----
SERVER = "compute_path_keep"
PLANNER = "compute_path_to_pose"
COSTMAP = "global_costmap/costmap"
COSTMAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
POLL_S = 0.005
PLANNER_WAIT_S = 1.0
CANCEL_WAIT_S = 1.0
MAX_RUNNING = 2
THREADS = MAX_RUNNING + 2
UNKNOWN_ERROR = getattr(ComputePathToPose.Result, "UNKNOWN", 200)
REASONS = {OFF: "the robot is off the kept route", SILENT: "no global costmap to check",
           SHORTER: "the new plan is clearly shorter"}


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
        self.blocked_cost = int(declare("blocked_cost", 99).value)
        self._lock = threading.Lock()
        self._kept = None
        self._kept_goal = None
        self._running = 0
        self.grid = None
        self.create_subscription(OccupancyGrid, COSTMAP, self._on_costmap, COSTMAP_QOS)
        self.planner = ActionClient(self, ComputePathToPose, PLANNER,
                                    callback_group=ReentrantCallbackGroup())
        self.server = ActionServer(self, ComputePathToPose, SERVER, self.execute,
                                   goal_callback=self._take,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"{SERVER} -> {PLANNER}, keep the route: {self.keep}")

    # ---- 전역 비용 지도 ----
    def _on_costmap(self, msg):
        info = msg.info
        q = info.origin.orientation
        self.grid = Grid(origin_x=info.origin.position.x, origin_y=info.origin.position.y,
                         yaw=2.0 * math.atan2(q.z, q.w), resolution=info.resolution,
                         width=info.width, height=info.height, data=msg.data)

    # ---- 동시에 처리하는 요청 수 ----
    def _take(self, _request):
        with self._lock:
            if self._running >= MAX_RUNNING:
                self.get_logger().warn(f"{MAX_RUNNING} requests running, refusing another")
                return GoalResponse.REJECT
            self._running += 1
        return GoalResponse.ACCEPT

    def execute(self, handle):
        try:
            return self._execute(handle)
        except Exception:
            self._remember(None, None)
            raise
        finally:
            with self._lock:
                self._running -= 1

    # ---- 요청 하나 처리 ----
    def _execute(self, handle):
        if not self.planner.wait_for_server(timeout_sec=PLANNER_WAIT_S):
            self.get_logger().warn(f"{PLANNER} is not up")
            return self._abort(handle, ComputePathToPose.Result(error_code=UNKNOWN_ERROR))
        sending = self.planner.send_goal_async(handle.request)
        sent = self._wait(sending, handle)
        if sent is None:
            sending.add_done_callback(_cancel_when_accepted)
            handle.canceled()
            return ComputePathToPose.Result()
        if not sent.accepted:
            return self._abort(handle, ComputePathToPose.Result(error_code=UNKNOWN_ERROR))
        got = self._wait(sent.get_result_async(), handle)
        if got is None:
            self._cancel_inner(sent)
            handle.canceled()
            return ComputePathToPose.Result()
        result = got.result
        if got.status != GoalStatus.STATUS_SUCCEEDED:
            return self._abort(handle, result)
        goal = handle.request.goal.pose.position
        remember = None
        if self.keep:
            result.path, remember = self._choose((goal.x, goal.y), result.path)
        if handle.is_cancel_requested:
            handle.canceled()
            return ComputePathToPose.Result()
        if self.keep:
            self._remember(result.path, remember)
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
    def _choose(self, goal_xy, new_path):
        new = _points(new_path)
        with self._lock:
            kept_path, kept_goal = self._kept, self._kept_goal
        kept = _points(kept_path) if kept_path is not None else []
        look, start = first_look(kept, kept_goal, new, goal_xy, self.rules)
        if look not in (OFF, OTHER):
            return new_path, goal_xy
        rest = kept[start:]
        reason = None
        if look == OFF:
            verdict = OFF
        else:
            found = check_route(self.grid, rest, self.blocked_cost)
            valid = None if found == route.UNKNOWN else found == OPEN
            verdict = second_look(rest, new, valid, self.rules)
            if verdict == BLOCKED:
                x, y, cost = found
                where = "outside the costmap" if cost is None else f"cost {cost}"
                reason = f"the kept route is blocked at ({x:.2f}, {y:.2f}) {where}"
        lengths = f"new plan {length(new):.2f} m"
        if verdict == KEEP:
            self.get_logger().info(f"kept the route: {lengths} by another way, "
                                   f"kept {length(rest):.2f} m")
            return self._restamped(kept_path, start, new_path), kept_goal
        self.get_logger().info(f"changed the route: {lengths}, old {length(rest):.2f} m "
                               f"({reason or REASONS[verdict]})")
        return new_path, goal_xy

    def _remember(self, path, goal_xy):
        with self._lock:
            self._kept, self._kept_goal = path, goal_xy

    def _restamped(self, kept_path, start, new_path):
        stamp = self.get_clock().now().to_msg()
        path = Path(poses=copy.deepcopy(kept_path.poses[start:]))
        path.header.frame_id = kept_path.header.frame_id
        path.header.stamp = stamp
        for pose in path.poses:
            pose.header.stamp = stamp
        path.poses[-1].pose.orientation = copy.deepcopy(new_path.poses[-1].pose.orientation)
        return path


# ---- 승인된 뒤에 취소하기 ----
def _cancel_when_accepted(future):
    sent = future.result()
    if sent is not None and sent.accepted:
        sent.cancel_goal_async()


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
    executor = MultiThreadedExecutor(num_threads=THREADS)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
