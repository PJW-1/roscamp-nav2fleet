"""plan_keep 노드: 가짜 planner(compute_path_to_pose)와 가짜 전역 비용 지도
(global_costmap/costmap)를 같은 프로세스에 띄우고 compute_path_keep에 묻는다
(ROS 2 rclpy와 nav2_msgs가 있을 때만).

ROS_DOMAIN_ID는 환경 그대로 쓰고, 다른 실행과 섞이지 않게 이 프로세스만의 namespace 안에서 돈다.
"""
import math
import os
import threading
import time
from collections import deque

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")

from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

NAMESPACE = f"/plan_keep_test_{os.getpid()}"
OLD_STAMP_S = 1
NO_VALID_PATH = 208
GOAL = (1.0, 1.0)
# Jazzy Costmap2DPublisher가 global_costmap/costmap을 내는 QoS
# (KeepLast 1, transient_local, reliable)
COSTMAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)


def line(*corners, step=0.05):
    points = [corners[0]]
    for a, b in zip(corners, corners[1:], strict=False):
        n = max(1, round(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 / step))
        points += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                   for i in range(1, n + 1)]
    return points


EAST = line((0.0, 0.0), (1.0, 0.0), GOAL)
NORTH = line((0.0, 0.0), (0.0, 1.0), GOAL)
EAST_NEAR = line((0.0, 0.0), (0.0, 0.1), (0.9, 0.1), GOAL)


def as_path(points) -> Path:
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp.sec = OLD_STAMP_S
    for x, y in points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def points_of(path) -> list:
    return [(round(p.pose.position.x, 6), round(p.pose.position.y, 6)) for p in path.poses]


def rounded(points) -> list:
    return [(round(x, 6), round(y, 6)) for x, y in points]


class FakePlanner:
    """plans: 다음 계획들(점 목록, 오류 코드 int, 또는 "hang").
    heading_z: 계획 끝 자세의 방향(쿼터니언 z, w는 맞춘다).
    costmap(): x -0.5~1.5, y -0.5~1.5, 5 cm 격자를 낸다. cells는 {(x, y): 값}."""

    def __init__(self, node):
        group = ReentrantCallbackGroup()
        self.plans, self.canceled, self.heading_z = deque(), 0, 0.0
        self.server = ActionServer(node, ComputePathToPose, "compute_path_to_pose", self._plan,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=group)
        self.costmap_pub = node.create_publisher(OccupancyGrid, "global_costmap/costmap",
                                                 COSTMAP_QOS)

    def costmap(self, keeper, cells=None, broken=False):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution, grid.info.width, grid.info.height = 0.05, 40, 40
        grid.info.origin.position.x = grid.info.origin.position.y = -0.5
        grid.info.origin.orientation.w = 1.0
        data = [0] * (40 * 40)
        for (x, y), value in (cells or {}).items():
            data[math.floor((y + 0.5) / 0.05) * 40 + math.floor((x + 0.5) / 0.05)] = value
        grid.data = data[:10] if broken else data
        self.costmap_pub.publish(grid)
        end = time.monotonic() + 5.0
        while keeper.grid is None or list(keeper.grid.data) != list(grid.data):
            assert time.monotonic() < end, "plan_keep did not get the costmap"
            time.sleep(0.01)

    def _plan(self, handle):
        plan = self.plans.popleft()
        result = ComputePathToPose.Result()
        if plan == "hang":
            while not handle.is_cancel_requested:
                time.sleep(0.01)
            self.canceled += 1
            handle.canceled()
            return result
        if isinstance(plan, int):
            result.error_code = plan
            handle.abort()
            return result
        result.path = as_path(plan)
        end = result.path.poses[-1].pose.orientation
        end.z, end.w = self.heading_z, (1.0 - self.heading_z ** 2) ** 0.5
        handle.succeed()
        return result


@pytest.fixture
def ros():
    rclpy.init(args=["--ros-args", "-r", f"__ns:={NAMESPACE}"])
    nodes, executor = [], MultiThreadedExecutor(num_threads=8)

    def start(*overrides):
        from pode_plan_keep.node import PlanKeep
        keeper = PlanKeep(parameter_overrides=list(overrides))
        fake_node = rclpy.create_node("fake_planner_server")
        fake = FakePlanner(fake_node)
        client_node = rclpy.create_node("plan_keep_client")
        client = ActionClient(client_node, ComputePathToPose, "compute_path_keep")
        for node in (keeper, fake_node, client_node):
            nodes.append(node)
            executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
        assert client.wait_for_server(timeout_sec=5.0)
        return keeper, fake, client

    yield start
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()


def wait(future, timeout=5.0):
    end = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < end, "no answer"
        time.sleep(0.005)
    return future.result()


def ask(client, goal=GOAL):
    request = ComputePathToPose.Goal()
    request.goal.header.frame_id = "map"
    request.goal.pose.position.x, request.goal.pose.position.y = goal
    request.goal.pose.orientation.w = 1.0
    handle = wait(client.send_goal_async(request))
    assert handle.accepted
    got = wait(handle.get_result_async())
    return got.status, got.result


def test_the_server_names_match_the_tree_and_nav2():
    from pode_plan_keep import node
    assert (node.SERVER, node.PLANNER, node.COSTMAP) == (
        "compute_path_keep", "compute_path_to_pose", "global_costmap/costmap")


def test_another_way_to_the_same_goal_keeps_the_route_with_fresh_stamps(ros):
    keeper, fake, client = ros()
    fake.costmap(keeper, {(0.5, 0.5): 100})  # 두 길 사이의 장애물은 상관없다
    fake.plans.extend([EAST, NORTH])
    status, result = ask(client)
    assert status == GoalStatus.STATUS_SUCCEEDED and points_of(result.path) == rounded(EAST)
    status, result = ask(client)
    assert status == GoalStatus.STATUS_SUCCEEDED
    assert points_of(result.path) == rounded(EAST), "the kept route, not the new plan"
    assert result.path.header.stamp.sec > OLD_STAMP_S
    assert all(p.header.stamp.sec > OLD_STAMP_S for p in result.path.poses)
    assert result.path.header.frame_id == "map"


def test_the_same_route_always_takes_the_new_plan(ros):
    keeper, fake, client = ros()
    fake.costmap(keeper)
    fake.plans.extend([EAST, EAST_NEAR])
    ask(client)
    status, result = ask(client)
    assert status == GoalStatus.STATUS_SUCCEEDED
    assert points_of(result.path) == rounded(EAST_NEAR)


@pytest.mark.parametrize("cells", [{(0.51, 0.01): 99}, {(0.51, 0.01): 100}, None])
def test_a_blocked_kept_route_or_no_costmap_takes_the_new_plan(ros, cells):
    keeper, fake, client = ros()
    if cells is not None:
        fake.costmap(keeper, cells)
    fake.plans.extend([EAST, NORTH, EAST])
    ask(client)
    status, result = ask(client)
    assert status == GoalStatus.STATUS_SUCCEEDED and points_of(result.path) == rounded(NORTH)
    fake.costmap(keeper)
    _, result = ask(client)  # 이제는 NORTH를 지켜 둔다
    assert points_of(result.path) == rounded(NORTH)


def test_a_planner_failure_is_passed_on_and_forgets_the_kept_route(ros):
    keeper, fake, client = ros()
    fake.plans.extend([EAST, NO_VALID_PATH, NORTH])
    fake.costmap(keeper)
    ask(client)
    status, result = ask(client)
    assert status == GoalStatus.STATUS_ABORTED and result.error_code == NO_VALID_PATH
    status, result = ask(client)
    assert status == GoalStatus.STATUS_SUCCEEDED and points_of(result.path) == rounded(NORTH)


def test_keep_false_always_passes_the_planner_result_on(ros):
    keeper, fake, client = ros(Parameter("keep", value=False))
    fake.costmap(keeper)
    fake.plans.extend([EAST, NORTH])
    ask(client)
    _, result = ask(client)
    assert points_of(result.path) == rounded(NORTH)


def test_a_canceled_request_cancels_the_planner_goal(ros):
    _, fake, client = ros()
    fake.plans.append("hang")
    request = ComputePathToPose.Goal()
    request.goal.pose.position.x, request.goal.pose.position.y = GOAL
    handle = wait(client.send_goal_async(request))
    time.sleep(0.2)
    wait(handle.cancel_goal_async())
    got = wait(handle.get_result_async())
    assert got.status == GoalStatus.STATUS_CANCELED
    assert fake.canceled == 1


def test_a_kept_route_ends_with_the_new_goal_heading(ros):
    """goal 위치가 같고 방향만 바뀌면 지켜 둔 길의 끝 자세는 새 계획의 끝 방향을 따른다."""
    keeper, fake, client = ros()
    fake.costmap(keeper)
    fake.plans.extend([EAST, NORTH])
    ask(client)
    fake.heading_z = 0.7071068
    _, result = ask(client)
    assert points_of(result.path) == rounded(EAST)
    assert result.path.poses[-1].pose.orientation.z == pytest.approx(0.7071068)


def test_a_kept_route_keeps_the_goal_it_was_made_for(ros):
    """goal이 5 cm 안에서 조금씩 움직여도 지켜 둔 길의 goal은 그 길을 만든 goal이다(오차가 쌓이지
    않게). 처음 goal에서 5 cm 넘게 멀어지면 새 goal로 본다."""
    keeper, fake, client = ros()
    fake.costmap(keeper)
    fake.plans.extend([EAST, NORTH, NORTH])
    ask(client)
    _, result = ask(client, (1.04, 1.0))
    assert points_of(result.path) == rounded(EAST)
    _, result = ask(client, (1.08, 1.0))
    assert points_of(result.path) == rounded(NORTH)


def test_an_error_while_choosing_forgets_the_kept_route(ros):
    keeper, fake, client = ros()
    fake.costmap(keeper)
    fake.plans.extend([EAST, NORTH, NORTH])
    ask(client)
    fake.costmap(keeper, broken=True)  # 격자 값이 모자라 판정에서 예외가 난다
    status, _ = ask(client)
    assert status == GoalStatus.STATUS_ABORTED
    fake.costmap(keeper)
    _, result = ask(client)
    assert points_of(result.path) == rounded(NORTH), "nothing kept after the error"


def test_more_than_two_requests_at_once_are_refused(ros):
    _, fake, client = ros()
    fake.plans.extend(["hang", "hang", "hang"])
    request = ComputePathToPose.Goal()
    handles = [wait(client.send_goal_async(request)) for _ in range(3)]
    assert [h.accepted for h in handles] == [True, True, False]
    for handle in handles[:2]:
        wait(handle.cancel_goal_async())
        assert wait(handle.get_result_async()).status == GoalStatus.STATUS_CANCELED


def test_the_rule_values_are_parameters(ros):
    keeper, _, _ = ros(Parameter("same_route_m", value=0.1), Parameter("blocked_cost", value=100))
    assert keeper.rules.same_route_m == 0.1 and keeper.blocked_cost == 100
    assert keeper.rules.off_route_m == 0.30 and keeper.keep is True


def test_one_process_can_host_a_node_for_every_robot():
    """Gazebo 세 대는 DDS 참여자를 아끼려고 sim_stand_in 프로세스 안에 로봇마다 하나씩 둔다."""
    rclpy.init(args=["--ros-args", "-r", "__node:=sim_stand_in", "-r", f"__ns:={NAMESPACE}"])
    try:
        from pode_plan_keep.node import make_fleet
        robots = [f"{NAMESPACE[1:]}_a", f"{NAMESPACE[1:]}_b"]  # 진짜 로봇 이름과 겹치지 않게
        keepers = make_fleet(robots, keep=False, use_sim_time=True)
        try:
            assert [k.get_fully_qualified_name() for k in keepers] == [
                f"/{robot}/plan_keep" for robot in robots]
            assert [k.keep for k in keepers] == [False, False]
            assert all(k.get_parameter("use_sim_time").value for k in keepers)
        finally:
            for keeper in keepers:
                keeper.destroy_node()
    finally:
        rclpy.try_shutdown()
