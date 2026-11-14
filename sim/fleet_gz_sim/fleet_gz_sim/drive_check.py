"""관제 없이 Gazebo 핑키 한 대에 이동을 바로 보내고 2단계 지표를 잰다(지시서 6-3, 7절 2단계).

    ros2 run fleet_gz_sim drive_check --layout d2 --plan stage2 --laps 3 --json out.json

stage2_plan의 계획(nav_goal, follow_path, retreat)을 차례로 보낸다. 앞 이동이 성공해야 다음을
보낸다. /fleet/heartbeat는 2 Hz로 보낸다. 이동마다:
  - 결과(성공, 실패 이유), 걸린 시간(Gazebo 시각, 실제 위치 기록의 stamp)
  - 끝났을 때 실제 위치(/{robot}/sim/truth)와 목표 사이 거리, 방향 오차
  - follow_path, retreat: 실제 위치가 고정 경로에서 가장 멀리 벗어난 거리
  - nav_goal: Nav2가 낸 계획 경로(/{robot}/plan)와 실제 주행이 지난 keepout 구역(shared 아닌 구역)
  - 그동안 로봇이 보고한 상태(BLOCKED, RECOVERING 등)
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time

import rclpy
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.fleet_map import NAV_GOAL, RETREAT
from fleet_core.traffic.site import load_site
from fleet_interfaces.action import MoveTo, Retreat
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header

from fleet_gz_sim.stage2_plan import (
    build,
    cross_track,
    keepout_zones,
    start_node,
    yaw_error,
)

HEARTBEAT_PERIOD_S = 0.5
START_M = 0.15  # 로봇이 계획의 시작 지점에서 이만큼 안에 있어야 한다
RETREAT_SPEED_MPS = 0.10  # TRAFFIC_CONTRACT 6절 retreat_speed_mps
SETTLE_S = 0.3  # 결과 뒤 실제 위치를 이만큼 더 받는다(멈춘 자리)


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _pose(x: float, y: float, yaw: float) -> Pose:
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class DriveCheck(Node):
    def __init__(self, robot: str, contract_path: str):
        super().__init__("drive_check")
        contract = load_contract(contract_path)
        self.lock = threading.Lock()
        self.state: RobotState | None = None
        self.states: list[str] = []
        self.truth: list[tuple[float, float, float, float]] = []  # (sim t, x, y, yaw)
        self.plans: list[list[tuple[float, float]]] = []
        self.create_subscription(RobotState, contract.name("robot_state", pinky=robot),
                                 self._on_state, to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(Odometry, f"/{robot}/sim/truth", self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, f"/{robot}/plan", self._on_plan, 10)
        self.move = ActionClient(self, MoveTo, contract.name("move_to", pinky=robot))
        self.retreat = ActionClient(self, Retreat, contract.name("retreat", pinky=robot))
        self.beat = self.create_publisher(Header, contract.name("heartbeat"),
                                          to_ros_qos(contract.qos("heartbeat")))
        self.create_timer(HEARTBEAT_PERIOD_S,
                          lambda: self.beat.publish(Header(frame_id="drive_check")),
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_state(self, msg):
        with self.lock:
            self.state = msg
            if not self.states or self.states[-1] != msg.state:
                self.states.append(msg.state)

    def _on_truth(self, msg):
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.truth.append((t, p.position.x, p.position.y, _yaw(p.orientation)))

    def _on_plan(self, msg):
        with self.lock:
            self.plans.append([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])

    def last_truth(self):
        with self.lock:
            return self.truth[-1] if self.truth else None

    def send(self, step) -> dict:
        """이동 하나를 보내고 결과를 기다리지 않는다. 결과는 돌려준 dict에 채워진다."""
        x, y, yaw = step.target
        if step.kind == RETREAT:
            length = sum(math.dist(a[:2], b[:2])
                         for a, b in zip(step.poses, step.poses[1:], strict=False))
            goal = Retreat.Goal(distance_m=float(length), speed_mps=RETREAT_SPEED_MPS,
                                path=[_pose(*p) for p in step.poses])
            client = self.retreat
        else:
            goal = MoveTo.Goal(target_id=step.nodes[-1],
                               target_pose=Pose2D(x=x, y=y, theta=yaw), final_goal=True,
                               mode=step.kind)
            if step.kind != NAV_GOAL:
                goal.path = [_pose(*p) for p in step.poses]
            client = self.move
        record = {"done": False, "handle": None}
        fut = client.send_goal_async(goal)
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        if not fut.done():  # a late acceptance must not leave the robot driving
            fut.add_done_callback(lambda f: f.result().accepted and f.result().cancel_goal_async())
        handle = fut.result() if fut.done() else None
        if handle is None or not handle.accepted:
            record.update(done=True, success=False, reason="NOT_ACCEPTED")
            return record
        record["handle"] = handle

        def finished(f, record=record):
            res = f.result().result
            record.update(done=True, success=bool(res.success), reason=res.reason)
        handle.get_result_async().add_done_callback(finished)
        return record


def wait_for(predicate, timeout: float, step: float = 0.05) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


def run_step(node: DriveCheck, site, step, timeout: float) -> dict:
    with node.lock:
        node.states, node.plans = [], []
        first = len(node.truth)
    wall = time.monotonic()
    record = node.send(step)
    if not wait_for(lambda: record["done"], timeout, 0.1) and record["handle"] is not None:
        record["handle"].cancel_goal_async()  # stop the robot instead of leaving the goal running
    wall_s = time.monotonic() - wall
    ended = node.last_truth()  # sim time of the result; the settle wait below is wall time
    time.sleep(SETTLE_S)
    with node.lock:
        truth, states, plans = node.truth[first:], list(node.states), list(node.plans)
    result = "success" if record.get("success") else record.get("reason", "timeout")
    out = {"step": step.label, "result": result, "wall_s": round(wall_s, 2), "states": states}
    if truth:
        t0 = truth[0][0]
        t1, x, y, yaw = truth[-1]
        tx, ty, tyaw = step.target
        sim_end = ended[0] if ended is not None else t1
        out.update(sim_s=round(sim_end - t0, 2), final=[round(x, 3), round(y, 3)],
                   pos_error_m=round(math.dist((x, y), (tx, ty)), 3),
                   yaw_error_deg=round(math.degrees(yaw_error(yaw, tyaw)), 1))
        if step.kind == NAV_GOAL:
            out["truth_keepout"] = keepout_zones(site, [(px, py) for _, px, py, _ in truth])
        else:
            out["max_cross_track_m"] = round(max(cross_track(step.poses, px, py)
                                                 for _, px, py, _ in truth), 3)
    if step.kind == NAV_GOAL:
        out["plans"] = len(plans)
        out["plan_keepout"] = sorted({z for plan in plans for z in keepout_zones(site, plan)})
    return out


def run(args) -> dict:
    site = load_site(args.layout)
    steps = build(site, args.plan, args.laps)
    node = DriveCheck(args.robot, args.contract)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if not (node.move.wait_for_server(timeout_sec=30.0)
                and node.retreat.wait_for_server(timeout_sec=30.0)):
            raise RuntimeError("move_to or retreat server not found: is pinky_fsm running?")
        if not wait_for(lambda: node.state is not None and node.state.state == "IDLE"
                        and node.last_truth() is not None, 90.0):
            raise RuntimeError("robot is not IDLE with a pose (AMCL, Gazebo truth)")
        _, x, y, _ = node.last_truth()
        begin = start_node(args.plan)
        if math.dist((x, y), site.node_pose(begin)) > START_M:
            raise RuntimeError(f"robot at ({x:.2f}, {y:.2f}) is not on {begin}")
        results = []
        for step in steps:
            wait_for(lambda: node.state is not None and node.state.state == "IDLE", 10.0)
            results.append(run_step(node, site, step, args.timeout))
            print(json.dumps(results[-1], ensure_ascii=False), flush=True)
            if results[-1]["result"] != "success":
                break
            time.sleep(1.0)
        return {"layout": args.layout, "plan": args.plan, "laps": args.laps,
                "steps": len(steps), "done": len(results),
                "succeeded": sum(1 for r in results if r["result"] == "success"),
                "results": results}
    finally:
        executor.shutdown()
        node.destroy_node()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", default="pinky1")
    parser.add_argument("--layout", default="d2")
    parser.add_argument("--plan", default="stage2")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="이동 하나의 한도(벽시계 초)")
    parser.add_argument("--contract", default=None)
    parser.add_argument("--json", default=None, help="요약을 이 파일에도 쓴다")
    args = parser.parse_args(argv)
    if args.contract is None:
        from ament_index_python.packages import get_package_share_directory
        args.contract = get_package_share_directory("fleet_interfaces") + "/contract.yaml"
    rclpy.init()
    try:
        summary = run(args)
    finally:
        rclpy.try_shutdown()
    text = json.dumps(summary, ensure_ascii=False, indent=2)
    print(text)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
