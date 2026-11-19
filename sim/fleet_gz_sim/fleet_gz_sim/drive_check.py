"""관제 없이 Gazebo Pode 한 대에 이동을 바로 보내고 2단계 지표를 잰다.

    ros2 run fleet_gz_sim drive_check --layout d2 --plan stage2 --laps 3 --json out.json

stage2_plan의 계획(nav_goal, follow_path, retreat)을 차례로 보낸다. 앞 이동이 성공해야 다음을
보낸다. /fleet/heartbeat는 2 Hz로 보낸다. 이동마다:
  - 결과(성공, 실패 이유), 걸린 시간(Gazebo 시각, 실제 위치 기록의 stamp)
  - 끝났을 때 실제 위치(/{robot}/sim/truth)와 목표 사이 거리, 방향 오차
  - follow_path, retreat: 실제 위치가 고정 경로에서 가장 멀리 벗어난 거리
  - nav_goal: Nav2가 낸 계획 경로(/{robot}/plan)와 실제 주행이 지난 keepout 구역(shared 아닌 구역)
  - 그동안 로봇이 보고한 상태(BLOCKED, RECOVERING 등)와 BLOCKED, RECOVERING에 들어간 횟수
  - nav_goal: 실제 위치를 0.05 m마다 줄인 자취(track). v7은 가운데 통로를 지났는지(through_corridor)
  - 상자(/sim/obstacle_markers)가 놓인 동안 실제 위치와 상자 중심 사이의 가장 짧은 거리(box_min_m)

  - 실제 위치와 시각이 가장 가까운 AMCL 추정(/{robot}/amcl_pose) 사이의 가장 큰
    거리(amcl_max_error_m)
  - v7: 남쪽 진열대 사이 틈을 지났는지(through_gap)와 틈의 네 모서리까지 가장 짧은 거리(중심
    gap_corner_min_m, 몸 gap_corner_body_min_m: local costmap 발자국 다각형, 못 읽으면 중심 - 0.056)

인자 없이 돌리면 v7(실제 경기장)의 corridor 계획이다. --plan만 주면 그 계획의 배치(stage2, cross는 d2).
가운데 통로 재현(v7): --layout v7 --plan corridor --wait-box 120. 상자가 놓인 뒤에 출발한다.
틈 건너기(v7): --layout v7 --plan gap. 끝점은 --point GAP_N=1.12,0.55처럼 바꾼다.
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
from geometry_msgs.msg import Pose, Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from fleet_gz_sim.quaternion import yaw_of
from fleet_gz_sim.stage2_plan import (
    CORRIDOR_GATES,
    GAP_GATES,
    body_distance,
    build,
    cross_track,
    decimate,
    keepout_zones,
    max_pose_error,
    min_distance,
    parse_footprint,
    pick_layout_plan,
    start_node,
    through_gate,
    yaw_error,
)

HEARTBEAT_PERIOD_S = 0.5
START_M = 0.15  # 로봇이 계획의 시작 지점에서 이만큼 안에 있어야 한다
RETREAT_SPEED_MPS = 0.05  # TRAFFIC_CONTRACT 6절 retreat_speed_mps
SETTLE_S = 0.3  # 결과 뒤 실제 위치를 이만큼 더 받는다(멈춘 자리)
TRACK_STEP_M = 0.05  # 자취를 줄이는 간격
AMCL_MATCH_S = 0.1  # AMCL 추정과 실제 위치의 시각 차 한도(실제 위치는 약 30 Hz)
BODY_HALF_WIDTH_M = 0.056  # 발자국을 못 읽을 때 몸 거리 = 중심 거리 - 이것
# obstacle_node의 MARKER_QOS와 같다(늦게 붙어도 마지막 표시를 받는다)
MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def box_centres(msg) -> dict[str, tuple[float, float]]:
    """상자 표시 한 묶음에서 놓인 상자 이름 -> 중심. 묶음마다 전체를 다시 낸다(obstacle_node)."""
    return {f"obstacle_{m.id}": (m.pose.position.x, m.pose.position.y)
            for m in msg.markers
            if m.action == Marker.ADD and m.type == Marker.CUBE and m.ns == "obstacles"}


def load_footprint(path: str | None = None) -> list[tuple[float, float]] | None:
    """pode_nav_config nav2_params의 local costmap 발자국(base_footprint 틀). 못 읽으면 None."""
    import yaml
    try:
        if path is None:
            from ament_index_python.packages import get_package_share_directory
            path = get_package_share_directory("pode_nav_config") + "/params/nav2_params.yaml"
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return parse_footprint(
            data["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
    except Exception:  # noqa: BLE001 - 없으면 중심 거리로 잰다
        return None


def gap_corners(gate) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = gate
    return [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]


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
        self.boxes: dict[str, tuple[float, float]] = {}  # 지금 놓인 상자
        self.box_min: dict[str, float] = {}  # 이동마다: 상자 -> 놓인 동안 가장 짧은 거리
        self.amcl: list[tuple[float, float, float]] = []  # 이동마다: (sim t, x, y)
        self.footprint = load_footprint()
        self.create_subscription(RobotState, contract.name("robot_state", pode=robot),
                                 self._on_state, to_ros_qos(contract.qos("robot_state")))
        self.create_subscription(Odometry, f"/{robot}/sim/truth", self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, f"/{robot}/plan", self._on_plan, 10)
        self.create_subscription(MarkerArray, "/sim/obstacle_markers", self._on_boxes, MARKER_QOS)
        self.create_subscription(PoseWithCovarianceStamped, f"/{robot}/amcl_pose",
                                 self._on_amcl, 10)
        self.move = ActionClient(self, MoveTo, contract.name("move_to", pode=robot))
        self.retreat = ActionClient(self, Retreat, contract.name("retreat", pode=robot))
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
            self.truth.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
            for name, centre in self.boxes.items():
                d = math.dist((p.position.x, p.position.y), centre)
                self.box_min[name] = min(d, self.box_min.get(name, math.inf))

    def _on_amcl(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.amcl.append((t, p.x, p.y))

    def _on_boxes(self, msg):
        with self.lock:
            self.boxes = box_centres(msg)

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
        node.states, node.plans, node.box_min, node.amcl = [], [], {}, []
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
        box_min, amcl = dict(node.box_min), list(node.amcl)
    result = "success" if record.get("success") else record.get("reason", "timeout")
    out = {"step": step.label, "result": result, "wall_s": round(wall_s, 2), "states": states,
           "blocked": states.count("BLOCKED"), "recovering": states.count("RECOVERING"),
           "box_min_m": {name: round(d, 3) for name, d in sorted(box_min.items())}}
    error = max_pose_error(amcl, truth, AMCL_MATCH_S)  # 잴 수 없으면 None
    out["amcl_max_error_m"] = None if error is None else round(error, 3)
    if truth:
        t0 = truth[0][0]
        t1, x, y, yaw = truth[-1]
        tx, ty, tyaw = step.target
        sim_end = ended[0] if ended is not None else t1
        out.update(sim_s=round(sim_end - t0, 2), final=[round(x, 3), round(y, 3)],
                   pos_error_m=round(math.dist((x, y), (tx, ty)), 3),
                   yaw_error_deg=round(math.degrees(yaw_error(yaw, tyaw)), 1))
        if step.kind == NAV_GOAL:
            xy = [(px, py) for _, px, py, _ in truth]
            out["truth_keepout"] = keepout_zones(site, xy)
            if site.name in CORRIDOR_GATES:
                out["through_corridor"] = through_gate(xy, CORRIDOR_GATES[site.name])
            out["track"] = [[round(px, 3), round(py, 3)] for px, py in decimate(xy, TRACK_STEP_M)]
        else:
            out["max_cross_track_m"] = round(max(cross_track(step.poses, px, py)
                                                 for _, px, py, _ in truth), 3)
        if site.name in GAP_GATES:
            gate = GAP_GATES[site.name]
            corners = gap_corners(gate)
            out["through_gap"] = through_gate([p[1:3] for p in truth], gate, axis=1)
            out["gap_corner_min_m"] = round(min(min_distance([p[1:3] for p in truth], c)
                                                for c in corners), 3)
            if node.footprint:
                body = min(body_distance(p[1:4], node.footprint, c) for p in truth for c in corners)
                out["gap_corner_body_from"] = "footprint"
            else:
                body = out["gap_corner_min_m"] - BODY_HALF_WIDTH_M
                out["gap_corner_body_from"] = "centre-0.056"
            out["gap_corner_body_min_m"] = round(body, 3)
    if step.kind == NAV_GOAL:
        out["plans"] = len(plans)
        out["plan_keepout"] = sorted({z for plan in plans for z in keepout_zones(site, plan)})
    return out


def run(args) -> dict:
    site = load_site(args.layout)
    steps = build(site, args.plan, args.laps, points=dict(args.point))
    node = DriveCheck(args.robot, args.contract)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if not (node.move.wait_for_server(timeout_sec=30.0)
                and node.retreat.wait_for_server(timeout_sec=30.0)):
            raise RuntimeError("move_to or retreat server not found: is pode_fsm running?")
        if not wait_for(lambda: node.state is not None and node.state.state == "IDLE"
                        and node.last_truth() is not None, 90.0):
            raise RuntimeError("robot is not IDLE with a pose (AMCL, Gazebo truth)")
        _, x, y, _ = node.last_truth()
        begin = start_node(args.plan)
        if math.dist((x, y), site.node_pose(begin)) > START_M:
            raise RuntimeError(f"robot at ({x:.2f}, {y:.2f}) is not on {begin}")
        if args.wait_box > 0 and not wait_for(lambda: bool(node.boxes), args.wait_box):
            raise RuntimeError(f"no box on /sim/obstacle_markers after {args.wait_box:g} s"
                               " (sim_obstacles started after at_s, or a robot within 0.35 m"
                               " made it give the box up)")
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


def _named_point(text: str) -> tuple[str, tuple[float, float]]:
    try:
        name, xy = text.split("=")
        x, y = (float(v) for v in xy.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"--point must be NAME=X,Y, got {text!r}") from None
    return name.strip(), (x, y)


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", default="pode1")
    parser.add_argument("--layout", default=None,
                        help="배치. 비면 계획의 배치, 둘 다 비면 v7(실제 경기장)")
    parser.add_argument("--plan", default=None,
                        help="계획. 비면 배치의 첫 계획(v7은 corridor, 그 밖은 stage2)")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="이동 하나의 한도(벽시계 초)")
    parser.add_argument("--wait-box", type=float, default=0.0,
                        help="상자가 하나라도 놓일 때까지 이만큼(벽시계 초) 기다린 뒤 출발. "
                             "0이면 안 기다림")
    parser.add_argument("--point", type=_named_point, action="append", default=[],
                        metavar="NAME=X,Y",
                        help="계획 전용 점을 바꾼다(gap의 GAP_N, GAP_S). 여러 번")
    parser.add_argument("--contract", default=None)
    parser.add_argument("--json", default=None, help="요약을 이 파일에도 쓴다")
    args = parser.parse_args(argv)
    args.layout, args.plan = pick_layout_plan(args.layout, args.plan)
    return args


def main(argv=None) -> None:
    args = parse_args(argv)
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
