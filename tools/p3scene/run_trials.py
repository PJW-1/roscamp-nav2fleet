"""P-3 끼임 장면을 Gazebo 한 판에서 여러 번 되풀이한다(gz.launch.py가 떠 있어야 한다).

판마다: pode3을 P-3 안의 정한 자세로, pode2를 E-2(북쪽 보기)로 옮기고(gz set_pose), 두 대의 AMCL에
initialpose를 주고, pode2 costmap을 비운 뒤 pode2에 Nav2 goal Q2를 보낸다(기본 BT = goal BT).
결과(실제 위치 궤적, 첫 계획, 상태, 복구 수, 벽시계 구간)를 판마다 json 한 줄로 쓴다.

    python3 run_trials.py --config C0 --out runs/C0.jsonl [--timeout 240] [--sets A,B]
"""
import argparse
import json
import math
import os
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from fleet_interfaces.msg import RobotState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---- 장면 ----
E2 = (0.38, 0.24, math.radians(90))
Q2 = (1.66, 0.16, 0.0)
# 기록된 판의 pode3 실제 자세 범위(x 0.62~0.651, y 0.10~0.14, yaw 90~113도) 안에서 고른 다섯
POSES = [(0.620, 0.100, 90.0), (0.651, 0.140, 113.0), (0.635, 0.120, 100.0),
         (0.651, 0.100, 105.0), (0.620, 0.140, 95.0)]
# pode1 자리: A = P-1(비켜 둠), B = 가운데 통로 서쪽(북쪽 길을 막아 NavFn이 틈으로 가게)
PODE1 = {"A": (0.12, 0.10, math.radians(90)), "B": (0.62, 0.56, 0.0)}
WORLD = "arena"
BEST = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def teleport(name, x, y, yaw):
    """gz set_pose. launch와 같은 칸막이(GZ_PARTITION)를 쓴다."""
    z, w = quat(yaw)
    env = dict(os.environ, GZ_IP="127.0.0.1",
               GZ_PARTITION=os.environ.get("GZ_PARTITION",
                                           f"fleet_gz_{os.environ['ROS_DOMAIN_ID']}"))
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.01}}, '
           f'orientation: {{x: 0, y: 0, z: {z}, w: {w}}}')
    out = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose",
                          "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                          "--timeout", "3000", "--req", req],
                         env=env, capture_output=True, text=True, timeout=10)
    return "true" in out.stdout


class Trials(Node):
    def __init__(self):
        super().__init__("p3scene_trials", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.truth = {"pode2": [], "pode3": []}
        self.last = {}
        self.amcl = {}
        self.state = {}
        self.plans = []
        self.recording = False
        for r in ("pode1", "pode2", "pode3"):
            self.create_subscription(Odometry, f"/{r}/sim/truth",
                                     lambda m, r=r: self.on_truth(r, m), BEST)
            self.create_subscription(PoseWithCovarianceStamped, f"/{r}/amcl_pose",
                                     lambda m, r=r: self.amcl.__setitem__(r, m.pose.pose), LATCHED)
            self.create_subscription(RobotState, f"/{r}/state",
                                     lambda m, r=r: self.state.__setitem__(r, m), BEST)
        self.create_subscription(Path, "/pode2/plan", self.on_plan, 5)
        # 계획 실패 때 출발 칸의 global 값을 보려고 마지막 global costmap(1 Hz, 통째)을 둔다
        self.global_map = None
        self.create_subscription(OccupancyGrid, "/pode2/global_costmap/costmap",
                                 lambda m: setattr(self, "global_map", m), LATCHED)
        self.init_pub = {r: self.create_publisher(PoseWithCovarianceStamped, f"/{r}/initialpose", 5)
                         for r in ("pode1", "pode2", "pode3")}
        self.clear = [self.create_client(ClearEntireCostmap, f"/pode2/{c}/clear_entirely_{c}")
                      for c in ("local_costmap", "global_costmap")]
        self.nav = ActionClient(self, NavigateToPose, "/pode2/navigate_to_pose")

    def on_truth(self, r, m):
        p = m.pose.pose
        self.last[r] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.recording and r in self.truth:
            self.truth[r].append((round(self.now(), 3), round(p.position.x, 4),
                                  round(p.position.y, 4), round(yaw_of(p.orientation), 4)))

    def on_plan(self, m):
        if self.recording and len(self.plans) < 40:  # 1 Hz 재계획: 앞의 40개만
            self.plans.append([self.now(), [(round(s.pose.position.x, 3), round(s.pose.position.y, 3))
                                            for s in m.poses[::3]]])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sim_s, wall_cap=60.0):
        t0, w0 = self.now(), time.monotonic()
        while self.now() - t0 < sim_s and time.monotonic() - w0 < wall_cap:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_until(self, cond, wall_s):
        w0 = time.monotonic()
        while time.monotonic() - w0 < wall_s:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def initialpose(self, r, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = quat(yaw)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.0004
        m.pose.covariance[35] = 0.003
        self.init_pub[r].publish(m)

    def near(self, pose, x, y, yaw, d=0.03, a=0.1):
        if pose is None:
            return False
        dyaw = abs(math.atan2(math.sin(pose[2] - yaw), math.cos(pose[2] - yaw)))
        return math.hypot(pose[0] - x, pose[1] - y) < d and dyaw < a

    def idle(self, r):
        s = self.state.get(r)
        return s is not None and s.state == RobotState.STATE_IDLE

    def amcl_pose(self, r):
        p = self.amcl.get(r)
        return None if p is None else (p.position.x, p.position.y, yaw_of(p.orientation))

    def state_pose(self, r):
        s = self.state.get(r)
        return None if s is None else (s.pose.x, s.pose.y, s.pose.theta)

    def place(self, targets):
        """옮기고 AMCL을 맞춘다. 맞으면 True. 시도마다 다시 옮긴다(다른 노드가 몰았을 수 있다)."""
        for attempt in range(4):
            for r, (x, y, yaw) in targets.items():
                if not teleport(r, x, y, yaw):
                    self.get_logger().error(f"set_pose {r} failed")
            self.spin_for(1.0)
            for r in targets:
                self.amcl.pop(r, None)  # 옮긴 뒤의 새 값만 본다
                self.state.pop(r, None)
            for r, (x, y, yaw) in targets.items():
                self.initialpose(r, x, y, yaw)
            ok = self.spin_until(lambda: all(
                self.idle(r) and self.near(self.last.get(r), *t, d=0.01, a=0.03)
                and self.near(self.amcl_pose(r), *t) and self.near(self.state_pose(r), *t)
                for r, t in targets.items()), 15.0)
            if ok:
                return True
            self.get_logger().warn(f"localize retry {attempt}: truth {self.last} "
                                   f"amcl {[self.amcl_pose(r) for r in targets]}")
        return False

    def cost_probe(self, pose):
        """pose(AMCL, 계획의 출발점) 칸과 둘레 5x5의 global 값(OccupancyGrid: 100 치명, 99 내접)."""
        m = self.global_map
        if m is None or pose is None:
            return None
        info = m.info
        i = int((pose[0] - info.origin.position.x) / info.resolution)
        j = int((pose[1] - info.origin.position.y) / info.resolution)

        def at(a, b):
            ok = 0 <= a < info.width and 0 <= b < info.height
            return m.data[b * info.width + a] if ok else None
        return {"pose": pose, "cell": [i, j], "centre": at(i, j),
                "patch": [[at(i + di, j + dj) for di in range(-2, 3)] for dj in range(2, -3, -1)],
                "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, "now": self.now()}

    def clear_costmaps(self):
        """비운 costmap 수(둘이어야 한다)."""
        done = 0
        for c in self.clear:
            if c.wait_for_service(timeout_sec=5.0):
                f = c.call_async(ClearEntireCostmap.Request())
                done += self.spin_until(f.done, 5.0) and f.exception() is None
        return done

    def trial(self, k, pose3, timeout_s, pode1="A"):
        x3, y3, yaw3 = pose3[0], pose3[1], math.radians(pose3[2])
        rec = {"trial": k, "pode3_set": pose3, "pode1_set": pode1, "ok": False}
        if not self.place({"pode1": PODE1[pode1], "pode3": (x3, y3, yaw3), "pode2": E2}):
            rec["error"] = "place failed"
            return rec
        rec["cleared"] = self.clear_costmaps()
        if rec["cleared"] != 2:
            rec["error"] = "costmap clear failed"
            return rec
        self.spin_for(2.0)  # peer 층(5 Hz)과 라이다가 다시 찍을 시간
        rec["pode3_truth"] = self.last.get("pode3")
        rec["pode3_state"] = self.state_pose("pode3")
        rec["pode2_amcl"] = self.amcl_pose("pode2")
        rec["pode1_truth"] = self.last.get("pode1")
        self.truth = {"pode2": [], "pode3": []}
        self.plans = []
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = Q2[0], Q2[1]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat(Q2[2])
        recov = [0]

        def feedback(fb):
            recov[0] = max(recov[0], fb.feedback.number_of_recoveries)

        self.recording = True
        rec["wall_t0"], rec["sim_t0"] = time.time(), self.now()
        f = self.nav.send_goal_async(goal, feedback_callback=feedback)
        if not self.spin_until(f.done, 30.0) or not f.result().accepted:
            self.recording = False
            rec["error"] = "goal rejected" if f.done() else "goal not answered"
            rec["stop_run"] = not f.done()  # 나중에 수락되면 다음 판과 겹친다
            return rec
        handle = f.result()
        rf = handle.get_result_async()
        t0, w0 = rec["sim_t0"], time.monotonic()
        while (not rf.done() and self.now() - t0 < timeout_s
               and time.monotonic() - w0 < 3 * timeout_s):  # 시계가 멈춰도 끝난다
            rclpy.spin_once(self, timeout_sec=0.05)
        if rf.done():
            status = rf.result().status
            rec["status"] = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                             GoalStatus.STATUS_ABORTED: "aborted",
                             GoalStatus.STATUS_CANCELED: "canceled"}.get(status, str(status))
            rec["error_code"] = rf.result().result.error_code
        else:
            rec["status"] = "timeout"
            cf = handle.cancel_goal_async()
            self.spin_until(cf.done, 5.0)
            rec["cancel_done"] = self.spin_until(rf.done, 15.0)  # 끝난 뒤에 다음 판
            rec["stop_run"] = not rec["cancel_done"]
        rec["wall_t1"], rec["sim_t1"] = time.time(), self.now()
        self.spin_for(0.5)
        self.recording = False
        rec["duration_s"] = round(rec["sim_t1"] - rec["sim_t0"], 2)
        rec["recoveries"] = recov[0]
        rec["pode2_end"] = self.last.get("pode2")
        rec["global_at_end"] = self.cost_probe(self.amcl_pose("pode2"))
        rec["truth"], rec["plans"] = self.truth, self.plans
        rec["ok"] = True
        return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--sets", default="A,B", help="pode1 자리 차례(A: P-1, B: 북쪽 길)")
    a = ap.parse_args()
    rclpy.init()
    node = Trials()
    try:
        ready = node.spin_until(lambda: node.nav.server_is_ready() and "pode3" in node.state
                                and "pode2" in node.last and "pode3" in node.last, 300.0)
        if not ready:
            print("not ready: nav server or topics missing", flush=True)
            return 2
        # 관제는 뜨자마자 로봇 자리를 확인하고 제자리가 아니면 돌려보낸다(COMM_RESTORED): 그 뒤에 시작
        fleet = node.spin_until(lambda: "fleet_manager" in node.get_node_names(), 300.0)
        if not fleet:
            print("not ready: fleet_manager missing", flush=True)
            return 2
        node.spin_for(15.0)
        k = 0
        for pode1 in a.sets.split(","):
            for pose3 in POSES:
                rec = node.trial(k, pose3, a.timeout, pode1)
                rec["config"] = a.config
                with open(a.out, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec) + "\n")
                print(f"{a.config} trial {k} set {pode1} pode3 {pose3}: {rec.get('status')} "
                      f"{rec.get('duration_s')} s rec {rec.get('recoveries')} {rec.get('error', '')}",
                      flush=True)
                k += 1
                if rec.get("stop_run"):  # 이전 goal이 살아 있을 수 있다: 더 돌지 않는다
                    print("stop: previous goal may still be running", flush=True)
                    return 3
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
