import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import movers as mv  # noqa: E402

GZ_PY = ("/opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
         "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python")
sys.path[:0] = [p for p in GZ_PY if p not in sys.path]
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402

STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}


# ---- 쿼터니언과 CPU ----
def quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def find_pid(name: str, ns: str):
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            argv = Path(f"/proc/{entry}/cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].endswith(name.encode()) and any(ns.encode() in a for a in argv):
            return int(entry)
    return None


def cpu_seconds(pid):
    if pid is None:
        return None
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    except OSError:
        return None
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


# ---- 노드 ----
class Bench(Node):
    def __init__(self, cfg, scenario, out):
        super().__init__("nav_bench", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cfg, self.scenario, self.out = cfg, scenario, Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        ns = cfg["robot"]
        self.active = [cfg["movers"][n] for n in cfg["scenarios"][scenario]]
        self.travelled = {m.name: 0.0 for m in self.active}
        self.poses = {m.name: self._park(i) for i, m in enumerate(self.active)}
        self.moving = False
        self.pose_errors = 0
        self.robot = None
        self.rec = None
        self.last_t = None
        self.gz = GzNode()
        self.world = cfg["world"]
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.select_pub = self.create_publisher(String, f"/{ns}/controller_selector", latched)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, f"/{ns}/initialpose", 1)
        self.resets = 0
        self.create_subscription(Odometry, f"/{ns}/sim/truth", self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, f"/{ns}/cmd_vel", self._on_cmd, 50)
        self.create_subscription(LaserScan, f"/{ns}/scan", self._on_scan, qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, f"/{ns}/navigate_to_pose")
        self.create_timer(0.05, self._tick)
        self.ctl_pid = find_pid("controller_server", f"/{ns}")

    def _park(self, i):
        return (self.cfg["park"]["x"] + 0.5 * i, self.cfg["park"]["y"], 0.0)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spawn(self):
        for m in self.active:
            req = EntityFactory()
            req.sdf = mv.sdf(m)
            x, y, _ = self.poses[m.name]
            req.pose.position.x, req.pose.position.y = x, y
            ok, rep = self.gz.request(f"/world/{self.world}/create", req, EntityFactory,
                                      Boolean, 5000)
            self.get_logger().info(f"spawn {m.name}: {ok and rep.data}")
            if not (ok and rep.data):
                raise RuntimeError(f"Gazebo did not create mover {m.name}")

    def push_poses(self):
        if not self.active:
            return
        req = Pose_V()
        for m in self.active:
            x, y, yaw = self.poses[m.name]
            p = req.pose.add()
            p.name = f"bench_{m.name}"
            p.position.x, p.position.y = x, y
            p.orientation.z, p.orientation.w = quat(yaw)
        ok, rep = self.gz.request(f"/world/{self.world}/set_pose_vector", req, Pose_V, Boolean, 500)
        if not (ok and rep.data):
            self.pose_errors += 1

    def place(self, on: bool):
        for i, m in enumerate(self.active):
            self.travelled[m.name] = 0.0
            self.poses[m.name] = mv.pose_at(m, 0.0) if on else self._park(i)
        self.moving = on
        self.push_poses()

    def _tick(self):
        t = self.now()
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        if self.moving and dt > 0:
            for m in self.active:
                self.travelled[m.name] = mv.step(m, self.travelled[m.name], dt, self.robot,
                                                 self.poses[m.name])
                self.poses[m.name] = mv.pose_at(m, self.travelled[m.name])
            self.push_poses()
        if self.rec is not None and self.active:
            self.rec["movers"].append([t] + [list(self.poses[m.name]) for m in self.active])

    def _on_truth(self, msg):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y)
        if self.rec is not None:
            self.rec["truth"].append([self.now(), p.x, p.y, yaw_of(msg.pose.pose.orientation)])

    def _on_cmd(self, msg):
        if self.rec is not None:
            self.rec["cmd"].append([self.now(), msg.linear.x, msg.angular.z])

    def _on_scan(self, msg):
        if self.rec is not None:
            valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            self.rec["scan_min"].append([self.now(), min(valid) if valid else None])

    def wait(self, sim_s):
        end = self.now() + sim_s
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def select(self, name):
        self.select_pub.publish(String(data=name))
        self.wait(self.cfg["settle_s"])

    def go(self, pose, timeout_s):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose["x"], pose["y"]
        z, w = quat(math.radians(pose["yaw_deg"]))
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = z, w
        if not self.nav.wait_for_server(timeout_sec=30.0):
            return "NO_SERVER", self.now(), self.now()
        sent = self.now()
        future = self.nav.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = future.result()
        if not handle.accepted:
            return "REJECTED", sent, self.now()
        result = handle.get_result_async()
        while rclpy.ok() and not result.done() and self.now() - sent < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result.done():
            cancel = handle.cancel_goal_async()
            while rclpy.ok() and not cancel.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            self.wait(1.0)
            return "TIMEOUT", sent, self.now()
        return STATUS.get(result.result().status, str(result.result().status)), sent, self.now()

    def leg(self, controller, repeat):
        cfg = self.cfg
        self.place(False)
        self.select(controller)
        self.place(True)
        self.wait(0.5)
        self.pose_errors = 0
        self.rec = {"truth": [], "cmd": [], "scan_min": [], "movers": []}
        cpu0, wall0 = cpu_seconds(self.ctl_pid), time.monotonic()
        status, t0, t1 = self.go(cfg["goal"], cfg["leg_timeout_s"])
        retried = False
        if status == "ABORTED" and t1 - t0 < cfg["flake_s"] and not self._moved(self.rec):
            retried = True
            self.rec = {"truth": [], "cmd": [], "scan_min": [], "movers": []}
            self.place(True)
            self.wait(0.5)
            cpu0, wall0 = cpu_seconds(self.ctl_pid), time.monotonic()
            status, t0, t1 = self.go(cfg["goal"], cfg["leg_timeout_s"])
        cpu1, wall1 = cpu_seconds(self.ctl_pid), time.monotonic()
        rec, self.rec = self.rec, None
        self.place(False)
        static = [m.name for m in self.active if m.speed == 0.0]
        if static:
            self.place_static()
        rec.update({"scenario": self.scenario, "controller": controller, "repeat": repeat,
                    "status": status, "t_start": t0, "t_end": t1, "retried": retried,
                    "pose_errors": self.pose_errors,
                    "movers_names": [m.name for m in self.active],
                    "cpu_s": None if cpu0 is None or cpu1 is None else cpu1 - cpu0,
                    "wall_s": wall1 - wall0})
        name = f"{self.scenario}_{controller}_{repeat}.json"
        (self.out / name).write_text(json.dumps(rec), encoding="utf-8")
        self.get_logger().info(f"leg {name}: {status} {t1 - t0:.1f} s")
        return status

    def _moved(self, rec):
        pts = [p[1:3] for p in rec["truth"]]
        return bool(pts) and max(math.dist(pts[0], p) for p in pts) > 0.02

    def place_static(self):
        for m in self.active:
            if m.speed == 0.0:
                self.poses[m.name] = mv.pose_at(m, 0.0)
        self.push_poses()

    def back(self):
        self.select(self.cfg["controllers"][0])
        for _ in range(3):
            status, _, _ = self.go(self.cfg["start"], self.cfg["leg_timeout_s"])
            if status == "SUCCEEDED":
                return True
        return self.teleport_to_start()

    def teleport_to_start(self):
        st = self.cfg["start"]
        z, w = quat(math.radians(st["yaw_deg"]))
        req = Pose_V()
        p = req.pose.add()
        p.name = self.cfg["robot"]
        p.position.x, p.position.y, p.position.z = st["x"], st["y"], 0.01
        p.orientation.z, p.orientation.w = z, w
        ok, rep = self.gz.request(f"/world/{self.world}/set_pose_vector", req, Pose_V, Boolean,
                                  2000)
        if not (ok and rep.data):
            return False
        init = PoseWithCovarianceStamped()
        init.header.frame_id = "map"
        init.pose.pose.position.x, init.pose.pose.position.y = st["x"], st["y"]
        init.pose.pose.orientation.z, init.pose.pose.orientation.w = z, w
        init.pose.covariance[0] = init.pose.covariance[7] = 0.0025
        init.pose.covariance[35] = 0.01
        for _ in range(3):
            init.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(init)
            self.wait(1.0)
        self.resets += 1
        self.get_logger().warn(f"teleported to the start (reset {self.resets})")
        if self.robot is None:
            return False
        return math.hypot(self.robot[0] - st["x"], self.robot[1] - st["y"]) <= self.cfg["arrive_m"]


# ---- 실행 ----
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--repeats", type=int)
    args = ap.parse_args(argv)
    cfg = mv.load(args.config)
    repeats = args.repeats or cfg["repeats"]
    rclpy.init()
    node = Bench(cfg, args.scenario, args.out)
    node.wait(1.0)
    node.spawn()
    node.place(False)
    done = 0
    try:
        for r in range(repeats):
            order = cfg["controllers"] if r % 2 == 0 else list(reversed(cfg["controllers"]))
            for ctl in order:
                node.leg(ctl, r)
                if not node.back():
                    node.get_logger().error("could not return or reset to the start, stop")
                    raise SystemExit(2)
            done += 1
    finally:
        node.select(cfg["controllers"][0])
        print(f"bench {args.scenario}: {done}/{repeats} repeats, {node.resets} resets", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
