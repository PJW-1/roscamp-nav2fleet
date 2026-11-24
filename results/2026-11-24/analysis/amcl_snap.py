#!/usr/bin/env python3
"""amcl_pose를 지금 한 번, N초 뒤 한 번 받아 x, y, yaw와 표준편차를 jsonl에 적는다. 읽기만 한다."""
import argparse, json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

ap = argparse.ArgumentParser()
ap.add_argument("--ns", default="pode2"); ap.add_argument("--label", required=True)
ap.add_argument("--later", type=float, default=10.0); ap.add_argument("--out", required=True)
a = ap.parse_args()
rclpy.init(); node = Node("amcl_snap_probe"); last = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(PoseWithCovarianceStamped, f"/{a.ns}/amcl_pose", lambda m: last.update(m=m, t=time.time()), qos)

def grab(tag):
    t0 = time.time()
    last.clear()
    while rclpy.ok() and "m" not in last and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    m = last.get("m")
    if m is None:
        return {"tag": tag, "error": "no amcl_pose in 5 s"}
    p, q, c = m.pose.pose.position, m.pose.pose.orientation, m.pose.covariance
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    return {"tag": tag, "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, "x": round(p.x, 4), "y": round(p.y, 4),
            "yaw_deg": round(yaw, 2), "sx": round(math.sqrt(max(c[0], 0)), 4), "sy": round(math.sqrt(max(c[7], 0)), 4),
            "syaw_deg": round(math.degrees(math.sqrt(max(c[35], 0))), 2)}

now = grab("now")
end = time.time() + a.later
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
later = grab(f"+{a.later:g}s")
row = {"label": a.label, "wall": time.strftime("%H:%M:%S"), "now": now, "later": later}
print(json.dumps(row, ensure_ascii=False))
with open(a.out, "a", encoding="utf-8") as f:
    f.write(json.dumps(row, ensure_ascii=False) + "\n")
node.destroy_node(); rclpy.shutdown()
