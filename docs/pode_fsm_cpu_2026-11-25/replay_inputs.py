#!/usr/bin/env python3
"""실물 bag의 odom·tf 구간을 지금 시각으로 다시 찍어 되풀이 발행하고, 가짜 Nav2 서버와 local costmap을 띄운다.

pode_fsm이 실물에서 받던 입력을 PC에서 재현하려는 것이다. 발행 쪽 CPU는 재지 않는다.
사용: replay_inputs.py <bag> --ns pode2 --start-s 60 --len-s 90
"""
import argparse
import threading
import time

import rclpy
import rosbag2_py
from nav2_msgs.action import BackUp, FollowPath, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def load(bag, ns, start_s, len_s):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    topics = {f"/{ns}/odom": Odometry, f"/{ns}/tf": TFMessage, f"/{ns}/tf_static": TFMessage}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    msgs, static, t0 = [], None, None
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, topics[topic])
        if topic.endswith("tf_static"):
            static = m
            continue
        t0 = t0 or t
        rel = (t - t0) * 1e-9 - start_s
        if rel < 0:
            continue
        if rel > len_s:
            break
        msgs.append((rel, topic, m))
    return msgs, static


class Inputs(Node):
    def __init__(self, ns, msgs, static, costmap_hz):
        super().__init__("replay_inputs", namespace=ns)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom = self.create_publisher(Odometry, "odom", qos_profile_sensor_data)
        self.tf = self.create_publisher(TFMessage, "tf", 100)
        self.tf_static = self.create_publisher(TFMessage, "tf_static", latched)
        self.costmap = self.create_publisher(
            OccupancyGrid, "local_costmap/costmap",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.msgs, self.static = msgs, static
        # 가짜 Nav2: 서버만 있고 goal은 오지 않는다(실물 측정 때 pode_fsm에는 goal이 없었다)
        for kind, name in ((FollowPath, "follow_path"), (BackUp, "backup"),
                           (NavigateToPose, "navigate_to_pose")):
            ActionServer(self, kind, name, lambda h, k=kind: k.Result())
        self.create_service(ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap",
                            lambda _q, s: s)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "odom"
        self.grid.info.resolution = 0.025
        self.grid.info.width = self.grid.info.height = 120
        self.grid.data = [0] * (120 * 120)
        self.create_timer(1.0 / costmap_hz, self._pub_grid)

    def _pub_grid(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.costmap.publish(self.grid)

    def run(self, alive):
        now = self.get_clock().now().to_msg()
        for t in self.static.transforms:
            t.header.stamp = now
        self.tf_static.publish(self.static)
        while alive():
            start = time.monotonic()
            for rel, topic, m in self.msgs:
                delay = start + rel - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                if not alive():
                    return
                stamp = self.get_clock().now().to_msg()
                if topic.endswith("odom"):
                    m.header.stamp = stamp
                    self.odom.publish(m)
                else:
                    for t in m.transforms:
                        t.header.stamp = stamp
                    self.tf.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--ns", default="pode2")
    ap.add_argument("--start-s", type=float, default=60.0)
    ap.add_argument("--len-s", type=float, default=90.0)
    ap.add_argument("--costmap-hz", type=float, default=2.0)
    args = ap.parse_args()
    msgs, static = load(args.bag, args.ns, args.start_s, args.len_s)
    print(f"loaded {len(msgs)} messages over {args.len_s} s", flush=True)
    rclpy.init()
    node = Inputs(args.ns, msgs, static, args.costmap_hz)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        node.run(rclpy.ok)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
