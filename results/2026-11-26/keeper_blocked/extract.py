#!/usr/bin/env python3
"""bag 두 개를 읽어 분석용 pickle로 저장(scan은 사건 구간만).

사용(source 없이):
  PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$WS/install/fleet_interfaces/lib/python3.12/site-packages \
  LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$WS/install/fleet_interfaces/lib python3 extract.py 출력.pkl
"""
import math
import pickle
import sys
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DATA = "/home/ane/ros2camp/real_data/2026-11-26/bags"
BAGS = {"pode2": f"{DATA}/1126_real", "pode1": f"{DATA}/1126_real_pode1"}
# 사건 구간(epoch s): scan을 이 안에서만 모은다
WINDOWS = [(1795688160, 1795688190), (1795691330, 1795691360), (1795691400, 1795691430)]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = defaultdict(list)
    while r.has_next():
        topic, raw, t_rec = r.read_next()
        name = topic.rsplit("/", 1)[-1]
        t = t_rec * 1e-9
        if name == "particle_cloud":
            continue
        if name == "scan" and not any(a <= t <= b for a, b in WINDOWS):
            continue
        m = deserialize_message(raw, get_message(types[topic]))
        if name in ("tf", "tf_static"):
            for tr in m.transforms:
                q, p = tr.transform.rotation, tr.transform.translation
                out[name].append((stamp(tr.header), tr.header.frame_id, tr.child_frame_id,
                                  p.x, p.y, yaw_of(q)))
        elif name == "odom":
            p = m.pose.pose
            out["odom"].append((stamp(m.header), p.position.x, p.position.y,
                                yaw_of(p.orientation), m.twist.twist.linear.x,
                                m.twist.twist.angular.z))
        elif name == "cmd_vel":
            out["cmd_vel"].append((t, m.linear.x, m.angular.z))
        elif name == "plan":
            out["plan"].append((t, stamp(m.header), m.header.frame_id,
                                [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]))
        elif name == "state":
            out["state"].append((t, m.state, m.local_yield, m.current_goal, m.pose.x, m.pose.y,
                                 m.pose.theta, m.obstacle_x, m.obstacle_y, m.obstacle_kind))
        elif name == "amcl_pose":
            p = m.pose.pose
            out["amcl"].append((stamp(m.header), p.position.x, p.position.y,
                                yaw_of(p.orientation)))
        elif name == "scan":
            out["scan"].append((stamp(m.header), t, m.angle_min, m.angle_increment,
                                list(m.ranges)))
    return dict(out)


def main():
    res = {r: read(p) for r, p in BAGS.items()}
    for r, d in res.items():
        print(r, {k: len(v) for k, v in d.items()})
    with open(sys.argv[1], "wb") as f:
        pickle.dump(res, f)


if __name__ == "__main__":
    main()
