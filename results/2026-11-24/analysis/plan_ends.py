#!/usr/bin/env python3
"""bag의 plan 토픽에서 구간마다 경로 끝점이 바뀔 때만 시각, 끝점, 길이를 낸다. 읽기만 한다."""
import datetime, math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path

bag, t_from, t_to = sys.argv[1], sys.argv[2], sys.argv[3]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
last = None
while r.has_next():
    topic, data, t = r.read_next()
    if not topic.endswith("/plan"):
        continue
    s = datetime.datetime.fromtimestamp(t / 1e9).strftime("%H:%M:%S.%f")[:-4]
    if not (t_from <= s[:8] <= t_to):
        continue
    m = deserialize_message(data, Path)
    if not m.poses:
        print(s, "empty plan"); continue
    e = m.poses[-1].pose.position
    end = (round(e.x, 3), round(e.y, 3))
    if end != last:
        print(s, "end", end, "poses", len(m.poses), "frame", m.header.frame_id)
        last = end
print("last end", last)
