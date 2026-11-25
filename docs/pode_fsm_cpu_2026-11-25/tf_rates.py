"""bag의 /pode2/tf를 child frame별로 세고, 메시지당 변환 수를 낸다."""
import collections
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

bag = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/pode2/tf"]))
cnt = collections.Counter()
per_msg = collections.Counter()
t0 = t1 = None
while r.has_next():
    _, data, t = r.read_next()
    t0 = t0 or t
    t1 = t
    m = deserialize_message(data, TFMessage)
    per_msg[len(m.transforms)] += 1
    for tr in m.transforms:
        cnt[(tr.header.frame_id, tr.child_frame_id)] += 1
dur = (t1 - t0) * 1e-9
print(f"duration {dur:.1f} s")
for k, v in cnt.most_common():
    print(k, v, f"{v / dur:.1f} Hz")
print("transforms per msg", dict(per_msg))
