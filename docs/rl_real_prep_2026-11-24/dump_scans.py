import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'),
       rosbag2_py.ConverterOptions('', ''))
rows = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic.endswith('/scan'):
        m = deserialize_message(data, LaserScan)
        rows.append(np.array(m.ranges, dtype=np.float32))
        meta = np.array([m.angle_min, m.angle_increment, m.range_min, m.range_max])
np.savez(sys.argv[2], ranges=np.stack(rows), meta=meta)
print(len(rows), meta)
