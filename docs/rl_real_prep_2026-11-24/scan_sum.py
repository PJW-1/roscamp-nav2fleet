import math, sys, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'),
       rosbag2_py.ConverterOptions('', ''))
beams, amin, amax, inc, rmin, rmax, frames, stime = set(), set(), set(), set(), set(), set(), set(), set()
stamps, zeros, infs, nans, small, total, finite_max = [], 0, 0, 0, 0, 0, 0.0
below = {0.10: 0, 0.15: 0, 0.20: 0}
first = None
statics = []
cmd_rx = []
while r.has_next():
    topic, data, t = r.read_next()
    if topic.endswith('/scan'):
        m = deserialize_message(data, LaserScan)
        beams.add(len(m.ranges)); amin.add(round(m.angle_min, 5)); amax.add(round(m.angle_max, 5))
        inc.add(round(m.angle_increment, 7)); rmin.add(m.range_min); rmax.add(m.range_max)
        frames.add(m.header.frame_id); stime.add(round(m.scan_time, 4))
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if first is None:
            first = m
        for x in m.ranges:
            total += 1
            if math.isnan(x): nans += 1
            elif math.isinf(x): infs += 1
            elif x == 0.0: zeros += 1
            else:
                finite_max = max(finite_max, x)
                if x < m.range_min: small += 1
                for k in below:
                    if x < k: below[k] += 1
    elif topic.endswith('tf_static'):
        m = deserialize_message(data, TFMessage)
        for tr in m.transforms:
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            statics.append((tr.header.frame_id, tr.child_frame_id, round(tr.transform.translation.x, 3),
                            round(tr.transform.translation.y, 3), round(yaw, 3)))
    elif topic.endswith('/cmd_vel'):
        cmd_rx.append(t * 1e-9)
d = [b - a for a, b in zip(stamps, stamps[1:])]
print('beams', sorted(beams)[:5], len(beams), 'angle_min', sorted(amin)[:3], 'angle_max', sorted(amax)[:3])
print('inc_deg', sorted(math.degrees(i) for i in inc)[:5], 'range_min', rmin, 'range_max', rmax)
print('frames', frames, 'scan_time', sorted(stime)[:5])
print('n', len(stamps), 'rate_hz', (len(stamps) - 1) / (stamps[-1] - stamps[0]),
      'dt median', st.median(d), 'p95', sorted(d)[int(.95 * len(d))], 'max', max(d))
print('total', total, 'zeros', zeros, 'infs', infs, 'nans', nans, 'below range_min(nonzero)', small,
      'below', below, 'finite max', finite_max)
print('first scan beams', len(first.ranges), 'first 5', list(first.ranges[:5]))
print('static tf', statics)
cd = [b - a for a, b in zip(cmd_rx, cmd_rx[1:])]
print('cmd_vel n', len(cmd_rx), 'median gap', st.median(cd))
