import math, sys
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
        amin, inc = m.angle_min, m.angle_increment
a = np.stack(rows)
n = a.shape[1]
lidar_deg = np.degrees(amin + inc * np.arange(n))
base_deg = (lidar_deg + 180.0 + 180.0) % 360.0 - 180.0  # rplidar_link yaw pi in base
short = np.where(np.isfinite(a), a, np.inf) < 0.15
frac = short.mean(axis=0)
med = np.nanmedian(np.where(np.isfinite(a), a, np.nan), axis=0)
mn = np.nanmin(np.where(np.isfinite(a), a, np.nan), axis=0)
infr = (~np.isfinite(a)).mean(axis=0)
order = np.argsort(base_deg)
print('scans', a.shape)
print('beams always (>=95%) under 0.15 m:', int((frac >= 0.95).sum()), ' >=50%:', int((frac >= 0.5).sum()))
# group in 10-degree bins of robot frame
for lo in range(-180, 180, 10):
    sel = (base_deg >= lo) & (base_deg < lo + 10)
    print(f'{lo:5d}..{lo+10:4d} short<0.15 frac {frac[sel].mean():.2f}  median {np.nanmedian(med[sel]):.3f}  min {np.nanmin(mn[sel]):.3f}  inf frac {infr[sel].mean():.2f}')
idx = np.where(frac >= 0.9)[0]
print('beams >=90% short:', [(int(i), round(float(base_deg[i]), 1), round(float(frac[i]), 2), round(float(med[i]), 3)) for i in idx])
idx = np.where(infr >= 0.6)[0]
print('beams >=60% inf:', [(int(i), round(float(base_deg[i]), 1), round(float(infr[i]), 2)) for i in idx])
# sim-like training sectors: 36 sectors of 10 rays from -180 deg in base frame; sector min of nearest beams
