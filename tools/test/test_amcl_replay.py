"""AMCL 재생 비교 도구(tools/real_pode/amcl_replay)의 bag 처리."""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

TOOL = Path(__file__).resolve().parents[1] / "real_pode" / "amcl_replay" / "amcl_replay.py"


def _tool():
    spec = importlib.util.spec_from_file_location("amcl_replay", TOOL)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["amcl_replay"] = mod
    spec.loader.exec_module(mod)
    return mod


def _tf(parent, child, stamp, x=0.0, yaw=0.0):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec, t.header.stamp.nanosec = int(stamp), int((stamp % 1) * 1e9)
    t.transform.translation.x = x
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def _write(path, msgs):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    w.create_topic(rosbag2_py.TopicMetadata(0, "/pode2/tf", "tf2_msgs/msg/TFMessage", "cdr"))
    for stamp, transforms in msgs:
        w.write("/pode2/tf", serialize_message(TFMessage(transforms=transforms)), int(stamp * 1e9))
    del w


def _pairs(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = []
    while r.has_next():
        out += [(t.header.frame_id, t.child_frame_id)
                for t in deserialize_message(r.read_next()[1], TFMessage).transforms]
    return out


def test_prep_drops_only_the_live_map_to_odom(tmp_path):
    src = tmp_path / "src"
    _write(src, [(100.0, [_tf("map", "odom", 101.0), _tf("map", "marker", 100.0),
                          _tf("odom", "base_footprint", 100.0)])])
    tool = _tool()
    tool.prep(type("A", (), {"bag": str(src), "out": str(tmp_path / "out"), "ns": "pode2",
                             "map_frame": "map", "odom_frame": "odom",
                             "range_scale": 1.0, "range_offset": 0.0})())
    assert sorted(_pairs(tmp_path / "out")) == [("map", "marker"), ("odom", "base_footprint")]


def test_live_pose_uses_the_map_to_odom_stamped_at_or_before_the_time(tmp_path):
    # AMCL은 map→odom을 transform_tolerance(1 s) 앞당긴 시각으로 낸다. tf 조회처럼 그 시각 이하 값을 쓴다
    src = tmp_path / "src"
    _write(src, [(100.0, [_tf("map", "odom", 100.5, x=1.0)]),
                 (100.6, [_tf("map", "odom", 101.5, x=2.0)]),
                 (100.7, [_tf("odom", "base_footprint", 100.9, x=0.1)])])
    pose = _tool()._live(str(src), "pode2", [101.0])[101.0]
    assert pose[0] == pytest.approx(1.1)
