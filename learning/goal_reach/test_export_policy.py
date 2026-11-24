"""export_policy.py 의 policy_spec 내용 테스트. 실행: python -m pytest learning/goal_reach"""

import dataclasses
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
pytest.importorskip("onnxruntime")
import export_policy as ep  # noqa: E402
import task_spec as ts  # noqa: E402

GOAL = {"rule": "path_end"}


def test_default_spec_is_format_2():
    out = ep.make_spec({}, GOAL)
    assert out["format"] == 2 and "obs_blocks" not in out and out["obs_dim"] == 41


def test_blocks_give_format_3_with_their_entries():
    spec = dataclasses.replace(
        ts.SPEC, prev_action_obs=1, lidar_frames=3, speed_cap_obs=1, speed_cap_range=(0.08, 0.16)
    )
    out = ep.make_spec({}, GOAL, spec)
    assert out["format"] == 3
    assert out["obs_blocks"] == ["prev_action", "lidar_history", "speed_cap"]
    assert out["prev_action"] == {"v_scale": 0.12, "w_scale": 1.0}
    assert out["lidar_history"] == {"frames": 3, "stride_steps": 4}
    assert out["speed_cap"] == {"min": 0.08, "max": 0.16, "obs_scale": 0.12}
    assert len(out["obs_layout"]) == out["obs_dim"] == 41 + 2 + 72 + 1


def test_a_cap_other_than_the_top_speed_without_its_block_is_refused():
    spec = dataclasses.replace(ts.SPEC, speed_cap_range=(0.08, 0.08))
    with pytest.raises(ValueError):
        ep.make_spec({}, GOAL, spec)
