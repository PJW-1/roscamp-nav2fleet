"""Tests for ceiling.py. Run: python -m pytest learning/store_arena"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ceiling as C  # noqa: E402

KEYS = ["success", "collision", "oob", "timeout"]


def test_ceiling_counts_failures_that_also_collide_standing_still():
    env = np.arange(6)
    policy = np.array([0, 1, 1, 3, 0, 0])  # 3 failures
    still = np.array([3, 1, 3, 1, 1, 3])  # still hit in envs 1, 3, 4
    out = C.ceiling(env, policy, env, still, KEYS)
    assert out["still_collision_rate"] == round(3 / 6, 4)
    assert out["failures_also_hit_still"] == 2  # envs 1 and 3
    assert out["share_of_failures_hit_still"] == round(2 / 3, 4)
    assert out["policy_success_on_still_hit"] == round(1 / 3, 4)  # env 4
    assert out["success_ceiling"] == 0.5


def test_runs_with_different_envs_are_refused():
    with pytest.raises(ValueError):
        C.ceiling(np.arange(3), np.zeros(3), np.array([0, 1, 1]), np.zeros(3), KEYS)
