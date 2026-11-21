"""Tests for the simulator-free task rules. Run: <venv python> -m pytest learning/goal_reach -q"""

import os
import sys

import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import task_spec as ts  # noqa: E402


def test_outcome_is_exclusive_and_failure_wins():
    # env 0: reached; 1: reached while colliding; 2: reached while out of bounds; 3: collides only
    dist = torch.tensor([0.01, 0.01, 0.01, 1.0])
    force = torch.tensor([0.0, 5.0, 0.0, 5.0])
    xy = torch.tensor([[1.0, 0.0], [1.0, 0.0], [3.5, 0.0], [0.5, 0.0]])
    succ, coll, oob = ts.episode_outcome(dist, force, xy, torch.zeros(4, 2))
    assert succ.tolist() == [True, False, False, False]
    assert coll.tolist() == [False, True, False, True]
    assert oob.tolist() == [False, False, True, False]
    assert ((succ.int() + coll.int() + oob.int()) <= 1).all()


def test_oob_with_collision_counts_as_collision_only():
    succ, coll, oob = ts.episode_outcome(
        torch.tensor([1.0]), torch.tensor([5.0]), torch.tensor([[3.5, 0.0]]), torch.zeros(1, 2)
    )
    assert (succ.item(), coll.item(), oob.item()) == (False, True, False)


def test_reward_no_arrival_bonus_on_collision():
    succ, coll, oob = ts.episode_outcome(
        torch.tensor([0.01]), torch.tensor([5.0]), torch.tensor([[1.0, 0.0]]), torch.zeros(1, 2)
    )
    _, terms = ts.compute_reward(torch.tensor([0.02]), torch.tensor([0.01]), succ, coll | oob)
    assert terms["arrival"].item() == 0.0
    assert terms["collision"].item() == ts.SPEC.w_collision


def test_spec_overrides_reject_env_shaping_fields():
    assert ts.make_spec({"w_time": -0.2}).w_time == -0.2
    for k in ("control_dt", "episode_s", "lidar_sectors", "lidar_obs_max"):
        with pytest.raises(ValueError):
            ts.make_spec({k: 1.0})


def test_rate_limit_and_wheels():
    cmd = ts.rate_limit(torch.zeros(1, 2), torch.tensor([[0.25, 1.0]]), 0.05)
    assert torch.allclose(cmd, torch.tensor([[0.1, 0.1]]))
    w = ts.twist_to_wheels(torch.tensor([[0.0, 1.0]]))
    assert torch.allclose(w, torch.tensor([[-0.0961 / 2 / 0.028, 0.0961 / 2 / 0.028]]))


def test_lidar_sectors_index_and_inf():
    r = torch.full((1, 360), float("inf"))
    r[0, 185] = 0.3  # ray 185 = +5 deg, sector 18 covers rays 180..189
    s = ts.lidar_sectors(r)
    assert s[0, 18].item() == pytest.approx(0.3)
    assert (s[0, :18] == ts.SPEC.lidar_obs_max).all()


def test_a_tuple_field_is_set_with_comma_separated_numbers():
    got = ts.parse_overrides(["goal_dist_range=0.5,1.0", "w_time=-0.2"])
    assert got == {"goal_dist_range": (0.5, 1.0), "w_time": -0.2}
    assert ts.make_spec(got).goal_dist_range == (0.5, 1.0)


def test_a_tuple_field_with_the_wrong_count_is_refused():
    with pytest.raises(ValueError, match="goal_dist_range"):
        ts.parse_overrides(["goal_dist_range=1.5"])


def test_an_outcome_on_the_last_step_is_not_also_a_timeout():
    # env 0: success on the last step; 1: still driving at the last step; 2: success earlier
    done = torch.tensor([True, False, True])
    length = torch.tensor([400, 400, 120])
    terminated, time_out = ts.split_dones(done, length, 400)
    assert terminated.tolist() == [True, False, True]
    assert time_out.tolist() == [False, True, False]
