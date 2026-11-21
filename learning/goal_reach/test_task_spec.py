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


def _obs_inputs(n=2):
    return torch.full((n, 360), 1.0), torch.ones(n), torch.zeros(n), torch.zeros(n, 2)


def test_the_default_observation_is_the_41_dim_layout():
    assert ts.OBS_DIM == ts.obs_dim(ts.SPEC) == 41 == len(ts.obs_names())
    obs = ts.build_observation(*_obs_inputs())
    assert obs.shape == (2, 41)


def test_intent_observation_layout_mask_and_clip():
    spec = ts.make_spec(ts.parse_overrides(["intent_robots=2"]))
    assert spec.intent_robots == 2 and isinstance(spec.intent_robots, int)
    assert ts.obs_dim(spec) == 41 + 2 * 7 == len(ts.obs_names(spec))
    intent = torch.zeros(2, 2, 3, 2)
    intent[:, 0] = torch.tensor([[1.0, 0.0], [3.0, 0.5], [-5.0, -1.0]])  # m, robot frame
    intent[:, 1] = 9.0  # absent robot: must come out as zeros
    mask = torch.tensor([[True, False], [True, False]])
    obs = ts.build_observation(*_obs_inputs(), spec, intent=intent, intent_mask=mask)
    names = ts.obs_names(spec)
    row = dict(zip(names, obs[0].tolist(), strict=True))
    assert row["robot0_present"] == 1.0 and row["robot1_present"] == 0.0
    assert (row["robot0_p1_x"], row["robot0_p2_x"], row["robot0_p3_x"]) == (0.5, 1.0, -1.0)
    assert row["robot0_p2_y"] == 0.25
    assert all(row[f"robot1_p{k}_{a}"] == 0.0 for k in (1, 2, 3) for a in "xy")


def test_intent_layout_without_intent_input_is_all_absent():
    # an env with no fleet robots (goal-reach) still builds the wider layout: robots absent
    spec = ts.make_spec({"intent_robots": 2})
    obs = ts.build_observation(*_obs_inputs(), spec)
    assert obs.shape == (2, 55)
    assert (obs[:, 41:] == 0).all()


def test_the_top_speed_is_the_one_source(tmp_path):
    import yaml

    with open(ts.NAV_SPEED_FILE, encoding="utf-8") as f:
        want = yaml.safe_load(f)["nav_speed_mps"]
    assert ts.ACTION_LIMITS.v_max == ts.NAV_SPEED_MPS == want
    other = tmp_path / "nav_speed.yaml"
    other.write_text("nav_speed_mps: 0.2\n", encoding="utf-8")
    assert ts.nav_speed_mps(str(other)) == 0.2
    other.write_text("nav_speed_mps: 0\n", encoding="utf-8")
    with pytest.raises(ValueError):
        ts.nav_speed_mps(str(other))


def test_times_stretch_so_the_robot_covers_the_tuned_distance():
    """Rule: a time tuned at TIMES_TUNED_AT_MPS (0.25 m/s) is multiplied by 0.25 / v."""
    assert ts.TIMES_TUNED_AT_MPS == 0.25
    assert ts.travel_time(20.0, 0.25) == pytest.approx(20.0)
    assert ts.travel_time(20.0, 0.12) == pytest.approx(20.0 * 0.25 / 0.12)
    assert ts.SPEC.episode_s == pytest.approx(ts.travel_time(20.0))
    assert ts.SPEC.episode_s * ts.NAV_SPEED_MPS == pytest.approx(20.0 * 0.25)
