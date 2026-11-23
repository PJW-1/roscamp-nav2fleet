"""Tests for the simulator-free task rules. Run: <venv python> -m pytest learning/goal_reach -q"""

import dataclasses
import math
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


def test_priority_flag_follows_the_present_flag_when_on():
    spec = ts.make_spec(ts.parse_overrides(["intent_robots=2", "intent_priority=1"]))
    names = ts.obs_names(spec)
    assert ts.obs_dim(spec) == 41 + 2 * 8 == len(names)
    assert names[41:43] == ["robot0_present", "robot0_priority"]
    intent = torch.ones(2, 2, 3, 2)
    mask = torch.tensor([[True, True], [True, False]])
    prio = torch.tensor([[True, False], [True, True]])
    obs = ts.build_observation(*_obs_inputs(), spec, intent=intent, intent_mask=mask,
                               intent_priority=prio)
    row1 = dict(zip(names, obs[1].tolist(), strict=True))
    assert row1["robot0_priority"] == 1.0 and row1["robot1_priority"] == 0.0  # absent: 0
    off = ts.make_spec(ts.parse_overrides(["intent_robots=2"]))
    assert "robot0_priority" not in ts.obs_names(off)  # default off: 55 dims as before


def _base_inputs(n=3, seed=0):
    g = torch.Generator().manual_seed(seed)
    ranges = 0.1 + 2.5 * torch.rand(n, 360, generator=g)
    vel = torch.rand(n, 2, generator=g)
    return ranges, torch.rand(n, generator=g), torch.rand(n, generator=g), vel


def test_optional_blocks_go_between_base_and_intent_in_order():
    spec = dataclasses.replace(
        ts.SPEC, prev_action_obs=1, lidar_frames=3, speed_cap_obs=1, intent_robots=1
    )
    assert ts.obs_blocks(spec) == ["prev_action", "lidar_history", "speed_cap", "intent"]
    names = ts.obs_names(spec)
    assert len(names) == ts.obs_dim(spec) == 41 + 2 + 72 + 1 + 7
    assert names[41:43] == ["prev_v", "prev_w"]
    assert names[43] == "lidar_hist1_sector_0" and names[43 + 36] == "lidar_hist2_sector_0"
    assert names[115] == "speed_cap" and names[116] == "robot0_present"
    ranges, d, a, vel = _base_inputs()
    hist = torch.rand(3, 2, 36)
    obs = ts.build_observation(
        ranges, d, a, vel, spec, prev_target=torch.tensor([[0.06, -0.5]] * 3),
        lidar_history=hist, speed_cap=torch.full((3,), 0.16),
    )
    base = ts.build_observation(ranges, d, a, vel)
    assert obs.shape == (3, ts.obs_dim(spec))
    assert torch.equal(obs[:, :41], base)  # base entries unchanged
    v_scale = ts.ACTION_LIMITS.v_max
    assert torch.allclose(obs[:, 41:43], torch.tensor([0.06 / v_scale, -0.5]).expand(3, 2))
    assert torch.equal(obs[:, 43:115], hist.reshape(3, -1))  # newest first, as given
    assert torch.allclose(obs[:, 115], torch.full((3,), 0.16 / v_scale))
    assert torch.equal(obs[:, 116:], torch.zeros(3, 7))  # intent None: absent


def test_missing_block_inputs_take_their_episode_start_values():
    spec = dataclasses.replace(ts.SPEC, prev_action_obs=1, lidar_frames=3, speed_cap_obs=1)
    ranges, d, a, vel = _base_inputs()
    obs = ts.build_observation(ranges, d, a, vel, spec)
    assert torch.equal(obs[:, 41:43], torch.zeros(3, 2))
    assert torch.equal(obs[:, 43:79], obs[:, :36]) and torch.equal(obs[:, 79:115], obs[:, :36])
    assert torch.allclose(obs[:, 115], torch.ones(3))


def test_default_spec_keeps_the_ppo_k_layout():
    assert ts.obs_blocks(ts.SPEC) == []
    assert ts.obs_dim(ts.SPEC) == ts.BASE_DIM == 41


def test_speed_cap_sets_the_v_upper_limit_and_still_action_stops():
    lim = ts.ACTION_LIMITS
    cap = torch.tensor([0.08, 0.12, 0.16])
    full = ts.action_to_target(torch.ones(3, 2), v_cap=cap)
    assert torch.allclose(full[:, 0], cap)
    low = ts.action_to_target(-torch.ones(3, 2), v_cap=cap)
    assert torch.allclose(low[:, 0], torch.full((3,), lim.v_min))
    assert torch.allclose(ts.action_to_target(torch.ones(1, 2))[:, 0], torch.tensor([lim.v_max]))
    still = ts.action_to_target(ts.still_action(3, v_cap=cap), v_cap=cap)
    assert torch.allclose(still, torch.zeros(3, 2), atol=1e-7)
    assert torch.allclose(ts.action_to_target(ts.still_action(2)), torch.zeros(2, 2), atol=1e-7)


def test_capped_action_is_the_full_speed_target_clamped_to_the_cap():
    lim = ts.ACTION_LIMITS
    cap = torch.tensor([0.08, 0.08, 0.16, 0.16, 0.12])
    a = torch.tensor([[1.0, 0.3], [-1.0, -0.5], [1.0, 1.0], [0.2, 0.0], [1.5, -2.0]])
    full = ts.action_to_target(a)
    got = ts.action_to_target(ts.capped_action(a, cap), v_cap=cap)
    assert torch.allclose(got[:, 0], torch.minimum(full[:, 0], cap), atol=1e-7)
    assert torch.allclose(got[:, 1], full[:, 1])
    assert torch.allclose(got[2, 0], torch.tensor(lim.v_max))  # no faster than its own top
    assert torch.allclose(got[4], full[4], atol=1e-7)  # at the trained top it changes nothing


def test_speed_cap_draw_stays_in_range_and_a_single_value_draws_nothing():
    spec = dataclasses.replace(ts.SPEC, speed_cap_range=(0.08, 0.16))
    torch.manual_seed(0)
    c = ts.sample_speed_cap(10000, "cpu", spec)
    assert c.min() >= 0.08 and c.max() <= 0.16 and abs(c.mean().item() - 0.12) < 0.002
    state = torch.random.get_rng_state()
    assert torch.equal(ts.sample_speed_cap(4, "cpu"), torch.full((4,), ts.NAV_SPEED_MPS))
    assert torch.equal(state, torch.random.get_rng_state())  # fixed cap: random stream unchanged


def test_flip_step_counts_like_nav_bench_sign_changes():
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(here, "..", "..", "tools", "nav_bench"))
    import metrics

    g = torch.Generator().manual_seed(3)
    seq = (torch.rand(200, 5, generator=g) - 0.5) * 0.2  # many values near the deadband
    seq[:, 0] = 0.0
    seq[50, 1] = 0.02  # exactly on the deadband counts (nav_bench skips only |w| < deadband)
    last = torch.zeros(5)
    count = torch.zeros(5)
    for w in seq:
        f, last = ts.flip_step(w, last, 0.02)
        count += f.float()
    for e in range(5):
        assert count[e].item() == metrics.sign_changes(seq[:, e].tolist(), 0.02)


def test_action_rate_terms():
    spec = dataclasses.replace(ts.SPEC, w_action_rate=-0.1, w_w_flip=-0.5)
    lim = ts.ACTION_LIMITS
    t = torch.tensor([[lim.v_max, 1.0], [0.0, 0.0]])
    p = torch.tensor([[0.0, -1.0], [0.0, 0.0]])
    terms = ts.action_rate_terms(t, p, torch.tensor([True, False]), spec)
    assert torch.allclose(terms["action_rate"], torch.tensor([-0.1 * (1 + 4), 0.0]))
    assert torch.allclose(terms["w_flip"], torch.tensor([-0.5, 0.0]))


def test_progress_reward_scales_with_the_episode_cap():
    lim = ts.ACTION_LIMITS
    dt = ts.SPEC.control_dt
    no = torch.zeros(2, dtype=torch.bool)
    cap = torch.tensor([0.08, 0.16])
    _, terms = ts.compute_reward(cap * dt, torch.zeros(2), no, no, v_cap=cap)
    assert torch.allclose(terms["progress"], torch.ones(2))
    _, terms = ts.compute_reward(torch.tensor([lim.v_max * dt]), torch.zeros(1), no[:1], no[:1])
    assert torch.allclose(terms["progress"], torch.ones(1))


def test_privileged_block_is_in_our_frame_and_zero_when_absent():
    xy = torch.tensor([[1.0, 1.0]])
    yaw = torch.tensor([math.pi / 2])  # facing +y
    pos = torch.tensor([[[1.0, 2.0], [5.0, 5.0]]])  # 1 m ahead; far away
    vel = torch.tensor([[[0.25, 0.0], [0.1, 0.1]]])  # moving +x = to our right
    out = ts.privileged_block(pos, vel, torch.tensor([[True, False]]), xy, yaw)
    assert out.shape == (1, 2 * ts.PRIV_PER_OBJECT)
    assert torch.allclose(out[0, :5], torch.tensor([1.0, 0.5, 0.0, 0.0, -0.5]), atol=1e-6)
    assert torch.equal(out[0, 5:], torch.zeros(5))
