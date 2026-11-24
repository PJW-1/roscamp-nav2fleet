"""넓힌 체크포인트가 옛 출력을 그대로 내는지. 실행: python -m pytest learning/store_arena"""

import dataclasses
import os
import sys

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))
import mover_spec as ms  # noqa: E402
import task_spec as ts  # noqa: E402
import widen_checkpoint as wc  # noqa: E402

PPO_K = "D:/isaac/runs/store_arena/ppo_k/c2/model_998.pt"
EPS = 1e-2  # rsl_rl EmpiricalNormalization 기본값


def forward(sd: dict, x: torch.Tensor) -> torch.Tensor:
    """관측 정규화가 있는 rsl_rl MLPModel: 정규화, Linear-ELU-Linear-ELU-Linear."""
    sd = {k: v.to(x.dtype) if v.is_floating_point() else v for k, v in sd.items()}
    x = (x - sd["obs_normalizer._mean"]) / (sd["obs_normalizer._std"] + EPS)
    x = torch.nn.functional.elu(x @ sd["mlp.0.weight"].T + sd["mlp.0.bias"])
    x = torch.nn.functional.elu(x @ sd["mlp.2.weight"].T + sd["mlp.2.bias"])
    return x @ sd["mlp.4.weight"].T + sd["mlp.4.bias"]


def fake_checkpoint(dim=41, seed=0) -> dict:
    g = torch.Generator().manual_seed(seed)

    def model(out):
        var = 0.01 + torch.rand(1, dim, generator=g)
        return {
            "obs_normalizer._mean": torch.randn(1, dim, generator=g),
            "obs_normalizer._var": var,
            "obs_normalizer._std": var.sqrt(),
            "obs_normalizer.count": torch.tensor(10**8),
            "mlp.0.weight": torch.randn(128, dim, generator=g) * 0.3,
            "mlp.0.bias": torch.randn(128, generator=g) * 0.1,
            "mlp.2.weight": torch.randn(128, 128, generator=g) * 0.1,
            "mlp.2.bias": torch.randn(128, generator=g) * 0.1,
            "mlp.4.weight": torch.randn(out, 128, generator=g) * 0.1,
            "mlp.4.bias": torch.randn(out, generator=g) * 0.1,
        }

    actor = model(2)
    actor["distribution.std_param"] = torch.tensor([0.1, 0.15])
    return {
        "actor_state_dict": actor,
        "critic_state_dict": model(1),
        "iter": 998,
        "optimizer_state_dict": {"state": {}},
        "infos": None,
    }


SPECS = {
    "control": {},
    "a_prev_action": {"prev_action_obs": 1},
    "b_lidar_history": {"lidar_frames": 3},
    "c_privileged": {"critic_privileged": 1},
    "d_speed_cap": {"speed_cap_obs": 1, "speed_cap_range": (0.08, 0.16)},
    "all": {
        "prev_action_obs": 1,
        "lidar_frames": 3,
        "critic_privileged": 1,
        "speed_cap_obs": 1,
        "speed_cap_range": (0.08, 0.16),
    },
}


def check_same_outputs(ck: dict, over: dict):
    spec = dataclasses.replace(ts.SPEC, **over)
    n_obj = wc.N_MOVERS + 2
    new = wc.widen_checkpoint(ck, spec, n_obj)
    a_dim = ts.obs_dim(spec)
    c_dim = a_dim + (ts.PRIV_PER_OBJECT * n_obj if spec.critic_privileged else 0)
    assert new["actor_state_dict"]["mlp.0.weight"].shape[1] == a_dim
    assert new["critic_state_dict"]["mlp.0.weight"].shape[1] == c_dim
    assert "optimizer_state_dict" not in new
    g = torch.Generator().manual_seed(1)
    x = torch.rand(4096, c_dim, generator=g, dtype=torch.float64) * 4 - 2  # 새 항목 포함
    # float64 에서는 정확히 같음 (0 가중치는 정확한 0 을 더함)
    # float32 는 넓은 행렬곱의 합산 순서가 달라 끝자리가 다를 수 있음
    old_a = forward(ck["actor_state_dict"], x[:, :41])
    old_c = forward(ck["critic_state_dict"], x[:, :41])
    assert torch.equal(forward(new["actor_state_dict"], x[:, :a_dim]), old_a)
    assert torch.equal(forward(new["critic_state_dict"], x), old_c)
    x32 = x.float()
    new_a32 = forward(new["actor_state_dict"], x32[:, :a_dim])
    assert torch.allclose(
        new_a32, forward(ck["actor_state_dict"], x32[:, :41]), rtol=1e-5, atol=1e-5
    )
    for key in ("mlp.2.weight", "mlp.4.weight", "mlp.4.bias"):
        assert torch.equal(new["actor_state_dict"][key], ck["actor_state_dict"][key])
    sd = new["actor_state_dict"]
    assert torch.equal(
        sd["distribution.std_param"], ck["actor_state_dict"]["distribution.std_param"]
    )
    assert torch.isfinite(sd["obs_normalizer._std"]).all() and (sd["obs_normalizer._std"] > 0).all()


@pytest.mark.parametrize("name", list(SPECS))
def test_widened_fake_checkpoint_gives_the_same_outputs(name):
    check_same_outputs(fake_checkpoint(), SPECS[name])


@pytest.mark.skipif(not os.path.exists(PPO_K), reason="ppo_k checkpoint is on this PC's D: only")
@pytest.mark.parametrize("name", list(SPECS))
def test_widened_ppo_k_gives_the_same_outputs(name):
    check_same_outputs(torch.load(PPO_K, map_location="cpu", weights_only=False), SPECS[name])


def test_new_entries_take_the_stats_of_the_entries_they_repeat():
    ck = fake_checkpoint()
    spec = dataclasses.replace(ts.SPEC, prev_action_obs=1, lidar_frames=3)
    sd = wc.widen_checkpoint(ck, spec)["actor_state_dict"]
    old = ck["actor_state_dict"]["obs_normalizer._mean"][0]
    m = sd["obs_normalizer._mean"][0]
    assert torch.equal(m[41:43], old[39:41])  # 직전 v, w 는 v, w 를 따름
    assert torch.equal(m[43:79], old[:36]) and torch.equal(m[79:115], old[:36])


def test_the_privileged_object_count_matches_the_movers():
    assert wc.N_MOVERS == len(ms.MOVERS)


@pytest.mark.parametrize("prio", [0, 1])
@pytest.mark.parametrize("privileged", [0, 1])
def test_adding_intent_to_a_recipe_checkpoint_keeps_the_outputs(prio, privileged):
    recipe = {"prev_action_obs": 1, "lidar_frames": 3, "critic_privileged": privileged}
    spec0 = dataclasses.replace(ts.SPEC, **recipe)
    ck = wc.widen_checkpoint(fake_checkpoint(), spec0, wc.N_MOVERS + 2)
    g0 = torch.Generator().manual_seed(5)
    for key in ("actor_state_dict", "critic_state_dict"):  # 학습됨: 모든 열이 0 아님
        w = ck[key]["mlp.0.weight"]
        ck[key]["mlp.0.weight"] = torch.randn(w.shape, generator=g0) * 0.3
    spec = dataclasses.replace(spec0, intent_robots=2, intent_priority=prio)
    new = wc.add_intent(ck, spec)
    at, a_dim = ts.obs_dim(spec0), ts.obs_dim(spec)
    n_priv = ts.PRIV_PER_OBJECT * (wc.N_MOVERS + 2) if privileged else 0
    assert new["actor_state_dict"]["mlp.0.weight"].shape[1] == a_dim
    assert new["critic_state_dict"]["mlp.0.weight"].shape[1] == a_dim + n_priv
    g = torch.Generator().manual_seed(2)
    x = torch.rand(2048, a_dim + n_priv, generator=g, dtype=torch.float64) * 4 - 2
    keep = torch.cat((torch.arange(at), torch.arange(a_dim, a_dim + n_priv)))
    old_a = forward(ck["actor_state_dict"], x[:, :at])
    old_c = forward(ck["critic_state_dict"], x[:, keep])
    assert torch.equal(forward(new["actor_state_dict"], x[:, :a_dim]), old_a)
    assert torch.equal(forward(new["critic_state_dict"], x), old_c)
