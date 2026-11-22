"""Widen an rsl_rl checkpoint for new observation entries so the policy starts unchanged.

The new entries are appended after the old ones (task_spec puts the optional blocks after the 41
base entries, and ppo_k has no intent block; the critic's privileged entries come after the
actor's). Their first-layer weights are zero, so actor and critic outputs equal the old ones for
any values in the new entries. Their normaliser mean and variance are set from given values
(e.g. the base entry they repeat): the normaliser has seen about 1e8 samples, so it barely
moves later, and a fixed (0, 1) would leave a badly scaled input. The optimizer state is dropped
(its moment tensors have the old shapes); train.py then loads without it (--init).

No Isaac needed:
  python learning/store_arena/widen_checkpoint.py <model.pt> <out.pt> --set prev_action_obs=1 ...
"""

import argparse
import dataclasses
import os
import sys

import torch

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "goal_reach"))
import task_spec as ts  # noqa: E402

N_MOVERS = 2  # mover_spec.MOVERS: a person and a cart


def widen_state(sd: dict, new_dim: int, mean: torch.Tensor, var: torch.Tensor) -> dict:
    """One model state dict (actor or critic) with new_dim inputs: old entries first."""
    old = sd["mlp.0.weight"].shape[1]
    k = new_dim - old
    if k < 0 or len(mean) != k or len(var) != k:
        raise ValueError(f"widen {old} -> {new_dim} needs {k} stats, got {len(mean)}, {len(var)}")
    out = dict(sd)
    w = sd["mlp.0.weight"]
    out["mlp.0.weight"] = torch.cat((w, w.new_zeros(w.shape[0], k)), 1)
    m, v = mean.to(w.dtype), var.to(w.dtype)
    out["obs_normalizer._mean"] = torch.cat((sd["obs_normalizer._mean"], m[None]), 1)
    out["obs_normalizer._var"] = torch.cat((sd["obs_normalizer._var"], v[None]), 1)
    out["obs_normalizer._std"] = torch.cat((sd["obs_normalizer._std"], v.sqrt()[None]), 1)
    return out


def block_stats(sd: dict, spec) -> tuple[torch.Tensor, torch.Tensor]:
    """Normaliser mean and variance for the optional actor blocks of spec (not intent): each
    repeats the base entry it copies; the speed cap is uniform over speed_cap_range."""
    mean, var = sd["obs_normalizer._mean"][0], sd["obs_normalizer._var"][0]
    lid = slice(0, ts.OBS_LIDAR)
    ms, vs = [], []
    if spec.prev_action_obs:
        ms.append(mean[ts.BASE_DIM - 2 :])
        vs.append(var[ts.BASE_DIM - 2 :])
    for _ in range(1, spec.lidar_frames):
        ms.append(mean[lid])
        vs.append(var[lid])
    if spec.speed_cap_obs:
        lo, hi = (c / ts.ACTION_LIMITS.v_max for c in spec.speed_cap_range)
        ms.append(mean.new_tensor([(lo + hi) / 2]))
        vs.append(mean.new_tensor([max((hi - lo) ** 2 / 12, 1e-4)]))
    if spec.intent_robots:
        raise ValueError("widening for the intent block is not supported")
    if not ms:
        return mean.new_zeros(0), var.new_zeros(0)
    return torch.cat(ms), torch.cat(vs)


def privileged_stats(n_objects: int) -> tuple[torch.Tensor, torch.Tensor]:
    """Rough normaliser stats for the critic's privileged entries [present, x, y, vx, vy]:
    present about half the time, positions and velocities around 0 with std 0.5 (scaled)."""
    m = torch.tensor([0.5, 0.0, 0.0, 0.0, 0.0]).repeat(n_objects)
    v = torch.tensor([0.25, 0.25, 0.25, 0.25, 0.25]).repeat(n_objects)
    return m, v


def widen_checkpoint(ck: dict, spec, n_privileged_objects: int = 0) -> dict:
    """The checkpoint for spec's actor layout (and critic: actor layout plus privileged)."""
    actor, critic = ck["actor_state_dict"], ck["critic_state_dict"]
    a_dim = ts.obs_dim(spec)
    am, av = block_stats(actor, spec)
    cm, cv = block_stats(critic, spec)
    c_dim = a_dim
    if spec.critic_privileged:
        pm, pv = privileged_stats(n_privileged_objects)
        cm, cv = torch.cat((cm, pm)), torch.cat((cv, pv))
        c_dim += len(pm)
    out = {k: v for k, v in ck.items() if k != "optimizer_state_dict"}
    out["actor_state_dict"] = widen_state(actor, a_dim, am, av)
    out["critic_state_dict"] = widen_state(critic, c_dim, cm, cv)
    out["widened_from"] = {
        "actor": actor["mlp.0.weight"].shape[1],
        "critic_privileged": c_dim - a_dim,
    }
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("checkpoint")
    p.add_argument("out")
    p.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    p.add_argument("--robot_max", type=int, default=2, help="ArenaSpec.robot_max (privileged)")
    args = p.parse_args()
    spec = ts.make_spec(ts.parse_overrides(args.set))
    ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    out = widen_checkpoint(ck, spec, N_MOVERS + args.robot_max)
    out["widened_spec"] = {k: v for k, v in dataclasses.asdict(spec).items()}
    torch.save(out, args.out)
    a_dim = out["actor_state_dict"]["mlp.0.weight"].shape[1]
    c_dim = out["critic_state_dict"]["mlp.0.weight"].shape[1]
    print(f"actor {out['widened_from']['actor']} -> {a_dim}, critic -> {c_dim}: {args.out}")


if __name__ == "__main__":
    main()
