"""rsl_rl 체크포인트를 새 관측 항목만큼 넓혀 정책이 바뀌지 않은 채 시작하게 한다.

새 항목은 옛 항목 뒤에 붙고 첫 층 가중치가 0 이라 actor, critic 출력이 그대로다.
정규화 평균·분산은 주어진 값(보통 따라 하는 기본 항목)으로 둔다. optimizer 상태는 버린다(train.py
--init).

Isaac 불필요:
  python learning/store_arena/widen_checkpoint.py <model.pt> <out.pt> --set prev_action_obs=1 ...
"""

import argparse
import dataclasses
import os
import sys

import torch

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "goal_reach"))
import task_spec as ts  # noqa: E402

N_MOVERS = 2  # mover_spec.MOVERS: 사람과 카트


def widen_state(sd: dict, new_dim: int, mean: torch.Tensor, var: torch.Tensor) -> dict:
    """입력을 new_dim 개로 넓힌 모델 state dict (actor 또는 critic), 옛 항목이 앞."""
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


def insert_state(sd: dict, at: int, mean: torch.Tensor, var: torch.Tensor) -> dict:
    """입력 번호 at 앞에 새 입력 len(mean) 개를 끼운 모델 state dict."""
    k = len(mean)
    out = dict(sd)
    w = sd["mlp.0.weight"]
    out["mlp.0.weight"] = torch.cat((w[:, :at], w.new_zeros(w.shape[0], k), w[:, at:]), 1)
    m, v = mean.to(w.dtype)[None], var.to(w.dtype)[None]
    for key, new in (("_mean", m), ("_var", v), ("_std", v.sqrt())):
        old = sd["obs_normalizer." + key]
        out["obs_normalizer." + key] = torch.cat((old[:, :at], new, old[:, at:]), 1)
    return out


def intent_stats(spec) -> tuple[torch.Tensor, torch.Tensor]:
    """intent 블록의 대략적 정규화 통계: 존재 약 절반, 우선순위 약 1/4, 경로점은 0 근처 표준편차 약
    0.3."""
    head_m, head_v = [0.5], [0.25]
    if spec.intent_priority:
        head_m, head_v = head_m + [0.25], head_v + [0.1875]
    m = torch.tensor(head_m + [0.0] * (2 * spec.intent_points)).repeat(spec.intent_robots)
    v = torch.tensor(head_v + [0.1] * (2 * spec.intent_points)).repeat(spec.intent_robots)
    return m, v


def add_intent(ck: dict, spec) -> dict:
    """intent 블록 없이 학습한 ck 에 intent 블록을 더함 (item 6).
    actor 입력 끝, critic 특권 항목 앞에 첫 층 가중치 0 으로 넣어 두 출력이 그대로."""
    if not spec.intent_robots:
        raise ValueError("spec has no intent block")
    base = dataclasses.replace(spec, intent_robots=0, intent_priority=0)
    at = ts.obs_dim(base)
    actor, critic = ck["actor_state_dict"], ck["critic_state_dict"]
    if actor["mlp.0.weight"].shape[1] != at:
        raise ValueError(f"checkpoint actor takes {actor['mlp.0.weight'].shape[1]}, not {at}")
    m, v = intent_stats(spec)
    out = {k: x for k, x in ck.items() if k != "optimizer_state_dict"}
    out["actor_state_dict"] = insert_state(actor, at, m, v)
    out["critic_state_dict"] = insert_state(critic, at, m, v)
    return out


def block_stats(sd: dict, spec) -> tuple[torch.Tensor, torch.Tensor]:
    """spec 의 선택 actor 블록 (intent 제외) 정규화 평균·분산: 따라 하는 기본 항목 값, 속도 상한은
    speed_cap_range 균등."""
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
    """critic 특권 항목 [존재, x, y, vx, vy] 의 대략적 정규화 통계: 존재 약 절반, 위치·속도는 0 근처
    표준편차 0.5."""
    m = torch.tensor([0.5, 0.0, 0.0, 0.0, 0.0]).repeat(n_objects)
    v = torch.tensor([0.25, 0.25, 0.25, 0.25, 0.25]).repeat(n_objects)
    return m, v


def widen_checkpoint(ck: dict, spec, n_privileged_objects: int = 0) -> dict:
    """spec 의 actor 배치 (critic 은 actor 배치 + 특권 항목) 에 맞춘 체크포인트."""
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
    p.add_argument(
        "--add_intent",
        action="store_true",
        help="the checkpoint already has the --set layout without intent (a recipe run): only "
        "insert the intent block",
    )
    args = p.parse_args()
    spec = ts.make_spec(ts.parse_overrides(args.set))
    ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    if args.add_intent:
        out = add_intent(ck, spec)
    else:
        out = widen_checkpoint(ck, spec, N_MOVERS + args.robot_max)
    out["widened_spec"] = {k: v for k, v in dataclasses.asdict(spec).items()}
    torch.save(out, args.out)
    a_dim = out["actor_state_dict"]["mlp.0.weight"].shape[1]
    c_dim = out["critic_state_dict"]["mlp.0.weight"].shape[1]
    print(
        f"actor {ck['actor_state_dict']['mlp.0.weight'].shape[1]} -> {a_dim}, critic -> {c_dim}"
        f": {args.out}"
    )


if __name__ == "__main__":
    main()
