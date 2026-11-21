"""Train the Pinky goal-reach policy with rsl_rl PPO on Newton (headless).

Run from D:\\isaac\\IsaacLab with the venv python (see run_train.sh). Writes into --out:
metrics.csv (episode outcomes per iteration with wall time), model_*.pt, tensorboard events,
run.json (settings and timing).
"""

import argparse
import json
import os
import sys
import time

import warp as wp

wp.config.enable_backward = False  # as Isaac Lab's train.py: skip adjoint kernels

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=300)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--out", type=str, required=True)
parser.add_argument(
    "--set",
    action="append",
    default=[],
    metavar="FIELD=VALUE",
    help="Override a TaskSpec field (reward weights, success_radius), e.g. --set w_time=-0.1",
)
add_launcher_args(parser)
args_cli = parser.parse_args()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def parse_overrides(items):
    out = {}
    for it in items:
        k, v = it.split("=", 1)
        out[k] = float(v)
    return out


def main():
    from goal_reach_env import GoalReachEnvCfg

    env_cfg = GoalReachEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.spec_overrides = parse_overrides(args_cli.set)
    os.makedirs(args_cli.out, exist_ok=True)
    out = os.path.abspath(args_cli.out)
    env_cfg.metrics_csv = os.path.join(out, "metrics.csv")
    if os.path.exists(env_cfg.metrics_csv):
        os.remove(env_cfg.metrics_csv)

    with launch_simulation(env_cfg, args_cli):
        import torch
        from agent_cfg import GoalReachPPORunnerCfg
        from goal_reach_env import GoalReachEnv
        from isaaclab_rl.rsl_rl import (
            RslRlVecEnvWrapper,
            check_rsl_rl_version,
            create_rsl_rl_runner,
            handle_deprecated_rsl_rl_cfg,
        )

        agent_cfg = GoalReachPPORunnerCfg()
        agent_cfg.max_iterations = args_cli.max_iterations
        agent_cfg.seed = args_cli.seed
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, check_rsl_rl_version())
        env_cfg.metrics_every = agent_cfg.num_steps_per_env

        t0 = time.time()
        env = GoalReachEnv(env_cfg)
        t_env = time.time() - t0
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(env, agent_cfg, log_dir=out)
        t1 = time.time()
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        torch.cuda.synchronize()
        t_learn = time.time() - t1
        steps = agent_cfg.max_iterations * agent_cfg.num_steps_per_env * args_cli.num_envs
        info = {
            "num_envs": args_cli.num_envs,
            "max_iterations": agent_cfg.max_iterations,
            "num_steps_per_env": agent_cfg.num_steps_per_env,
            "seed": args_cli.seed,
            "spec_overrides": env_cfg.spec_overrides,
            "env_build_s": round(t_env, 1),
            "learn_s": round(t_learn, 1),
            "env_steps": steps,
            "env_steps_per_s_incl_update": round(steps / t_learn, 0),
            "cuda_max_mem_alloc_MB": round(torch.cuda.max_memory_allocated() / 2**20, 1),
        }
        with open(os.path.join(out, "run.json"), "w", encoding="utf-8") as f:
            json.dump(info, f, indent=1)
        print("[GOAL] run info:", json.dumps(info))
        env.close()


if __name__ == "__main__":
    main()
