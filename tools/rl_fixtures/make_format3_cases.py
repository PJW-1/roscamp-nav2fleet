import argparse
import ast
import json
import sys
import types
from pathlib import Path

import numpy as np
import torch

SETS = {
    "prev_action": {"prev_action_obs": 1},
    "lidar_history": {"lidar_frames": 3, "lidar_stride_steps": 4},
    "speed_cap": {"speed_cap_obs": 1, "speed_cap_range": (0.06, 0.12)},
    "all": {
        "prev_action_obs": 1,
        "lidar_frames": 3,
        "lidar_stride_steps": 4,
        "speed_cap_obs": 1,
        "speed_cap_range": (0.06, 0.12),
    },
}
EPISODES = (13, 6)


# ---- 레시피 코드 불러오기 ----
def load_recipe(root: Path):
    sys.path.insert(0, str(root / "learning"))
    from goal_reach import export_policy, task_spec

    src = (root / "learning" / "store_arena" / "store_arena_env.py").read_text(encoding="utf-8")
    fn = next(
        n
        for n in ast.walk(ast.parse(src))
        if isinstance(n, ast.FunctionDef) and n.name == "lidar_history"
    )
    ns = {"ts": task_spec, "torch": torch}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "store_arena_env.py", "exec"), ns)
    return task_spec, export_policy, ns["lidar_history"]


# ---- 입력 순서 ----
def make_steps(rng):
    steps = []
    for length in EPISODES:
        cap = round(float(rng.uniform(0.06, 0.12)), 3)
        for k in range(length):
            rays = np.round(rng.uniform(0.06, 3.0, 360), 3)
            for sector in rng.choice(36, size=4, replace=False):
                rays[sector * 10 : sector * 10 + 10] = np.round(rng.uniform(2.05, 3.0, 10), 3)
            steps.append(
                {
                    "reset": k == 0,
                    "rays": rays.tolist(),
                    "goal_dist": round(float(rng.uniform(0.0, 4.0)), 4),
                    "goal_angle": round(float(rng.uniform(-3.1, 3.1)), 4),
                    "v": round(float(rng.uniform(-0.05, 0.12)), 4),
                    "w": round(float(rng.uniform(-1.0, 1.0)), 4),
                    "speed_cap": cap,
                    "action": np.round(rng.uniform(-1.3, 1.3, 2), 4).tolist(),
                }
            )
    return steps


# ---- 기대값: 레시피 env와 같은 순서로 관측 -> 행동 -> target ----
def expected(ts, lidar_history, spec, steps):
    hist = max(spec.lidar_frames - 1, 0) * spec.lidar_stride_steps + 1
    env = types.SimpleNamespace(
        spec=spec,
        lid_buf=torch.zeros(1, hist, spec.lidar_sectors),
        lid_fresh=torch.ones(1, dtype=torch.bool),
    )
    lo, hi = spec.speed_cap_range
    cap_active = lo != hi or lo != ts.NAV_SPEED_MPS
    target = torch.zeros(1, 2)
    obs_out, target_out = [], []
    for s in steps:
        cap = torch.tensor([s["speed_cap"]]) if cap_active else None
        if s["reset"]:
            target = torch.zeros(1, 2)
            env.lid_fresh[:] = True
        ranges = torch.tensor([s["rays"]], dtype=torch.float32)
        obs = ts.build_observation(
            ranges,
            torch.tensor([s["goal_dist"]]),
            torch.tensor([s["goal_angle"]]),
            torch.tensor([[s["v"], s["w"]]]),
            spec,
            prev_target=target,
            lidar_history=lidar_history(env, ranges),
            speed_cap=cap if cap is not None else torch.tensor([ts.ACTION_LIMITS.v_max]),
        )[0]
        target = ts.action_to_target(torch.tensor([s["action"]]), v_cap=cap)
        obs_out.append([float(x) for x in obs])
        target_out.append([float(x) for x in target[0]])
    return obs_out, target_out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recipe", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--commit", required=True)
    args = ap.parse_args()
    ts, ex, lidar_history = load_recipe(args.recipe)
    steps = make_steps(np.random.default_rng(20261122))
    sets = {}
    for name, overrides in SETS.items():
        spec = ts.make_spec(overrides)
        obs, target = expected(ts, lidar_history, spec, steps)
        sets[name] = {
            **ex.block_entries(spec),
            "obs_dim": ts.obs_dim(spec),
            "obs_layout": ts.obs_names(spec),
            "obs": obs,
            "target": target,
        }
    out = {"source": {"recipe_commit": args.commit}, "steps": steps, "sets": sets}
    args.out.write_text(json.dumps(out, separators=(",", ":")) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
