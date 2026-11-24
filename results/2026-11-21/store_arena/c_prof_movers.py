"""키네마틱 mover를 옮기는 방식별로 N개 환경의 env.step 시간을 잰다(Isaac)."""

import argparse
import json
import os
import sys
import time

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--steps", type=int, default=60)
parser.add_argument("--out", type=str, required=True)
add_launcher_args(parser)
args_cli = parser.parse_args()
REPO = "C:/Users/pjw/ros2camp/pd-arena/learning"
for p in ("store_arena", "goal_reach", "sim_check"):
    sys.path.insert(0, os.path.join(REPO, p))


def main():
    import store_arena_env as sae

    cfg = sae.StoreArenaEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.arena_overrides = {"robot_max": 0}  # mover만 둔 장면
    with launch_simulation(cfg, args_cli):
        import torch
        from isaaclab_newton.physics import NewtonManager as SimulationManager

        env = sae.StoreArenaEnv(cfg)
        env.reset()
        n, dev = env.num_envs, env.device
        act = torch.zeros(n, 2, device=dev)
        act[:, 0] = 0.3
        objs = env.mover_objs
        report = {"is_floating_base": [bool(o.root_view.is_floating_base) for o in objs]}
        orig_write = env._write_movers

        def write_pose_only(ids, skip=False, vel=False):
            origins = env.scene.env_origins[ids]
            for obj, (pos, yaw, _, _) in zip(objs, env.mover_state(ids), strict=True):
                pose = obj.data.default_root_pose.torch[ids].clone()
                pose[:, :2] = origins[:, :2] + pos
                z = torch.zeros_like(yaw)
                pose[:, 3:7] = torch.stack((z, z, torch.sin(yaw / 2), torch.cos(yaw / 2)), -1)
                obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids, skip_forward=skip)

        solver = SimulationManager._solver
        orig_notify = solver.notify_model_changed

        def write_no_notify(ids):
            solver.notify_model_changed = lambda *a, **k: None
            try:
                orig_write(ids)
            finally:
                solver.notify_model_changed = orig_notify

        variants = {
            "A_committed": orig_write,
            "B_no_writes": lambda ids: None,
            "C_pose_only": lambda ids: write_pose_only(ids),
            "D_pose_skip_forward": lambda ids: write_pose_only(ids, skip=True),
            "E_no_notify": write_no_notify,
        }
        for name, fn in variants.items():
            env._write_movers = fn
            for _ in range(5):
                env.step(act)
            torch.cuda.synchronize()
            t0 = time.time()
            for _ in range(args_cli.steps):
                env.step(act)
            torch.cuda.synchronize()
            dt = (time.time() - t0) / args_cli.steps
            report[name] = {"ms_per_step": round(1000 * dt, 2)}
            # 마지막 쓰기 뒤 sim mover가 mover_state 자리에 있는지 본다
            org = env.scene.env_origins[:, :2]
            err = 0.0
            for obj, (pos, _, _, _) in zip(objs, env.mover_state(), strict=True):
                err = max(err, float((obj.data.root_pos_w.torch[:, :2] - org - pos).norm(dim=-1).max()))
            report[name]["sim_vs_state_max_m"] = round(err, 6)
            print(name, report[name], flush=True)
        env._write_movers = orig_write
        os.makedirs(args_cli.out, exist_ok=True)
        with open(os.path.join(args_cli.out, "prof.json"), "w") as f:
            json.dump(report, f, indent=1)
        print(json.dumps(report))
        env.close()


main()
