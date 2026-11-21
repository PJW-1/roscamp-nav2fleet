"""Evaluate a trained store-arena policy (deterministic: action mean), headless.

Every env runs until it has finished --episodes episodes; the outcomes of those episodes are
counted, also split by the number of obstacle boxes on the route and by how close the robot came
to a mover (smallest gap between the robot centre and a mover surface in the episode).
Trajectories of the first --track envs are saved to rollouts.npz with their routes and movers
(plot_arena.py traj / anim draw them).

Run from D:\\isaac\\IsaacLab with the venv python, e.g.
  .venv/Scripts/python.exe <repo>/learning/store_arena/play.py \\
    --checkpoint <run>/c4/model_999.pt --out <dir> --viz none
"""

import argparse
import json
import os
import sys

import warp as wp

wp.config.enable_backward = False

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=1024)
parser.add_argument("--episodes", type=int, default=4, help="Episodes counted per env.")
parser.add_argument("--track", type=int, default=16, help="Envs whose trajectories are saved.")
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
add_launcher_args(parser)
args_cli = parser.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))


def main():
    import arena_spec as arena
    import store_arena_env as sae
    import timeouts

    task_over, arena_over = arena.split_overrides(args_cli.set)
    env_cfg = sae.StoreArenaEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.spec_overrides = task_over
    env_cfg.arena_overrides = arena_over
    os.makedirs(args_cli.out, exist_ok=True)

    with launch_simulation(env_cfg, args_cli):
        import numpy as np
        import torch
        from agent_cfg import GoalReachPPORunnerCfg
        from isaaclab_rl.rsl_rl import (
            RslRlVecEnvWrapper,
            check_rsl_rl_version,
            create_rsl_rl_runner,
            handle_deprecated_rsl_rl_cfg,
        )

        agent_cfg = handle_deprecated_rsl_rl_cfg(GoalReachPPORunnerCfg(), check_rsl_rl_version())
        raw = sae.StoreArenaEnv(env_cfg)
        env = RslRlVecEnvWrapper(raw, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(env, agent_cfg)
        runner.load(args_cli.checkpoint)
        policy = runner.get_inference_policy(device=raw.device)

        routes = raw.routes
        n_boxes = torch.as_tensor(
            (~np.isnan(routes.obstacle_xy[..., 0])).sum(-1), device=raw.device
        )  # per route
        n, E = args_cli.num_envs, args_cli.episodes
        ep_done = torch.zeros(n, dtype=torch.long, device=raw.device)
        keys = ("success", "collision", "oob", "timeout")
        by_boxes = {}  # boxes on the route -> outcome counts
        succ_times, succ_ratio, lidar_min = [], [], float("inf")
        K = min(args_cli.track, n)
        traj = {"xy": [], "done": [], "route": [], "cmd": [], "mover_xy": [], "mover_on": []}
        traj.update(robot_xy=[], robot_on=[], outcome=[])  # outcome: keys index at done, else -1
        traj.update(front_m=[])  # nearest lidar range within 30 deg of straight ahead
        n_rays = raw.lidar_ranges().shape[1]  # rays from -180 deg: straight ahead is n_rays // 2
        front = slice(n_rays // 2 - n_rays // 12, n_rays // 2 + n_rays // 12 + 1)
        # per counted episode: smallest mover gap, smallest scripted-robot gap, outcome index,
        # what the collision is put down to
        gaps, rgaps, codes, hits = [], [], [], []
        obs = env.get_observations()
        route = raw.route.clone()
        step = 0
        while True:
            with torch.inference_mode():
                obs, _, dones, _ = env.step(policy(obs))
            step += 1
            d = dones.bool()
            lidar_min = min(lidar_min, float(raw.lidar_ranges().min()))
            counted = d & (ep_done < E)
            if counted.any():
                outs = (
                    raw.success,
                    raw.collided,
                    raw.oob,
                    ~(raw.success | raw.collided | raw.oob),
                )
                for b in n_boxes[route[counted]].unique().tolist():
                    sel = counted & (n_boxes[route] == b)
                    row = by_boxes.setdefault(int(b), dict.fromkeys(keys, 0))
                    for k, o in zip(keys, outs, strict=True):
                        row[k] += int((o & sel).sum())
                gaps += raw.last_ep_mover_gap[counted].tolist()
                rgaps += raw.last_ep_robot_gap[counted].tolist()
                hits += raw.last_ep_hit_kind[counted].tolist()
                code = torch.stack(outs, -1).float().argmax(-1)
                codes += code[counted].tolist()
                s = raw.success & counted
                succ_times += raw.last_ep_len_s[s].tolist()
                # episode time vs the global path at full speed
                path_s = (
                    torch.as_tensor(routes.total, device=raw.device)[route[s]]
                    / arena.ts.ACTION_LIMITS.v_max
                )
                succ_ratio += (raw.last_ep_len_s[s] / path_s).tolist()
            ep_done += d.long()
            o_all = torch.stack(
                (raw.success, raw.collided, raw.oob, ~(raw.success | raw.collided | raw.oob)), -1
            ).float().argmax(-1)
            traj["outcome"].append(torch.where(d, o_all, -1)[:K].cpu().numpy())
            pos, _ = raw.robot_pose()
            traj["xy"].append(pos[:K].cpu().numpy())
            traj["done"].append(d[:K].cpu().numpy())
            traj["route"].append(route[:K].cpu().numpy())
            traj["cmd"].append(raw.cmd[:K].cpu().numpy())
            traj["front_m"].append(raw.lidar_ranges()[:K, front].amin(-1).cpu().numpy())
            ms_state = raw.mover_state()
            traj["mover_xy"].append(torch.stack([m[0][:K] for m in ms_state], 1).cpu().numpy())
            traj["mover_on"].append(torch.stack([m[3][:K] for m in ms_state], 1).cpu().numpy())
            rb = raw.robot_state() or [(pos * 0 + 9.0, None, pos[:, 0] < -1)]  # none: one absent
            traj["robot_xy"].append(torch.stack([b[0][:K] for b in rb], 1).cpu().numpy())
            traj["robot_on"].append(torch.stack([b[2][:K] for b in rb], 1).cpu().numpy())
            route = raw.route.clone()
            if bool((ep_done >= E).all()):
                break
            if step > 20 * raw.max_episode_length * E:
                print("[PLAY] step limit reached")
                break

        total = {k: sum(r[k] for r in by_boxes.values()) for k in keys}
        gaps, rgaps, codes = np.array(gaps), np.array(rgaps), np.array(codes)

        def split_by_gap(g, none_label):
            bins = {none_label: np.isinf(g)}
            edges = (-np.inf, 0.10, 0.25, 0.50, np.inf)  # below 0: centre inside a person
            for lo, hi in zip(edges[:-1], edges[1:], strict=True):
                bins[f"gap {lo:.2f}-{hi:.2f} m"] = (g >= lo) & (g < hi) & ~np.isinf(g)
            out = {}
            for name, sel in bins.items():
                row = {k: int(((codes == i) & sel).sum()) for i, k in enumerate(keys)}
                row["success_rate"] = round(row["success"] / max(int(sel.sum()), 1), 4)
                out[name] = row
            return out

        by_gap = split_by_gap(gaps, "no mover")
        hits = np.array(hits)
        coll = codes == keys.index("collision")
        by_hit = {kind: int((coll & (hits == i)).sum()) for i, kind in enumerate(sae.HIT_KINDS)}
        n_ep = max(sum(total.values()), 1)
        report = {
            "checkpoint": args_cli.checkpoint,
            "num_envs": n,
            "episodes": n_ep,
            "rates": {k: round(v / n_ep, 4) for k, v in total.items()},
            "by_boxes_on_route": {
                b: {**r, "success_rate": round(r["success"] / max(sum(r.values()), 1), 4)}
                for b, r in sorted(by_boxes.items())
            },
            "by_mover_gap": by_gap,
            "by_robot_gap": split_by_gap(rgaps, "no robot"),
            # nearest of walls/shelves, boxes, movers, robots to our robot centre at the collision
            "collisions_by_nearest": by_hit,
            "success_time_s_mean": round(float(np.mean(succ_times)), 2) if succ_times else None,
            "success_time_s_p90": round(float(np.percentile(succ_times, 90)), 2)
            if succ_times
            else None,
            "success_time_over_path_at_vmax_mean": round(float(np.mean(succ_ratio)), 3)
            if succ_ratio
            else None,
            "lidar_min_m": round(lidar_min, 4),
            "spec_overrides": task_over,
            "arena_overrides": arena_over,
        }
        tz = {k: np.stack(v) for k, v in traj.items()}
        tz.update(dt=raw.step_dt, node_xy=raw.graph.xy, node_names=np.array(raw.graph.names))
        # each env's first E episodes: an env keeps running until all envs have E
        rows = timeouts.classify(tz, raw.max_episode_length * raw.step_dt, max_episodes=E)
        # timeouts of the tracked envs by what was near; alone ones are the policy's own
        report["timeouts_tracked"] = {k: sum(r["kind"] == k for r in rows) for k in timeouts.KINDS}
        report["alone_timeouts"] = timeouts.alone_summary(tz, rows, max_episodes=E)
        with open(os.path.join(args_cli.out, "eval.json"), "w", encoding="utf-8") as f:
            json.dump(report, f, indent=1)
        np.savez_compressed(
            os.path.join(args_cli.out, "rollouts.npz"),
            **{k: v for k, v in tz.items() if k != "dt"},
            path=routes.path,
            npts=routes.npts,
            obstacle_xy=routes.obstacle_xy,
            obstacle_sizes=np.array(routes.obstacle_sizes),
            boxes=np.array([(b.x0, b.y0, b.x1, b.y1) for b in raw.boxes]),
            dt=raw.step_dt,
        )
        print("[PLAY]", json.dumps(report))
        env.close()


if __name__ == "__main__":
    main()
