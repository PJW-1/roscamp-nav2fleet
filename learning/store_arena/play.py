"""학습한 매장 아레나 정책을 평가한다 (결정적, headless).

모든 env 가 --episodes 개를 끝낼 때까지 돌려 결과를 세고 박스 수, mover 접근 거리별로도 나눈다.
앞쪽 --track 개 env 의 궤적은 rollouts.npz 로 저장한다 (plot_arena.py traj / anim).
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
        )  # 경로별
        n, E = args_cli.num_envs, args_cli.episodes
        ep_done = torch.zeros(n, dtype=torch.long, device=raw.device)
        keys = ("success", "collision", "oob", "timeout")
        by_boxes = {}  # 경로의 박스 수 -> 결과 개수
        succ_times, succ_ratio, lidar_min = [], [], float("inf")
        K = min(args_cli.track, n)
        traj = {"xy": [], "done": [], "route": [], "cmd": [], "mover_xy": [], "mover_on": []}
        traj.update(robot_xy=[], robot_on=[], outcome=[])  # outcome: 끝난 스텝에 키 인덱스, 아니면 -1
        traj.update(front_m=[])  # 정면 30 deg 이내 최소 라이다 거리
        n_rays = raw.lidar_ranges().shape[1]  # 레이는 -180 deg 부터: 정면은 n_rays // 2
        front = slice(n_rays // 2 - n_rays // 12, n_rays // 2 + n_rays // 12 + 1)
        # 센 에피소드별: 최소 mover 간격, 최소 로봇 간격, 결과 인덱스, 충돌 원인
        gaps, rgaps, codes, hits = [], [], [], []
        grazes, corners = [], []  # 센 에피소드별: 스침 충돌만 여부, 코너 시작 여부
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
                grazes += raw.graze_only[counted].tolist()
                corners += raw.last_ep_corner[counted].tolist()
                code = torch.stack(outs, -1).float().argmax(-1)
                codes += code[counted].tolist()
                s = raw.success & counted
                succ_times += raw.last_ep_len_s[s].tolist()
                # 에피소드 시간 vs 최고속도로 가는 전역 경로 시간
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
            rb = raw.robot_state() or [(pos * 0 + 9.0, None, pos[:, 0] < -1)]  # 없으면 부재 하나
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
            edges = (-np.inf, 0.10, 0.25, 0.50, np.inf)  # 0 미만: 중심이 사람 안
            for lo, hi in zip(edges[:-1], edges[1:], strict=True):
                bins[f"gap {lo:.2f}-{hi:.2f} m"] = (g >= lo) & (g < hi) & ~np.isinf(g)
            out = {}
            for name, sel in bins.items():
                row = {k: int(((codes == i) & sel).sum()) for i, k in enumerate(keys)}
                row["success_rate"] = round(row["success"] / max(int(sel.sum()), 1), 4)
                out[name] = row
            return out

        by_gap = split_by_gap(gaps, "no mover")
        grazes, corners = np.array(grazes, dtype=bool), np.array(corners, dtype=bool)

        def rates_of(sel):
            m = max(int(sel.sum()), 1)
            return {"episodes": int(sel.sum()),
                    **{k: round(float(((codes == i) & sel).sum()) / m, 4)
                       for i, k in enumerate(keys)}}
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
            # 충돌 순간 로봇 중심에 가장 가까운 것 (벽/선반, 박스, mover, 로봇)
            "collisions_by_nearest": by_hit,
            # 접촉 센서가 놓친 충돌: 발자국 여유 <= graze_m
            "collisions_graze_only": int(grazes.sum()),
            # 경로 반대쪽을 보고 벽 근처에서 시작한 에피소드 (corner_start_prob)
            "by_start": {"corner": rates_of(corners), "other": rates_of(~corners)},
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
        # env 마다 첫 E 개 에피소드: env 는 모두가 E 개가 될 때까지 계속 돎
        rows = timeouts.classify(tz, raw.max_episode_length * raw.step_dt, max_episodes=E)
        # 추적 env 의 시간초과를 주변 상황별로: alone 은 정책 자체 문제
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
