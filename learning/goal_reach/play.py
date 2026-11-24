"""학습한 goal-reach 정책을 평가하고 선택적으로 영상을 녹화한다 (Newton Warp 렌더러, headless).

결정적 정책으로 모든 env 가 --episodes 개를 끝낼 때까지 돌려 결과를 세고, 앞쪽 --track 개 env 의 궤적을
rollouts.npz 로 저장한다. --video 면 env 0 위 카메라와 목표 원판을 더해 video.mp4 (20 fps) 를 쓴다.
--video 에서는 env 마다 카메라가 생기므로 env 를 적게 쓴다.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

import warp as wp

wp.config.enable_backward = False

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--episodes", type=int, default=4, help="Episodes counted per env.")
parser.add_argument("--track", type=int, default=8, help="Envs whose trajectories are saved.")
parser.add_argument("--video", action="store_true")
parser.add_argument("--video_s", type=float, default=40.0, help="Video length in sim seconds.")
parser.add_argument("--res", type=int, default=720)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
add_launcher_args(parser)
args_cli = parser.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

CAM_H = (
    5.0  # env 0 위 높이 m; 시야각 47 deg 로 약 +-2.2 m, 목표 범위에 여유를 더함
)


def main():
    import goal_reach_env as gre

    scene_cls = gre.GoalReachSceneCfg
    if args_cli.video:
        import isaaclab.sim as sim_utils
        from isaaclab.assets import RigidObjectCfg
        from isaaclab.sensors import CameraCfg
        from isaaclab.utils import configclass
        from isaaclab_newton.renderers import NewtonWarpRendererCfg

        @configclass
        class VideoSceneCfg(gre.GoalReachSceneCfg):
            goal = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/Goal",
                spawn=sim_utils.CylinderCfg(
                    radius=0.05,  # = 성공 반경
                    height=0.005,
                    rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                    mass_props=sim_utils.MassCfg(mass=0.1),
                    collision_props=sim_utils.UsdPhysicsCollisionCfg(collision_enabled=False),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 0.0025)),
            )
            camera = CameraCfg(
                prim_path="{ENV_REGEX_NS}/TopCam",
                offset=CameraCfg.OffsetCfg(
                    pos=(0.0, 0.0, CAM_H), rot=(1.0, 0.0, 0.0, 0.0), convention="ros"
                ),
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(
                    focal_length=24.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
                ),
                width=args_cli.res,
                height=args_cli.res,
                renderer_cfg=NewtonWarpRendererCfg(),
            )

        scene_cls = VideoSceneCfg

    env_cfg = gre.GoalReachEnvCfg()
    env_cfg.scene = scene_cls(num_envs=args_cli.num_envs, env_spacing=8.0, replicate_physics=True)
    env_cfg.seed = args_cli.seed
    env_cfg.spec_overrides = gre.ts.parse_overrides(args_cli.set)
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
        raw = gre.GoalReachEnv(env_cfg)
        env = RslRlVecEnvWrapper(raw, clip_actions=agent_cfg.clip_actions)
        runner = create_rsl_rl_runner(env, agent_cfg)
        runner.load(args_cli.checkpoint)
        policy = runner.get_inference_policy(device=raw.device)
        try:
            runner.export_policy_to_onnx(path=args_cli.out, filename="policy.onnx")
        except Exception as e:  # 내보내기는 편의 기능, 평가에는 불필요
            print(f"[PLAY] onnx export failed: {e}")

        n, E = args_cli.num_envs, args_cli.episodes
        ep_done = torch.zeros(n, dtype=torch.long, device=raw.device)
        outcome = {"success": 0, "collision": 0, "oob": 0, "timeout": 0}
        succ_times, final_d, lidar_min = [], [], float("inf")
        K = min(args_cli.track, n)
        traj = {"xy": [], "yaw": [], "goal": [], "done": [], "cmd": []}
        frames_dir = os.path.join(args_cli.out, "frames")
        if args_cli.video:
            os.makedirs(frames_dir, exist_ok=True)
            import PIL.Image
            import PIL.ImageDraw

            goal_obj = raw.scene["goal"]
            cam = raw.scene["camera"]
            px_per_m = args_cli.res * 24.0 / (20.955 * CAM_H)  # 초점 / 조리개 / 높이
            trail = []

            def to_px(xy):  # env 0 바닥 (x, y) -> 픽셀; 오른쪽 = +x, 위 = +y
                return (
                    args_cli.res / 2 + float(xy[0]) * px_per_m,
                    args_cli.res / 2 - float(xy[1]) * px_per_m,
                )

        video_steps = int(args_cli.video_s / raw.step_dt)
        obs = env.get_observations()
        step = 0
        while True:
            with torch.inference_mode():
                obs, _, dones, _ = env.step(policy(obs))
            step += 1
            d = dones.bool()
            lidar_min = min(lidar_min, float(raw.lidar_ranges().min()))
            counted = d & (ep_done < E)
            if counted.any():
                s = raw.success & counted
                c = raw.collided & counted
                o = raw.oob & counted  # 결과는 배타적 (task_spec.episode_outcome)
                t = counted & ~(raw.success | raw.collided | raw.oob)
                outcome["success"] += int(s.sum())
                outcome["collision"] += int(c.sum())
                outcome["oob"] += int(o.sum())
                outcome["timeout"] += int(t.sum())
                # episode_length_buf 는 이미 리셋됨: env 가 끝난 길이를 따로 보관
                succ_times += raw.last_ep_len_s[s].tolist()
                final_d += raw.last_ep_dist[counted].tolist()
            ep_done += d.long()
            pos, yaw = raw.robot_pose()
            traj["xy"].append(pos[:K].cpu().numpy())
            traj["yaw"].append(yaw[:K].cpu().numpy())
            traj["goal"].append(raw.goal_xy[:K].cpu().numpy())
            traj["done"].append(d[:K].cpu().numpy())
            traj["cmd"].append(raw.cmd[:K].cpu().numpy())
            if args_cli.video and step <= video_steps:
                gp = goal_obj.data.default_root_pose.torch.clone()
                gp[:, :2] = raw.goal_xy + raw.scene.env_origins[:, :2]
                gp[:, 2] = 0.0025
                goal_obj.write_root_pose_to_sim_index(root_pose=gp)
                img = PIL.Image.fromarray(cam.data.output["rgb"][0].cpu().numpy()[..., :3])
                # 렌더 프레임에 그림: 에피소드 시작 이후 경로, 시작 십자, 글자
                if d[0]:
                    trail.clear()
                trail.append(to_px(pos[0]))
                draw = PIL.ImageDraw.Draw(img)
                if len(trail) > 1:
                    draw.line(trail, fill=(255, 255, 255), width=2)
                sx, sy = to_px(torch.zeros(2))
                draw.line([(sx - 6, sy - 6), (sx + 6, sy + 6)], fill=(20, 20, 20), width=2)
                draw.line([(sx - 6, sy + 6), (sx + 6, sy - 6)], fill=(20, 20, 20), width=2)
                draw.text(
                    (10, 10),
                    f"t {step * raw.step_dt:5.1f} s   goal dist {float(raw.d_now[0]):.2f} m   "
                    f"v {float(raw.cmd[0, 0]):.2f} m/s  w {float(raw.cmd[0, 1]):+.2f} rad/s   "
                    f"episodes done {int(ep_done[0])}",
                    fill=(0, 0, 0),
                )
                img.save(os.path.join(frames_dir, f"f{step:05d}.png"))
            if bool((ep_done >= E).all()) and (not args_cli.video or step >= video_steps):
                break
            if step > 20 * raw.max_episode_length * E:
                print("[PLAY] step limit reached")
                break

        total = sum(outcome.values())
        res = {
            "checkpoint": args_cli.checkpoint,
            "num_envs": n,
            "episodes": total,
            **{f"{k}_rate": round(v / max(total, 1), 4) for k, v in outcome.items()},
            "success_time_s_mean": round(float(np.mean(succ_times)), 3) if succ_times else None,
            "success_time_s_p95": round(float(np.percentile(succ_times, 95)), 3)
            if succ_times
            else None,
            "final_dist_m_mean": round(float(np.mean(final_d)), 4),
            "lidar_min_seen_m": round(lidar_min, 4),
            "spec_overrides": env_cfg.spec_overrides,
        }
        with open(os.path.join(args_cli.out, "eval.json"), "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
        np.savez_compressed(
            os.path.join(args_cli.out, "rollouts.npz"),
            dt=raw.step_dt,
            success_radius=raw.spec.success_radius,
            **{k: np.stack(v) for k, v in traj.items()},
        )
        print("[PLAY] eval:", json.dumps(res))
        if args_cli.video:
            ff = shutil.which("ffmpeg")
            if ff:
                cmd = [
                    ff,
                    "-y",
                    "-loglevel",
                    "error",
                    "-framerate",
                    str(round(1 / raw.step_dt)),
                    "-i",
                    os.path.join(frames_dir, "f%05d.png"),
                    "-c:v",
                    "libx264",
                    "-pix_fmt",
                    "yuv420p",
                    "-crf",
                    "28",
                    os.path.join(args_cli.out, "video.mp4"),
                ]
                print("[PLAY] ffmpeg exit", subprocess.call(cmd))
            else:
                print("[PLAY] ffmpeg not found; frames are in", frames_dir)
        env.close()


if __name__ == "__main__":
    main()
