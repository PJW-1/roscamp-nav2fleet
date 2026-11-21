"""Phase A checks of the store arena (no policy, scripted actions). One mode per run:

  scene     build the env, report build time and GPU memory, compare every env's lidar scan with
            a 2D ray cast of the yaml boxes; --render saves a top-down Newton Warp image of env 0
  contact   drive robots into walls and shelves; log when the contact sensors / collision flag fire,
            which body, the gap to the obstacle and the lidar minimum at that step
  crossenv  arenas placed on top of each other (--set env_spacing=0.5) with a second robot in each
            env: does env i's lidar see env j's walls or robots, does it see its own second robot
  reset     time out some envs while the robot is far from its start: is the first observation
            after the reset built from the new pose or the old one

Writes <out>/<mode>.json (and scene_topdown.png with --render). Run from D:\\isaac\\IsaacLab:
  .venv/Scripts/python.exe <repo>/learning/store_arena/arena_check.py --mode scene --num_envs 64 \
      --out D:/isaac/runs/store_arena/scene_64 --viz none
"""

import argparse
import dataclasses
import json
import math
import os
import subprocess
import sys
import time

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", choices=("scene", "contact", "crossenv", "reset"), required=True)
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--render", action="store_true", help="scene mode: top-down image of env 0")
parser.add_argument("--res", type=int, default=800)
parser.add_argument(
    "--set", action="append", default=[], metavar="FIELD=VALUE", help="TaskSpec/ArenaSpec field"
)
add_launcher_args(parser)
args_cli = parser.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))
sys.path.insert(0, os.path.join(HERE, "..", "sim_check"))

CAM_H = 2.6  # m above the floor, over the arena centre
ROBOT2_XY = (0.90, 0.55)  # crossenv: second robot in the same env, ahead of the first


def gpu_mib() -> int:
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout
        return int(out.split()[0])
    except Exception:
        return -1


def main():
    import arena_spec as arena
    import store_arena_env as sae

    task_over, arena_over = arena.split_overrides(args_cli.set)
    env_cfg = sae.StoreArenaEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.spec_overrides = task_over
    env_cfg.arena_overrides = arena_over
    spec_a = arena.make_arena_spec(arena_over)

    if args_cli.mode == "crossenv":
        from pinky_cfg import make_pinky_cfg

        env_cfg.scene.robot2 = make_pinky_cfg("{ENV_REGEX_NS}/Robot2")
        env_cfg.scene.robot2.init_state.pos = (*ROBOT2_XY, 0.002)
    if args_cli.mode == "scene" and args_cli.render:
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import CameraCfg
        from isaaclab_newton.renderers import NewtonWarpRendererCfg

        env_cfg.scene.camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/TopCam",
            offset=CameraCfg.OffsetCfg(
                pos=(0.99, 0.49, CAM_H), rot=(1.0, 0.0, 0.0, 0.0), convention="ros"
            ),
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
            ),
            width=args_cli.res,
            height=args_cli.res,
            renderer_cfg=NewtonWarpRendererCfg(),
        )
    os.makedirs(args_cli.out, exist_ok=True)

    with launch_simulation(env_cfg, args_cli):
        import torch

        mib_before = gpu_mib()
        t0 = time.time()
        env = sae.StoreArenaEnv(env_cfg)
        obs0, _ = env.reset()
        build_s = time.time() - t0
        mib_built = gpu_mib()
        rects = arena.boxes_tensor(env.boxes, env.device)
        lim = arena.ts.ACTION_LIMITS
        report = {
            "mode": args_cli.mode,
            "num_envs": env.num_envs,
            "arena": vars(spec_a) if hasattr(spec_a, "__dict__") else str(spec_a),
            "boxes": [b.__dict__ for b in env.boxes],
            "build_s": round(build_s, 2),
            "gpu_mib_before_env": mib_before,
            "gpu_mib_after_build": mib_built,
        }

        def action(v: float, w: float = 0.0) -> torch.Tensor:
            a0 = 2.0 * (v - lim.v_min) / (lim.v_max - lim.v_min) - 1.0
            a = torch.tensor((a0, w / lim.w_max), device=env.device)
            return a.expand(env.num_envs, 2).clone()

        def actions(vs) -> torch.Tensor:
            return torch.cat([action(v)[:1] for v in vs], 0)

        def lidar_center(pos, yaw):  # lidar is 0.017 m behind base_link
            return pos - 0.017 * torch.stack((torch.cos(yaw), torch.sin(yaw)), -1)

        def expected_scan(pos, yaw, grow=0.0):
            r = rects + torch.tensor((-grow, -grow, grow, grow), device=env.device)
            return arena.raycast_rects(lidar_center(pos, yaw), yaw, r)

        def scan_error(sim, pos, yaw, mask=None):
            """Sim lidar vs the 2D ray cast of the yaml boxes. A ray that grazes a box corner can
            hit or miss on a 1 mm pose difference, so a ray also counts as matching when it lies
            within 1 cm of the ray cast with every box grown or shrunk by 2 mm."""
            exp = expected_scan(pos, yaw)
            near, far = expected_scan(pos, yaw, 0.002), expected_scan(pos, yaw, -0.002)
            mask = torch.ones_like(sim, dtype=torch.bool) if mask is None else mask
            fin_s, fin_e = torch.isfinite(sim), torch.isfinite(exp)
            both = fin_s & fin_e & mask
            err = (sim - exp).abs().where(both, torch.zeros_like(sim))
            near, far, s = (t.nan_to_num(posinf=1e9) for t in (near, far, sim))  # miss = 1e9
            lo, hi = torch.minimum(near, far) - 0.01, torch.maximum(near, far) + 0.01
            in_band = (s >= lo) & (s <= hi)
            return {
                "max_abs_err_m": round(float(err.max()), 5),
                "rays_err_gt_1cm": int((err > 0.01).sum()),
                "rays_hit_miss_disagree": int(((fin_s != fin_e) & mask).sum()),
                "rays_outside_2mm_corner_band": int((~in_band & mask).sum()),
                "rays_compared": int(mask.sum()),
            }

        def place(ids, xy, yaw):
            """Teleport robots of envs ids to env-frame (x, y, yaw), at rest, goal far away."""
            ids = torch.as_tensor(ids, device=env.device, dtype=torch.long)
            n = len(ids)
            xy = torch.as_tensor(xy, device=env.device, dtype=torch.float32).reshape(n, 2)
            yaw = torch.as_tensor(yaw, device=env.device, dtype=torch.float32).reshape(n)
            pose = env.robot.data.default_root_pose.torch[ids].clone()
            pose[:, :3] += env.scene.env_origins[ids]
            pose[:, :2] += xy
            z = torch.zeros_like(yaw)
            pose[:, 3:7] = torch.stack((z, z, torch.sin(yaw / 2), torch.cos(yaw / 2)), -1)
            env.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
            env.robot.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros(n, 6, device=env.device), env_ids=ids
            )
            env.start_xy[ids] = xy
            env.goal_xy[ids] = 50.0
            env.cmd[ids] = 0.0

        pose0 = env.robot_pose()  # right after the initial reset, before any step
        mode = args_cli.mode
        if mode == "scene":
            stop = action(0.0)
            for _ in range(5):
                env.step(stop)
            pos, yaw = env.robot_pose()
            sim = env.lidar_ranges()
            report["scan_vs_yaml_all_envs"] = scan_error(sim, pos, yaw)
            report["pass"] = report["scan_vs_yaml_all_envs"]["rays_outside_2mm_corner_band"] == 0
            t1 = time.time()
            steps = 100
            for _ in range(steps):
                env.step(stop)
            torch.cuda.synchronize()
            report["env_steps_per_s"] = round(steps * env.num_envs / (time.time() - t1), 1)
            report["gpu_mib_after_steps"] = gpu_mib()
            report["torch_max_alloc_mib"] = round(torch.cuda.max_memory_allocated() / 2**20, 1)
            report["env0"] = {
                "pos": pos[0].tolist(),
                "yaw": float(yaw[0]),
                "lidar": [float(x) for x in sim[0].tolist()],
                "lidar_expected": [float(x) for x in expected_scan(pos, yaw)[0].tolist()],
            }
            if args_cli.render:
                import numpy as np
                import PIL.Image

                env.step(stop)
                rgb = env.scene["camera"].data.output["rgb"]
                rgb = rgb.torch if hasattr(rgb, "torch") else rgb
                img = rgb[0, ..., :3].cpu().numpy().astype(np.uint8)
                PIL.Image.fromarray(img).save(os.path.join(args_cli.out, "scene_topdown.png"))
                report["camera"] = {"height_m": CAM_H, "centre": [0.99, 0.49], "res": args_cli.res}

        elif mode == "contact":
            # (start xy, yaw, speed, obstacle face the robot heads for: axis, coordinate)
            cases = [
                ("west_wall_forward", (0.30, 0.55), math.pi, 0.25, ("x", 0.0)),
                ("north_wall_forward", (0.30, 0.55), math.pi / 2, 0.25, ("y", 0.98)),
                ("shelfA_south_face_forward", (1.00, 0.55), math.pi / 2, 0.25, ("y", 0.77)),
                ("south_shelf_north_face_forward", (0.85, 0.55), -math.pi / 2, 0.25, ("y", 0.405)),
                ("east_wall_slow", (1.70, 0.55), 0.0, 0.10, ("x", 1.98)),
                ("west_wall_reverse", (0.30, 0.55), 0.0, -0.05, ("x", 0.0)),
                ("west_wall_diag45", (0.30, 0.40), 3 * math.pi / 4, 0.25, ("x", 0.0)),
                ("south_wall_forward", (1.10, 0.20), -math.pi / 2, 0.25, ("y", 0.0)),
            ]
            n = env.num_envs
            assert n >= len(cases), f"contact mode needs --num_envs >= {len(cases)}"
            m = len(cases)
            ids = list(range(m))
            place(ids, [c[1] for c in cases], [c[2] for c in cases])
            speeds = [c[3] for c in cases] + [0.0] * (n - m)
            act = actions(speeds)
            cb, cp = env.scene["contact_base"], env.scene["contact_parts"]
            names = list(cb.sensor_names) + list(cp.sensor_names)
            report["contact_bodies"] = names
            first = {}
            rows = []
            done_once = torch.zeros(n, dtype=torch.bool, device=env.device)
            for k in range(200):  # 10 s
                pos_prev, yaw_prev = env.robot_pose()
                lid_prev = env.lidar_ranges().clone()
                obs, rew, term, trunc, extras = env.step(act)
                # the step's dones were computed before the reset: env.collided is that step's flag
                f = torch.cat(
                    (
                        cb.data.net_forces_w.torch.norm(dim=-1),
                        cp.data.net_forces_w.torch.norm(dim=-1),
                    ),
                    -1,
                )  # after reset of collided envs these are zero; keep pre-reset logs below
                coll = env.collided.clone()
                for i, c in enumerate(cases):
                    if done_once[i]:
                        continue
                    if coll[i] or term[i] or trunc[i]:
                        done_once[i] = True
                        first[c[0]] = {
                            "step": k + 1,
                            "collided": bool(coll[i]),
                            "oob": bool(env.oob[i]),
                        }
                rows.append(
                    (
                        k,
                        pos_prev[:m].tolist(),
                        yaw_prev[:m].tolist(),
                        lid_prev[:m].min(-1).values.tolist(),
                        f[:m].tolist(),
                    )
                )
                if done_once[:m].all():
                    break
            # per case: last pre-collision state and the force profile seen before the reset
            summary = {}
            for i, c in enumerate(cases):
                axis, face = c[4]
                ax = 0 if axis == "x" else 1
                end = first.get(c[0], {"step": 1e9})["step"]
                hist = [(r[0], r[1][i], r[3][i], r[4][i]) for r in rows if r[0] < end]
                if not hist:
                    continue
                k_last, p_last, lid_last, _ = hist[-1]
                summary[c[0]] = {
                    "result": first.get(c[0]),
                    "start": list(c[1]),
                    "yaw_deg": round(math.degrees(c[2]), 1),
                    "speed": c[3],
                    "last_pose_before_end": [round(x, 4) for x in p_last],
                    "centre_to_face_m_before_end": round(abs(p_last[ax] - face), 4),
                    "lidar_min_m_before_end": round(lid_last, 4),
                }
            report["cases"] = summary
            report["pass"] = all(first.get(c[0], {}).get("collided") for c in cases)
            contact_pass = report["pass"]
            # forces at the collision step are read inside env._get_dones; log them via a second
            # pass that does not terminate (collision_force huge) to see which body touches first
            report["note"] = "see contact_push.json for the non-terminating pass"
            with open(os.path.join(args_cli.out, "contact.json"), "w", encoding="utf-8") as fh:
                json.dump(report, fh, indent=1)

            # non-terminating pass: same cases, collision disabled, log forces per body per step
            env.spec = dataclasses.replace(env.spec, collision_force=1e9)
            env.reset()
            place(ids, [c[1] for c in cases], [c[2] for c in cases])
            push = {c[0]: [] for c in cases}
            for k in range(160):  # 8 s
                obs, rew, term, trunc, extras = env.step(act)
                pos, yaw = env.robot_pose()
                lid = env.lidar_ranges()
                f = torch.cat(
                    (
                        cb.data.net_forces_w.torch.norm(dim=-1),
                        cp.data.net_forces_w.torch.norm(dim=-1),
                    ),
                    -1,
                )
                for i, c in enumerate(cases):
                    axis, face = c[4]
                    ax = 0 if axis == "x" else 1
                    push[c[0]].append(
                        {
                            "t": round((k + 1) * env.step_dt, 2),
                            "centre_to_face_m": round(abs(float(pos[i, ax]) - face), 4),
                            "lidar_min_m": round(float(lid[i].min()), 4),
                            "forces_N": {
                                nm: round(float(x), 3) for nm, x in zip(names, f[i], strict=True)
                            },
                            "vx": round(float(env.body_velocity()[i, 0]), 4),
                        }
                    )
            report = {"contact_bodies": names, "cases": push}
            with open(os.path.join(args_cli.out, "contact_push.json"), "w", encoding="utf-8") as fh:
                json.dump(report, fh)
            print(f"[CHECK] contact done, pass={contact_pass}")
            if not contact_pass:
                sys.exit(1)
            return

        elif mode == "crossenv":
            robot2 = env.scene["robot2"]
            n = env.num_envs
            stop = action(0.0)
            place(list(range(n)), [(spec_a.start_x, spec_a.start_y)] * n, [0.0] * n)
            for _ in range(10):
                env.step(stop)
            pos, yaw = env.robot_pose()
            sim = env.lidar_ranges()
            # rays that do not point at the second robot (it is not in the yaml)
            ang = yaw[:, None] + arena.ray_angles(device=env.device)[None, :]
            r2 = robot2.data.root_link_pos_w.torch[:, :2] - env.scene.env_origins[:, :2]
            bearing = torch.atan2(r2[:, 1] - pos[:, 1], r2[:, 0] - pos[:, 0])
            off = torch.atan2(torch.sin(ang - bearing[:, None]), torch.cos(ang - bearing[:, None]))
            away = off.abs() > math.radians(20)
            report["origins_first4"] = env.scene.env_origins[:4].tolist()
            report["scan_vs_yaml_rays_away_from_robot2"] = scan_error(sim, pos, yaw, away)
            # power of the test: rays that would change if env i saw the other envs' boxes and
            # robots (robots drawn as 0.1 m squares), at this spacing
            org = env.scene.env_origins[:, :2]
            sq = torch.tensor((-0.05, -0.05, 0.05, 0.05), device=env.device)
            leak = 0
            for i in range(n):
                shift = (org - org[i]).repeat(1, 2)  # (n, 4)
                others = torch.arange(n, device=env.device) != i
                world = torch.cat(
                    (
                        (rects[None] + shift[:, None]).reshape(-1, 4),
                        (pos.repeat(1, 2) + sq + shift)[others],
                        (r2.repeat(1, 2) + sq + shift)[others],
                    )
                )
                c = lidar_center(pos[i : i + 1], yaw[i : i + 1])
                seen = arena.raycast_rects(c, yaw[i : i + 1], world)
                own = arena.raycast_rects(c, yaw[i : i + 1], rects)
                diff = (seen.nan_to_num(posinf=9.0) - own.nan_to_num(posinf=9.0)).abs() > 0.01
                leak += int((diff & away[i : i + 1]).sum())
            report["rays_that_would_differ_if_envs_leaked"] = leak
            fwd = sim[:, 180]  # ray 180 = straight ahead, at the second robot
            report["robot2_ahead_range_m"] = {
                "mean": round(float(fwd.mean()), 4),
                "min": round(float(fwd.min()), 4),
                "max": round(float(fwd.max()), 4),
                "centre_gap_m": round(float((r2[:, 0] - pos[:, 0]).mean()), 4),
            }
            # move the second robot 0.2 m further (teleport): does the lidar follow it?
            p2 = robot2.data.root_link_pose_w.torch.clone()
            p2[:, 0] += 0.2
            robot2.write_root_pose_to_sim_index(root_pose=p2)
            robot2.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros(n, 6, device=env.device)
            )
            for _ in range(5):
                env.step(stop)
            fwd2 = env.lidar_ranges()[:, 180]
            report["robot2_after_move_0p2_range_m"] = {
                "mean": round(float(fwd2.mean()), 4),
                "delta_mean": round(float((fwd2 - fwd).mean()), 4),
            }
            z = env.robot.data.root_link_pos_w.torch[:, 2]
            report["robot_z_range_m"] = [round(float(z.min()), 4), round(float(z.max()), 4)]
            drift = (env.robot_pose()[0] - pos).norm(dim=-1)
            report["robot_xy_drift_max_m"] = round(float(drift.max()), 5)
            report["pass"] = (
                report["scan_vs_yaml_rays_away_from_robot2"]["rays_outside_2mm_corner_band"] == 0
                and abs(report["robot2_after_move_0p2_range_m"]["delta_mean"] - 0.2) < 0.005
                and float(fwd.max()) < 1.0
            )

        elif mode == "reset":
            n = env.num_envs
            half = n // 2
            # drive east along the middle aisle for 2 s, then time out the first half of the envs
            place(list(range(n)), [(spec_a.start_x, spec_a.start_y)] * n, [0.0] * n)
            go = action(0.25)
            for _ in range(40):
                env.step(go)
            pos_old, yaw_old = env.robot_pose()
            env.episode_length_buf[:half] = env.max_episode_length - 1
            obs, rew, term, trunc, extras = env.step(go)
            assert bool(trunc[:half].all()) and not bool(trunc[half:].any())
            pos_new, yaw_new = env.robot_pose()
            spec_t = env.spec
            sectors = obs["policy"][:, : arena.ts.OBS_LIDAR] * spec_t.lidar_obs_max

            def sec(r):
                return arena.ts.lidar_sectors(r, spec_t)

            def sector_misses(obs_sec, pos, yaw):
                """(envs, sectors) of the observation that are more than 1 cm outside the ray
                cast of the yaml boxes grown and shrunk by 2 mm (as scan_error: a ray grazing a
                thin divider top moves a sector minimum by up to its whole range)."""
                a, b = sec(expected_scan(pos, yaw, 0.002)), sec(expected_scan(pos, yaw, -0.002))
                lo, hi = torch.minimum(a, b) - 0.01, torch.maximum(a, b) + 0.01
                bad = (obs_sec < lo) | (obs_sec > hi)
                return {"envs": int(bad.any(-1).sum()), "sectors": int(bad.sum())}

            rs, run = slice(0, half), slice(half, n)
            report["reset_envs"] = half
            report["moved_m"] = round(float((pos_new[rs] - pos_old[rs]).norm(dim=-1).mean()), 3)
            report["first_obs_off_vs_new_pose"] = sector_misses(
                sectors[rs], pos_new[rs], yaw_new[rs]
            )
            report["first_obs_off_vs_old_pose"] = sector_misses(
                sectors[rs], pos_old[rs], yaw_old[rs]
            )
            # envs that kept going: their obs should match their current pose
            report["running_envs_obs_off"] = sector_misses(sectors[run], pos_new[run], yaw_new[run])
            obs2, *_ = env.step(action(0.0))
            pos2, yaw2 = env.robot_pose()
            s2 = obs2["policy"][:, : arena.ts.OBS_LIDAR] * spec_t.lidar_obs_max
            report["second_obs_off"] = sector_misses(s2, pos2, yaw2)
            # the observation returned by the initial env.reset() (pose written, no step yet)
            s0 = obs0["policy"][:, : arena.ts.OBS_LIDAR] * spec_t.lidar_obs_max
            report["initial_reset_obs_off"] = sector_misses(s0, *pose0)
            report["pass"] = (
                report["first_obs_off_vs_new_pose"]["envs"] == 0
                and report["initial_reset_obs_off"]["envs"] == 0
                and report["running_envs_obs_off"]["envs"] == 0
                and report["second_obs_off"]["envs"] == 0
                # the test can tell: (nearly) every reset env's obs differs from the old pose
                and report["first_obs_off_vs_old_pose"]["envs"] >= 0.9 * half
            )

        report["gpu_mib_end"] = gpu_mib()
        with open(os.path.join(args_cli.out, f"{mode}.json"), "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1)
        print(json.dumps({k: v for k, v in report.items() if k not in ("boxes", "env0")}, indent=1))
        print(f"[CHECK] {mode} done, pass={report.get('pass')}")
        if report.get("pass") is False:
            sys.exit(1)


if __name__ == "__main__":
    main()
