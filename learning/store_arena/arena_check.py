"""Phase A checks of the store arena (no policy, scripted actions). One mode per run:

  scene     build the env, report build time and GPU memory, compare every env's lidar scan with
            a 2D ray cast of the yaml boxes; --render saves a top-down Newton Warp image of env 0
  contact   drive robots into walls and shelves; log when the contact sensors / collision flag fire,
            which body, the gap to the obstacle and the lidar minimum at that step
  crossenv  arenas placed on top of each other (--set env_spacing=0.5) with a second robot in each
            env: does env i's lidar see env j's walls or robots, does it see its own second robot
  reset     time out some envs while the robot is far from its start: is the first observation
            after the reset built from the new pose (and the new obstacle boxes) or the old one
  obstacle  drive robots into the obstacle boxes of their route: does the collision fire, does the
            box stay where it was put (kinematic), does the lidar see it

The expected scans include each env's obstacle boxes. contact and crossenv place robots by hand,
so run them with --set obstacle_max=0. scene, contact, crossenv and reset park the movers
(mover_prob 0) unless --set mover_prob is given: their expected scans have no movers.
  mover     movers follow their legs (sim pose vs mover_spec), the lidar sees the person, and
            driving into a mover that has stopped is a collision
  robot     scripted fleet robots (robot_max 2, movers parked) follow their routes (sim pose vs
            robot_spec), the lidar sees them, the intent mask counts them (with --set
            intent_robots=2), and driving into a robot that is held still is a collision put down
            to a robot

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
parser.add_argument(
    "--mode",
    choices=("scene", "contact", "crossenv", "reset", "obstacle", "mover", "robot"),
    required=True,
)
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
# obstacle/mover drives: half-width of the strip that must be free between robot and target.
# Robot half-width 0.056 m; 0.07 let 3 of 190 robots start 1-2 cm from a divider or wall
STRIP_HALF_M = 0.09


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
    if args_cli.mode in ("scene", "contact", "crossenv", "reset"):
        # their expected scans have no movers: movers stay parked unless asked for explicitly
        arena_over.setdefault("mover_prob", 0.0)
    if args_cli.mode == "robot":
        arena_over.setdefault("robot_max", 2)
        task_over.setdefault("intent_robots", 2)  # so the present flags are checked
        arena_over.setdefault("mover_prob", 0.0)  # the expected scans have no movers
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
        obst0 = env.obstacle_xy().clone()  # obstacle boxes of the initial reset
        sides = torch.tensor(env.routes.obstacle_sizes, device=env.device)
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

        def world_rects(ids=slice(None), obst=None):
            """(n, B + slots, 4): yaml boxes plus each env's obstacle boxes (parked included)."""
            xy = (env.obstacle_xy() if obst is None else obst)[ids]
            half = 0.5 * sides[None, :, None]
            boxes = torch.cat((xy - half, xy + half), -1)
            return torch.cat((rects.expand(len(xy), -1, -1), boxes), 1)

        def expected_scan(pos, yaw, grow=0.0, ids=slice(None), obst=None):
            g = torch.tensor((-grow, -grow, grow, grow), device=env.device)
            return arena.raycast_rects(lidar_center(pos, yaw), yaw, world_rects(ids, obst) + g)

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
            # miss = 1e9. The sim must equal one of the three casts (a corner hit or miss), not
            # anything between them. Grown/shrunk alone is not enough: at shallow incidence 2 mm
            # moves a hit by more than 1 cm.
            ex, near, far, s = (t.nan_to_num(posinf=1e9) for t in (exp, near, far, sim))
            in_band = ((s - ex).abs() <= 0.01) | ((s - near).abs() <= 0.01)
            in_band |= (s - far).abs() <= 0.01
            in_band &= ~torch.isnan(sim)
            off = (~in_band & mask).nonzero()[:10]
            sample = [
                {
                    "env": int(e),
                    "ray": int(r),
                    "pose": [round(float(v), 4) for v in (*pos[e], yaw[e])],
                    "sim": round(float(sim[e, r]), 4),
                    "exact": round(float(exp[e, r]), 4),
                    "grown": round(float(near[e, r]), 4),
                    "shrunk": round(float(far[e, r]), 4),
                }
                for e, r in off.tolist()
            ]
            return {
                "max_abs_err_m": round(float(err.max()), 5),
                "rays_err_gt_1cm": int((err > 0.01).sum()),
                "rays_hit_miss_disagree": int(((fin_s != fin_e) & mask).sum()),
                "rays_off_both_2mm_casts": int((~in_band & mask).sum()),
                "rays_compared": int(mask.sum()),
                "off_rays_sample": sample,
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
            err_s = report["scan_vs_yaml_all_envs"]
            # tolerance: at most 0.01 % of rays may match none of the three casts (grazing rays;
            # a missing or moved box changes far more rays than that)
            report["pass"] = err_s["rays_off_both_2mm_casts"] <= 1e-4 * err_s["rays_compared"]
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
            # every drive ends in a collision, and only once it has reached its wall or shelf
            report["pass"] = all(
                first.get(c[0], {}).get("collided")
                and summary.get(c[0], {}).get("centre_to_face_m_before_end", 1.0) < 0.10
                for c in cases
            )
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
            # robot2 (about 0.6 m ahead, under 0.1 m half-size) spans less than +-10 deg
            away = off.abs() > math.radians(12)
            report["origins_first4"] = env.scene.env_origins[:4].tolist()
            report["scan_vs_yaml_rays_not_at_robot2"] = scan_error(sim, pos, yaw, away)
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
                "delta_max_err_vs_0p2": round(float((fwd2 - fwd - 0.2).abs().max()), 4),
            }
            z = env.robot.data.root_link_pos_w.torch[:, 2]
            report["robot_z_range_m"] = [round(float(z.min()), 4), round(float(z.max()), 4)]
            drift = (env.robot_pose()[0] - pos).norm(dim=-1)
            report["robot_xy_drift_max_m"] = round(float(drift.max()), 5)
            report["pass"] = (
                report["scan_vs_yaml_rays_not_at_robot2"]["rays_off_both_2mm_casts"] == 0
                and float((fwd2 - fwd - 0.2).abs().max()) < 0.005  # every env sees its robot2
                and float(fwd.max()) < 1.0
                and leak > 0  # the arenas overlap, so a leak would show
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
            obst_old = env.obstacle_xy().clone()
            env.episode_length_buf[:half] = env.max_episode_length - 1
            obs, rew, term, trunc, extras = env.step(go)
            assert bool(trunc[:half].all()) and not bool(trunc[half:].any())
            pos_new, yaw_new = env.robot_pose()
            spec_t = env.spec
            sectors = obs["policy"][:, : arena.ts.OBS_LIDAR] * spec_t.lidar_obs_max

            def sec(r):
                return arena.ts.lidar_sectors(r, spec_t)

            def sector_misses(obs_sec, pos, yaw, ids=slice(None), obst=None):
                """(envs, sectors) of the observation that are more than 1 cm outside the ray
                cast of the yaml boxes grown and shrunk by 2 mm (as scan_error: a ray grazing a
                thin divider top moves a sector minimum by up to its whole range)."""
                a = sec(expected_scan(pos, yaw, 0.002, ids, obst))
                b = sec(expected_scan(pos, yaw, -0.002, ids, obst))
                e = sec(expected_scan(pos, yaw, 0.0, ids, obst))
                bad = ((obs_sec - a).abs() > 0.01) & ((obs_sec - b).abs() > 0.01)
                bad &= (obs_sec - e).abs() > 0.01
                bad |= ~torch.isfinite(obs_sec)  # a NaN would pass every comparison above
                return {"envs": int(bad.any(-1).sum()), "sectors": int(bad.sum())}

            rs, run = slice(0, half), slice(half, n)
            report["reset_envs"] = half
            report["moved_m"] = round(float((pos_new[rs] - pos_old[rs]).norm(dim=-1).mean()), 3)
            report["first_obs_off_vs_new_pose"] = sector_misses(
                sectors[rs], pos_new[rs], yaw_new[rs], rs
            )
            report["first_obs_off_vs_old_pose"] = sector_misses(
                sectors[rs], pos_old[rs], yaw_old[rs], rs, obst_old
            )
            # envs that kept going: their obs should match their current pose
            report["running_envs_obs_off"] = sector_misses(
                sectors[run], pos_new[run], yaw_new[run], run
            )
            moved = (env.obstacle_xy()[rs] - obst_old[rs]).norm(dim=-1).amax(-1) > 0.01
            report["reset_envs_whose_obstacles_moved"] = int(moved.sum())
            obs2, *_ = env.step(action(0.0))
            pos2, yaw2 = env.robot_pose()
            s2 = obs2["policy"][:, : arena.ts.OBS_LIDAR] * spec_t.lidar_obs_max
            report["second_obs_off"] = sector_misses(s2, pos2, yaw2)
            # the observation returned by the initial env.reset() (pose written, no step yet)
            s0 = obs0["policy"][:, : arena.ts.OBS_LIDAR] * spec_t.lidar_obs_max
            report["initial_reset_obs_off"] = sector_misses(s0, *pose0, obst=obst0)
            report["pass"] = (
                report["first_obs_off_vs_new_pose"]["envs"] == 0
                and report["initial_reset_obs_off"]["envs"] == 0
                and report["running_envs_obs_off"]["envs"] == 0
                and report["second_obs_off"]["sectors"] <= 1e-4 * n * arena.ts.OBS_LIDAR
                # the test can tell: (nearly) every reset env's obs differs from the old pose
                and report["first_obs_off_vs_old_pose"]["envs"] >= 0.9 * half
            )

        elif mode == "mover":
            import mover_spec as ms_
            import numpy as np
            import route_spec as rs_

            n = env.num_envs
            stop = action(0.0)
            err = 0.0
            seen = {m.name: [] for m in ms_.MOVERS}  # (sim range, nearest, farthest possible)
            org = env.scene.env_origins[:, :2]
            hit_standing = 0
            for k in range(100):  # 5 s standing; movers walk, but wait for a robot in their way
                env.step(stop)
                hit_standing += int(env.collided.sum())
                for obj, (pos, _, _, _) in zip(env.mover_objs, env.mover_state(), strict=True):
                    sim_xy = obj.data.root_pos_w.torch[:, :2] - org
                    err = max(err, float((sim_xy - pos).norm(dim=-1).max()))
                if k % 10 == 9:  # lidar ray toward each mover vs the distance to its surface
                    rp, ryaw = env.robot_pose()
                    c = lidar_center(rp, ryaw)
                    st = env.mover_state()
                    rows = torch.arange(n, device=env.device)
                    lid = env.lidar_ranges()
                    exp_all = expected_scan(rp, ryaw)  # walls, shelves, boxes
                    for m, shape in enumerate(ms_.MOVERS):
                        pos, _, _, act = st[m]
                        opos, _, _, oact = st[1 - m]
                        d = (pos - c).norm(dim=-1)
                        bearing = torch.atan2(pos[:, 1] - c[:, 1], pos[:, 0] - c[:, 0]) - ryaw
                        i = torch.remainder(torch.round(torch.rad2deg(bearing)) + 180, 360).long()
                        other_far = ~oact | ((opos - c).norm(dim=-1) > d + 0.2)
                        ok = act & (exp_all[rows, i] > d + 0.1) & other_far & (d < 1.8)
                        if shape.kind == "cylinder":
                            near, far = d - shape.size[0], d - shape.size[0]
                        else:  # a ray at the centre meets a box face between these distances
                            near, far = d - shape.radius, d - 0.5 * min(shape.size[:2])
                        seen[shape.name].append(
                            torch.stack((lid[rows, i][ok], near[ok], far[ok]), -1)
                        )
            # the ray at the nearest whole degree passes up to 0.5 deg off the centre: at 1.8 m
            # that is 1.6 cm sideways, which lengthens the hit on a 4 cm radius by up to 0.4 cm
            report["mover_pose_err_max_m"] = round(err, 6)
            report["standing_robots_hit_by_movers"] = hit_standing
            ray_ok = True
            for name, rows_ in seen.items():
                t_ = torch.cat(rows_)
                out = (t_[:, 1] - t_[:, 0]).clamp(min=0) + (t_[:, 0] - t_[:, 2]).clamp(min=0)
                report[f"{name}_rays_compared"] = len(t_)
                if len(t_):
                    report[f"{name}_ray_outside_band_m"] = {
                        "max": round(float(out.max()), 4),
                        "p99": round(float(out.quantile(0.99)), 4),
                    }
                ray_ok &= len(t_) > 0 and float(out.quantile(0.99)) < 0.01
            act = [legs["active"].float().mean().item() for legs in env.mover_legs]
            report["mover_active_share"] = dict(
                zip([m.name for m in ms_.MOVERS], [round(a, 3) for a in act], strict=True)
            )
            # collision: stop every mover where it is, put each robot 0.20 m from the person's or
            # (if absent) the cart's surface facing it with a free strip, drive at 0.25 m/s
            states = env.mover_state()
            for legs, (pos, _, _, _) in zip(env.mover_legs, states, strict=True):
                legs["a"], legs["b"] = pos.clone(), pos.clone()
                legs["speed"].zero_()
            static = rects.cpu().numpy()
            obst = env.obstacle_xy().cpu().numpy()
            sz = np.array(env.routes.obstacle_sizes)
            ids, starts, yaws, target = [], [], [], []
            for e in range(n):
                k = 0 if bool(states[0][3][e]) else 1 if bool(states[1][3][e]) else -1
                if k < 0:
                    continue
                centre = states[k][0][e].cpu().numpy()
                r_m = ms_.MOVERS[k].radius
                world = list(static) + [rs_.box_rect(obst[e, j], sz[j]) for j in range(len(sz))]
                other = 1 - k
                if bool(states[other][3][e]):
                    world.append(rs_.box_rect(states[other][0][e].cpu().numpy(), 0.2))
                world = np.array(world)
                for ang in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    u = np.array((math.cos(ang), math.sin(ang)))
                    pnt = centre - u * (r_m + 0.20)
                    a_, b_ = pnt - u * 0.09, centre - u * r_m
                    w = STRIP_HALF_M * np.abs(u[::-1])
                    strip = np.concatenate((np.minimum(a_, b_) - w, np.maximum(a_, b_) + w))
                    if not rs_.rects_overlap(strip, world):
                        ids.append(e)
                        starts.append(pnt)
                        yaws.append(ang)
                        target.append(k)
                        break
            n_with_mover = int((states[0][3] | states[1][3]).sum())
            report["envs_with_a_mover"] = n_with_mover
            report["envs_driven"] = len(ids)
            place(ids, np.array(starts), yaws)
            tested = torch.zeros(n, dtype=torch.bool, device=env.device)
            tested[ids] = True
            act_ = torch.where(tested[:, None], action(0.25), stop)
            first = torch.full((n,), -1, dtype=torch.long, device=env.device)
            gap_at = torch.full((n,), float("nan"), device=env.device)
            for k in range(60):
                rp, _ = env.robot_pose()
                g = env.mover_gaps(rp)
                env.step(act_)
                hit = env.collided & tested & (first < 0)
                first[hit] = k + 1
                gap_at[hit] = g[hit]
            t_ids = torch.tensor(ids, device=env.device, dtype=torch.long)
            ok = first[t_ids] > 0
            g = gap_at[t_ids][ok]
            report["collided"] = int(ok.sum())
            report["targets"] = {"person": target.count(0), "cart": target.count(1)}
            if len(g):
                report["centre_to_mover_surface_before_collision_m"] = [
                    round(float(g.min()), 4),
                    round(float(g.max()), 4),
                ]
            report["pass"] = (
                err < 1e-3
                and ray_ok
                and hit_standing == 0
                and len(ids) >= 0.8 * n_with_mover
                and min(target.count(0), target.count(1)) > 0
                and bool(ok.all())
                and len(g) > 0
                and float(g.max()) < 0.10
            )

        elif mode == "obstacle":
            import numpy as np
            import route_spec as rs_

            n = env.num_envs
            stop = action(0.0)
            for _ in range(5):
                env.step(stop)
            xy0 = env.obstacle_xy().clone()
            for _ in range(100):  # 5 s standing: kinematic, gravity-free boxes must not move
                env.step(stop)
            xy1 = env.obstacle_xy()
            used = (xy1[..., 1] > 0.0).cpu().numpy()  # parked boxes wait at y = -0.4
            report["box_drift_standing_max_m"] = round(float((xy1 - xy0).norm(dim=-1).max()), 6)
            want = torch.as_tensor(env.r_obstacle, device=env.device)[env.route]
            report["box_vs_route_layout_max_m"] = round(float((xy1 - want).norm(dim=-1).max()), 6)
            # one robot per env with a box: 0.20 m from the face of its biggest box, facing it, with
            # a free strip in between; the collision must fire within 0.10 m of the face
            static = rects.cpu().numpy()
            xyn = xy1.cpu().numpy()
            sz = np.array(env.routes.obstacle_sizes)
            ids, starts, yaws, faces = [], [], [], []
            for e in range(n):
                ks = [k for k in np.argsort(-sz) if used[e, k]]
                if not ks:
                    continue
                k = ks[0]
                js = [j for j in range(len(sz)) if used[e, j]]
                others = [rs_.box_rect(xyn[e, j], sz[j]) for j in js if j != k]
                world = np.array(list(static) + others)
                for ang in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    u = np.array((math.cos(ang), math.sin(ang)))
                    p = xyn[e, k] - u * (0.5 * sz[k] + 0.20)
                    f = xyn[e, k] - u * 0.5 * sz[k]
                    # the strip the robot sweeps from its start (rear corner 0.085 m behind the
                    # centre) to the box face, STRIP_HALF_M to each side, must be free
                    a, b = p - u * 0.09, f
                    lo, hi = np.minimum(a, b) - STRIP_HALF_M * np.abs(u[::-1]), np.maximum(a, b)
                    hi = hi + STRIP_HALF_M * np.abs(u[::-1])
                    if not rs_.rects_overlap(np.concatenate((lo, hi)), world):
                        ids.append(e)
                        starts.append(p)
                        yaws.append(ang)
                        faces.append((0 if abs(u[0]) > 0.5 else 1, f))
                        break
            report["envs_with_a_box"] = int(used.any(1).sum())
            report["envs_driven"] = len(ids)
            tested = torch.zeros(n, dtype=torch.bool, device=env.device)
            tested[ids] = True
            env.reset()
            # the reset drew new routes: put back the boxes this check measured
            for k, obj in enumerate(env.obstacles):
                pose = obj.data.root_pose_w.torch.clone()
                pose[:, :2] = xy1[:, k] + env.scene.env_origins[:, :2]
                obj.write_root_pose_to_sim_index(root_pose=pose)
            place(ids, np.array(starts), yaws)
            back = (env.obstacle_xy() - xy1).norm(dim=-1).max()
            report["boxes_put_back_max_err_m"] = round(float(back), 6)
            act = torch.where(tested[:, None], action(0.25), stop)
            first = torch.full((n,), -1, dtype=torch.long, device=env.device)
            gap = torch.full((n,), float("nan"), device=env.device)
            ax = torch.zeros(n, dtype=torch.long, device=env.device)
            face = torch.zeros(n, device=env.device)
            for e, (a_, f_) in zip(ids, faces, strict=True):
                ax[e] = a_
                face[e] = float(f_[a_])
            rows = torch.arange(n, device=env.device)
            for k in range(60):
                pos_prev, _ = env.robot_pose()
                env.step(act)
                hit = env.collided & tested & (first < 0)
                first[hit] = k + 1
                gap[hit] = (pos_prev[rows, ax] - face).abs()[hit]
            t_ids = torch.tensor(ids, device=env.device, dtype=torch.long)
            ok = first[t_ids] > 0
            g = gap[t_ids][ok]
            report["collided"] = int(ok.sum())
            if len(g):  # empty when no drive collided: still write the failing report
                report["first_collision_step"] = [int(first[t_ids].min()), int(first[t_ids].max())]
                report["centre_to_box_face_before_collision_m"] = [
                    round(float(g.min()), 4),
                    round(float(g.max()), 4),
                ]
            report["per_env"] = [
                {
                    "env": e,
                    "step": int(first[e]),
                    "centre_to_face_m": round(float(gap[e]), 4),
                    "start": [round(float(v), 3) for v in st],
                    "yaw_deg": round(math.degrees(yw)),
                }
                for e, st, yw in zip(ids, starts, yaws, strict=True)
            ]
            report["pass"] = (
                report["box_drift_standing_max_m"] < 1e-3
                and report["box_vs_route_layout_max_m"] < 1e-3
                and report["boxes_put_back_max_err_m"] < 1e-3
                and len(ids) >= 0.8 * report["envs_with_a_box"]
                and bool(ok.all())
                and len(g) > 0
                and float(g.max()) < 0.10
            )

        elif mode == "robot":
            import numpy as np
            import robot_spec as rbs_
            import route_spec as rs_

            n = env.num_envs
            stop = action(0.0)
            err, hit_standing, mask_bad = 0.0, 0, 0
            seen = []  # (sim range, nearest, farthest possible) on the ray toward a robot
            org = env.scene.env_origins[:, :2]
            half = 0.5 * min(rbs_.ROBOT.size[:2])
            for k in range(100):  # 5 s standing; robots drive, but wait for our robot
                obs, _, _, _, _ = env.step(stop)
                hit_standing += int(env.collided.sum())
                st = env.robot_state()
                for obj, (pos, _, _) in zip(env.robot_objs, st, strict=True):
                    sim_xy = obj.data.root_pos_w.torch[:, :2] - org
                    err = max(err, float((sim_xy - pos).norm(dim=-1).max()))
                if env.spec.intent_robots:  # present flags = active robots (up to intent_robots)
                    names = sae.ts.obs_names(env.spec)
                    cols = [names.index(f"robot{r}_present") for r in range(env.spec.intent_robots)]
                    present = obs["policy"][:, cols].sum(-1)
                    active = torch.stack([a for _, _, a in st], -1).sum(-1)
                    want = active.clamp(max=env.spec.intent_robots).float()
                    mask_bad += int((present != want).sum())
                if k % 10 == 9:
                    rp, ryaw = env.robot_pose()
                    c = lidar_center(rp, ryaw)
                    rows = torch.arange(n, device=env.device)
                    lid = env.lidar_ranges()
                    exp_all = expected_scan(rp, ryaw)  # walls, shelves, boxes
                    for m, (pos, _, act) in enumerate(st):
                        d = (pos - c).norm(dim=-1)
                        bearing = torch.atan2(pos[:, 1] - c[:, 1], pos[:, 0] - c[:, 0]) - ryaw
                        i = torch.remainder(torch.round(torch.rad2deg(bearing)) + 180, 360).long()
                        other_far = torch.ones_like(act)
                        for m2, (p2, _, a2) in enumerate(st):
                            if m2 != m:
                                other_far &= ~a2 | ((p2 - c).norm(dim=-1) > d + 0.2)
                        ok = act & (exp_all[rows, i] > d + 0.1) & other_far & (d < 1.8)
                        near, far = d - rbs_.ROBOT.radius, d - half
                        seen.append(torch.stack((lid[rows, i][ok], near[ok], far[ok]), -1))
            report["robot_pose_err_max_m"] = round(err, 6)
            report["standing_robots_hit_by_robots"] = hit_standing
            report["intent_present_flags_wrong"] = mask_bad
            t_ = torch.cat(seen)
            out = (t_[:, 1] - t_[:, 0]).clamp(min=0) + (t_[:, 0] - t_[:, 2]).clamp(min=0)
            report["robot_rays_compared"] = len(t_)
            ray_ok = len(t_) > 0
            if len(t_):
                report["robot_ray_outside_band_m"] = {
                    "max": round(float(out.max()), 4),
                    "p99": round(float(out.quantile(0.99)), 4),
                }
                ray_ok = float(out.quantile(0.99)) < 0.01
            st = env.robot_state()
            report["robot_active_share"] = [round(float(a.float().mean()), 3) for _, _, a in st]
            # collision: hold every robot where it is, put our robot 0.20 m from a face of the
            # first active robot with a free strip, drive at 0.25 m/s
            sae.rbs.robot_step = lambda route, t, wait, back, *a_: (t, wait, back)
            static = rects.cpu().numpy()
            obst = env.obstacle_xy().cpu().numpy()
            sz = np.array(env.routes.obstacle_sizes)
            ids, starts, yaws = [], [], []
            for e in range(n):
                act_k = [k for k in range(len(st)) if bool(st[k][2][e])]
                if not act_k:
                    continue
                k = act_k[0]
                centre = st[k][0][e].cpu().numpy()
                ryaw0 = float(st[k][1][e])
                world = list(static) + [rs_.box_rect(obst[e, j], sz[j]) for j in range(len(sz))]
                for k2 in act_k[1:]:
                    world.append(rs_.box_rect(st[k2][0][e].cpu().numpy(), 0.17))
                world = np.array(world)
                for q in range(4):
                    ang = ryaw0 + q * math.pi / 2
                    u = np.array((math.cos(ang), math.sin(ang)))
                    face = rbs_.ROBOT.size[q % 2] / 2
                    pnt = centre - u * (face + 0.20)
                    a_, b_ = pnt - u * 0.09, centre - u * face
                    w = STRIP_HALF_M * np.abs(u[::-1])
                    strip = np.concatenate((np.minimum(a_, b_) - w, np.maximum(a_, b_) + w))
                    if not rs_.rects_overlap(strip, world):
                        ids.append(e)
                        starts.append(pnt)
                        yaws.append(ang)
                        break
            n_with = int(torch.stack([a for _, _, a in st], -1).any(-1).sum())
            report["envs_with_a_robot"] = n_with
            report["envs_driven"] = len(ids)
            place(ids, np.array(starts), yaws)
            tested = torch.zeros(n, dtype=torch.bool, device=env.device)
            tested[ids] = True
            act_ = torch.where(tested[:, None], action(0.25), stop)
            first = torch.full((n,), -1, dtype=torch.long, device=env.device)
            gap_at = torch.full((n,), float("nan"), device=env.device)
            kind_at = torch.full((n,), -1, dtype=torch.long, device=env.device)
            for k in range(60):
                rp, _ = env.robot_pose()
                g = env.robot_gaps(rp)
                env.step(act_)
                hit = env.collided & tested & (first < 0)
                first[hit] = k + 1
                gap_at[hit] = g[hit]
                kind_at[hit] = env.hit_kind[hit]
            t_ids = torch.tensor(ids, device=env.device, dtype=torch.long)
            ok = first[t_ids] > 0
            g = gap_at[t_ids][ok]
            report["collided"] = int(ok.sum())
            report["collisions_put_down_to_robot"] = int((kind_at[t_ids][ok] == 3).sum())
            rp, ryaw = env.robot_pose()
            st = env.robot_state()
            report["not_collided"] = [
                {
                    "env": int(e),
                    "start": [round(float(v), 3) for v in starts[j]],
                    "yaw_deg": round(math.degrees(yaws[j])),
                    "end": [round(float(v), 3) for v in rp[e]],
                    "robots": [[round(float(v), 3) for v in b[0][e]] + [bool(b[2][e])] for b in st],
                    "ep_len": int(env.episode_length_buf[e]),
                }
                for j, e in enumerate(ids)
                if not bool(ok[j])
            ]
            if len(g):
                report["centre_to_robot_surface_before_collision_m"] = [
                    round(float(g.min()), 4),
                    round(float(g.max()), 4),
                ]
            report["pass"] = (
                err < 1e-3
                and ray_ok
                and hit_standing == 0
                and mask_bad == 0
                and env.spec.intent_robots > 0
                and len(ids) >= 0.8 * n_with
                and bool(ok.all())
                and report["collisions_put_down_to_robot"] == len(ids)
                and len(g) > 0
                and float(g.max()) < 0.10
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
