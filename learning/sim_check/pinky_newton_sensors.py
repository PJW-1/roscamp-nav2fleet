"""Newton sensor check on Pinky: 2D lidar (NewtonRaycastSensorCfg) and contact sensors.

Phase A (0-4 s): robot still; a box (0.1 x 0.3 x 0.3 m) is teleported across in front at x=+0.6 m,
  y from -1.0 to +1.0 m. Logs the forward ray distance and the minimum over all rays.
Phase B (4-16 s): box parked aside; robot drives forward at 0.15 m/s into a fixed wall at x=+1.5 m.
  Logs contact sensor net forces on base_link, front_camera_mount, screen_mount (the front-most
  collider, which is what touches the wall) and l_wheel (wheel-ground control), and the robot x.
Writes <tag>_lidar.csv, <tag>_contact.csv and a JSON with the lidar scan at three instants.
"""

import argparse
import json
import os
import sys

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--tag", type=str, default="sensors")
parser.add_argument(
    "--min_range", type=float, default=0.05, help="Lidar rays start this far out [m]."
)
add_launcher_args(parser)
args_cli = parser.parse_args()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DT = 1.0 / 200.0
DECIM = 10
BOX_X = 0.6
WALL_X = 1.5


def main():
    from pinky_cfg import BODY, WHEEL_RADIUS, make_newton_cfg

    with launch_simulation(cfg=make_newton_cfg(), launcher_args=args_cli) as physics_cfg:
        import isaaclab.sim as sim_utils
        import torch
        from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab.sensors.ray_caster import patterns
        from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
        from isaaclab.utils import configclass
        from isaaclab_newton.sensors import NewtonRaycastSensorCfg
        from pinky_cfg import make_pinky_cfg

        def lidar_min_range_pattern(cfg, device):
            """Lidar rays that start ``min_range`` out along their direction, so the robot's own
            rplidar collider (which surrounds the sensor origin) is not hit. Distances are then
            measured from the start point: true range = distance + min_range."""
            starts, dirs = patterns.lidar_pattern(cfg, device)
            return starts + cfg.min_range * dirs, dirs

        @configclass
        class MinRangeLidarPatternCfg(LidarPatternCfg):
            func: object = lidar_min_range_pattern
            min_range: float = 0.0

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(
                prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg()
            )
            robot = make_pinky_cfg()
            box = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/Box",
                spawn=sim_utils.CuboidCfg(
                    size=(0.1, 0.3, 0.3),
                    rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                    mass_props=sim_utils.MassCfg(mass=1.0),
                    collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(BOX_X, -1.0, 0.15)),
            )
            wall = AssetBaseCfg(
                prim_path="{ENV_REGEX_NS}/Wall",
                spawn=sim_utils.CuboidCfg(
                    size=(0.1, 1.0, 0.3), collision_props=sim_utils.UsdPhysicsCollisionCfg()
                ),
                init_state=AssetBaseCfg.InitialStateCfg(pos=(WALL_X + 0.05, 0.0, 0.15)),
            )
            # rplidar_link in base_link: (-0.017, 0, 0.097), yaw 180 deg. The ray caster rotates ray
            # directions by offset.rot but not ray starts, so with non-zero starts (min range) the
            # yaw is left out here and rays are indexed in the base frame (ray angle 0 = base +x).
            lidar = NewtonRaycastSensorCfg(
                prim_path="{ENV_REGEX_NS}/Robot/" + BODY,
                offset=NewtonRaycastSensorCfg.OffsetCfg(pos=(-0.017, 0.0, 0.097)),
                pattern_cfg=MinRangeLidarPatternCfg(
                    channels=1,
                    vertical_fov_range=(0.0, 0.0),
                    horizontal_fov_range=(-180.0, 179.0),
                    horizontal_res=1.0,
                    min_range=args_cli.min_range,
                ),
                ray_alignment="base",
                max_distance=12.0 - args_cli.min_range,
            )
            bumper = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + BODY, history_length=1)
            # the camera mount sticks out ~2 mm further forward than base_link's collider
            bumper_cam = ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/Robot/" + BODY + "/front_camera_mount"
            )
            # wheel-ground and screen contacts, as a control for the sensor itself
            contact_lwheel = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + BODY + "/l_wheel")
            contact_screen = ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/Robot/" + BODY + "/screen_mount"
            )

        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(dt=DT, device=args_cli.device, physics=physics_cfg)
        )
        scene = InteractiveScene(SceneCfg(num_envs=args_cli.num_envs, env_spacing=4.0))
        sim.reset()
        robot, box, lidar, bumper = scene["robot"], scene["box"], scene["lidar"], scene["bumper"]
        bumper_cam = scene["bumper_cam"]
        n = args_cli.num_envs
        wheel_ids, _ = robot.find_joints(["l_wheel_joint", "r_wheel_joint"], preserve_order=True)
        print(f"[CHECK] lidar rays: {lidar.num_rays}; bumper bodies: {bumper.body_names}")
        # ray directions in the base frame (offset applied) -> the ray pointing along base +x
        dirs = lidar.ray_directions
        dirs = (dirs.torch if hasattr(dirs, "torch") else dirs)[0]
        fwd = int(torch.argmax(dirs[:, 0]).item())
        print(f"[CHECK] forward ray index (base +x) = {fwd}, dir = {dirs[fwd].tolist()}")

        origins = scene.env_origins
        lidar_rows, contact_rows, scans = [], [], {}
        steps = int(16.0 / DT)
        target = torch.zeros(n, 2, device=robot.device)
        for k in range(steps):
            t = k * DT
            if t < 4.0:
                y = -1.0 + 2.0 * t / 4.0
            else:
                y = 3.0
            pose = box.data.default_root_pose.torch.clone()
            pose[:, 0] = origins[:, 0] + BOX_X
            pose[:, 1] = origins[:, 1] + y
            pose[:, 2] = 0.15
            box.write_root_pose_to_sim_index(root_pose=pose)
            box.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros(n, 6, device=robot.device)
            )
            if k % DECIM == 0:
                v = 0.15 if t >= 4.5 else 0.0
                target[:] = v / WHEEL_RADIUS
                robot.set_joint_velocity_target_index(target=target, joint_ids=wheel_ids)
            scene.write_data_to_sim()
            sim.step()
            scene.update(DT)
            if (k + 1) % DECIM == 0:
                tt = round((k + 1) * DT, 3)
                d = (
                    lidar.data.ray_distances.torch + args_cli.min_range
                )  # range from the lidar centre
                finite = torch.isfinite(d) & (d < 12.0)
                dmin = torch.where(finite, d, torch.full_like(d, float("inf"))).min(dim=1).values
                rx = robot.data.root_link_pos_w.torch[:, 0] - origins[:, 0]
                lidar_rows.append(
                    (
                        tt,
                        y,
                        float(d[0, fwd]),
                        float(dmin[0]),
                        int(finite[0].sum()),
                        float(d[:, fwd].std()),
                        float(rx[0]),
                    )
                )
                f = bumper.data.net_forces_w.torch[:, 0, :]
                fc = bumper_cam.data.net_forces_w.torch[:, 0, :]
                fs = scene["contact_screen"].data.net_forces_w.torch[:, 0, :]
                fw = scene["contact_lwheel"].data.net_forces_w.torch[:, 0, :]
                contact_rows.append(
                    (
                        tt,
                        float(rx[0]),
                        float(f[0].norm()),
                        float(fc[0].norm()),
                        float(fs[0].norm()),
                        float(fs[0, 0]),
                        float(fs.norm(dim=-1).mean()),
                        float(fw[0, 0]),
                        float(fw[0, 2]),
                        float(robot.data.root_link_lin_vel_b.torch[0, 0]),
                    )
                )
                if k + 1 == DECIM:
                    print(
                        "[CHECK] lidar pos_w env0:",
                        lidar.data.pos_w.torch[0].tolist(),
                        "base pos_w env0:",
                        robot.data.root_link_pos_w.torch[0].tolist(),
                    )
                    for ri in (0, 90, 180, 270):
                        print(
                            f"[CHECK] ray {ri}: start_w {lidar.ray_starts_w.torch[0, ri].tolist()} "
                            f"hit_w {lidar.data.ray_hits_w.torch[0, ri].tolist()} "
                            f"dist {float(d[0, ri]):.4f}"
                        )
                if tt in (2.0, 10.0, 15.0):
                    for name in ("bumper", "bumper_cam", "contact_lwheel", "contact_screen"):
                        fw = scene[name].data.net_forces_w.torch[0, 0]
                        print(
                            f"[CHECK] t={tt} contact {name}: net_forces_w env0 = "
                            f"{[round(float(x), 3) for x in fw]}"
                        )
                if tt in (1.0, 2.0, 3.0, 10.0):
                    scans[str(tt)] = [round(float(x), 4) for x in d[0].tolist()]

        os.makedirs(args_cli.out, exist_ok=True)
        with open(
            os.path.join(args_cli.out, f"{args_cli.tag}_lidar.csv"), "w", encoding="utf-8"
        ) as fh:
            fh.write(
                "t,box_y,fwd_ray_dist_env0,min_ray_dist_env0,n_hits_env0,fwd_ray_std_over_envs,robot_x_env0\n"
            )
            for r in lidar_rows:
                fh.write(",".join(f"{x:.5g}" for x in r) + "\n")
        with open(
            os.path.join(args_cli.out, f"{args_cli.tag}_contact.csv"), "w", encoding="utf-8"
        ) as fh:
            fh.write(
                "t,robot_x_env0,base_force_norm_env0,cam_force_norm_env0,screen_force_norm_env0,"
                "screen_fx_env0,screen_force_norm_mean,lwheel_fx_env0,lwheel_fz_env0,vx_b_env0\n"
            )
            for r in contact_rows:
                fh.write(",".join(f"{x:.5g}" for x in r) + "\n")
        with open(
            os.path.join(args_cli.out, f"{args_cli.tag}_scans.json"), "w", encoding="utf-8"
        ) as fh:
            json.dump(
                {"forward_ray_index": fwd, "num_rays": lidar.num_rays, "scans_env0": scans}, fh
            )
        print("[CHECK] done")


if __name__ == "__main__":
    main()
