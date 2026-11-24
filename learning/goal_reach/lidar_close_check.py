"""라이다 최소 거리 0.05 m 안쪽의 물체를 goal-reach 라이다가 어떻게 돌려주는지 잰다.

Pinky 를 고정하고 kinematic 박스를 라이다 중심에서 d m 뒤(레이 0)와 왼쪽(레이 270)에 두어 원시 레이 거리,
정책이 보는 섹터 최솟값, 섹터의 적중 여부를 기록한다. <out>/lidar_close.csv 를 쓰고 표를 출력한다.
"""

import argparse
import math
import os
import sys

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", type=str, required=True)
add_launcher_args(parser)
args_cli = parser.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sim_check"))

DT = 1.0 / 200.0
LIDAR_XY = (-0.017, 0.0)  # base_link 기준
LIDAR_Z = 0.125  # 바닥 위
BOX_T = 0.02  # 레이 방향 박스 두께
DISTS = [0.30, 0.15, 0.10, 0.07, 0.06, 0.055, 0.05, 0.045, 0.04, 0.03, 0.02, 0.01, 0.0]
DIRS = {
    "rear_ray0": (180.0, 0),
    "left_ray270": (90.0, 270),
}  # (base 프레임 방향 deg, 레이 인덱스)


def main():
    from pinky_cfg import make_newton_cfg

    with launch_simulation(cfg=make_newton_cfg(), launcher_args=args_cli) as physics_cfg:
        import isaaclab.sim as sim_utils
        import task_spec as ts
        import torch
        from goal_reach_env import make_lidar_cfg
        from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.utils import configclass
        from pinky_cfg import make_pinky_cfg

        robot_cfg = make_pinky_cfg()
        robot_cfg.spawn.fix_root_link = True

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(
                prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg()
            )
            robot = robot_cfg
            box = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/Box",
                spawn=sim_utils.CuboidCfg(
                    size=(BOX_T, 0.30, 0.30),
                    rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                    mass_props=sim_utils.MassCfg(mass=1.0),
                    collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(3.0, 0.0, 0.15)),
            )
            lidar = make_lidar_cfg(ts.SPEC.lidar_obs_max)

        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(dt=DT, device=args_cli.device, physics=physics_cfg)
        )
        scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=4.0))
        sim.reset()
        box, lidar = scene["box"], scene["lidar"]
        origin = scene.env_origins[0]

        def settle(k=4):
            for _ in range(k):
                scene.write_data_to_sim()
                sim.step()
                scene.update(DT)

        def ranges():
            return (lidar.data.ray_distances.torch + ts.SPEC.lidar_min_range)[0]

        rows = []
        settle(20)
        r0 = ranges()
        empty = {
            "min": float(r0.min()),
            "max": float(r0.max()),
            "n_finite": int(torch.isfinite(r0).sum()),
        }
        print(
            f"[LIDAR] no box: raw min {empty['min']:.4f} max {empty['max']:.4f} "
            f"finite {empty['n_finite']}/360"
        )
        for name, (deg, ray) in DIRS.items():
            ux, uy = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            sector = ray // (ts.SPEC.lidar_rays // ts.SPEC.lidar_sectors)
            for d in DISTS:
                c = d + BOX_T / 2  # 레이를 따라 박스 중심
                pose = box.data.default_root_pose.torch.clone()
                pose[0, 0] = origin[0] + LIDAR_XY[0] + ux * c
                pose[0, 1] = origin[1] + LIDAR_XY[1] + uy * c
                pose[0, 2] = LIDAR_Z
                yaw = math.radians(deg)
                pose[0, 3:7] = torch.tensor((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
                box.write_root_pose_to_sim_index(root_pose=pose)
                settle()
                r = ranges()
                sec = ts.lidar_sectors(r.unsqueeze(0))[0, sector]
                lo = sector * 10
                n_hit = int((r[lo : lo + 10] < ts.SPEC.lidar_obs_max - 1e-3).sum())
                rows.append((name, d, float(r[ray]), float(sec), n_hit))
                print(
                    f"[LIDAR] {name} d={d:.3f}: ray {float(r[ray]):.4f}  "
                    f"sector_min {float(sec):.4f}  hits_in_sector {n_hit}/10"
                )
        os.makedirs(args_cli.out, exist_ok=True)
        with open(os.path.join(args_cli.out, "lidar_close.csv"), "w", encoding="utf-8") as f:
            f.write(
                f"# no box: raw min {empty['min']:.4f} max {empty['max']:.4f} "
                f"finite {empty['n_finite']}/360\n"
            )
            f.write("direction,box_face_dist_m,ray_range_m,sector_min_m,hits_in_sector\n")
            for r in rows:
                f.write(f"{r[0]},{r[1]:.3f},{r[2]:.4f},{r[3]:.4f},{r[4]}\n")
        print("[LIDAR] done")


if __name__ == "__main__":
    main()
