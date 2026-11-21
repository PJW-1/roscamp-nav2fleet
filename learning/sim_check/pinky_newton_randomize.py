"""Check friction and mass randomization events on Newton with Pinky, inside a ManagerBasedEnv.

Startup events: randomize_rigid_body_material on the wheel and caster shapes (friction range below),
randomize_rigid_body_mass on base_link (scale 0.5-2.0). The values written into the Newton model are
read back per env. Then every env runs the same command: 0.5 s still, step to 0.2 m/s for 2 s, step
stop for 1 s, spin 1.0 rad/s for 2 s. Per env we record distance covered in the first 0.3 s of the
step (traction-limited), steady speed, skid after the stop, and spin rate.
Writes <tag>_per_env.csv and prints correlations.
"""

import argparse
import os
import sys

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--out", type=str, required=True)
parser.add_argument("--tag", type=str, default="rand")
parser.add_argument(
    "--ground_mu", type=float, default=None, help="Ground friction; default leaves the plane as is."
)
parser.add_argument("--mu_range", type=float, nargs=2, default=(0.05, 1.0))
parser.add_argument("--mass_scale", type=float, nargs=2, default=(0.5, 2.0))
parser.add_argument("--wheel_damping", type=float, default=0.05)
add_launcher_args(parser)
args_cli = parser.parse_args()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DT = 1.0 / 200.0
DECIM = 10


def main():
    import isaaclab.envs.mdp as mdp
    import isaaclab.sim as sim_utils
    import torch
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.envs import ManagerBasedEnvCfg
    from isaaclab.managers import EventTermCfg as EventTerm
    from isaaclab.managers import ObservationGroupCfg as ObsGroup
    from isaaclab.managers import ObservationTermCfg as ObsTerm
    from isaaclab.managers import SceneEntityCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sim.spawners.materials import UsdPhysicsRigidBodyMaterialCfg
    from isaaclab.utils import configclass, instantiate
    from pinky_cfg import WHEEL_RADIUS, WHEEL_SEPARATION, make_newton_cfg, make_pinky_cfg

    ground_spawn = sim_utils.GroundPlaneCfg()
    if args_cli.ground_mu is not None:
        ground_spawn = sim_utils.GroundPlaneCfg(
            physics_material=UsdPhysicsRigidBodyMaterialCfg(
                static_friction=args_cli.ground_mu, dynamic_friction=args_cli.ground_mu
            )
        )

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=ground_spawn)
        robot = make_pinky_cfg(wheel_damping=args_cli.wheel_damping)

    @configclass
    class ActionsCfg:
        wheels = mdp.JointVelocityActionCfg(
            asset_name="robot",
            joint_names=["l_wheel_joint", "r_wheel_joint"],
            scale=1.0,
            use_default_offset=False,
        )

    @configclass
    class ObservationsCfg:
        @configclass
        class PolicyCfg(ObsGroup):
            base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
            base_ang_vel = ObsTerm(func=mdp.base_ang_vel)

        policy: PolicyCfg = PolicyCfg()

    @configclass
    class EventCfg:
        wheel_friction = EventTerm(
            func=mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", body_names=["l_wheel", "r_wheel", "caster_wheel"]
                ),
                "static_friction_range": tuple(args_cli.mu_range),
                "dynamic_friction_range": tuple(args_cli.mu_range),
                "restitution_range": (0.0, 0.0),
                "num_buckets": 64,
            },
        )
        base_mass = EventTerm(
            func=mdp.randomize_rigid_body_mass,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]),
                "mass_distribution_params": tuple(args_cli.mass_scale),
                "operation": "scale",
            },
        )

    @configclass
    class EnvCfg(ManagerBasedEnvCfg):
        scene = SceneCfg(num_envs=args_cli.num_envs, env_spacing=3.0)
        observations = ObservationsCfg()
        actions = ActionsCfg()
        events = EventCfg()

        def __post_init__(self):
            self.decimation = DECIM
            self.sim.dt = DT
            self.sim.physics = make_newton_cfg()

    env_cfg = EnvCfg()
    env_cfg.sim.device = args_cli.device
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        robot = env.scene["robot"]
        n = env.num_envs
        model = env.sim.physics_manager.get_model()
        mu_view = robot._root_view.get_attribute("shape_material_mu", model)[:, 0]
        mu = wp_to_torch(mu_view)
        body_ids, body_names = robot.find_bodies(
            ["l_wheel", "r_wheel", "caster_wheel", "base_link"], preserve_order=True
        )
        shapes = robot._root_view.body_shapes
        backend_ids = robot.map_body_ids_to_backend(body_ids)
        print(
            "[CHECK] shape mu shape:",
            tuple(mu.shape),
            "bodies:",
            body_names,
            "backend ids:",
            list(backend_ids),
        )
        mu_by_body = {
            name: mu[:, list(shapes[b])].mean(dim=1)
            for name, b in zip(body_names, backend_ids, strict=True)
        }
        mass = robot.data.body_mass.torch
        base_mass = mass[:, body_ids[3]]
        print("[CHECK] base_link mass per env:", [round(float(x), 3) for x in base_mass])
        for name, v in mu_by_body.items():
            print(f"[CHECK] {name} mu per env:", [round(float(x), 3) for x in v])
        gm = model.shape_material_mu.numpy()
        sw = model.shape_world.numpy()
        print("[CHECK] global (ground) shape mu:", [round(float(x), 3) for x in gm[sw == -1]])

        env.reset()
        t, rows = 0.0, []
        x_at = {}
        spin_rates = []
        with torch.inference_mode():
            for k in range(int(5.5 / (DT * DECIM))):
                if t < 0.5:
                    v, w = 0.0, 0.0
                elif t < 2.5:
                    v, w = 0.2, 0.0
                elif t < 3.5:
                    v, w = 0.0, 0.0
                else:
                    v, w = 0.0, 1.0
                wl = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
                wr = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
                act = torch.tensor([[wl, wr]], device=env.device).repeat(n, 1)
                env.step(act)
                t = round((k + 1) * DT * DECIM, 4)
                pos = robot.data.root_link_pos_w.torch - env.scene.env_origins
                for key in (0.5, 0.8, 2.5, 3.5):
                    if abs(t - key) < 1e-6:
                        x_at[key] = pos[:, 0].clone()
                if 2.0 < t <= 2.5:
                    rows.append(robot.data.root_link_lin_vel_b.torch[:, 0].clone())
                if 4.5 < t <= 5.5:
                    spin_rates.append(robot.data.root_link_ang_vel_b.torch[:, 2].clone())
        v_ss = torch.stack(rows).mean(0)
        spin = torch.stack(spin_rates).mean(0)
        d03 = x_at[0.8] - x_at[0.5]
        skid = x_at[3.5] - x_at[2.5]
        finite = bool(torch.isfinite(robot.data.root_link_pos_w.torch).all())
        os.makedirs(args_cli.out, exist_ok=True)
        path = os.path.join(args_cli.out, f"{args_cli.tag}_per_env.csv")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(
                "env,mu_l_wheel,mu_r_wheel,mu_caster,base_mass,dist_first_0.3s_mm,v_steady,skid_after_stop_mm,"
                "spin_rate\n"
            )
            for e in range(n):
                fh.write(
                    f"{e},{mu_by_body['l_wheel'][e]:.4f},{mu_by_body['r_wheel'][e]:.4f},"
                    f"{mu_by_body['caster_wheel'][e]:.4f},{base_mass[e]:.4f},{d03[e] * 1000:.2f},"
                    f"{v_ss[e]:.4f},{skid[e] * 1000:.2f},{spin[e]:.4f}\n"
                )

        def corr(a, b):
            a, b = a.float() - a.float().mean(), b.float() - b.float().mean()
            return float((a * b).sum() / (a.norm() * b.norm() + 1e-12))

        mu_min = torch.minimum(mu_by_body["l_wheel"], mu_by_body["r_wheel"])
        print(f"[CHECK] finite={finite}")
        print(
            f"[CHECK] dist_first_0.3s mm: min {d03.min() * 1000:.2f} max {d03.max() * 1000:.2f}; "
            f"corr with min wheel mu {corr(mu_min, d03):.3f}, "
            f"with base mass {corr(base_mass, d03):.3f}"
        )
        print(f"[CHECK] v_steady: min {v_ss.min():.4f} max {v_ss.max():.4f}")
        print(
            f"[CHECK] skid after stop mm: min {skid.min() * 1000:.2f} max {skid.max() * 1000:.2f}; "
            f"corr with min wheel mu {corr(mu_min, skid):.3f}, "
            f"with base mass {corr(base_mass, skid):.3f}"
        )
        print(
            f"[CHECK] spin rate: min {spin.min():.4f} max {spin.max():.4f}; corr with min wheel mu "
            f"{corr(mu_min, spin):.3f}"
        )
        print(f"[CHECK] wrote {path}")
        env.close()


def wp_to_torch(arr):
    import warp as wp

    return wp.to_torch(arr) if not hasattr(arr, "torch") else arr.torch


if __name__ == "__main__":
    main()
