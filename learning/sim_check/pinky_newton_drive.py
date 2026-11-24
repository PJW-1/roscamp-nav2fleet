"""Pinky(Pode)를 Newton/MuJoCo-Warp headless 에서 바퀴 속도 명령으로 몰고 움직임을 기록한다.

순서: 안정 1 s, 직진 0.10/0.20 m/s, 정지, 제자리 회전 1.0 rad/s, 호 주행 두 가지, 정지. 물리 200 Hz, 제어 20 Hz.
CSV(모든 env, 50 Hz)와 JSON 요약을 쓴다 (실행법은 README.md).
"""

import argparse
import json
import os
import sys
import time

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--out", type=str, required=True, help="Output directory for CSV/JSON.")
parser.add_argument("--wheel_damping", type=float, default=0.05)
parser.add_argument("--wheel_effort", type=float, default=0.5)
parser.add_argument("--caster_damping", type=float, default=0.0)
parser.add_argument("--dt", type=float, default=1.0 / 200.0)
parser.add_argument("--decimation", type=int, default=10)
parser.add_argument("--log_every", type=int, default=4, help="Physics steps between CSV rows.")
parser.add_argument("--tag", type=str, default="drive")
parser.add_argument(
    "--accel", type=float, default=0.0, help="Linear accel limit on commands, m/s^2 (0 = step)."
)
parser.add_argument(
    "--alpha", type=float, default=0.0, help="Angular accel limit on commands, rad/s^2 (0 = step)."
)
add_launcher_args(parser)
args_cli = parser.parse_args()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PHASES = [  # (이름, 길이 s, v m/s, w rad/s)
    ("settle", 1.0, 0.0, 0.0),
    ("straight_0.10", 3.0, 0.10, 0.0),
    ("straight_0.20", 3.0, 0.20, 0.0),
    ("stop1", 1.0, 0.0, 0.0),
    ("spin_1.0", 3.0, 0.0, 1.0),
    ("arc_left", 3.0, 0.12, 0.6),
    ("arc_right", 3.0, 0.12, -0.6),
    ("stop2", 2.0, 0.0, 0.0),
]


def phase_at(t: float):
    t0 = 0.0
    for name, dur, v, w in PHASES:
        if t < t0 + dur - 1e-9:
            return name, v, w
        t0 += dur
    return None


def clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


def main():
    from pinky_cfg import make_newton_cfg

    newton_cfg = make_newton_cfg()
    with launch_simulation(cfg=newton_cfg, launcher_args=args_cli) as physics_cfg:
        import isaaclab.sim as sim_utils
        import torch
        from isaaclab.assets import AssetBaseCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.utils import configclass
        from pinky_cfg import WHEEL_RADIUS, WHEEL_SEPARATION, make_pinky_cfg

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(
                prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg()
            )
            robot = make_pinky_cfg(
                wheel_damping=args_cli.wheel_damping,
                wheel_effort=args_cli.wheel_effort,
                caster_damping=args_cli.caster_damping,
            )

        print(
            f"[CHECK] physics cfg: {type(physics_cfg).__name__} "
            f"solver={type(physics_cfg.solver_cfg).__name__}"
        )
        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(dt=args_cli.dt, device=args_cli.device, physics=physics_cfg)
        )
        t_build = time.time()
        scene = InteractiveScene(SceneCfg(num_envs=args_cli.num_envs, env_spacing=3.0))
        sim.reset()
        print(f"[CHECK] scene build + reset: {time.time() - t_build:.1f} s")

        robot = scene["robot"]
        print("[CHECK] joint names:", robot.joint_names)
        print("[CHECK] body names:", robot.body_names)
        wheel_ids, wheel_names = robot.find_joints(
            ["l_wheel_joint", "r_wheel_joint"], preserve_order=True
        )
        print("[CHECK] wheel ids:", wheel_ids, wheel_names)
        masses = robot.data.body_mass.torch[0] if hasattr(robot.data, "body_mass") else None
        if masses is not None:
            print(f"[CHECK] body masses env0: {masses.tolist()} total={float(masses.sum()):.3f}")

        n = args_cli.num_envs
        dev = robot.device
        origins = scene.env_origins
        total_time = sum(p[1] for p in PHASES)
        steps = int(round(total_time / args_cli.dt))
        rows = []
        target = torch.zeros(n, 2, device=dev)
        nan_seen = False
        v = w = 0.0
        dt_ctrl = args_cli.dt * args_cli.decimation
        t_start = time.time()
        for k in range(steps):
            t = k * args_cli.dt
            if k % args_cli.decimation == 0:
                _, v_goal, w_goal = phase_at(t)
                v = (
                    v_goal
                    if args_cli.accel <= 0
                    else v + clamp(v_goal - v, args_cli.accel * dt_ctrl)
                )
                w = (
                    w_goal
                    if args_cli.alpha <= 0
                    else w + clamp(w_goal - w, args_cli.alpha * dt_ctrl)
                )
                wl = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
                wr = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
                target[:, 0] = wl
                target[:, 1] = wr
                robot.set_joint_velocity_target_index(target=target, joint_ids=wheel_ids)
            scene.write_data_to_sim()
            sim.step()
            scene.update(args_cli.dt)
            if args_cli.log_every > 0 and (k + 1) % args_cli.log_every == 0:
                pos = robot.data.root_link_pos_w.torch - origins
                quat = robot.data.root_link_quat_w.torch  # x, y, z, w
                lin_b = robot.data.root_link_lin_vel_b.torch
                ang_b = robot.data.root_link_ang_vel_b.torch
                jv = robot.data.joint_vel.torch[:, wheel_ids]
                x, y, z, qw = quat.unbind(-1)
                roll = torch.atan2(2 * (qw * x + y * z), 1 - 2 * (x * x + y * y))
                pitch = torch.asin(torch.clamp(2 * (qw * y - z * x), -1, 1))
                yaw = torch.atan2(2 * (qw * z + x * y), 1 - 2 * (y * y + z * z))
                block = torch.stack(
                    [
                        pos[:, 0],
                        pos[:, 1],
                        pos[:, 2],
                        roll,
                        pitch,
                        yaw,
                        lin_b[:, 0],
                        lin_b[:, 1],
                        lin_b[:, 2],
                        ang_b[:, 0],
                        ang_b[:, 1],
                        ang_b[:, 2],
                        target[:, 0],
                        target[:, 1],
                        jv[:, 0],
                        jv[:, 1],
                    ],
                    dim=-1,
                ).cpu()
                if not torch.isfinite(block).all():
                    nan_seen = True
                tt = (k + 1) * args_cli.dt
                for e in range(n):
                    rows.append(
                        (
                            round(tt, 4),
                            phase_at(min(tt, total_time - 1e-6))[0],
                            e,
                            *block[e].tolist(),
                        )
                    )
        torch.cuda.synchronize()
        wall = time.time() - t_start
        # 전체 env 의 종료 상태 점검 (--log_every 0 벤치마크에도 해당)
        end_pos = robot.data.root_link_pos_w.torch - origins
        end_ok = (
            torch.isfinite(end_pos).all(dim=-1) & (end_pos[:, 2] > 0.02) & (end_pos[:, 2] < 0.04)
        )
        end_up = robot.data.projected_gravity_b.torch[:, 2]  # 똑바로 서면 -1

        os.makedirs(args_cli.out, exist_ok=True)
        cols = [
            "t",
            "phase",
            "env",
            "x",
            "y",
            "z",
            "roll",
            "pitch",
            "yaw",
            "vx_b",
            "vy_b",
            "vz_b",
            "wx_b",
            "wy_b",
            "wz_b",
            "wl_cmd",
            "wr_cmd",
            "wl",
            "wr",
        ]
        csv_path = os.path.join(args_cli.out, f"{args_cli.tag}_log.csv")
        if rows:
            with open(csv_path, "w", encoding="utf-8") as f:
                f.write(",".join(cols) + "\n")
                for r in rows:
                    f.write(
                        ",".join(
                            r_
                            if isinstance(r_, str)
                            else (f"{r_:.6g}" if isinstance(r_, float) else str(r_))
                            for r_ in r
                        )
                        + "\n"
                    )
        info = {
            "end_envs_ok": int(end_ok.sum()),
            "end_min_upright_cos": round(float(-end_up.max()), 5),
            "end_xy_spread_mm": [round(float(torch.std(end_pos[:, i]) * 1000), 3) for i in (0, 1)],
            "num_envs": n,
            "dt": args_cli.dt,
            "decimation": args_cli.decimation,
            "physics_steps": steps,
            "sim_time_s": total_time,
            "wall_time_loop_s": round(wall, 2),
            "physics_steps_per_s": round(steps / wall, 1),
            "env_steps_per_s": round(steps * n / wall, 1),
            "realtime_factor_per_env": round(total_time / wall, 2),
            "nan_seen": nan_seen,
            "wheel_damping": args_cli.wheel_damping,
            "wheel_effort": args_cli.wheel_effort,
            "caster_damping": args_cli.caster_damping,
            "accel_limit": args_cli.accel,
            "alpha_limit": args_cli.alpha,
            "cuda_max_mem_alloc_MB": round(torch.cuda.max_memory_allocated() / 2**20, 1),
            "joint_names": list(robot.joint_names),
            "body_names": list(robot.body_names),
            "body_mass_env0": None if masses is None else [round(float(m), 4) for m in masses],
        }
        with open(
            os.path.join(args_cli.out, f"{args_cli.tag}_run.json"), "w", encoding="utf-8"
        ) as f:
            json.dump(info, f, indent=1)
        print("[CHECK] run info:", json.dumps(info))
        print(f"[CHECK] wrote {csv_path}")


if __name__ == "__main__":
    main()
