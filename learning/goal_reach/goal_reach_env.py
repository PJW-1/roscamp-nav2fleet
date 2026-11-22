"""Isaac Lab (Newton / MuJoCo-Warp) direct RL env: Pinky drives to a random goal on an empty floor.

The task itself (observation, action -> (v, w), reward, success, collision) lives in
``task_spec.py`` and has no simulator code. This file only reads sim state, feeds it to those
functions and writes wheel velocity targets. Next week's store arena adds walls and shelves to
``GoalReachSceneCfg`` and changes the start/goal sampling; the rest should stay.
"""

from __future__ import annotations

import csv
import os
import sys
import time

import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sensors.ray_caster import patterns
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_newton.sensors import NewtonRaycastSensorCfg

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sim_check"))

import task_spec as ts  # noqa: E402
from pinky_cfg import BODY, make_newton_cfg, make_pinky_cfg  # noqa: E402

PHYSICS_DT = 1.0 / 200.0
DECIMATION = round(ts.SPEC.control_dt / PHYSICS_DT)  # 10 -> 20 Hz control


def lidar_min_range_pattern(cfg, device):
    """Rays start ``min_range`` out so the robot's own rplidar collider is not hit
    (see learning/sim_check/README.md). Returned distance + min_range = range from the centre."""
    starts, dirs = patterns.lidar_pattern(cfg, device)
    return starts + cfg.min_range * dirs, dirs


@configclass
class MinRangeLidarPatternCfg(LidarPatternCfg):
    func: object = lidar_min_range_pattern
    min_range: float = 0.0


def make_lidar_cfg(max_range: float) -> NewtonRaycastSensorCfg:
    """Lidar at rplidar_link, base_link (-0.017, 0, 0.097); ray i at (-180 + i) deg, base frame."""
    return NewtonRaycastSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + BODY,
        offset=NewtonRaycastSensorCfg.OffsetCfg(pos=(-0.017, 0.0, 0.097)),
        pattern_cfg=MinRangeLidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 179.0),
            horizontal_res=360.0 / ts.SPEC.lidar_rays,
            min_range=ts.SPEC.lidar_min_range,
        ),
        ray_alignment="base",
        max_distance=max_range - ts.SPEC.lidar_min_range,
    )


@configclass
class GoalReachSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    robot = make_pinky_cfg()
    # Rays only hit shapes of their own env (Newton world) and the global ground: checked with
    # overlapping arenas in learning/store_arena/arena_check.py --mode crossenv.
    lidar = make_lidar_cfg(ts.SPEC.lidar_obs_max)
    contact_base = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + BODY)
    contact_parts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/"
        + BODY
        + "/(screen_mount|front_camera_mount|rplidar_mount)"
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


@configclass
class GoalReachEnvCfg(DirectRLEnvCfg):
    decimation = DECIMATION
    episode_length_s = ts.SPEC.episode_s
    action_space = ts.ACT_DIM
    observation_space = ts.OBS_DIM
    state_space = 0
    sim: SimulationCfg = SimulationCfg(
        dt=PHYSICS_DT, render_interval=DECIMATION, physics=make_newton_cfg()
    )
    # 8 m cells keep the robots (out_of_bounds 3 m) visibly apart; lidar and contacts do not need
    # it (envs are separate Newton worlds, see the scene comment above).
    scene: GoalReachSceneCfg = GoalReachSceneCfg(
        num_envs=4096, env_spacing=8.0, replicate_physics=True
    )
    # if set, the env appends one row of episode outcomes per metrics_every steps
    metrics_csv: str = ""
    metrics_every: int = 24
    # TaskSpec fields to override; only task_spec.OVERRIDABLE (reward weights, success_radius, ...)
    spec_overrides: dict = {}


class GoalReachEnv(DirectRLEnv):
    cfg: GoalReachEnvCfg

    def __init__(self, cfg: GoalReachEnvCfg, render_mode: str | None = None, **kwargs):
        self.spec = ts.make_spec(cfg.spec_overrides)
        cfg.observation_space = ts.obs_dim(self.spec)  # wider with intent_robots
        super().__init__(cfg, render_mode, **kwargs)
        self.robot = self.scene["robot"]
        self.lidar = self.scene["lidar"]
        self.wheel_ids, _ = self.robot.find_joints(
            ["l_wheel_joint", "r_wheel_joint"], preserve_order=True
        )
        n, dev = self.num_envs, self.device
        self.cmd = torch.zeros(n, 2, device=dev)  # rate-limited (v, w) command
        self.start_xy = torch.zeros(n, 2, device=dev)
        self.goal_xy = torch.zeros(n, 2, device=dev)
        self.d_prev = torch.zeros(n, device=dev)
        self.d_now = torch.zeros(n, device=dev)
        self.ang_now = torch.zeros(n, device=dev)
        self.success = torch.zeros(n, dtype=torch.bool, device=dev)
        self.collided = torch.zeros(n, dtype=torch.bool, device=dev)
        self.oob = torch.zeros(n, dtype=torch.bool, device=dev)
        self.last_ep_len_s = torch.zeros(n, device=dev)  # length and final distance of the last
        self.last_ep_dist = torch.zeros(n, device=dev)  # finished episode, per env (for play.py)
        self.ep_reward_terms: dict[str, torch.Tensor] = {}
        # outcome counts since the last metrics row: done, success, collision, oob, timeout
        self._win = torch.zeros(5, device=dev)
        self._win_succ_time = torch.zeros((), device=dev)
        self._win_final_dist = torch.zeros((), device=dev)
        self._steps = 0
        self._t0 = time.time()
        print(f"[GOAL] contact_parts bodies: {self.scene['contact_parts'].body_names}")
        print(
            f"[GOAL] obs dim {ts.obs_dim(self.spec)}, decimation {self.cfg.decimation}, "
            f"limits {ts.ACTION_LIMITS}"
        )
        print(f"[GOAL] spec {self.spec}")

    # ---------------------------------------------------------------- sim state -> task inputs
    def robot_pose(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos = self.robot.data.root_link_pos_w.torch[:, :2] - self.scene.env_origins[:, :2]
        q = self.robot.data.root_link_quat_w.torch  # x, y, z, w
        x, y, z, w = q.unbind(-1)
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return pos, yaw

    def lidar_ranges(self) -> torch.Tensor:
        return self.lidar.data.ray_distances.torch + ts.SPEC.lidar_min_range

    def max_contact_force(self) -> torch.Tensor:
        fb = self.scene["contact_base"].data.net_forces_w.torch.norm(dim=-1).amax(dim=-1)
        fp = self.scene["contact_parts"].data.net_forces_w.torch.norm(dim=-1).amax(dim=-1)
        return torch.maximum(fb, fp)

    def body_velocity(self) -> torch.Tensor:
        return torch.stack(
            (
                self.robot.data.root_link_lin_vel_b.torch[:, 0],
                self.robot.data.root_link_ang_vel_b.torch[:, 2],
            ),
            dim=-1,
        )

    # ---------------------------------------------------------------- DirectRLEnv hooks
    def action_target(self, actions: torch.Tensor) -> torch.Tensor:
        """(v, w) target of the policy action (the store arena maps v onto a per-episode cap)."""
        return ts.action_to_target(actions)

    def reward_v_cap(self) -> torch.Tensor | None:
        """Per-env top speed for the progress reward (None: ActionLimits.v_max)."""
        return None

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.target = target = self.action_target(actions)
        self.cmd = ts.rate_limit(self.cmd, target, self.step_dt)
        self._wheel_target = ts.twist_to_wheels(self.cmd)

    def _apply_action(self) -> None:
        self.robot.set_joint_velocity_target_index(
            target=self._wheel_target, joint_ids=self.wheel_ids
        )

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy, yaw = self.robot_pose()
        self.d_now, self.ang_now = ts.goal_in_robot_frame(xy, yaw, self.goal_xy)
        self.success, self.collided, self.oob = ts.episode_outcome(
            self.d_now, self.max_contact_force(), xy, self.start_xy, self.spec
        )
        return ts.split_dones(
            self.success | self.collided | self.oob,
            self.episode_length_buf,
            self.max_episode_length,
        )

    def _get_rewards(self) -> torch.Tensor:
        rew, terms = ts.compute_reward(
            self.d_prev,
            self.d_now,
            self.success,
            self.collided | self.oob,
            self.spec,
            v_cap=self.reward_v_cap(),
        )
        self.d_prev = self.d_now.clone()
        for k, v in terms.items():
            self.ep_reward_terms.setdefault(k, torch.zeros_like(v))
            self.ep_reward_terms[k] += v
        self._count_outcomes()
        return rew

    def _count_outcomes(self) -> None:
        done = self.reset_terminated | self.reset_time_outs
        succ = self.success & done
        self._win += torch.stack(
            [
                done.sum(),
                succ.sum(),
                (self.collided & done).sum(),
                (self.oob & done).sum(),
                (self.reset_time_outs & ~self.reset_terminated).sum(),
            ]
        ).float()
        self._win_succ_time += (self.episode_length_buf.float() * self.step_dt * succ.float()).sum()
        self._win_final_dist += (self.d_now * done.float()).sum()
        self._steps += 1
        if self.cfg.metrics_csv and self._steps % self.cfg.metrics_every == 0:
            self._write_metrics()

    def _write_metrics(self) -> None:
        w = self._win.tolist()
        st, fd = float(self._win_succ_time), float(self._win_final_dist)
        n_done = max(w[0], 1.0)
        row = {
            "wall_s": round(time.time() - self._t0, 2),
            "iteration": self._steps // self.cfg.metrics_every,
            "env_steps": self._steps * self.num_envs,
            "episodes": int(w[0]),
            "success_rate": round(w[1] / n_done, 4),
            "collision_rate": round(w[2] / n_done, 4),
            "oob_rate": round(w[3] / n_done, 4),
            "timeout_rate": round(w[4] / n_done, 4),
            "success_time_s": round(st / max(w[1], 1.0), 3),
            "final_dist_m": round(fd / n_done, 4),
        }
        new = not os.path.exists(self.cfg.metrics_csv)
        with open(self.cfg.metrics_csv, "a", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(row))
            if new:
                wr.writeheader()
            wr.writerow(row)
        self._win.zero_()
        self._win_succ_time.zero_()
        self._win_final_dist.zero_()

    def sample_start(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        """Start (x, y) in the env frame and yaw for n resetting envs: cell centre, random yaw."""
        yaw = (2 * torch.rand(n, device=self.device) - 1) * torch.pi
        return torch.zeros(n, 2, device=self.device), yaw

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        self.last_ep_len_s[env_ids] = self.episode_length_buf[env_ids].float() * self.step_dt
        self.last_ep_dist[env_ids] = self.d_now[env_ids]
        # mean episode sums of the reward terms, for tensorboard
        log = self.extras.setdefault("log", {})
        for k, v in self.ep_reward_terms.items():
            log[f"Episode_Reward/{k}"] = v[env_ids].mean()
            v[env_ids] = 0.0
        super()._reset_idx(env_ids)
        n = len(env_ids)
        origins = self.scene.env_origins[env_ids]
        start, yaw = self.sample_start(n)
        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += origins
        pose[:, :2] += start
        pose[:, 3:7] = torch.stack(
            (torch.zeros_like(yaw), torch.zeros_like(yaw), torch.sin(yaw / 2), torch.cos(yaw / 2)),
            -1,
        )
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros(n, 6, device=self.device), env_ids=env_ids
        )
        jp = self.robot.data.default_joint_pos.torch[env_ids].clone()
        self.robot.write_joint_position_to_sim_index(position=jp, env_ids=env_ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(jp), env_ids=env_ids)
        self.cmd[env_ids] = 0.0
        self.start_xy[env_ids] = start
        offset = ts.sample_goal(n, self.device, self.spec)
        self.goal_xy[env_ids] = start + offset
        self.d_prev[env_ids] = offset.norm(dim=-1)
        self.d_now[env_ids] = self.d_prev[env_ids]
        self.ang_now[env_ids] = torch.atan2(offset[:, 1], offset[:, 0]) - yaw
        self.ang_now[env_ids] = torch.atan2(
            torch.sin(self.ang_now[env_ids]), torch.cos(self.ang_now[env_ids])
        )

    def _get_observations(self) -> dict:
        obs = ts.build_observation(
            self.lidar_ranges(), self.d_now, self.ang_now, self.body_velocity(), self.spec
        )
        return {"policy": obs}
