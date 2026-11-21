"""Goal-reach task definition that does not depend on any simulator.

Everything here is plain PyTorch on batched tensors (first dim = env), so the Isaac Lab env and a
future 2D PyTorch simulator can share the same observation, action, reward, success and collision
definitions. The simulator only has to supply: robot (x, y, yaw), body-frame (v, w), the 360 lidar
ranges, and the largest contact force on the bodies listed in ``COLLISION_BODIES``.

Frames: world x/y on the floor, yaw counter-clockwise from +x. Lidar ray i points at
(-180 + i) deg in the robot base frame (ray 180 = straight ahead), as set up in
``learning/sim_check/pinky_newton_sensors.py``.
"""

from __future__ import annotations

import dataclasses
import math
import os
from dataclasses import dataclass

import torch
import yaml

# One top speed for every Pode limit (user decision 2026-11-22): the Nav2 package's file.
NAV_SPEED_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "pode", "pode_nav_config",
    "params", "nav_speed.yaml"
)
# Episode lengths and the waits that depend on robot travel were tuned at this speed.
TIMES_TUNED_AT_MPS = 0.25


def nav_speed_mps(path: str = NAV_SPEED_FILE) -> float:
    with open(path, encoding="utf-8") as f:
        value = float(yaml.safe_load(f)["nav_speed_mps"])
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{path}: nav_speed_mps must be above 0, got {value!r}")
    return value


NAV_SPEED_MPS = nav_speed_mps()


def travel_time(seconds_at_tuned_speed: float, speed: float = NAV_SPEED_MPS) -> float:
    """A time tuned at TIMES_TUNED_AT_MPS, stretched so the robot covers the same distance."""
    return seconds_at_tuned_speed * TIMES_TUNED_AT_MPS / speed


@dataclass(frozen=True)
class ActionLimits:
    """The one place for command limits. The env, the action mapping and any 2D sim read this."""

    v_min: float = (
        -0.05
    )  # m/s; small reverse, the speed of the Nav2 retreat (BackUp) on the real robot
    v_max: float = NAV_SPEED_MPS  # m/s; pode_nav_config/params/nav_speed.yaml
    w_max: float = 1.0  # rad/s; Gazebo DiffDrive max_velocity, Nav2 max_rotational_vel
    acc_v: float = 2.0  # m/s^2; Gazebo DiffDrive max_acceleration
    acc_w: float = 2.0  # rad/s^2; same Gazebo limit (the plugin applies it to both)


@dataclass(frozen=True)
class RobotGeometry:
    wheel_radius: float = 0.028  # m, URDF collision sphere
    wheel_separation: float = 0.0961  # m, real bringup and Gazebo value (user decision 2026-11-21)


@dataclass(frozen=True)
class TaskSpec:
    control_dt: float = 0.05  # s, 20 Hz policy
    # s; straight 2 m at 0.25 m/s plus a half turn is about 11 s: 20 s at 0.25 m/s
    episode_s: float = travel_time(20.0)
    goal_dist_range: tuple[float, float] = (1.0, 2.0)  # m from the start
    success_radius: float = 0.05  # m; same as Nav2 xy_goal_tolerance on the real robot
    out_of_bounds: float = (
        3.0  # m from the start: the episode fails (keeps robots in their own cell)
    )
    # lidar
    lidar_rays: int = 360
    lidar_sectors: int = 36  # min over 10 deg sectors
    lidar_min_range: float = 0.05  # m; rays start this far out (RPLIDAR C1 minimum is also 0.05 m)
    lidar_obs_max: float = 2.0  # m; ranges are clipped here before scaling to [0, 1]
    goal_obs_max: float = 3.0  # m; goal distance is clipped here before scaling
    # fleet intent (store arena stage 4c): for the intent_robots nearest other robots, a mask and
    # intent_points route points intent_spacing_m apart ahead of each, in our robot frame, / and
    # clipped at intent_obs_max. 0 robots = the 41-dim layout of the goal-reach, 4a and 4b policies
    intent_robots: int = 0
    intent_points: int = 3
    intent_spacing_m: float = 0.2  # m along the robot's route (what fleet_manager can publish)
    intent_obs_max: float = 2.0  # m, as the lidar
    # collision: largest contact-force norm on COLLISION_BODIES above this ends the episode
    collision_force: float = 1.0  # N; the robot weighs 15 N, a wall push measured 5-11 N
    # reward weights (per 20 Hz step)
    w_progress: float = 1.0  # times (d_prev - d_now) / (v_max * dt): +1 per step at full speed
    w_time: float = -0.05  # every step
    w_arrival: float = 10.0  # once, on success
    w_collision: float = -10.0  # once, on collision or out of bounds


# Bodies whose contact counts as a collision. Wheels and caster are excluded (they touch the floor).
# screen_mount is the front-most collider and touches a wall first (newton_pinky_check summary).
COLLISION_BODIES = ("base_link", "screen_mount", "front_camera_mount", "rplidar_mount")

ACTION_LIMITS = ActionLimits()
GEOMETRY = RobotGeometry()
SPEC = TaskSpec()

# TaskSpec fields an experiment may change (train.py --set). The rest shape the env itself
# (decimation, episode length, sensor, observation size) and are read once at import.
OVERRIDABLE = (
    "goal_dist_range",
    "success_radius",
    "out_of_bounds",
    "goal_obs_max",
    "collision_force",
    "w_progress",
    "w_time",
    "w_arrival",
    "w_collision",
    # these change the observation size: an env built with them sets its observation_space from
    # obs_dim(spec), and a policy only runs on the layout it was trained with
    "intent_robots",
    "intent_points",
    "intent_spacing_m",
    "intent_obs_max",
)


def parse_overrides(items, spec=SPEC) -> dict:
    """FIELD=VALUE strings from --set, typed by the field's default in ``spec`` (a dataclass).
    A tuple field takes comma-separated numbers (a,b); a str field keeps the text."""
    out = {}
    for item in items:
        key, text = item.split("=", 1)
        default = getattr(spec, key, None)
        if isinstance(default, str):
            out[key] = text
        elif isinstance(default, int) and not isinstance(default, bool):
            out[key] = int(float(text))
        elif isinstance(default, tuple):
            values = tuple(float(x) for x in text.split(","))
            if len(values) != len(default):
                raise ValueError(
                    f"{key} takes {len(default)} comma-separated numbers, got {text!r}"
                )
            out[key] = values
        else:
            out[key] = float(text)
    return out


def split_dones(outcome_done: torch.Tensor, length: torch.Tensor, max_length: int):
    """(terminated, time_out). An episode that ends by success, collision or out of bounds on
    the last step is terminated only: PPO bootstraps the value on time_out."""
    time_out = (length >= max_length) & ~outcome_done
    return outcome_done, time_out


def make_spec(overrides: dict) -> TaskSpec:
    bad = sorted(set(overrides) - set(OVERRIDABLE))
    if bad:
        raise ValueError(f"TaskSpec fields {bad} cannot be overridden; allowed: {OVERRIDABLE}")
    return dataclasses.replace(SPEC, **overrides)


OBS_LIDAR = SPEC.lidar_sectors


def obs_dim(spec: TaskSpec = SPEC) -> int:
    """lidar sectors, goal (dist, sin, cos), (v, w), then per intent robot a mask and its points."""
    return spec.lidar_sectors + 3 + 2 + spec.intent_robots * (1 + 2 * spec.intent_points)


def obs_names(spec: TaskSpec = SPEC) -> list[str]:
    """Name of every observation entry, in order (for the policy export's policy_spec.json)."""
    names = [f"lidar_sector_{i}" for i in range(spec.lidar_sectors)]
    names += ["goal_dist", "goal_sin", "goal_cos", "v", "w"]
    for r in range(spec.intent_robots):
        names.append(f"robot{r}_present")
        for k in range(spec.intent_points):
            names += [f"robot{r}_p{k + 1}_x", f"robot{r}_p{k + 1}_y"]
    return names


OBS_DIM = obs_dim(SPEC)  # 41
ACT_DIM = 2


def action_to_target(actions: torch.Tensor, lim: ActionLimits = ACTION_LIMITS) -> torch.Tensor:
    """Policy action in [-1, 1]^2 -> target (v, w). v maps affinely onto [v_min, v_max]."""
    a = actions.clamp(-1.0, 1.0)
    v = lim.v_min + (a[:, 0] + 1.0) * 0.5 * (lim.v_max - lim.v_min)
    w = a[:, 1] * lim.w_max
    return torch.stack((v, w), dim=-1)


def rate_limit(
    cmd: torch.Tensor, target: torch.Tensor, dt: float, lim: ActionLimits = ACTION_LIMITS
) -> torch.Tensor:
    """Move the commanded (v, w) toward the target by at most (acc_v, acc_w) * dt."""
    step = torch.tensor((lim.acc_v * dt, lim.acc_w * dt), device=cmd.device, dtype=cmd.dtype)
    return cmd + (target - cmd).clamp(-step, step)


def twist_to_wheels(cmd: torch.Tensor, geo: RobotGeometry = GEOMETRY) -> torch.Tensor:
    """(v, w) -> (left, right) wheel speeds in rad/s."""
    v, w = cmd[:, 0], cmd[:, 1]
    half = 0.5 * geo.wheel_separation
    return torch.stack(((v - w * half) / geo.wheel_radius, (v + w * half) / geo.wheel_radius), -1)


def goal_in_robot_frame(
    robot_xy: torch.Tensor, robot_yaw: torch.Tensor, goal_xy: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """Distance [m] and bearing [rad, (-pi, pi]] of the goal seen from the robot."""
    d = goal_xy - robot_xy
    dist = d.norm(dim=-1)
    ang = torch.atan2(d[:, 1], d[:, 0]) - robot_yaw
    ang = torch.atan2(torch.sin(ang), torch.cos(ang))
    return dist, ang


def lidar_sectors(ranges: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """(N, 360) ranges [m] -> (N, sectors) minimum range per sector, clipped to lidar_obs_max.

    Non-finite values (no hit) count as lidar_obs_max."""
    r = torch.nan_to_num(ranges, nan=spec.lidar_obs_max, posinf=spec.lidar_obs_max)
    r = r.clamp(max=spec.lidar_obs_max)
    return r.view(r.shape[0], spec.lidar_sectors, -1).min(dim=-1).values


def build_observation(
    lidar_ranges: torch.Tensor,
    goal_dist: torch.Tensor,
    goal_ang: torch.Tensor,
    vel: torch.Tensor,
    spec: TaskSpec = SPEC,
    lim: ActionLimits = ACTION_LIMITS,
    intent: torch.Tensor | None = None,
    intent_mask: torch.Tensor | None = None,
) -> torch.Tensor:
    """Observation vector, every entry roughly in [-1, 1]:
    [lidar sector / lidar_obs_max (36), goal dist / goal_obs_max, sin(bearing), cos(bearing),
     v / v_max, w / w_max], then for each of spec.intent_robots robots (nearest first):
    [present (0/1), x1, y1, ..., xP, yP] with the route points (intent (N, R, P, 2), our robot
    frame, m) / intent_obs_max clipped to [-1, 1]; an absent robot (intent_mask (N, R) false) is
    all zeros, and so is every robot when intent is None (an env without fleet robots)."""
    lid = lidar_sectors(lidar_ranges, spec) / spec.lidar_obs_max
    g = torch.stack(
        (
            goal_dist.clamp(max=spec.goal_obs_max) / spec.goal_obs_max,
            torch.sin(goal_ang),
            torch.cos(goal_ang),
        ),
        dim=-1,
    )
    v = torch.stack((vel[:, 0] / lim.v_max, vel[:, 1] / lim.w_max), dim=-1)
    parts = [lid, g, v]
    if spec.intent_robots and intent is None:
        parts.append(lid.new_zeros(lid.shape[0], obs_dim(spec) - OBS_DIM))
    elif spec.intent_robots:
        m = intent_mask[:, : spec.intent_robots].float()
        p = (intent[:, : spec.intent_robots] / spec.intent_obs_max).clamp(-1.0, 1.0)
        p = p.flatten(2) * m[..., None]
        parts.append(torch.cat((m[..., None], p), -1).flatten(1))
    return torch.cat(parts, dim=-1)


def is_success(goal_dist: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    return goal_dist < spec.success_radius


def is_collision(max_contact_force: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """max_contact_force: (N,) largest force norm over COLLISION_BODIES."""
    return max_contact_force > spec.collision_force


def is_out_of_bounds(robot_xy: torch.Tensor, start_xy: torch.Tensor, spec: TaskSpec = SPEC):
    return (robot_xy - start_xy).norm(dim=-1) > spec.out_of_bounds


def episode_outcome(
    goal_dist: torch.Tensor,
    max_contact_force: torch.Tensor,
    robot_xy: torch.Tensor,
    start_xy: torch.Tensor,
    spec: TaskSpec = SPEC,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """(success, collision, out_of_bounds), at most one true per env. Failure wins: reaching the
    goal while colliding is a collision, and out of bounds while colliding is a collision.
    A timeout on the same step as success is the env's business (it counts success first)."""
    collision = is_collision(max_contact_force, spec)
    oob = is_out_of_bounds(robot_xy, start_xy, spec) & ~collision
    success = is_success(goal_dist, spec) & ~collision & ~oob
    return success, collision, oob


def compute_reward(
    d_prev: torch.Tensor,
    d_now: torch.Tensor,
    success: torch.Tensor,
    failed: torch.Tensor,
    spec: TaskSpec = SPEC,
    lim: ActionLimits = ACTION_LIMITS,
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    """Per-step reward and its terms. failed = collision or out of bounds (not timeout)."""
    terms = {
        "progress": spec.w_progress * (d_prev - d_now) / (lim.v_max * spec.control_dt),
        "time": torch.full_like(d_now, spec.w_time),
        "arrival": spec.w_arrival * success.float(),
        "collision": spec.w_collision * failed.float(),
    }
    return sum(terms.values()), terms


def sample_goal(n: int, device, spec: TaskSpec = SPEC) -> torch.Tensor:
    """Goal offsets from the start: distance uniform in goal_dist_range, bearing uniform."""
    lo, hi = spec.goal_dist_range
    r = lo + (hi - lo) * torch.rand(n, device=device)
    th = (2 * torch.rand(n, device=device) - 1) * math.pi
    return torch.stack((r * torch.cos(th), r * torch.sin(th)), dim=-1)
