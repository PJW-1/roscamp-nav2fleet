"""시뮬레이터에 의존하지 않는 goal-reach 과제 정의: 관측, 행동, 보상, 성공, 충돌.

배치 텐서(첫 차원 = env)로만 쓰여 Isaac Lab env 와 2D 시뮬이 같은 정의를 공유한다.
좌표: 바닥 x/y, yaw 는 +x 에서 반시계. 라이다 레이 i 는 base 프레임 (-180 + i) deg (180 = 정면).
"""

from __future__ import annotations

import dataclasses
import math
import os
from dataclasses import dataclass

import torch
import yaml

# 모든 Pode 한도의 최고속도: Nav2 패키지 파일
NAV_SPEED_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "pode", "pode_nav_config",
    "params", "nav_speed.yaml"
)
# 에피소드 길이와 이동 시간 기반 대기는 이 속도 기준으로 맞춤
TIMES_TUNED_AT_MPS = 0.25


def nav_speed_mps(path: str = NAV_SPEED_FILE) -> float:
    with open(path, encoding="utf-8") as f:
        value = float(yaml.safe_load(f)["nav_speed_mps"])
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{path}: nav_speed_mps must be above 0, got {value!r}")
    return value


NAV_SPEED_MPS = nav_speed_mps()


def travel_time(seconds_at_tuned_speed: float, speed: float = NAV_SPEED_MPS) -> float:
    """맞춘 속도 기준 시간을 현재 속도에서 같은 거리를 가도록 늘린다."""
    return seconds_at_tuned_speed * TIMES_TUNED_AT_MPS / speed


@dataclass(frozen=True)
class ActionLimits:
    """명령 한도 한 곳. env, 행동 매핑, 2D 시뮬이 함께 읽는다."""

    v_min: float = (
        -0.05
    )  # m/s; 소폭 후진 (실로봇 Nav2 BackUp 속도)
    v_max: float = NAV_SPEED_MPS  # m/s; pode_nav_config/params/nav_speed.yaml
    w_max: float = 1.0  # rad/s
    acc_v: float = 2.0  # m/s^2
    acc_w: float = 2.0  # rad/s^2


@dataclass(frozen=True)
class RobotGeometry:
    wheel_radius: float = 0.028  # m
    wheel_separation: float = 0.0961  # m, 실로봇 bringup 과 Gazebo 값


@dataclass(frozen=True)
class TaskSpec:
    control_dt: float = 0.05  # s, 20 Hz 정책
    episode_s: float = travel_time(20.0)  # 0.25 m/s 로 20 s 거리
    goal_dist_range: tuple[float, float] = (1.0, 2.0)  # 시작점에서 m
    success_radius: float = 0.05  # m; 실로봇 Nav2 xy_goal_tolerance 와 같음
    out_of_bounds: float = (
        3.0  # 시작점에서 m: 넘으면 실패 (로봇을 자기 칸에 가둠)
    )
    # ---- 라이다 ----
    lidar_rays: int = 360
    lidar_sectors: int = 36  # 10 deg 섹터별 최솟값
    lidar_min_range: float = 0.05  # m; 레이 시작 거리 (RPLIDAR C1 최소와 같음)
    lidar_obs_max: float = 2.0  # m; [0, 1] 로 스케일하기 전 클리핑
    goal_obs_max: float = 3.0  # m; 목표 거리 클리핑
    # ---- 플릿 intent (4c): 가까운 intent_robots 대마다 존재 마스크와 경로점 intent_points 개 ----
    # 0 대 = goal-reach, 4a, 4b 정책의 41차원 배치
    intent_robots: int = 0
    intent_points: int = 3
    intent_spacing_m: float = 0.2  # 경로점 간격 [m]
    intent_obs_max: float = 2.0  # m, 라이다와 같음
    # 1: 존재 플래그 뒤에 우선권 플래그 추가 (그 로봇이 우리보다 우선)
    intent_priority: int = 0
    # ---- 선택 관측 블록: 기본 41개와 intent 블록 사이에 이 순서로 (모두 끄면 ppo_k 배치) ----
    # prev_action_obs 1: 직전 제어 스텝의 자른 목표 (v, w) / (v_max, w_max), 에피소드 시작에는 0
    prev_action_obs: int = 0
    # lidar_frames > 1: 이전 섹터 벡터 frames - 1 개 (같은 배율, 최신 먼저, lidar_stride_steps 간격)
    lidar_frames: int = 1
    lidar_stride_steps: int = 4
    # speed_cap_obs 1: 에피소드 최고 선속도 / v_max. 상한은 speed_cap_range 에서 에피소드마다 뽑음
    # v 행동 상한, 에피소드 시간, 스크립트 로봇 속도가 상한을 따름. v 관측은 그대로 / v_max
    speed_cap_obs: int = 0
    speed_cap_range: tuple[float, float] = (NAV_SPEED_MPS, NAV_SPEED_MPS)  # m/s
    # critic_privileged 1: 크리틱(액터 아님)이 움직이는 물체와 스크립트 로봇의 실제 위치, 속도도
    # 받음
    critic_privileged: int = 0
    # ---- 충돌: COLLISION_BODIES 의 최대 접촉력이 이를 넘으면 종료 ----
    collision_force: float = 1.0  # N; 로봇 무게 15 N, 벽 밀기 5-11 N
    # ---- 보상 가중치 (20 Hz 스텝당) ----
    w_progress: float = 1.0  # (d_prev - d_now) / (v_max * dt) 배: 최고속도에서 스텝당 +1
    w_time: float = -0.05  # 매 스텝
    w_arrival: float = 10.0  # 성공 시 한 번
    w_collision: float = -10.0  # 충돌 또는 범위 이탈 시 한 번
    # 부드러움: 정규화한 (v, w) 목표의 스텝당 변화 제곱, w 목표 부호 반전마다 곱함
    w_action_rate: float = 0.0
    w_w_flip: float = 0.0
    flip_deadband: float = 0.02  # rad/s; 부호 반전을 셀 때 |w| 가 이보다 작으면 건너뜀


# 접촉하면 충돌로 치는 몸체 (바퀴와 캐스터는 바닥에 닿으므로 제외)
COLLISION_BODIES = ("base_link", "screen_mount", "front_camera_mount", "rplidar_mount")

ACTION_LIMITS = ActionLimits()
GEOMETRY = RobotGeometry()
SPEC = TaskSpec()

# 실험에서 바꿀 수 있는 TaskSpec 필드 (train.py --set), 나머지는 import 때 한 번 읽음
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
    # 관측 크기가 바뀜: 정책은 학습한 배치에서만 동작
    "intent_robots",
    "intent_points",
    "intent_spacing_m",
    "intent_obs_max",
    "intent_priority",
    "prev_action_obs",
    "lidar_frames",
    "lidar_stride_steps",
    "speed_cap_obs",
    "speed_cap_range",
    "critic_privileged",
    "w_action_rate",
    "w_w_flip",
    "flip_deadband",
)


def parse_overrides(items, spec=SPEC) -> dict:
    """--set 의 FIELD=VALUE 문자열을 spec 필드 기본값 타입으로 변환 (튜플은 a,b)."""
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
    """(terminated, time_out). 마지막 스텝의 성공/충돌/이탈은 terminated 만 (PPO 가 time_out 에서 가치를 부트스트랩)."""
    time_out = (length >= max_length) & ~outcome_done
    return outcome_done, time_out


def make_spec(overrides: dict) -> TaskSpec:
    bad = sorted(set(overrides) - set(OVERRIDABLE))
    if bad:
        raise ValueError(f"TaskSpec fields {bad} cannot be overridden; allowed: {OVERRIDABLE}")
    return dataclasses.replace(SPEC, **overrides)


OBS_LIDAR = SPEC.lidar_sectors


BASE_DIM = OBS_LIDAR + 3 + 2  # 라이다 섹터, 목표 (거리, sin, cos), (v, w): ppo_k 배치


def obs_blocks(spec: TaskSpec = SPEC) -> list[str]:
    """기본 항목 뒤에 있는 선택 블록 이름 (정해진 순서)."""
    out = []
    if spec.prev_action_obs:
        out.append("prev_action")
    if spec.lidar_frames > 1:
        out.append("lidar_history")
    if spec.speed_cap_obs:
        out.append("speed_cap")
    if spec.intent_robots:
        out.append("intent")
    return out


def obs_dim(spec: TaskSpec = SPEC) -> int:
    """기본 항목, 이어서 prev_action (2), lidar_history (섹터 x (frames - 1)), speed_cap (1),
    intent 로봇마다 마스크, 우선순위 플래그(선택), 경로점."""
    per = 1 + int(spec.intent_priority > 0) + 2 * spec.intent_points
    return (
        BASE_DIM
        + 2 * int(spec.prev_action_obs > 0)
        + spec.lidar_sectors * max(spec.lidar_frames - 1, 0)
        + int(spec.speed_cap_obs > 0)
        + spec.intent_robots * per
    )


def obs_names(spec: TaskSpec = SPEC) -> list[str]:
    """관측 항목 이름 순서 (정책 내보내기 policy_spec.json 용)."""
    names = [f"lidar_sector_{i}" for i in range(spec.lidar_sectors)]
    names += ["goal_dist", "goal_sin", "goal_cos", "v", "w"]
    if spec.prev_action_obs:
        names += ["prev_v", "prev_w"]
    for f in range(1, spec.lidar_frames):
        names += [f"lidar_hist{f}_sector_{i}" for i in range(spec.lidar_sectors)]
    if spec.speed_cap_obs:
        names.append("speed_cap")
    for r in range(spec.intent_robots):
        names.append(f"robot{r}_present")
        if spec.intent_priority:
            names.append(f"robot{r}_priority")
        for k in range(spec.intent_points):
            names += [f"robot{r}_p{k + 1}_x", f"robot{r}_p{k + 1}_y"]
    return names


OBS_DIM = obs_dim(SPEC)  # 41
ACT_DIM = 2


def action_to_target(
    actions: torch.Tensor, lim: ActionLimits = ACTION_LIMITS, v_cap: torch.Tensor | None = None
) -> torch.Tensor:
    """정책 행동 [-1, 1]^2 를 목표 (v, w) 로. v 는 [v_min, v_max] 로 선형 변환 (env 별 v_cap (N,) 이
    있으면 [v_min, v_cap])."""
    a = actions.clamp(-1.0, 1.0)
    top = lim.v_max if v_cap is None else v_cap
    v = lim.v_min + (a[:, 0] + 1.0) * 0.5 * (top - lim.v_min)
    w = a[:, 1] * lim.w_max
    return torch.stack((v, w), dim=-1)


def still_action(n: int, device=None, lim: ActionLimits = ACTION_LIMITS, v_cap=None):
    """목표가 (0, 0) 인 (n, 2) 행동: 로봇이 서 있음 (item 7 상한 실행용)."""
    top = lim.v_max if v_cap is None else v_cap
    a0 = 2.0 * (0.0 - lim.v_min) / (top - lim.v_min) - 1.0
    out = torch.zeros(n, 2, device=device)
    out[:, 0] = a0
    return out


def capped_action(actions: torch.Tensor, v_cap: torch.Tensor, lim: ActionLimits = ACTION_LIMITS):
    """speed_cap_obs 없이 학습한 정책의 행동을 v 를 [v_min, v_cap] 로 맞추는 env 용으로 바꿈.
    목표 v 는 lim.v_max 기준 값을 v_cap 으로 자른 것 (낮은 최고속도에서 플러그인이 형식 2 정책에
    하는 일)."""
    a = actions.clamp(-1.0, 1.0)
    v = lim.v_min + (a[:, 0] + 1.0) * 0.5 * (lim.v_max - lim.v_min)
    v = torch.minimum(v, v_cap)
    out = a.clone()
    out[:, 0] = 2.0 * (v - lim.v_min) / (v_cap - lim.v_min) - 1.0
    return out


def sample_speed_cap(n: int, device, spec: TaskSpec = SPEC) -> torch.Tensor:
    """에피소드별 최고속도 (n,), speed_cap_range 균등 (한 값이면 뽑지 않음)."""
    lo, hi = spec.speed_cap_range
    if lo == hi:
        return torch.full((n,), float(lo), device=device)
    return lo + (hi - lo) * torch.rand(n, device=device)


def flip_step(w: torch.Tensor, last_sign: torch.Tensor, deadband: float):
    """모든 env 에 tools/nav_bench sign_changes 한 스텝: (반전 여부 (N,), 새 last_sign).
    |w| < deadband 는 건너뜀, last_sign 0 은 에피소드에서 아직 부호를 못 봄."""
    s = torch.sign(w) * (w.abs() >= deadband)
    flipped = (s != 0) & (last_sign != 0) & (s != last_sign)
    return flipped, torch.where(s != 0, s, last_sign)


def action_rate_terms(
    target: torch.Tensor,
    prev_target: torch.Tensor,
    flipped: torch.Tensor,
    spec: TaskSpec = SPEC,
    lim: ActionLimits = ACTION_LIMITS,
) -> dict[str, torch.Tensor]:
    """부드러움 보상 항: w_action_rate x (v / v_max, w / w_max) 변화 제곱, w 목표 반전마다
    w_w_flip."""
    scale = torch.tensor((lim.v_max, lim.w_max), device=target.device)
    d = ((target - prev_target) / scale).square().sum(-1)
    return {"action_rate": spec.w_action_rate * d, "w_flip": spec.w_w_flip * flipped.float()}


PRIV_POS_M = 2.0  # m; 특권 위치를 이 값으로 나눔 (라이다 범위와 같음)
PRIV_VEL_MPS = 0.5  # m/s; 특권 속도를 이 값으로 나눔
PRIV_PER_OBJECT = 5  # 존재, x, y, vx, vy


def privileged_block(
    pos: torch.Tensor, vel: torch.Tensor, present: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor
) -> torch.Tensor:
    """크리틱 전용 항목 (critic_privileged): 물체마다 (움직이는 물체, 이어서 스크립트 로봇)
    우리 로봇 프레임의 [존재, x, y, vx, vy] / PRIV_POS_M·PRIV_VEL_MPS, [-1, 1] 로 자름. 없는 물체는
    0."""
    c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    d = pos - xy[:, None]
    p = torch.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]), -1)
    v = torch.stack((c * vel[..., 0] + s * vel[..., 1], -s * vel[..., 0] + c * vel[..., 1]), -1)
    m = present.float()[..., None]
    p = (p / PRIV_POS_M).clamp(-1, 1) * m
    v = (v / PRIV_VEL_MPS).clamp(-1, 1) * m
    return torch.cat((m, p, v), -1).flatten(1)


def rate_limit(
    cmd: torch.Tensor, target: torch.Tensor, dt: float, lim: ActionLimits = ACTION_LIMITS
) -> torch.Tensor:
    """명령 (v, w) 를 목표로 (acc_v, acc_w) * dt 이내로만 옮긴다."""
    step = torch.tensor((lim.acc_v * dt, lim.acc_w * dt), device=cmd.device, dtype=cmd.dtype)
    return cmd + (target - cmd).clamp(-step, step)


def twist_to_wheels(cmd: torch.Tensor, geo: RobotGeometry = GEOMETRY) -> torch.Tensor:
    """(v, w) -> (왼, 오른) 바퀴 속도 rad/s."""
    v, w = cmd[:, 0], cmd[:, 1]
    half = 0.5 * geo.wheel_separation
    return torch.stack(((v - w * half) / geo.wheel_radius, (v + w * half) / geo.wheel_radius), -1)


def goal_in_robot_frame(
    robot_xy: torch.Tensor, robot_yaw: torch.Tensor, goal_xy: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """로봇에서 본 목표의 거리 [m] 와 방위 [rad]."""
    d = goal_xy - robot_xy
    dist = d.norm(dim=-1)
    ang = torch.atan2(d[:, 1], d[:, 0]) - robot_yaw
    ang = torch.atan2(torch.sin(ang), torch.cos(ang))
    return dist, ang


def lidar_sectors(ranges: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """(N, 360) 거리를 섹터별 최솟값으로 줄이고 lidar_obs_max 로 자른다 (비유한값은 lidar_obs_max)."""
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
    intent_priority: torch.Tensor | None = None,
    prev_target: torch.Tensor | None = None,
    lidar_history: torch.Tensor | None = None,
    speed_cap: torch.Tensor | None = None,
) -> torch.Tensor:
    """관측 벡터, 각 항목 대략 [-1, 1]:
    [라이다 섹터 / lidar_obs_max (36), 목표 거리 / goal_obs_max, sin, cos, v / v_max, w / w_max],
    이어서 intent 로봇마다 [존재 (0/1), 경로점 x, y ...] (우리 프레임 m / intent_obs_max).
    없는 로봇과 intent 없는 env 는 모두 0.
    선택 블록(obs_blocks)은 (v, w) 와 intent 사이: prev_target (N, 2) m/s·rad/s, lidar_history
    (N, frames - 1, sectors) 현재 섹터와 같은 배율·최신 먼저, speed_cap (N,) m/s.
    입력이 없으면 각각 0, 현재 프레임, v_max 로 채움."""
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
    n = lid.shape[0]
    if spec.prev_action_obs:
        p = lid.new_zeros(n, 2) if prev_target is None else prev_target
        parts.append(torch.stack((p[:, 0] / lim.v_max, p[:, 1] / lim.w_max), -1))
    if spec.lidar_frames > 1:
        h = lidar_history
        if h is None:
            h = lid[:, None].expand(n, spec.lidar_frames - 1, -1)
        parts.append(h.reshape(n, -1))
    if spec.speed_cap_obs:
        cap = lid.new_full((n,), lim.v_max) if speed_cap is None else speed_cap
        parts.append((cap / lim.v_max)[:, None])
    per = 1 + int(spec.intent_priority > 0) + 2 * spec.intent_points
    if spec.intent_robots and intent is None:
        parts.append(lid.new_zeros(n, spec.intent_robots * per))
    elif spec.intent_robots:
        m = intent_mask[:, : spec.intent_robots].float()
        p = (intent[:, : spec.intent_robots] / spec.intent_obs_max).clamp(-1.0, 1.0)
        p = p.flatten(2) * m[..., None]
        head = [m[..., None]]
        if spec.intent_priority:  # (N, R) bool; 없는 로봇은 0
            prio = torch.zeros_like(m) if intent_priority is None else intent_priority.float()
            head.append((prio[:, : spec.intent_robots] * m)[..., None])
        parts.append(torch.cat((*head, p), -1).flatten(1))
    return torch.cat(parts, dim=-1)


def is_success(goal_dist: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    return goal_dist < spec.success_radius


def is_collision(max_contact_force: torch.Tensor, spec: TaskSpec = SPEC) -> torch.Tensor:
    """max_contact_force: COLLISION_BODIES 의 최대 접촉력 노름 (N,)."""
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
    """(성공, 충돌, 범위 이탈), env 마다 최대 하나. 실패가 우선하고 같은 스텝의 시간초과는 env 가 성공으로 센다."""
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
    v_cap: torch.Tensor | None = None,
) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    """스텝 보상과 항별 값. failed = 충돌 또는 이탈 (시간초과 제외).
    진행 보상은 최고속도(v_max 또는 env 별 v_cap)에서 스텝당 +1."""
    top = lim.v_max if v_cap is None else v_cap
    terms = {
        "progress": spec.w_progress * (d_prev - d_now) / (top * spec.control_dt),
        "time": torch.full_like(d_now, spec.w_time),
        "arrival": spec.w_arrival * success.float(),
        "collision": spec.w_collision * failed.float(),
    }
    return sum(terms.values()), terms


def sample_goal(n: int, device, spec: TaskSpec = SPEC) -> torch.Tensor:
    """시작점 기준 목표 오프셋: 거리 균등(goal_dist_range), 방위 균등."""
    lo, hi = spec.goal_dist_range
    r = lo + (hi - lo) * torch.rand(n, device=device)
    th = (2 * torch.rand(n, device=device) - 1) * math.pi
    return torch.stack((r * torch.cos(th), r * torch.sin(th)), dim=-1)
