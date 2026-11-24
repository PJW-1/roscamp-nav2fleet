"""매장 아레나 정의(시뮬레이터 없이): 설정, 플릿 맵의 박스, 2D 레이캐스트.

아레나는 실행 시 플릿 맵 yaml 로 만들고 맵 (0, 0) 이 env 원점이다. sim_shelves 는 읽지 않는다.
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from dataclasses import dataclass

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
for _p in (
    os.path.join(REPO, "pc", "fleet_core"),
    os.path.join(REPO, "sim", "fleet_gz_sim"),
    os.path.join(HERE, "..", "goal_reach"),
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import task_spec as ts  # noqa: E402
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_gz_sim import arena as gz_arena  # noqa: E402

Box = gz_arena.Box  # name, x0, y0, x1, y1, kind, height_m


@dataclass(frozen=True)
class ArenaSpec:
    """매장 아레나 설정 한 곳. 모든 필드는 --set 으로 바꾼다."""

    layout: str = "v7"  # 맵 이름 또는 yaml 경로
    wall_height_m: float = gz_arena.WALL_HEIGHT_M
    shelf_height_m: float = 0.30  # 로봇 전체보다 높게
    # 아레나 칸 간격 [m]
    env_spacing: float = 3.0
    # arena_check.py 가 로봇을 손으로 놓을 때 위치
    start_x: float = 0.30
    start_y: float = 0.55
    # 에피소드 시간: 0.25 m/s 로 30 m 거리
    episode_s: float = ts.travel_time(30.0)
    # ---- 시작/목표 (route_spec.hall_nodes) ----
    excluded_node_kinds: str = "parking"  # 제외할 노드 종류
    min_start_goal_m: float = 0.40  # 시작-목표 직선거리 최소
    # ---- 전역 경로 (Nav2 NavFn 식) ----
    grid_m: float = 0.025  # 전역 코스트맵 해상도
    inscribed_m: float = 0.07
    inflation_m: float = 0.15
    cost_scaling: float = 15.0
    lookahead_m: float = 0.40  # 관측 목표: 로봇 앞 경로상 거리
    follow_window: int = 20  # 스텝당 진행 인덱스가 건너뛸 수 있는 점 수
    # ---- 정적 장애물 (4a): 슬롯마다 크기가 다른 정사각 박스 ----
    obstacle_sizes: tuple[float, float, float] = (0.10, 0.15, 0.20)  # 한 변 [m]
    obstacle_height_m: float = 0.30
    obstacle_max: int = 3  # 에피소드당 박스 수, 0..obstacle_max 균등
    # 시작-목표 쌍당 배치 수 (배치 0 은 박스 없음)
    obstacle_layouts: int = 96
    obstacle_on_path_prob: float = 0.7  # 박스를 경로 위에 둘 확률, 아니면 아무 데나
    obstacle_lateral_m: float = 0.08  # 경로 위 박스의 옆 이동
    obstacle_node_clear_m: float = 0.25  # 박스 가장자리와 시작/목표 노드 간격
    route_seed: int = 0  # 배치는 env 생성 때 이 시드로 한 번 뽑음
    # ---- 이동 장애물 (4b, mover_spec.MOVERS), 리셋마다 뽑음 ----
    mover_prob: float = 0.75  # 각 mover 가 에피소드에 나올 확률
    mover_on_path_prob: float = 0.6  # 왕복 구간을 전역 경로 중심에 둘 확률
    mover_leg_m: tuple[float, float] = (0.3, 0.8)  # 왕복 구간 길이 범위
    mover_speed_scale: tuple[float, float] = (0.5, 1.5)  # 기준 속도 배율
    mover_pause_s: tuple[float, float] = (0.0, 1.0)  # 구간 양끝 정지 시간
    mover_node_clear_m: float = 0.25  # 구간 전체가 시작/목표에서 떨어질 거리
    # mover 표면과 경로 사이 최소 거리 (경로를 순찰하면 시간초과가 늘어남)
    mover_end_path_clear_m: float = 0.07
    # 다음 스텝에 로봇 중심과 이 거리보다 가까워지면 mover 는 멈춰 기다림
    mover_yield_m: float = 0.15
    # 이만큼 기다리면 돌아섬 (서로 기다리는 교착 방지)
    mover_wait_s: float = ts.travel_time(2.0)
    # 절대 안 기다리는 mover 에피소드 비율
    mover_noyield_prob: float = 0.0
    # 1: 에피소드마다 양보 비율 U(0, 1) 을 뽑아 mover 가 그 확률로 기다림 (mover_noyield_prob 대체)
    mover_yield_mix: int = 1
    # ---- 벽 스침: 0 이면 끔. 접촉력 임계값이 못 잡는 mm 단위 스침을 막는다 ----
    # 우리 로봇 몸체 (base_link 기준)
    footprint_x: tuple[float, float] = (-0.0745, 0.0387)
    footprint_y: tuple[float, float] = (-0.047, 0.047)
    graze_m: float = 0.005  # 여유가 이 이하면 충돌로 에피소드 종료
    near_wall_m: float = 0.05  # 벽/선반/박스 여유가 이 미만이면 벌점
    w_near_wall: float = -0.1  # 여유 0 에서 스텝당 벌점, near_wall_m 에서 0 까지 선형
    # 시작 노드가 벽/선반 면에서 corner_start_m 이내인 경로로 시작하는 에피소드 비율
    # 로봇은 경로 반대쪽(± corner_yaw_jitter rad)을 봄
    corner_start_prob: float = 0.25
    corner_start_m: float = 0.17
    corner_yaw_jitter: float = 0.6
    # ---- RL+shim 인계 시작 (RL 플러그인 트랙): 이 비율의 에피소드, 0 이면 뽑지 않음 ----
    # handover_nodes 에서 나가는 경로 위에 서서(v 0) w0 = +-handover_w (경로 쪽 부호)로 돌며 시작
    # 경로 대비 방향 오차: handover_normal_deg +- jitter (정상 인계) 또는 handover_cut_deg 균등
    # (shim 회전 중 끊긴 인계)
    handover_start_prob: float = 0.0
    handover_nodes: str = "Q2,M_E,E-3"
    handover_normal_deg: float = 21.0
    handover_normal_jitter_deg: float = 5.0
    handover_normal_share: float = 0.5  # 나머지는 끊긴 인계
    handover_cut_deg: tuple[float, float] = (40.0, 160.0)
    handover_w: tuple[float, float] = (0.5, 1.0)  # rad/s, 시작의 절반씩
    # ---- 스크립트 플릿 로봇 (4c, robot_spec): HALL 내비 그래프 경로 ----
    robot_max: int = 0  # 에피소드당 로봇 수, 0..robot_max 균등 (0: 4b 아레나)
    # 속도 배율은 1.0 이하 (Pode 최고속도 초과 불가)
    robot_speed_scale: tuple[float, float] = (0.67, 1.0)
    robot_pause_s: tuple[float, float] = (0.0, 2.0)  # 경로 목표마다 정지
    robot_routes: int = 12  # 에피소드당 이어 붙이는 경로 수
    # 다음 스텝에 우리 로봇과 이 거리 이내로 가까워지면 기다림
    robot_yield_m: float = 0.25
    # 이만큼 기다리면 경로를 따라 물러남
    robot_wait_s: float = ts.travel_time(3.0)
    robot_backoff_s: float = ts.travel_time(2.0)  # 물러나는 시간
    robot_retreat: int = 1  # 1: 직전 경로 노드까지 물러남, 0: robot_backoff_s 동안
    robot_lock_m: float = 0.0  # > 0: 우리 로봇이 다음 링크에 이만큼 가까우면 노드에서 대기
    # fleet_manager 식 노드 예약 (robot_spec.robot_step): 1 = 켬
    # 우리 로봇은 경로의 robot_reserve_m 이내 노드를 robot_release_m 지날 때까지 잡음
    robot_reserve: int = 1
    robot_reserve_m: float = 0.12
    robot_release_m: float = 0.2
    # 우선권: 에피소드마다 이 확률로 로봇이 우리보다 우선 (우리를 안 기다림)
    robot_priority_prob: float = 0.0
    # 1: 평가용, 로봇 0 이 우리 목표 노드에서 시작 노드로 정면 주행
    robot_headon: int = 0
    intent_ablate: int = 0  # 1: intent 항목을 0 으로 (로봇이 없는 듯 보임), 평가 전용


ARENA = ArenaSpec()


def make_arena_spec(overrides: dict) -> ArenaSpec:
    return dataclasses.replace(ARENA, **overrides)


def split_overrides(items) -> tuple[dict, dict]:
    """--set FIELD=VALUE 문자열을 (TaskSpec 덮어쓰기, ArenaSpec 덮어쓰기) 로 나눈다."""
    arena_fields = {f.name for f in dataclasses.fields(ArenaSpec)}
    task_items = [i for i in items if i.split("=", 1)[0] not in arena_fields]
    arena_items = [i for i in items if i.split("=", 1)[0] in arena_fields]
    return ts.parse_overrides(task_items), ts.parse_overrides(arena_items, ARENA)


def _box_points_clearance(pts: torch.Tensor, half) -> torch.Tensor:
    """box 좌표계 점 pts (..., 2) 에서 박스까지 거리 (안쪽은 0)."""
    dx = (pts[..., 0].abs() - half[0]).clamp(min=0.0)
    dy = (pts[..., 1].abs() - half[1]).clamp(min=0.0)
    return torch.sqrt(dx * dx + dy * dy)


def _corners(xy: torch.Tensor, yaw: torch.Tensor, half) -> torch.Tensor:
    """박스의 네 모서리 (..., 4, 2)."""
    sx = torch.tensor([1.0, 1.0, -1.0, -1.0], device=xy.device) * half[0]
    sy = torch.tensor([1.0, -1.0, -1.0, 1.0], device=xy.device) * half[1]
    c, s = torch.cos(yaw)[..., None], torch.sin(yaw)[..., None]
    return torch.stack((xy[..., 0:1] + c * sx - s * sy, xy[..., 1:2] + s * sx + c * sy), -1)


def _to_frame(p: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """점 p (N, M, 2) 를 xy, yaw 가 정한 좌표계로 옮김."""
    d = p - xy[:, None]
    c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    return torch.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]), -1)


def _boxes_overlap(c1, yaw1, h1, c2, yaw2, h2) -> torch.Tensor:
    """두 회전 박스가 겹치는지 분리축 검사."""
    def axes(yaw):
        c, s = torch.cos(yaw), torch.sin(yaw)
        return torch.stack((c, s), -1), torch.stack((-s, c), -1)

    a1, a2 = axes(yaw1)
    b1, b2 = axes(yaw2)
    d = c2 - c1
    sep = torch.zeros(d.shape[:-1], dtype=torch.bool, device=d.device)
    def reach(u, v, h, ax):  # ax 방향 박스 반폭
        return h[..., 0] * (u * ax).sum(-1).abs() + h[..., 1] * (v * ax).sum(-1).abs()

    for ax in (a1, a2, b1, b2):
        span = reach(a1, a2, h1, ax) + reach(b1, b2, h2, ax)
        sep = sep | ((d * ax).sum(-1).abs() > span)
    return ~sep


def graze_flags(clearance: torch.Tensor, graze_m: float) -> torch.Tensor:
    """(n,) bool: 여유가 graze_m 이하 (graze_m <= 0 이면 없음)."""
    if graze_m <= 0:
        return torch.zeros_like(clearance, dtype=torch.bool)
    return clearance <= graze_m


def footprint_box(xy, yaw, fx, fy):
    """몸체 박스의 중심 (N, 2) 과 반폭: base_link 위치 xy, yaw, 몸체 범위 fx, fy."""
    cx, cy = 0.5 * (fx[0] + fx[1]), 0.5 * (fy[0] + fy[1])
    c, s = torch.cos(yaw), torch.sin(yaw)
    centre = torch.stack((xy[:, 0] + c * cx - s * cy, xy[:, 1] + s * cx + c * cy), -1)
    return centre, (0.5 * (fx[1] - fx[0]), 0.5 * (fy[1] - fy[0]))


def footprint_clearance(xy, yaw, half, rects) -> torch.Tensor:
    """우리 발자국에서 가장 가까운 축정렬 rect (N, B, 4) 까지 거리 (N,), 겹치면 0."""
    fc = _corners(xy, yaw, half)  # (N, 4, 2)
    r = rects[:, None]  # (N, 1, B, 4)
    dx = torch.clamp(torch.maximum(r[..., 0] - fc[..., 0:1], fc[..., 0:1] - r[..., 2]), min=0.0)
    dy = torch.clamp(torch.maximum(r[..., 1] - fc[..., 1:2], fc[..., 1:2] - r[..., 3]), min=0.0)
    d1 = torch.sqrt(dx * dx + dy * dy).flatten(1).amin(-1)
    rc = torch.stack(
        (rects[..., [0, 2, 2, 0]], rects[..., [1, 1, 3, 3]]), -1
    ).flatten(1, 2)  # (N, 4B, 2)
    d2 = _box_points_clearance(_to_frame(rc, xy, yaw), half).amin(-1)
    # 모서리가 서로 안에 없는 겹침 (발자국을 가로지르는 막대)
    hf = torch.tensor(half, device=xy.device)
    hr = 0.5 * (rects[..., 2:] - rects[..., :2])
    over = _boxes_overlap(
        xy[:, None], yaw[:, None], hf, 0.5 * (rects[..., :2] + rects[..., 2:]),
        torch.zeros_like(rects[..., 0]), hr,
    ).any(-1)
    return torch.where(over, 0.0, torch.minimum(d1, d2))


def box_clearance(xy, yaw, half, pos, yaw2, half2) -> torch.Tensor:
    """우리 발자국에서 박스 (pos, yaw2, half2) 까지 거리 (N,)."""
    d1 = _box_points_clearance(_to_frame(_corners(xy, yaw, half), pos, yaw2), half2).amin(-1)
    d2 = _box_points_clearance(_to_frame(_corners(pos, yaw2, half2), xy, yaw), half).amin(-1)
    hf, h2 = torch.tensor(half, device=xy.device), torch.tensor(half2, device=xy.device)
    over = _boxes_overlap(xy, yaw, hf, pos, yaw2, h2)
    return torch.where(over, 0.0, torch.minimum(d1, d2))


def cylinder_clearance(xy, yaw, half, pos, radius: float) -> torch.Tensor:
    """우리 발자국에서 원기둥까지 거리 (N,), 겹치면 0."""
    d = _box_points_clearance(_to_frame(pos[:, None], xy, yaw), half)[:, 0]
    return (d - radius).clamp(min=0.0)


def near_wall_penalty(clearance: torch.Tensor, near_m: float, weight: float) -> torch.Tensor:
    """스텝당 보상: 여유 0 에서 weight, near_m 에서 0 까지 선형."""
    if near_m <= 0:
        return torch.zeros_like(clearance)
    return weight * (1.0 - clearance / near_m).clamp(min=0.0)


def near_wall_starts(start_xy: torch.Tensor, rects: torch.Tensor, max_m: float) -> torch.Tensor:
    """시작점 (R, 2) 이 rect 면에서 max_m 이내인 경로 (R,)."""
    x, y = start_xy[:, 0:1], start_xy[:, 1:2]
    dx = torch.clamp(torch.maximum(rects[:, 0] - x, x - rects[:, 2]), min=0.0)
    dy = torch.clamp(torch.maximum(rects[:, 1] - y, y - rects[:, 3]), min=0.0)
    return torch.sqrt(dx * dx + dy * dy).amin(-1) < max_m


def path_heading(path: torch.Tensor, ahead_m: float = 0.1) -> torch.Tensor:
    """경로 (N, L, 2) 시작의 방향 (N,): ahead_m 이상 떨어진 첫 점 (없으면 끝점) 쪽."""
    d = path - path[:, :1]
    far = d.norm(dim=-1) >= ahead_m
    k = torch.where(far.any(-1), far.float().argmax(-1), torch.full_like(far[:, 0], -1).long())
    v = d[torch.arange(len(path), device=path.device), k]
    return torch.atan2(v[:, 1], v[:, 0])


def away_heading(path: torch.Tensor, jitter: torch.Tensor, ahead_m: float = 0.1) -> torch.Tensor:
    """경로 시작 방향(path_heading)의 반대 yaw (N,): pi + jitter 만큼 돌림."""
    yaw = path_heading(path, ahead_m) + math.pi + jitter
    return torch.atan2(torch.sin(yaw), torch.cos(yaw))


def handover_start(path_dir: torch.Tensor, u: torch.Tensor, spec: ArenaSpec = ARENA):
    """인계 시작의 yaw (N,) 와 초기 w (N,). u (N, 4) 는 [0, 1) 균등 값: 종류, 오차 크기, 오차 방향,
    |w|.
    w 는 로봇을 경로 쪽으로 돌림."""
    normal = u[:, 0] < spec.handover_normal_share
    j = math.radians(spec.handover_normal_jitter_deg)
    err_n = math.radians(spec.handover_normal_deg) + (2 * u[:, 1] - 1) * j
    lo, hi = (math.radians(d) for d in spec.handover_cut_deg)
    err_c = lo + u[:, 1] * (hi - lo)
    side = torch.where(u[:, 2] < 0.5, 1.0, -1.0)
    err = side * torch.where(normal, err_n, err_c)
    yaw = path_dir + err
    w_mag = torch.where(u[:, 3] < 0.5, spec.handover_w[0], spec.handover_w[1])
    return torch.atan2(torch.sin(yaw), torch.cos(yaw)), -side * w_mag


def load_site(spec: ArenaSpec = ARENA):
    return load_fleet_map(spec.layout)


def arena_boxes(spec: ArenaSpec = ARENA, site=None) -> list[Box]:
    """맵 좌표의 벽(bounds)과 실측 선반으로 박스 목록을 만든다."""
    site = load_site(spec) if site is None else site
    walls = [dataclasses.replace(b, height_m=spec.wall_height_m) for b in gz_arena.walls(site)]
    # real_site: Gazebo 전용 sim_shelves 를 실측 선반으로 바꿈
    shelves = [
        dataclasses.replace(b, height_m=spec.shelf_height_m)
        for b in gz_arena.shelves(gz_arena.real_site(site))
    ]
    return walls + shelves


def boxes_tensor(boxes: list[Box], device="cpu") -> torch.Tensor:
    """(B, 4) x0, y0, x1, y1 텐서."""
    return torch.tensor([(b.x0, b.y0, b.x1, b.y1) for b in boxes], device=device)


def ray_angles(n_rays: int = ts.SPEC.lidar_rays, device="cpu") -> torch.Tensor:
    """로봇 베이스 프레임에서 i 번째 레이 각도 (-180 + i * 360 / n) deg."""
    return torch.deg2rad(-180.0 + torch.arange(n_rays, device=device) * (360.0 / n_rays))


def raycast_rects(
    origin: torch.Tensor,
    yaw: torch.Tensor,
    rects: torch.Tensor,
    min_range: float = ts.SPEC.lidar_min_range,
    max_range: float = ts.SPEC.lidar_obs_max,
    n_rays: int = ts.SPEC.lidar_rays,
) -> torch.Tensor:
    """높이가 라이다 위인 축정렬 rect 에 대한 2D 라이다 [m], (N, n_rays).

    레이는 min_range 에서 시작하고 거리는 중심 기준이다. rect 안에서 시작하면 먼 면, 미적중은 inf."""
    ang = yaw[:, None] + ray_angles(n_rays, origin.device)[None, :]  # (N, R)
    dx, dy = torch.cos(ang), torch.sin(ang)
    sx = origin[:, 0:1] + min_range * dx  # ray start (N, R)
    sy = origin[:, 1:2] + min_range * dy
    big = torch.tensor(1e9, device=origin.device)

    def slab(s, d, lo, hi):  # 한 축의 진입/이탈 파라미터
        d = d[..., None]
        s = s[..., None]
        inv = torch.where(d.abs() > 1e-12, 1.0 / d, big)
        t1, t2 = (lo - s) * inv, (hi - s) * inv
        inside = (s >= lo) & (s <= hi)
        parallel = d.abs() <= 1e-12
        tmin = torch.where(parallel, torch.where(inside, -big, big), torch.minimum(t1, t2))
        tmax = torch.where(parallel, torch.where(inside, big, -big), torch.maximum(t1, t2))
        return tmin, tmax

    rr = rects if rects.dim() == 2 else rects[:, None]  # (B, 4) 또는 (N, 1, B, 4)
    x0, y0, x1, y1 = (rr[..., i] for i in range(4))
    txn, txx = slab(sx, dx, x0, x1)
    tyn, tyx = slab(sy, dy, y0, y1)
    tmin = torch.maximum(txn, tyn)
    tmax = torch.minimum(txx, tyx)
    hit = tmax >= torch.clamp(tmin, min=0.0)
    t = torch.where(tmin >= 0.0, tmin, tmax)  # 안에서 시작: 먼 면
    t = torch.where(hit, t, torch.full_like(t, math.inf)).amin(dim=-1)
    r = t + min_range
    return torch.where(r <= max_range, r, torch.full_like(r, math.inf))

