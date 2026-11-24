"""시뮬레이터 없이 쓰는 이동 장애물(사람과 수레): 구간 샘플링과 움직임.

모양과 기준 속도는 Gazebo 벤치마크 값이다. 에피소드마다 mover 별로 등장 여부, HALL 빈 공간의 직선 구간,
속도, 양끝 정지, 시작 위치를 뽑는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class MoverShape:
    name: str
    kind: str  # "cylinder" (size = 반지름, 높이) 또는 "box" (size = 길이, 폭, 높이)
    size: tuple[float, ...]
    speed: float  # 기준 m/s

    @property
    def radius(self) -> float:
        """여유 반지름: 원기둥은 반지름, 박스는 반대각 절반."""
        if self.kind == "cylinder":
            return self.size[0]
        return 0.5 * math.hypot(self.size[0], self.size[1])

    @property
    def height(self) -> float:
        return self.size[-1]


MOVERS = (
    MoverShape("person", "cylinder", (0.04, 0.25), 0.10),
    MoverShape("cart", "box", (0.14, 0.09, 0.20), 0.06),
)


def point_rect_distance(p: torch.Tensor, rects: torch.Tensor) -> torch.Tensor:
    """점 p (N, M, 2) 에서 가장 가까운 rect (N, B, 4) 까지 거리 (N, M), 안쪽은 0."""
    x, y = p[..., 0:1], p[..., 1:2]  # (N, M, 1)
    r = rects[:, None]  # (N, 1, B, 4)
    dx = torch.clamp(torch.maximum(r[..., 0] - x, x - r[..., 2]), min=0.0)
    dy = torch.clamp(torch.maximum(r[..., 1] - y, y - r[..., 3]), min=0.0)
    return torch.sqrt(dx * dx + dy * dy).amin(-1)


def point_segment_distance(p: torch.Tensor, a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """점 p (N, 2) 에서 선분 a-b (N, C, 2) 까지 거리 (N, C)."""
    ab = b - a
    t = ((p[:, None] - a) * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-12)
    q = a + t.clamp(0.0, 1.0)[..., None] * ab
    return (p[:, None] - q).norm(dim=-1)


# 구간은 2 * LEG_CHECK_MARGIN_M 이하 간격으로 검사하므로 실제 여유는 검사값보다 최대 LEG_CHECK_MARGIN_M 작음
LEG_CHECK_MARGIN_M = 0.01


def leg_clearance(a: torch.Tensor, b: torch.Tensor, rects: torch.Tensor) -> torch.Tensor:
    """구간 a, b (N, C, 2) 위 점들에서 rect (N, B, 4) 까지 최소 거리 (N, C)."""
    n, c = a.shape[:2]
    m = int(math.ceil(float((b - a).norm(dim=-1).max()) / (2 * LEG_CHECK_MARGIN_M))) + 1
    s = torch.linspace(0.0, 1.0, max(m, 2), device=a.device)
    pts = a[:, :, None] + s[None, None, :, None] * (b - a)[:, :, None]  # (N, C, m, 2)
    return point_rect_distance(pts.reshape(n, -1, 2), rects).reshape(n, c, -1).amin(-1)


def sample_legs(
    rects: torch.Tensor,
    bounds: tuple[float, float, float, float],
    path: torch.Tensor,
    npts: torch.Tensor,
    start: torch.Tensor,
    goal: torch.Tensor,
    shape: MoverShape,
    spec,
    gen: torch.Generator | None = None,
    candidates: int = 256,
) -> dict[str, torch.Tensor]:
    """mover 하나의 env 별 구간 샘플링.

    후보 구간은 경로점 중심(mover_on_path_prob) 또는 임의 위치, 임의 방향, mover_leg_m 길이다.
    벽/선반/박스에서 떨어지고, 범위 안이며, 시작/목표에서 mover_node_clear_m, 양끝이 경로에서
    mover_end_path_clear_m 떨어지면 유효하다. 첫 유효 후보를 쓰고, 없거나 안 뽑히면 비활성."""
    n, dev = start.shape[0], start.device

    def rand(*s):
        return torch.rand(*s, device=dev, generator=gen)

    x0, y0, x1, y1 = bounds
    c = candidates
    on_path = rand(n, c) < spec.mover_on_path_prob
    j = (rand(n, c) * npts[:, None]).long().clamp(max=path.shape[1] - 1)
    centre_path = torch.gather(path, 1, j[..., None].expand(n, c, 2))
    centre_any = torch.stack((x0 + rand(n, c) * (x1 - x0), y0 + rand(n, c) * (y1 - y0)), -1)
    centre = torch.where(on_path[..., None], centre_path, centre_any)
    ang = rand(n, c) * 2 * math.pi
    lo, hi = spec.mover_leg_m
    half = 0.5 * (lo + rand(n, c) * (hi - lo))
    d = torch.stack((torch.cos(ang), torch.sin(ang)), -1)
    a, b = centre - half[..., None] * d, centre + half[..., None] * d
    r = shape.radius
    ends = torch.stack((a, b), -2)  # 직선 구간은 양끝이 범위 안이면 안쪽
    inside = (ends[..., 0] > x0 + r) & (ends[..., 0] < x1 - r)
    inside &= (ends[..., 1] > y0 + r) & (ends[..., 1] < y1 - r)
    ok = (leg_clearance(a, b, rects) > r + LEG_CHECK_MARGIN_M) & inside.all(-1)
    ok &= point_segment_distance(start, a, b) > spec.mover_node_clear_m
    ok &= point_segment_distance(goal, a, b) > spec.mover_node_clear_m
    # 양끝(멈추고 돌아서는 곳)이 경로 옆이어야 유일한 길을 막지 않음
    on = torch.arange(path.shape[1], device=dev)[None, None] < npts[:, None, None]  # (N, 1, L)
    for end in (a, b):
        d = (end[:, :, None] - path[:, None]).norm(dim=-1)  # (N, C, L)
        d = torch.where(on, d, torch.full_like(d, 9.0)).amin(-1)
        ok &= d - r >= spec.mover_end_path_clear_m
    first = ok.float().argmax(-1)  # 유효 없으면 0 (그때는 비활성)
    rows = torch.arange(n, device=dev)
    active = ok.any(-1) & (rand(n) < spec.mover_prob)
    k0, k1 = spec.mover_speed_scale
    speed = shape.speed * (k0 + rand(n) * (k1 - k0))
    p0, p1 = spec.mover_pause_s
    return {
        "a": a[rows, first],
        "b": b[rows, first],
        "speed": speed,
        "pause": p0 + rand(n) * (p1 - p0),
        "phase": rand(n),  # 주기 중 시작 위치 비율
        "active": active,
    }


def mover_pose(legs: dict[str, torch.Tensor], t: torch.Tensor):
    """시작 후 시간 t (N,) [s] 의 위치 (N, 2), yaw (N,), 속도 (N, 2).
    주기: a 에서 정지, b 로 이동, b 에서 정지, 복귀. 속도 0 이면 a 에 머문다."""
    a, b, v, pause = legs["a"], legs["b"], legs["speed"], legs["pause"]
    seg = b - a
    length = seg.norm(dim=-1).clamp(min=1e-9)
    u = seg / length[:, None]
    yaw = torch.atan2(u[:, 1], u[:, 0])
    moving = v > 0
    t_leg = torch.where(moving, length / v.clamp(min=1e-9), torch.ones_like(v))
    cycle = 2 * (t_leg + pause)
    tau = torch.remainder(t + legs["phase"] * cycle, cycle)
    out1 = torch.minimum((tau - pause).clamp(min=0.0), t_leg)  # 가는 길 0..t_leg
    back = (tau - (2 * pause + t_leg)).clamp(min=0.0)  # 오는 길 0..t_leg
    going_back = tau >= t_leg + pause
    s = torch.where(going_back, t_leg - torch.minimum(back, t_leg), out1) / t_leg  # 0..1 a->b
    s = torch.where(moving, s, torch.zeros_like(s))
    pos = a + s[:, None] * seg
    in_out = (tau > pause) & (tau < pause + t_leg)
    in_back = tau > 2 * pause + t_leg
    sign = torch.where(in_out, 1.0, torch.where(in_back, -1.0, 0.0)) * moving.float()
    vel = (sign * v)[:, None] * u
    return pos, yaw, vel


def surface_gap(shape: MoverShape, xy: torch.Tensor, pos: torch.Tensor, yaw: torch.Tensor):
    """점 xy (n, 2) 에서 mover 표면까지 거리 (n,), 사람 안이면 음수."""
    d = xy - pos
    if shape.kind == "cylinder":
        return d.norm(dim=-1) - shape.size[0]
    c, s_ = torch.cos(yaw), torch.sin(yaw)  # yaw 만큼 돌린 박스까지 거리
    lx, ly = c * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c * d[:, 1]
    ex = (lx.abs() - 0.5 * shape.size[0]).clamp(min=0.0)
    ey = (ly.abs() - 0.5 * shape.size[1]).clamp(min=0.0)
    return torch.sqrt(ex * ex + ey * ey)


def turn_back(legs, t: torch.Tensor) -> torch.Tensor:
    """같은 자리에서 반대로 가는 시계 시간 (정지 중이면 그대로): 주기를 중앙에서 접음."""
    v, pause = legs["speed"], legs["pause"]
    length = (legs["b"] - legs["a"]).norm(dim=-1).clamp(min=1e-9)  # mover_pose 와 같음
    t_leg = torch.where(v > 0, length / v.clamp(min=1e-9), torch.ones_like(v))
    cycle = 2 * (t_leg + pause)
    tau = torch.remainder(t + legs["phase"] * cycle, cycle)
    return t + torch.remainder(3 * pause + 2 * t_leg - 2 * tau, cycle)


def mixed_yield(n: int, movers: int, yield_m: float, gen=None, device="cpu") -> torch.Tensor:
    """(movers, n) mover 별 양보 거리: env 마다 U(0, 1) 비율을 뽑아 그 확률로 yield_m, 아니면 0."""
    frac = torch.rand(n, generator=gen, device=device)
    waits = torch.rand(movers, n, generator=gen, device=device) < frac
    return torch.where(waits, yield_m, 0.0)


def yield_step(legs, t, dt: float, shape: MoverShape, robot_xy, yield_m: float, wait, wait_s):
    """mover 시계 t 와 대기 시간 wait [s] 의 제어 한 스텝.
    로봇 중심에 yield_m 이내로 다가가면 기다리고, 로봇이 떠나면 가며, wait_s 기다리면 돌아선다. 반환 (t, wait, turned)."""
    pos0, yaw0, _ = mover_pose(legs, t)
    pos1, yaw1, _ = mover_pose(legs, t + dt)
    g0 = surface_gap(shape, robot_xy, pos0, yaw0)
    g1 = surface_gap(shape, robot_xy, pos1, yaw1)
    held = (g1 < yield_m) & (g1 < g0) & (torch.as_tensor(yield_m) > 0)  # 0: 안 기다림
    wait = torch.where(held, wait + dt, torch.zeros_like(wait))
    turned = held & (wait >= wait_s)
    t = torch.where(turned, turn_back(legs, t), torch.where(held, t, t + dt))
    return t, torch.where(turned, torch.zeros_like(wait), wait), turned
