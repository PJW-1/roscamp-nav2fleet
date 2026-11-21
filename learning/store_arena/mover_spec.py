"""Moving obstacles (a person and a cart) without a simulator: leg sampling and motion, batched
torch on the env device.

Shapes and nominal speeds are the Gazebo benchmark's (pd-mppi tools/nav_bench/scenarios.yaml):
person = cylinder r 0.04 m, h 0.25 m, 0.10 m/s; cart = box 0.14 x 0.09 x 0.20 m, 0.06 m/s; both go
back and forth along one straight leg. Here each episode draws, per mover, whether it is there, a
leg in free HALL space (often across the global path), a speed around the nominal one, a pause at
each end and a starting point on the leg.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class MoverShape:
    name: str
    kind: str  # "cylinder" (size = radius, height) or "box" (size = length, width, height)
    size: tuple[float, ...]
    speed: float  # nominal m/s

    @property
    def radius(self) -> float:
        """Clearance radius: cylinder radius, or the box's half-diagonal."""
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
    """p (N, M, 2) points, rects (N, B, 4) -> (N, M) distance to the nearest rect (0 inside)."""
    x, y = p[..., 0:1], p[..., 1:2]  # (N, M, 1)
    r = rects[:, None]  # (N, 1, B, 4)
    dx = torch.clamp(torch.maximum(r[..., 0] - x, x - r[..., 2]), min=0.0)
    dy = torch.clamp(torch.maximum(r[..., 1] - y, y - r[..., 3]), min=0.0)
    return torch.sqrt(dx * dx + dy * dy).amin(-1)


def point_segment_distance(p: torch.Tensor, a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """p (N, 2), a, b (N, C, 2) -> (N, C) distance from p to each segment a-b."""
    ab = b - a
    t = ((p[:, None] - a) * ab).sum(-1) / (ab * ab).sum(-1).clamp(min=1e-12)
    q = a + t.clamp(0.0, 1.0)[..., None] * ab
    return (p[:, None] - q).norm(dim=-1)


# Legs are checked at points at most 2 * LEG_CHECK_MARGIN_M apart, so the true clearance of the
# whole leg is at most LEG_CHECK_MARGIN_M below the checked one (a corner halfway between points).
LEG_CHECK_MARGIN_M = 0.01


def leg_clearance(a: torch.Tensor, b: torch.Tensor, rects: torch.Tensor) -> torch.Tensor:
    """a, b (N, C, 2) legs, rects (N, B, 4) -> (N, C) smallest distance from points along each
    leg (spaced <= 2 * LEG_CHECK_MARGIN_M) to the rects."""
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
    candidates: int = 128,
) -> dict[str, torch.Tensor]:
    """One leg per env for one mover. rects (N, B, 4) static boxes plus this env's obstacle boxes,
    path (N, L, 2) / npts (N,) the env's global path, start / goal (N, 2).

    A candidate leg is centred on a random path point (probability mover_on_path_prob) or anywhere
    in the bounds, has a random direction and a length in mover_leg_m. It is valid when every
    point of it keeps the mover (clearance radius + LEG_CHECK_MARGIN_M at the checked points, so
    at least the radius everywhere) off walls, shelves and boxes, stays inside the bounds and the
    whole leg stays mover_node_clear_m from the start and the goal. The first valid candidate is
    used; an env with none, or not drawn (mover_prob), has the mover inactive (parked)."""
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
    ends = torch.stack((a, b), -2)  # a straight leg is inside the bounds when both ends are
    inside = (ends[..., 0] > x0 + r) & (ends[..., 0] < x1 - r)
    inside &= (ends[..., 1] > y0 + r) & (ends[..., 1] < y1 - r)
    ok = (leg_clearance(a, b, rects) > r + LEG_CHECK_MARGIN_M) & inside.all(-1)
    ok &= point_segment_distance(start, a, b) > spec.mover_node_clear_m
    ok &= point_segment_distance(goal, a, b) > spec.mover_node_clear_m
    first = ok.float().argmax(-1)  # 0 when none is valid (then inactive)
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
        "phase": rand(n),  # where on its cycle the mover starts, as a fraction
        "active": active,
    }


def mover_pose(legs: dict[str, torch.Tensor], t: torch.Tensor):
    """Position (N, 2), yaw (N,) and velocity (N, 2) at time t (N,) [s] since the episode start.
    Cycle: pause at a, go to b, pause at b, go back. A mover with speed 0 stays at a. The yaw is
    the leg direction (a box keeps it both ways)."""
    a, b, v, pause = legs["a"], legs["b"], legs["speed"], legs["pause"]
    seg = b - a
    length = seg.norm(dim=-1).clamp(min=1e-9)
    u = seg / length[:, None]
    yaw = torch.atan2(u[:, 1], u[:, 0])
    moving = v > 0
    t_leg = torch.where(moving, length / v.clamp(min=1e-9), torch.ones_like(v))
    cycle = 2 * (t_leg + pause)
    tau = torch.remainder(t + legs["phase"] * cycle, cycle)
    out1 = torch.minimum((tau - pause).clamp(min=0.0), t_leg)  # 0..t_leg on the way out
    back = (tau - (2 * pause + t_leg)).clamp(min=0.0)  # 0..t_leg on the way back
    going_back = tau >= t_leg + pause
    s = torch.where(going_back, t_leg - torch.minimum(back, t_leg), out1) / t_leg  # 0..1 a->b
    s = torch.where(moving, s, torch.zeros_like(s))
    pos = a + s[:, None] * seg
    in_out = (tau > pause) & (tau < pause + t_leg)
    in_back = tau > 2 * pause + t_leg
    sign = torch.where(in_out, 1.0, torch.where(in_back, -1.0, 0.0)) * moving.float()
    vel = (sign * v)[:, None] * u
    return pos, yaw, vel
