"""Store arena definition without a simulator: settings, boxes from the fleet map, a 2D ray cast.

The arena is built from the fleet map yaml at run time (``pc/fleet_core/fleet_core/data``), so a
re-measured map changes the arena without code changes. Walls are the ``bounds`` ring and shelves
are the measured ``shelves`` list; ``sim_shelves`` (a Gazebo-only correction) is never read here.
Map coordinates are env-local coordinates: map (0, 0) sits at the env origin.

Everything here is plain Python / PyTorch, so tests and a future 2D simulator can use it.
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

Box = gz_arena.Box  # name, x0, y0, x1, y1, kind ("wall" or "shelf"), height_m


@dataclass(frozen=True)
class ArenaSpec:
    """The one place for store-arena settings. Every field can be changed with --set."""

    layout: str = "v7"  # map name (fleet_core/data/fleet_map_<name>.yaml) or a yaml path
    wall_height_m: float = gz_arena.WALL_HEIGHT_M  # 0.30, the Gazebo world value
    shelf_height_m: float = 0.30  # above the whole robot; the map's 0.15 is a lidar-only minimum
    # Arena cells are this far apart. Rays and contacts stay inside their own env (Newton world),
    # checked by arena_check.py --mode crossenv, so this only has to keep the arenas from touching.
    env_spacing: float = 3.0
    # Fixed start for phase A checks (start sampling from HALL nodes is the next phase).
    start_x: float = 0.30
    start_y: float = 0.55


ARENA = ArenaSpec()


def make_arena_spec(overrides: dict) -> ArenaSpec:
    return dataclasses.replace(ARENA, **overrides)


def split_overrides(items) -> tuple[dict, dict]:
    """--set FIELD=VALUE strings -> (TaskSpec overrides, ArenaSpec overrides), routed by field
    name. Values are parsed with task_spec.parse_overrides against the matching dataclass."""
    arena_fields = {f.name for f in dataclasses.fields(ArenaSpec)}
    task_items = [i for i in items if i.split("=", 1)[0] not in arena_fields]
    arena_items = [i for i in items if i.split("=", 1)[0] in arena_fields]
    return ts.parse_overrides(task_items), ts.parse_overrides(arena_items, ARENA)


def load_site(spec: ArenaSpec = ARENA):
    return load_fleet_map(spec.layout)


def arena_boxes(spec: ArenaSpec = ARENA, site=None) -> list[Box]:
    """Walls around ``bounds`` and the measured ``shelves`` of the fleet map, in map coordinates."""
    site = load_site(spec) if site is None else site
    walls = [dataclasses.replace(b, height_m=spec.wall_height_m) for b in gz_arena.walls(site)]
    # real_site swaps the Gazebo-only sim_shelves for the measured shelves
    shelves = [
        dataclasses.replace(b, height_m=spec.shelf_height_m)
        for b in gz_arena.shelves(gz_arena.real_site(site))
    ]
    return walls + shelves


def boxes_tensor(boxes: list[Box], device="cpu") -> torch.Tensor:
    """(B, 4) tensor of x0, y0, x1, y1."""
    return torch.tensor([(b.x0, b.y0, b.x1, b.y1) for b in boxes], device=device)


def ray_angles(n_rays: int = ts.SPEC.lidar_rays, device="cpu") -> torch.Tensor:
    """Ray i at (-180 + i * 360 / n) deg in the robot base frame, as in goal_reach_env."""
    return torch.deg2rad(-180.0 + torch.arange(n_rays, device=device) * (360.0 / n_rays))


def raycast_rects(
    origin: torch.Tensor,
    yaw: torch.Tensor,
    rects: torch.Tensor,
    min_range: float = ts.SPEC.lidar_min_range,
    max_range: float = ts.SPEC.lidar_obs_max,
    n_rays: int = ts.SPEC.lidar_rays,
) -> torch.Tensor:
    """2D lidar against axis-aligned rectangles taller than the lidar, the way the sim lidar does
    it: each ray starts min_range out and the range is measured from the centre. A ray that starts
    inside a rectangle returns its far side. No hit within max_range -> inf.

    origin (N, 2) lidar centre, yaw (N,) robot yaw, rects (B, 4) -> (N, n_rays) ranges [m]."""
    ang = yaw[:, None] + ray_angles(n_rays, origin.device)[None, :]  # (N, R)
    dx, dy = torch.cos(ang), torch.sin(ang)
    sx = origin[:, 0:1] + min_range * dx  # ray start (N, R)
    sy = origin[:, 1:2] + min_range * dy
    big = torch.tensor(1e9, device=origin.device)

    def slab(s, d, lo, hi):  # (N, R, B) entry and exit parameters along one axis
        d = d[..., None]
        s = s[..., None]
        inv = torch.where(d.abs() > 1e-12, 1.0 / d, big)
        t1, t2 = (lo - s) * inv, (hi - s) * inv
        inside = (s >= lo) & (s <= hi)
        parallel = d.abs() <= 1e-12
        tmin = torch.where(parallel, torch.where(inside, -big, big), torch.minimum(t1, t2))
        tmax = torch.where(parallel, torch.where(inside, big, -big), torch.maximum(t1, t2))
        return tmin, tmax

    x0, y0, x1, y1 = (rects[:, i] for i in range(4))
    txn, txx = slab(sx, dx, x0, x1)
    tyn, tyx = slab(sy, dy, y0, y1)
    tmin = torch.maximum(txn, tyn)
    tmax = torch.minimum(txx, tyx)
    hit = tmax >= torch.clamp(tmin, min=0.0)
    t = torch.where(tmin >= 0.0, tmin, tmax)  # start inside -> far side
    t = torch.where(hit, t, torch.full_like(t, math.inf)).amin(dim=-1)
    r = t + min_range
    return torch.where(r <= max_range, r, torch.full_like(r, math.inf))

