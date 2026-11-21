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
    # Robot position used by arena_check.py when it places robots by hand (phase A checks).
    start_x: float = 0.30
    start_y: float = 0.55
    episode_s: float = 30.0  # longest route ~3 m at 0.25 m/s is 12 s, plus turns and detours
    # --- start and goal (route_spec.hall_nodes): nodes in zone HALL except these kinds
    excluded_node_kinds: str = "parking"  # P-1..3 sit between the charging dividers
    min_start_goal_m: float = 0.40  # straight-line start-goal distance
    # --- global path (route_spec, like Nav2 NavFn on the robot's global costmap)
    grid_m: float = 0.025  # global costmap resolution
    inscribed_m: float = 0.07  # footprint half-side 0.06 + footprint_padding 0.01
    inflation_m: float = 0.15  # inflation_radius
    cost_scaling: float = 15.0  # cost_scaling_factor
    lookahead_m: float = 0.40  # observed goal: this far along the path beyond the robot
    follow_window: int = 20  # path points the progress index may jump per step (~0.5 m)
    # --- static obstacles (stage 4a): square boxes, one size per slot
    obstacle_sizes: tuple[float, float, float] = (0.10, 0.15, 0.20)  # side [m]
    obstacle_height_m: float = 0.30
    obstacle_max: int = 3  # boxes per episode, uniform 0..obstacle_max
    # layouts per start-goal pair (layout 0 has no boxes). 96 since 4b: the 4a policy trained on
    # 32 dropped from 91.4 to 87.0 % on unseen 3-box layouts; 4a's own runs used 32
    obstacle_layouts: int = 96
    obstacle_on_path_prob: float = 0.7  # a box is put on the obstacle-free path, else anywhere
    obstacle_lateral_m: float = 0.08  # sideways shift of a box put on the path
    obstacle_node_clear_m: float = 0.25  # box edge to the start and goal nodes
    route_seed: int = 0  # layouts are drawn once at env build with this seed
    # --- moving obstacles (stage 4b, mover_spec.MOVERS: person, cart), drawn at every reset
    mover_prob: float = 0.75  # each mover is in an episode with this probability
    mover_on_path_prob: float = 0.6  # its leg is centred on the global path, else anywhere
    mover_leg_m: tuple[float, float] = (0.3, 0.8)  # leg length range
    mover_speed_scale: tuple[float, float] = (0.5, 1.5)  # times the nominal speed
    mover_pause_s: tuple[float, float] = (0.0, 1.0)  # stop at each end of the leg
    mover_node_clear_m: float = 0.25
    # Mover surface to the global path at both leg ends: the planner's inscribed 0.07 m. With
    # ends on the path (half of the legs) a mover patrolled the route: 21 % of episodes timed out.
    mover_end_path_clear_m: float = 0.07
    # A mover waits while its next step would bring it closer than this to the robot centre
    # (surface to centre; the robot's footprint reaches 0.06-0.082 m): any contact is then the
    # robot's doing. Movers that walked into a waiting robot made half of all episodes collide.
    mover_yield_m: float = 0.15
    # ... and turns back after waiting this long (as a person would), so a robot and a mover
    # waiting for each other do not stand until the episode times out.
    mover_wait_s: float = 2.0  # the whole leg keeps this far from the start and goal


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

    origin (N, 2) lidar centre, yaw (N,) robot yaw, rects (B, 4) or per env (N, B, 4)
    -> (N, n_rays) ranges [m]."""
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

    rr = rects if rects.dim() == 2 else rects[:, None]  # (B, 4) or (N, 1, B, 4)
    x0, y0, x1, y1 = (rr[..., i] for i in range(4))
    txn, txx = slab(sx, dx, x0, x1)
    tyn, tyx = slab(sy, dy, y0, y1)
    tmin = torch.maximum(txn, tyn)
    tmax = torch.minimum(txx, tyx)
    hit = tmax >= torch.clamp(tmin, min=0.0)
    t = torch.where(tmin >= 0.0, tmin, tmax)  # start inside -> far side
    t = torch.where(hit, t, torch.full_like(t, math.inf)).amin(dim=-1)
    r = t + min_range
    return torch.where(r <= max_range, r, torch.full_like(r, math.inf))

