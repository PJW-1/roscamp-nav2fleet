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
    # longest route ~3 m at 0.25 m/s is 12 s, plus turns and detours: 30 s at 0.25 m/s
    episode_s: float = ts.travel_time(30.0)
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
    mover_node_clear_m: float = 0.25  # the whole leg keeps this far from the start and goal
    # Mover surface to the global path at both leg ends: the planner's inscribed 0.07 m. With
    # ends on the path (half of the legs) a mover patrolled the route: 21 % of episodes timed out.
    mover_end_path_clear_m: float = 0.07
    # A mover waits while its next step would bring it closer than this to the robot centre
    # (surface to centre; the robot's footprint reaches 0.06-0.082 m): any contact is then the
    # robot's doing. Movers that walked into a waiting robot made half of all episodes collide.
    mover_yield_m: float = 0.15
    # ... and turns back after waiting this long (as a person would), so a robot and a mover
    # waiting for each other do not stand until the episode times out.
    mover_wait_s: float = ts.travel_time(2.0)  # 2 s at 0.25 m/s
    # share of episodes whose movers never wait (yield 0, as the Gazebo benchmark's movers)
    mover_noyield_prob: float = 0.0
    # 1 (B48, user decision, default since 2026-11-22): each episode draws a yield fraction
    # U(0, 1) and each mover then waits (mover_yield_m) with that probability, else never waits.
    # Replaces mover_noyield_prob; 0 for the 4b/4c arena (all movers wait)
    mover_yield_mix: int = 1
    # --- wall grazes (user decision 2026-11-22, defaults since then; 0 turns each off). The
    # contact force threshold misses 0-3 mm grazes (Gazebo Q2 -> E-3: 0.000-0.003 m)
    # our robot's body in base_link (rearmost collider; RL track measurement 2026-11-22)
    footprint_x: tuple[float, float] = (-0.0745, 0.0387)
    footprint_y: tuple[float, float] = (-0.047, 0.047)
    graze_m: float = 0.005  # footprint clearance at or below this ends the episode as a collision
    near_wall_m: float = 0.05  # walls, shelves, boxes: penalty while the clearance is below this
    w_near_wall: float = -0.1  # per step at clearance 0, linear to 0 at near_wall_m
    # share of episodes that start on a route whose start node is within corner_start_m of a wall
    # or shelf (centre to face), facing away from the path (+- corner_yaw_jitter rad)
    # (0.17: E-1 0.12, E-3 0.131, Q1 0.131, Q2 0.15, M_E 0.168 m from a face: the starts of the
    # Gazebo and sweep grazes; E-2 0.184, O_OUT 0.18, O_IN 0.202 stay out). The jitter covers
    # goals 145-215 deg behind (the sweep grazes: goal 145-175 deg behind)
    corner_start_prob: float = 0.25
    corner_start_m: float = 0.17
    corner_yaw_jitter: float = 0.6
    # share of episodes that start as an RL+shim handover (RL plugin track, 2026-11-22): on a
    # route from one of handover_nodes, standing (v 0) but turning at w0 = +-handover_w (one of
    # the two, sign towards the path), heading error to the path either about handover_normal_deg
    # (+- jitter; a normal handover) or uniform in handover_cut_deg (a handover cut short while
    # the shim still spins). Drawn before the corner starts; 0: no draw (earlier arenas)
    handover_start_prob: float = 0.0
    handover_nodes: str = "Q2,M_E,E-3"
    handover_normal_deg: float = 21.0
    handover_normal_jitter_deg: float = 5.0
    handover_normal_share: float = 0.5  # the rest are cut short
    handover_cut_deg: tuple[float, float] = (40.0, 160.0)
    handover_w: tuple[float, float] = (0.5, 1.0)  # rad/s, half the starts each
    # --- scripted fleet robots (stage 4c, robot_spec): routes over the HALL nav graph
    robot_max: int = 0  # robots per episode, uniform 0..robot_max (0: the 4b arena; 4c: 2)
    # at most 1.0 (B60): no stand-in Pode beats the one top speed (4c used 0.67-1.33)
    robot_speed_scale: tuple[float, float] = (0.67, 1.0)  # times robot_spec.ROBOT.speed
    robot_pause_s: tuple[float, float] = (0.0, 2.0)  # at each route goal
    # 12 routes + retreat: user decision 2026-11-22 (4c, ppo_h and the 4c worklog used 4 and 0:
    # reproduce them with --set robot_routes=4 --set robot_retreat=0)
    robot_routes: int = 12  # routes chained per episode (then it stays at the last goal)
    # waits while its next step would bring its centre within this of our robot's centre and
    # closer (Gazebo nav_bench robot: yield_m 0.25, whatever its direction)
    robot_yield_m: float = 0.25
    # ... and after waiting this long backs off along its route (3 s at 0.25 m/s)
    robot_wait_s: float = ts.travel_time(3.0)
    robot_backoff_s: float = ts.travel_time(2.0)  # for this long, then drives on (2 s)
    robot_retreat: int = 1  # 1: backs off to the last route node behind it; 0: for robot_backoff_s
    robot_lock_m: float = 0.0  # > 0: waits at a node while our robot is this near the next link
    # node reservation as fleet_manager's (robot_spec.robot_step blocked): 1 = on. Our robot holds
    # every node within robot_reserve_m of its route until it is robot_release_m past it; robots
    # hold the node they are at and the end of the link they drive; robots never start on ours.
    # Default 1 since 2026-11-22 (B52: ppo_h timeouts 8.5 -> 3.7 %); 0 for the arena before
    robot_reserve: int = 1
    robot_reserve_m: float = 0.12
    robot_release_m: float = 0.2
    # right of way: each robot has priority over ours with this probability per episode; it then
    # never waits for our robot (TaskSpec intent_priority shows the flag)
    robot_priority_prob: float = 0.0
    # 1: evaluation set where robot 0 always drives head-on, from our goal node to our start node
    # by the graph (routes whose start and goal are graph nodes), then on at random. It starts on
    # our goal node; with robot_reserve it then waits there until our nodes ahead are free
    robot_headon: int = 0
    intent_ablate: int = 0  # 1: the intent entries are zeros (robots look absent): evaluation only


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


def _box_points_clearance(pts: torch.Tensor, half) -> torch.Tensor:
    """pts (..., 2) in a box frame -> (...) distance to the box [-hx, hx] x [-hy, hy] (0 inside)."""
    dx = (pts[..., 0].abs() - half[0]).clamp(min=0.0)
    dy = (pts[..., 1].abs() - half[1]).clamp(min=0.0)
    return torch.sqrt(dx * dx + dy * dy)


def _corners(xy: torch.Tensor, yaw: torch.Tensor, half) -> torch.Tensor:
    """(..., 4, 2) corners of boxes at xy (..., 2), yaw (...), half sizes (hx, hy)."""
    sx = torch.tensor([1.0, 1.0, -1.0, -1.0], device=xy.device) * half[0]
    sy = torch.tensor([1.0, -1.0, -1.0, 1.0], device=xy.device) * half[1]
    c, s = torch.cos(yaw)[..., None], torch.sin(yaw)[..., None]
    return torch.stack((xy[..., 0:1] + c * sx - s * sy, xy[..., 1:2] + s * sx + c * sy), -1)


def _to_frame(p: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """Points p (N, M, 2) into the frames at xy (N, 2), yaw (N,)."""
    d = p - xy[:, None]
    c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
    return torch.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]), -1)


def _boxes_overlap(c1, yaw1, h1, c2, yaw2, h2) -> torch.Tensor:
    """Separating axis test of boxes (centres (..., 2), yaws (...), half sizes (..., 2))."""
    def axes(yaw):
        c, s = torch.cos(yaw), torch.sin(yaw)
        return torch.stack((c, s), -1), torch.stack((-s, c), -1)

    a1, a2 = axes(yaw1)
    b1, b2 = axes(yaw2)
    d = c2 - c1
    sep = torch.zeros(d.shape[:-1], dtype=torch.bool, device=d.device)
    def reach(u, v, h, ax):  # half extent of a box with axes u, v along ax
        return h[..., 0] * (u * ax).sum(-1).abs() + h[..., 1] * (v * ax).sum(-1).abs()

    for ax in (a1, a2, b1, b2):
        span = reach(a1, a2, h1, ax) + reach(b1, b2, h2, ax)
        sep = sep | ((d * ax).sum(-1).abs() > span)
    return ~sep


def graze_flags(clearance: torch.Tensor, graze_m: float) -> torch.Tensor:
    """(n,) bool: clearance at or below graze_m (none when graze_m <= 0)."""
    if graze_m <= 0:
        return torch.zeros_like(clearance, dtype=torch.bool)
    return clearance <= graze_m


def footprint_box(xy, yaw, fx, fy):
    """Centre (N, 2) and half sizes of our body box: base_link at xy (N, 2), yaw (N,), body
    x range fx and y range fy in base_link."""
    cx, cy = 0.5 * (fx[0] + fx[1]), 0.5 * (fy[0] + fy[1])
    c, s = torch.cos(yaw), torch.sin(yaw)
    centre = torch.stack((xy[:, 0] + c * cx - s * cy, xy[:, 1] + s * cx + c * cy), -1)
    return centre, (0.5 * (fx[1] - fx[0]), 0.5 * (fy[1] - fy[0]))


def footprint_clearance(xy, yaw, half, rects) -> torch.Tensor:
    """(N,) distance from our footprint (centre xy (N, 2), yaw (N,), half sizes) to the nearest
    axis-aligned rect (N, B, 4) x0 y0 x1 y1; 0 when a corner of either is inside the other. The
    nearest points of two disjoint convex polygons include a vertex of one of them."""
    fc = _corners(xy, yaw, half)  # (N, 4, 2)
    r = rects[:, None]  # (N, 1, B, 4)
    dx = torch.clamp(torch.maximum(r[..., 0] - fc[..., 0:1], fc[..., 0:1] - r[..., 2]), min=0.0)
    dy = torch.clamp(torch.maximum(r[..., 1] - fc[..., 1:2], fc[..., 1:2] - r[..., 3]), min=0.0)
    d1 = torch.sqrt(dx * dx + dy * dy).flatten(1).amin(-1)
    rc = torch.stack(
        (rects[..., [0, 2, 2, 0]], rects[..., [1, 1, 3, 3]]), -1
    ).flatten(1, 2)  # (N, 4B, 2)
    d2 = _box_points_clearance(_to_frame(rc, xy, yaw), half).amin(-1)
    # overlaps with no corner inside the other (a bar through the footprint)
    hf = torch.tensor(half, device=xy.device)
    hr = 0.5 * (rects[..., 2:] - rects[..., :2])
    over = _boxes_overlap(
        xy[:, None], yaw[:, None], hf, 0.5 * (rects[..., :2] + rects[..., 2:]),
        torch.zeros_like(rects[..., 0]), hr,
    ).any(-1)
    return torch.where(over, 0.0, torch.minimum(d1, d2))


def box_clearance(xy, yaw, half, pos, yaw2, half2) -> torch.Tensor:
    """(N,) distance from our footprint to a box at pos (N, 2), yaw2 (N,), half sizes half2."""
    d1 = _box_points_clearance(_to_frame(_corners(xy, yaw, half), pos, yaw2), half2).amin(-1)
    d2 = _box_points_clearance(_to_frame(_corners(pos, yaw2, half2), xy, yaw), half).amin(-1)
    hf, h2 = torch.tensor(half, device=xy.device), torch.tensor(half2, device=xy.device)
    over = _boxes_overlap(xy, yaw, hf, pos, yaw2, h2)
    return torch.where(over, 0.0, torch.minimum(d1, d2))


def cylinder_clearance(xy, yaw, half, pos, radius: float) -> torch.Tensor:
    """(N,) distance from our footprint to a cylinder at pos (N, 2) (0 when they overlap)."""
    d = _box_points_clearance(_to_frame(pos[:, None], xy, yaw), half)[:, 0]
    return (d - radius).clamp(min=0.0)


def near_wall_penalty(clearance: torch.Tensor, near_m: float, weight: float) -> torch.Tensor:
    """Per-step reward: weight at clearance 0, linear to 0 at near_m and beyond."""
    if near_m <= 0:
        return torch.zeros_like(clearance)
    return weight * (1.0 - clearance / near_m).clamp(min=0.0)


def near_wall_starts(start_xy: torch.Tensor, rects: torch.Tensor, max_m: float) -> torch.Tensor:
    """(R,) routes whose start (R, 2) is within max_m (centre to face) of a rect (B, 4)."""
    x, y = start_xy[:, 0:1], start_xy[:, 1:2]
    dx = torch.clamp(torch.maximum(rects[:, 0] - x, x - rects[:, 2]), min=0.0)
    dy = torch.clamp(torch.maximum(rects[:, 1] - y, y - rects[:, 3]), min=0.0)
    return torch.sqrt(dx * dx + dy * dy).amin(-1) < max_m


def path_heading(path: torch.Tensor, ahead_m: float = 0.1) -> torch.Tensor:
    """(N,) direction of the path (N, L, 2) at its start: towards its first point at least
    ahead_m away (else its last point)."""
    d = path - path[:, :1]
    far = d.norm(dim=-1) >= ahead_m
    k = torch.where(far.any(-1), far.float().argmax(-1), torch.full_like(far[:, 0], -1).long())
    v = d[torch.arange(len(path), device=path.device), k]
    return torch.atan2(v[:, 1], v[:, 0])


def away_heading(path: torch.Tensor, jitter: torch.Tensor, ahead_m: float = 0.1) -> torch.Tensor:
    """(N,) yaw facing away from the path direction at its start (path_heading), turned by
    pi + jitter (N,)."""
    yaw = path_heading(path, ahead_m) + math.pi + jitter
    return torch.atan2(torch.sin(yaw), torch.cos(yaw))


def handover_start(path_dir: torch.Tensor, u: torch.Tensor, spec: ArenaSpec = ARENA):
    """Yaw (N,) and initial w (N,) of handover starts; u (N, 4) uniform draws in [0, 1): kind
    (normal or cut short), error size, error side, |w|. w turns the robot towards the path."""
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

