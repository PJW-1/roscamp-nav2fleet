"""Isaac Lab (Newton) store arena: the goal-reach env plus walls and shelves from the fleet map.

The boxes come from ``arena_spec.arena_boxes`` when the env is constructed, so a re-measured
``fleet_map_v7.yaml`` changes the arena without code changes. Each box is a static collider under
the env namespace, so it is cloned into every env (Newton world) like the robot. Map coordinates
are env-local: map (0, 0) is the env origin.

Each episode drives one route (route_spec.build_routes): a start and a goal HALL node, a layout of
static obstacle boxes and the global path planned around them. The observed goal is the point
``lookahead_m`` ahead on that path; success is reaching the final goal (TaskSpec.success_radius);
the progress reward follows the remaining path length. Obstacle boxes are kinematic bodies moved
at reset; unused ones wait behind the south wall. Movers (mover_spec: a person and a cart) are
kinematic bodies moved every control step along a leg drawn at reset; they are not in the global
path, as a person who just appeared is not in the deployed Nav2 plan.
"""

from __future__ import annotations

import os
import sys

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.utils import configclass
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))

import arena_spec as arena  # noqa: E402
import mover_spec as ms  # noqa: E402
import numpy as np  # noqa: E402
import robot_spec as rbs  # noqa: E402
import route_spec as rs  # noqa: E402
import task_spec as ts  # noqa: E402
import torch  # noqa: E402
from goal_reach_env import GoalReachEnv, GoalReachEnvCfg  # noqa: E402

COLORS = {
    "wall": (0.55, 0.55, 0.6),
    "shelf": (0.75, 0.55, 0.3),
    "obstacle": (0.2, 0.45, 0.8),
    "mover": (0.85, 0.2, 0.25),
    "robot": (0.2, 0.7, 0.35),
}
# what a collision is put down to (the nearest of them at the collision step)
HIT_KINDS = ("wall_shelf", "box", "mover", "robot")
LIFT_M = 0.002  # obstacle boxes float this far above the floor (no floor contacts)


def box_asset_cfg(box: arena.Box) -> AssetBaseCfg:
    """A static box collider (no rigid body) at map coordinates inside each env."""
    sx, sy = box.x1 - box.x0, box.y1 - box.y0
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/arena_" + box.name,
        spawn=sim_utils.CuboidCfg(
            size=(sx, sy, box.height_m),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS[box.kind]),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(box.x0 + 0.5 * sx, box.y0 + 0.5 * sy, 0.5 * box.height_m)
        ),
    )


def add_arena(scene_cfg, boxes: list[arena.Box]) -> None:
    """Add one scene entry per box (the scene builds every non-field attribute of its cfg)."""
    for box in boxes:
        setattr(scene_cfg, "arena_" + box.name, box_asset_cfg(box))


def park_xy(k: int) -> tuple[float, float]:
    """Where unused obstacle slot k waits: behind the south wall, out of lidar sight."""
    return (0.3 + 0.5 * k, -0.4)


def obstacle_cfg(k: int, side: float, height: float) -> RigidObjectCfg:
    """Obstacle slot k: a kinematic, gravity-free square box moved per env at reset."""
    x, y = park_xy(k)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/obstacle_{k}",
        spawn=sim_utils.CuboidCfg(
            size=(side, side, height),
            rigid_props=[
                sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                PhysxRigidBodyCfg(disable_gravity=True),
            ],
            mass_props=sim_utils.MassCfg(mass=10.0),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS["obstacle"]),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, 0.5 * height + LIFT_M)),
    )


def mover_park_xy(i: int) -> tuple[float, float]:
    """Where an inactive mover waits: behind the south wall, east of the obstacle slots."""
    return (1.6 + 0.3 * i, -0.4)


def mover_cfg(i: int, shape: ms.MoverShape) -> RigidObjectCfg:
    """Mover i: a kinematic, gravity-free cylinder or box moved every control step."""
    common = dict(
        rigid_props=[
            sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
            PhysxRigidBodyCfg(disable_gravity=True),
        ],
        mass_props=sim_utils.MassCfg(mass=10.0),
        collision_props=sim_utils.UsdPhysicsCollisionCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS["mover"]),
    )
    if shape.kind == "cylinder":
        spawn = sim_utils.CylinderCfg(radius=shape.size[0], height=shape.height, **common)
    else:
        spawn = sim_utils.CuboidCfg(size=shape.size, **common)
    x, y = mover_park_xy(i)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/mover_{shape.name}",
        spawn=spawn,
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, 0.5 * shape.height + LIFT_M)),
    )


def robot_park_xy(k: int) -> tuple[float, float]:
    """Where an inactive scripted robot waits: behind the south wall, below the other slots."""
    return (0.3 + 0.5 * k, -0.9)


def robot_cfg(k: int) -> RigidObjectCfg:
    """Scripted fleet robot k: a kinematic, gravity-free Pode-sized box moved every step."""
    sx, sy, sz = rbs.ROBOT.size
    x, y = robot_park_xy(k)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/fleet_robot_{k}",
        spawn=sim_utils.CuboidCfg(
            size=(sx, sy, sz),
            rigid_props=[
                sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                PhysxRigidBodyCfg(disable_gravity=True),
            ],
            mass_props=sim_utils.MassCfg(mass=10.0),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS["robot"]),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, 0.5 * sz + LIFT_M)),
    )


@configclass
class StoreArenaEnvCfg(GoalReachEnvCfg):
    # ArenaSpec fields to override (arena_spec.split_overrides); read when the env is built
    arena_overrides: dict = {}


class StoreArenaEnv(GoalReachEnv):
    cfg: StoreArenaEnvCfg

    def __init__(self, cfg: StoreArenaEnvCfg, render_mode: str | None = None, **kwargs):
        a = self.arena = arena.make_arena_spec(cfg.arena_overrides)
        self.boxes = arena.arena_boxes(a)
        add_arena(cfg.scene, self.boxes)
        static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in self.boxes])
        site = arena.load_site(a)
        self.routes = r = rs.build_routes(a, site, static)
        self.routes_bounds = site.bounds()
        for k, side in enumerate(r.obstacle_sizes):
            setattr(cfg.scene, f"obstacle_{k}", obstacle_cfg(k, side, a.obstacle_height_m))
        for i, shape in enumerate(ms.MOVERS):
            setattr(cfg.scene, f"mover_{shape.name}", mover_cfg(i, shape))
        for k in range(int(a.robot_max)):
            setattr(cfg.scene, f"fleet_robot_{k}", robot_cfg(k))
        self.graph = rbs.node_graph(site)
        cfg.scene.env_spacing = a.env_spacing
        cfg.episode_length_s = a.episode_s
        self._routes_ready = False
        super().__init__(cfg, render_mode, **kwargs)
        print(f"[ARENA] {a}")
        for b in self.boxes:
            print(
                f"[ARENA] {b.name}: x {b.x0:.3f}..{b.x1:.3f} y {b.y0:.3f}..{b.y1:.3f} "
                f"h {b.height_m:.2f}"
            )
        print(
            f"[ARENA] routes {r.count} = {len(r.pairs)} pairs x {r.layouts} layouts, nodes "
            f"{r.node_names}, path length {r.total.min():.2f}..{r.total.max():.2f} m"
        )
        dev, n = self.device, self.num_envs
        f32 = dict(device=dev, dtype=torch.float32)
        ends = np.array([r.start_goal(i) for i in range(r.count)])  # (R, 2, 2)
        self.r_path = torch.as_tensor(r.path, **f32)
        self.r_cum = torch.as_tensor(r.cum, **f32)
        self.r_npts = torch.as_tensor(r.npts, device=dev)
        self.r_start = torch.as_tensor(ends[:, 0], **f32)
        self.r_goal = torch.as_tensor(ends[:, 1], **f32)
        park = torch.tensor([park_xy(k) for k in range(len(r.obstacle_sizes))], **f32)
        box_xy = torch.as_tensor(r.obstacle_xy, **f32)
        self.r_obstacle = torch.where(box_xy.isnan(), park.expand_as(box_xy), box_xy)
        self.obstacles = [self.scene[f"obstacle_{k}"] for k in range(len(r.obstacle_sizes))]
        self.route = torch.zeros(n, dtype=torch.long, device=dev)
        self.prog = torch.zeros(n, dtype=torch.long, device=dev)
        self.la_dist = torch.zeros(n, device=dev)
        self.la_ang = torch.zeros(n, device=dev)
        self.d_goal = torch.zeros(n, device=dev)
        # movers: one leg per env and mover (mover_spec.sample_legs), time since reset, and the
        # smallest gap between the robot centre and a mover surface in the episode (inf: none)
        self.static_rects = torch.as_tensor(static, **f32)
        self.mover_objs = [self.scene[f"mover_{m.name}"] for m in ms.MOVERS]
        self.mover_legs = [
            {
                "a": torch.zeros(n, 2, **f32),
                "b": torch.zeros(n, 2, **f32),
                "speed": torch.zeros(n, **f32),
                "pause": torch.zeros(n, **f32),
                "phase": torch.zeros(n, **f32),
                "active": torch.zeros(n, dtype=torch.bool, device=dev),
            }
            for _ in ms.MOVERS
        ]
        self.mover_park = torch.tensor([mover_park_xy(i) for i in range(len(ms.MOVERS))], **f32)
        self.mover_t = torch.zeros(len(ms.MOVERS), n, **f32)  # each mover's own clock
        self.mover_held = torch.zeros(len(ms.MOVERS), n, dtype=torch.bool, device=dev)  # waiting
        self.mover_wait = torch.zeros(len(ms.MOVERS), n, **f32)  # how long it has waited [s]
        # yield distance per mover and env (mover_yield_mix, mover_noyield_prob)
        self.mover_yield = torch.full((len(ms.MOVERS), n), float(a.mover_yield_m), **f32)
        # wall grazes and corner starts (arena_spec graze_m, near_wall_m, corner_start_prob)
        self.wall_clear = torch.full((n,), float("inf"), **f32)
        self.graze_only = torch.zeros(n, dtype=torch.bool, device=dev)
        self.r_corner = torch.nonzero(
            arena.near_wall_starts(self.r_start, self.static_rects, a.corner_start_m)
        )[:, 0]
        self.corner_start = torch.zeros(n, dtype=torch.bool, device=dev)
        self.last_ep_corner = torch.zeros(n, dtype=torch.bool, device=dev)
        self._new_corner = torch.zeros(n, dtype=torch.bool, device=dev)
        self.mover_gap = torch.full((n,), float("inf"), **f32)
        self.last_ep_mover_gap = torch.full((n,), float("inf"), **f32)
        # scripted fleet robots
        g = self.graph
        self.g_t = rbs.graph_tensors(g, dev)
        # our route's nodes for the reservation rule: arc length along each route where it passes
        # within robot_reserve_m of a graph node (inf: it does not)
        d = np.linalg.norm(r.path[:, :, None] - g.xy[None, None], axis=-1)  # (R, L, K)
        d[np.arange(r.path.shape[1])[None, :] >= r.npts[:, None]] = np.inf
        near = d.min(1) < a.robot_reserve_m
        node_arc = np.take_along_axis(r.cum, d.argmin(1), 1)  # (R, K)
        self.r_node_arc = torch.as_tensor(np.where(near, node_arc, np.inf), **f32)
        gi = {name: i for i, name in enumerate(g.names)}
        # graph index of each route's start and goal node (-1: not a graph node, e.g. Q1)
        self.r_sg_node = torch.tensor(
            [[gi.get(r.node_names[s], -1), gi.get(r.node_names[e], -1)] for s, e in r.pairs],
            device=dev,
        ).repeat_interleave(r.layouts, 0)  # (R, 2)
        nk = int(a.robot_max)
        self.robot_objs = [self.scene[f"fleet_robot_{k}"] for k in range(nk)]
        self.robot_park = torch.tensor([robot_park_xy(k) for k in range(nk)], **f32)
        lay = rbs.chain_route(
            self.g_t,
            torch.zeros(n, int(a.robot_routes) + 1, dtype=torch.long, device=dev),
            torch.ones(n, device=dev),
            torch.zeros(n, int(a.robot_routes), device=dev),
        )
        self.robot_routes = [{k_: v.clone() for k_, v in lay.items()} for _ in range(nk)]
        self.robot_active = torch.zeros(nk, n, dtype=torch.bool, device=dev)
        self.robot_t = torch.zeros(nk, n, **f32)
        self.robot_wait = torch.zeros(nk, n, **f32)
        self.robot_back = torch.zeros(nk, n, **f32)
        self.robot_priority = torch.zeros(nk, n, dtype=torch.bool, device=dev)
        self.robot_gap_min = torch.full((n,), float("inf"), **f32)
        self.last_ep_robot_gap = torch.full((n,), float("inf"), **f32)
        self.hit_kind = torch.zeros(n, dtype=torch.long, device=dev)
        self.last_ep_hit_kind = torch.zeros(n, dtype=torch.long, device=dev)
        self._routes_ready = True

    def sample_start(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        """Start node of a random route and a random yaw; both are kept for _reset_idx."""
        yaw = (2 * torch.rand(n, device=self.device) - 1) * torch.pi
        if not self._routes_ready:  # a reset inside super().__init__: fixed start, no route
            xy = torch.tensor((self.arena.start_x, self.arena.start_y), device=self.device)
            return xy.expand(n, 2).clone(), yaw
        self._new_route = torch.randint(0, self.routes.count, (n,), device=self.device)
        p = self.arena.corner_start_prob
        self._new_corner = torch.zeros(n, dtype=torch.bool, device=self.device)
        if p > 0 and len(self.r_corner):  # no draw otherwise: earlier arenas stay the same
            self._new_corner = torch.rand(n, device=self.device) < p
            pick = self.r_corner[torch.randint(0, len(self.r_corner), (n,), device=self.device)]
            self._new_route = torch.where(self._new_corner, pick, self._new_route)
            jit = (2 * torch.rand(n, device=self.device) - 1) * self.arena.corner_yaw_jitter
            away = arena.away_heading(self.r_path[self._new_route], jit)
            yaw = torch.where(self._new_corner, away, yaw)
        self._new_yaw = yaw
        return self.r_start[self._new_route], yaw

    def _follow(self, ids, xy, yaw) -> None:
        """Path progress, remaining length (d_now), lookahead goal and goal distance for ids."""
        r = self.route[ids]
        self.prog[ids], self.d_now[ids], la = rs.follow(
            self.r_path[r],
            self.r_cum[r],
            self.r_npts[r],
            self.prog[ids],
            xy,
            self.arena.lookahead_m,
            int(self.arena.follow_window),
        )
        self.la_dist[ids], self.la_ang[ids] = ts.goal_in_robot_frame(xy, yaw, la)
        self.d_goal[ids] = (self.goal_xy[ids] - xy).norm(dim=-1)

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if self._routes_ready:
            self.last_ep_mover_gap[env_ids] = self.mover_gap[env_ids]
            self.last_ep_robot_gap[env_ids] = self.robot_gap_min[env_ids]
            self.last_ep_hit_kind[env_ids] = self.hit_kind[env_ids]
            self.last_ep_corner[env_ids] = self.corner_start[env_ids]
        super()._reset_idx(env_ids)
        if not self._routes_ready:
            return
        self.corner_start[env_ids] = self._new_corner
        r = self._new_route
        self.route[env_ids] = r
        self.goal_xy[env_ids] = self.r_goal[r]
        self.prog[env_ids] = 0
        origins = self.scene.env_origins[env_ids]
        zero_vel = torch.zeros(len(env_ids), 6, device=self.device)
        for k, obj in enumerate(self.obstacles):
            pose = obj.data.default_root_pose.torch[env_ids].clone()
            pose[:, :2] = origins[:, :2] + self.r_obstacle[r, k]
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
            obj.write_root_velocity_to_sim_index(root_velocity=zero_vel, env_ids=env_ids)
        self._follow(env_ids, self.start_xy[env_ids], self._new_yaw)
        self.d_prev[env_ids] = self.d_now[env_ids]
        self._sample_movers(env_ids, r)
        self.mover_t[:, env_ids] = 0.0
        self.mover_held[:, env_ids] = False
        self.mover_wait[:, env_ids] = 0.0
        a = self.arena
        if a.mover_yield_mix:  # B48
            n_ = len(env_ids)
            self.mover_yield[:, env_ids] = ms.mixed_yield(
                n_, len(ms.MOVERS), a.mover_yield_m, device=self.device
            )
        elif a.mover_noyield_prob > 0:  # no draw otherwise: 4b and 4c runs stay the same
            noyield = torch.rand(len(env_ids), device=self.device) < a.mover_noyield_prob
            self.mover_yield[:, env_ids] = torch.where(noyield, 0.0, a.mover_yield_m)
        self._write_movers(env_ids)
        self.mover_gap[env_ids] = self.mover_gaps(self.start_xy[env_ids], env_ids)  # at t = 0
        self._sample_robots(env_ids, r)
        self._write_robots(env_ids)
        self.robot_gap_min[env_ids] = self.robot_gaps(self.start_xy[env_ids], env_ids)

    def _sample_robots(self, env_ids, r) -> None:
        """Route chains for env_ids: 0..robot_max robots, starting and stopping on graph nodes
        other than our start and goal (and other robots' starts)."""
        a, n, dev = self.arena, len(env_ids), self.device
        nk = len(self.robot_objs)
        if nk == 0:  # no draw at all: the 4b arena keeps its random sequence (same seed, same run)
            return
        count = torch.randint(0, nk + 1, (n,), device=dev)
        banned = torch.zeros(n, len(self.graph.names), dtype=torch.bool, device=dev)
        sg = self.r_sg_node[r]
        for c in range(2):
            ok = sg[:, c] >= 0
            banned[ok.nonzero()[:, 0], sg[ok, c]] = True
        lo, hi = a.robot_speed_scale
        p0, p1 = a.robot_pause_s
        legs = int(a.robot_routes)
        ours = self.our_nodes(env_ids) if a.robot_reserve else None
        headon = bool(a.robot_headon) & (sg >= 0).all(-1)
        if a.robot_headon:
            count = torch.where(headon, count.clamp(min=1), count)
        for k in range(nk):
            chain = rbs.sample_chains(self.g_t, n, legs, banned, start_banned=ours)
            if k == 0 and a.robot_headon:  # our goal -> our start, then on (never back to start)
                chain[headon, 0], chain[headon, 1] = sg[headon, 1], sg[headon, 0]
                if legs > 1:
                    again = headon & (chain[:, 2] == sg[:, 0])
                    chain[again, 2] = sg[again, 1]
            speed = rbs.ROBOT.speed * (lo + torch.rand(n, device=dev) * (hi - lo))
            pause = p0 + torch.rand(n, legs, device=dev) * (p1 - p0)
            route = rbs.chain_route(self.g_t, chain, speed, pause)
            for key, v in route.items():
                self.robot_routes[k][key][env_ids] = v
            banned[torch.arange(n, device=dev), chain[:, 0]] = True
            self.robot_active[k, env_ids] = k < count
            self.robot_t[k, env_ids] = 0.0
            self.robot_wait[k, env_ids] = 0.0
            self.robot_back[k, env_ids] = 0.0
            if a.robot_priority_prob > 0:  # no draw otherwise: earlier runs stay the same
                self.robot_priority[k, env_ids] = torch.rand(n, device=dev) < a.robot_priority_prob

    def our_nodes(self, ids=None) -> torch.Tensor:
        """(n, K) graph nodes our robot holds: on its route and not yet robot_release_m behind."""
        ids = slice(None) if ids is None else ids
        r = self.route[ids]
        done_arc = self.r_cum[r].gather(1, self.prog[ids][:, None])[:, 0]  # path behind us
        return rbs.route_nodes_held(self.r_node_arc[r], done_arc, self.arena.robot_release_m)

    def blocked_nodes(self, k: int) -> torch.Tensor:
        """(n, K) nodes robot k may only drive through: ours and the other robots'."""
        held = self.our_nodes()
        n = self.num_envs
        rows = torch.arange(n, device=self.device)
        for j, route in enumerate(self.robot_routes):
            if j == k:
                continue
            act = self.robot_active[j]
            for node in rbs.robot_nodes(route, self.robot_t[j]):
                on = act & (node >= 0)
                held[rows[on], node[on]] = True
        return held

    def robot_state(self, ids=None):
        """Per scripted robot: position (n, 2), yaw (n,), active (n,); inactive ones parked."""
        ids = slice(None) if ids is None else ids
        out = []
        for k, route in enumerate(self.robot_routes):
            sub = {key: v[ids] for key, v in route.items()}
            pos, yaw, _ = rbs.robot_pose(sub, self.robot_t[k, ids])
            act = self.robot_active[k, ids]
            pos = torch.where(act[:, None], pos, self.robot_park[k].expand_as(pos))
            out.append((pos, yaw, act))
        return out

    def _write_robots(self, ids) -> None:
        origins = self.scene.env_origins[ids]
        for obj, (pos, yaw, _) in zip(self.robot_objs, self.robot_state(ids), strict=True):
            pose = obj.data.default_root_pose.torch[ids].clone()
            pose[:, :2] = origins[:, :2] + pos
            z = torch.zeros_like(yaw)
            pose[:, 3:7] = torch.stack((z, z, torch.sin(yaw / 2), torch.cos(yaw / 2)), -1)
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)

    def robot_gaps(self, xy: torch.Tensor, ids=None) -> torch.Tensor:
        """(n,) our robot centre to the nearest active scripted robot's box; inf without one."""
        gap = torch.full((len(xy),), float("inf"), device=self.device)
        for pos, yaw, act in self.robot_state(ids):
            gap = torch.where(act, torch.minimum(gap, rbs.robot_gap(xy, pos, yaw)), gap)
        return gap

    def robot_intent(self, xy, yaw):
        """Fleet intent of the intent_robots nearest active robots, nearest first: route points
        (n, R, P, 2) in our robot frame, a mask (n, R) and the priority flags (n, R)."""
        s = self.spec
        pts, dist, act = [], [], []
        for k, (pos, _, a_) in enumerate(self.robot_state()):
            p = rbs.intent_points(
                self.robot_routes[k], self.robot_t[k], s.intent_spacing_m, s.intent_points
            )
            pts.append(rbs.to_body(p, xy, yaw))
            dist.append(torch.where(a_, (pos - xy).norm(dim=-1), torch.full_like(xy[:, 0], 1e9)))
            act.append(a_)
        n, r = len(xy), s.intent_robots
        out_p = torch.zeros(n, r, s.intent_points, 2, device=self.device)
        out_m = torch.zeros(n, r, dtype=torch.bool, device=self.device)
        out_q = torch.zeros(n, r, dtype=torch.bool, device=self.device)
        if pts and not self.arena.intent_ablate:
            pts, dist, act = torch.stack(pts, 1), torch.stack(dist, 1), torch.stack(act, 1)
            order = dist.argsort(1)[:, :r]
            m = min(r, order.shape[1])
            rows = torch.arange(n, device=self.device)[:, None]
            out_p[:, :m] = pts[rows, order[:, :m]]
            out_m[:, :m] = act[rows, order[:, :m]]
            out_q[:, :m] = self.robot_priority.T[rows, order[:, :m]]
        return out_p, out_m, out_q

    def nearest_hit_kind(self, xy: torch.Tensor) -> torch.Tensor:
        """(n,) index into HIT_KINDS of what is nearest to our robot centre."""
        n = len(xy)
        d_static = ms.point_rect_distance(xy[:, None], self.static_rects.expand(n, -1, -1))[:, 0]
        sizes = torch.tensor(self.routes.obstacle_sizes, device=self.device)
        bxy = self.r_obstacle[self.route]
        half = 0.5 * sizes[None, :, None]
        d_box = ms.point_rect_distance(xy[:, None], torch.cat((bxy - half, bxy + half), -1))[:, 0]
        d = torch.stack((d_static, d_box, self.mover_gaps(xy), self.robot_gaps(xy)), -1)
        return d.argmin(-1)

    def _sample_movers(self, env_ids, r, chunk: int = 256) -> None:
        """New legs for env_ids (routes r), avoiding walls, shelves and the route's boxes."""
        sizes = torch.tensor(self.routes.obstacle_sizes, device=self.device)
        bounds = tuple(float(v) for v in self.routes_bounds)
        for c0 in range(0, len(env_ids), chunk):
            ids, rr = env_ids[c0 : c0 + chunk], r[c0 : c0 + chunk]
            xy = self.r_obstacle[rr]  # (n, slots, 2), parked boxes stay behind the wall
            half = 0.5 * sizes[None, :, None]
            boxes = torch.cat((xy - half, xy + half), -1)
            rects = torch.cat((self.static_rects.expand(len(ids), -1, -1), boxes), 1)
            for shape, legs in zip(ms.MOVERS, self.mover_legs, strict=True):
                new = ms.sample_legs(
                    rects,
                    bounds,
                    self.r_path[rr],
                    self.r_npts[rr],
                    self.r_start[rr],
                    self.r_goal[rr],
                    shape,
                    self.arena,
                )
                for k, v in new.items():
                    legs[k][ids] = v

    def mover_state(self, ids=None):
        """Per mover: position (n, 2), yaw (n,), velocity (n, 2) in the env frame, and active."""
        ids = slice(None) if ids is None else ids
        out = []
        for i, legs in enumerate(self.mover_legs):
            sub = {k: v[ids] for k, v in legs.items()}
            pos, yaw, vel = ms.mover_pose(sub, self.mover_t[i, ids])
            act = sub["active"]
            pos = torch.where(act[:, None], pos, self.mover_park[i].expand_as(pos))
            moving = act & ~self.mover_held[i, ids]
            vel = torch.where(moving[:, None], vel, torch.zeros_like(vel))
            out.append((pos, yaw, vel, act))
        return out

    def _write_movers(self, ids) -> None:
        origins = self.scene.env_origins[ids]
        for obj, (pos, yaw, _, _) in zip(self.mover_objs, self.mover_state(ids), strict=True):
            pose = obj.data.default_root_pose.torch[ids].clone()
            pose[:, :2] = origins[:, :2] + pos
            z = torch.zeros_like(yaw)
            pose[:, 3:7] = torch.stack((z, z, torch.sin(yaw / 2), torch.cos(yaw / 2)), -1)
            # teleported every control step at rest: the lidar sees exactly mover_state()
            v6 = torch.zeros(len(pos), 6, device=self.device)
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
            obj.write_root_velocity_to_sim_index(root_velocity=v6, env_ids=ids)

    def mover_gaps(self, xy: torch.Tensor, ids=None) -> torch.Tensor:
        """(n,) distance from the robot centre xy (n, 2) of envs ids (all if None) to the nearest
        active mover surface; negative if the centre is inside a person; inf without movers."""
        gap = torch.full((len(xy),), float("inf"), device=self.device)
        for shape, (pos, yaw, _, act) in zip(ms.MOVERS, self.mover_state(ids), strict=True):
            g = ms.surface_gap(shape, xy, pos, yaw)
            gap = torch.where(act, torch.minimum(gap, g), gap)
        return gap

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        super()._pre_physics_step(actions)
        if self._routes_ready:
            xy, _ = self.robot_pose()
            for i, (shape, legs) in enumerate(zip(ms.MOVERS, self.mover_legs, strict=True)):
                t, self.mover_wait[i], _ = ms.yield_step(
                    legs,
                    self.mover_t[i],
                    self.step_dt,
                    shape,
                    xy,
                    self.mover_yield[i],
                    self.mover_wait[i],
                    self.arena.mover_wait_s,
                )
                self.mover_held[i] = t == self.mover_t[i]
                self.mover_t[i] = t
            all_ids = torch.arange(self.num_envs, device=self.device)
            self._write_movers(all_ids)
            a = self.arena
            for k, route in enumerate(self.robot_routes):
                # after the robots before it moved: two robots never take one node in one step
                blocked = self.blocked_nodes(k) if a.robot_reserve else None
                yield_m = torch.where(self.robot_priority[k], 0.0, a.robot_yield_m)
                self.robot_t[k], self.robot_wait[k], self.robot_back[k] = rbs.robot_step(
                    route,
                    self.robot_t[k],
                    self.robot_wait[k],
                    self.robot_back[k],
                    self.step_dt,
                    xy,
                    yield_m,
                    a.robot_wait_s,
                    a.robot_backoff_s,
                    retreat=bool(a.robot_retreat),
                    lock_m=a.robot_lock_m,
                    blocked=blocked,
                    node_m=a.robot_reserve_m,
                )
            if self.robot_objs:
                self._write_robots(all_ids)

    def obstacle_xy(self) -> torch.Tensor:
        """(N, slots, 2) obstacle box centres in the env frame (parked ones included)."""
        org = self.scene.env_origins[:, :2]
        return torch.stack([o.data.root_pos_w.torch[:, :2] - org for o in self.obstacles], 1)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy, yaw = self.robot_pose()
        self._follow(torch.arange(self.num_envs, device=self.device), xy, yaw)
        self.mover_gap = torch.minimum(self.mover_gap, self.mover_gaps(xy))
        self.robot_gap_min = torch.minimum(self.robot_gap_min, self.robot_gaps(xy))
        self.hit_kind = self.nearest_hit_kind(xy)
        self.success, self.collided, self.oob = ts.episode_outcome(
            self.d_goal, self.max_contact_force(), xy, self.start_xy, self.spec
        )
        clear = self.footprint_clearance(xy, yaw)
        graze = arena.graze_flags(clear, self.arena.graze_m)
        self.graze_only = graze & ~self.collided  # the contact sensor did not see it
        self.collided = self.collided | graze
        self.success = self.success & ~graze
        self.oob = self.oob & ~graze
        return ts.split_dones(
            self.success | self.collided | self.oob,
            self.episode_length_buf,
            self.max_episode_length,
        )

    def footprint_clearance(self, xy, yaw) -> torch.Tensor:
        """(n,) our footprint to the nearest wall, shelf, box, mover or robot; also keeps the part
        to walls, shelves and boxes in self.wall_clear (near-wall penalty)."""
        n = len(xy)
        xy, h = arena.footprint_box(xy, yaw, self.arena.footprint_x, self.arena.footprint_y)
        sizes = torch.tensor(self.routes.obstacle_sizes, device=self.device)
        bxy = self.r_obstacle[self.route]
        half = 0.5 * sizes[None, :, None]
        boxes = torch.cat((bxy - half, bxy + half), -1)
        rects = torch.cat((self.static_rects.expand(n, -1, -1), boxes), 1)
        self.wall_clear = arena.footprint_clearance(xy, yaw, h, rects)
        clear = self.wall_clear
        inf = torch.full_like(clear, float("inf"))
        for shape, (pos, myaw, _, act) in zip(ms.MOVERS, self.mover_state(), strict=True):
            if shape.kind == "cylinder":
                c = arena.cylinder_clearance(xy, yaw, h, pos, shape.size[0])
            else:
                hm = (0.5 * shape.size[0], 0.5 * shape.size[1])
                c = arena.box_clearance(xy, yaw, h, pos, myaw, hm)
            clear = torch.minimum(clear, torch.where(act, c, inf))
        hr = (0.5 * rbs.ROBOT.size[0], 0.5 * rbs.ROBOT.size[1])
        for pos, ryaw, act in self.robot_state():
            c = arena.box_clearance(xy, yaw, h, pos, ryaw, hr)
            clear = torch.minimum(clear, torch.where(act, c, inf))
        return clear

    def _get_rewards(self) -> torch.Tensor:
        rew = super()._get_rewards()
        if not self._routes_ready:
            return rew
        a = self.arena
        pen = arena.near_wall_penalty(self.wall_clear, a.near_wall_m, a.w_near_wall)
        self.ep_reward_terms.setdefault("near_wall", torch.zeros_like(pen))
        self.ep_reward_terms["near_wall"] += pen
        return rew + pen

    def _get_observations(self) -> dict:
        if not self._routes_ready:
            return super()._get_observations()  # no intent before the routes exist
        intent = mask = prio = None
        if self.spec.intent_robots:
            intent, mask, prio = self.robot_intent(*self.robot_pose())
        obs = ts.build_observation(
            self.lidar_ranges(),
            self.la_dist,
            self.la_ang,
            self.body_velocity(),
            self.spec,
            intent=intent,
            intent_mask=mask,
            intent_priority=prio,
        )
        return {"policy": obs}
