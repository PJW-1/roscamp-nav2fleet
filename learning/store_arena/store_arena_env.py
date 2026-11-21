"""Isaac Lab (Newton) store arena: the goal-reach env plus walls and shelves from the fleet map.

The boxes come from ``arena_spec.arena_boxes`` when the env is constructed, so a re-measured
``fleet_map_v7.yaml`` changes the arena without code changes. Each box is a static collider under
the env namespace, so it is cloned into every env (Newton world) like the robot. Map coordinates
are env-local: map (0, 0) is the env origin.

Each episode drives one route (route_spec.build_routes): a start and a goal HALL node, a layout of
static obstacle boxes and the global path planned around them. The observed goal is the point
``lookahead_m`` ahead on that path; success is reaching the final goal (TaskSpec.success_radius);
the progress reward follows the remaining path length. Obstacle boxes are kinematic bodies moved
at reset; unused ones wait behind the south wall.
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
import numpy as np  # noqa: E402
import route_spec as rs  # noqa: E402
import task_spec as ts  # noqa: E402
import torch  # noqa: E402
from goal_reach_env import GoalReachEnv, GoalReachEnvCfg  # noqa: E402

COLORS = {"wall": (0.55, 0.55, 0.6), "shelf": (0.75, 0.55, 0.3), "obstacle": (0.2, 0.45, 0.8)}
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
        self.routes = r = rs.build_routes(a, arena.load_site(a), static)
        for k, side in enumerate(r.obstacle_sizes):
            setattr(cfg.scene, f"obstacle_{k}", obstacle_cfg(k, side, a.obstacle_height_m))
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
        self._routes_ready = True

    def sample_start(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        """Start node of a random route and a random yaw; both are kept for _reset_idx."""
        yaw = (2 * torch.rand(n, device=self.device) - 1) * torch.pi
        if not self._routes_ready:  # a reset inside super().__init__: fixed start, no route
            xy = torch.tensor((self.arena.start_x, self.arena.start_y), device=self.device)
            return xy.expand(n, 2).clone(), yaw
        self._new_route = torch.randint(0, self.routes.count, (n,), device=self.device)
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
        super()._reset_idx(env_ids)
        if not self._routes_ready:
            return
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

    def obstacle_xy(self) -> torch.Tensor:
        """(N, slots, 2) obstacle box centres in the env frame (parked ones included)."""
        org = self.scene.env_origins[:, :2]
        return torch.stack([o.data.root_pos_w.torch[:, :2] - org for o in self.obstacles], 1)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy, yaw = self.robot_pose()
        self._follow(torch.arange(self.num_envs, device=self.device), xy, yaw)
        self.success, self.collided, self.oob = ts.episode_outcome(
            self.d_goal, self.max_contact_force(), xy, self.start_xy, self.spec
        )
        return ts.split_dones(
            self.success | self.collided | self.oob,
            self.episode_length_buf,
            self.max_episode_length,
        )

    def _get_observations(self) -> dict:
        if not self._routes_ready:
            return super()._get_observations()
        obs = ts.build_observation(
            self.lidar_ranges(), self.la_dist, self.la_ang, self.body_velocity(), self.spec
        )
        return {"policy": obs}
