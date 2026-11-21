"""Isaac Lab (Newton) store arena: the goal-reach env plus walls and shelves from the fleet map.

The boxes come from ``arena_spec.arena_boxes`` when the env is constructed, so a re-measured
``fleet_map_v7.yaml`` changes the arena without code changes. Each box is a static collider under
the env namespace, so it is cloned into every env (Newton world) like the robot. Map coordinates
are env-local: map (0, 0) is the env origin.

Phase A only: a fixed start (``ArenaSpec.start_x/y``); start/goal sampling from HALL nodes comes
next.
"""

from __future__ import annotations

import os
import sys

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils import configclass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))

import arena_spec as arena  # noqa: E402
import torch  # noqa: E402
from goal_reach_env import GoalReachEnv, GoalReachEnvCfg  # noqa: E402

COLORS = {"wall": (0.55, 0.55, 0.6), "shelf": (0.75, 0.55, 0.3)}


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


@configclass
class StoreArenaEnvCfg(GoalReachEnvCfg):
    # ArenaSpec fields to override (arena_spec.split_overrides); read when the env is built
    arena_overrides: dict = {}


class StoreArenaEnv(GoalReachEnv):
    cfg: StoreArenaEnvCfg

    def __init__(self, cfg: StoreArenaEnvCfg, render_mode: str | None = None, **kwargs):
        self.arena = arena.make_arena_spec(cfg.arena_overrides)
        self.boxes = arena.arena_boxes(self.arena)
        add_arena(cfg.scene, self.boxes)
        cfg.scene.env_spacing = self.arena.env_spacing
        super().__init__(cfg, render_mode, **kwargs)
        print(f"[ARENA] {self.arena}")
        for b in self.boxes:
            print(
                f"[ARENA] {b.name}: x {b.x0:.3f}..{b.x1:.3f} y {b.y0:.3f}..{b.y1:.3f} "
                f"h {b.height_m:.2f}"
            )

    def sample_start(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        yaw = (2 * torch.rand(n, device=self.device) - 1) * torch.pi
        xy = torch.tensor((self.arena.start_x, self.arena.start_y), device=self.device)
        return xy.expand(n, 2).clone(), yaw
