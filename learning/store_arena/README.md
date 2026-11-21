# Store arena (Isaac Lab, Newton): phase A

The goal-reach env (`../goal_reach/`) inside the real store arena: walls around the fleet map's
`bounds` and the measured `shelves` of `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml`, built
from the yaml every time the env is constructed (never `sim_shelves`, the Gazebo-only correction).
Results: `results/2026-11-21/store_arena/`. Design choices and what was learned:
`docs/worklog_store_arena_2026-11-21.md`.

| file | what it does |
|---|---|
| `arena_spec.py` | no simulator: `ArenaSpec` (the one place for arena settings, all `--set`-able), boxes from the yaml through the fleet_core loader and `sim/fleet_gz_sim` `walls()` / `real_site()`, a 2D lidar ray cast of those boxes |
| `store_arena_env.py` | `StoreArenaEnv`: `GoalReachEnv` plus one static box collider per yaml box in every env; fixed start for phase A |
| `arena_check.py` | Isaac checks without a policy: `scene` (build time, GPU memory, lidar vs yaml, `--render` top-down image), `contact` (drive into walls and shelves), `crossenv` (overlapping arenas and a second robot), `reset` (first observation after a reset) |
| `plot_arena.py` | top-down plot of `scene.json`: yaml boxes, robot, simulated lidar hits over the 2D ray cast (no Isaac) |
| `test_arena_spec.py` | tests without Isaac: boxes equal the yaml `bounds` and `shelves`, a changed yaml changes the arena, `sim_shelves` is ignored, `--set` routing, the ray cast |

## Commands (Git Bash)

```bash
python -m pytest learning -q -p no:cacheprovider       # system python (torch cpu, pytest)

cd /d/isaac/IsaacLab
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
S=<repo>/learning/store_arena/arena_check.py
.venv/Scripts/python.exe $S --mode scene --num_envs 4 --render --out /d/isaac/runs/store_arena/scene_4 --viz none
.venv/Scripts/python.exe $S --mode scene --num_envs 4096 --out /d/isaac/runs/store_arena/scene_4096 --viz none
.venv/Scripts/python.exe $S --mode contact --num_envs 8 --out /d/isaac/runs/store_arena/contact_8 --viz none
.venv/Scripts/python.exe $S --mode crossenv --num_envs 16 --set env_spacing=0.5 --out /d/isaac/runs/store_arena/crossenv_16 --viz none
.venv/Scripts/python.exe $S --mode reset --num_envs 8 --out /d/isaac/runs/store_arena/reset_8 --viz none
python <repo>/learning/store_arena/plot_arena.py /d/isaac/runs/store_arena/scene_4/scene.json scene.png
```

`--set FIELD=VALUE` takes both `TaskSpec` fields (`task_spec.OVERRIDABLE`) and `ArenaSpec` fields
(e.g. `--set layout=<path to a re-measured yaml>`, `--set shelf_height_m=0.15`). Use the venv
python directly, never `uv run` / `uv sync`. Run one Isaac job at a time.
