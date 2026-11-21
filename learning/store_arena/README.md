# Store arena (Isaac Lab, Newton)

The goal-reach env (`../goal_reach/`) inside the real store arena: walls around the fleet map's
`bounds` and the measured `shelves` of `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml`, built
from the yaml every time the env is constructed (never `sim_shelves`, the Gazebo-only correction).
Each episode drives a route between two HALL nodes along a Nav2-like global path, with up to
three static boxes on the way and a moving person and cart (stage 4b). Results:
`results/2026-11-21/store_arena/`. Design choices and what was learned:
`docs/worklog_store_arena_2026-11-21.md` (phase A: arena and checks),
`docs/worklog_store_arena_b_2026-11-21.md` (phase B: routes, obstacles, PPO),
`docs/worklog_store_arena_4b_2026-11-21.md` (stage 4b: movers).

| file | what it does |
|---|---|
| `arena_spec.py` | no simulator: `ArenaSpec` (the one place for arena, route and obstacle settings, all `--set`-able), boxes from the yaml through the fleet_core loader and `sim/fleet_gz_sim` `walls()` / `real_site()`, a 2D lidar ray cast of those boxes |
| `route_spec.py` | no simulator: HALL start/goal nodes, the Nav2-like global costmap and Dijkstra path, obstacle layouts, all routes precomputed as padded arrays, `follow()` (progress, remaining path length, lookahead point) |
| `mover_spec.py` | no simulator: the person and cart (Gazebo benchmark sizes and speeds), leg sampling, back-and-forth motion, waiting for the robot and turning back |
| `store_arena_env.py` | `StoreArenaEnv`: `GoalReachEnv` plus one static collider per yaml box, a route per episode, kinematic obstacle boxes moved at reset, kinematic movers moved every control step, the lookahead point as the observed goal |
| `arena_check.py` | Isaac checks without a policy: `scene` (build time, GPU memory, lidar vs yaml and boxes, `--render` top-down image), `contact` (drive into walls and shelves), `crossenv` (overlapping arenas and a second robot), `reset` (first observation after a reset), `obstacle` (boxes stay put, driving into one is a collision), `mover` (movers where they should be, lidar sees both, driving into one is a collision) |
| `train.py`, `run_train.sh` | PPO (goal_reach settings) in chunks under the shared GPU lock, each chunk resuming the last (`INIT=<model.pt>` starts the first from a checkpoint) |
| `play.py` | deterministic evaluation, outcomes split by boxes on the route and by the closest approach to a mover, trajectories to `rollouts.npz` |
| `plot_arena.py` | `scene`, `contact`, `routes`, `curves`, `traj` plots and an `anim` top-down GIF (no Isaac) |
| `normalizer_std.py` | per-dimension mean and std of a checkpoint's actor observation normalizer (no Isaac) |
| `test_arena_spec.py`, `test_route_spec.py`, `test_mover_spec.py` | tests without Isaac |

## Commands (Git Bash)

```bash
python -m pytest learning -q -p no:cacheprovider       # system python (torch cpu, pytest)
python learning/store_arena/plot_arena.py routes routes.png

# training: chunks of N iterations; takes /c/Users/pjw/ros2camp/.heavy_run.lock per chunk
bash learning/store_arena/run_train.sh /d/isaac/runs/store_arena/ppo_g 1 500 4096
python learning/store_arena/plot_arena.py anim out.gif <eval dir>/rollouts.npz 8 12  # env 8, 12 s

cd /d/isaac/IsaacLab
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
S=/c/Users/pjw/ros2camp/pd-arena/learning/store_arena   # this checkout
.venv/Scripts/python.exe $S/arena_check.py --mode scene --num_envs 64 --out /d/isaac/runs/store_arena/b_scene_64 --viz none
.venv/Scripts/python.exe $S/arena_check.py --mode obstacle --num_envs 256 --set mover_prob=0 --out /d/isaac/runs/store_arena/c_obstacle_256 --viz none
.venv/Scripts/python.exe $S/arena_check.py --mode mover --num_envs 256 --out /d/isaac/runs/store_arena/c_mover_256 --viz none
.venv/Scripts/python.exe $S/arena_check.py --mode contact --num_envs 8 --set obstacle_max=0 --out /d/isaac/runs/store_arena/contact_8 --viz none
.venv/Scripts/python.exe $S/play.py --checkpoint /d/isaac/runs/store_arena/ppo_g/c1/model_499.pt --num_envs 1024 --out /d/isaac/runs/store_arena/c_eval_499 --viz none
```

Take the lock (`mkdir /c/Users/pjw/ros2camp/.heavy_run.lock`, owner file inside) around any
direct Isaac command; the Gazebo track uses the same lock. `contact` and `crossenv` place robots
by hand, so run them with `--set obstacle_max=0`; `scene`, `contact`, `crossenv` and `reset`
park the movers unless `--set mover_prob=...` is given. `--set FIELD=VALUE` takes both `TaskSpec`
fields (`task_spec.OVERRIDABLE`) and `ArenaSpec` fields (e.g. `--set layout=<re-measured yaml>`,
`--set lookahead_m=0.3`). Use the venv python directly, never `uv run` / `uv sync`.
