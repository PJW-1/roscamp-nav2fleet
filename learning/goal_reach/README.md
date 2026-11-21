# Goal reach on an empty floor (Isaac Lab, Newton, rsl_rl PPO)

First RL task for Pinky / Pode: drive to a random goal 1-2 m away. Plan step 5-4 items 5 and 6.
Results: `results/2026-11-21/goal_reach_rl/`. Design choices and what was learned:
`docs/worklog_goal_reach_rl_2026-11-21.md`.

| file | what it does |
|---|---|
| `task_spec.py` | the task without a simulator: action limits (one `ActionLimits`), action -> (v, w) with rate limit, (v, w) -> wheel speeds, observation, reward, success, collision, out of bounds, goal sampling. Plain PyTorch, so a 2D simulator can reuse it |
| `goal_reach_env.py` | `DirectRLEnv` on Newton: Pinky from `../sim_check/pinky_cfg.py`, 360-ray lidar, contact sensors on the non-wheel bodies; reads sim state and calls `task_spec` |
| `agent_cfg.py` | rsl_rl PPO runner settings |
| `train.py`, `run_train.sh` | training (writes `metrics.csv` with wall time per iteration, checkpoints, tensorboard, `run.json`); the shell wrapper also logs `nvidia-smi` |
| `play.py` | evaluates a checkpoint (deterministic policy), saves trajectories, exports ONNX, `--video` renders a top-down camera with the Newton Warp renderer |
| `lidar_close_check.py` | what the lidar returns when a box is closer than the 0.05 m minimum range |
| `plot_results.py` | success-rate curve, trajectory plot, top-down animation from saved rollouts (no Isaac needed) |

## Commands (Git Bash, repo root)

```bash
bash learning/goal_reach/run_train.sh /d/isaac/runs/goal_reach/base 4096 300        # about 4 min
bash learning/goal_reach/run_train.sh /d/isaac/runs/goal_reach/try 4096 150 --set w_time=-0.2

cd /d/isaac/IsaacLab
export PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1
.venv/Scripts/python.exe <repo>/learning/goal_reach/play.py --checkpoint /d/isaac/runs/goal_reach/base/model_299.pt \
  --out /d/isaac/runs/goal_reach/base/play --viz none
.venv/Scripts/python.exe <repo>/learning/goal_reach/play.py --checkpoint /d/isaac/runs/goal_reach/base/model_299.pt \
  --out /d/isaac/runs/goal_reach/base/video --num_envs 4 --episodes 1 --video --viz none
.venv/Scripts/python.exe <repo>/learning/goal_reach/lidar_close_check.py --out <dir> --viz none

.venv/Scripts/python.exe <repo>/learning/goal_reach/plot_results.py curve out.png base=/d/isaac/runs/goal_reach/base/metrics.csv
```

Use the venv python directly, never `uv run` / `uv sync` (see `learning/isaac_setup.md`). Run one
Isaac job at a time.
