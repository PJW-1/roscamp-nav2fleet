# Newton (MuJoCo-Warp) check for Pinky / Pode

Scripts that check whether Isaac Lab 3.0's Newton backend (`newton_mjwarp`, headless) can drive the
Pinky Pro URDF. Results and the summary are in `results/2026-11-21/newton_pinky_check/`; the log of
what was tried is `docs/worklog_newton_pinky_check_2026-11-21.md`.

| file | what it does |
|---|---|
| `pinky_cfg.py` | shared config: converted USD path, wheel radius/separation, `NewtonCfg` (MJWarp, implicitfast, pyramidal), `ArticulationCfg` with wheels in velocity mode (kp=0, kd=0.05, effort 0.5 N·m) and passive casters |
| `pinky_newton_drive.py` | drive sequence (settle, straight 0.10/0.20, stop, spin, arcs, stop), 200 Hz physics, 20 Hz control, CSV log + run JSON; `--accel/--alpha` ramp the commands, `--log_every 0` is the benchmark mode |
| `analyze_drive.py` | per-phase steady-state table, phase CSV, trajectory and speed plots |
| `pinky_newton_sensors.py` | `NewtonRaycastSensorCfg` 2D lidar (360 rays, 0.05-12 m) with a box moving across, then drive into a wall with contact sensors |
| `pinky_newton_randomize.py` | `ManagerBasedEnv` with startup events `randomize_rigid_body_material` (wheel/caster friction) and `randomize_rigid_body_mass` (base_link); reads the values back from the Newton model and measures their effect |
| `run_drive.sh`, `sweep_wheel_gain.sh`, `run_bench.sh` | wrappers for the drive runs, the wheel-gain sweep and the env-count benchmark |

## Environment

Isaac Lab 3.0.0 at `D:\isaac\IsaacLab` with its uv venv. Run the venv python directly from that folder:

```bash
cd /d/isaac/IsaacLab
PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1 \
  .venv/Scripts/python.exe <repo>/learning/sim_check/pinky_newton_drive.py --num_envs 16 --out <dir> --viz none
```

Do not use `uv run` here unless `UV_PYTHON_INSTALL_DIR` points at the interpreter the venv was built
with: on 2026-11-21 a `uv run` without it downloaded a new CPython and tried to delete and recreate
`.venv` (it failed only because another process held a file open). `isaaclab -p` is the same as
running the venv python.

## USD conversion

```bash
cd /d/isaac/IsaacLab
PYTHONUTF8=1 OMNI_KIT_ACCEPT_EULA=YES PXR_WORK_THREAD_LIMIT=1 .venv/Scripts/python.exe scripts/tools/convert_urdf.py \
  'C:\isaac_assets\pinky\pinky.urdf' 'C:\isaac_assets\pinky\usd\nomerge' \
  --joint_target_type velocity --joint_stiffness 0 --joint_damping 1 --viz none
```

`standalone_importers_available()` is False in this venv, so this used the Isaac Sim (Kit) importer,
not the kit-less one. Output: `C:\isaac_assets\pinky\usd\nomerge\pinky\pinky.usda`. The drive gains in
the USD are overridden by the actuator config at run time.

## Lidar note

The Newton ray caster hits every collision shape, including the robot's own. The lidar origin sits
inside the rplidar collider, so plain rays all return ~0.026 m. `pinky_newton_sensors.py` starts each
ray 0.05 m out (a custom pattern) and adds 0.05 m back. The ray caster rotates ray directions by
`offset.rot` but not ray starts, so with non-zero starts the 180° lidar yaw is left out and rays are
indexed in the base frame (ray 180 = base +x).
