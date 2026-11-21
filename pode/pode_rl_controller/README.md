# pode_rl_controller

Nav2 controller plugin `pode_rl_controller::FollowPathRL`: runs an RL driving policy (ONNX, ONNX
Runtime CPU) trained in `learning/goal_reach` or `learning/store_arena`. Design, decisions and results:
`docs/worklog_follow_path_rl_2026-11-21.md`.

| file | what it does |
|---|---|
| `src/policy_spec.cpp` | reads `policy_spec.json` (written by `learning/goal_reach/export_policy.py` from `task_spec.py`) and refuses an observation layout it does not build |
| `src/observation.cpp` | LaserScan -> the 360 training rays, the 41-dim observation, action -> (v, w), rate limit; path progress and lookahead point (`route_spec.follow`) |
| `src/onnx_policy.cpp` | ONNX Runtime session (CPU, 1 thread, sequential, no spinning) |
| `src/follow_path_rl.cpp` | the plugin: scan subscription, goal point from the plan (the spec's goal rule), policy call, turn in place inside the goal checker's xy tolerance, timing log |
| `policies/<name>/` | `policy.onnx` + `policy_spec.json` (goal rule, lookahead, origin); parameter `policy` picks one, default `store_arena_4b` |
| `test/test_parity.cpp` | gtest against `test/fixtures/<policy>` (made by `export_policy.py --fixtures`), built once per policy |
| `test/test_observation.cpp` | gtest: lookahead point and progress against `route_spec.follow`, goal and velocity terms against `task_spec`, lidar with the yaw-pi mount |

Not loaded by default: `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml` adds it to one_pode
(`nav_overrides:=<that file>`), then `controller_selector` picks it.

Build needs `onnxruntime_vendor` (apt `ros-jazzy-onnxruntime-vendor`, ORT 1.24.3; amd64 and arm64).
Without sudo, unpack the .deb with `dpkg -x` and put `<dir>/opt/ros/jazzy` on `CMAKE_PREFIX_PATH`;
the library is found through its RPATH at run time.

```bash
colcon build --packages-select pode_rl_controller
colcon test --packages-select pode_rl_controller --event-handlers console_direct+
```
