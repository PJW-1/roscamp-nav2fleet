# pode_rl_controller

Nav2 controller plugin `pode_rl_controller::FollowPathRL`: runs an RL driving policy (ONNX, ONNX
Runtime CPU) trained in `learning/goal_reach` or `learning/store_arena`. Design, decisions and results:
`docs/worklog_follow_path_rl_2026-11-21.md`.

| file | what it does |
|---|---|
| `src/policy_spec.cpp` | reads `policy_spec.json` (written by `learning/goal_reach/export_policy.py` from `task_spec.py`), format 2 or 3, and refuses an observation layout it does not build (format 3 adds `obs_blocks`: `prev_action`, `lidar_history`, `speed_cap`; `intent` is refused) |
| `src/observation.cpp` | LaserScan -> the 360 training rays, the 41-dim observation, action -> (v, w), rate limit; path progress and lookahead point (`route_spec.follow`) |
| `src/onnx_policy.cpp` | ONNX Runtime session (CPU, 1 thread, sequential, no spinning) |
| `src/follow_path_rl.cpp` | the plugin: scan subscription, goal point from the plan (the spec's goal rule), policy call, speed cap `max_linear_vel` (0 = the policy's v_max; the sim launch sets it to `nav_max_speed` like RPP and MPPI; never above `pode_nav_config/params/nav_speed.yaml`, and a policy whose v_max differs from that file logs an ERROR at configure and every 10 s while driving), reverse guard (no backing up while a lidar ray within `rear_half_width_deg` of straight back is under `rear_stop_range`, default off (0); a candidate value is 0.12 m / 30 deg), turn in place inside the goal checker's xy tolerance, timing log |
| `policies/<name>/` | `policy.onnx` + `policy_spec.json` (goal rule, lookahead, origin); parameter `policy` picks one, default `store_arena_4b`. Only the deployed policy's onnx is committed (user decision 2026-11-22); `goal_reach_base_4096` keeps only its spec, under `test/fixtures` |
| `test/test_parity.cpp` | gtest against `test/fixtures/<policy>` (made by `export_policy.py --fixtures`), built once per policy |
| `test/test_observation.cpp` | gtest: lookahead point and progress against `route_spec.follow`, goal and velocity terms against `task_spec`, lidar with the yaw-pi mount, reverse guard |
| `test/test_format3.cpp` | gtest: format 3 spec loading and refusals, `ObsMemory` (prev_action, lidar history, speed_cap), the cap-aware action mapping, when the memory resets |

Format 3 memory (`ObsMemory`): prev_action is the policy's mapped target before the guard, the speed limit
and the rate limit; the lidar history holds earlier sector vectors, newest first. Both return to the episode
start (prev_action 0, every history slot = the current frame) on activate, `reset()`, a plan whose last pose
moved more than 1 mm, and whenever the policy did not run in the previous control step (more than 2 control
periods since its last call: the RotationShim rotated or the plugin turned in place at the goal).

Not loaded by default: `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml` adds two controllers to
one_pode (`nav_overrides:=<that file>`): `FollowPathRL`, the default RL option, is the policy wrapped in
RotationShim; `FollowPathRLAlone` is the same policy unwrapped. `controller_selector` picks one.

Build needs `onnxruntime_vendor` (apt `ros-jazzy-onnxruntime-vendor`, ORT 1.24.3; amd64 and arm64).
Without sudo, unpack the .deb with `dpkg -x` and put `<dir>/opt/ros/jazzy` on `CMAKE_PREFIX_PATH`;
the library is found through its RPATH at run time.

```bash
colcon build --packages-select pode_rl_controller
colcon test --packages-select pode_rl_controller --event-handlers console_direct+
```
