# item 8 다섯 컨트롤러 비교 결과

원본: `/home/pjw/rl_gz_check/item8_1123`

## 컨트롤러

- **RPP** (`FollowPathGoal`): `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`, desired_linear_vel 0.12 m/s, lookahead 0.15 m
- **MPPI** (`FollowPathMPPI`): `nav2_rotation_shim_controller::RotationShimController` wrapping `nav2_mppi_controller::MPPIController` (shim angular_dist_threshold 0.25, angular_disengage_threshold default 0.3925, rotate 1.0 rad/s, accel 2.0, simulate_ahead_time 1.0 s, rotate_to_goal_heading True); vx_max 0.12 m/s, time_steps 83, model_dt 0.05
- **RL+shim 4b** (`FollowPathRLShim`): `nav2_rotation_shim_controller::RotationShimController` wrapping `pode_rl_controller::FollowPathRL` (shim angular_dist_threshold 0.785, angular_disengage_threshold 0.39, rotate 1.0 rad/s, accel 2.0, simulate_ahead_time 1.0 s, rotate_to_goal_heading True); policy `store_arena_4b` (format 2, 41 inputs, blocks [], c2/model_998.pt from feat/retrain-012-2026-11-22 95e2054); max_linear_vel 0.12 m/s
- **RL+shim c** (`FollowPathRLShimC`): `nav2_rotation_shim_controller::RotationShimController` wrapping `pode_rl_controller::FollowPathRL` (shim angular_dist_threshold 0.785, angular_disengage_threshold 0.39, rotate 1.0 rad/s, accel 2.0, simulate_ahead_time 1.0 s, rotate_to_goal_heading True); policy `store_arena_c` (format 2, 41 inputs, blocks [], t_c/model_1197.pt from feat/recipe-2026-11-22 f178e50); max_linear_vel 0.12 m/s
- **RL+shim acd** (`FollowPathRLShimACD`): `nav2_rotation_shim_controller::RotationShimController` wrapping `pode_rl_controller::FollowPathRL` (shim angular_dist_threshold 0.785, angular_disengage_threshold 0.39, rotate 1.0 rad/s, accel 2.0, simulate_ahead_time 1.0 s, rotate_to_goal_heading True); policy `store_arena_acd` (format 3, 44 inputs, blocks ['prev_action', 'speed_cap'], t_combo_acd/model_1197.pt from feat/recipe-2026-11-22 f178e50); speed cap obs = min(max_linear_vel 0.12, nav_speed), trained range 0.08-0.16

## 결과

| name | scenario | runs | successes | wall_obstacle_contacts | robot_contacts | timeouts | mean_time_s | p90_time_s | min_clearance_m |
|---|---|---|---|---|---|---|---|---|---|
| RPP | a_hall | 6 | 6 | 0 | 0 | 0 | 14.1 | 15.6 | 0.092 |
| RPP | c_moving | 6 | 5 | 12 | 0 | 0 | 38.0 | 43.6 | 0.0 |
| RPP | d_robot | 6 | 6 | 0 | 0 | 0 | 15.5 | 20.9 | 0.044 |
| RPP | e_corridor | 6 | 6 | 0 | 0 | 0 | 13.4 | 13.8 | 0.077 |
| RPP | q2e3 | 3 | 3 | 0 | 0 | 0 | 14.1 | 14.3 | 0.065 |
| MPPI | a_hall | 6 | 6 | 0 | 0 | 0 | 13.9 | 15.7 | 0.088 |
| MPPI | c_moving | 6 | 5 | 8 | 0 | 0 | 36.8 | 55.2 | 0.0 |
| MPPI | d_robot | 6 | 6 | 0 | 0 | 0 | 17.1 | 19.4 | 0.038 |
| MPPI | e_corridor | 6 | 6 | 0 | 0 | 0 | 13.8 | 14.3 | 0.067 |
| MPPI | q2e3 | 3 | 3 | 0 | 0 | 0 | 13.7 | 14.7 | 0.061 |
| RL+shim 4b | a_hall | 6 | 6 | 0 | 0 | 0 | 12.6 | 12.8 | 0.097 |
| RL+shim 4b | c_moving | 6 | 5 | 2 | 0 | 0 | 25.3 | 32.1 | 0.0 |
| RL+shim 4b | d_robot | 6 | 6 | 0 | 0 | 0 | 13.1 | 15.8 | 0.031 |
| RL+shim 4b | e_corridor | 6 | 6 | 0 | 0 | 0 | 12.8 | 12.9 | 0.074 |
| RL+shim 4b | q2e3 | 3 | 3 | 0 | 0 | 0 | 17.9 | 21.4 | 0.061 |
| RL+shim c | a_hall | 6 | 6 | 0 | 0 | 0 | 12.6 | 12.7 | 0.097 |
| RL+shim c | c_moving | 6 | 5 | 14 | 0 | 0 | 24.4 | 30.1 | 0.0 |
| RL+shim c | d_robot | 6 | 6 | 0 | 0 | 0 | 13.0 | 13.2 | 0.022 |
| RL+shim c | e_corridor | 6 | 6 | 0 | 0 | 0 | 12.6 | 12.8 | 0.074 |
| RL+shim c | q2e3 | 3 | 3 | 0 | 0 | 0 | 14.9 | 16.5 | 0.062 |
| RL+shim acd | a_hall | 6 | 6 | 0 | 0 | 0 | 12.5 | 12.7 | 0.095 |
| RL+shim acd | c_moving | 6 | 4 | 11 | 0 | 0 | 26.8 | 33.0 | 0.0 |
| RL+shim acd | d_robot | 6 | 6 | 0 | 0 | 0 | 12.5 | 12.7 | 0.029 |
| RL+shim acd | e_corridor | 6 | 6 | 0 | 0 | 0 | 12.5 | 12.9 | 0.078 |
| RL+shim acd | q2e3 | 3 | 3 | 0 | 0 | 0 | 14.5 | 18.4 | 0.03 |
## 셈 기준

- runs·successes: 성공은 SUCCEEDED이면서 끝점이 목표에서 `arrive_m` 안일 때만이다(bench `leg_metrics`의 `ok`).
  c_moving 4번 판의 RL+shim 4b(1.26 m)와 RL+shim acd(0.57 m)는 Nav2가 SUCCEEDED를 냈지만 실제 위치가 목표에서
  멀어 실패로 셌다. Nav2가 보는 위치 추정이 실제와 어긋난 것으로 보인다 [추측].
- wall_obstacle_contacts는 벽과 `robot` 아닌 모든 장애물(c_moving의 움직이는 장애물 포함), robot_contacts는 `robot` 장애물에 대해,
  몸 윤곽 거리가 `collision_m` 아래로 내려간 횟수다(bench `events_below`). 실제 접촉 판정이 아니라 거리 문턱이다.
- timeouts는 bench 상태 TIMEOUT 수다. TIMEOUT은 없었다. 실패 여섯은 모두 c_moving이다: ABORTED 넷(RPP, MPPI, c, acd 하나씩)과 위의 SUCCEEDED 둘.
- 시간은 성공한 판만, p90은 가장 가까운 순위 값이다. q2e3은 Q2 -> E-3 3판, 나머지는 O_IN -> M_E 6판이다.

## shim 이른 넘김 (판 기록으로 센 추정치, DEBUG 로그 없음)

| 시나리오 | RL+shim 4b | RL+shim c | RL+shim acd |
|---|---|---|---|
| q2e3 (3판) | 12 (4.0/판) | 7 (2.3/판) | 4 (1.3/판) |
| a_hall (6판) | 3 | 1 | 2 |
| e_corridor (6판) | 1 | 0 | 0 |
| d_robot (6판) | 3 | 2 | 0 |
| c_moving (6판) | 10 | 7 | 14 |
