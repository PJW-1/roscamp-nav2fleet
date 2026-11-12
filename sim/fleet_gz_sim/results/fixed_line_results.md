# 고정 선 주행, 충돌 검사 켬 (추가지시서3 2-1, 2026-11-13)

Gazebo 헤드리스, D2, 핑키 한 대(`one_pinky.launch.py`), 라이다 잡음 0.02, ROS 도메인 60.
주행은 `drive_check --plan stage2`(한 바퀴: P-1 → N_W → W → N_W(후진) → 급행 → N_E → O → N_E(후진) → 급행 → N_W → P-1)입니다.
끝 오차는 `/sim/truth`에서 잽니다.

## 실행 중 확인한 파라미터 (L1, `ros2 param get`)

| 것 | 값 |
|---|---|
| FollowPath, FollowPathThrough, FollowPathReverse, FollowPathGoal `use_collision_detection` | 모두 True |
| FollowPath, FollowPathThrough `lookahead_dist` / `use_velocity_scaled_lookahead_dist` | 0.1 / False |
| FollowPath, FollowPathThrough `rotate_to_heading_min_angle` | 0.7 |
| FollowPathGoal | 0.15, True, 0.25 (바꾸지 않음) |
| local footprint | 몸 모양 다각형 14점(`nav2_params.yaml`) |
| pinky_fsm `turn_radius_m` | 0.1 |

## 결과

| 실행 | 발자국 | 구간 | 결과 | 끝 위치(truth) | 끝 오차 |
|---|---|---|---|---|---|
| L1 | 몸 모양 다각형(폭 0.112) | 마당 goal P-1 → N_W | 성공 | (0.337, 0.627) | 0.026 m, 6.5° |
| L1 | 같음 | work_in N_W → X_W → W | `NAV_ABORTED` | (0.317, 0.861), 139° | 목표에서 0.140 m |
| L2n | 몸통만 직사각형(폭 0.090, 진단용 덮어쓰기) | 마당 goal P-1 → N_W | 성공 | (0.335, 0.627) | 0.028 m, 8.2° |
| L2n | 같음 | work_in N_W → X_W → W | `NAV_ABORTED` | (0.307, 0.868) | 목표에서 0.129 m |

두 실행 모두 work_in에서 멈췄으므로 그 뒤 구간(도크 후진, 급행 두 방향, 갈림목 내려오기, out_in)은 Gazebo에서 재지 못했습니다.
합격 기준(구간마다 3번 성공)은 채우지 못했습니다.

## 멈춘 까닭

- 멈춘 곳은 X_W 모서리 호(반지름 0.10 m)를 돌아 W로 들어가는 중입니다. 선에서 가장 멀어진 거리(`max_cross_track_m`)는 0.031 m였습니다.
- controller_server 로그는 매번 `RegulatedPurePursuitController detected collision ahead!`이고, FollowPath가 1초(`failure_tolerance`) 뒤 104로 끝납니다.
- L1에서 그 순간 local costmap의 치명 칸은 y 약 0.963(벽 면 0.99)과 잡음 점 (0.242, 0.94)였습니다. 선 가운데는 y 0.89입니다.
- 몸 모양 발자국의 바퀴 바깥은 선에서 0.056 m(y 0.946), 몸통만 쓴 발자국은 0.045 m(y 0.935)입니다. 라이다 잡음(σ 0.02)은 벽 면보다 2.5~5 cm 안쪽에 점을 찍으므로 두 발자국 모두 RPP가 앞으로 그려 보는 발자국에 걸립니다.
- 그래서 1~3단계(제자리 회전 없애기, 몸 모양 발자국, lookahead 0.10)만으로는 부족합니다. 발자국 폭을 0.090까지 줄여도 같은 곳에서 멈췄으므로 남은 원인은 벽 쪽 잡음 점입니다.

## 남은 차이 (수치)

| 것 | 값 |
|---|---|
| 띠 폭 | 0.20 m (선에서 벽 면까지 0.10 m) |
| 몸 모양 발자국 바깥 | 선에서 0.056 m, 벽 면까지 0.044 m |
| 걸린 잡음 점 | 벽 면에서 0.05 m 안쪽(y 0.94) |
| 지시서의 `wall_band_m` | 0.04 m: 벽 면에서 0.04 m 안의 점만 지우므로 위 점(0.05 m)은 남습니다 |
| 잡음 점을 지우는 데 필요한 띠 | 약 0.06 m(추정). 대가: 띠 안에서 벽에 붙은 0.06 m보다 얇은 물체는 보이지 않습니다 |

4단계(스캔 거르개)는 시간 안에 만들지 못했습니다. 위 수치로 보면 0.04 m 띠로는 이 점이 지워지지 않으므로 0.06 m 근처가 필요하다고 봅니다(추정, Gazebo에서 확인하지 않음).
