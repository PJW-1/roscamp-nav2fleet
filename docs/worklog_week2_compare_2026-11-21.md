# 2주차 묶음: RL 벽 스침 원인과 RPP·MPPI·RL 세 controller 비교 — 2026-11-21

통합 브랜치 `integrate/week2-2026-11-21`(origin/main 0aeffaa에서 시작). 세 갈래를 `--no-ff`로 합치고, 합친 뒤에야
드러나는 어긋남을 고쳤다. 그다음 같은 속도 상한으로 RPP(FollowPathGoal), MPPI(FollowPathMPPI), RL(FollowPathRL,
store_arena_4b 정책)을 Gazebo nav_bench에서 견줬다. 그 전에 RL 브랜치에서 Q2 → E-3 벽 스침의 원인을 찾고 고쳤다.

**한 줄 결론:** 벽 스침은 시뮬 차이가 아니라 정책의 행동이다(같은 자세에서 Isaac 기하와 Gazebo 기하의 관측이 똑같고,
정책을 오프라인으로 돌리면 기록된 명령이 그대로 나온다). 플러그인 안의 작은 후진 막기로 Q2 → E-3 세 판의 충돌이
1/2/1에서 0/0/0이 됐다. 세 controller 비교 결과는 5절에 있다.

## 1. 합친 것

| 순서 | 브랜치 | 합친 커밋 | 병합 커밋 |
|---|---|---|---|
| 1 | `feat/store-arena-2026-11-21` | 6510857 (push된 것. 로컬 pd-arena의 2ddcef5는 아직 push 전이라 넣지 않음) | 78ea734 |
| 2 | `feat/mppi-controller-2026-11-21` | bd3ef24 | 1ee1dc9 |
| 3 | `feat/follow-path-rl-2026-11-21` | 4d0f3d5 (후진 막기 포함) | 2167f77 |

글자 충돌은 없었다. 합친 뒤 고친 것(의미 충돌):

| 커밋 | 고친 것 |
|---|---|
| d072514 | `follow_path_rl.yaml`의 `controller_plugins`에 FollowPathMPPI를 더함(목록이 통째로 바뀌므로 빠지면 MPPI가 꺼진다). nav_bench가 컨트롤러 셋을 받게 함: `run_order`를 n개 순서 돌리기로, `bench_node --controllers`, `run_bench.sh`의 `BENCH_CONFIG`·`BENCH_CONTROLLERS`, 잠금 재시도 60 s(전체 대기 시간은 같게). `report.py`가 기록에 있는 컨트롤러를 모두 표에 넣고, RL 이름표와 Welch 차이 표를 더함 |
| 8a41559 | 속도 상한 하나로: FollowPathRL에 `max_linear_vel`(0이면 정책의 `v_max`), 시뮬 `sim_nav_params`가 RL controller의 `max_linear_vel`도 `nav_max_speed`로 자름. MPPI 브랜치가 더한 `vx_max` 자르기를 기존 시험 `test_sim_params_cap_every_forward_controller_and_change_nothing_else`가 몰라서 실패하던 것을 시험에 반영(값을 명시적으로 확인하므로 기준을 낮춘 것은 아님) |
| ef6fbd6 | 리뷰가 찾은 다섯: `report.figure`가 `label()` 함수를 반복 변수로 가려 그림을 못 그리던 것, 4개 이상에서 이웃 균형이 깨지던 순서(Williams 설계로), 범례 색 이름, `max_linear_vel: 0` 덮어쓰기에 상한이 안 걸리던 것, 100 % 넘는 속도 비율 |

시험(WSL, 통합 브랜치 install):

```
colcon test pode_rl_controller pode_nav_config  ->  Summary: 138 tests, 0 errors, 0 failures, 0 skipped
pytest test_sim_nav_params test_launch_wiring test_nav_bench test_real_pode  ->  256 passed, 2 skipped
```

Windows: `learning` pytest(goal_reach, store_arena) 42 passed, `ruff check pc pode sim tools learning`는 4건인데
모두 main(0aeffaa)에도 있는 E501(arena.py, drive_check.py, fleet_setup.py, one_pode.launch.py)이다.

## 2. Q2 → E-3 벽 스침의 원인

지난 Gazebo 확인(`4b_215805`)에서 RL은 Q2 → E-3 세 판 모두 몸 최소 거리 0.000/0.003/0.002 m(충돌 1/2/1)였다.
판 0과 2는 출발 2.2 s에 남쪽 벽 (1.60, 0.085)에, 판 1은 9.1 s에 칸막이 2 끝 (0.553, 0.174)에 닿았다.

### 2.1 시뮬 차이인가 정책인가

Gazebo 기록의 실제 자세(0.2 s 간격, 처음 3 s)마다 학습 코드(`arena_spec.raycast_rects`, `task_spec.build_observation`,
`route_spec.follow`)로 관측을 다시 만들었다. 기하는 둘: Isaac의 실측 진열대(`shelves`)와 Gazebo의 `sim_shelves`.
그 관측을 정책(`policy.onnx`, Windows onnxruntime)에 넣었다.

| t s | x, y, yaw | 뒤 섹터 최소 m | 행동(Isaac 기하) | 행동(Gazebo 기하) | 목표 (v, w) | 기록된 명령 |
|---|---|---|---|---|---|---|
| 0.06 | 1.658, 0.150, +0.00 | 0.946 | (−3.57, +1.09) | 같음 | (−0.050, +1.00) | (−0.050, +0.20) |
| 0.92 | 1.617, 0.139, +0.65 | 0.178 | (−1.82, +1.32) | 같음 | (−0.050, +1.00) | (−0.050, +1.00) |
| 1.57 | 1.599, 0.113, +1.29 | 0.097 | (−2.80, +1.57) | 같음 | (−0.050, +1.00) | (−0.050, +1.00) |
| 2.00 | 1.598, 0.092, +1.72 | 0.075 | (−0.96, +2.52) | 같음 | (−0.044, +1.00) | (−0.050, +1.00) |
| 2.22 | 1.600, 0.083, +1.94 | 0.069 | (−0.48, +2.17) | 같음 | (+0.028, +1.00) | (+0.006, +1.00) |

(판 0. 판 1, 2도 소수 둘째 자리까지 같은 흐름이다.) 알 수 있는 것:

- 두 기하의 관측 차이는 모든 시점에서 0이다. 진열대 A 높이(0.793 대 0.803)는 Q2 근처 광선에 들어오지 않는다.
- 정책은 출발부터 2 s 동안 뒤로(−0.05 m/s) 가며 왼쪽으로 돈다. 기록된 명령과 같다(첫 명령의 w가 작은 것은 가속 제한).
- 가속 제한까지 넣은 운동학 굴리기(정책 + 몸 다각형, Gazebo 기하)도 2.20 s에 몸 거리 0.000 m가 된다. Gazebo와 같은 시각이다.

그래서 시뮬 차이가 아니라 정책이 고른 움직임이다.

### 2.2 학습이 이 상황을 봤나

store-arena 출발 분포(`store_arena_env.sample_start`): 경로(HALL 노드 쌍, `parking` 제외, 0.40 m 이상 떨어진 것) 44개에서
고르게 하나, 방향은 −π~π 고르게.

- Q2에서 출발하는 경로는 6개(13.6 %). Q2 → E-3도 그중 하나다(1/44).
- 6개 중 5개는 첫 경로 방향이 서쪽(+171~+175°)이다. 방향이 고르니 그중 약 25 %는 목표가 뒤(|상대각| > 135°)에서 시작한다.
- Q2는 남쪽 벽에서 0.15 m다. 0.20 m 안에 있는 출발 노드는 일곱(O_OUT, M_E, Q1, Q2, E-1, E-2, E-3)이다.

즉 학습은 이 상황을 자주 봤다. 그런데 보상은 접촉 힘 1 N을 넘을 때만 충돌로 끝내고(`collision_force`), 벽과의 여유에는
값을 주지 않는다. 진행 보상과 시간 벌점은 "제자리에서 돌며 조금 물러나 바로 출발"을 좋게 본다. 1 N 아래로 스치는 것은
학습에서 실패가 아니다(추정: Isaac에서 이 자세의 접촉 힘은 재지 않았다). nav_bench는 0.005 m 이하를 충돌로 센다.

몸 크기도 확인했다. URDF 충돌체 중 가장 뒤는 base_link 메시(x −0.0745 m)이고, 뒤 램프(−0.0725)와 캐스터 받침(−0.0725)은
그 안이다. 학습의 `COLLISION_BODIES`에 램프는 없지만 base_link가 더 뒤라 결과는 같다.

### 2.3 고른 것: 플러그인 안의 후진 막기 (기본)

`block_reverse`(`pode/pode_rl_controller/src/observation.cpp`): 정책의 목표 v가 음수이고, 정면 뒤 ±`rear_half_width_deg`
(30°) 안 광선 중 하나라도 `rear_stop_range`(0.12 m, 라이다 중심에서)보다 가까우면 v를 0으로 둔다. w는 그대로다.
`rear_stop_range: 0`이면 끈다. 몸 뒤끝이 라이다에서 0.058 m 뒤라 0.12 m는 몸 기준 약 6 cm 여유다.

| 대안 | 고르지 않은 까닭 |
|---|---|
| 다음 명령의 발자국을 local costmap에서 검사 | 더 일반적이지만 costmap 구독, 발자국 굴리기가 들어가 플러그인이 커진다. 계획은 충돌 감시를 collision_monitor로 따로 둔다 |
| 학습 바꾸기(벽 여유 벌점, 접촉 힘 문턱 낮추기, 후진 금지) | 근본 해결이지만 Isaac 재학습이 든다. Isaac 갈래의 몫으로 남긴다(7절) |
| 기하 바꾸기(Q2를 벽에서 더 떼기) | 실물 경기장 노드라 controller 문제를 지도에 떠넘기는 것이다 |

Gazebo 확인(잠금 아래, `guard_225128`, Q2 → E-3 세 판씩):

| | 전(4b_215805) | 후(guard_225128) |
|---|---|---|
| RL 성공 | 3/3 | 3/3 |
| RL 시간 s | 8.0 / 11.4 / 8.4 | 8.6 / 10.7 / 11.7 |
| RL 몸 최소 m | 0.000 / 0.003 / 0.002 | 0.050 / 0.032 / 0.029 |
| RL 충돌 | 1 / 2 / 1 | 0 / 0 / 0 |
| RPP 시간 s, 몸 최소 m | 11.2 / 10.9 / 10.3, 0.065 | 10.4 / 10.5 / 10.1, 0.065 |

후진 막기가 걸린 뒤 남쪽 벽 최소 거리는 0.050 m(판 0, t 1.4 s)다. 판 1, 2의 0.03 m는 E-3에 들어갈 때 칸막이 2 끝이다
(후진과 무관). 1 s 로그의 `rear blocks`는 판 동안 0~20회(20 Hz 기준 최대 1 s)였다.
