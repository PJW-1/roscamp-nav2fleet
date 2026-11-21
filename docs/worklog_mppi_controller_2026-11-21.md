# 작업 기록: Nav2 goal용 MPPI 컨트롤러 추가 (2026-11-21)

브랜치 `feat/mppi-controller-2026-11-21`(main 0aeffaa에서). 계획 5번: Nav2 MPPI를 새 컨트롤러 ID `FollowPathMPPI`로 더했다.
`FollowPathGoal`(회피 담당의 RPP)은 그대로이고 기본도 그대로다. MPPI는 BT의 ControllerSelector로 골랐을 때만
마당(HALL) Nav2 goal을 몬다. 고정 경로(`FollowPath`, `FollowPathThrough`, `FollowPathReverse`)는 pode_fsm이
controller의 `follow_path`를 바로 부르므로 언제나 RPP다. RPP와 MPPI 비교 묶음은 다음 단계다(이번에는 짧은 연기 실행만).

같은 날 오후(2·3단계): 흔들림의 원인을 찾아 `CostCritic` 무게를 1.0으로 낮추고, 제자리 회전을 `RotationShimController`에 맡겼다
(`FollowPathMPPI`의 플러그인이 shim이고 그 안의 `primary_controller`가 MPPI다). 그 뒤 움직이는 장애물을 넣은 비교 묶음(`tools/nav_bench`)을 돌렸다.
아래 "2단계", "3단계" 절에 있다.

## 만든 것

| 파일 | 바꾼 것 |
|---|---|
| `pode/pode_nav_config/params/nav2_params.yaml` | `controller_plugins`에 `FollowPathMPPI`를 더하고(맨 끝), `FollowPathMPPI` 블록을 새로 두었다. MPPI 값은 모두 이 블록에만 있다 |
| `sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py` | `sim_nav_params`가 `nav_max_speed`로 RPP의 `desired_linear_vel`을 낮출 때 MPPI의 `vx_max`도 같이 낮춘다. 비교 때 두 컨트롤러의 최고 속도가 같아야 한다 |
| `tools/real_pode/make_params.py` | 실물 사본은 원래 nav2_params.yaml을 통째로 베끼므로 MPPI 블록도 그대로 따라간다(값을 따로 두지 않음). `--speed S`일 때 `FollowPathMPPI.vx_max`도 S로 바꾼다 |
| `pode/pode_nav_config/test/test_nav_config.py` | 새 시험 넷: MPPI가 뜨고 블록이 맞는지(플러그인, DiffDrive, `model_dt` = 1/`controller_frequency`, critic 모두 켬), 한계가 FollowPathGoal·Isaac `ActionLimits`·Gazebo DiffDrive와 같은지, BT 기본이 여전히 `FollowPathGoal`이고 저장소의 어떤 Python도 `controller_selector`에 쓰지 않는지(3단계부터 비교 도구 `tools/nav_bench/bench_node.py`만 예외), `FollowPathGoal`이 여전히 회피 담당 파일의 `follow_path_goal` 그대로인지 |
| `pode/pode_nav_config/test/test_nav2_launch.py`, `tools/test/test_real_pode.py`, `sim/fleet_gz_sim/test/test_launch_wiring.py` | 컨트롤러 목록, make_params의 바뀐 줄 수(5 → 6), 시뮬 속도 상한이 `vx_max`에도 걸리는지 |

BT(`bt/navigate_to_pose_w_replanning_and_recovery_goal.xml`)는 바꾸지 않았다. 이미 ControllerSelector
(`topic_name="controller_selector"`, `default_controller="FollowPathGoal"`)가 있다. pode_fsm, fleet, sim, tools 어디에도
`controller_selector`에 쓰는 코드가 없어(시험이 확인, 비교 도구만 예외) 아무도 고르지 않으면 지금과 똑같이 돈다.
`FollowPathGoal` 값(`pode/pode_avoidance/params/avoidance.yaml`)은 건드리지 않았다(`git diff main -- pode/pode_avoidance` 비어 있음).

## 컨트롤러 바꾸는 법

토픽은 `/<로봇>/controller_selector`(`std_msgs/msg/String`), 구독 QoS는 RELIABLE + TRANSIENT_LOCAL이다
(Gazebo에서 `ros2 topic info -v /pode1/controller_selector`로 확인). 보내는 쪽도 같은 QoS로 보낸다.

```bash
# MPPI로
ros2 topic pub -t 3 -r 2 --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathMPPI}"
# RPP(기본)로 되돌리기
ros2 topic pub -t 3 -r 2 --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathGoal}"
```

- 고른 값은 BT가 다음에 틱할 때(다음 Nav2 goal부터, 달리는 중이면 다음 틱에) 읽는다. goal 사이에 보내면 된다.
  BT 노드는 마지막 하나(KeepLast 1)만 본다.
- 고른 값은 같은 BT 트리가 쓰이는 동안만 남는다. bt_navigator가 다시 뜨거나 다른 BT 파일을 지정한 goal로 트리가 새로 만들어지면
  기본 `FollowPathGoal`로 돌아간다(위 `ros2 topic pub -t 3`은 보내고 끝나므로 새 트리에 값을 다시 주지 않는다). 그때는 다시 보낸다.
  되돌리려면 `FollowPathGoal`을 보낸다.
- 없는 이름을 보내면 controller_server가 그 goal을 거절한다(Nav2 코드 기준 `INVALID_CONTROLLER`, 시험하지 않음). 오타에 조심한다.
- 확인: controller_server를 debug로 띄우면 1 Hz 다시 계획 때마다 `Selected controller: FollowPathMPPI.`가 찍힌다.
  Gazebo: `ros2 launch fleet_gz_sim one_pode.launch.py ... nav_log_level:=pode1.controller_server:=debug`
  (그 노드만 debug, 나머지는 info).
- 비교 묶음에서 값을 바꿔 보려면 파일을 고치지 말고 `nav_overrides:=<yaml>`(시뮬 전용)로 `FollowPathMPPI`의 키만 덮는다.

## 결정 대기 목록

모두 `nav2_params.yaml`의 `FollowPathMPPI` 블록에 있다. 1단계에서 처음 돌려 본 값이다. 2·3단계에서 바꾸거나 더한 것은 아래 "결정 대기 목록 (2·3단계에서 더한 것)"에 있다.

| 항목 | 후보 | 고른 값 | 까닭 |
|---|---|---|---|
| `motion_model` | DiffDrive, Omni, Ackermann | DiffDrive | 핑키는 두 바퀴 차동 구동이다 |
| `model_dt` | 0.05, 0.1 | 0.05 | `controller_frequency` 20 Hz와 같아야 한다(Nav2 문서 권고). 시험이 이 관계를 본다 |
| `time_steps`(앞보기 시간) | 56(기본, 2.8 s), 40(2.0 s), 30(1.5 s) | 40 | 0.25 m/s에서 0.5 m 앞. 경기장이 2.3 m 대각선이고 통로가 0.38 m라 2.8 s(0.7 m)는 길고, 1.5 s(0.375 m)는 굽이를 늦게 본다. 계산량은 time_steps에 비례한다 |
| `batch_size`(표본 수) | 2000(기본), 1000, 500 | 1000 | 실물은 Pi4에서 Nav2가 돈다. 기본 2000 × 56은 Pi4에서 20 Hz를 못 맞출 수 있다(재지 않음). 1000 × 40은 기본의 약 36 %다. 실물 CPU를 재고 정한다 |
| `iteration_count` | 1, 2 | 1 | Nav2 권고. 반복을 늘리기보다 표본을 늘리는 쪽이 낫다고 한다 |
| `temperature` | 0.1, 0.3(기본), 1.0 | 0.3 | 낮으면 가장 좋은 궤적 몇 개만, 높으면 평균에 가깝게 섞는다. 기본에서 시작 |
| `gamma` | 0.015(기본) | 0.015 | 제어 크기 벌점. 기본 |
| `vx_std`, `wz_std` | 0.2/0.4(기본), 0.1/0.4 | 0.1, 0.4 | 속도 범위가 -0.05~0.25(0.3 폭)라 0.2는 표본 대부분이 한계에 잘린다. 회전은 ±1.0이라 기본 0.4를 둔다 |
| `vx_max` | 0.25 | 0.25 | FollowPathGoal `desired_linear_vel`, Isaac `v_max`와 같다. 시뮬은 `nav_max_speed`(기본 0.20)로, 실물은 `make_params.py --speed`로 낮춘다 |
| `vx_min` | 0.0(후진 없음), -0.05 | -0.05 | Isaac `v_min`(실물 BackUp 속도)과 같다. RPP(FollowPathGoal)는 `allow_reversing: false`라 다르다. MPPI가 goal 앞에서 조금 물러나 방향을 맞추는 것을 허용한다. 후진을 막으려면 0.0 |
| `wz_max` | 1.0 | 1.0 | Gazebo DiffDrive `max_velocity` 1.0(회전에도 걸림), Isaac `w_max`, FollowPathGoal `rotate_to_heading_angular_vel` |
| `ax_max`, `ax_min`, `az_max` | 2.0(Gazebo), 3.0/-3.0/3.5(기본) | 2.0, -2.0, 2.0 | Gazebo DiffDrive `max_acceleration` 2.0, Isaac `acc_v`, `acc_w`. FollowPathGoal의 `max_angular_accel`은 3.2지만 Gazebo가 2.0으로 자른다 |
| `prune_distance` | 1.7(기본), 1.0 | 1.0 | 앞보기 0.5 m의 두 배. 짧게 잘라 계산을 줄인다 |
| `critics` | 기본 8개, 줄인 것 | 기본 8개: Constraint, Cost, Goal, GoalAngle, PathAlign, PathFollow, PathAngle, PreferForward | Nav2 문서의 DiffDrive 예와 같다. 비교 전에는 빼지 않는다 |
| critic 무게 | Nav2 기본 | 기본에서 Cost만 1.0(Constraint 4, Cost 1.0, Goal 5, GoalAngle 3, PathAlign 14, PathFollow 5, PathAngle 2, PreferForward 5) | 기본값에서 시작. Cost는 2단계에서 3.81 → 1.0(아래 "2단계") |
| critic `threshold_to_consider` | 기본 1.4 m / 0.5 m | 1.4 → 0.5, 0.5 → 0.25 | 기본은 vx 0.5 m/s, 2.8 s(1.4 m 앞보기)에 맞춘 값이다. 앞보기가 0.5 m라 1.4는 0.5로, 0.5는 0.25로 줄였다(goal 가까이서 경로 따르기에서 goal 맞추기로 넘어가는 거리) |
| `CostCritic.consider_footprint` | false(원), true(다각형) | true | 몸 모양 다각형 발자국(local_costmap)으로 본다. 0.38 m 통로와 진열대 끝에서 원으로 보면 너무 넓거나 좁다. 계산이 늘어 `trajectory_point_step` 2로 줄였다 |
| `CostCritic.near_goal_distance` | 0.5(기본), 0.3 | 0.3 | goal 가까이서 비용 벌점을 늦게 끈다. 지점이 진열대·벽에 가깝다 |
| `PathAlignCritic.offset_from_furthest` 등 | 기본 20/5/4 | 기본 | 경로 점 개수 단위다. NavFn 경로가 2.5 cm 지도라 점 간격이 기본(5 cm)의 반이다. 그대로 두면 앞보기가 반이 되는 셈이다. 비교 묶음에서 본다 |
| `visualize` | false, true | false | 궤적 표시는 계산이 든다. 볼 때만 `nav_overrides`로 켠다 |
| 시뮬 속도 상한 | 안 걸기, `vx_max`에 걸기 | 걸기 | RPP는 `nav_max_speed`(기본 0.20)로 낮아지는데 MPPI만 0.25면 비교가 공정하지 않다. 회피 약속의 "속도 상한은 nav_max_speed"도 지킨다 |
| 실물 사본(`make_params.py`) | 0.25 그대로, `--speed` 따르기 | 따르기 | 같은 까닭. 값은 저장소 블록에서 그대로 베끼고 `vx_max` 줄만 바꾼다 |
| 목표 판정 | 따로 둠, 같은 `general_goal_checker` | 같은 것 | BT가 FollowPath에 `goal_checker_id="general_goal_checker"`를 준다. 두 컨트롤러를 같은 기준(0.05 m, 0.12 rad)으로 비교한다 |

## 결과

### 시험

- `ruff check pode/pode_nav_config tools/real_pode/make_params.py tools/test/test_real_pode.py sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py sim/fleet_gz_sim/test/test_launch_wiring.py`: All checks passed.
- WSL `wt_build`: `Summary: 15 packages finished`.
- 도메인 61: `pode_nav_config/test` 110 passed, `tools/test/test_real_pode.py` 124 passed, `sim/fleet_gz_sim/test/test_launch_wiring.py` 71 passed, `pode/pode_fsm/test/test_core.py` 146 passed.
- `git diff 0aeffaa -- pode/pode_avoidance pode/pode_nav_config/bt`는 비어 있다. nav2_params.yaml에서 지운 줄은 `controller_plugins` 한 줄(두 줄로 늘림)뿐이다.

### Gazebo 연기 실행(비교 묶음 아님)

한 대, v7, `one_pode.launch.py gui:=false rviz:=false fsm:=false start:=O_IN nav_log_level:=pode1.controller_server:=debug`,
도메인 61, `nav_max_speed` 기본 0.20(MPPI `vx_max`도 0.20으로 낮아진 것을 시뮬 사본에서 확인). 마당 Nav2 goal 넷을 차례로:
A O_IN → M_E(기본, FollowPathGoal), MPPI 고름, B M_E → O_IN, C O_IN → M_E, FollowPathGoal로 되돌림, D M_E → O_IN.
시간은 sim 시간(실제 위치 `/pode1/sim/truth` 기록), 오차는 실제 위치와 goal의 차이다. 판 1은 위치 기록 없이 돌렸다.

- controller_server 로그: `Created controller : FollowPathMPPI of type nav2_mppi_controller::MPPIController`,
  `Controller Server has FollowPath FollowPathThrough FollowPathReverse FollowPathGoal FollowPathMPPI  controllers available.`
- 고른 컨트롤러(debug `Selected controller:` 줄 수, 차례대로): 판 2 `10 FollowPathGoal → 96 FollowPathMPPI → 11 FollowPathGoal`.
  MPPI 구간에는 `Providing path to the controller FollowPathMPPI`가 찍혔다.

| 판 | goal | 컨트롤러 | 결과 | sim 시간 | 달린 길 | 끝 오차 | progress 실패 |
|---|---|---|---|---|---|---|---|
| 1 | A | FollowPathGoal | SUCCEEDED | (벽시계 13.3 s) | | | 0 |
| 1 | B | FollowPathMPPI | SUCCEEDED | (벽시계 52.9 s) | | | 3 |
| 1 | C | FollowPathMPPI | SUCCEEDED | (벽시계 50.5 s) | | | 2 |
| 1 | D | FollowPathGoal | SUCCEEDED | (벽시계 12.2 s) | | | 0 |
| 2 | A | FollowPathGoal | SUCCEEDED | 12.1 s | 1.13 m | 4.6 cm, 3.5° | 0 |
| 2 | B | FollowPathMPPI | SUCCEEDED | 28.1 s | 1.48 m | 1.4 cm, 7.1° | 1 |
| 2 | C | FollowPathMPPI | SUCCEEDED | 73.6 s | 2.21 m | 3.8 cm, 0.2° | 4 |
| 2 | D | FollowPathGoal | SUCCEEDED | 11.7 s | 1.10 m | 5.5 cm, 0.6° | 0 |
| 3(`vx_min` 0.0) | B | FollowPathMPPI | SUCCEEDED | 18.8 s | 1.22 m | 1.6 cm, 7.2° | 0 |
| 3(`vx_min` 0.0) | C | FollowPathMPPI | 끝나지 않음(150 s CLI 시간 초과) | 150 s+ | 1.24 m | 10.4 cm에서 83 s 머묾 | 12 |
| 3(`vx_min` 0.0) | D | FollowPathGoal | SUCCEEDED | 13.4 s | 1.17 m | 5.4 cm, 2.6° | 0 |

한 줄 결론: 바꾸기와 되돌리기는 된다. MPPI는 지금 값으로 Nav2 goal을 끝까지 몰지만, RPP보다 2~6배 느리고
제자리에서 앞뒤로 1~3 cm씩 흔들리다 progress checker(10 s 동안 0.05 m)에 걸려 BT 복구를 거친 뒤에야 도착한다.
판 3(후진 없음, `nav_overrides`로만 바꿈)은 B는 나았지만 C에서 goal 10 cm 앞(옆으로 10 cm 어긋남)에서 후진 없이는 다가가지 못해 끝나지
않았다. 그래서 기본은 `vx_min` -0.05 그대로 둔다. 값 조정은 비교 묶음 단계의 몫이다.

## 막힌 것

- MPPI 흔들림(위 표). → 2단계에서 원인(경로가 비용 띠 안, `CostCritic` 3.81)을 확인하고 고쳤다. 판 2 C에서 통로 x 0.61 부근, 방향은 맞는데 x가 0.58~0.63을 30 s 오갔다. 판 2 B는 출발 때 168° 제자리 회전에 9 s가 걸렸다(RPP는 1 rad/s로 3 s 안).
  짐작하는 원인(확인하지 않음): (1) NavFn 경로 점 간격이 2.5 cm라 `offset_from_furthest`(점 개수)가 앞보기를 반으로 줄임,
  (2) 앞보기 0.5 m와 progress checker의 0.05 m/10 s, (3) `CostCritic` 발자국 비용이 좁은 통로에서 앞으로 가는 표본을 깎음,
  (4) `vx_std` 0.1이 속도 0.2에 비해 작아 정지 근처 표본이 많음. 비교 묶음 전에 하나씩 `nav_overrides`로 본다.
- 컨트롤러 루프가 goal을 받은 직후 한 번씩 20 Hz를 놓쳤다(4.9~12.5 Hz, 판마다 3~4번). 계속 놓치지는 않았다. 실물 Pi4에서는 재지 않았다.
- 실물 로봇에 `ros-jazzy-nav2-mppi-controller`가 깔려 있는지 확인하지 않았다. `FollowPathMPPI`는 늘 뜨므로 없으면 Nav2 전체가 뜨지 않는다.
  `pode_nav_config/package.xml`에 `exec_depend`를 넣었다(rosdep으로 깔린다). 실물 배포 전에 로봇에서 확인해야 한다.

## 배운 것

- Nav2 BT의 ControllerSelector는 TRANSIENT_LOCAL 구독이라, goal 사이에 한 번 보내 두면 다음 goal에서 읽는다. 고른 값은 노드가 다시 뜰 때까지 남는다.
- MPPI는 제자리 회전을 따로 하지 않고 표본 궤적 중에서 고른다. 그래서 RPP의 `rotate_to_heading`처럼 1 rad/s로 깔끔하게 돌지 않았다.
- progress checker는 위치만 본다. 제자리 회전이나 앞뒤 흔들림은 "진전 없음"으로 센다. MPPI 판의 실패는 거의 이것이었다.
- 후진 허용(`vx_min` < 0)은 goal 바로 앞에서 옆으로 어긋났을 때 다가가는 데 필요했다(판 3 C).

## 2단계: 흔들림의 원인과 조정

### 원인 (증거로 확인)

`nav_overrides`로 `visualize: true`를 켜고, controller_server debug 로그, `/pode1/local_costmap/costmap`, 실제 위치, `cmd_vel`을 bag으로 남긴 판(trace1,
`CostCritic.cost_weight` 3.81, WSL `~/gz_runs/mppi_tune_2026-11-21/trace1`)에서 본 것:

- 흔들린 곳은 가운데 통로 x 0.61~0.65와 x 1.17~1.23, y 약 0.55다. NavFn 경로는 y 0.55~0.57로 난다(통로 한가운데 0.5875보다 남쪽).
- 로봇 중심은 local costmap 비용 0인 칸에 있었고, 앞쪽 경로 칸은 OccupancyGrid 70~87(Nav2 비용 약 180~223)이었다. 남쪽 진열대(y 0.365~0.405)에서
  0.15 m(local inflation 반경) 안쪽 띠다.
- 그 사이 `cmd_vel` vx는 1초에 1~4번 부호가 바뀌고 평균은 약 0이었다. C 구간은 150 s 안에 끝나지 않았고 그 구간에서만 `Failed to make progress` 10번(판 전체 12번).
- 0.2 m/s로 빠져나간 때는 BT 복구가 local costmap을 처음 지운 뒤 약 4 s였다(장애물 띠가 잠깐 사라진 때).

결론: Nav2 기본 `CostCritic` 무게 3.81에서는 경로를 따라 비용 띠로 들어가는 궤적의 벌점이 경로 따르기 critic들의 이득보다 커서, 비용 0인 칸에 멈춰 앞뒤 표본을
섞은 평균(≈ 0)을 냈다. 1단계에서 짐작한 원인 넷 중 (3)이 맞았다. (1) 경로 점 간격, (4) `vx_std`는 바꾸지 않고도 아래 조정으로 흔들림이 사라졌다.

### 무게 시험 (같은 네 구간 C, B, C2, B2, 판마다 한 번)

| `CostCritic.cost_weight` | 도착 | progress 실패 | sim 시간(구간별) |
|---|---|---|---|
| 3.81(기본, trace1) | C에서 못 감 | 12 | C 150 s+ |
| 1.0 | 4/4 | 0 | 13.8 ~ 19.7 s |
| 0.3 | 4/4 | 0 | 13.8 ~ 17.3 s |

1.0을 골랐다. 0.3도 됐지만 진열대에 붙는 쪽으로 더 기우는 값이라, 실패가 없어지는 가장 큰 무게 근처를 둔다. 충돌 판정(`collision_cost`, 발자국)은 무게와 상관없이 그대로다.

### 반복 연기 실행 (무게 1.0, 덮어쓰기 없음)

같은 goal 묶음: RPP로 O_IN → M_E(RC), M_E → O_IN(RB), MPPI로 다시 C(MC), B(MB). sim 시간, 판마다 새로 띄움.

| 판 | RC (RPP) | RB (RPP) | MC (MPPI) | MB (MPPI) | progress 실패 |
|---|---|---|---|---|---|
| final_1 | planner 응답 시간 초과(아래) | 출발점이라 짧음 | 14.0 s | planner 시간 초과 | 0 |
| final_2 | 11.0 s | 12.8 s | 16.4 s | 18.5 s | 0 |
| final_3 | 10.8 s | 13.1 s | 18.6 s | 15.8 s | 0 |
| final_4 | planner 응답 시간 초과 | 출발점이라 짧음 | 15.1 s | 16.7 s | 0 |

MPPI는 모든 판에서 HALL goal에 progress 실패 없이 갔다. RPP보다 약 1.3~1.5배 느리다.
"planner 응답 시간 초과"는 띄운 직후 첫 goal에서 BT가 `compute_path_to_pose` 응답을 20 ms 안에 못 받아 ABORT한 것으로, 컨트롤러와 상관없다
(`Timed out while waiting for action server to acknowledge goal request`). 비교 묶음은 3 s 안의 ABORT를 한 번 다시 보내고 `retried`로 남긴다.

### 남은 느림: 제자리 회전

비교 묶음 시범(`~/gz_runs/nav_bench_smoke`, 판마다 1번)에서 MPPI는 a_hall 23.5 s(RPP 9.5 s)였다. 출발 때 방향 오차 55°에서 wz가 -0.03~-0.09 rad/s뿐이었다.
`PathAngleCritic`은 `max_angle_to_furthest` 1.0 rad(57°)을 넘을 때만 일하므로 55°에서는 꺼져 있고, 다른 critic은 제자리 회전을 거의 밀지 않는다.
두 판을 견줬다(a_hall, c_moving, 판마다 2번, RPP와 번갈아):

| 판(덮어쓰기) | 시나리오 | RPP 시간 | MPPI 시간 | MPPI 성공 | MPPI 충돌·BLOCKED |
|---|---|---|---|---|---|
| 회전 shim(`RotationShimController`가 MPPI를 감쌈) | a_hall | 9.7, 9.9 s | 14.1, 16.1 s | 2/2 | 0, 0 |
| 회전 shim | c_moving | 20.2, 19.4 s | 61.0, 53.2 s | 2/2 | 0, 1 |
| PathAngle 무게 5, `max_angle_to_furthest` 0.35 rad | a_hall | 9.4, 9.6 s | 12.2, 12.2 s | 2/2 | 0, 0 |
| PathAngle 무게 5, 0.35 rad | c_moving | 20.0 s | 시간 초과 91 s, 52.6 s(끝 위치가 goal에서 벗어나 실패) | 0/2 | 1, 3 |

shim을 골랐다. PathAngle 판은 마당에서는 더 빨랐지만 움직이는 장애물 판에서 두 번 다 실패했다(진열대 서쪽 끝 x 0.55, y 0.3 부근에서 70 s 넘게 맴돌고,
돌아오기도 못 해 그 판은 1회로 끝났다). shim은 경로 방향과 14° 넘게 틀어지면 MPPI를 멈추고 1 rad/s로 제자리에서 돈 뒤 MPPI에 넘긴다.

## 3단계: 움직이는 장애물과 비교 묶음 (계획 6번)

### 만든 것

- `tools/nav_bench/scenarios.yaml`: 장애물 크기·속도·길, 시나리오, 지표 기준값을 모두 여기 한 곳에 둔다.
- `tools/nav_bench/movers.py`: 장애물 모델(SDF), 길 위의 위치(왕복 또는 한 번), 몸 다각형과 진열대·벽·장애물 사이 거리.
- `tools/nav_bench/bench_node.py`: 장애물을 Gazebo `set_pose_vector`로 20 Hz 움직이고, `controller_selector`로 컨트롤러를 고르고, goal을 보내고,
  실제 위치(`/pode1/sim/truth`), `cmd_vel`, `scan`, controller_server CPU 시간을 판마다 JSON으로 남긴다. 판이 끝나면 RPP로 출발점에 돌아온다
  (세 번 실패하면 Gazebo에서 출발점으로 옮기고 AMCL에 `initialpose`를 준다).
- `tools/nav_bench/metrics.py`, `report.py`: 지표와 표·그림.
- `tools/nav_bench/run_bench.sh`: 시나리오마다 잠금(`.heavy_run.lock`)을 잡고 Gazebo 한 대를 띄워 RPP와 MPPI를 판마다 번갈아(짝수 판 RPP 먼저, 홀수 판 MPPI 먼저) 돈 뒤
  모두 끄고 잠금을 놓는다. 다음 시나리오 전에 180 s 쉰다.

다시 돌리기(WSL, install을 source한 셸):

```bash
BENCH_REPEATS=5 bash tools/nav_bench/run_bench.sh ~/gz_runs/nav_bench_2026-11-21 a_hall b_static c_moving d_robot
python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml \
  --legs ~/gz_runs/nav_bench_2026-11-21/legs --out results/2026-11-21/mppi_benchmark
```

### 결정 대기 목록 (2·3단계에서 더한 것)

MPPI 값은 `nav2_params.yaml`의 `FollowPathMPPI` 블록에, 장애물·지표 값은 `tools/nav_bench/scenarios.yaml`에만 있다.

| 항목 | 후보 | 고른 값 | 까닭과 잰 것 |
|---|---|---|---|
| `CostCritic.cost_weight` | 3.81(기본), 1.0, 0.3 | 1.0 | 3.81은 통로에서 흔들려 C 구간을 못 갔다(progress 실패 12). 1.0과 0.3은 네 구간 모두 실패 0(위 표). 실패가 없는 쪽에서 큰 값을 골라 진열대에서 떨어지려는 힘을 남겼다 |
| 제자리 회전 | MPPI 그대로, PathAngle 무게 5·문턱 0.35 rad, `RotationShimController` | shim(문턱 0.25 rad, 1.0 rad/s, 2.0 rad/s², 앞 0.15 m, 1.0 s 앞 충돌 검사, goal 방향도 제자리 회전) | 그대로는 a_hall 23.5 s(RPP 9.5 s). PathAngle 판은 a_hall 12.2 s지만 c_moving 0/2. shim은 a_hall 14.1·16.1 s, c_moving 2/2(위 표). 회전 값은 FollowPathGoal(1.0 rad/s)과 `az_max`·Gazebo(2.0)와 같다 |
| MPPI 전용 progress checker | 같이 씀, 따로 둠 | 같이 씀(0.05 m/10 s) | 무게 1.0과 shim으로 HALL goal에서 progress 실패가 0이 되어 따로 둘 까닭이 없었다. RPP 값도 그대로다 |
| (b) 가만히 있는 상자 | | 0.10 × 0.10 × 0.25 m, (1.10, 0.66) | 통로 북쪽 절반을 막아 남쪽으로 0.215 m(몸 폭 0.112 m + 여유)를 남긴다. 피해 갈지 돌아갈지 컨트롤러와 planner가 정한다 |
| (c) 사람 대역 | | 원기둥 반지름 0.04 m, 높이 0.25 m, (1.12, 0.28)↔(1.12, 0.70), 0.10 m/s | 축척 1/10쯤의 사람 발 둘레. 남쪽 진열대 틈(x 0.99~1.25)으로 통로를 가로지른다. 걷는 속도의 1/10 |
| (c) 카트 대역 | | 상자 0.14 × 0.09 × 0.20 m, (0.80, 0.68)↔(1.45, 0.68), 0.06 m/s | 통로 북쪽 줄을 천천히 오가는 짐수레. 우리 로봇보다 느려 앞지르거나 마주친다 |
| (d) 마주 오는 로봇 | 둘째 Nav2 로봇, 대본대로 움직이는 Pode 크기 상자 | 대본 상자 0.12 × 0.112 × 0.20 m, (1.62, 0.70) → (0.50, 0.70) 한 번, 0.15 m/s, 우리 로봇 중심과 0.25 m 안이면 멈춤 | 둘째 Nav2는 판마다 결과가 그 로봇의 컨트롤러에도 달려 비교가 흐려지고 메모리도 두 배다. 멈춤은 Pode가 충돌 검사로 서는 것을 흉내 낸다. 처음 둔 y 0.65는 남쪽 진열대와의 틈이 몸 폭과 비슷해 RPP도 끝나지 않아 0.70으로 옮겼다 |
| 장애물 높이 | | 모두 0.20 m 이상 | 라이다 높이 약 0.125 m 위로 올라와야 scan에 보인다 |
| 충돌 | Gazebo 접촉, 거리 | 몸 다각형(local_costmap 발자국을 실제 위치에 놓음)과 벽·진열대(`sim_shelves`)·장애물 면의 거리 ≤ 0.005 m가 된 횟수(떨어졌다 다시 닿으면 다시 셈) | 접촉 센서를 붙이려면 모델을 바꿔야 한다. 거리는 실제 위치에서 바로 잰다. 0.005 m는 다각형과 실제 몸의 차보다 작다 |
| BLOCKED | | 실제 위치가 10 s 동안 0.05 m보다 덜 움직인 구간 하나 | pode_fsm `blocked_after_s` 10 s, progress checker 0.05 m와 같다 |
| 성공 | Nav2 결과만, 실제 위치 | SUCCEEDED이면서 실제 끝 위치가 goal에서 0.10 m 안 | 시범 판에서 AMCL이 틀린 채 SUCCEEDED로 끝난 판이 있었다(실제로는 0.2 m 넘게 벗어남) |
| 시간 | | goal을 보낸 때부터 결과까지 sim 초 | 출발·도착 사건. 시간 초과는 90 s |
| 라이다 최소 | | 그 판 scan 최솟값 | 계획 7절. 라이다는 몸 가운데라 몸 끝과의 거리와 다르다. 그래서 몸 최소 거리를 따로 둔다 |
| 도리도리 | | `cmd_vel` 각속도 부호가 바뀐 횟수, \|wz\| < 0.02 rad/s는 건너뜀 | 0 근처 잡음까지 세지 않게 |
| CPU 대역 | 루프 시간 직접, 프로세스 CPU | controller_server 프로세스 CPU %(한 코어 기준, /proc)와 `cmd_vel` 간격 p95 | 루프 시간을 Nav2가 바깥으로 내지 않는다. 같은 PC에서 두 컨트롤러가 같은 프로세스라 차이가 컨트롤러 몫이다 |
| 시작 직후 ABORT | 실패로 셈, 다시 보냄 | 3 s 안에 움직이지 않고 ABORT면 한 번 다시 보내고 `retried`로 남김 | planner 응답 20 ms 시간 초과로, 컨트롤러와 상관없다(2단계) |
| 돌아오기 실패 | 그 시나리오를 멈춤, 옮김 | RPP로 세 번 못 돌아오면 Gazebo에서 출발점으로 옮기고 AMCL에 알림 | 시범에서 한 번의 돌아오기 실패로 시나리오가 반만 돌았다. 옮긴 횟수는 `runs.txt`에 남는다 |

### 비교 표 (2026-11-21 17:05~17:42, 시나리오마다 컨트롤러마다 5판)

조건: v7 경기장 한 대, O_IN → M_E, `nav_max_speed` 기본 0.20 m/s(두 컨트롤러 모두), 도메인 61, `GALLIUM_DRIVER=d3d12`, 덮어쓰기 없음
(커밋 4456358의 `FollowPathMPPI` = shim + MPPI, `CostCritic` 1.0). 시간·거리는 성공한 판의 평균 ± 표준편차, 충돌과 BLOCKED는 5판의 합이다.
같은 표와 판별 값, 그림은 `results/2026-11-21/mppi_benchmark/`(`table.md`, `legs.csv`, `compare.png`, `tracks.png`, `runs.txt`)에 있고,
판별 원본 JSON과 launch 로그는 WSL `~/gz_runs/nav_bench_2026-11-21/`에 있다.

| 시나리오 | 컨트롤러 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 | 충돌 | BLOCKED | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|
| a_hall | RPP | 5/5 | 9.6 ± 0.2 | 1.095 ± 0.028 | 0.097 ± 0.003 | 0.101 ± 0.004 | 9 ± 2 | 0 | 0 | 10 ± 0 | 52.0 ± 0.0 |
| a_hall | MPPI | 5/5 | 16.6 ± 5.1 | 1.126 ± 0.065 | 0.097 ± 0.010 | 0.098 ± 0.012 | 7 ± 4 | 0 | 1 | 12 ± 0 | 60.0 ± 0.0 |
| b_static | RPP | 5/5 | 16.5 ± 3.7 | 1.544 ± 0.364 | 0.075 ± 0.023 | 0.066 ± 0.019 | 9 ± 2 | 0 | 0 | 10 ± 0 | 52.0 ± 0.0 |
| b_static | MPPI | 5/5 | 33.6 ± 7.4 | 1.782 ± 0.322 | 0.061 ± 0.014 | 0.053 ± 0.020 | 10 ± 3 | 0 | 0 | 12 ± 1 | 60.0 ± 0.0 |
| c_moving | RPP | 5/5 | 20.7 ± 4.0 | 1.666 ± 0.309 | 0.075 ± 0.015 | 0.060 ± 0.034 | 12 ± 4 | 2 | 0 | 10 ± 0 | 52.0 ± 0.0 |
| c_moving | MPPI | 5/5 | 37.2 ± 13.4 | 1.981 ± 0.140 | 0.056 ± 0.005 | 0.039 ± 0.022 | 11 ± 4 | 1 | 0 | 11 ± 0 | 60.0 ± 0.0 |
| d_robot | RPP | 5/5 | 11.5 ± 1.0 | 1.108 ± 0.023 | 0.053 ± 0.003 | 0.046 ± 0.005 | 13 ± 3 | 0 | 0 | 10 ± 0 | 52.0 ± 0.0 |
| d_robot | MPPI | 5/5 | 25.4 ± 8.3 | 1.380 ± 0.286 | 0.060 ± 0.019 | 0.055 ± 0.013 | 11 ± 3 | 0 | 2 | 13 ± 1 | 60.0 ± 0.0 |

- 40판 모두 성공(Nav2 SUCCEEDED, 실제 끝 위치 0.10 m 안)이고, 돌아오기 실패로 옮긴 판도, 시작 직후 ABORT로 다시 보낸 판(`retried`)도 없다.
  controller_server의 `Failed to make progress`는 a_hall·b_static·c_moving 0줄, d_robot 2줄(둘 다 MPPI 판 3·4, 로그 시각으로 맞춤)이다.
- MPPI는 모든 시나리오에서 RPP보다 1.7~2.2배 느리고 퍼짐도 크다. 도리도리(각속도 부호 바꿈)는 비슷하다(퍼짐 안).
- 길: b_static과 c_moving은 planner가 가운데 통로가 막혔다고 보고 남쪽 진열대 바깥(y 약 0.18)으로 돌아간 판이 많다.
  남쪽으로 돈 판: b_static RPP 3/5, MPPI 4/5, c_moving RPP 4/5, MPPI 5/5, d_robot MPPI 1/5. 그래서 이 두 시나리오의 시간 차이는
  "통로에서 피하는 솜씨"보다 "긴 길을 도는 속도와 길 고르기"의 차이가 크다. 통로 안 회피만 견주려면 장애물 자리를 바꿔야 한다(남은 일).
- 충돌 셋은 모두 사람 대역과의 접촉이다(c_moving RPP는 한 판에서 두 번, x 1.04 통로 안. MPPI는 한 판에서 한 번, x 1.14·y 0.20 남쪽 길). 사람 대역은 비키지 않고
  정해진 길(y 0.28~0.70)을 가므로, 사람이 로봇에 걸어 들어온 경우도 충돌로 센다. 진열대·벽과의 최소 거리는 모든 판에서 0.045 m 이상이었다.
- CPU %는 controller_server 프로세스 전체로 RPP 10 %, MPPI 11~13 %(이 PC 한 코어 기준)이다. `cmd_vel` 간격 p95는 RPP 52 ms, MPPI 60 ms(20 Hz면 50 ms)로,
  MPPI 루프가 가끔 50 ms를 넘는다는 뜻이다. Pi4에서는 재지 않았다.
- a_hall MPPI 다섯 판은 progress 실패 없이 갔다(2단계의 "약 3번 반복" 기준을 shim 판에서도 확인). 실제 위치로 센 BLOCKED 1번은 25.3 s 판이다.

## 알아야 할 포인트 (개념 E: MPPI 기초)

- **MPPI가 하는 일**: 매 주기(0.05 s) 지금 명령 둘레에 잡음을 섞은 속도 열을 `batch_size`(1000)개 만들고, 각 열을 `time_steps`(40) × `model_dt`(0.05 s) = 2 s 앞까지
  DiffDrive 모델로 굴려 궤적을 얻는다. critic들이 궤적마다 비용을 매기고, 비용이 낮을수록 큰 무게 `exp(-비용/temperature)`로 섞은 평균 명령을 낸다.
  - 표본 수: 많을수록 좋은 궤적을 찾을 가능성이 크지만 CPU가 비례해 든다.
  - 앞보기(horizon): 길면 굽이를 일찍 보지만 계산이 늘고 좁은 곳에서 먼 장애물 비용에 휘둘린다.
  - temperature: 낮으면 가장 좋은 몇 개만 따르고(날카롭지만 흔들림), 높으면 평균에 가깝다(부드럽지만 둔함).
  - critic: 경로 따르기(PathFollow, PathAlign, PathAngle), goal 맞추기(Goal, GoalAngle), 안전(Cost), 한계(Constraint), 앞으로 가기(PreferForward). 무게의 비가 성격을 정한다.
- **왜 MPPI인가, 다른 후보**: RPP는 경로 위 한 점(당근)만 보고 곡률을 정해 단순하고 빠르지만 장애물을 스스로 피하지 않는다(충돌 검사로 설 뿐).
  DWB는 속도 표본을 한 걸음 앞만 굴려 평가하고 튜닝 항목이 많다. MPPI는 여러 걸음 앞까지 굴리고 비용으로 피해 가므로, 다른 로봇이 있는 마당에서 비교해 볼 가치가 있다.
  나중의 RL 정책과도 "명령 열을 평가해 고른다"는 점이 비슷해 비교 기준으로 좋다.
- **코드에서 볼 곳(셋)**:
  1. `pode/pode_nav_config/params/nav2_params.yaml`의 `FollowPathMPPI` 블록: 모든 값.
  2. `pode/pode_nav_config/bt/navigate_to_pose_w_replanning_and_recovery_goal.xml` 23행 `ControllerSelector`: 어떤 이름을 쓸지 고르는 곳.
  3. `sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py`의 `sim_nav_params`: 시뮬에서 `vx_max`를 `nav_max_speed`로 낮추는 곳.
- **숫자 읽는 법**: 표의 sim 시간은 goal 보낸 때부터 결과까지, 달린 길은 실제 위치를 이은 길이(곧은 거리는 약 1.16 m)다.
  달린 길이 곧은 거리보다 많이 길면 돌아가거나 흔들린 것이다. "progress 실패"는 controller_server의 `Failed to make progress` 줄 수이고,
  하나마다 BT가 costmap을 지우고 다시 보낸다(약 10 s 손해). 끝 오차는 goal checker 기준(0.05 m, 0.12 rad ≈ 6.9°)과 견준다.
  실제 위치로 잰 값이라 AMCL이 본 오차와 몇 cm 다를 수 있다(goal checker는 AMCL 자세로 판정하므로 판 2 B는 실제 7.1°여도 성공으로 끝났다).
- **스스로 점검**:
  1. `time_steps`를 40에서 60으로 늘리면 앞보기 거리와 계산량은 각각 어떻게 바뀌나? `model_dt`는 왜 그대로 두나?
  2. `temperature`를 0.3에서 0.05로 낮추면 흔들림이 늘까 줄까? 이유는?
  3. 판 3에서 `vx_min`을 0으로 두자 goal 10 cm 앞에서 왜 멈췄나? RPP(FollowPathGoal)는 같은 자리에서 어떻게 했을까?

### 2단계에서 더한 것 (개념 E: critic 무게의 균형, 제자리 회전)

- **critic 무게는 서로의 비로 읽는다**: MPPI는 궤적마다 critic 비용을 더한 합으로 고른다. `CostCritic`은 궤적이 지나는 칸의 costmap 비용(0~254)을
  254로 나눠 무게를 곱한다. 경로가 inflation 띠(비용 약 180) 안으로 나 있으면 경로를 따라가는 궤적일수록 벌점을 받는다. 그 벌점이 PathFollow·PathAlign의
  이득보다 크면, 로봇은 비용 0인 칸에 멈춰 앞뒤 표본의 평균(≈ 0)을 낸다. 이것이 1~3 cm 흔들림의 정체였다.
- **원인을 찾는 순서**: (1) 흔들리는 자리와 시간을 실제 위치로 찾는다. (2) 그때의 local costmap에서 로봇 중심 칸과 앞쪽 경로 칸의 비용을 읽는다.
  (3) `cmd_vel` vx의 부호 바꿈과 평균을 본다. (4) 의심 critic 하나의 무게만 `nav_overrides`로 바꿔 같은 구간을 다시 몬다. 하나씩 바꿔야 무엇이 고쳤는지 안다.
- **제자리 회전은 MPPI의 약점**: MPPI는 잡음 섞인 표본의 평균이라, 큰 각속도 하나로 도는 궤적이 드물고 평균하면 작아진다. `PathAngleCritic`은
  `max_angle_to_furthest`(1.0 rad)를 넘어야 일한다. `RotationShimController`는 경로 방향(0.15 m 앞 점)과 로봇 방향이 0.25 rad 넘게 다르면
  MPPI를 쉬게 하고 1 rad/s로 제자리에서 돈 뒤 넘긴다. 플러그인 이름만 shim으로 바뀌고 MPPI 값은 같은 `FollowPathMPPI` 이름 아래에서 그대로 읽힌다.
- **스스로 점검**:
  1. 경로가 통로 한가운데(y 0.5875)로 났다면 `CostCritic` 3.81에서도 흔들렸을까? 남쪽 진열대에서 그 점까지의 거리와 inflation 반경 0.15 m로 따져 본다.
  2. `CostCritic` 무게를 0으로 하면 무엇을 잃나? 충돌 판정(`collision_cost`)도 사라지나?
  3. shim의 `angular_dist_threshold`를 0.25에서 0.8로 올리면 a_hall 시간은 어느 쪽으로 움직일까?

### 3단계에서 더한 것 (개념 C: 같은 조건에서 비교하고 재기)

- **공정한 비교의 조건**: 같은 Gazebo 한 번 안에서 같은 출발점·goal·장애물 시작 위치로, 두 컨트롤러를 판마다 번갈아(순서도 판마다 바꿔) 돈다.
  그래야 시간이 지나며 생기는 차이(costmap에 남은 흔적, AMCL 흔들림, PC 부하)가 한쪽에 몰리지 않는다. 장애물의 길은 sim 시간으로 정해져 판마다 같다.
- **AMCL 없이 잰다**: 성공, 시간, 거리, 충돌은 모두 Gazebo의 실제 위치(`/pode1/sim/truth`)로 잰다. Nav2가 SUCCEEDED라고 해도 AMCL이 틀리면 엉뚱한 곳에
  서 있을 수 있다(시범 판에서 실제로 0.2 m 넘게 벗어난 SUCCEEDED가 있었다).
- **평균과 퍼짐, 실패는 따로**: 시간·거리는 성공한 판만으로 평균 ± 표준편차를 낸다. 실패한 판의 시간을 섞으면 시간 초과(90 s)가 평균을 끌어올려 뜻이 흐려진다.
  그래서 성공 수를 같이 본다. 판 수가 5라 1~2 s 차이는 우연일 수 있다. 퍼짐보다 큰 차이만 차이로 읽는다.
- **대역 지표의 한계**: 라이다 최소는 몸 가운데에서 잰 값이라 몸 끝과의 거리보다 크다. CPU %는 controller_server 프로세스 전체(costmap 갱신 포함)라
  컨트롤러만의 몫이 아니다. 같은 프로세스에서 컨트롤러만 바꿨으니 차이는 컨트롤러 몫으로 읽는다. Pi4에서의 값은 따로 재야 한다.
- **스스로 점검**:
  1. RPP 판을 모두 먼저 돌고 MPPI 판을 나중에 몰아 돌면 어떤 차이가 한쪽에 몰릴 수 있나?
  2. 성공 3/5, 시간 20 ± 2 s인 컨트롤러와 성공 5/5, 시간 25 ± 8 s인 컨트롤러 중 어느 쪽이 나은가? 무엇을 더 보고 정하나?
  3. 충돌을 "라이다 최소 ≤ 0.10 m"로 정의하면 무엇이 잘못 세어질까?

## 직접 해 볼 것

`PathAlignCritic`의 `cost_weight`(14.0)를 반으로 줄여 경로에 덜 붙게 하고, 같은 C 구간(O_IN → M_E)을 다시 몰아 시간과 progress 실패 수를 견준다.
파일은 고치지 않는다.

```bash
cat > /tmp/mppi_align7.yaml <<'Y'
controller_server:
  ros__parameters:
    FollowPathMPPI:
      PathAlignCritic:
        cost_weight: 7.0
Y
# 두 터미널 모두 같은 도메인(43~101 중 남이 안 쓰는 것)과 LOCALHOST로 둔다. 안 두면 launch가 격리 검사에서 멈춘다
export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 터미널 1
ros2 launch fleet_gz_sim one_pode.launch.py start:=O_IN nav_overrides:=/tmp/mppi_align7.yaml \
  nav_log_level:=pode1.controller_server:=debug
# 터미널 2
ros2 topic pub -t 3 -r 2 --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathMPPI}"
time ros2 action send_goal /pode1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.645, y: 0.55}, orientation: {w: 1.0}}}}"
```

RViz에서 로봇이 앞뒤로 흔들리는지 보고, 위 표의 판 2 C(73.6 s, 실패 4번)와 견준다.


### 3단계: 흔들림을 직접 되살려 보기

`CostCritic` 무게를 Nav2 기본 3.81로 되돌려 a_hall만 한 판(컨트롤러마다 1번) 돌리고 위 비교 표의 a_hall 줄과 견준다. 파일은 고치지 않는다.
Gazebo를 쓰므로 `.heavy_run.lock`을 run_bench.sh가 잡는다(다른 작업이 잡고 있으면 기다린다).

```bash
cat > /tmp/cost381.yaml <<'Y'
controller_server:
  ros__parameters:
    FollowPathMPPI:
      CostCritic:
        cost_weight: 3.81
Y
BENCH_REPEATS=1 BENCH_OVERRIDES=/tmp/cost381.yaml bash tools/nav_bench/run_bench.sh ~/gz_runs/try_cost381 a_hall
python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml --legs ~/gz_runs/try_cost381/legs --out /tmp/try_cost381
cat /tmp/try_cost381/table.md
```

MPPI 줄의 BLOCKED, 도리도리, 시간이 어떻게 바뀌는지 보고 `/tmp/try_cost381/tracks.png`에서 멈춘 자리가 x 0.6, 1.2 부근인지 확인한다.

## 남은 일

- ~~RPP와 MPPI 비교 묶음~~ 3단계에서 했다. 흔들림 원인은 2단계에서 찾았다.
- MPPI가 RPP보다 1.7~2.2배 느린 까닭: 통로에서 속도를 덜 내는지, shim 회전과 MPPI가 넘겨받을 때 멈추는지 판별 기록(`cmd`)으로 나눠 본다. 이번에는 조사하지 않았다.
- 통로 안 회피만 보는 시나리오: b_static·c_moving은 planner가 남쪽으로 돌아가는 판이 많아 컨트롤러끼리의 회피 비교가 흐리다. 장애물을 통로가 열려 있는 자리에 두는 판을 더한다.
- 사람 대역이 비키지 않아 생긴 접촉을 충돌에서 따로 셀지(누가 다가갔는지) 정한다.
- 판 수 5는 작다. MPPI 시간 퍼짐(± 5~13 s)이 커서 값을 바꾼 판끼리 견주려면 판을 늘린다.
- 실물 Pi4에서 MPPI 계산 시간(`batch_size` 1000, `time_steps` 40이 20 Hz를 맞추는지)과 패키지 설치 확인.
