# 작업 기록: Nav2 goal용 MPPI 컨트롤러 추가 (2026-11-21)

브랜치 `feat/mppi-controller-2026-11-21`(main 0aeffaa에서). 계획 5번: Nav2 MPPI를 새 컨트롤러 ID `FollowPathMPPI`로 더했다.
`FollowPathGoal`(회피 담당의 RPP)은 그대로이고 기본도 그대로다. MPPI는 BT의 ControllerSelector로 골랐을 때만
마당(HALL) Nav2 goal을 몬다. 고정 경로(`FollowPath`, `FollowPathThrough`, `FollowPathReverse`)는 pode_fsm이
controller의 `follow_path`를 바로 부르므로 언제나 RPP다. RPP와 MPPI 비교 묶음은 다음 단계다(이번에는 짧은 연기 실행만).

## 만든 것

| 파일 | 바꾼 것 |
|---|---|
| `pode/pode_nav_config/params/nav2_params.yaml` | `controller_plugins`에 `FollowPathMPPI`를 더하고(맨 끝), `FollowPathMPPI` 블록을 새로 두었다. MPPI 값은 모두 이 블록에만 있다 |
| `sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py` | `sim_nav_params`가 `nav_max_speed`로 RPP의 `desired_linear_vel`을 낮출 때 MPPI의 `vx_max`도 같이 낮춘다. 비교 때 두 컨트롤러의 최고 속도가 같아야 한다 |
| `tools/real_pode/make_params.py` | 실물 사본은 원래 nav2_params.yaml을 통째로 베끼므로 MPPI 블록도 그대로 따라간다(값을 따로 두지 않음). `--speed S`일 때 `FollowPathMPPI.vx_max`도 S로 바꾼다 |
| `pode/pode_nav_config/test/test_nav_config.py` | 새 시험 넷: MPPI가 뜨고 블록이 맞는지(플러그인, DiffDrive, `model_dt` = 1/`controller_frequency`, critic 모두 켬), 한계가 FollowPathGoal·Isaac `ActionLimits`·Gazebo DiffDrive와 같은지, BT 기본이 여전히 `FollowPathGoal`이고 저장소의 어떤 Python도 `controller_selector`에 쓰지 않는지, `FollowPathGoal`이 여전히 회피 담당 파일의 `follow_path_goal` 그대로인지 |
| `pode/pode_nav_config/test/test_nav2_launch.py`, `tools/test/test_real_pode.py`, `sim/fleet_gz_sim/test/test_launch_wiring.py` | 컨트롤러 목록, make_params의 바뀐 줄 수(5 → 6), 시뮬 속도 상한이 `vx_max`에도 걸리는지 |

BT(`bt/navigate_to_pose_w_replanning_and_recovery_goal.xml`)는 바꾸지 않았다. 이미 ControllerSelector
(`topic_name="controller_selector"`, `default_controller="FollowPathGoal"`)가 있다. pode_fsm, fleet, sim, tools 어디에도
`controller_selector`에 쓰는 코드가 없어(시험이 확인) 아무도 고르지 않으면 지금과 똑같이 돈다.
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
- 고른 값은 bt_navigator가 다시 뜰 때까지 남는다. 되돌리려면 `FollowPathGoal`을 보낸다.
- 없는 이름을 보내면 controller_server가 그 goal을 거절한다(Nav2 코드 기준 `INVALID_CONTROLLER`, 시험하지 않음). 오타에 조심한다.
- 확인: controller_server를 debug로 띄우면 1 Hz 다시 계획 때마다 `Selected controller: FollowPathMPPI.`가 찍힌다.
  Gazebo: `ros2 launch fleet_gz_sim one_pode.launch.py ... nav_log_level:=pode1.controller_server:=debug`
  (그 노드만 debug, 나머지는 info).
- 비교 묶음에서 값을 바꿔 보려면 파일을 고치지 말고 `nav_overrides:=<yaml>`(시뮬 전용)로 `FollowPathMPPI`의 키만 덮는다.

## 결정 대기 목록

모두 `nav2_params.yaml`의 `FollowPathMPPI` 블록에 있다. 지금 값은 "처음 돌려 볼 값"이고 비교 묶음에서 다시 본다.

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
| critic 무게 | Nav2 기본 | 기본(Constraint 4, Cost 3.81, Goal 5, GoalAngle 3, PathAlign 14, PathFollow 5, PathAngle 2, PreferForward 5) | 기본값에서 시작. 무게를 바꾼 판은 비교 묶음에서 |
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

- MPPI 흔들림(위 표). 판 2 C에서 통로 x 0.61 부근, 방향은 맞는데 x가 0.58~0.63을 30 s 오갔다. 판 2 B는 출발 때 168° 제자리 회전에 9 s가 걸렸다(RPP는 1 rad/s로 3 s 안).
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
# 터미널 1 (도메인 43~101 중 남이 안 쓰는 것, LOCALHOST)
ros2 launch fleet_gz_sim one_pode.launch.py start:=O_IN nav_overrides:=/tmp/mppi_align7.yaml \
  nav_log_level:=pode1.controller_server:=debug
# 터미널 2
ros2 topic pub -t 3 -r 2 --qos-durability transient_local --qos-reliability reliable \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathMPPI}"
time ros2 action send_goal /pode1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.645, y: 0.55}, orientation: {w: 1.0}}}}"
```

RViz에서 로봇이 앞뒤로 흔들리는지 보고, 위 표의 판 2 C(73.6 s, 실패 4번)와 견준다.

## 남은 일

- RPP와 MPPI 비교 묶음(Isaac 학습과 겹치지 않게). 그 전에 "막힌 것"의 원인 넷을 하나씩.
- 실물 Pi4에서 MPPI 계산 시간(`batch_size` 1000, `time_steps` 40이 20 Hz를 맞추는지)과 패키지 설치 확인.
