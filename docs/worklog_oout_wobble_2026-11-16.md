# O_OUT 도리도리 측정 (2026-11-16)

브랜치 `docs/oout-wobble-measure`. 바탕은 main 7c7d04b입니다. 코드는 바꾸지 않았고 측정 한 판의 기록만 남깁니다.

## 1. 무엇을 봤나

- 사용자가 Gazebo에서 출고 뒤 고정 경로(FollowPath)에서 Nav2 goal로 넘어갈 때 Pode가 좌우로 흔들리는 것("도리도리")을 봤습니다. Nav2 goal을 미리 보내면 부드럽게 넘어가는지 물었습니다.
- 같은 날 앞선 판(gz_order_1116)에서 본 것입니다.
  - 출고마다 관제가 먼저 로봇을 대기점 P-1로 보냅니다.
  - 로봇은 OUTBOUND에서 나오는 고정 경로로 O_OUT(지도 0.15, 0.62, 서쪽 벽에서 약 0.15 m)까지 가서 남쪽을 보고 1.2~2.8초 섭니다(한 번은 RECOVERING과 함께 5.7초).
  - 12번 중 8번은 그때 관제가 P-1을 버리고(이벤트 `STANDBY_LEFT`, `pc/fleet_core/fleet_core/traffic/zone/drive.py:97`) Q2로 보냈습니다. Q2로 가려면 55~90° 돌아야 합니다.
  - "collision ahead" 경고 196건 중 140건이 O_OUT 전환 뒤 8초 안에 나왔습니다.

## 2. 측정 판 설정

| 항목 | 값 |
|---|---|
| 코드 | main 7c7d04b, 변경 없음 |
| 모드 | v7, traffic_mode zones, use_order_model true |
| 시나리오 | gz_v7_rush (pode1~3, 주문 12건), sim_dock_stub true |
| 환경 | domain 61 LOCALHOST, Gazebo GUI + RViz |
| 기록 | rosbag(mcap, 153 MiB, 284초) |

rosbag에 담은 토픽은 `/podeN/`의 `cmd_vel`(Twist, controller 20 Hz), `odom`, `plan`, `local_costmap/costmap`(odom 좌표, 0.025 m), `published_footprint`, `lookahead_point`, `lookahead_collision_arc`, `received_global_plan`, `scan`, `scan_filtered`, `state`입니다.

판 결과는 PASS입니다.

| 항목 | 값 |
|---|---|
| 출고 | 12/12 |
| 걸린 시간 | 271.2초 |
| 로봇 사이 최소 거리 | 0.222 m |
| 잡은 구역 밖 | 0 |
| keepout | 0 |
| 시간 거꾸로 | 0 |
| real time factor | 0.969 |

방법은 이렇습니다.
- 전환 시각은 launch.log에서 뽑았습니다. `SEGMENT_DONE`(O_OUT 또는 E-x로 가는 follow_path)과 `STANDBY_LEFT`입니다. bag의 받은 시각과 launch.log는 둘 다 벽시계입니다.
- 제자리 회전은 cmd_vel의 |linear.x| < 0.01 이고 |angular.z| > 0.05 인 구간입니다. "부호 바뀜"은 이어진 제자리 회전 명령 사이에서 angular.z 부호가 바뀐 것입니다.
- 분석 스크립트는 세션 scratchpad에 있었고 저장소에는 두지 않았습니다.
- 숫자는 이 정의에 따라 달라집니다.
  - 전환 시각은 Nav2 goal을 받은 시각이 아니라 로그의 `SEGMENT_DONE`, `STANDBY_LEFT` 시각입니다. 같은 로봇의 1.5초 안 사건은 하나로 셌습니다.
  - 부호 바뀜은 제자리 회전 명령만 남겨 비교하므로, 사이에 멈춤이나 전진이 있어도 한 번으로 셉니다.
  - "0.25 m 벗어나기"는 전환 때 odom 위치 기준이고, 10초 안에 못 벗어나면 10초로 셉니다.
  - bag 받은 시각과 로그 시각 사이의 지연은 따로 재지 않았습니다. 0.4~0.6초 선후 판단에는 이 오차가 섞일 수 있습니다.

## 3. 결과

| 전환 자리 | 횟수 | 0.25 m 벗어나기까지 평균 | 회전 부호 바뀜 | collision ahead |
|---|---|---|---|---|
| O_OUT | 12 | 5.6초 | 합 32 (전환마다 1~6, 12번 모두) | 215 |
| E-x (충전 칸 입구) | 3 | 3.3초 | 0 | 26 |

앞으로 처음 움직이기 전(O_OUT)과 E-x의 비교입니다.

| 항목 | O_OUT | E-x |
|---|---|---|
| 첫 전진까지 평균 | 2.7초 | 1.9초 |
| 부호 바뀜 | 11 | 0 |
| 방향이 돈 총량 / 순 변화 | 921° / 799° (12번 합) | 191° / 191° |

- O_OUT의 부호 바뀜 11 가운데 9는 새 `/plan`이 오고 0.6초 안에 났습니다. 2는 새 plan과 collision 경고가 모두 앞섰습니다. 경고만 앞선 것은 0입니다.
- O_OUT에서 앞뒤로 더 돈 양은 약 122°(921 - 799)입니다.
- 다시 짠 경로의 첫 방향(0.15 m 앞)은 다시 짤 때마다 O_OUT에서 최대 58° 바뀌었습니다(20~47°가 흔했습니다). E-x는 0~11°입니다.
- O_OUT에서 전환 직후 local costmap의 가장 가까운 lethal(100) 칸은 로봇 중심에서 0.098~0.117 m였습니다. 제자리 회전으로 쓸고 가는 반지름은 0.083 m(`docs/실측_2026-11-10.md:31`)이므로 여유는 대략 1.5~3.4 cm입니다. 칸 중심까지의 거리이고 costmap과 odom을 따로 골랐으므로 정확한 충돌 여유는 아닙니다.
- 첫 collision 경고 때 가장 가까운 inscribed 또는 lethal 칸은 로봇 기준 방위 약 -90~-150°에 있었습니다. 로봇이 남쪽을 보므로 서쪽에서 북서쪽입니다.
- 관련 설정입니다.
  - FollowPathGoal(`pode/pode_avoidance/params/avoidance.yaml:48`): `rotate_to_heading_min_angle` 0.25 rad, `use_collision_detection` true.
  - 고정 경로 FollowPath: 0.70 rad(`pode/pode_nav_config/params/nav2_params.yaml:161`).
  - BT는 1 Hz로 다시 짭니다.

## 4. 해석

아래는 측정이 아니라 해석입니다.

- 해석: 같은 전환을 하는 E-x에서는 부호 바뀜이 없었으므로, 전환 자체가 늘 흔들림을 만드는 것은 아닙니다. 다만 E-x와 O_OUT은 다음 목표 방향, 들어오는 자세, 주변 장애물이 달라서 "O_OUT 자리만이 원인"이라고 가르지는 못합니다.
- (틀린 추정, 7절에서 바로잡음) 서쪽 벽 가까이에서는 다시 짠 경로의 시작 방향이 1 Hz 재계획마다 바뀝니다. 기준 각도 0.25 rad이면 제어기가 그때마다 반대로 다시 맞춥니다. 여유 1.5~3.4 cm의 충돌 검사가 더 느리게 만듭니다.
- 재지 않은 것: RPP의 충돌 검사가 실제로 어느 costmap 칸에 걸렸는지. 새 plan 직후 부호가 바뀐 것은 시간이 겹친다는 것이고 원인을 증명하지 않습니다. 고침 각각의 효과.
- Nav2 goal을 일찍 보내는 안의 효과는 재지 않았으므로 판단을 보류합니다. 다만 O_OUT은 도크 keepout 띠에서 약 2.5 cm 밖일 뿐입니다(`pc/fleet_core/fleet_core/data/fleet_map_v7.yaml:84-93`). 좁은 띠 안의 제자리 회전은 금지입니다(`docs/Pode_계약.md:155`, 12번).

## 5. 후보 (정하지 않았습니다)

- O_OUT을 서쪽 벽에서 동쪽으로 옮깁니다. 지도 변경이라 사용자가 정합니다. O_IN이 x 0.44라서 OUTBOUND에서 나오는 고정 경로 모양을 다시 잡아야 합니다. `docs/주행_계획서.md:299`에 이미 적혀 있습니다.
- FollowPathGoal의 `rotate_to_heading_min_angle`을 올리거나 `nav2_rotation_shim_controller`로 감쌉니다. 회피 담당의 파일입니다.
  - 문턱 값은 Gazebo `nav_overrides`로 저장소를 바꾸지 않고 시험할 수 있습니다. 시뮬은 avoidance 값을 먼저 넣고 그 위에 덮습니다(`sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py:181-197`).
  - Shim은 plugin을 바꾸는 것이라 `nav_overrides`로는 안 되고(`sim/fleet_gz_sim/launch/gz.launch.py:103`) `avoidance_package:=`가 필요합니다.
- 고정 경로의 끝을 다음 Nav2 방향으로 맞춥니다. 다음 목적지가 달라서 모두에게 맞출 수는 없습니다.
- 후보마다 한 가지만 바꿔 같은 측정으로 고치기 전과 후의 Gazebo 판을 비교해야 합니다. 이 프로젝트는 성능(처리 시간, 자연스러운 움직임)을 먼저 봅니다(사용자 기준 2026-11-16).

## 6. 하지 않은 것

- 고친 것은 없습니다.
- 측정은 한 판뿐입니다.
- 실물 로봇에서는 시험하지 않았습니다.

## 7. 원인 다시 보기 (같은 날, 같은 bag 재계산)

3절과 같은 bag을 다시 계산했습니다. 새 Gazebo 판은 돌리지 않았습니다. main은 2a06d72로 올라왔지만 주행에 닿는 파일(avoidance.yaml, nav2_params.yaml, order_fsm.py, adapters.py, drive.py, Gazebo launch)은 바뀌지 않았습니다. 스크립트는 `docs/oout_wobble_2026-11-16/`에 있습니다(`phases.py`, `timeline.py`는 같은 폴더의 `analyze_cause.py`를 읽습니다).

### 7.1 도리도리는 세 번의 회전입니다

- 고정 경로 out_out(O → (0.16, 0.86) → O_OUT, `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml:161`)은 서쪽으로 가다 왼쪽으로 꺾어 남쪽으로 들어옵니다.
- 끝에서 FollowPath가 오른쪽으로 돕니다(최대 약 0.9 rad/s, 약 0.6초). 이때 carrot은 왼쪽(+8~+27°)에 있습니다. 그래서 경로를 따르는 회전이 아니라 끝 자세(마지막 선분 방향, 남쪽)에 맞추는 회전으로 봅니다(해석). 끝 자세 허용치는 0.12 rad입니다(`pode/pode_nav_config/params/nav2_params.yaml:109`).
- 그다음 Nav2 goal이 왼쪽으로 42~78° 돌고 나서 출발합니다. 전환 0.3초 뒤 carrot은 12번 모두 로봇 왼쪽 +85~+106°(동쪽)였습니다.
- 출발 전 회전 방향 바뀜은 12번 모두 정확히 2번(왼 → 오 → 왼)입니다. 이 셈은 창을 전환 1.5초 전부터 잡아서 3절(1초 전부터)과 숫자가 다릅니다.
- 3절의 "새 plan 직후 바뀜"은 새 Nav2 goal의 첫 plan이 전환과 함께 오기 때문입니다. 바뀜 직전과 직후의 carrot 방위는 같은 쪽이었습니다(예: 88 → 89, 87 → 87). 4절의 "1 Hz 재계획이 방향을 바꾼다"는 추정은 틀렸습니다.
- 오른쪽 회전에서 왼쪽 회전으로 넘어갈 때 명령은 주기마다 odom 회전 속도 + 0.16 rad/s까지만 바뀝니다(`max_angular_accel` 3.2 × 0.05초, `pode/pode_avoidance/params/avoidance.yaml:49`).
- 도착에서 출발까지 1.7~3.8초였습니다.

### 7.2 collision ahead

- 위치: Nav2 Regulated Pure Pursuit 제어기입니다. 설치본 `/opt/ros/jazzy/lib/libnav2_regulated_pure_pursuit_controller.so`에 이 문자열이 있습니다. FollowPathGoal은 `use_collision_detection` true, `max_allowed_time_to_collision_up_to_carrot` 0.6입니다(`avoidance.yaml:38-39`).
- 하는 일: 제어 주기(20 Hz)마다 지금 명령으로 0.6초 앞까지 몸을 옮겨 보고, 발자국 외곽선이 치명 칸에 닿으면 그 주기의 명령을 버립니다. 발자국은 local costmap 발자국에 기본 덧댐 0.01을 더한 것입니다. controller_server는 `failure_tolerance` 1.0초 안이면 경고만 남기고 0 속도를 냅니다(`nav2_params.yaml:92`). 이 판에서 "patience exceeded"는 0번입니다.
- 제자리 회전의 투영 한 걸음은 14.6°입니다(칸 0.025 m, 덧댄 발자국의 원 반경 0.098 m). 0.6초 안에 한 걸음이 들어가려면 |w| > 0.424 rad/s여야 합니다. 이 식은 이 컴퓨터에 소스가 없어 Nav2 Jazzy 코드 기억으로 썼습니다. 아래 측정과 맞는지만 확인했습니다.
- 측정: 출발 전 collision이 난 주기의 예상 명령(odom |w| + 0.16, 상한 1.0)은 54번 모두 0.43~1.00이었습니다. 경고가 없던 주기는 0.17~1.00입니다. 명령이 0.43을 넘으려 할 때마다 막히고, 0.3~0.4로 떨어지면 투영이 없어져 다시 돕니다. 회전을 막지 못하고 0.3~0.4 rad/s로 덜컥거리게 합니다.
- 닿는 것: 왼쪽으로 돌면 뒤가 서쪽으로 갑니다. 로봇은 O_OUT보다 벽 쪽(x 0.12~0.14, y 0.65~0.67, `robots.csv`)에 섭니다. Gazebo 세계에서 그 근처 물체는 서쪽 벽(벽면 x 0.00, `sim/fleet_gz_sim/worlds/arena_v7.sdf` wall_west)뿐입니다.
  - 재계산은 231건 중 164건을 재현했습니다(지금 자세 64, 투영 100). 67건은 재현하지 못했습니다. costmap과 odom을 따로 골라 생긴 시각 차이로 봅니다(추정).
  - O_OUT 도착 자세를 기준으로 바꾼 걸린 칸의 x는 0.04~0.08입니다. 기준을 O_OUT(0.15)으로 둔 오차만큼 벽면에서 밀려 있습니다.
- 실제 여유: 덧대지 않은 뒤 모서리는 0.0845 m입니다(`nav2_params.yaml:272-276` 주석). 벽까지 0.12~0.14 m이므로 3.5~5.5 cm입니다. 덧댐 0.01과 칸 0.025가 이 여유를 먹습니다.

### 7.3 출발 뒤

- 출발 뒤(전환 10초 안)에 제자리 회전 방향 바뀜이 42번 더 있었습니다(출고마다 2~5).
- 한 예(pode1 첫 출고)에서는 carrot 방위가 14.3°(`rotate_to_heading_min_angle` 0.25)를 넘을 때마다 멈춰서 돌고 다시 달렸습니다. 나머지는 따로 보지 않았습니다.

### 7.4 확정 정도

- 녹화 데이터로 메커니즘을 다시 계산했습니다. 바꿔서 사라지는지 보는 Gazebo 시험은 하지 않았습니다.
- 고침 후보와 효과는 측정하지 않았습니다.
