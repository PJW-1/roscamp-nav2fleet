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
- 추정: 서쪽 벽 가까이에서는 다시 짠 경로의 시작 방향이 1 Hz 재계획마다 바뀝니다. 기준 각도 0.25 rad이면 제어기가 그때마다 반대로 다시 맞춥니다. 여유 1.5~3.4 cm의 충돌 검사가 더 느리게 만듭니다.
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
