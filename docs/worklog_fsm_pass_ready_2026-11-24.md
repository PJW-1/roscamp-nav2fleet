# pode_fsm: 먼저 넘기는 거리 0.12, Nav2 준비 전 LOCALIZING (2026-11-24)

브랜치 `fix/fsm-pass-ready-2026-11-24`, 출발점 origin/main aff00d7. 주석 정리가 들어간 main(c1e7de1)을 합쳤다. plan-flip(feat/plan-flip-2026-11-24)과 따로 둔다.
Gazebo와 시험만 했다(실물·deploy 없음).

## 1. 먼저 넘기는 거리 pass_m 0.15 → 0.12

원인(plan-flip 판 `s2_a1_A_rpp_v1`, `docs/report_2026-11-24_plan_flip.md` 판 실패 조사, feat/plan-flip 브랜치):

- `order_fsm._passes`는 nav_goal 조각 끝까지 ≤ `pass_m`이고 다음 follow_path 조각까지 ≤ `connect_tolerance_m`일 때
  다음 조각을 먼저 넘긴다. 둘 다 0.15였다.
- 넘긴 뒤 Nav2 goal이 아직 달려 넘겨받기(`handover_since`)로 들어갔고, 멈춘 뒤 `nav_moves._hand_over`가 경로까지
  거리를 다시 잰다. 그 사이 truth는 그대로인데 AMCL 추정 위치만 2.2 cm 뒤로 물러나 0.144 → 0.163 m가 되어
  `PATH_NOT_CONNECTED`가 났다.

바꾼 것:

| 곳 | 전 | 후 |
|---|---|---|
| `pode/pode_fsm/pode_fsm/order_fsm.py` `OrderParams.pass_m` | 0.15 | 0.12 |
| `pc/fleet_core/fleet_core/data/fleet_params.yaml` `nav_pass_m`(관제 ETA가 같은 값을 쓴다) | 0.15 | 0.12 |

이음 허용치 0.15와 3 cm 차이가 생겨, 넘긴 뒤 위치가 3 cm 안에서 흔들려도 이어 간다.

시험(`pode/pode_fsm/test/test_order.py`): 판의 좌표(Q2 (1.66, 0.15), 다음 조각 끝 (1.849, 0.26))로

- 0.144 m 지점에서는 이제 넘기지 않고, (1.545, 0.18)에서 넘긴다.
- 넘긴 뒤 (1.523, 0.18)로 2.2 cm 물러나도 이어 간다(SendPath가 q1에서 끝나고 Finish 없음).
- 옛 값 `OrderParams(pass_m=0.15)`로는 0.144 m에서 넘기고, (1.4990, 0.1800)으로 물러나면
  `PATH_NOT_CONNECTED`로 끝난다(재현).

## 2. Nav2 서버가 보이기 전에는 LOCALIZING

원인(11-23 실물 2대 시험): pode2가 Nav2 `follow_path` 서버가 뜨기 전에 IDLE을 보고했다. 관제가 보낸 goal은 서버가
없어 보내지 못했고, 로봇은 이를 `NAV_ABORTED`로 끝냈다. 관제는 세 번을 실패 시도로 세어 격리했다.

로봇(pode_fsm):

- `PodeFsm.set_nav_ready(ready)`: 처음은 `None`(따지지 않음, 순수 FSM과 가짜 로봇은 그대로). node는 시작 때 False로
  두고 tick마다 `follow_path`와 `navigate_to_pose` 액션 클라이언트의 `server_is_ready()`를 본다. 둘 다 보이면 True로
  고정하고 다시 보지 않는다.
- 고정 전에는 `state()`가 `LOCALIZING`이다. 그때 온 move·order·retreat goal은 움직이지 않고
  `NAV_UNAVAILABLE`로 끝난다.
- 보낼 때 서버가 없어 보내지 못한 FollowPath, NavigateToPose, BackUp(역방향 FollowPath 포함)도 `NAV_UNAVAILABLE`로
  끝난다. 서버가 goal을 거절한 경우는 그대로다(`NAV_ABORTED`, 물러나기는 `FAILED`).
- 새 상태를 만들지 않고 `LOCALIZING`을 쓴 이유: 관제는 이미 `LOCALIZING` 로봇을 지도에 올리지 않고 goal을 보내지 않는다
  (`WAIT_LOCALIZING`). 새 상태는 RobotState와 관제의 배치 규칙을 함께 바꿔야 한다.

계약: `interfaces/fleet_interfaces/action/MoveTo.action`에 `REASON_NAV_UNAVAILABLE="NAV_UNAVAILABLE"`를 더했다.
`docs/Pode_계약.md`(3절 6번, Result 사유, 5절 표), `docs/관제_구역교통.md`(이벤트 표), `pode/pode_fsm/README.md`를 고쳤다.

관제(fleet_core `traffic/zone/results.py`): `NAV_UNAVAILABLE`은 시도로 세지 않는다(`attempts` 그대로). 
`move_retry_wait_s`(2 s) 뒤 같은 goal을 다시 보내고 `GOAL_NAV_UNAVAILABLE`(WARN)을 낸다. 로봇이 목표 지점에 이미 서 있으면
다른 실패 사유처럼 `GOAL_ENDED_AT_TARGET`이 먼저다(기존 규칙).

알아 둘 점:

- 로봇과 PC를 함께 올려야 한다. 로봇만 올리면 시작 때 LOCALIZING으로 goal은 막지만, 고정 뒤 서버가 사라져 생긴
  `NAV_UNAVAILABLE`을 옛 관제는 실패 시도로 센다. `fleet_interfaces`도 다시 빌드한다.
- 고정 뒤 서버가 사라지면 로봇은 IDLE로 남고, 관제는 2 s마다 한도 없이 다시 보낸다(`GOAL_NAV_UNAVAILABLE`이 쌓인다).

## 3. 시험 인자: 한 로봇의 주행 서버만 늦게 켜기

- `pode/pode_nav_config/launch/nav2.launch.py` `navigation_delay_s`(기본 0): 0보다 크면 `lifecycle_manager_navigation`을
  autostart 없이 띄우고, 그만큼 뒤에 `manage_nodes` STARTUP(command 0)을 부른다. 위치(AMCL)는 먼저 뜬다.
- `sim/fleet_gz_sim/launch/gz.launch.py` `nav_late:=로봇:초`(예 `pode2:120`).
- 처음 판(9dbe416)은 Gazebo에서 지연 없이 바로 STARTUP을 불렀다. Jazzy `TimerAction`은 period를 기다리기 시작할 때
  계산하는데, include 인자는 바깥으로 새어 나가 그때는 마지막 로봇(pode3)의 값 0.0이 됐다. 로봇 launch를 읽을 때
  지연과 서비스 이름을 고정하도록 고쳤다(fc46c75, 시험 `test_the_delay_is_fixed_when_the_robot_s_launch_is_read`).

## 4. 리뷰

Codex 1회차(aff00d7..0cb8219) 확정 5건:

| 지적 | 처리 |
|---|---|
| 준비 전에도 retreat가 시작된다 | 고침: 준비 전 retreat도 `NAV_UNAVAILABLE` |
| 서버가 없어 못 보낸 물러나기가 `FAILED`가 되어 관제가 시도로 센다 | 고침: BackUp·역방향 FollowPath 모두 `NAV_UNAVAILABLE` |
| 목표 지점에 서 있으면 `GOAL_ENDED_AT_TARGET`이 먼저다 | 그대로: 모든 실패 사유에 같은 기존 규칙이고 그 자리면 목표를 이룬 것이다 |
| `autostart:=1`이면 주행이 안 뜬다 | 고침: true/1 모두 참 |
| namespace가 비면 STARTUP 서비스 이름이 `//...` | 고침 |

2회차(0cb8219..5c5844d): 결함 없음. 지연 고정(5c5844d..fc46c75): 결함 없음.

## 5. 결과

시험(pd-fsm-ready 빌드, ROS 시험은 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 도메인 95~98):

| 묶음 | 결과 |
|---|---|
| pode_fsm(ROS 없이) | 443 passed, 43 skipped |
| pode_fsm(ROS 포함) | ②까지(19:20, 부하 낮음) 477 passed, 6 skipped. 마지막 코드는 load 약 58에서 3~6건 실패(아래) |
| pode_nav_config | 121 passed |
| fleet_core | 1270 passed, 7 xfailed |
| fleet_gz_sim | 616 passed(main 병합 뒤 launch 배선 시험 101 passed) |
| tools | 289 passed(첫 실행 1건 실패는 다시 돌리니 통과, CPU 부하 때 흔들리던 시험) |

ROS 시험 실패: `test_ros_fsm::test_obstacle_ahead_backs_up_twice_then_reports_blocked`와 `test_ros_local_yield`의 둘(모두
FakeNav 후진 시간을 5 s 안에 기다리는 시험)이다. 다른 세션의 시험·Gazebo로 load average가 약 58(12코어)일 때 났다.
같은 부하에서 ②가 없는 9dbe416도 세 판 모두 3건 가운데 1건씩 실패했고, 0cb8219와 마지막 코드도 같은 비율이었다.
그래서 부하 탓으로 본다. 부하가 낮을 때 다시 돌려 확인해야 한다.

Gazebo(`gz_v7_rush`, `nav_late:=pode2:120`, 도메인 97, GUI 없음, 기록 `~/gz_runs/fsm_ready_2026-11-24/`).
고치기 전 판은 같은 빌드(pd-fsm-ready)에 pode_fsm·fleet_core 파이썬만 9dbe416 소스로 덮었다(`before_setup.bash`).
시각은 launch 시작부터 잰 벽시계다. Gazebo launch는 모든 로봇이 IDLE을 1 s 보고해야 관제와 시나리오를 띄운다(`wait_ready`).

| | 고치기 전 `late_before_r1` | 고친 뒤 `late_after_r1` |
|---|---|---|
| pode2 pode_fsm 뜸 | t+8.8 s | t+12.0 s |
| pode2 Nav2 서버 준비 | (STARTUP t+120 s 전에 멈춤) | t+134.8 s(`servers are ready`) |
| 모든 로봇 IDLE(`wait_ready`) | t+13.8 s | t+136.0 s(pode2 서버 준비 1.2 s 뒤) |
| pode2 첫 order | t+36.3 s | t+159.1 s(관제 첫 order는 t+142.8 s) |
| pode2 쪽 | `follow_path server is not available` 3번 | 없음 |
| 관제 | GOAL_RETRY 2번, GOAL_FAILED, t+41.0 s `ROBOT_ISOLATED NAV_ABORTED` | GOAL_NAV_UNAVAILABLE 0, 격리 0 |
| 판 | 격리 45 s 뒤 감시가 멈춤 | 12/12 통과, 409.7 s |

- 고치기 전 판은 11-23 실물의 격리를 그대로 재현했다.
- 고친 뒤에는 pode2가 서버 준비 전까지 IDLE을 보고하지 않아 시나리오가 그만큼 늦게 시작했고, goal 실패는 없었다.
  관제 쪽 `NAV_UNAVAILABLE` 재시도는 이 판에서 쓰이지 않았다(로봇이 먼저 막는다). 그 길은 단위 시험으로만 봤다.
- 이 브랜치(main 기준) 러시 판의 비교 기준은 돌리지 않았으므로 판 시간은 비교하지 않는다. 다른 세션의 시험·Gazebo로
  load average가 높던 때다.
- GOAL_RETRY 1건은 pode3의 NAV_ABORTED(attempt 1)로, 이 수정과 상관없다.
- 지연을 처음 40 s로 둔 판은 지연 버그(3절)로 멈췄다(`late_before_delay40_stopped`, `late_before_delay120_timer_bug_stopped`).
