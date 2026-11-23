# 2026-11-24 밤 작업: 팀 인터페이스 명세에 맞추기

브랜치 `feat/interface-align-2026-11-24`(origin/main `b2aea4d`에서 시작, 끝 `dfc5a7c`). main에는 병합하지 않았다.

## 내가 이해한 일

- 팀 문서에 맞춰 우리 `fleet_interfaces`, `fleet_core`/`fleet_manager`, `pode_fsm`이 같은 이름 · 같은 정의로 말하게 한다. 내일 팀의 central, 브리지, 팔 노드가 바로 붙을 수 있어야 한다.
  - 팀 문서: `인터페이스 명세` v0.2, `통신 규칙` IR, `fleet 작업 목록` W, `설계 검토 사항` Q.
- 1단계(필수):
  - W-01 ~ W-11: 이름 · 정의 · contract.
  - W-22: 밖으로 보낼 때 `CANCELLED`.
  - W-46: STOP을 쓰지 않음.
  - W-50: 팔 상태 구독, `/fleet/robots`에 팔 3대.
  - `pode_fsm`이 RobotState 새 칸을 채움.
  - `net.sh` 도메인 12 → 6(IR-81).
- 2단계(조사만): W-20 · W-21 · W-24 · W-40의 상대편(브리지 · 서버)이 팀 브랜치에 있는지.
- 3단계(1단계 확인 뒤):
  - `pode_fsm` 쪽 `set_lock` · `set_mode` · `charge_end` 서버.
  - 켜질 때 UNKNOWN · 잠김(IR-72).
  - request_id 기억(IR-21).
  - fleet 쪽 W-30 ~ 34, W-51 ~ 53, W-60 ~ 62는 하지 않는다.
- Q-01 ~ Q-06의 "미정"에 걸린 것은 추측하지 않는다.

### 팀원 쪽에 이미 있는 정의 (팀 `main` = `1d44cfa`, contract 0.6)

| 파일 | 팀 main에 있음 | CMakeLists에 등록 | 비고 |
|---|---|---|---|
| `msg/ArmState.msg` | 있음 | **없음** | 예전 모양(`STATE_IDLE/PICKING/PLACING/SWAPPING/ERROR`, `arm_id · state · detail · stamp`). 명세 4.5 모양과 다르다 → 명세 모양으로 새로 씀 |
| `msg/ItemArray.msg` | 있음 | 없음 | 명세에 없음. 우리 쪽에 넣지 않음 |
| `srv/SetRunMode.srv` | 있음 | 없음 | 명세가 따로 정의하지 않아 팀 파일을 글자 그대로 옮김 |
| `srv/SubmitOrder.srv` | 있음 | 없음 | 명세 3.4: 쓰지 않음(`SubmitTrip`으로 대신). 우리 쪽에 넣지 않고 contract 머리말에 적음 |
| 그 밖의 action · msg · srv | 우리 `b2aea4d`와 같음 | 있음 | 우리 쪽에만 `srv/ForceShip.srv`가 더 있었다 → 지움(명세: `force_ship`은 `TripCommand`) |

다른 팀 브랜치:

- `feat/order-hold-interfaces`: main과 같다.
- `refactor/rename-edge-mode-route`: contract 0.5로, main보다 오래됐다.
- `Phondy` · `docs/design` · `feat/ros2-guide-loss-tests` · `feat/service-bridge-unit-tests`: 인터페이스 파일이 11-06 커밋(`b0ec7ff`) 그대로이거나 지워져 있다.
- `Phondy`의 `INTERFACES.md`는 팔 자체 토픽(도메인 83)만 다룬다. fleet 인터페이스는 없다.

## 한눈에

- 1단계와 3단계를 끝내고 push했다.
- 2단계 조사 결과: 브리지 · central 서버 코드는 **팀 어느 브랜치에도 없다**. DB 설계에 `bridge_session`/`epoch` 칸만 있다.
- 정의:
  - 새 정의 19개(msg 4 · srv 15)와 `SetRunMode`를 더했다.
  - 바뀌는 정의 7개를 고쳤다: `OrderBoard` · `RobotState` · `RobotStatus` · `ArmState` · `FillBasket` · `ConfirmClear` · `Reinstate`.
  - 명세 4장 정의 23개 모두 칸 이름 · 순서가 명세와 같다(아래 비교표).
- 팔 이름은 `arm_left` · `arm_right` · `arm_ship`이다. 지도, 시나리오, 가짜 노드, 시험, 설계 문서까지 60여 개 파일을 함께 바꿨다.
- `/fleet/robots`에 팔 3대가 `kind=ARM`으로 붙는다. 가짜 팔도 `/{arm}/state`를 5 Hz로 낸다.
- 시험은 모두 통과했다(아래 "시험 결과").
- Gazebo gz_v7 한 판: 통과, 8/8 출고, 218.2 s, ROBOT_ISOLATED 0, BLOCKED 0.
- 팀 main과 다른 점은 모두 "우리가 명세대로 칸을 더한 것"이다. 팀 main 쪽에 있고 우리에게 없는 칸은 없다.
- **DDS 타입 해시 때문에 내일 연결 전에 팀 쪽도 이 브랜치의 `interfaces/fleet_interfaces`를 그대로 써야 한다.**

## 사용자가 정할 것

1. **새 칸의 자리.**
   - 명세는 바뀌는 메시지에서 새 칸의 위치를 정하지 않았다. 나는 모두 **기존 칸 뒤에** 붙였다. `ConfirmClear` · `Reinstate` 요청의 `request_id`도 맨 뒤다.
   - 팀이 파일을 따로 고쳐 쓰면 해시가 달라져 붙지 않는다. 이 브랜치의 파일을 팀 기준으로 삼을지 정해야 한다.
2. **`force_ship`의 거절 사유.** 타입은 `TripCommand`로 바꿨지만, fleet core가 내는 사유(`UNKNOWN_ORDER` · `NOT_HELD`)는 명세 사유 코드 표에 없다. 명세 코드는 `NOT_FOUND` · `ORDER_STATE`다. W-33에서 바꿀지 정한다.
3. **contract의 "central ↔ fleet" 구역.** 명세 구역 5개 외에, central과 fleet이 바로 주고받는 `report_order` · `order_board`를 따로 묶었다. 명세 구역으로 합칠지 정한다.
4. **`not_in_fake` 표시.**
   - contract에 새로 만든 표시다. 가짜 노드가 열지 않는 항목(브리지 · central 서비스, 아직 서버가 없는 fleet 관리 명령)을 가짜 모드 검사에서 뺀다.
   - 실물 모드(`--mode real`)에서는 모두 본다.
   - 팀 contract 규칙에 넣을지 정한다.
5. **`/fleet/robots` 발행 주기.** 명세 F-11은 "초당 5번"인데 지금은 1 Hz 그대로다. 올릴지 정한다.
6. **`MODE_STOP` 상수.**
   - W-46대로 STOP을 내는 곳은 없다.
   - `OrderBoard.MODE_STOP` 상수와 core의 STOP 처리는 지우지 않았다. 팀 main과 해시를 맞추려면 상수가 있어야 한다.
   - 지울지 정한다(지우려면 팀과 같이 지워야 한다).
7. **IR-72를 언제 켤지.** 아래 "알아야 할 포인트" 첫 줄 참고.

## 알아야 할 포인트

- **IR-72는 값만 지킨다.**
  - `pode_fsm`은 켜지면 `lock=true`, `mode="UNKNOWN"`, 매 시작마다 새 `boot_id`(UUID)를 낸다.
  - 하지만 **UNKNOWN이나 락이 풀린 상태에서도 이동 목표를 거절하지 않는다.** 거절하면 fleet이 `set_mode`를 부르기 전까지(W-52) 아무 일도 못 한다.
  - W-34 · W-52가 들어온 뒤 거절을 켜야 한다(Codex도 지적함).
- `set_lock`:
  - 다른 로봇 이름이면 `UNKNOWN_TARGET`.
  - 목표가 있는 동안 바꾸려 하면 잠그든 풀든 `NOT_STOPPED`.
  - 이미 그 값이면 그냥 받는다.
  - 실제로 락을 풀어도 주행은 바뀌지 않는다(위와 같은 이유).
- `set_mode`: `NORMAL` · `TEST`만 받는다. 그 밖은 `MODE_MISMATCH`.
- `charge_end`:
  - 시험 모드가 아니면 `NOT_TEST_MODE`.
  - 배터리가 35 % 미만이거나 모르면(-1) `BATTERY_LOW`.
  - 나머지는 받는다. Pode에는 충전 상태가 없어 받아도 하는 일이 없다.
- 같은 `request_id`는 다시 반영하지 않는다(IR-20 · IR-21).
  - 최근 32개를 60초 동안 기억하고, 처음 답을 그대로 돌려준다.
  - 거절도 기억한다. 빈 `request_id`는 기억하지 않는다.
- `pode_fsm` 상태 발행을 10 Hz → 5 Hz로 낮췄다(IR-34). 시험 기준은 "2초에 9 ~ 11개"로 바꿨다(조인 쪽).
- 팔 상태가 3초(`comm_timeout_s`) 넘게 없으면 `/fleet/robots`에서 그 팔은 `fault=COMM_LOST`다. 한 번도 상태를 안 낸 팔도 COMM_LOST로 나온다.
- 로컬 양보(`local_yield`)와 Gazebo 보기 노드는 `kind=ARM` 줄을 건너뛴다.
- W-22: `adapters.trip_status()`가 내부 `CANCELED`를 밖으로 `CANCELLED` + `NOTHING_TO_FILL`로 바꾼다. 아직 부르는 곳은 없다. `ReportTrip` 보내기는 W-21이다.
- `FillBasket` 결과의 `REASON_CANCELED`(L 하나)는 액션 안의 상수라 그대로 뒀다.
- 서비스 일시 정지(PAUSE)는 지금 진행 중인 담기를 멈추지 않는다. 담기 목표 안에서 멈추는 신호가 미정(Q-05, W-42)이라 `ArmJobs.on_run_mode`에 `TODO(Q-05)`만 남겼다.
- `force_ship` 서버는 응답에 `request_id`를 돌려줄 뿐 중복을 기억하지 않는다. IR-20 서버 쪽은 W-30이다.

## W별 상태

| W | 내용 | 상태 | 커밋 |
|---|---|---|---|
| W-01 | contract 0.7, 팔 이름 3개, `/arm_ship/swap_basket` | 끝 | `81f7602`, `32ac66c`(설계 문서) |
| W-02 | 구역 머리말, 사유 코드 표(7묶음 31개) | 끝 | `81f7602` |
| W-03 | `/{arm}/state` · `/system/mode` · `/bridge/close_state` | 이름은 끝. `/system/mode` · `/bridge/close_state`는 아직 구독하지 않음(W-51 · W-73) | `81f7602` |
| W-04 | `/central/submit_trip` 등 5개 | 이름만 | `81f7602` |
| W-05 | `/bridge/hello` 등 6개 | 이름만 | `81f7602` |
| W-06 | `/fleet/*` 관리 명령 | 이름만. 서버는 기존 `confirm_clear` · `reinstate` · `force_ship`뿐 | `81f7602` |
| W-07 | `/{robot}/set_lock` · `set_mode` · `force`, `/{arm}/fault_resolved`, `/{pode}/charge_end` | 이름은 끝. Pode 쪽 `set_lock` · `set_mode` · `charge_end` 서버도 끝. `force`는 안 함 | `81f7602`, `54700c1` |
| W-08 | 새 정의 파일 | 끝(명세 글자 그대로) | `81f7602` |
| W-09 | 바뀌는 정의 | 끝(새 칸은 뒤에 붙임) | `81f7602` |
| W-10 | `SubmitOrder` 안 씀 | 끝(contract 머리말) | `81f7602` |
| W-11 | CMakeLists 등록, contract_check | 끝. contract에 적힌 모든 타입이 빌드에 있는지 보는 시험을 더함 | `81f7602` |
| W-20 · W-21 · W-24 · W-40 | Hello · ReportTrip · ReportEvent · CheckStock | 조사만(아래 2단계 표) | - |
| W-22 | `CANCELLED`로 바꿔 보내기 | 바꾸는 함수와 시험은 끝. 보내는 곳은 W-21 | `81f7602` |
| W-46 | STOP 안 씀 | 확인함: STOP을 내는 코드 없음. 상수는 남김 | - |
| W-50 | 팔 상태 구독, `/fleet/robots`에 팔 3대 | 끝 | `81f7602` |
| IR-81 | `net.sh` 도메인 6 | 끝 | `7128c20`, `dfc5a7c` |
| IR-72 · IR-21 · IR-34 | Pode 켜질 때 값, request_id 기억, 5 Hz | 끝(IR-72는 값만, 거절은 안 함) | `54700c1`, `dfc5a7c` |
| W-30 ~ 34, W-51 ~ 53, W-60 ~ 62 | fleet 관리 명령 · 모드 · 호출 규칙 | 하지 않음(지시 범위 밖) | - |

## TODO(Q-nn)

코드 안에 남긴 표시는 `pc/fleet_core/fleet_core/arm/jobs.py`의 `TODO(Q-05)` 하나다. 다음 일이 미정에 걸려 있다.

| Q | 걸린 일 | 이번에 한 것 |
|---|---|---|
| Q-03 | Phody 장애 동안 새 배정 막기, `holding` · `uncertain` 조건 | `ArmState.holding`, `FillBasket.uncertain` 칸만 만듦. 이 칸을 쓰는 판단은 없음 |
| Q-04 | `resume_held` · `requeue_trip`을 받는 조건(W-36) | 이름 · 타입만 contract에 |
| Q-05 | 팔 "멈춤 · 이어서" 신호(W-35 `resume_arm`, W-42) | 이름 · 타입만 contract에. `FillBasket` 피드백 주석에 `PAUSED_DISTANCE · PAUSED_SERVICE`. `ArmJobs.on_run_mode`에 `TODO(Q-05)` |
| Q-06 | 한 팔만 락이 풀릴 때 공용 FillBasket | 팔 `set_lock`은 팀 팔 노드 몫. 손대지 않음 |
| Q-01 · Q-02 | 취소 판단, central 재시작 | central 몫. 손대지 않음 |

## 2단계 조사: 브리지 · 서버가 있는가

팀 원격의 모든 브랜치에서 `hello` · `list_open_trips` · `report_trip` · `report_event` · `check_stock` · `epoch` 등을 찾았다(`.md` 제외).

- 찾은 브랜치: `main`, `Phondy`, `docs/design`, `feat/order-hold-interfaces`, `feat/ros2-guide-loss-tests`, `feat/service-bridge-unit-tests`, `refactor/rename-edge-mode-route`.

| W | 상대편 | 누가 · 어디 | 어디까지 |
|---|---|---|---|
| W-20 Hello · ListOpenTrips | 브리지 서버 | 서버는 없음. `docs/design`(ManCheol Yang, 11-20 `5b48a34`)의 DB 설계에 `bridge_session(node, epoch)` 표, `trip.report_epoch · report_n` 칸과 제약이 있음 | DB 스키마(SQL)와 그 시험까지. ROS 서버 코드 없음 |
| W-21 ReportTrip | 브리지 서버 | 서버는 없음. 같은 DB 설계의 `trip.report_epoch`/`report_n` | DB 칸까지 |
| W-24 ReportEvent | 브리지 서버 | 서버는 없음. `docs/design` DB에 `ws_event` NOTIFY 트리거(`fn_ws_event_notify`) | DB 트리거까지 |
| W-40 CheckStock | 브리지 서버 | 서버는 없음. DB에 재고 제약(`fn_product_stock_guard`)만 | DB까지 |
| (참고) 브리지 | `feat/service-bridge-unit-tests`(ane, 11-10 `9f21e33`) | `tests/service_bridge/reference/sbt_bridge/core/`(alert_inbox · db · orders · outbox · ws_queue) | 참고 구현과 단위 시험 증거까지. 명세 이전이고 ROS 서비스(`create_service`)는 없음 |
| (참고) central | 모든 브랜치의 `pc/central` | `__init__.py`와 README뿐 | 빈 패키지 |
| (참고) 팔 | `Phondy`(happious, 11-21 `c6bd0d4`) | `jetcobot_dual_bridge`(하드웨어 브리지) | 팔 자체 토픽(도메인 83)만. `/{arm}/state` · `set_lock` 등 명세 인터페이스는 없음 |

결론: 내일 연결 시험에서 브리지 · central 쪽 서비스는 상대가 없다. 우리가 볼 수 있는 것은 두 가지 정도다.

- 팔 노드가 `/{arm}/state`를 내는지.
- 우리 `/{pode}/set_*` 서버를 팀 도구로 부를 수 있는지.

## 비교표

### 팀 main 빌드와 우리 빌드 (`ros2 interface show --no-comments`)

팀 main `1d44cfa`의 `fleet_interfaces`를 WSL의 다른 작업공간(`~/gg_build/team_iface`)에서 빌드했다. 우리 빌드(`~/gg_build/pd-iface`)와 인터페이스마다 비교했다.

| 인터페이스 | 칸 | 팀 main | 우리 | 같은가 |
|---|---|---|---|---|
| action/Dock · MoveTo · Retreat · SwapBasket | 전체 | - | - | 같음 |
| msg/Assignment · AssignmentArray · FleetEvent · Order · OrderItem · Segment · ZoneState · ZoneStateArray | 전체 | - | - | 같음 |
| srv/CheckClear · ReportOrder | 전체 | - | - | 같음 |
| action/FillBasket | 결과의 `REASON_PICK_FAILED` · `REASON_RECOGNITION_ERROR` · `REASON_OUT_OF_STOCK` · `REASON_ARM_FAULT` · `OrderItem[] placed_confirmed` · `OrderItem[] uncertain` | 없음 | 있음(뒤에) | 다름(명세대로 더함) |
| msg/OrderBoard | `bool assign_hold` · `string close_id` | 없음 | 있음(뒤에) | 다름(명세대로 더함) |
| msg/RobotState | `boot_id` · `lock` · `last_request_id` · `mode` · `forced[]` · `detail` | 없음 | 있음(뒤에) | 다름(명세대로 더함) |
| msg/RobotStatus(와 이를 담는 RobotStatusArray) | `KIND_POD` · `KIND_ARM` · `kind` · `lock` · `mode` · `forced[]` · `boot_id` | 없음 | 있음(뒤에) | 다름(명세대로 더함) |
| srv/ConfirmClear · Reinstate | 요청의 `request_id`, 응답의 `request_id` · `detail` | 없음 | 있음(뒤에) | 다름(명세대로 더함) |
| msg/ArmState · ItemArray, srv/SetRunMode · SubmitOrder | - | 파일만 있고 빌드 안 됨 | ArmState(명세 모양) · SetRunMode(팀 글자 그대로)만 빌드 | 팀 빌드에는 없음 |
| 새 정의 19개 | - | 없음 | 있음 | 우리에게만 있음 |

팀 main에 있고 우리에게 없는 칸: **없음**.

### 명세 4장 정의와 우리 파일

명세의 코드 블록 23개에서 칸 줄을 뽑아 우리 파일과 비교했다. 주석은 뺐고, `(공통 응답)`은 `accepted · reason · request_id · detail`로 펼쳤다.

| 정의 | 종류 | 빠진 칸 | 순서 같음 | 글자 그대로 |
|---|---|---|---|---|
| msg/OpenTrip · SystemMode · CloseState | 신규 | 없음 | 예 | 예 |
| srv/SubmitTrip · CancelOrder · SetAssignHold · GetCloseState | 신규 | 없음 | 예 | 예 |
| srv/Hello · ListOpenTrips · ReportTrip · ReportCommand · ReportEvent · CheckStock | 신규 | 없음 | 예 | 예 |
| srv/TripCommand · RobotCommand · SetLock · ForceInput · SetMode | 신규 | 없음 | 예 | 예 |
| msg/ArmState | 바뀜(명세 전체 목록) | 없음 | 예 | - |
| msg/OrderBoard · RobotState · RobotStatus, action/FillBasket | 바뀜(더하는 칸) | 없음 | 예 | - |

## 시험 결과

환경: WSL, `ROS_DOMAIN_ID=48`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 빌드 `~/gg_build/pd-iface`. fleet_manager 시나리오 시험은 스스로 50번대 도메인을 고른다.

| 패키지 | 결과 | 시점 |
|---|---|---|
| fleet_core(contract · contract_check 포함) | 1296 passed, 7 xfailed | `32ac66c` 전 전체 실행 |
| pode_fsm | 481 passed, 6 skipped | `dfc5a7c` 뒤 다시 실행 |
| tools | 279 passed | `dfc5a7c` 뒤 다시 실행 |
| fleet_manager(시나리오 통합 포함) | 146 passed, 2 warnings (814.72 s) | `32ac66c` 전 전체 실행 |
| fake_robots(`test_ros_check_interfaces` 포함) | 1156 passed, 1 skipped, 30 xfailed (921.41 s) | `32ac66c` 전 전체 실행 |
| fleet_gz_sim | 610 passed | `32ac66c` 전 전체 실행 |
| fleet_interfaces 빌드 | 성공 | - |

- 첫 전체 실행에서 실패가 넷 있었다. `32ac66c` 전 실행이다.
  - pode_fsm 2개: 내가 새로 쓴 시험이 잘못됐다.
    - 하나는 움직이는 중에 락을 잠그면 받아 줄 것으로 기대했다. 코드는 W-34대로 `NOT_STOPPED`로 거절한다.
    - 다른 하나는 첫 상태가 오기 전에 상태를 읽었다.
  - tools 2개: `test_real_rviz`가 아직 `NET=12`를 기대했다. 도메인 변경에서 빠뜨린 사용처다.
- `dfc5a7c`에서 셋 모두 고쳤고, 다시 돌려 위 결과가 나왔다.
- `32ac66c` · `dfc5a7c`는 fleet_core의 주석 한 줄, 시험과 문서만 바꿨다. 그래서 fleet_core · fleet_manager · fake_robots · fleet_gz_sim은 다시 돌리지 않았다.
- 불안정한 시험을 따로 다시 돌린 일은 없다.

## Gazebo 결과

- 조건: gz_v7, 화면 없음(`rviz:=none gui:=false nav_max_speed:=0.12`), 도메인 61, 이 브랜치 빌드를 install로 씀.
- 결과: `pass True`, 8/8 출고, 218.2 s, ROBOT_ISOLATED 0, BLOCKED 0, LOCAL_YIELD 2.
  - 이전 기준(218 ~ 221 s, 8/8, 0/0)과 같다.
  - launch 종료 코드 1은 이전 판들과 같은 정리 단계 종료다.
- 결과 폴더: WSL `~/gz_runs/iface_1124_0022/v7`.

## 바를 느슨하게 한 것

- **contract `not_in_fake`.** 다음 항목들이 가짜 모드 인터페이스 검사(`check_interfaces.py` 기본, 시나리오 통합 시험)에서 빠진다.
  - 토픽: `/system/mode`, `/bridge/close_state`.
  - 로봇 관리 서비스 5종: `/{robot}/set_lock` · `set_mode` · `force`, `/{arm}/fault_resolved`, `/{pode}/charge_end`.
  - central 서비스 5종, 브리지 보고 · 문의 서비스 6종.
  - `force_ship`을 뺀 새 fleet 관리 서비스 10종.
  - 이유: 가짜 노드나 아직 없는 서버가 열어야 하는 이름이라, 넣으면 검사가 늘 실패한다.
  - 실물 모드(`--mode real`)에서는 모두 검사한다.
  - `force_ship` · `confirm_clear` · `reinstate`는 그대로 검사한다.
- **`tools/test/test_real_pode.py`의 실물 FSM 도크 launch 시험.** 건너뛰는 도메인에 6을 더했다.
  - 이유: 6은 이제 로봇 네트워크 도메인이고, 통신 규칙상 시험은 6에서 돌리지 않는다. 예전에 12를 뺐던 것과 같은 이유다.
- 조인 쪽(언급만): `pode_fsm` 상태 주기 시험을 "2초에 9 ~ 11개"(5 Hz)로 바꿨다.

## 합친 main과의 충돌

- 오늘 밤 pode-drive main에 `docs/report-2026-11-23`와 `feat/real-bag-record-2026-11-23`를 합쳐 push했다(main `c62c03d`).
- `feat/real-side-by-side-2026-11-23`는 bag-record와 `tools/real_pode/README.md`에서 부딪혀 빠졌다.
- bag-record 검토의 후속 수정은 `fix/bag-record-followups-2026-11-24`(`26abbbe`, main 미병합)에 있다.
- `git merge-tree`로 확인한 결과(이 브랜치 `dfc5a7c` 기준). 이 브랜치는 rebase하지 않았다:
  - **이 브랜치와 main `c62c03d`: 충돌 없음.** `tools/real_pode/README.md`를 양쪽이 고쳤지만 자동으로 합쳐진다.
  - **이 브랜치와 `feat/real-side-by-side-2026-11-23`: `tools/real_pode/README.md`에서 충돌.** `test_real_pode.py`는 자동으로 합쳐진다.
  - **이 브랜치와 `fix/bag-record-followups-2026-11-24`: `tools/real_pode/README.md`에서 충돌**(실행 순서 줄. 이 브랜치는 같은 절의 도메인 12 → 6 문구를 고쳤다).

## Codex 검토

`b2aea4d..7128c20`을 한 번 검토했다. 지적 9건은 모두 [확정]으로 왔고, 하나씩 확인했다.

- 고침:
  - `docs/IFP_공통설계.md`에 남은 `arm1/2/3`, `/arm3/swap_basket` → `32ac66c`.
- 표시만 남김:
  - PAUSE가 진행 중인 담기를 멈추지 않음. 신호 방식이 Q-05라 `TODO(Q-05)`만 달았다.
- 일부러 둠(위 "알아야 할 포인트"와 "사용자가 정할 것"에 적음):
  - IR-72 거절을 하지 않음.
  - `charge_end`가 아무 동작도 하지 않음.
  - `trip_status`를 부르는 곳이 없음(W-21).
  - `MODE_STOP` 상수가 남음.
  - `force_ship`이 중복 요청을 기억하지 않음(W-30).
- 느슨한 바로 적음:
  - `not_in_fake`.
  - 도메인 6 건너뛰기.
- 고친 커밋(`32ac66c..dfc5a7c`)은 따로 다시 검토하지 않았다.

## 내일 연결 시험 체크리스트

### 누가 무엇을 여는가

| 이름 | 종류 | 여는 쪽(서버 · 발행) | 받는 쪽 | 상태 |
|---|---|---|---|---|
| `/pode{1,2,3}/state`(RobotState, STATE) | 토픽 | 우리 `pode_fsm` | 우리 `fleet_manager` | 있음, 5 Hz |
| `/arm_left/state` · `/arm_right/state` · `/arm_ship/state`(ArmState, STATE) | 토픽 | **팀 팔 노드** | 우리 `fleet_manager` | 우리 구독은 있음, 팀 발행은 없음 |
| `/fleet/robots`(RobotStatusArray, LATCHED) | 토픽 | 우리 `fleet_manager` | 팀 브리지(화면) | 있음, 1 Hz |
| `/fleet/assignments` · `/fleet/zones` | 토픽 | 우리 `fleet_manager` | 팀 브리지 | 있음 |
| `/central/orders`(OrderBoard, LATCHED) | 토픽 | **팀 central** | 우리 `fleet_manager` | 팀 쪽 없음(시험은 가짜 central로) |
| `/central/events`(FleetEvent) | 토픽 | 우리 `fleet_manager` | 팀 central · 브리지 | 있음 |
| `/fleet/heartbeat` | 토픽 | 우리 `fleet_manager` | 우리 `pode_fsm` | 있음 |
| `/system/mode` · `/bridge/close_state` | 토픽 | 팀 브리지 | 우리(아직 구독 안 함) | 둘 다 없음 |
| `/pode{1,2,3}/move_to` · `dock` · `retreat` | 액션 | 우리 `pode_fsm` | 우리 `fleet_manager` | 있음 |
| `/workcell/fill_basket`(FillBasket) | 액션 | **팀 팔 노드(Phody)** | 우리 `fleet_manager` | 팀 쪽 없음 |
| `/arm_ship/swap_basket`(SwapBasket) | 액션 | **팀 팔 노드(Phony)** | 우리 `fleet_manager` | 팀 쪽 없음 |
| `/pode{1,2,3}/set_lock` · `set_mode` · `charge_end` | 서비스 | 우리 `pode_fsm` | fleet(W-31 전까지는 손으로 부름) | 있음 |
| `/pode{1,2,3}/force` | 서비스 | 우리 `pode_fsm` | fleet | 없음 |
| `/arm_*/set_lock` · `set_mode` · `force`, `/arm_{left,right}/fault_resolved` | 서비스 | 팀 팔 노드 | 우리 fleet | 양쪽 다 없음 |
| `/fleet/confirm_clear` · `reinstate` · `force_ship` | 서비스 | 우리 `fleet_manager` | 팀 브리지 | 있음(`force_ship`은 `TripCommand`) |
| `/fleet/set_lock` 등 나머지 관리 명령 | 서비스 | 우리 `fleet_manager` | 팀 브리지 | 없음(W-30) |
| `/central/report_order` | 서비스 | 팀 central | 우리 `fleet_manager` | 팀 쪽 없음 |
| `/central/submit_trip` · `cancel_order` · `set_run_mode` · `set_assign_hold`, `/bridge/*` | 서비스 | 팀 central · 브리지 | 브리지 · 우리 | 양쪽 다 없음 |

### 먼저 맞출 것

1. 팀과 우리가 **같은 `interfaces/fleet_interfaces`** 를 빌드했는지 확인한다.
   - 양쪽에서 `ros2 interface show fleet_interfaces/msg/RobotState`와 `.../msg/ArmState` 출력을 견준다.
   - 하나라도 다르면 토픽이 이어지지 않는다(타입 해시).
2. 도메인과 디스커버리:
   - 관제 PC: `bash tools/real_pode/discovery_server.sh`를 먼저 켠다. 포트 11811로, 모든 망에서 듣는다.
   - 관제 PC 터미널: `source tools/real_pode/pc_env.sh`.
     - `net.sh`가 정하는 값: `ROS_DOMAIN_ID=6`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS_DISCOVERY_SERVER=192.168.0.252:11811`.
     - ros2 명령 터미널은 `SUPER=1 source tools/real_pode/pc_env.sh`.
   - 로봇: `source tools/real_pode/env.sh`(같은 `net.sh`를 읽음).
     - **로봇에 새 `net.sh`를 올려야 6이 된다.** 이번에는 로봇에 배포하지 않았다.
   - 팀 노드도 같은 디스커버리 서버를 써야 한다. 팀이 서버 없이 멀티캐스트로 띄우면, 우리 쪽을 `DISCOVERY_SERVER=off source ...`로 맞춘다.
3. 이 PC의 시험 도메인(48, 50 ~ 59, 61, 62)은 6과 겹치지 않는다. 그래도 연결 시험 동안 이 PC에서 Gazebo · 시나리오 시험을 돌리지 않는다.

### 확인 명령 (`SUPER=1 source tools/real_pode/pc_env.sh` 뒤)

```bash
echo $ROS_DOMAIN_ID $ROS_DISCOVERY_SERVER          # 6 192.168.0.252:11811
ros2 node list                                       # 팀 노드와 우리 노드가 함께 보이는지
ros2 topic info -v /arm_left/state                   # 발행 1(팀 팔), 구독 1(fleet_manager), 타입 같은지
ros2 topic echo --once /arm_left/state
ros2 topic echo --once /fleet/robots                 # pode 3줄 뒤에 kind=ARM 3줄
ros2 topic hz /pode1/state                           # 약 5 Hz
ros2 action info /workcell/fill_basket               # 서버 1(팀), 클라이언트 1(fleet_manager)
ros2 action info /arm_ship/swap_basket
ros2 service call /pode1/set_mode fleet_interfaces/srv/SetMode "{request_id: t-1, mode: TEST}"
ros2 service call /pode1/set_lock fleet_interfaces/srv/SetLock "{request_id: t-2, robot_id: pode1, locked: true}"
ros2 topic echo --once /pode1/state                  # lock, mode, last_request_id 확인
python3 scripts/check_interfaces.py --mode real      # contract 전체 대조 (실물 모드)
```

`check_interfaces.py --mode real`은 `not_in_fake` 항목까지 모두 본다. 그래서 팀 쪽에 아직 없는 이름(브리지 · central 서비스 등)은 빠졌다고 나온다. 위 표의 "없음" 줄과 맞는지만 본다.
