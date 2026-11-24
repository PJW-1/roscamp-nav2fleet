# 팀 레포 개인 브랜치 기록 (2026-11-24)

할 일 문서 9판 ②를 했다. 팀 레포 `addinedu-roscamp-16th/roscamp-repo-2`의 개인 브랜치 `feat/pinky-arm-fleet-0.7`에 커밋 a~f를 넣고 push했다. **PR은 열지 않았다**(사용자 지시 "pr도 올리지마, 개인 브랜치까지만"). `origin/dev` merge도 아직 하지 않았다(12판 "PR 절차"의 첫 단계라 PR 때 한다). 12판 ①의 PR 남은 것 1·2는 후속 커밋 두 개로 했다.

## 출발점

- 9판은 "`feat/contract-0.7`의 `fbe6051` 이후"를 출발점으로 정했다.
- 16시 무렵 PR #6(contract-0.7 → `prepare_dev`)이 merge되어 `prepare_dev` = `317daaa`가 되었고, 원격 `feat/contract-0.7`은 지워졌다.
- `317daaa`의 트리는 `fbe6051`과 같다. 그래서 `317daaa`(`prepare_dev`)에서 출발했다.
- **PR 대상은 팀 개발 통합 브랜치 `dev`다**(`a356b57`, 팀 `docs/브랜치_운영_안내.md` v1.0). `dev`는 `317daaa`에 문서 커밋 하나를 더한 판이다. PR을 열기 전에 `origin/dev`를 개인 브랜치에 merge하고 push한다(rebase 안 함). 그 커밋이 바꾼 파일(AGENTS.md, CLAUDE.md, `docs/` 아래 넷)은 우리 파일과 겹치지 않는다.

## 커밋

| 커밋 | 9판 | 내용 |
|---|---|---|
| `f6c4fea` | a | 교통 층(`traffic/zone` 6개 파일)과 `fleet_map`: Nav2 goal ETA를 남은 거리로, `nav_eta_speed_ratio` 등 파라미터, v7 지도. `fleet_manager`는 `distance_remaining`을 코어에 넘기고 값 변화 필터를 뺐다 |
| `716c02d` | b | `pode_fsm` 동작 없음, `start.sh`·`stop.sh`의 `NO_MOTION`, 속도 단일 출처 `nav_speed.yaml`(0.12) |
| `b767f09` | c | 가짜 Pode·팔 노드와 ROS 시험, `fake_arms:=none` 고침(`70a5438`) |
| `a868a77` | d | `/fleet/robots` 5 Hz, IR-41(지연 ±20%, 요청 상한, 3 s 넘은 요청 버리기). 네트워크 파일 없음 |
| `8b1eb89` | e | `ReportTripCmd` 정의와 시험(`ReportTrip.srv` 철자 대조 포함) |
| `612afd0` | f | `/system/mode` 구독, 바뀔 때 로그만. `TODO(Q-nn)` 두 곳(IR-73, IR-30) |
| `5441653` | 후속 | `make_params.py`의 `MAX_SPEED` 주석과 만든 설정에 남는 메모를 "저장소 값"에서 "상한"으로(저장소 값은 이제 0.12) |
| `9080e7c` | 후속 | Codex 리뷰 결함 수정(아래 "리뷰") |

커밋 a~f 합계 91개 파일, +7,483 −221. 후속 두 커밋 뒤(head `9080e7c`)에도 91개, +7,430 −219다(`make_params.py`가 더해지고 `test_input_reject.py`는 팀 판과 같아져 빠짐). 팀 파일을 지운 것은 0개다. 문서를 빼고 `arm[123]`은 0줄이다(팀 `contract.yaml` 변경 이력 주석 한 줄 빼고). 커밋 메시지에 AI 표시가 없다.

## 옮긴 방법

- 원본은 no-motion + arm-rename을 합친 로컬 브랜치 `combo-0.6-2026-11-24`다(9판대로 interface-align은 원본으로 쓰지 않음).
- 기준점 `936ae42`(팀 PR #1이 옮긴 시점)로 파일마다 3-way merge(`git merge-file`)를 해서, 팀 파일 위에 우리 차이만 얹었다. 팀이 바꾼 팔 이름과 contract 0.7은 그대로 남는다.
- 충돌 17곳은 이렇게 풀었다.
  - 팀 쪽: 지도 yaml 머리 주석, `pc.launch.py`·`fakes.launch.py`(monitor 없음), `FORCE_NOT_IMPLEMENTED`, 조건부 `force_ship` 등록, 주문 시험 덩어리.
  - 우리 쪽: `fleet_map_v7.yaml` 주석 이력.
- `ship_partial`은 팀 params에 남겼다. 팀 pseudocode가 "지울지는 주문 담당이 정한다"고 적어 두었기 때문이다.
- d·e는 pode-drive `feat/fleet-net-2026-11-24`(`52e5b82`)와 `feat/report-trip-cmd-2026-11-24`(`a06c84d`)를 같은 방식으로 얹었다.

## 넣지 않은 것과 까닭

| 뺀 것 | 까닭 |
|---|---|
| `orders/` 구현, 주문 처리 시험 | 통합 관제 몫(9판 "뺌") |
| 가짜 세계(`world/world.py`, `stations.py` 등), 시나리오 10개, judge·monitor, 그 시험들 | 주문 코어가 있어야 돈다. 팀 PR #1도 같은 까닭으로 뺐다 |
| 가짜 central의 강제 출고·보류 처리 | 팀 레포에 없는 `ForceShip`을 import한다. 넣으면 가짜 central이 뜨자마자 멈춘다 |
| `test_ros_check_interfaces.py`, `arm_fault` 시나리오 시험 | `ForceShip`·주문 시나리오가 필요하다 |
| `test_zone_distance_left.py`의 기록·재생 시험 2개 | 주문 코어가 로봇을 불러야 돈다 |
| 네트워크 파일(`net.sh`, `discovery_server.sh`, `pc_env.sh`, `env.sh`·`rviz_pc.sh` 도메인)과 `tools/real_pode`의 나머지 | 9판: ③-5 합의 뒤 따로 PR |
| 시나리오 설정 `force_ship_after_s`, 기대값 `orders_canceled`·`robots_isolated`·`hold_reasons` (`9080e7c`에서 뺌) | 강제 출고를 부르는 가짜 central 부분과 기대값을 판정하는 judge가 팀 레포에 없다. 남겨 두면 시나리오에 적어도 아무 일이 일어나지 않는다 |
| Nav2 MPPI 블록과 그 시험, `nav2_mppi_controller` 의존 | 시뮬 전용이고 팀 레포에 없는 `fleet_gz_sim`·`learning/`·`tools/nav_bench`를 가리킨다 |

## 팀 파일을 고친 곳

- `pode_nav_config/test/test_nav_config.py`: `desired_linear_vel == 0.25`를 `nav_speed.yaml`(0.12) 비교로 바꾸고, 세 사본이 같은지 보는 시험을 더했다. 실물 시작(`make_params.py --speed 0.12`)은 이미 0.12였다.
- `pc/fleet_manager/test/test_order_report_adapters.py`: 가짜 노드 객체에 `_on_system_mode`를 더했다(한 줄).
- 둘 다 PJW-1이 11-19에 쓴 파일이다.

## 시험

private `_build`/`_install`, `nice`, `ROS_DOMAIN_ID=74`, `LOCALHOST`로 돌렸다. 마지막 커밋 뒤 전체:

| 패키지 | 결과 |
|---|---|
| `fleet_core` | 1182 통과, 7 xfail(팀에 원래 있던 예상 실패) |
| `fleet_manager` | 60 통과 |
| `pode_fsm` | 490 통과, 6 건너뜀 |
| `pode_avoidance` · `pode_nav_config` · `pode_peer_obstacles` · `pode_scan_filter` | 18 · 60 · 9 · 16 통과 |
| `sim/fake_robots` | 410 통과 |

- 빌드: 11개 패키지 모두 성공.
- 포팅한 패키지(`fake_robots`, `fleet_manager`, `pode_fsm`, `fleet_core`)의 모든 모듈이 팀 인터페이스 위에서 import된다.

- 후속 커밋 뒤 다시 돌린 결과: `fleet_core` 1182 통과·7 xfail, `fleet_manager` 60, `pode_fsm` 490·6 건너뜀, 나머지 `pode_*` 4개 같음, `fake_robots` 407 통과·3 실패. 실패 3개는 위에서 뺀 옵션을 검사하던 옮겨 온 시험이라 지웠고, 지운 뒤 `test_scenario.py`·`test_fault_events.py` 137 통과. `make_params.py --speed 0.12`로 설정을 만들어 메모가 "상한 0.25"로 남는 것을 확인했다.
- `pode_fsm`의 6개 건너뜀은 `test_ros_matches_fake_pode.py`(lane 지도용으로 쓰였다는 사유)로, pode-drive에서도 원래 건너뛰던 것이다.

## 리뷰

Codex 리뷰(`317daaa..612afd0`)의 지적 세 건을 모두 확인하고 `9080e7c`에서 고쳤다.

- `force_ship_after_s`와 주문 기대값 세 개를 읽기만 하고 쓰는 곳이 없음: 팀 판으로 되돌렸다(`ORDER_RESULTS`도 팀 판). 그 옵션만 검사하던, 옮겨 온 시나리오 시험 3개도 지웠다(팀 판에는 없던 시험).
- `test_input_reject.py`에 같은 이름의 시험이 둘이라 옮긴 쪽이 수집되지 않음: 옮긴 쪽을 지웠다. 그 시험은 팀 코어가 내는 `NOT_IMPLEMENTED`가 아니라 `UNKNOWN_ORDER`를 기대했다.
- `pode_fsm` README가 팀 레포에 없는 `sim/fleet_gz_sim`을 가리킴: 그 줄을 지웠다.

기록 문서 리뷰(`a944a1b..71d084d`)의 지적 세 건(초안 9번 IR-61, 8번 IR-73, 10번 IR-62)도 모두 확인하고 아래 초안을 고쳤다. 그중 IR-73 설명 오류는 코드 주석에도 있어서 같은 커밋에서 고쳤다(IR-73은 "TEST 모드 주문 거부"가 아니라 "모드가 다르면 연결 거부·알림, 로봇 `set_mode`, 그래도 다르면 배정 제외"다). `612afd0`의 커밋 메시지도 같은 오류가 있으나 force push를 하지 않으므로 `9080e7c` 메시지에 바로잡았다고 적었다.

## 남은 것

- **PR(④):** 사용자가 말하면 연다. 대상은 **`dev`**. 절차는 할 일 문서 12판 "PR 절차"를 따른다: 열기 전에 `origin/dev`를 merge·push, base `dev` ← compare `feat/pinky-arm-fleet-0.7`, 제목은 Conventional Commits, 본문에 Jira 번호·커밋·시험 방법, AI 표시 없음, 병합은 Create a merge commit.
- **도메인(①-3):** 보류. ③-5 합의 뒤.
- **IR-41 정할 문제:** 답 없는 보고가 상한 4자리를 다 차지하면 뒤 보고가 못 나간다(같은 주문 보고 순서를 지키려고 둠). `docs/report_2026-11-24_fleet_net.md` 참고.

## ③ 팀에 보낼 초안 (보내지 않음)

### 5. 네트워크 (연동 시험 때)

> 연동 시험 네트워크를 정하고 싶습니다. `docs/개발_규칙.md`는 "도메인 6은 로봇 네트워크, 시험은 반드시 다른 도메인 그리고 `LOCALHOST`"입니다. 그런데 여러 PC 연동 시험은 `LOCALHOST`를 쓸 수 없습니다(주문 흐름 문서 9장). 이 경우만 예외로 두고 싶습니다.
> 1. 연동 시험 도메인 번호(예: 20번대 하나)와, 실물 망(6)과 동시에 돌리지 않는다는 규칙.
> 2. 도메인·discovery 범위·discovery server 설정을 `config/`나 bringup 한 곳에 두고 Pode·팔·central이 모두 읽게 하기. 지금은 `tools/real_pode/net.sh`에만 있습니다.
> 3. discovery server를 모두 같이 쓸지. 한쪽만 켜면 서로 못 찾습니다. 팔 쪽은 지금 도메인 83입니다. 같은 discovery server 아래에서는 도메인으로 갈라지지 않는다는 점도 적어 두겠습니다.
> 정해지면 공통 설정 파일로 작은 PR을 따로 내겠습니다.

### 6. 팔 팀 합의 (실제 팔 노드에 붙이기 전)

> 관제(fleet)가 팔 액션에 기대는 것을 맞추고 싶습니다(주문 흐름 문서 8장).
> - `/workcell/fill_basket`: result `success`·`placed`·`missing`·`reason`(취소면 `CANCELED`). feedback `arm_id`·`phase`·`placed_total`은 어떤 순서로 몇 번 오는지, `phase` 값 목록.
> - `/arm_ship/swap_basket`: result `took_full`·`put_empty_done`. 둘 중 하나만 된 경우가 실제로 생기는지.
> - 결품 흉내: 시험에서 `missing`을 일부러 만드는 방법(재고 표를 바꾸는지, 인자로 주는지).

### 7. 동작 없음 결정 공유 (주문 흐름 문서 9장)

> 이번 주 로봇은 "동작 없음"으로 연결만 합니다. 세 가지를 이렇게 정했습니다.
> 1. 모르는 `dock_id`가 오면 경고만 남기고 성공으로 답합니다.
> 2. `current_goal`은 실주행처럼 구간 끝까지 간 것으로 답합니다.
> 3. 시작 위치는 실행 인자로 받고, 안 주면 로봇별 주차 자리(v7 P-1~P-3)를 씁니다. `NO_MOTION=1 start.sh pode2`처럼 띄웁니다.

### 8. 설계 검토 사항 (팀 `docs/설계/설계_검토_사항.md`에 더할 안)

Q-18~20은 양만철 님의 팀 브랜치 `docs/discovery-server`(설계 검토 사항 v1.1: Q-18 통합 시험 서버, Q-19 팔 스택 이름 공간, Q-20 central PC 고정 주소)가 썼으므로 Q-21부터 쓴다(할 일 13판). 등록할 때 그사이 번호가 또 쓰였으면 다음 번호로 옮긴다.

```markdown
| Q-21 | fleet_manager 단일 스레드 실행 (IR-30 예외) | 제안 | IR-30, `fleet_manager_node.py` |
| Q-22 | 이번 주 `/system/mode`는 로그만 (IR-73) | 제안 | IR-73, 주문 흐름 1차 6.4 |
| Q-23 | IR-41 보고 상한은 먼저 들어온 보고부터 (순서 유지) | 제안 | IR-41 |

### Q-21 fleet_manager 단일 스레드 실행 (IR-30 예외)
- **상태:** 제안
- **문제:** IR-30은 비동기 호출과 MultiThreadedExecutor를 요구한다. fleet_manager는 `rclpy.spin` 단일 스레드로 돈다.
- **안:** 예외로 둔다. `call_async`만 쓰고 콜백 안에서 기다리지 않아 교착이 없다. 코어가 결정적이어야 기록 재생 시험(`tools/replay.py`)이 맞는다.
- **결정되면 고칠 곳:** IR-30에 예외 한 줄, 코드의 `TODO(Q-21)`.

### Q-22 이번 주 `/system/mode`는 로그만 (IR-73)
- **상태:** 제안
- **문제:** IR-73은 fleet·central이 켜질 때 `/system/mode`를 읽고 자기 모드와 다르면 연결을 거부하고 알리라고 한다. fleet은 로봇이 다시 연결되거나 보고한 모드가 다를 때 `set_mode`를 부르고, 그래도 다르면 배정에서 뺀다. 주문 흐름 1차 6.4 표는 이번 주를 "로그만"으로 줄였고(F-13 "다르면 거부는 다음 주"), fleet은 모드가 바뀔 때 로그만 남긴다.
- **안:** 다음 주(스프린트 5)에 모드 불일치 처리를 넣는다. fleet이 비교할 자기 모드를 어디서 읽을지(실행 인자·설정 파일, 명세 1장)도 그때 정한다.
- **결정되면 고칠 곳:** `fleet_manager_node.py`의 `TODO(Q-22)`.

### Q-23 IR-41 보고 상한을 채우는 순서
- **상태:** 제안
- **문제:** IR-41은 노드별 동시 요청 수에 상한을 두라고만 하고, 상한이 찼을 때 어느 보고를 먼저 보낼지는 정하지 않았다. fleet은 상한 4(`report_max_in_flight`)를 먼저 들어온 보고부터 채운다. 그래서 central이 답하지 못하는 보고가 네 건 쌓이면, 뒤 보고는 그동안 나가지 못한다.
- **안:** 순서 유지로 둔다. 새 보고를 먼저 보내면 같은 주문의 HELD와 SHIPPED 순서가 뒤바뀌고, `ReportTrip`의 `report_n` 순서 판정(`LATE_REPORT`, IR-24)에서 정상 보고가 버려질 수 있다. central이 네 건을 받지 못하는 상황은 이미 장애이므로, 막히는 쪽이 원인을 찾기 쉽다.
- **결정되면 고칠 곳:** IR-41에 한 줄. 코드는 그대로다.
```

### 9. IR-61 (로봇 거절을 응답으로)

> IR-61(로봇의 거절은 서비스·액션 응답으로, 상태 토픽으로 알리지 않음)을 확인했습니다. `pode_fsm`은 `move_to`·`dock`·`retreat`를 액션 서버로 받습니다. goal은 모두 받아들이고, 할 수 없는 명령은 액션 결과(`success=false`, `reason`)로 돌려줍니다. 상태 토픽(`RobotState`)에는 거절을 알리는 칸이 없습니다. goal 단계에서 거절하지 않는 점만 다르니, 이것으로 IR-61을 만족하는지 확인 부탁드립니다.

(9판·12판 메모의 "seq 토픽 명령이라 `rejected` 이벤트"는 틀렸다. `seq`와 `rejected`는 `pode_fsm` 안에서 Nav2 `follow_path` 결과를 다루는 이름이고, 관제에는 액션 결과로 나간다.)

### 10. IR-62 (로봇 무응답 → `ROBOT_TIMEOUT`, 30초 안에 끝 상태)

> 주문 흐름 1차 6.3 표는 IR-62(로봇 응답이 없으면 실패가 아니라 "결과 모름" `ROBOT_TIMEOUT`, 30초 안에 끝 상태를 한 번만 정함)를 이번 주 관제 몫으로 둡니다. 지금 fleet은 상태 보고가 끊긴 로봇을 통신 끊김(`COMM_LOST`)으로, 목표가 열린 채 제자리인 로봇을 멈춤(`stall_s`)으로 처리합니다. `ROBOT_TIMEOUT` 상태와 30초 확정은 아직 없습니다. IR-62 원문은 관리 명령(접수 응답 + `ReportCommand`) 규칙이라, 이번 주 이동·도킹 액션에는 어떻게 적용할지 여쭙니다. 예를 들어 goal 결과가 오지 않으면 `/fleet/robots` 관측으로 끝 상태를 정하고, 30초 안에 못 정하면 TIMEOUT으로 확정해 알리는 방식입니다.
