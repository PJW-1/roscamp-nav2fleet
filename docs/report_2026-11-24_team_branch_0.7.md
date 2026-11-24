# 팀 레포 개인 브랜치 기록 (2026-11-24)

할 일 문서 9판 ②를 했다. 팀 레포 `addinedu-roscamp-16th/roscamp-repo-2`의 개인 브랜치 `feat/pinky-arm-fleet-0.7`에 커밋 a~f를 넣고 push했다. **PR은 열지 않았다**(사용자 지시 "pr도 올리지마, 개인 브랜치까지만"). `prepare_dev` merge도 하지 않았다.

## 출발점

- 9판은 "`feat/contract-0.7`의 `fbe6051` 이후"를 출발점으로 정했다.
- 16시 무렵 PR #6(contract-0.7 → `prepare_dev`)이 merge되어 `prepare_dev` = `317daaa`가 되었고, 원격 `feat/contract-0.7`은 지워졌다.
- `317daaa`의 트리는 `fbe6051`과 같다. 그래서 `317daaa`(`prepare_dev`)에서 출발했다. 이 브랜치로 PR을 열면 9판 ③의 `prepare_dev` merge는 필요 없다.

## 커밋

| 커밋 | 9판 | 내용 |
|---|---|---|
| `f6c4fea` | a | 교통 층(`traffic/zone` 6개 파일)과 `fleet_map`: Nav2 goal ETA를 남은 거리로, `nav_eta_speed_ratio` 등 파라미터, v7 지도. `fleet_manager`는 `distance_remaining`을 코어에 넘기고 값 변화 필터를 뺐다 |
| `716c02d` | b | `pode_fsm` 동작 없음, `start.sh`·`stop.sh`의 `NO_MOTION`, 속도 단일 출처 `nav_speed.yaml`(0.12) |
| `b767f09` | c | 가짜 Pode·팔 노드와 ROS 시험, `fake_arms:=none` 고침(`70a5438`) |
| `a868a77` | d | `/fleet/robots` 5 Hz, IR-41(지연 ±20%, 요청 상한, 3 s 넘은 요청 버리기). 네트워크 파일 없음 |
| `8b1eb89` | e | `ReportTripCmd` 정의와 시험(`ReportTrip.srv` 철자 대조 포함) |
| `612afd0` | f | `/system/mode` 구독, 바뀔 때 로그만. `TODO(Q-nn)` 두 곳(IR-73, IR-30) |

합계 91개 파일, +7,483 −221. 팀 파일을 지운 것은 0개다. 문서를 빼고 `arm[123]`은 0줄이다(팀 `contract.yaml` 변경 이력 주석 한 줄 빼고). 커밋 메시지에 AI 표시가 없다.

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

- `pode_fsm`의 6개 건너뜀은 `test_ros_matches_fake_pode.py`(lane 지도용으로 쓰였다는 사유)로, pode-drive에서도 원래 건너뛰던 것이다.

## 남은 것

- **PR(④):** 사용자가 말하면 연다. 대상 `prepare_dev`. `prepare_dev`가 지금 그대로면 merge 없이 우리 커밋만 보인다.
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

다른 사람이 먼저 쓰지 않았다면 Q-18부터 쓴다.

```markdown
| Q-18 | fleet_manager 단일 스레드 실행 (IR-30 예외) | 제안 | IR-30, `fleet_manager_node.py` |
| Q-19 | 이번 주 `/system/mode`는 로그만 (IR-73) | 제안 | IR-73, 주문 흐름 1차 6.4 |

### Q-18 fleet_manager 단일 스레드 실행 (IR-30 예외)
- **상태:** 제안
- **문제:** IR-30은 비동기 호출과 MultiThreadedExecutor를 요구한다. fleet_manager는 `rclpy.spin` 단일 스레드로 돈다.
- **안:** 예외로 둔다. `call_async`만 쓰고 콜백 안에서 기다리지 않아 교착이 없다. 코어가 결정적이어야 기록 재생 시험(`tools/replay.py`)이 맞는다.
- **결정되면 고칠 곳:** IR-30에 예외 한 줄, 코드의 `TODO(Q-18)`.

### Q-19 이번 주 `/system/mode`는 로그만 (IR-73)
- **상태:** 제안
- **문제:** IR-73은 TEST 모드에서 주문을 거부하라고 한다. 이번 주 fleet은 모드가 바뀔 때 로그만 남긴다(주문 흐름 1차 6.4 표).
- **안:** 다음 주(스프린트 5)에 거부를 넣는다.
- **결정되면 고칠 곳:** `fleet_manager_node.py`의 `TODO(Q-19)`.
```

### 9. IR-61 (로봇 거절을 응답으로)

> `pode_fsm`은 이동 명령을 seq 토픽으로 받아서, 거절을 서비스 응답이 아니라 `"rejected"` 이벤트로 돌려줍니다. 이번 주는 예외로 적고 다음 주에 맞추겠습니다.

### 10. IR-62 (로봇 무응답 → `ROBOT_TIMEOUT`, 30초 안에 끝 상태)

> IR-62는 관리 명령 규칙으로 읽힙니다. 관리 명령은 이번 주 범위 밖이라, 이번 주 이동·도킹 명령에도 적용해야 하는지 여쭙니다.
