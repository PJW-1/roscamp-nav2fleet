# interface-align에 이번 주 브랜치 합치기 — 2026-11-24

`feat/interface-align-2026-11-24`(다음 주 판, contract 0.7)에 이번 주 브랜치(0.6 기준)를 merge로 합쳤다. rebase와 force push는 쓰지 않았다.

## 합친 순서

| 커밋 | 합친 것 | 충돌 |
|---|---|---|
| 5d655b0 | main c62c03d (bag_record, 11-23 보고서) | 없음 |
| b6d2d5d | `feat/arm-rename-0.6-2026-11-24` (1dae72b) | 9개 파일, 아래 |
| f0577b2 | `feat/no-motion-2026-11-24` (8afadcb) | `pode/pode_fsm/pode_fsm/node.py` |
| 00c8d2e | (고침) pode_fsm README의 상태 주기 10 Hz → 5 Hz | — |
| ab22bce | `feat/no-motion-2026-11-24` (80f02fd, 보고서 고침) | 없음 |
| 449d2bd | (고침) `test_ros_no_motion.py` 시나리오의 팔 이름 | — |
| f3f156a | main aff00d7 (bag_record 후속) | `tools/real_pode/README.md` |

## 충돌을 맞춘 방법

### 팔 이름 (b6d2d5d)

interface-align에는 이미 같은 이름 바꾸기가 들어 있어서, 이름이 서로 다른 곳은 없었다. 충돌은 줄 바꿈 위치와 interface-align에만 있는 줄 때문에 났다.

| 파일 | 고른 쪽 | 이유 |
|---|---|---|
| `interfaces/fleet_interfaces/contract.yaml` | interface-align | `placeholders.arm`이 팔 셋 모두(`/{arm}/state`에 필요), `robot` 틀도 v0.2에서 더한 것. `swap_basket`의 `instances`는 양쪽 모두 `[arm_ship]` |
| `bringup/iponic_bringup/launch/pc.launch.py`, `sim/fake_robots/launch/fakes.launch.py` | interface-align | `fake_arms:=none` 설명 줄이 interface-align에만 있음 |
| fleet_core, fake_robots 시험 5개 | interface-align | 내용은 같고 줄 바꿈만 다름. `fake_arms:=none` 시험은 interface-align에만 있음 |
| `docs/IFP_공통설계.md` 두 줄 | arm-rename | 조사 고침(`arm_left과` → `arm_left와`)만 다름 |

### pode_fsm (f0577b2)

no-motion과 interface-align이 `node.py`의 같은 자리를 고쳤다. 둘 다 동작하게 합쳤다.

- 관리 서비스(`set_lock`, `set_mode`, `charge_end`)는 `check_clear` 다음에 그대로 만든다. 동작 없음 모드에서도 같이 뜬다.
- `GoalServers`는 no-motion 쪽 시그니처(끝에 `self._nm`)를 쓴다.
- 상태 주기는 interface-align의 5 Hz(IR-34)를 두 모드에 같이 쓴다. 그래서 no-motion의 `NO_MOTION_STATE_HZ` 상수는 지웠다.
- 자동으로 합쳐진 `core.py`, `order_fsm.py`: interface-align의 `has_goal()`과 no-motion의 `dock_wait()`가 함께 있다. 동작 없음 도킹 중에는 `_active`가 Dock이라 `has_goal()`이 참이다. 그래서 도킹 중에 새 요청으로 락 상태를 바꾸려 하면 `NOT_STOPPED`로 거절된다("락은 멈춘 로봇에서만"과 같다). 지금과 같은 값을 요청하면 성공하고, 같은 `request_id`로 다시 보내면 처음 응답을 그대로 돌려준다.
- RobotState의 boot 칸(`boot_id`, `lock`, `mode`, `forced`, `last_request_id`)은 동작 없음 모드에서도 같은 코드로 나간다.

### bag_record 후속 (f3f156a)

`tools/real_pode/README.md` 실행 순서 7, 8번: interface-align의 도메인 6(IR-81)과 main의 `bag_record.sh stop` 단계를 둘 다 넣었다.

## 시험

별도 설치 공간, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 묶음마다 다른 도메인(68~71), fleet_manager는 `SCENARIO_FIRST_DOMAIN=80 FMS_RESTART_DOMAIN=79`.

| 묶음 | 판 | 결과 |
|---|---|---|
| fleet_core (contract, contract_check 포함) | ab22bce | 1296 passed, 7 xfailed |
| fleet_manager | ab22bce | 147 passed |
| fake_robots | ab22bce | 1160 passed, 1 skipped, 30 xfailed |
| pode_fsm | ab22bce | 3 failed, 500 passed, 6 skipped → 아래 1 |
| pode_fsm `test_ros_no_motion.py` | 449d2bd | 8 passed |
| tools | ab22bce | 1 failed, 287 passed → 아래 2 |
| tools | f3f156a | 289 passed (따로 돌림) |
| pode_fsm | f3f156a | 503 passed, 6 skipped |

`scripts/check_interfaces.py`는 떠 있는 노드를 보는 도구라 따로 돌리지 않았다. 가짜 노드를 띄워 같은 비교를 하는 `sim/fake_robots/test/test_ros_check_interfaces.py`가 fake_robots 묶음에서 통과했다.

## 실패와 원인

1. **pode_fsm 3개 (고침, 449d2bd)**: `test_ros_no_motion.py`의 시나리오 재고가 옛 이름 `arm1`, `arm2`를 썼다. no-motion 브랜치는 0.6 main(옛 이름)에서 시작해서 그 브랜치에서는 맞지만, 새 이름 판에서는 가짜 노드가 `stock for arm1, which is not one of ['arm_left', 'arm_right']`로 거부한다. FakePode 비교 시험 세 개가 비교 전에 멈췄다. 새 이름으로 바꾸자 그 파일 8개가 모두 통과했다. Codex 리뷰가 찾았다.
2. **tools 1개 (부하 때 시간 문제)**: `tools/test/test_step_probe.py::test_a_second_probe_is_refused`. 시험 네 묶음과 다른 세션의 시험이 함께 돌던 때(load 10 안팎) 실패했다. 따로 세 번 돌리면 세 번 모두 통과한다. 합친 판은 이 도구를 바꾸지 않았다.

## 뒤에 더 합친 것 (2026-11-24 오후)

no-motion의 결정 세 가지(사용자 결정)를 no-motion 브랜치에 넣은 뒤 같은 방식으로 합쳤다. 충돌은 없었다.

| 커밋 | 합친 것 |
|---|---|
| 1d507bf (21acc69 합침) | 모르는 `dock_id`는 경고만(`no_motion_dock_ids`, 기본 `WORK`, `OUTBOUND`), `NO_MOTION=1 start.sh NAMESPACE [X Y YAW]`(로봇별 기본 자리 P-1~P-3은 `start.sh`에만), `stop.sh`가 이것도 내림, order current_goal은 실주행처럼 유지 |
| 978f533 | 리뷰로 고친 것: 자세는 숫자만, `PEER_ROBOTS`는 비어야 함, `stop.sh`는 `real_pode.launch.py`나, 명령줄에 `pode_fsm.launch.py namespace:=… no_motion:=true initial_x:=`가 이 순서로 든 그룹만 내림(그 뒤 인자는 보지 않음) |

| 시험 (합친 판) | 결과 |
|---|---|
| pode_fsm | 506 passed, 6 skipped (1d507bf) |
| tools 전체 | 1 failed, 304 passed, 1 skipped (1d507bf) → 아래 |
| `test_real_pode.py -k "no_motion or stop or start"` | 72 passed (978f533) |

- tools의 실패는 앞의 2번과 같은 `test_step_probe.py::test_a_second_probe_is_refused`다. 다른 세션의 Gazebo 측정으로 load average가 50을 넘던 때였다. 그때 따로 다섯 번 돌리면 세 번 실패했다. 이 도구와 시험은 main과 같고, 합친 것들이 건드리지 않았다. 부하가 낮던 때의 전체 실행(f3f156a, 289 passed)과 no-motion 브랜치 전체 실행(304 passed, 1 skipped)에서는 통과했다.
- `start.sh`와 `stop.sh`는 no-motion 브랜치의 판과 같다. 거기서 같은 시험 72개가 통과했다.

## 앞으로

이번 주 브랜치(`feat/no-motion-2026-11-24`, `feat/arm-rename-0.6-2026-11-24`, main)에 고친 것이 생기면 같은 방식(merge, force 없음)으로 이 브랜치에 계속 합친다.

no-motion 브랜치를 0.6 main에 합칠 때도 같은 문제가 생긴다. arm-rename이 main에 먼저 들어가면 `test_ros_no_motion.py`의 `arm1`, `arm2`를 같은 커밋에서 바꿔야 한다.
