# 주문 보류 v2.4 작업 기록 (2026-11-19, 브랜치 fix/order-hold-v2.4)

## 결론
- 계약 0.6과 설계 v2.4의 주문 보류 동작(①~⑦)을 넣었다.
- colcon 빌드, 전체 시험, ruff, 가짜 벤치 시나리오가 모두 통과했다.
- 교통(traffic), Pode, bringup, Gazebo sim 코드는 바꾸지 않았다.
- 교통에 닿는 변화는 하나다. 팔 고장이 나도 Pode를 격리하지 않는다.

## 계약 0.6
- **`ReportOrder.srv`**: `STATUS_CANCELED`, `HOLD_ROBOT_FAULT`, `HOLD_ARM_FAULT` 상수와 `hold_reason` 필드를 더했다.
- **`ForceShip.srv`(새 파일)**: 요청은 `order_id`, 응답은 `accepted`와 `reason`이다. 받아들이면 reason은 빈 문자열이다. 거절 사유 상수는 NOT_HELD, ROBOT_FAULT, UNKNOWN_ORDER, NOT_IMPLEMENTED 넷이다.
- **`contract.yaml`**: `version: 0.6`으로 올리고 `force_ship`(`/fleet/force_ship`)을 더했다.
- **CMakeLists**: ForceShip.srv를 더했다.
- 두 srv는 팀 지시서의 글자와 같다. `test_contract.py`가 이를 확인한다.
- **fleet_manager**
  - `ReportOrder` 요청에 `hold_reason`을 넘긴다.
  - `/fleet/force_ship` 서비스 서버를 열어 요청을 `FleetManager.force_ship(order_id)`로 넘긴다.
  - 새 노드는 만들지 않았다. 기존 fleet_manager 노드에 서버만 더했다.

## 동작 (fleet_core)
- **보류의 뜻**: HELD는 끝(최종)이 아니다. 끝은 SHIPPED, PARTIAL, CANCELED 셋뿐이다. 보류 중인 주문은 다시 배정하지 않는다.
- **① 용량 초과**: 물건 수 합이 바구니 용량을 넘는 주문은 배정하지 않는다. WARN `ORDER_OVER_CAPACITY`를 주문마다 한 번 내고, ReportOrder는 보내지 않는다.
- **② 하나도 못 담음**: CANCELED로 보고한다.
- **③ `ship_partial` 삭제**: 파라미터를 지웠다. 동작은 늘 "모자라도 내보냄"이다.
- **④ 운반 중 Pode 고장(격리, 치움)**: 바구니에 물건이 있으면 HELD/ROBOT_FAULT로 한 번 보고하고 되돌리지 않는다. 비어 있으면 예전처럼 ORDER_REQUEUED다.
- **⑤ 담는 중 Pode 고장**
  - 격리: 팔 결과를 기다린 뒤 ④와 같은 기준으로 판단한다.
  - 치움(confirm_clear): 팔 작업을 버리고 그때까지 담은 것으로 판단한다.
- **⑥ 팔 고장**
  - `arm/jobs.py`가 재시도 한도와 시간 초과 때 `RobotFaultCmd` 대신 `ArmFault(robot, kind, server, reason)`를 낸다.
  - 주문은 HELD/ARM_FAULT가 된다. Pode는 격리하지 않고 작업대에 붙잡아 둔다.
  - 예외가 둘 있다.
    - 이미 격리된 Pode면 늦은 결과나 치움을 기다린 뒤 ④ 기준으로 판단한다. 늦은 SWAP 결과가 가득 찬 바구니를 가져갔다고 하면 먼저 출고를 보고한다.
    - 출고 보고 뒤 빈 바구니를 넣다가 고장 나면 주문이 없으므로 Pode만 붙잡는다.
- **⑦ 강제 출고**: `on_force_ship`이 ARM_FAULT 보류만 받는다.
  - 채우기 보류에서 담은 것이 있으면 출고한다. 출고 보고는 모자란 것이 있으면 PARTIAL, 없으면 SHIPPED다.
  - 출고 팔(SWAP) 보류면 교환을 다시 시도한다. 보고 규칙은 같다.
  - 채우기 보류에서 담은 것이 없으면 출고하지 않는다. CANCELED로 보고하고 Pode를 놓아 준다.
  - 그 밖에는 UNKNOWN_ORDER, ROBOT_FAULT, NOT_HELD로 거절한다.
  - 입력은 기록에 남고 재생된다.

## 교통 쪽 영향
- **팔 고장**: 이제 교통에 알리지 않는다. 팔 고장 때문에 ROBOT_ISOLATED가 나오지는 않는다. 다른 원인(주행 고장, 무응답)의 격리는 예전과 같다.
  - 붙잡힌 Pode는 사용 가능 상태로 작업대 구역을 계속 잡고 있다.
  - force_ship이 오면 채우기 보류는 담은 것이 있을 때만 출고 경로로 떠나고, 없으면 놓아 준다. 출고 팔 보류는 출고 작업대에서 교환을 다시 시도하며 새 배차는 없다.
  - confirm_clear는 예전 조건대로 거절될 수 있다. 받아들여져도 요청에 적은 구역만 놓는다.
- **교통에 고장을 알리는 길**: 주문 쪽에 남은 것은 출고 배차 재시도 한도 초과 하나다. `_fail`은 `orchestrator.py`에 정의되어 있고 `stations.py`에서 부른다.
- **`ship_partial`**: 파라미터 로더는 모르는 키를 거부한다. 그래서 `ship_partial`이 든 예전 `fleet_params.yaml` 복사본으로는 fleet manager가 뜨지 않는다.

## 가짜 노드와 시나리오
- **가짜 central**
  - CANCELED는 최종이다. HELD는 주문을 판에 남기고 사유를 기록한다.
  - `force_ship_after_s` 옵션을 주면 ARM_FAULT 보류 뒤 `/fleet/force_ship`을 부른다.
- **판정기**: 끝난 주문에 다시 보고가 오면 위반으로 센다(`order_reported_after_final`).
- **세계 안 central**: 판에 남은 ROBOT_FAULT 보류는 central이 닫을 주문으로 보고 끝난 것으로 센다.
- **새 시나리오 `arm_fault`**: HELD/ARM_FAULT 뒤 force_ship이 와서 PARTIAL로 끝난다.
- **바꾼 시나리오**: `stock_shortage`(CANCELED 2, 용량 초과 1), `auto_isolation`과 `operator_recovery`(격리 0, ORDER_HELD 1).

## 시험 (전 936ae42 → 후 4e733ab)
| 묶음 | 전 | 후 |
|---|---|---|
| fleet_core | 1111 (+7 xfail) | 1147 (+7 xfail) |
| fleet_manager | 131 | 140 |
| fleet_gz_sim | 598 | 598 |
| tools | 156 | 156 |
| pode_fsm | 467 (+6 skip) | 467 (+6 skip) |
| pode_avoidance, nav_config, peer_obstacles, scan_filter | 26, 106, 19, 23 | 26, 106, 19, 23 |
| fake_robots | 1065 (+1 skip, +27 xfail) | 1118 (+1 skip, +27 xfail) |

- ruff는 전후 모두 106건이고 목록도 같다.

## 가짜 벤치 (`pc.launch.py use_fake:=true`)
| 시나리오 | 결과 |
|---|---|
| real_two_orders (`fake_robots:="pode1 pode2" map_file:=v7`) | 4건 모두 SHIPPED, 156 s |
| arm_fault (3회) | O-001 HELD ARM_FAULT → FORCE_SHIP → PARTIAL, O-002 SHIPPED |
| stock_shortage (3회) | SHIPPED 2, PARTIAL 2, CANCELED 2, OVER_CAPACITY 1 |
| auto_isolation (2회) | 4건 모두 SHIPPED |
| operator_recovery | 4건 모두 SHIPPED |
| reinstate | 7건 모두 SHIPPED |

## 재생 비교 (실물 기록 8개)
- **대상**: run8 기록은 없어 남은 8개를 썼다.
- **결과**
  - 줄 수는 8개 모두 같다.
  - 다른 줄은 `"hold_reason": ""` 필드가 붙은 ReportOrderCmd 5줄뿐이다.
- **한계**
  - 원본 peer_on_order_on_3에는 짐을 실은 Pode의 격리가 있다(2132~2133행). v2.4라면 이 주문은 HELD/ROBOT_FAULT다.
  - 하지만 재생이 그 장면을 다시 만들지 못한다. main으로 재생해도 421행부터 원본과 갈라진다.

## 확인 필요 (지시서가 정하지 않아 기본값을 고른 곳)
1. 출고 팔(SWAP) 고장은 HELD/ARM_FAULT로 보고한다.
   - force_ship이 오면 교환을 다시 시도한다.
   - 모자라면 PARTIAL, 다 있으면 SHIPPED로 보고한다.
2. ARM_FAULT 보류 주문이 주문판에서 사라지면 Pode를 붙잡아 둔다. 운영자가 confirm_clear와 reinstate로 푼다.
3. ARM_FAULT로 붙잡힌 Pode가 격리되거나 치워지면 ④ 규칙을 따른다.
4. 이미 격리된 Pode의 팔이 시간 초과되면 늦은 결과나 치움을 기다린다.
5. HELD 뒤 늦은 팔 결과가 오면 처리가 갈린다.
   - 늦은 FILL 결과는 placed와 missing을 고친다. 격리된 Pode면 그 뒤 ④ 기준(HELD/ROBOT_FAULT 또는 되돌림)으로 판단한다.
   - 늦은 SWAP 결과는 바구니 상태를 바꾸고 출고를 보고할 수 있다.
   - ARM_LATE_RESULT 문구는 그대로다.
6. 가짜 central의 재고 보정에는 SWAP 강제 출고와 응답 경합 때 한계가 있다.
7. `tools/situations.py`의 미완료 원인 분류는 ROBOT_FAULT 보류를 아직 모른다.
