# ReportTripCmd 정의와 단계 대응표 제안 (2026-11-24)

- 브랜치: `feat/report-trip-cmd-2026-11-24`. pode-drive main `aff00d7`(인터페이스 0.6)에서 시작했다.
- 할 일: Drive `통신규칙_인터페이스_정렬_할일_2026-11-24.md`의 이번 주 5번이다. 주문 쪽 TT-27(`fleet_manager_node`의 `Hello`·`ReportTrip` 보내기)이 이 정의를 기다린다.
- 기준: 팀 레포 `docs/design` 브랜치 `docs/설계/주문_흐름_1차_인터페이스.md` 4.4 ①(정의 초안)과 ②(단계 대응표). 팀 `feat/contract-0.7`의 `srv/ReportTrip.srv`와도 맞췄다.
- 범위: 정의(`common/commands.py`)와 단계 대응표 합의는 핑키·팔 관제가 맡는다. `orders/`에서 단계마다 내보내는 일은 통합 관제가 맡고, 노드에서 `ReportTrip`으로 바꿔 보내는 일은 주문이 맡는다. 그래서 이 브랜치는 `orders/`와 `fleet_manager_node`를 건드리지 않았다.

## 한 일

| 파일 | 바뀐 것 |
|---|---|
| `pc/fleet_core/fleet_core/common/commands.py` | `ReportTripCmd(trip_id, robot, status, placed, missing, reason="")`를 더했다. 상수는 `TRIP_ASSIGNED`·`TRIP_FILLING`·`TRIP_TRANSPORTING`과 `TRIP_STATUSES`(정상 흐름 순서)다. 문서 4.4 ① 초안과 같고, 번호와 시각은 넣지 않았다(IR-22, 노드 한 곳에서 붙임) |
| `pc/fleet_core/test/test_report_trip_cmd.py` (새 파일) | 칸 이름·순서·기본값, 상태 철자(`ReportTrip.srv`의 `STATUS_*`와 같음), 끝 상태가 없는 것, frozen을 확인한다. `FleetManager`를 거쳐 ROS 쪽 출력까지 같은 객체가 순서 그대로 나오는지, 기록 파일에 `"out": "ReportTripCmd"` 줄로 남는지도 확인한다 |
| `pc/fleet_core/test/test_fms_boundary.py` | `ORDER_SIDE_NAMES`에 `ReportTripCmd`를 넣었다. 교통 층(`traffic/`)이 이 이름을 가져오면 실패한다(검사 강화) |
| `sim/fake_robots/fake_robots/world/apply.py`, `world/stations.py` | 가짜 세계가 `ReportTripCmd`를 받아 가짜 central의 `trip_reports`에 도착 순으로 남긴다. 아래 "가짜 세계" 참고 |
| `sim/fake_robots/test/test_world_central.py`, `test_world_faults.py` | 가짜 central의 받기 규칙 7개와, 세계에 넣어도 `unknown output` 위반이 생기지 않는지 확인한다 |
| `docs/IFP_공통설계.md` | FMS 모듈 표에 "진행 보고" 줄을 하나 더했다 |

- 관리자 경로는 고칠 것이 없었다. `ManagerRoute._route`가 모르는 항목을 바깥 출력으로 그대로 넘기고, 기록(`_note`)은 `asdict`로 어떤 데이터 클래스든 적는다. 위의 시험이 이를 확인한다.
- 지표(`metrics.on_output`), 반응 감시(`reaction.on_command`), triage 읽기(`tools/triage/readers.py`)는 모르는 종류를 그냥 넘긴다. 그래서 고치지 않았다.

### 가짜 세계

- 원래 `apply.py`는 모르는 출력을 `unknown output` 위반으로 적었다. 그대로 두었다면 통합 관제가 `orders/`에서 `ReportTripCmd`를 내기 시작하는 순간, 가짜 세계를 쓰는 시나리오 시험이 모두 위반으로 실패했을 것이다.
- `Central.take_trip_report`가 받는다. 실제로는 브리지가 받을 보고지만, 판(주문 목록)을 아는 곳이 가짜 central이라 그 안에 두었다.
- 위반으로 보는 경우: 진행 단계가 아닌 `status`, 판에 없는 건, 끝 보고를 이미 받은 건, `ROBOT_FAULT` 보류 뒤의 진행 보고(지금 `take_report`와 같은 규칙).
- 단계 순서는 보지 않는다. IR-25가 상태 순서로 가리지 않는다고 정했고, 대기열로 되돌린 주문은 다시 `ASSIGNED`가 되기 때문이다. `ARM_FAULT` 보류 뒤의 진행 보고도 받는다(보류 뒤 끝 보고를 기다리는 상태라서).

## 단계 대응표 제안 (4.4 ②, 통합 관제와 합의할 것)

pode-drive의 구현된 주문 코어(`orders/`)에서 자리를 찾았다. 팀 main에는 뼈대와 의사코드만 있으므로, 함수 이름은 의사코드 3장 단계와 같이 적었다.

| `status` | 내는 때 | 코드 자리 (pode-drive 구현) | `placed` · `missing` |
|---|---|---|---|
| `ASSIGNED` | 맨 앞 주문을 로봇에 묶을 때 한 번 (2단계) | `Stations._start_filling`, `ORDER_BOUND` 이벤트 바로 옆 | `{}` · `{}` |
| `FILLING` | 묶은 뒤 **첫** `ArmFillCmd`를 낼 때 한 번 (4단계) | `Stations._step_filling` | `{}` · `{}` |
| `TRANSPORTING` | 하나 이상 담고 출고로 떠날 때 한 번 (5단계) | `Stations._finish_fill`의 담은 것이 있는 갈래(`r.basket = FULL` 뒤). `_send_to_outbound` 안이 **아님** | `order.placed` · `order.missing` (`_finish_fill`이 바로 앞에서 `missing`을 계산함) |
| `SHIPPED`·`PARTIAL`·`CANCELED`·`HELD` | 지금 `ReportOrderCmd` 그대로 | `orchestrator.py` `_report`, `_hold` | 그대로 |

### 합의할 경계 사례

1. **`TRANSPORTING`의 자리.** 문서는 "`DispatchCmd(OUTBOUND)`를 낼 때"라고 적었다. 그런데 이 명령은 `_send_to_outbound` 한 함수에서 세 경우에 나온다.
   - 다 담은 뒤 출발할 때(`_finish_fill`).
   - 출고 배차를 다시 시도할 때(`_retry_outbound`).
   - 주문 없이 빈 바구니를 받으러 갈 때(`orchestrator.py`의 `basket == NONE and not r.order`).

   문서 문구대로 `_send_to_outbound`에 넣으면, 재시도마다 같은 상태의 보고가 새 번호로 또 나간다. 빈 바구니를 받으러 갈 때는 운반 건이 없어 `trip_id`를 채울 수 없다. 그래서 `_finish_fill`의 담은 것이 있는 갈래에서 한 번 내자고 제안한다.
2. **`FILLING` 반복.** 일부만 담긴 결과가 오면 `_step_filling`이 남은 것으로 `ArmFillCmd`를 다시 낸다. 문서대로 첫 번째에서만 낸다. 담는 도중 누적 `placed`를 화면에 보이려면 매번 내도 되지만, 고객 화면이 둘 다 "준비 중"이라 이번 주에는 필요 없다.
3. **`ASSIGNED`와 `FILLING`이 거의 함께 나감.** `on_dispatch_result`가 `_start_filling`으로 묶고, 그 로봇의 다음 `step`에서 `_step_filling`이 첫 `ArmFillCmd`를 낸다. `ManagerRoute._step_robot`이 `traffic.step` 뒤에 `orchestrator.step`을 한 번 더 부르므로, 대개 같은 tick이다. 노드가 기다리는 보고를 최신 상태로 바꿔 끼우면(IR-23), 브리지는 `ASSIGNED`를 못 받을 수 있다. 보고가 늘 지금 상태 전체라 문제는 없다. 문서에도 적혀 있다.
4. **대기열로 되돌린 주문 (`ORDER_REQUEUED`).** 로봇 장애나 격리로 담은 것 없이 주문을 놓으면(`faults.py`), 0.6 코어는 이벤트만 내고 보고는 내지 않는다.
   - 그동안 웹 화면은 옛 로봇의 `FILLING`("준비 중")으로 남는다. 다른 로봇이 다시 묶으면 같은 `trip_id`로 새 로봇의 `ASSIGNED`가 간다.
   - 브리지 쪽 확인이 필요하다. IR-24의 "허용되지 않는 전이" 목록이 설계 문서 어디에도 없다. 브리지가 `FILLING → ASSIGNED`를 허용되지 않는 전이로 보면, 새 배정이 `LATE_REPORT`로 남고 화면에 반영되지 않는다.
   - IR-25는 상태 순서로 가리지 않는다고 하므로 허용하는 것이 맞다고 본다. 주문 쪽 TT-24(`apply_report`)에서 확인한다. `RECEIVED` 되돌림 보고는 문서대로 다음 주에 정한다.
5. **담은 채 장애.** 지금처럼 `ReportOrderCmd(HELD, ROBOT_FAULT)`를 낸다. 그 뒤 같은 건의 진행 보고는 없어야 한다(가짜 세계가 위반으로 잡는다).
6. **`reason`.** 이번 주에는 비운다. `CANCELED → CANCELLED` 철자 바꾸기는 노드에서 하며, `ReportTripCmd`에는 끝 상태가 없어 해당하지 않는다.

## TT-27 검토 때 볼 것

11-24 오후에 팀 레포를 확인한 결과, 주문 쪽 노드 코드(`Hello`·`ReportTrip` 클라이언트)는 아직 어느 브랜치에도 없다. 그래서 검토할 코드가 아직 없고, 코드가 올라오면 아래를 본다.

- `fleet_manager_node._dispatch`의 `match`에는 `case _`가 없다. `case ReportTripCmd():`를 넣지 않으면 진행 보고가 로그 없이 버려진다.
- IR-22는 `Hello`를 받기 전에 `Report*`를 보내지 않는다고 정했다. 여기에 `ReportOrder`도 들어간다면, 브리지가 없는 우리 가짜 실행(`use_fake:=true`, 시나리오 시험)에서는 central에 끝 보고가 가지 않는다. 그러면 주문판이 비지 않는다. 가짜 노드에 가짜 브리지(`Hello`·`ReportTrip` 서버)를 더하거나, 브리지 없이 돌리는 실행 인자가 있어야 한다. 0.7 정의가 들어온 뒤에 정한다.
- `report_n`은 노드 하나의 카운터이고, 운반 건마다 하나씩 순서대로 보낸다(문서 4.4 ③).

## 팀 `feat/contract-0.7` 확인 (11-24, 읽기만 함)

양만철 님이 11-24에 올린 브랜치로, contract 0.7, 새 정의 4개(`SubmitTrip`, `Hello`, `ReportTrip`, `SystemMode`), 팔 이름 바꾸기가 들어 있다. Drive 할 일 문서 기준으로는 아직 통합 브랜치로 알리기 전이다.

- **팔 이름**: 우리 `feat/arm-rename-0.6-2026-11-24`와 새 이름이 같은 자리에 같다. contract의 `placeholders.arm`과 `swap_basket` instances, 지도의 `arms`·`server`, `fake_arms` 기본값, 가짜 팔 노드의 `arm_id` 기본값, 시험 값과 시나리오가 모두 같다.
- **다른 점**:
  - 팀 쪽 contract.yaml 5번 줄에만 바꾸기 설명 주석이 있다. 그 줄에 옛 이름이 들어 있어서 `git grep -nwE 'arm[123]'`에 한 줄 걸린다.
  - 팀 쪽에만 있는 파일(`ArmState.msg` 주석, README 몇 개, CODEOWNERS)이 있다.
  - `orders/`는 양쪽 다 바꾸지 않았다.
- **합칠 때 예상되는 충돌**: contract.yaml의 버전 줄(0.6/0.7)과 주석 정도다.
- **`commands.py`와 `fleet_manager/`**: 팀 브랜치는 `fleet_manager/test/fms_restart.yaml` 한 줄(재고의 팔 이름) 말고는 건드리지 않았다. 그래서 `ReportTripCmd`와 충돌하지 않는다.
- **`ReportTrip.srv`**: 진행 단계 상태 철자가 `TRIP_STATUSES`와 같다. 요청 칸은 `trip_id, report_epoch, report_n, status, reason, robot_id, placed, missing, stamp`이다.

## 할 일 문서 3·4번 상태 (이번에 하지 않음)

- **3번**: `/fleet/robots` 발행은 main에 이미 있다(`fleet_manager_node`가 1초마다 `RobotStatusArray` 발행, Pode 3대). 남은 일은 `/system/mode` 구독(로그만)이다. 0.7의 `SystemMode`가 필요하고, 담당(나 또는 통합 관제)도 아직 정하지 않았다.
- **4번**: 실행 인자 `use_fake`·`fake_robots`·`fake_central`·`fake_arms`는 이미 있다. `fake_robots:=none`은 `fakes.launch.py`의 `_names`가 빈 목록으로 바꾼다. 실제 팔 노드에 붙이기와 시험용 도메인은 통합 브랜치가 나온 뒤에 한다.

## 시험

- 먼저 새 시험을 쓰고, 정의가 없는 상태에서 import 오류로 실패하는 것을 확인했다.
- 구현 뒤 새 시험과 경계 시험, 가짜 central 시험을 돌렸다: 47개 통과.
- 소스에서 `pc/fleet_core/test` 전체와 `sim/fake_robots/test`의 `test_world_central.py`·`test_world_faults.py`·`test_world_planner.py`를 돌렸다: **1324 통과, 7 xfail**(원래 있던 예상 실패).
- 다른 세션이 Gazebo 측정을 돌려 부하 평균이 50을 넘었다. 그래서 `nice`로 돌렸고, fake_robots 시나리오 시험 전체는 25분 제한에 걸려 끝까지 돌지 못했다. 이 브랜치가 가짜 세계에 더한 것은 `ReportTripCmd` 갈래 하나뿐이고, 그 갈래로 들어오는 출력은 아직 없다(`orders/`가 내지 않음).
