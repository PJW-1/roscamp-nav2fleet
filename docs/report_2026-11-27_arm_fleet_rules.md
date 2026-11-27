# 팔 관제 합의 1~14 구현 보고 (2026-11-27)

브랜치 `feat/phody-fleet-decisions-2026-11-27` (개인 레포). 팀 레포 PR 브랜치는 `feat/arm-fleet-rules-2026-11-27` (push · PR 전).
contract 0.9 → **1.0** (다음 변경 basket_capacity는 1.1). 팀원 전달용 계약 정리는 별도 md(다운로드 폴더 `팔_관제_계약_정리_2026-11-27.md`, 2판).

## 1. 결정 (사용자 결정 그대로)

| # | 결정 | 상태 |
|---|---|---|
| 1 | `fill_timeout_s` 90 → 1200 s (통합 시험용, Phody 실측 뒤 다시 맞춤) | 완료 |
| 2 | 담기 실패: 다시 보내는 것은 `NOT_READY`만. `RECOGNITION_ERROR` · `OUT_OF_STOCK` · `PICK_FAILED` → FILLING_PAUSE. uncertain · `ARM_FAULT` · 그 밖 → HELD. `UNKNOWN_TARGET` → 다시 보내지 않음(HELD). PICK_FAILED는 자리만, Phody가 당분간 보내지 않음 | 완료 |
| 3 | [장애 처리 완료]: 보낼 때 ERROR였던 팔은 accepted여도 ERROR를 벗어날 때 ACCEPTED(첫 요청부터 30 s, 넘으면 TIMEOUT). 답이 없을 때도 같은 관찰. 실제로 보낸 뒤 답 전에 본 벗어남도 씀. ERROR가 아니던 팔은 accepted면 바로 확정 | 완료 (ae6f767e, 7bf36deb, 3차 후속) |
| 4 | 한 팔이라도 ERROR면 그 작업대로 배차하지 않음, 벗어나면 다시 엶 | 완료 |
| 5 | (변경됨) 담기 중 쥔 채 lock 해제 → 바로 HELD 아님. PAUSED로 [복구]를 기다리고, 목표가 끝날 때 uncertain이 있으면 HELD. 안전장치: 쥔 채 lock 해제가 있었으면 uncertain이 비어도 HELD | 완료 |
| 6 | 팔 상태가 3 s 끊기면 "상태 모름": 배차 막고 경고, 담기는 보류하지 않음. 한 번도 안 보낸 작업대 팔도 노드 시작부터 셈, 도착 시각으로 셈, 한도 직후 틱 전에 온 첫 상태도 끊김을 먼저 알림 | 완료 |
| 7 | 서비스 정지(PAUSE): 담기를 시작한 운반 건은 출고 · 교환까지, 대기 주문 배차와 새 담기는 막음, 팔에 신호 없음 | 완료 |
| 8 | `SetPause.srv`, `/{arm}/set_pause`, `/fleet/resume_arm` 서버 → 두 팔 set_pause(false) | 완료 (노드 merge 6ed3b137) |
| 9 | FillBasket 상수(NOT_READY, UNKNOWN_TARGET, phase 5개), contract 1.0. phase 주석의 PAUSED_DISTANCE · PAUSED_SERVICE 삭제 | 완료 |
| 10 | 가짜 팔이 위를 흉내 냄 | 완료 (merge d044f4c8) |
| 11 | 팔 재시작(boot_id 변경): [복구] 대기(HELD 아님, 옛 목표의 늦은 결과 버림), 담은 수는 마지막 feedback. [복구]: 목표가 살아 있으면 두 팔 set_pause(false), 사라졌으면 남은 상품으로 새 FillBasket(같은 trip_id). 재시작 직전 holding이면 HELD | 완료 |
| 12 | FillBasket feedback: placed_total이 바뀌면 ReportTrip(FILLING, placed=누적). `/fleet/robots` 팔 줄: fsm_state=팔 state, task_type=phase, order_id=담는 운반 건 | 완료 |
| 13 | Phody 정의를 인터페이스 주석 · 팀 문서(명세, 검토 사항 Q-03 · Q-05 · Q-06 · Q-21)에 적기. Q-05는 "서비스 정지용으로는 필요 없어짐, [복구] 신호로 set_pause(false)만 남김" | 완료 (팀 브랜치) |
| 14 | 가짜 팔: 스스로 멈춤 → set_pause(false)로 이어 감, 재시작 흉내(boot_id, 목표 사라짐, holding 옵션), phase · placed_total feedback | 완료 |
| 추가 | 관리자 버튼 이름은 [복구] = resume_arm = 두 팔 set_pause(false). 따로 만든 [복구] 인터페이스 없음 | 완료 |
| 추가 | `reason_codes.robot_state: [HEARTBEAT_LOST]` (ArmState.detail, 관제 heartbeat 30 s 끊김, 작업은 계속) | 완료 |
| 추가 | PAUSED 동안 fill_timeout 멈춤: 두 팔 중 하나라도 PAUSED인 동안 세지 않고, 벗어나면 남은 시간부터 이어 셈 (사용자 결정 11-27) | 완료 (3ed1bb65) |

### 사용자가 지금대로 두기로 한 것 (11-27)
- 시간 초과로 취소한 뒤 성공 결과가 오고 uncertain이 없으면(쥔 채 lock 해제도 없으면) 출고.
- 재시작 뒤 [복구]로 보내는 새 목표는 1200 s를 처음부터 셈. set_mode는 재시작을 알아채면 [복구]를 기다리지 않고 다시 보냄.
- 서비스 정지 때 작업대로 오던 Pode도 대기 자리로.
- 상태 모름 3 s는 Pode와 같이 쓰는 `comm_timeout_s`.

### 내가 정한 것 (정책 확인 필요할 수 있음)
- 목록에 없는 실패 사유 · 빈 사유 → HELD. 시간 초과 뒤 실패나 결과 없음 → HELD.
- [복구] 거절 코드: `NOT_FOUND`(모르는 이름), `ORDER_STATE`([복구]할 담기 없음), `NOT_READY`(앞 [복구]의 set_pause가 안 끝남).
- 격리된 Pode라도 상품 행방을 모르면(uncertain, 쥔 채 lock 해제, 쥔 채 재시작) 대기열로 되돌리지 않고 HELD(`ROBOT_FAULT`).
- 쥔 채 lock 해제 판단: lock=false 상태나 같은 담기 목표 안의 직전 상태에서 holding=true.
- [장애 처리 완료]의 "보낼 때 ERROR"는 관리 요청을 받은 시점의 상태로 봄(실제 전송은 다음 step, 약 0.1 s 뒤).
- 무음 기간(관제 재시작 직후)에는 끊김을 알리지 않음(전과 같음).

### 정한 것 (11-27 오후)
- PAUSED로 [복구]를 기다리는 동안에는 1200 s를 멈추고, 벗어나면 남은 시간부터 이어 셈.
- 시나리오 3개(arm_fault · operator_recovery · auto_isolation) 기대값을 새 규칙에 맞춤.

## 2. 바꾼 곳

| 파일 | 내용 |
|---|---|
| `interfaces/fleet_interfaces/srv/SetPause.srv` (새) | paused=false = [복구] |
| `interfaces/fleet_interfaces/contract.yaml` | version 1.0, `arm_set_pause`, `reason_codes.robot_state: [HEARTBEAT_LOST]` |
| `interfaces/fleet_interfaces/action/FillBasket.action` | NOT_READY · UNKNOWN_TARGET, PHASE_* 5개, placed_total = 두 팔 합, 사유별 처리 주석, PAUSED_* 주석 삭제 |
| `interfaces/fleet_interfaces/msg/ArmState.msg` | PAUSED · ERROR · boot_id · holding · detail 뜻 |
| `interfaces/fleet_interfaces/srv/SetLock.srv` | Phody 락 해제 동작 |
| `pc/fleet_core/fleet_core/arm/fill_outcome.py` (새) | `next_after_fill`: RETRY / FILLING_PAUSE / HELD |
| `pc/fleet_core/fleet_core/arm/jobs.py` | 작업대 막힘(ERROR · 상태 모름 · [복구] 대기), 쥔 채 락 해제 표시, 재시작 처리, [복구], feedback, 버린 목표의 늦은 결과 |
| `pc/fleet_core/fleet_core/common/commands.py` | ArmFillPaused · ArmFillProgress · ArmFillPlaced · ArmFillResumed · ArmStationBlocked, RESUME_*, TRIP_FILLING_PAUSE, ArmFillResult.held |
| `pc/fleet_core/fleet_core/manager.py` | on_arm_state(boot_id, lock) · on_arm_state_lost · on_fill_feedback · on_resume_arm · arm_task, 정지 중 tick |
| `pc/fleet_core/fleet_core/orders/*.py` (개인 레포만) | FILLING_PAUSE 보고, 진행 보고, 재시작 대기 · [복구] 뒤 남은 상품 요청, 작업대 막힘 · 정지 중 배차 요청 거둠, 격리 보류 |
| `pc/fleet_core/fleet_core/traffic/zone/watch.py`, `traffic/mission.py` | 정지 중에도 멈춤 감시 |
| `pc/fleet_core/fleet_core/data/fleet_params.yaml` | fill_timeout_s 1200 |
| `pc/fleet_manager/fleet_manager/fleet_manager_node.py` | ArmState boot_id · lock 입력, 도착 시각, 상태 끊김(틱 · 첫 상태), `/fleet/resume_arm`, set_pause 클라이언트, feedback, 상태판 팔 줄, fault_resolved confirm_accepted |
| `pc/fleet_manager/fleet_manager/robot_calls.py` | 한 request_id로 여러 팔(key · report), confirm_accepted(받았다는 답도 상태로 확정, 보내는 중 관측 기억) |
| `pc/fleet_manager/fleet_manager/adapters.py` | 팔 줄 phase · trip |
| `sim/fake_robots/...` | 가짜 팔 훅(fill_fail · not_ready · unknown_items · self_stop · recover · restart · app_unlock · error · state_loss), set_pause 서버 |
| `tools/arm_rules_gz.py`, `tools/arm_rules_gz/*.yaml` | Gazebo 확인 도구와 시나리오 (a)~(d) |

## 3. 시험 결과

- fleet_core 단위 (개인, ae6f767e): 1555 passed, 7 xfailed.
- fleet_manager (개인, 파일 하나씩, 도메인 86): adapters 79, admin 12, fms_restart 1, mode_sync 24, node_admin 26, node_arm_rules 11, node_inbox 7, node_order 6, node_reports 5, node_wiring 39(두 번), robot_calls 27, 그 밖 모두 통과.
  - node_inbox · node_wiring이 부하 때 가끔 실패하던 원인: 시험 World가 ArmState를 내지 않아, 담기 차례가 3 s보다 늦으면 작업대가 "상태 모름"으로 막힘(4 s 지연 실험으로 확인, `'WORK': 'arm_left state unknown, arm_right state unknown'`). World가 두 팔 상태를 내게 고친 뒤 4 s 지연 실험도 통과.
- 시나리오 고친 3개 (6e667577, 한 판씩 혼자, 도메인 70): arm_fault 통과(31 s, 처음 한 판은 심판이 팔 상태를 상태판으로 늦게 읽어 실패 → 토픽으로 고침), operator_recovery 통과(71 s), auto_isolation 통과(56 s). basic_orders는 앞서 혼자 통과해 생략.
  - arm_fault 흐름: 14.3 s HELD(ARM_FAULT, 다시 보내지 않음) → 14.9 s ARM_ERROR · 배차 막힘(pode1 TO_WORK → WAIT) → 34.4 s 강제 출고(PARTIAL) → 45.0 s [장애 처리 완료] → 45.2 s pode1 TO_WORK → O-002 SHIPPED.
- (a)~(d) 가짜 시스템 (담기 상품당 2.5 s, 시계 2배, 4판 동시, 도메인 71~74): (a) 42 s, (b) 45 s, (c) 39 s, (d) 39 s 모두 통과. Gazebo (b)는 가짜 (b)와 관제 경로가 같아 뺌.
- 이전 시나리오 전체 (ae6f767e, 도메인 70~79): 12개 중 8 passed, 4 failed.
  - basic_orders: WORK_DOCK을 0.07 m에서 놓음(다른 시험과 함께 돌린 부하 때). 혼자 돌리면 통과(53 s).
  - operator_recovery · auto_isolation · arm_fault: 시나리오 심판이 `ARM_ERROR`(ERROR 사건)를 예상하지 않음, 앞 두 개는 팔 ERROR로 작업대가 막혀 900 s 시간 초과. 변경 전(f4995a73)에도 같은 3개가 실패(D 확인). 시나리오 기대값을 새 규칙(ERROR면 작업대 막힘)에 맞춰 고쳐야 함.
- 팀 브랜치 (ca61c5c2, 도메인 87): fleet_core 1429 passed, 7 xfailed. fleet_manager 22개 파일 모두 통과(node_admin 26, node_arm_rules 11, node_wiring 27, robot_calls 27 등).
- 가짜 팔 시험 (B 브랜치 합친 뒤): test_ros_workcell_rules 21, 그 밖 31 · 12 · 3 passed.

### 바꾼 · 뺀 시험
- `test_a_lock_released_on_a_holding_arm_during_a_fill_cancels_it_and_holds_the_order` → 규칙 5 변경으로 새 규칙 시험 5개로 대체.
- `test_an_arm_state_input_is_recorded`: 마지막 줄 → 헤더 뒤 아무 줄 (입력 뒤에 작업대 막힘 줄이 붙음, 느슨해짐 아님으로 판단).
- `test_the_fill_result_fields_reach_the_arm_report`: 보고 중 ArmFillResult만 골라 봄.
- test_adapters 팔 줄 order_id 기대값: 팔 보고값 → 코어 trip (계약 변경).
- test_node_wiring World가 작업대 두 팔 상태를 냄. `test_the_status_lists_the_three_arms_after_the_podes`는 팔 상태를 내지 않는 World로 그대로 확인(arm_right 상태 없음 → COMM_LOST).
- 팀 레포에는 개인 레포 주문 코어에 기대는 시험(test_arm_fleet_rules · test_fill_pause · test_orchestrator · test_order_hold, 서비스 정지 시험 4개)을 넣지 않고 ArmJobs 단독 시험으로 대신함.

- 시나리오 설정 시험 3개: arm_fail 3번(재시도 2번) → 1번 + [장애 처리 완료] 단계 확인으로 바꿈(규칙 2: ARM_FAULT는 다시 보내지 않음).
- 팀 레포 가짜 작업셀에 `/{arm}/fault_resolved` 서버와 error_until_resolved만 넣음(시나리오가 돌게).

### 느슨해진 기준
- fill_timeout_s 90 → 1200 s (사용자 결정, 통합 시험용).
- PAUSED 동안 fill_timeout을 세지 않음 (사용자 결정).

## 4. 검토 이력 (Codex)
- 1~2번: 7건 중 uncertain 보류 3곳 고침, 3건 건너뜀(시간 초과 뒤 성공은 결과가 있음, 0개 담은 격리 로봇은 다시 대기열, 포기 경로는 닿지 않음).
- core 1~3차: 다른 팔 holding, 재시작 대기와 격리 · 치움 · ERROR, 정지 중 교환 · 빈 로봇, 늦은 uncertain, 다 담긴 뒤 재시작, 줄어든 placed_total, 정지 중 배차 요청 고침. 3차 결함 없음.
- 규칙 5 변경 1~3차: 취소 대기 중 락 해제, 그리퍼 연 뒤 상태, 격리, 목표 경계 고침. 3차 결함 없음.
- 노드 배선(D) 1~2차: 팔 줄 order_id, key 겹침 고침. 틱 사이 첫 상태 창은 사용자 지시로 고침(ae6f767e).
- 노드 + core 합침: 결함 없음.
- 계약 md 1판 8건, 2판 4건 모두 반영.
- fault_resolved · 끊김 창 1차(개인 · 팀): 도착 시각(고침), 보내는 중 관측(고침), 요청 시점 ERROR 판단 · 무음 기간(건너뜀, 위 "내가 정한 것").
- 후속 2차: 한 번도 못 보낸 요청의 관측을 확정에 씀(고침: 실제로 보낸 뒤만), 상태판 시험이 World 팔 상태를 끄기 전에 오른팔 상태가 나갈 수 있음(고침: World를 처음부터 팔 상태 없이 만듦).

## 5. 남은 것
- `/fleet/resume_fill`([담기 재개] · [보충 불가]) 처리 없음 → NOT_IMPLEMENTED. 주문 쪽(통합) 몫.
- FILLING_PAUSE 주문이 주문 보드에서 빠지면 Pode가 묶임.
- 팀 명세 5장 "담기 멈춤" 흐름, 2.1 FILLING 행, 7장 새 서비스 타입 목록은 새 정의와 다 맞추지 않음.
- 팀 레포 가짜 팔은 PI-2A 가짜 관리 기능이 없어 이번 PR에 가짜 팔 변경을 넣지 않음.
