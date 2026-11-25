# 병합 보고서 남은 결정 처리 (2026-11-25)

`docs/report_2026-11-25_merge_main.md` 6절의 1~4. 사용자 결정: "다 추천대로 하는데 4번은 통합 쪽에서 기능을 만들면 해결될 일이니 해결로 둔다."
브랜치 `fix/merge-report-leftovers-2026-11-25`(main `4572a23`에서 출발).

## 바꾼 것

| # | 무엇 | 바꾼 곳 |
|---|---|---|
| 1 | 물러나기(Retreat)가 Nav2 서버가 없어 못 보내면 `NAV_UNAVAILABLE`로 끝나는데 action에 상수가 없었다 | `Retreat.action`에 `REASON_NAV_UNAVAILABLE` 추가. `docs/Pode_계약.md` Retreat Result 줄에 추가하고 `FAILED` 설명에서 "서버 없음"을 뺐다(서버가 없으면 `NAV_UNAVAILABLE`이다) |
| 2 | `/fleet/force_ship` 거절 사유가 계약 `request_rejected`에 없는 코드였다 | `fleet_core/common/commands.py`: 모르는 주문 `UNKNOWN_ORDER` → `NOT_FOUND`, 보류 중이 아님 `NOT_HELD` → `ORDER_STATE`, 로봇 고장 보류 `ROBOT_FAULT` → `NOT_READY`. 계약 파일은 그대로다 |
| 3 | 가짜 로봇 상태 발행 기본 10 Hz, 실물 `pode_fsm` 5 Hz(IR-34) | `fake_pode_node.py` 기본값 5.0 |
| 4 | 관제가 `ReportTripCmd`를 조용히 버림(W-21) | 코드 변경 없음. 통합 쪽에서 ReportTrip 기능을 만들면 풀리므로 해결로 둔다 |

- 보류 사유 `ROBOT_FAULT`(`HOLD_ROBOT_FAULT`, 계약 `hold` 코드)는 그대로다. 바뀐 것은 강제 출고를 거절할 때 돌려주는 사유뿐이다.
- 계약 코드 뜻: `NOT_FOUND` 없는 주문, `ORDER_STATE` 주문 상태가 요청과 맞지 않음, `NOT_READY` 로봇이 요청을 받을 상태가 아님.

## 시험

먼저 시험을 고쳐 실패를 확인한 뒤 코드를 고쳤다.

- 새 시험: `pode_fsm/test/test_contract.py::test_retreat_has_the_nav_unavailable_reason`,
  `fleet_core/test/test_contract.py::test_force_ship_rejections_use_contract_codes`(`FORCE_*` 값이 모두 `request_rejected` 안),
  `fake_robots/test/test_state_rate.py`(가짜 로봇 기본값 = `pode_fsm` 기본값 = 5.0).
- 기대값을 바꾼 시험: fleet_core `test_input_reject.py`, `test_manager.py`, `test_order_hold.py`, fleet_manager `test_adapters.py`, `test_node_wiring.py`.
- 고치기 전: 새 시험 셋과 바꾼 기대값 시험이 실패(pode_fsm 1, fleet_core 6, fake_robots 1).
- 고친 뒤(`nav2fleet_ws/install` 818e60d 위에 이 브랜치 소스를 PYTHONPATH 앞에 둠, 도메인 93~96, LOCALHOST):
  fleet_core 1417 통과(xfail 7), pode_fsm 521 통과(건너뜀 6), fleet_manager 167 통과, fake_robots 1169 통과(건너뜀 1, xfail 30).
  `podedrive_ws/install`은 옛 빌드(`ArmState` 없음)라 ROS 시험이 import에서 실패해 쓰지 않았다.
- Codex 검토: 시험 두 곳이 가짜 응답으로 옛 코드(`ROBOT_FAULT`, `NOT_HELD`)를 넣고 있어 `NOT_READY`, `ORDER_STATE`로 바꿨다
  (fleet_manager `test_node_wiring.py`, fake_robots `test_ros_central.py`. 바꾼 시험 4 · 10 통과).
