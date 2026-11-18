# 한도에 닿은 로봇이 위치 잃음 한 번에 격리되던 결함 (2026-11-18)

브랜치: `fix/limit-localizing-isolation-2026-11-18` (origin/main 7b3a619에서 시작)

## 결함

- `pc/fleet_core/fleet_core/traffic/zone/results.py:223-225` (`_move_failed`): NAV_ABORTED가 먼저 오면
  `m.health.aborted_at`을 적고 `_goal_failed`를 부른다.
- `pc/fleet_core/fleet_core/traffic/zone/base.py:196-201` (`_goal_failed`, 7b3a619 기준): `attempts`를 하나 올리고,
  `move_retry_limit`을 넘으면 그 자리에서 `_fail` → `ROBOT_ISOLATED`(fault=ERROR).
- `base.py:132-140` (`_note_localizing`): 1 s(`LOCALIZING_GRACE_S`) 안에 LOCALIZING이 오면 시도를 되돌리지만
  `not m.health.fault` 조건 때문에 이미 격리된 로봇은 건너뛴다.
- 결과: 한도 아래에서는 "NAV_ABORTED → 1 s 안에 LOCALIZING"이 시도로 세지 않는데, 한도에 닿은 로봇은 같은
  순서로 위치를 한 번 잃으면 격리된다.
- 실제 로봇: 2026-11-18에 NAV_ABORTED와 LOCALIZING이 함께 온 일이 두 번 있었다(두 로봇 동시, 8-10 s, PC에서 RViz를
  띄운 때와 겹침). 출처: 이 작업을 맡긴 세션의 설명. 이 브랜치에서 로그로 다시 확인하지 않았다.

### 기존 시험이 못 잡은 까닭

`test_at_the_retry_limit_a_nav_aborted_while_localizing_does_not_isolate`는 한도까지 채운 뒤 **LOCALIZING을 먼저**
보고하고 NAV_ABORTED를 보낸다. 이 순서는 `_lost_pose_lately`(results.py:213)가 받아 시도로 세지 않으므로 통과한다.
결과가 먼저 오는 순서는 한도 아래(`test_nav_aborted_heard_just_before_the_localizing_report_gives_the_attempt_back`)에서만
시험했다.

## 고친 내용 (커밋 2c6288a, 리뷰 반영 후속 커밋)

- 한도를 넘기는 NAV_ABORTED(`_goal_failed(..., grace=True)`, results.py에서만)는 바로 격리하지 않고
  `health.final_abort = (지금 + LOCALIZING_GRACE_S, 사유)`로 미루고 `GOAL_FAIL_PENDING`(WARN)을 낸다.
  시도 수는 그대로 센다.
- 그 안에 LOCALIZING이 오면 기존 `_note_localizing`이 시도를 되돌리고(`GOAL_ABORTED_LOCALIZING`) 미룬 판정을 지운다.
  후속 커밋: 그새 배차 취소 등으로 시도 수가 0이 됐어도 판정은 지운다(2c6288a에서는 `attempts > 0` 조건 때문에
  남아 격리됐다. Codex 2c6288a 리뷰 지적 1, 확인함).
  위치를 다시 보고하면 goal을 다시 보낸다(한도 아래와 같다).
- 오지 않으면 `tick_progress`의 `_settle_final_abort`가 기간이 지난 첫 tick에 `GOAL_FAILED` + `_fail`로 격리한다.
  그새 다른 고장(`_fail`)이 났으면 그 사유를 덮지 않는다(`_fail`이 판정을 지운다). 그새 배차 취소 등으로
  시도 수가 0이 돼도 격리한다(예전과 같은 결과).
- 판정 중에는(Codex 계획 리뷰 지적을 확인하고 반영):
  - goal을 보내지 않는다: `drive._step_phase`, step을 거치지 않는 `watch._to_parking`(교착 비키기, 양보).
  - 열린 호출 후보가 아니고(`calls._robot_info`), 지정 배차는 `NOT_AVAILABLE`로 거절한다(`calls.on_dispatch`).
  - `_release_behind`로 구역·노드를 놓지 않는다(`progress.on_robot_state`).
- 한도 아래의 재시도는 바뀌지 않는다. 늦어지는 것은 한도를 넘는 NAV_ABORTED의 격리뿐이고 1 s가 지난 뒤 첫 `tick_progress`에서 정해진다(tick이 늦어지면 그만큼 늦다).
- 동작이 바뀐 기존 시험 하나: 위 기존 시험의 마지막 "다음 진짜 실패는 격리" 단언이 한 step 뒤가 아니라
  `LOCALIZING_GRACE_S + 0.1` s 뒤로 바뀌었다(한 step 뒤에는 아직 격리하지 않는다는 단언을 더했다).

## 시험 (pc/fleet_core/test/test_zone_wait_localizing.py)

새 시험:

- `test_at_the_retry_limit_nav_aborted_heard_just_before_localizing_does_not_isolate`: 한도에서 NAV_ABORTED, 0.5 s 뒤
  LOCALIZING → 격리 없음, 시도 수 한도 그대로, LOCALIZING 동안 goal 없음, 위치 보고 뒤 다시 보냄.
- `test_at_the_retry_limit_nav_aborted_without_localizing_still_isolates_after_the_grace`: LOCALIZING이 없으면 1.3 s 안에
  격리, 그 사이 goal 없음.
- `test_at_the_retry_limit_a_localizing_report_after_the_grace_still_isolates`: 1.01 s 뒤의 LOCALIZING은 격리를 막지 않는다.
- `test_at_the_retry_limit_nothing_is_released_or_sent_in_the_grace_window`: 판정 중 구역·노드·goal 그대로.
- `test_at_the_retry_limit_no_side_road_sends_a_goal_in_the_grace_window`: `_evade`가 한도 아래에서는 실제로
  보내는 자리에서, 판정 중에는 보내지 않는다.
- `test_at_the_retry_limit_a_new_leg_in_the_grace_window_still_isolates`: 판정 중 배차 취소로 시도 수가 0이 돼도 격리.
- `test_at_the_retry_limit_a_named_dispatch_in_the_grace_window_is_refused`: 판정 중 지정 배차는 `NOT_AVAILABLE`.
- `test_a_fault_in_the_grace_window_keeps_its_own_reason`: 판정 중 `on_robot_fault("ESTOP")`이면 격리 사유는 ESTOP 하나.
- (후속) `test_at_the_retry_limit_a_new_leg_then_localizing_in_the_grace_window_does_not_isolate`: 판정 중 배차 취소로
  시도 수가 0이 된 뒤 1 s 안에 LOCALIZING → 격리 없음.

### 결과 (`cd pc/fleet_core && python3 -m pytest -q`)

- 7b3a619 그대로(깨끗한 사본): `1106 passed, 7 xfailed in 61.30s`
- 첫 시험 네 개만 더하고 고치기 전(`-k retry_limit`의 "3 passed"에는 기존 시험 하나가 들어 있다): `2 failed, 1108 passed, 7 xfailed in 58.02s`
  - FAILED `test_at_the_retry_limit_nav_aborted_heard_just_before_localizing_does_not_isolate`
    (`AssertionError: assert (not [RobotIsolated(robot='pode2', reason='NAV_ABORTED')])`)
  - FAILED `test_at_the_retry_limit_nothing_is_released_or_sent_in_the_grace_window`
    (`assert not [RobotIsolated(robot='pode2', reason='NAV_ABORTED')]`)
- 첫 수정 뒤 이 파일: `1 failed, 28 passed` (위 기존 시험의 즉시 격리 단언 → 의도한 변화라 단언을 고침)
- 리뷰 반영 시험 넷을 더하고 고치기 전(이 파일만): `3 failed, 30 passed`
  (side_road, new_leg, named_dispatch. fault 사유 시험은 처음부터 통과)
- 커밋 2c6288a: `1114 passed, 7 xfailed in 43.89s`
- 후속 시험 하나를 더하고 고치기 전(이 파일만): `1 failed, 33 passed`
  (FAILED `..._a_new_leg_then_localizing_in_the_grace_window_does_not_isolate`,
  `assert not [RobotIsolated(robot='pode2', reason='NAV_ABORTED')]`)
- 후속 수정 뒤: `1115 passed, 7 xfailed in 47.91s`

## 확인하지 못한 것

- 실제 로봇이나 Gazebo에서 돌려 보지 않았다. 새 시나리오는 zone_harness로만 시험했다(전체 pytest에는
  FleetManager를 직접 만드는 시험 등도 들어 있다).
- 열린 호출 후보에서 빠지는 것(`_robot_info`)과 양보 경로(`_yield_to_robot` → `_to_parking`) 전체는 직접 시험하지
  않았다. `_evade`만 불러 확인했다. 한도 시험에서 정확히 1.0 s 경계도 시험하지 않았다.
- `_watch_retreat_wait`(yard_retreat.py:73)는 물러나기 대기가 `blocked_timeout_s`를 넘으면 LOCALIZING이든 판정
  중이든 BLOCKED로 바로 격리한다. 이번 결함 전부터 있던 동작이고(LOCALIZING을 보지 않음) 이번에 고치지 않았다.
- 실제 로봇에서 NAV_ABORTED와 LOCALIZING 사이 간격이 1 s 안인지는 이번에 재지 않았다. 1 s보다 벌어지면 이 수정으로도
  격리된다(`LOCALIZING_GRACE_S`는 그대로 두었다).
- 판정 중 통신이 끊기면(COMM_LOST) 1 s가 지난 뒤 첫 tick에서 격리된다. 예전에는 바로 격리됐으므로 결과는 같지만 따로 시험하지 않았다.
- 판정 중 `confirm_clear`(OUT_OF_SERVICE)는 fault라 덮지 않는다고 코드로만 확인했다(시험 없음).
