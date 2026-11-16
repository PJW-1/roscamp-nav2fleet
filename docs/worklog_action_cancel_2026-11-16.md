# Pode 액션 클라이언트 취소·결과 처리 (2026-11-16)

브랜치 `fix/action-cancel`, 바탕은 origin/main 7c7d04b.

## 1. 결론

- 결함은 `BackUpRunner`(pode/pode_fsm/pode_fsm/backup_ros.py)에만 있었습니다. FollowPath, NavigateToPose 실행기는 새 시험 27개를 고치기 전에도 모두 통과했습니다.
- 세 실행기에서 같은 부분(seq별 핸들, 수락 전 취소, 취소 응답 확인, 결과 future 예외 처리)을 `pode_fsm/action_client.py`의 `GoalRunner`로 모았습니다. FSM에 알리는 방법은 실행기마다 달라서 하위 클래스에 남겼습니다.
- 시험: 새 시험 36개 중 9개가 고치기 전에 실패했고(모두 BackUp), 고친 뒤 36개 모두 통과했습니다. pode_fsm 전체 시험은 425 passed입니다(도메인 66, LOCALHOST).

## 2. 찾은 결함 (모두 origin/main 7c7d04b의 backup_ros.py)

| # | 줄 | 내용 | 실패한 시험 |
|---|---|---|---|
| 1 | :64 `_on_response` | `future.result()` 예외를 잡지 않습니다. 콜백이 죽으면 FSM은 `backup_result`를 받지 못하고, 시간 초과(`allowance + backup_result_grace_s`)까지 기다립니다 | `test_a_goal_whose_send_future_fails_reports_rejected[backup]` |
| 2 | :81 `_on_done` | 결과 future가 예외를 내거나 `None`을 돌려주면(취소된 rclpy future) 콜백이 죽습니다. 후진이 아직 돌 수 있는데 취소도 보내지 않습니다 | `test_a_result_future_with_no_result_counts_as_lost[backup]`, `test_a_lost_backup_result_*` 3개 |
| 3 | :53 `cancel_all` | `cancel_goal_async()` 응답을 보지 않습니다. 거절되거나 응답이 없어도 로그가 남지 않고, 끝난 goal의 핸들은 결과가 올 때까지 남습니다 | `test_a_cancel_that_is_not_confirmed_keeps_the_goal_and_warns[*-backup]` 3개, `test_a_cancel_on_a_finished_goal_is_confirmed[backup]` |
| 4 | :67 | 수락 전에 취소를 요청한 goal이 거절되면 `_cancel_on_accept`에서 지우지 않습니다(작은 누수, 동작 영향 없음) | 직접 시험하지 않음 |

## 3. 세 실행기의 계약

| | FollowPath | NavigateToPose | BackUp |
|---|---|---|---|
| 거절(보내기 future 예외 포함) | `on_rejected(seq)` → 노드가 다음 tick에 `nav_result(seq, "rejected")` | `on_result(seq, "rejected", False)` | `on_result(seq, "rejected", 0, 간 거리)` |
| 결과 | `on_result(seq, outcome, code, False)` | `on_result(seq, outcome, False)` | `on_result(seq, outcome, code, 간 거리)` |
| 결과를 잃음 | 취소를 보내고 바로 `on_result(seq, "aborted", 0, True)` → `nav_lost`(core가 goal을 잊지 않음) | 취소를 보내고 바로 `on_result(seq, "aborted", True)`(`lost=True`면 core가 잊지 않음) | **이번에 바꿈**: 취소를 보내고, 취소가 확인된 뒤에만 `on_result(seq, "aborted", 0, 간 거리)`. 확인되지 않으면 알리지 않고 FSM 시간 초과에 맡김 |
| 취소 수락 | `on_cancel_accepted(seq)` → core가 `nav_result_grace_s` 뒤 잊음 | 같음(`nav_goal_cancel_accepted`) | 콜백 없음. FSM은 결과나 `_backup_unanswered` 시간 초과만 본다 |
| 취소 거절·응답 없음 | 경고 로그, 핸들 유지(다음 `cancel_all`이 다시 취소) | 같음 | **이번에 바꿈**: 같음 |

"취소 확인"은 응답의 `goals_canceling`에 goal이 있거나 `return_code`가 `ERROR_GOAL_TERMINATED`, `ERROR_UNKNOWN_GOAL_ID`일 때입니다(이전 FollowPath, NavigateToPose와 같은 기준).

## 4. 공통화 범위와 남긴 차이

`GoalRunner`(action_client.py)로 모은 것: 액션 상태 이름표(`OUTCOME`), 끝난 goal 응답(`GONE`), `cancel_confirmed`, `_handles`/`_pending`/`_cancel_on_accept` 관리, `cancel_all`, `_send_goal`, `_on_response`, `_on_done`, `_on_cancel`, 잃은 결과의 기본 처리(`_result_lost`: 핸들을 두고 취소, `_lost` 알림).

하위 클래스에 남긴 것과 이유:

- 알림 형식(`_rejected`, `_finished`, `_lost`, `_cancel_accepted`): 노드와 core의 함수 모양이 실행기마다 다릅니다. core를 바꾸지 않으려고 그대로 두었습니다.
- BackUp의 `_result_lost`: core에 BackUp용 `lost` 길이 없습니다. aborted를 바로 알리면 `backup_result`가 `_backup_unanswered`를 지워 후진이 도는 중에 다음 Nav2 goal이 나갈 수 있습니다. 그래서 취소가 확인된 뒤에만 알립니다. 그 전에 이미 취소가 확인된 goal(핸들이 없음)은 바로 알립니다.
- BackUp의 간 거리(`_travelled`)와 feedback, NavigateToPose의 남은 거리 feedback.

동작이 달라진 것(FollowPath, NavigateToPose): 없습니다. 로그 문구도 같습니다. BackUp은 실패 결과에 오류 코드가 있으면 info 로그 한 줄이 늘었습니다(공통 `_on_done`).

## 5. 바꾼 파일

- `pode/pode_fsm/pode_fsm/action_client.py` (새 파일)
- `pode/pode_fsm/pode_fsm/backup_ros.py`, `follow_ros.py`, `nav_goal_ros.py`
- `pode/pode_fsm/test/test_action_runners.py` (새 파일, 가짜 ActionClient와 가짜 future. 노드와 실행기 없이 돈다)

## 6. 시험

fleet_interfaces는 scratchpad에 따로 빌드했습니다(`colcon build --packages-select fleet_interfaces`, 1분 56초). 시험 환경은 `ROS_DOMAIN_ID=66 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, jazzy입니다.

| 시험 | 명령 (pode/pode_fsm에서) | 결과 |
|---|---|---|
| 새 시험, 고치기 전 | `python3 -m pytest -q test/test_action_runners.py` | 9 failed, 27 passed |
| 새 시험, 고친 뒤 | 같음 | 36 passed |
| pode_fsm 전체, 고친 뒤 | `python3 -m pytest -q $(ls test/test_*.py \| grep -v matches_fake)` | 425 passed in 69.17 s (ROS 시험 포함, 건너뜀 0) |

## 7. 실패한 것, 돌리지 않은 것

- worktree 경로에 `.claude`가 있어 `pytest test`(폴더 지정)는 시험을 하나도 모으지 못했습니다(pytest 7.4.4). 파일을 하나하나 넘기면 모입니다. 또 `fake_robots`가 없어 `test_ros_matches_fake_pode.py`가 모듈째 건너뛰면 같이 넘긴 파일들도 모이지 않아(no tests collected) 이 파일은 빼고 돌렸습니다. 이 파일은 원래 `pytestmark = skip`(옛 레인 지도)이라 빠진 시험은 없습니다.
- Gazebo, 실물 Pode, 실제 Nav2 서버로는 돌리지 않았습니다. 취소 거절이나 결과 유실은 가짜 future로만 만들었습니다.
- ruff는 이 컴퓨터에 없어 돌리지 않았고, pyflakes만 돌렸습니다(지적 없음).

## 8. 남은 위험

- BackUp 결과를 잃고 취소도 확인되지 않으면 FSM은 자기 시간 초과(물러나기 `allowance + backup_result_grace_s`, 복구 `backup_time_allowance_s + backup_result_grace_s`)까지 기다립니다. 그때 `TIMEOUT`이나 다음 단계로 넘어가며 `CancelBackUp`을 다시 보냅니다.
- 취소가 `goals_canceling`으로 확인되면 goal은 아직 멈추는 중일 수 있습니다. BackUp은 그 순간 aborted를 알리므로 FollowPath, NavigateToPose처럼 `nav_result_grace_s`만큼 기다리는 장치는 없습니다(core를 바꾸지 않는 범위).
- 보내기 future 예외는 세 실행기 모두 "거절"로 봅니다. 서버가 실제로는 받았다면 핸들이 없어 취소할 수 없습니다(이전 FollowPath, NavigateToPose와 같음).
