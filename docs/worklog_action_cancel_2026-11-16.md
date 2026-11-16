# Pode 액션 클라이언트 취소·결과 처리 (2026-11-16)

브랜치 `fix/action-cancel`, 바탕은 origin/main 7c7d04b. 커밋 f75977e(첫 수정), 그 뒤 Codex 리뷰 지적을 고친 후속 커밋(9절).

## 1. 결론

- 결함은 `BackUpRunner`(pode/pode_fsm/pode_fsm/backup_ros.py)에만 있었습니다. FollowPath, NavigateToPose 실행기는 새 시험 27개를 고치기 전에도 모두 통과했습니다.
- 세 실행기에서 같은 부분(seq별 핸들, 수락 전 취소, 취소 응답 확인, 결과 future 예외 처리)을 `pode_fsm/action_client.py`의 `GoalRunner`로 모았습니다. FSM에 알리는 방법은 실행기마다 달라서 하위 클래스에 남겼습니다.
- 시험: 새 시험 36개 중 9개가 고치기 전에 실패했고(모두 BackUp), 고친 뒤 36개 모두 통과했습니다. pode_fsm 전체 시험은 425 passed입니다(도메인 66, LOCALHOST).
- 후속(9절): Codex 리뷰가 첫 수정의 BackUp 잃은 결과 처리가 "취소 중"을 "끝남"으로 본다는 것을 찾았습니다. 이제 goal이 끝난 것이 확인될 때만 알립니다. future를 돌려주기 전에 나는 예외도 처리합니다. 새 시험 51 passed, 전체 440 passed.

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
| 결과를 잃음 | 취소를 보내고 바로 `on_result(seq, "aborted", 0, True)` → `nav_lost`(core가 goal을 잊지 않음) | 취소를 보내고 바로 `on_result(seq, "aborted", True)`(`lost=True`면 core가 잊지 않음) | **이번에 바꿈**: 취소를 보내고, 취소 응답이 `ERROR_GOAL_TERMINATED`/`ERROR_UNKNOWN_GOAL_ID`(goal이 이미 끝남)일 때만 `on_result(seq, "aborted", 0, 간 거리)`. `goals_canceling`(취소 중), 거절, 무응답이면 알리지 않고 FSM 시간 초과에 맡김(9절에서 고침) |
| 취소 수락 | `on_cancel_accepted(seq)` → core가 `nav_result_grace_s` 뒤 잊음 | 같음(`nav_goal_cancel_accepted`) | 콜백 없음. FSM은 결과나 `_backup_unanswered` 시간 초과만 본다 |
| 취소 거절·응답 없음 | 경고 로그, 핸들 유지(다음 `cancel_all`이 다시 취소) | 같음 | **이번에 바꿈**: 같음. `goals_canceling`도 핸들을 지우지 않음(9절) |
| 보내기·결과 요청·취소 호출 자체의 예외(9절) | 보내기: `send()`가 False → 노드가 다음 tick에 거절로. 결과 요청: 잃은 결과와 같음. 취소 호출: 경고, 핸들 유지 | 같음 | 같음(결과 요청 예외는 BackUp의 잃은 결과 처리를 따름) |

"취소 확인"은 응답의 `goals_canceling`에 goal이 있거나 `return_code`가 `ERROR_GOAL_TERMINATED`, `ERROR_UNKNOWN_GOAL_ID`일 때입니다(이전 FollowPath, NavigateToPose와 같은 기준).

## 4. 공통화 범위와 남긴 차이

`GoalRunner`(action_client.py)로 모은 것: 액션 상태 이름표(`OUTCOME`), 끝난 goal 응답(`GONE`), `cancel_confirmed`, `_handles`/`_pending`/`_cancel_on_accept` 관리, `cancel_all`, `_send_goal`, `_on_response`, `_on_done`, `_on_cancel`, 잃은 결과의 기본 처리(`_result_lost`: 핸들을 두고 취소, `_lost` 알림).

하위 클래스에 남긴 것과 이유:

- 알림 형식(`_rejected`, `_finished`, `_lost`, `_cancel_accepted`): 노드와 core의 함수 모양이 실행기마다 다릅니다. core를 바꾸지 않으려고 그대로 두었습니다.
- BackUp의 `_result_lost`와 `_cancel_confirmed`: core에 BackUp용 `lost` 길이 없습니다. aborted를 바로 알리면 `backup_result`가 `_backup_unanswered`를 지워 후진이 도는 중에 다음 Nav2 goal이 나갈 수 있습니다. 그래서 goal이 끝난 것(GONE 응답)이 확인된 뒤에만 알립니다. 핸들은 결과나 GONE 응답 때만 지우므로, 핸들이 없으면 이미 끝난 goal이라 바로 알립니다.
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
- (9절에서 해소) 첫 수정은 `goals_canceling` 순간 BackUp aborted를 알렸습니다.
- BackUp 결과를 잃으면 대개 FSM 시간 초과까지 기다립니다(속도보다 안전). Nav2가 취소 뒤 다음 취소에 `ERROR_GOAL_TERMINATED`로 답하면 그때 알립니다.
- 보내기 future 예외는 세 실행기 모두 "거절"로 봅니다. 서버가 실제로는 받았다면 핸들이 없어 취소할 수 없습니다(이전 FollowPath, NavigateToPose와 같음).

## 9. 후속: Codex 리뷰(f75977e) 지적 수정

리뷰 로그: `/tmp/claude-1000/-home-ane-ros2camp/255eff68-4c77-42b3-b9fc-325ae2037b61/scratchpad/codex_action.log`. 두 지적 모두 코드를 읽어 직접 확인했습니다.

1. [높음, 확정] f75977e의 backup_ros.py:71: 결과를 잃은 뒤 취소 응답이 `goals_canceling`이면 바로 aborted를 알렸습니다. `goals_canceling`은 취소 중일 뿐입니다. core.py:203이 `_backup_unanswered`를 지워 후진이 도는 중에 다음 동작이 나갈 수 있었습니다. 7c7d04b는 이때 콜백이 죽어 FSM 시간 초과까지 기다렸으니, 첫 수정이 오히려 덜 안전했습니다. 결과를 잃기 전에 `goals_canceling`을 받아 핸들을 지운 경우(:66)도 같았습니다.
   - 고침: 공통 `_on_cancel`이 `gone`(GONE 응답 여부)을 넘기고, BackUp의 `_cancel_confirmed`는 `gone`일 때만 핸들을 지우고 알립니다.
   - 시험: `test_a_lost_backup_result_is_not_reported_while_the_cancel_is_only_in_progress`, `test_a_backup_cancel_in_progress_keeps_the_handle`(고치기 전 실패), `test_a_lost_backup_result_is_reported_once_the_goal_is_known_to_have_ended[*]`, `test_a_lost_backup_result_after_the_goal_was_known_to_have_ended_is_reported_at_once`.
2. [중간, 확정] action_client.py의 `send_goal_async`, `get_result_async`, `cancel_goal_async`가 future를 돌려주기 전에 예외를 내면 처리하지 않았습니다(7c7d04b의 세 래퍼에도 있던 공백).
   - 고침: 보내기 예외 → `send()`가 False(노드가 다음 tick에 거절로 넘김. 노드 잠금 안에서 FSM을 다시 부르지 않으려고 콜백 대신 이 길을 씀). 결과 요청 예외 → 잃은 결과 처리. 취소 호출 예외 → 경고, 핸들 유지.
   - 시험: `test_a_send_call_that_raises_returns_false[*]`, `test_a_result_request_that_raises_counts_as_a_lost_result[*]`, `test_a_cancel_call_that_raises_warns_and_keeps_the_goal[*]`, `test_a_lost_result_whose_cancel_call_raises_is_still_reported[*]`(모두 고치기 전 실패).

| 시험 | 결과 |
|---|---|
| test_action_runners.py, 고치기 전 | 14 failed, 37 passed |
| test_action_runners.py, 고친 뒤 | 51 passed |
| pode_fsm 전체(matches_fake 제외), 고친 뒤 | 440 passed in 68.47 s, 다시 돌려 440 passed in 68.89 s |

첫 전체 실행 끝에 rclpy의 "The following exception was never retrieved: cannot use Destroyable because destruction was requested"가 두 줄 나왔고, 두 번째 실행에는 없었습니다. 시험 종료 때 노드를 없애는 중의 소리로 보이지만 원인은 확인하지 않았습니다.
