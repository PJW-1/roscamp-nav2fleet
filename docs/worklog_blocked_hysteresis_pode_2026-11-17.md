# 작업 기록: Pode BLOCKED 해제 조건 강화 (2026-11-17)

브랜치 `fix/blocked-hysteresis-pode` (main 956140d에서). 로봇 쪽(pode_fsm)만 고쳤다. 관제 쪽 10초
BLOCKED 시계가 틈마다 다시 시작하는 문제는 다른 브랜치가 고친다(pc/는 건드리지 않음).

## 1. 문제

좁은 곳에서 로봇이 보고하는 상태가 `BLOCKED`와 `MOVING`(가끔 `RECOVERING`) 사이를 오갔다.
`BLOCKED`가 너무 쉽게 풀렸기 때문이다.

## 2. 확인한 주장과 고친 주장 (main 956140d 기준 file:line)

| 주장 | 결과 |
|---|---|
| 로봇단 교착 처리는 기본 켬(`local_yield=True`) | 맞음. `params.py:55` |
| 약 1초 서면 `RECOVERING`, 2초 기다린 뒤 물러나지 않는 쪽은 `BLOCKED` | 맞음. nav_goal: 진전(0.05 m) 없이 1초(제자리 회전 창 4초가 시작을 늦춤) `yield_nav.py:28-32`, 2초 뒤 후진 거리 0이면 BLOCKED `yield_nav.py:43-49`. follow_path: 물체가 보이는 채 1초 섬 `obstacle_watch.py:41-42`, 2초 뒤 `yield_moves.py:59-64` |
| `blocked_after_s` 10초가 상한 | 맞음. `params.py:51`, nav_goal `yield_nav.py:38`, `nav_goal.py:105-106`. follow_path는 그때 물체가 보여야 함 `yield_moves.py:46` |
| nav_goal에서 5 cm 움직이면 바로 풀림 | 맞음. `nav_goal.py:100-101`(`nav_progress_m` 0.05, `params.py:52`) |
| follow_path에서 물체가 1초 안 보이면 풀림 | 맞음. `obstacle_watch.py:50-54`(`block_clear_s` 1.0) |
| 둘 다 참이면 `RECOVERING`이 `BLOCKED`보다 먼저 보고됨 | 코드 순서는 맞음(`core.py:103-106`). 다만 실제로 둘이 함께 서지 않는다: 복구를 시작할 때 막힘을 끄고(`recovery.py:41`, `yield_moves.py:35`), nav_goal은 막힌 동안 복구를 시작하지 않는다(`yield_nav.py:29`). 깜빡임의 원인은 순서가 아니다 |
| 그래서 MOVING / RECOVERING / BLOCKED가 번갈아 나옴 | 대체로 맞음. 고칠 점: 로봇단 교착 처리는 한 자리에서 한 번뿐이고(`recovery.py:19`, 0.3 m 벗어나야 다시) 그래서 보통은 `BLOCKED`↔`MOVING`이다. nav_goal은 풀린 뒤 다시 BLOCKED가 되려면 `blocked_after_s` 10초를 다시 기다려야 해 `MOVING`이 길게(최대 약 10초) 끼고, follow_path는 다시 1초 서면 바로 BLOCKED라 `MOVING`이 약 1초 낀다. `RECOVERING`은 첫 시도 자리에서 0.3 m 넘게 움직였을 때만 낀다 |

## 3. 규칙 (켜는 조건은 그대로, 끄는 조건만 바꿈)

> 첫 커밋 d1d9ac8의 규칙이다. Codex 리뷰 뒤 4f87eab에서 "판단용 막힘은 옛 규칙, 보고만 늦춤"으로
> 바꿨다. 지금 규칙은 8절을 본다. 3, 4절은 d1d9ac8 기준 기록으로 둔다.

- **nav_goal**: `BLOCKED`를 켠 자리를 기억한다. 로봇 중심이 그 자리에서 `unblock_m`(0.15 m) 이상
  벗어나야 끈다. 진전(0.05 m)은 그대로 `blocked_after_s`의 시계와 "섬" 판정만 다시 잡는다.
  goal을 다시 보내거나(`NavGoals.send`) 목표가 끝나면 끈다(전과 같음).
- **follow_path**: 물체가 `block_clear_s`(1초) 안 보이면 기다리던 FollowPath를 **바로 다시 보낸다**
  (전과 같음. 성능 우선이라 주행은 늦추지 않음). `BLOCKED`는 물체가 `unblock_clear_s`(3초) 동안
  계속 안 보이거나, 막힌 자리에서 `unblock_m`(0.15 m) 이상 벗어나야 끈다. 중간에 물체가 다시
  보이면 3초를 처음부터 센다.
- 목표를 취소한 뒤 남은 `BLOCKED`(옛 경로 앞 물체를 보는 것)도 같은 규칙이고, 0.15 m 벗어나면
  (예: 물러나기 goal) 끈다. 전에는 옛 경로 앞 물체가 보이는 한 물러나는 동안에도 `BLOCKED`였다.
- 거리는 경로 위 진행이 아니라 막힌 자리로부터의 직선 거리다. 두 모드를 같은 식으로 보고,
  물러나기처럼 경로 밖으로 움직이는 경우도 해제로 친다.

### 값과 이유 (모두 추정)

| 이름 | 값 | 이유 |
|---|---|---|
| `unblock_m` | 0.15 m | Nav2 progress checker 0.05 m의 3배. 다시 계획하며 꿈틀대는 몇 cm(시험은 최대 0.10 m)보다 크고, 시도 수를 되돌리는 `recovery_reset_m` 0.3 m보다 작다. nav_goal에서는 `nav_progress_m`보다 작게 두어도 `nav_progress_m`을 쓴다 |
| `unblock_clear_s` | 3.0 s | 라이다 잡음으로 물체가 1초쯤 사라지는 것을 넘기고, 관제 `blocked_mark_s` 10초보다 충분히 짧다 |

`params.py`의 `FsmParams`와 `params_ros.py`의 ROS 파라미터 선언에 넣었다(launch는 따로 넘기지 않으므로
기본값을 쓴다).

## 4. BLOCKED에 묶인 로봇 동작 (움직이는 로봇을 세우지 않는지)

- BLOCKED는 주행 명령을 막지 않는다. nav_goal은 막힌 동안에도 NavigateToPose가 계속 돈다.
  follow_path는 1초 뒤 경로를 다시 보내는 때를 바꾸지 않았다.
- 막힌 동안에는 로봇단 교착 처리나 자가 복구를 새로 시작하지 않는다(`yield_nav.py:29`,
  `obstacle_watch.py:41`, `core.py:216`의 aborted 처리). 막힘이 더 오래 남으니 이 상태도 더 오래
  간다. 다만 기본값(`local_yield` 켬)에서는 한 자리에서 한 번만 하므로 전에도 같은 자리에서 다시
  시작하지 못했다. `local_yield`를 끈 때(`recovery_attempts` 2)는 잡음으로 풀린 뒤 두 번째 시도를
  하던 경우가 이제 막힘으로 남는다.
- `comm_loss_policy: base_end`(기본은 `stop`)에서는 막힌 로봇이 통신이 끊기면 선다
  (`order_fsm.py:170`). 막힘이 길어진 만큼 이 경우에 서는 일이 조금 늘 수 있다.
- 노드는 BLOCKED 동안 앞 물체 보고를 붙인다(`node.py:398`). 보고가 더 오래 나온다.

## 5. 가짜 로봇 (sim/fake_robots)

(d1d9ac8 때 기록. 그 뒤 nav_goal 가짜는 맞췄다: 8절)

손대지 않았다.
- nav_goal 가짜(`world/nav_goal.py:70-73`, `nav_goal_drive.py:58-59`)는 자기 규칙(진전 0.02 m, 복구 없음)이고
  Pode의 해제 규칙을 따라 하지 않는다. `test_timing_agreement.py`는 `blocked_after_s`만 맞춘다.
- follow_path 가짜(`recovery.py:79-93`, `fake_pode_node.py`가 씀)는 Pode의 옛 1초 해제를 따라 한다.
  그러나 follow_path 모드는 11-13부터 쓰지 않고, 비교 시험(`test_ros_matches_fake_pode.py`)은 건너뛰며,
  이것을 바꾸면 도메인 50~59를 쓰는 fleet_manager 시험에 영향을 줄 수 있어 이번에는 두었다. 맞추려면
  `ObstacleRecovery.watch`에서 `_hold`(다시 감)는 1초, `blocked`는 3초 또는 0.15 m로 나누면 된다.

## 6. 시험

새 시험 `pode/pode_fsm/test/test_blocked_hysteresis.py`(7개):
- nav_goal: BLOCKED 뒤 최대 0.10 m로 꿈틀대도 계속 BLOCKED, 0.16 m 가면 MOVING, `unblock_m`이 파라미터.
- follow_path: 물체가 1.2초 안 보였다 다시 보여도 계속 BLOCKED이고 경로는 다시 보냄, 3초 안 보이면
  MOVING, 0.17 m 나아가면 3초 전에도 MOVING, 취소 뒤 남은 BLOCKED는 3초 뒤 IDLE.

고친 전 결과(새 시험만):
```
7 failed in 0.08s
(꿈틀댐: [(8.3, 'MOVING'), (8.4, 'MOVING'), (8.5, 'MOVING')]
 1.2초 틈: [(5.2, 'MOVING'), (5.3, 'MOVING'), (5.4, 'MOVING')])
```
고친 뒤 기존 시험 3개가 옛 규칙(1초, 5 cm)을 적고 있어 새 규칙으로 고쳤다:
`test_core.py::test_blocked_goal_survives_a_nav_abort_and_resumes_when_clear`,
`test_core.py::test_blocked_stays_after_cancel_while_the_obstacle_remains`,
`test_nav_goal.py::test_moving_on_clears_blocked`.

고친 뒤:
- ROS 없이: `361 passed, 8 skipped`(건너뛴 8개는 rclpy 등이 없어서).
- ROS 포함(`ROS_DOMAIN_ID=74 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, jazzy + 작업공간 install,
  worktree의 pode_fsm을 PYTHONPATH 앞에): `462 passed, 6 skipped`(6개는 레인 지도용이라 원래 건너뜀).
- sim: `test_timing_agreement.py`, `fake_robots/test/test_recovery.py`, `test_world_nav_goal.py`
  `37 passed`.

## 7. 실물에서 볼 것

- 좁은 곳에서 `/poden/state`가 BLOCKED 동안 MOVING으로 잠깐씩 바뀌지 않는지.
- 실제로 빠져나간 로봇이 0.15 m 안에 MOVING으로 돌아오는지(관제가 늦게 풀린 BLOCKED로 길을
  표시하거나 goal을 거두지 않는지). nav_goal은 진전만으로는 안 풀리므로, 0.15 m 안에서 오래 비비다
  빠져나가는 경우 BLOCKED가 그만큼 더 길다.
- AMCL 위치가 0.15 m 넘게 튀면 BLOCKED가 잘못 풀릴 수 있다(다시 켜지려면 nav_goal은 10초).
- follow_path에서 물체가 사라진 뒤 1초에 다시 출발하고 BLOCKED는 3초까지 남는지.

## 8. Codex 리뷰 뒤 고침 (4f87eab, 그다음 커밋)

d1d9ac8 리뷰(`codex_blocked_pode.log`)의 [확정] 4건을 코드에서 다시 확인했다.

| 지적 | 확인 | 처리 |
|---|---|---|
| 1. follow_path: 1초 뒤 경로를 다시 보내고도 막힘을 3초까지 쥐어서, 그 사이 FollowPath가 물체 없이 `aborted`면 복구 대신 `NAV_ABORTED` | 맞음(`core.py:216`의 `not self._blocked`). 시도 수가 되돌아간 경우에만 옛 코드와 결과가 다름 | 판단용 `_blocked`는 옛 규칙(1초)으로 되돌림. 보고용 hold(`_hold_at`)를 따로 둠 |
| 2. follow_path: 거리로 풀린 같은 tick에 옛 정지 시각으로 다시 막힘 | 맞음 | 거리는 이제 hold만 끈다. 다시 막히는 것은 옛 규칙의 판단이다. 처음에는 정지 시각도 지웠는데, 판단이 바뀐다는 2차 리뷰 지적에 따라 되돌림 |
| 3. nav_goal: 거리로 풀린 뒤 오래된 진전 자리 때문에 같은 tick에 다시 막힘 | 맞음(`nav_goal.py` watch 순서) | `g.blocked`는 옛 규칙(0.05 m 진전에서 해제)으로 되돌림. 보고는 `g.held_at`으로 따로 둠. 그 시나리오에서 10초 넘게 0.05 m를 못 나아갔다면 옛 10초 규칙으로 BLOCKED가 맞다(시험으로 고정) |
| 4. 가짜가 계약과 다름 | 맞음 | nav_goal 가짜 둘을 맞춤(아래). follow_path 가짜는 둠 |

### 지금 규칙

- 판단용 막힘(교착 처리 시작, 자가 복구, `aborted` 처리, 통신 끊김 `base_end`)은 main과 같다.
  - follow_path `_blocked`: 물체가 1초 안 보이면 풀림.
  - nav_goal `g.blocked`: 0.05 m 진전이면 풀림.
- 보고(`state()`)만 BLOCKED로 남는다.
  - nav_goal: `g.held_at`. 막힘을 켤 때 그 자리를 잡고, 막힘이 풀린 뒤 그 자리에서 `unblock_m` 0.15 m 벗어나면 끈다. goal을 다시 보내거나 교착 처리를 시작하거나 목표가 끝나면 함께 끈다.
  - follow_path: `_hold_at`. 1초 뒤 막힘이 풀릴 때 잡고, 막힌 자리에서 0.15 m 벗어나거나 물체가 `unblock_clear_s` 3초 동안 계속 안 보이면 끈다(경로를 달리는 중에는 물체가 다시 보이면 다시 센다. 목표가 없으면 시간으로만 끝낸다). 다시 1초 서서 막히거나 복구를 시작하면 hold는 버린다.
- 2차 리뷰(`codex_blocked_pode2.log`, d1d9ac8..4f87eab)에서 확인한 것:
  - 정지 시각을 지우던 것: 판단을 바꿔서 되돌림(위 2번).
  - 계약의 "끌 때 진전 자리와 시각도 새로 잡는다": 틀린 문장이라 고침.
  - 목표 취소 뒤 hold가 시간으로만 끝나는 것: 의도한 동작이고 계약에 적음(main은 1초 뒤 바로 IDLE이었고 물체를 다시 보지 않았다).
  - 취소 뒤 물러나는 동안 옛 경로 앞 물체 때문에 BLOCKED가 남는 것: main과 같은 동작이다(d1d9ac8은 거리로 풀었지만 판단을 바꾸므로 되돌림).

### 가짜 로봇

- `world/nav_goal.py`(`NavGoalRun`)와 `nav_goal_drive.py`(`NavGoalDrive`): 판단용 정체(진전 0.02 m, 10초)는 그대로 두고 보고용 `held_at`을 더했다. `UNBLOCK_M = 0.15`이고 `test_timing_agreement.py`가 `FsmParams().unblock_m`과 같은지 본다.
- 기존 시험 하나를 고쳤다: `test_nav_goal_drive.py::test_a_robot_standing_on_the_goal_blocks_after_ten_seconds`는 11초까지 돈 뒤 `blocked(8.0)`처럼 지난 시각을 물었다. 보고용 막힘이 상태로 남으므로 8초에서 먼저 확인하고 이어 달리게 바꿨다.
- follow_path 가짜(`recovery.py`)는 쓰지 않는 모드라 옛 1초 해제를 그대로 둔다(주석 남김).

### 시험

- 새 Pode 시험(4f87eab 앞, d1d9ac8 위에서 실패 확인):
```
FAILED test_follow_path_the_hold_only_reports_and_an_abort_after_resend_may_still_recover
  AssertionError: [Finish(goal_id='g1', success=False, reason='NAV_ABORTED', last_done_seq_id=0), CancelNav()]
FAILED test_follow_path_after_moving_off_a_new_stop_is_counted_from_zero   (뒤에 판단을 바꾸지 않는 시험으로 바꿈)
FAILED test_nav_goal_clearing_on_distance_does_not_block_again_in_the_same_tick   (10.5 s 시나리오)
3 failed, 7 passed
```
- 취소 뒤 hold가 시간으로 끝나는 시험도 먼저 실패를 보고(`'BLOCKED' == 'IDLE'`) 고쳤다.
- 가짜 새 시험 2개는 고치기 전 실패(`AssertionError: 0.06`).
- 결과는 아래 9절.

## 9. 시험 결과 (Codex 리뷰 뒤)

- pode_fsm, ROS 없이: `366 passed, 8 skipped`.
- pode_fsm, ROS 포함(도메인 74, LOCALHOST): `467 passed, 6 skipped in 70.34s`.
- sim/fake_robots/test 전체와 `test_timing_agreement.py`(도메인 74, LOCALHOST):
  - 가짜를 고치기 전: `1023 passed, 27 xfailed in 762.23s`.
  - 고친 뒤: `1026 passed, 27 xfailed in 788.46s`(새 시험 3개. 실패한 시험이나 결과가 바뀐 시험은 없다. 위의 지난 시각을 묻던 시험 하나는 고쳐 썼다).
- fleet_manager와 pc/ 시험은 도메인 50~59를 써서 돌리지 않았다. 가짜의 BLOCKED가 길어진 만큼 관제 시나리오
  시험의 시간이 달라질 수 있으니 그쪽 담당이 돌려 봐야 한다.

## 10. 3차 리뷰(4f87eab..e935998) 뒤

- [확정] `NavGoalRun`이 끝에서 돌기 시작하면 보고용 막힘을 지웠다: 끝 회전 중에도 `held_at`을 두게 고침
  (회전은 정체 시계만 다시 잡는다). 시험 `test_turning_at_the_end_keeps_the_blocked_report_until_the_goal_ends`가
  고치기 전 실패(`assert (False)`), 고친 뒤 통과.
- [확정] 계약의 nav_goal "끔"에 교착 처리 시작(`RECOVERING`)으로 끄는 경우가 빠져 있었다: 적음.
