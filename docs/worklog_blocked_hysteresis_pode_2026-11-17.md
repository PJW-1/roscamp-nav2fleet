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
