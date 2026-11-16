# 작업 기록: ROS 가짜 Pode가 order goal을 달린다 (2026-11-16)

브랜치 `fix/fake-pode-order` (origin/feat/order-default-on-2 5eecfd1에서). 푸시와 병합은 하지 않았다.

## 1. 결론

- `pc.launch.py use_fake:=true`와 `fakes.launch.py`가 띄우는 ROS 가짜 Pode(`sim/fake_robots/fake_robots/`)가 MoveTo의 order 칸을 읽고 달린다.
  그래서 시나리오 시험이 기본값(order 켬) 길을 시험한다. use_fake에서 order를 끄지 않았다(조정자 결정).
- 마지막 코드(3b853be)에서 order 켬 시나리오 9개가 모두 통과했다(11 passed: 시나리오 9 + 순수 2). basic_orders는 order 끔으로도 통과했다.
- `test_fms_restart.py`는 order 켬에서 3번 가운데 2번 실패했다. 재시작 검사는 모두 통과했고, 마지막 판정에서 구역을 일찍 놓은 것 2건이 걸렸다.
  같은 시험을 order 끔으로 돌리면(스크래치 사본) 통과한다. 원인은 관제 쪽 order 진행 계산으로 보인다(7절 첫 항목). 가짜 Pode 문제가 아니라고 판단해 고치지 않았다.
- 조각 줄, 진행(done_seq), base 끝 대기, 이음 검사는 세계 모의의 `OrderRun`과 `refusal`(`world/order_run.py`)을 그대로 쓴다.
  `pode_fsm`은 가져오지 않았다. `pode_fsm`이 `fake_robots`를 test_depend로 두어 순환이 생기고, Nav2 의존도 따라온다. `package.xml`은 바뀌지 않았다.

## 2. 바뀐 것

| 커밋 | 내용 |
|---|---|
| d0c4df0 | `test_scenario_integration.py`의 `FIRST_DOMAIN`은 `SCENARIO_FIRST_DOMAIN`(기본 50), `test_fms_restart.py`의 `DOMAIN`은 `FMS_RESTART_DOMAIN`(기본 56)으로 바꿀 수 있다 |
| 9a30706 | 시험 먼저: `sim/fake_robots/test/test_ros_pode_order.py` 9개 |
| 2d4b33e | 고침: `pode_order.py`(새 파일). `pode_goals.py`, `pode_goal.py`, `pode_retreat.py`, `fake_pode_node.py`도 고쳤다 |
| 45444e4 | Codex 리뷰 반영: 조각 검사, 이음 검사, 고장 돌려주기, 경합 둘. 시험 3개 더함 |
| df303a8 | Codex 리뷰 반영: base 끝에서 기다린 뒤 받은 조각은 새 줄로 시작한다. 시험 1개 더함 |
| 3b853be | blocked_lane 실패 원인 고침: base 끝에서 기다리는 order는 서 있는 로봇으로 본다. 시험 1개 더함 |
| (이 문서) | 작업 기록 |

order goal의 동작(실물 `pode/pode_fsm/pode_fsm/order_fsm.py`에 맞춤):

- `order_id`가 빈 MoveTo는 예전과 같다. 결과의 `last_done_seq_id`는 0이다.
- 새 order(`order_update_id` 0)는 하던 goal을 CANCELED로 끝내고 base 조각을 차례로 달린다.
  - follow_path는 path를 따라간다. nav_goal은 스스로 길을 짠다. retreat는 path를 따라 0.05 m/s로 뒤로 간다.
  - 첫 seq가 1이 아니거나, seq가 건너뛰거나, mode를 모르거나, path가 두 점보다 짧으면 NAV_ABORTED로 끝낸다.
- 피드백에는 `last_done_seq_id`와 `waiting_at_base_end`가 담긴다. 결과에는 `last_done_seq_id`가 담긴다.
- base가 끝나고 horizon이 남으면 base 끝에 서서 update를 기다린다. 이때 상태는 MOVING이다.
- update(`order_update_id` > 0)는 이음 조건을 검사한다: 같은 order_id, 다음 update id, base 끝 노드, 이어지는 seq, 조각 검사.
  - 맞으면 이어 붙이고 이전 goal을 SUPERSEDED(그때의 `last_done_seq_id`)로 끝낸다. 새 goal이 달리던 것을 그대로 이어받는다.
  - 안 맞으면 새 goal만 `ORDER_UPDATE_REJECTED: <이유>`로 끝내고, 하던 order는 계속 달린다.
- 연결 검사(PATH_NOT_CONNECTED)는 실물처럼 새로 시작하는 줄에만 한다.
  - 앞 조각에 0.02 m 안으로 이어진 같은 controller의 follow_path는 검사하지 않는다.
  - base 끝에서 기다린 뒤 받은 조각은 새 줄이므로 검사한다.
- 정지 오차(`stop_error_m`)는 마지막 조각에만 준다.
- Nav2 중단 고장은 조각 끝 노드로 잡는다. 조각이 먼저 끝나면 돌려준다.
- 취소하면 order 전체가 CANCELED로 끝난다.
- heartbeat가 끊기면 예전처럼 바로 선다(HEARTBEAT_LOST). 실물 기본값 `comm_loss_policy: stop`과 같다.
- 앞 물체 고장은 조각을 달릴 때만 잡는다. base 끝 대기 중에 나타난 물체는 goal이 없던 로봇처럼, 다음 조각을 시작한 자리에서 막는다.

## 3. 시험 (나온 그대로)

환경: 내 스크래치 빌드(`/tmp/claude-1000/fakeorder_ws`, `--symlink-install`, fleet_interfaces, fleet_core, fleet_manager, fake_robots, iponic_bringup)를 공유 install 위에 얹었다. `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
시나리오는 도메인 80-88, FMS 재시작과 order 끔 단독 실행은 89, 나머지는 79에서 돌렸다. 50-59에서는 돌리지 않았다(`pgrep -fa pc.launch.py` 빈 것 확인).

### 3.1 가짜 Pode order 시험 (`sim/fake_robots/test/test_ros_pode_order.py`)

- 고치기 전 코드(d0c4df0의 fake_robots)에 9개를 돌렸다: **6 실패, 3 통과**.
  통과 3개는 예전에도 되던 것이다: 취소, 새 order가 하던 것을 바꿈, order 없는 MoveTo.
- 2d4b33e 뒤: 9 통과.
- 45444e4의 시험 3개: 고치기 전 3 실패. 고친 뒤 test_ros_pode_order + test_ros_pode **70 통과**.
- df303a8의 시험 1개: 고치기 전 1 실패. 고친 뒤 13 통과.
- 3b853be의 시험 1개: 고치기 전 1 실패. 첫 고침 뒤에도 1 실패(아래 5절). 두 번째 고침 뒤 **14 통과**. 같은 코드에서 `test_ros_pode.py` **58 통과**.

### 3.2 시나리오 (`pc/fleet_manager/test/test_scenario_integration.py`)

명령: `SCENARIO_FIRST_DOMAIN=80 ROS_DOMAIN_ID=79 python3 -m pytest -v test/test_scenario_integration.py` (pc/fleet_manager에서).

- df303a8에서: **10 passed, 1 failed**(blocked_lane, 5절). 530.65 s.
- 3b853be에서: **11 passed**. 529.09 s.

order 끔(`ros2 launch iponic_bringup pc.launch.py use_fake:=true scenario:=basic_orders use_order_model:=false`, 도메인 89, summary.json 판정. 계약 검사와 replay는 빠짐):

- df303a8에서: 통과, 6/6, 192.6 s, ORDER_SENT 0.
- 3b853be에서: 통과, 6/6, 203.9 s, 위반 0, early 0, near 0, ORDER_SENT 0.
- blocked_lane order 끔(df303a8, 비교용): 통과, 6/6, 219.7 s.

### 3.3 그 밖

- `test_fms_restart.py`(`FMS_RESTART_DOMAIN=89 ROS_DOMAIN_ID=89`), 3b853be:
  - 1회: **1 failed**. 판정 위반 `t=370.4 J_E released from pode2 while 0.07 m from it`, 같은 때 OUT_DOCK. 그 밖의 재시작 검사는 모두 통과.
  - 2회: **1 passed**.
  - 3회: **1 failed**. `t=369.5 J_E ... 0.07 m`, OUT_DOCK 같은 때.
  - order 끔 비교(사본 `/tmp/claude-1000/fakeorder_ws/fms_off_copy`에 `-p use_order_model:=false`만 더함, 1회): **1 passed**.
  - 1, 2회는 시나리오 묶음과 동시에 돌았다. 3회는 fake_robots 전체 시험과 동시에 돌았다.
- `pc/fleet_manager`의 나머지 시험(test_adapters, test_ask_clear, test_goal_tracker, test_launch_args, test_node_order, test_node_wiring, test_order_goals), 도메인 79:
  - df303a8에서: **113 passed**.
  - 3b853be에서: **113 passed**, 경고 1.
- `pc/fleet_core/test`: **1075 passed, 3 xfailed**(45444e4. fleet_core는 이 브랜치에서 바뀌지 않았다).
- `sim/fake_robots/test` 전체(도메인 79), 3b853be: **1015 passed, 27 xfailed**, 1114.62 s.
  그 전에 두 번 시작했다가 중간에 멈췄다. 돌던 중에 fake 코드를 고쳐야 해서 마지막 코드로 다시 돌렸다.
- 시나리오 묶음도 두 번 중간에 멈췄다(Codex 지적을 고치기 전). 결과는 세지 않았다.

## 4. 시나리오별 결과 (3b853be, order 켬)

| 시나리오 | 판정 | 출고 | 시뮬 시간(s) | 위반 | early/near | ORDER_SENT | ORDER_UPDATED | ORDER_UPDATE_REJECTED |
|---|---|---|---|---|---|---|---|---|
| basic_orders | 통과 | 6/6 | 188.4 | 0 | 0/0 | 19 | 10 | 0 |
| order_burst | 통과 | 10/10 | 295.5 | 0 | 0/0 | 38 | 18 | 0 |
| shared_lane_failure | 통과 | 6/6 | 201.7 | 0 | 0/0 | 23 | 10 | 0 |
| stock_shortage | 통과 | 2/7(기대대로) | 138.8 | 0 | 0/0 | 17 | 7 | 0 |
| stuck_robot | 통과 | 2/6(기대대로) | 400.0 | 0 | 0/0 | 10 | 4 | 0 |
| operator_recovery | 통과 | 4/4 | 273.1 | 0 | 0/0 | 11 | 2 | 0 |
| blocked_lane | 통과 | 6/6 | 244.0 | 0 | 0/0 | 19 | 10 | 0 |
| auto_isolation | 통과 | 4/4 | 224.8 | 0 | 0/0 | 12 | 6 | 0 |
| reinstate | 통과 | 7/7 | 327.3 | 0 | 0/0 | 25 | 9 | 0 |

blocked_lane: pode1은 30.0 s에 첫 구간에서 막혔다. BLOCKED_WAIT은 32.1 s, ZONE_MARKED_BLOCKED는 42.1 s였다. order 끔 실행과 같은 모양이다.

## 5. 실패와 진단 기록

- 첫 order 켬 실행(df303a8): 시나리오 8 통과, blocked_lane 1 실패.
  - 6건 모두 출고, 위반 없음. `events_include`에서 `ZONE_MARKED_BLOCKED`만 빠졌다.
  - 같은 코드, order 끔으로 blocked_lane만 따로 돌렸다(도메인 89, `ros2 launch ... use_order_model:=false`): 통과(6/6, 219.7 s).
    pode1은 29.1 s에 W_Q를 떠나 첫 구간(W_Q→N_W)에서 막혔다. BLOCKED_WAIT은 32.4 s, ZONE_MARKED_BLOCKED는 42.4 s였다.
  - order 켬: pode1은 28.8 s에 떠났다. N_W까지 갔고(30.9 s SEGMENT_DONE), 다음 조각에서 33.3 s에 막혔다.
    BLOCKED_WAIT은 35.3 s였다. 물체는 44 s에 사라졌으므로 BLOCKED는 약 8.7 s 이어졌고, `blocked_mark_s` 10 s에 못 미쳤다.
  - 원인(가짜 Pode 결함): base 끝에서 기다리는 order goal을 "가는 중"으로 보았다.
    20 s에 나타난 물체 자리가 지난 조각의 지나온 길이(`motion.travelled`)로 잡혔고, 다음 조각이 그만큼 달린 뒤에야 섰다.
  - 첫 고침(대기 중에는 가는 중이 아니다)만으로는 시험이 계속 실패했다.
    update 뒤 첫 틱에 아직 지난 조각의 `travelled`(0.3 m)로 자리를 잡았다.
    그래서 조각을 달릴 때만 가는 중으로 보고, 조각이 목표를 정하는 그때 물체 자리를 잡게 했다.

## 6. 돌리지 않은 것

- 실물 로봇, Gazebo: 돌리지 않았다.
- `comm_loss_policy: base_end`: 가짜 Pode는 늘 stop처럼 선다. 실물 기본값이 stop이라 그대로 두었다.
- 경합 두 개(취소와 update, 피드백과 update)는 코드로 막았다. 직접 재현하는 시험은 없다.
- Nav2 중단 고장을 조각 경계에서 돌려주는 동작에는 시험이 없다.
- `horizon_warning`: 가짜 Pode는 horizon을 검사하지 않아 늘 빈 값이다.

## 7. 남은 점

- **관제: order의 물러나기 조각 중에 구역을 일찍 놓는다(test_fms_restart 실패).** 근거는 3회 실행(`/tmp/claude-1000/fakeorder_ws/fms_on3`)과 order 끔 사본을 비교한 것이다.
  - order 켬: pode2가 OUT_DOCK에서 N_E로 물러나는 조각(363.6 s 시작, 371.8 s 끝)이 끝나기 2.3 s 전인 369.5 s에 J_E와 OUT_DOCK을 놓았다(실제 0.066 m, 보고 0.094 m).
  - order 끔: 물러나기가 끝난 뒤인 373.8 s에 놓았다(실제 0.142 m).
  - [추측] 원인 후보(코드상 가능함은 Codex도 확인. 이 실행에서 그렇게 됐는지는 확인하지 않음):
    - `pc/fleet_core/fleet_core/traffic/zone/progress.py` `_track`(68-73행)은 goal의 마지막 링크가 아니고 NAV_GOAL이 아닌 링크를 `passed()`로 넘기며 `m.node`를 옮긴다.
      `passed()`(`trips.py`)는 링크 끝 3 cm 앞에서도 참이다.
    - order goal의 링크는 허가된 base 끝까지라서, 물러나기 링크 뒤에 base 링크가 이어지면 물러나기 링크가 마지막이 아니다.
      그러면 끝나기 전에 `m.node`가 N_E가 될 수 있다.
    - `_release_behind`(102-107행)는 보고 자세가 `m.node`에서 `release_margin_m`(0.05 m) 안이면 노드 자세로 거리를 잰다(놓는 거리 0.085 + 0.05 = 0.135 m).
      그래서 심판 반경(`safety.json` radius_m 0.08 m) 안에 몸이 남아 있을 때 놓을 수 있다.
    - order 끔에서는 같은 mode의 링크를 한 goal로 보내므로, 물러나기 링크가 그 goal의 마지막이라 넘기지 않는다.
    - 확인할 것: 해제 순간의 `m.node`, `_still_needed`, 연속 확인 횟수. 실물에서도 같은지는 모른다. 관제 고침은 이 작업 범위 밖이라 하지 않았다.
- 이음 거리 `JOIN_M` 0.02는 실물 `OrderParams.join_m`의 기본값을 베꼈다. 실물에서 이 값을 바꾸면 가짜와 달라진다(Codex 지적, 고치지 않음).
- 조각 검사(`segments_problem`, `joins`)는 pode_fsm `order_queue.py`의 같은 함수를 작게 다시 쓴 것이다. 의존 순환 때문에 가져오지 못했다. 한쪽을 바꾸면 다른 쪽도 바꿔야 한다.
- 조각 사이에서 한 틱(0.05 s) 멈춘다. 실물은 이어진 follow_path 조각을 한 줄로 합쳐 달린다. `distance_remaining`은 지금 조각의 남은 거리만 센다.
- `docs/worklog_order_default_on_2026-11-16.md` 65행의 "합치기 전에 정할 것"은 이 브랜치가 답한다. 그 문서는 고치지 않았다.
- `blocked_lane.yaml`의 물체 시각은 order 끔 시절에 맞춘 것이다. 고친 뒤에는 order 끔과 같은 곳에서 막혔지만 1회 실행이다.
- (해결) Codex(3b853be 리뷰)는 조각 경계를 넘어 남은 물체 자리(hold)가 다음 조각에서 틀린 거리로 쓰일 수 있다고 했다.
  나는 처음에 path 조각에서는 생기지 않는다고 판단했는데, 틀렸다. 문서 리뷰(302ea5f)에서 Codex가 반례를 들었다.
  반례: 조각이 이미 끝에서 시작하고 물체가 보이면, 첫 틱에 끝나면서 hold 0.01 m가 남는다.
  그 반례로 시험을 먼저 썼고(고치기 전 0.01 m 움직여 실패), 조각 시작마다 hold를 새 path의 시작으로 다시 잡게 고쳤다(`ObstacleRecovery.rebase`).
