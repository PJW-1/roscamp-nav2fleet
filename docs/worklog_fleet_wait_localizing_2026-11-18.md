# 작업 기록: LOCALIZING 로봇에게 일을 주지 않고 기다리기 (2026-11-18)

브랜치 `fix/fleet-wait-localizing-2026-11-18`, main d66d75d 기준.

## 문제 (실물, 2026-11-18)

- 노드 무리가 망에 붙으면(다른 로봇의 launch, PC launch) 라즈베리파이의 CPU와 Wi-Fi가 약 25 s 포화된다.
  map->base_link TF가 `pose_timeout_s` 1.0 s보다 오래되어 pode_fsm이 `LOCALIZING`과 자세 (0,0,0)을 보고했다.
- 관제는 pode2의 마지막 보고가 `LOCALIZING`인데도 order를 보냈다(기록: `LOCALIZING` 1.3 s 뒤 첫 `ORDER_SENT`).
- pode_fsm은 자세가 없으면 goal을 바로 `NAV_ABORTED`로 끝낸다(`pode/pode_fsm/pode_fsm/order_fsm.py:98-101`,
  `core.py:151-152`, 확인함).
- 관제는 그것을 시도로 셌다(`traffic/zone/base.py` `_goal_failed`, `move_retry_limit` 2, `move_retry_wait_s` 2 s).
  약 5 s 안에 세 번 실패해 격리: `GOAL_FAILED NAV_ABORTED after 3 attempts`, `ROBOT_ISOLATED NAV_ABORTED; holds ['HALL']`.

## 바꾼 것 (`pc/fleet_core/fleet_core/traffic/`)

규칙: 로봇의 마지막 보고가 `LOCALIZING`이면

1. `Drive.step`이 그 로봇에 아무것도 내지 않는다: 새 이동 goal/order, order 갱신(base 연장, horizon), 도킹 명령,
   재시도 모두. LOCALIZING 한 번에 `WAIT_LOCALIZING`(WARN)을 한 번 낸다. 위치를 다시 보고하면 `LOCALIZED`(INFO,
   걸린 시간)를 내고 다음 틱부터 전처럼 진행한다(밀린 goal이 그때 나간다).
2. 열린 호출의 후보에서 뺀다(`Calls._robot_info`의 `eligible`). 다른 로봇이 호출을 받는다. 이름으로 부른 배차는
   그대로 받아 두고, 위치가 돌아오면 보낸다.
3. 그동안 온 이동/후진 결과 `NAV_ABORTED`는 시도로 세지 않는다(`Results._move_failed`). goal을 잊고
   `retry_at = now + move_retry_wait_s`로 두고 `GOAL_ABORTED_LOCALIZING`(WARN)을 낸다. 위치가 돌아오고 그 대기가
   지나면 다시 보낸다.
4. 보고와 결과의 순서(`LOCALIZING_GRACE_S` = 1.0 s, Pode `pose_timeout_s`와 같은 값):
   - 결과가 먼저 오고 1.0 s 안에 LOCALIZING 보고가 오면, 방금 센 `NAV_ABORTED` 시도 하나를 되돌린다
     (`GOAL_ABORTED_LOCALIZING`). 이미 격리됐으면 되돌리지 않는다.
   - 위치를 다시 보고한 뒤 1.0 s 안에 온 `NAV_ABORTED`도 LOCALIZING 중의 것으로 본다(늦게 닿은 결과).
   - 그보다 떨어지면 전처럼 센다.
5. `GOAL_STALLED`: LOCALIZING 보고 동안은 제자리 멈춤 감시를 쉰다(`watch.py`). 자세가 굳어 있어 goal이 열린 채
   12 s가 지나면 `STALLED`로 시도를 셌을 것이다.

파일: `mission.py`(`NAV_ABORTED` 상수), `zone/robots.py`(`Health.localizing_since`), `zone/base.py`(`_localizing`,
`_wait_localizing`, `_localized`, `_lost_pose_lately`, `_note_localizing`), `zone/drive.py`, `zone/watch.py`, `zone/calls.py`, `zone/progress.py`, `zone/results.py`.
`docs/관제_구역교통.md` 사건 표에 세 코드를 더했다.

## 그대로인 것

- 위치를 잡은 로봇의 `NAV_ABORTED`, LOCALIZING 중이라도 다른 사유(`FAILED`, `REJECTED`, `OBSTACLE_BEHIND` 등)는
  전처럼 센다. `HEARTBEAT_LOST`, `NOT_AT_TARGET` 처리, 도킹 실패, LOCALIZING이 아닐 때의 `GOAL_STALLED`, `COMM_LOST`/복구, 시작 배치
  (`NOT_PLACEABLE`), 운영자 치우기는 손대지 않았다.
- 오래 LOCALIZING인 로봇: 새 시간 한도는 없다. `WAIT_LOCALIZING` 한 번, 그 뒤 주행 단계(TO_WORK 등)이고 goal이
  없으면 있던 `ROBOT_IDLE`이 `idle_watch_s`(10 s)마다 난다. 그 사건이 SILENT_STALL 감시를 매번 새로 시작하므로
  `SILENT_STALL`은 나지 않는다. 일은 그 로봇에 남는다: 이름 배차는 그대로 기다리고, 열린 호출은 다른 로봇이 받는다.
  (`ROBOT_IDLE` detail의 "nothing to wait for"는 이 경우 맞지 않는 말이다. 고치지 않았다.)
- `available`은 바꾸지 않았다(감시와 sim이 쓴다). 호출 후보 조건과 `step`에서만 본다.
- 출고 배차의 실패 횟수(`orders/orchestrator.py` `dispatch_failures`)는 관제가 실패한 `DispatchResult`를 낼 때만
  는다. 교통 층은 격리(`_fail`)나 취소 때만 그것을 내므로, 세지 않은 `NAV_ABORTED`는 그 횟수도 늘리지 않는다.
- order 결과는 지금처럼 `_order_result`가 끝낸 조각을 먼저 반영한 뒤 `_move_failed`로 온다(순서 그대로).
- 구역: LOCALIZING 보고는 자세를 쓰지 않으므로(`progress.py`의 이른 return) 놓는 계산을 하지 않는다. 로봇은 쥔
  구역과 지점을 그대로 쥐고, 그 구역이 필요한 다른 로봇은 기다린다. 구역 정책은 바꾸지 않았다.
- LOCALIZING 보고도 `last_seen`을 갱신하므로 COMM_LOST가 되지 않는다(코드 확인). `docs/IFP_공통설계.md` 306행의
  "LOCALIZING만 보내면 보고 시각을 갱신하지 않아 COMM_LOST"는 지금 코드와 다르다(문서는 고치지 않았다).

## 시험

새 파일 `pc/fleet_core/test/test_zone_wait_localizing.py`(10개). 바꾸기 전에 8개 실패, 2개 통과
(재시도 대기와 다른 사유 세기는 지키는 시험이라 원래 통과).

명령(worktree의 코드를 먼저 읽게 함):

```
W=<worktree>
export PYTHONPATH=$W/pc/fleet_core:$W/pc/fleet_core/test:$W/pc/fleet_manager:$PYTHONPATH
source /opt/ros/jazzy/setup.bash; source <ws>/install/local_setup.bash  # fleet_manager용
export ROS_DOMAIN_ID=70 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd pc/fleet_core && python3 -m pytest -q -p no:cacheprovider test
cd pc/fleet_manager && python3 -m pytest -q -p no:cacheprovider test
```

결과:

- 관련 시험(wait_localizing, failures, localization, at_target, edges, recovery, comm_loss_policy, selector):
  139 passed, 1 xfailed (새 시험 10개일 때). 순서/멈춤 시험 5개를 더한 뒤 새 파일 15 passed
  (더한 5개 가운데 3개는 바꾸기 전에 실패, 2개는 지키는 시험)
- pc/fleet_core 전체: `1095 passed, 5 xfailed in 41.94s`
- pc/fleet_manager 전체(`/opt/ros/jazzy`와 ws install을 source한 뒤, 그 위에 worktree 경로): `131 passed in 705.01s`.
  source 없이 돌리면 `fleet_interfaces`가 없어 `1 skipped`로 끝난다

## 확인하지 않은 것

- 실물 로봇, Gazebo 모두 돌리지 않았다. 25 s 포화 상황에서 실제로 order가 멈추고 격리되지 않는지는 실물에서 봐야 한다.
- 1.0 s 창은 실물에서 재지 않았다. 포화된 Wi-Fi에서 결과와 보고가 1 s 넘게 어긋나면 센다 [추측].
- LOCALIZING 중 goal이 열린 채 남으면(결과를 잃음) 멈춤 감시가 쉬므로 위치가 돌아올 때까지 그대로 열려 있다.
  pode_fsm은 자세가 없으면 바로 끝내므로 드물다고 본다 [추측].
- 짧게 깜박이는 LOCALIZING으로 불린 로봇(TO_WORK, 아직 도크 구역을 쥐지 않음)이 후보에서 빠지면 주차로 돌고 다른
  로봇이 호출을 받는다. 호출이 오가는 일이 늘 수 있다 [추측].
