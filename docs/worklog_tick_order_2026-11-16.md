# 작업 기록: 교통 시험의 tick 순서와 운영 순서 비교 (2026-11-16)

브랜치 `test/tick-order` (origin/main 7c7d04b에서). 운영 코드(`pc/fleet_core/fleet_core/`)는 바꾸지 않았다.

## 결론

- 지금의 하네스 시험에서는 두 순서의 결과가 같다. 하네스 시험 387개(Harness 세계 340개)를
  `ZoneTraffic.tick`과 운영 `FleetManager.tick`으로 각각 돌렸다. 기록 줄(log 4055줄), 안전 위반(`problems`),
  DispatchResult, 이벤트가 시험마다 모두 같았다. 다르게 나온 시험은 0개다.
- 결과가 같은 이유: 두 순서의 차이는 `_pump`로 주문 층과 팔이 같은 tick 안에 교통에 답하는 것뿐이다.
  그런데 하네스 시험에는 주문판도 팔 작업도 없다. 그래서 그 경로(아래 P1~P4)가 한 번도 열리지 않는다.
  다시 말해 하네스 시험은 그 경로의 결함을 잡지 못한다. 하네스가 틀린 것은 아니고, 원래 보지 않는 영역이다.
- 권고: (b) 하네스가 운영과 같은 순서 함수(`FleetManager.tick`)를 쓰게 한다. 이번 브랜치에 그 선택
  (`tick_order="manager"`, 환경 변수 `ZONE_TICK_ORDER`)을 넣었다. 기본값은 아직 `"zone"`이다. 기본값을
  바꾸는 한 줄과 (c) `ZoneTraffic.tick` 제거는 사용자가 정한다.

## 두 순서의 차이

운영 `FleetManager.tick` (manager.py:218-237)

```
_input("tick")
traffic.tick_begin(now);   _pump()
arm.check_timeouts(now);   _pump()
stopped = traffic.tick_progress(now);   _pump()
running이고 모두 자리에 있으면:
    traffic.tick_pick(stopped);   _pump()
    가용이고 stopped가 아닌 로봇마다 _step_robot(robot)
```

`_step_robot` (manager_route.py:19-25)은 `orchestrator.step → _pump → traffic.step → _pump →
orchestrator.step → _pump` 순서다. 교통 한 걸음 앞뒤로 주문 층이 한 번씩 더 돈다.
`_pump` (manager_route.py:27-34)는 arm, orchestrator, traffic의 출력을 빌 때까지 꺼내 `_route`로 넘긴다.
`_route`는 DispatchCmd, CancelDispatchCmd, ReleaseCmd, RobotFaultCmd를 교통에, DispatchResult,
DispatchFeedback, RobotIsolated, 팔 보고를 주문 층에, 팔 명령을 팔에 넘긴다.

하네스 `ZoneTraffic.tick` (traffic/zone/controller.py:34-43)은 같은 교통 단계
(`tick_begin → tick_progress → tick_pick → step`)를 부르지만 `_pump`, 팔, 주문 층이 없다. 출력은
tick 끝에 한 번에 꺼낸다. 비시험 호출은 없다(하네스 zone_harness.py에서만 쓴다). 다만 `tick`은
`TrafficApi` 계약(traffic/api.py:92)과 그 시험(test_traffic_api.py:22)에 들어 있다.

시계: 운영은 입력마다 `_input`에서 시각을 얼린다(manager_record.py:37-40). 하네스는 tick 안에서
시각을 바꾸지 않으므로 값이 같다.

## 차이가 결과를 바꿀 수 있는 경로 (코드로 찾음)

모두 운영에서는 같은 tick 안에, 하네스에서는 일어나지 않는다(하네스에는 주문 층이 없다).

- P1 격리 → 주문 되돌림 → 새 호출. 교통이 `tick_begin`(통신 끊김)이나 `tick_progress`(감시)에서
  RobotIsolated를 내면 `orchestrator.on_robot_isolated`(orders/faults.py:32)가 묶인 주문을 되돌리고
  `_sync_calls`로 DispatchCmd를 낸다. 이 호출이 같은 tick의 `tick_pick` 전에 교통에 들어간다.
  팔이 일하던 로봇이면 ArmStopCmd나 ArmDropCmd도 같은 tick에 팔로 간다.
- P2 도착 → 팔 명령, 호출 맞추기. DispatchResult가 `on_dispatch_result`(orchestrator.py:73)로 가서
  `_start_filling`(팔 명령)과 `_sync_calls`(DispatchCmd/CancelDispatchCmd)를 부른다. 열린 호출이 같은
  tick의 `tick_pick` 전에 바뀐다.
- P3 팔 timeout → 교통 고장. `arm.check_timeouts`(arm/jobs.py:94-110)가 RobotFaultCmd를 내면
  `tick_progress`와 `tick_pick` 전에 `traffic.on_robot_fault`가 불린다. 그 로봇은 그 tick에 뽑히지 않고,
  다시 P1로 이어진다. 하네스에는 팔이 없어 이 단계 자체가 없다.
- P4 `_step_robot`의 앞뒤 주문 층. 채우기가 끝나면 `orchestrator.step`이 ReleaseCmd를 내고, 같은 tick의
  `traffic.step`이 로봇을 내보낸다. `traffic.step` 뒤의 `orchestrator.step`은 출고 DispatchCmd를 낼 수 있다.
  주문 층의 진행은 교통 조건(running, 모두 자리에 있음, 가용, stopped 아님)에 묶여 있다.
- 결과를 바꾸지 않는 차이: 출력 목록 안의 순서(`_pump`가 arm, orchestrator, traffic 순으로 꺼낸다),
  goal 번호 발급기(운영은 팔과 공유, 하네스 시험에서는 팔이 일하지 않아 같은 순서).

## 실험 방법

1. `zone_harness.Harness`에 `tick_order`를 넣었다(`"zone"` 기본, `"manager"`). 환경 변수
   `ZONE_TICK_ORDER`로 모든 하네스 시험의 기본을 바꿀 수 있다.
   - `"manager"`는 운영 `FleetManager`를 그대로 만든다(`TappedManager`). 진짜 OrderOrchestrator, ArmJobs,
     `_pump`, `_route`, `FleetManager.tick`을 쓴다. 바꾼 것은 둘뿐이다.
     - `_route`를 지나는 항목을 `tap`에 남긴다(그 뒤 원래 `_route`를 부른다). 운영에서 주문 층이 먹는
       DispatchResult, RobotIsolated도 하네스가 똑같이 볼 수 있게 하려는 것이다.
     - `_frozen`이 시계를 그때그때 읽는다. 하네스는 로봇의 답을 매니저의 입력 함수를 거치지 않고 교통에
       바로 넣으므로, 운영의 "입력마다 `_input`에서 시각 갱신"을 이렇게 대신했다. tick 안에서는 값이 같다.
   - 로봇의 보고와 답은 두 순서 모두 교통에 바로 넣는다. 입력 검사(`on_robot_state`의 자세 범위 검사 등)의
     차이가 섞이지 않게 하려는 것이다. 그래서 이 실험은 tick 순서만 비교한다.
2. 하네스 시험 30개 파일(새 시험 파일 제외, 387개)을 두 순서로 돌렸다. pytest 플러그인으로 시험마다
   만들어진 Harness 세계의 `log`, `problems`, `results`, `events`를 모아 시험마다 비교했다.
   (플러그인과 비교 스크립트는 scratchpad에만 두었다. 저장소에는 넣지 않았다.)
3. 고정용 새 시험 `pc/fleet_core/test/test_zone_tick_order.py`(10개)
   - 같음 6개: 공용 시나리오 3개(serve_three, blocked_express, busy_outbound)와 통신 끊김, 고장 격리와
     운영자 해제, 정지와 재개를 두 순서로 돌려 log, problems, 결과가 같은지 본다. 운영 순서에 주문 층의
     일이 생기면 여기서 먼저 깨진다.
   - 진짜로 `FleetManager.tick`을 부르는지 1개, 모르는 순서 이름 거부 1개.
   - 의도된 차이 2개(운영 순서에서만): 주문판에 올린 주문이 같은 tick에 호출이 되어 교통이 로봇을 정한다.
     WORK 도착 DispatchResult와 ArmFillCmd가 같은 tick 기록에 함께 나온다.

## 결과 숫자

| 항목 | zone | manager |
|---|---|---|
| 하네스 시험(노드) | 387 통과 | 387 통과 |
| Harness 세계 | 340 | 340 |
| 세계를 만든 시험 노드 | 325 | 325 |
| log 줄 | 4055 | 같음 |
| log/problems/results/events가 다른 시험 | - | 0 |
| 걸린 시간 | 30.1 s | 30.7 s |

하네스 시험이 보는 안전 조건(세계를 만드는 시험 함수 298개, 시험 함수 본문의 낱말로 대충 센 값이다.
도우미 함수 안의 단언은 세지 못했다)

| 조건 | 시험 함수 수 |
|---|---|
| 구역 독점(`problems` 단언: 쥔 구역 밖, 한 줄 구역 맞방향, 도크 함께 씀) | 70 |
| 해제(on_release, ZONES_RETURNED/CLEARED) | 53 |
| 통신 끊김(silent, COMM_LOST/RESTORED) | 41 |
| 고장 격리와 운영자 해제(on_robot_fault, confirm_clear) | 24 |
| 취소(CancelCmd, STOP, on_dispatch_cancel) | 23 |
| 교착(deadlock, cycle, backoff) | 16 |

`Harness._check`(zone_harness.py)는 모든 세계에서 걸음마다 구역 독점을 검사하지만, `problems`를
단언하는 시험만 그 결과로 실패한다.

전체 시험: `PYTHONPATH=pc/fleet_core python3 -m pytest pc/fleet_core/test -q` → 755 passed
(바꾸기 전 745 + 새 시험 10).

## 권고와 근거

(b) 하네스가 운영과 같은 순서 함수(`FleetManager.tick`)를 쓰게 하는 것을 권한다.

- 지금 결과가 같다는 것은 "차이가 무해하다"가 아니라 "하네스가 차이를 볼 수 없다"는 뜻이다. P1~P4는 모두
  주문 층이나 팔에 상태가 있어야 열린다. 운영 순서로 돌리면 시험에 주문판이나 팔 일을 더하는 순간 그 경로가
  저절로 운영과 같은 순서로 시험된다.
- 비용이 없다. 387개 모두 같은 결과, 같은 시간(30 s대)이었다. 하네스 시험의 단언은 하나도 바꿀 필요가 없다.
- 두 곳에 같은 순서를 손으로 적어 둔 상태(controller.py:34-43, manager.py:218-233)는 한쪽만 바뀌면 시험이
  운영과 다른 것을 조용히 시험하게 된다. 하네스가 운영 함수를 부르면 그 위험이 없다.
- (a)는 차이를 문서와 시험으로 고정할 뿐 하네스가 운영 순서를 시험하지 않는 문제가 남는다. 이번 새 시험이
  (a)의 몫(같음 6개, 의도된 차이 2개)은 이미 한다.
- (c) `ZoneTraffic.tick` 제거는 (b) 뒤에 할 수 있지만 지금은 권하지 않는다. 운영 코드인 `TrafficApi`
  계약(api.py:92), 그 시험(test_traffic_api.py:22), 설계 문서(docs/IFP_공통설계.md:379 근처)를 함께 바꿔야
  하고, 하네스가 기본으로 `"manager"`를 쓰기 전에는 지울 수 없다. 운영 코드를 바꾸는 일이라 사용자 결정이다.

(b)를 마치는 방법: `zone_harness.py`의 `TICK_ORDER` 기본값을 `"manager"`로 바꾼다(한 줄). 같음 시험은
`"zone"`을 계속 비교하므로 두 순서가 갈라지면 알려 준다.

## 돌리지 않은 것

- 주문판과 팔이 실제로 일하는 상태의 두 순서 비교. 하네스 순서(`ZoneTraffic.tick`)에는 주문 층이 없으므로
  비교할 대상 자체가 없다. 운영 순서의 그 경로는 sim/fake_robots의 세계 시험(FleetManager를 쓴다)이 다룬다.
  이번에 그 시험들은 돌리지 않았다.
- 입력 함수를 거친 경우(`FleetManager.on_robot_state` 등의 입력 검사와 입력마다의 `_pump`). 이번 실험은
  tick 순서만 떼어 보려고 로봇 답을 교통에 바로 넣었다.
- `ZONE_TICK_ORDER=manager`로 전체 시험(하네스 밖 시험 포함) 돌리기. 하네스 시험 387개만 두 순서로 돌렸다.
- ROS, Gazebo, 실제 로봇.
