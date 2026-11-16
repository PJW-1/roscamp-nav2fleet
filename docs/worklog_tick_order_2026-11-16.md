# 작업 기록: 교통 시험의 tick 순서와 운영 순서 비교 (2026-11-16)

브랜치 `test/tick-order` (origin/main 7c7d04b에서). 운영 코드(`pc/fleet_core/fleet_core/`)는 바꾸지 않았다.
첫 커밋 1b3a353, Codex 리뷰 지적을 반영한 후속 커밋이 이어진다(아래 "후속 반영").

## 결론

- 교통 전용 시나리오에서는 tick 순서만 바꿔도 결과가 같다. fleet_core 전체 시험을 하네스 순서
  `"zone"`(`ZoneTraffic.tick`)과 운영 순서 `"manager"`(`FleetManager.tick`)로 각각 돌렸다. 환경 변수로
  순서가 실제로 바뀐 세계는 342개(시험 326개, log 4071줄)이고, 그 모두에서 log, problems, DispatchResult,
  이벤트(상세 포함), 명령이 같았다. 다른 세계는 0개다. 순서를 시험이 직접 고르는 세계 16개
  (`test_zone_tick_order.py`)는 두 실행에서 같은 순서로 돌므로 재현성 근거일 뿐 순서 변경의 증거가 아니다.
- 이것은 전체 운영 흐름이 같다는 뜻이 아니다. 두 모드 모두 로봇의 보고와 답을 교통에 바로 넣는다.
  매니저 입력 함수(입력 검사와 그 직후의 `_pump`)를 거치지 않는다. 또 교통 전용 시나리오에는 주문판도
  팔 작업도 없어 두 순서의 차이가 생기는 경로(아래 P1~P4)가 열리지 않는다. 하네스가 틀린 것은 아니고,
  원래 그 영역을 보지 않는다.
- 권고 (b)를 이 브랜치에서 실행했다. 하네스 기본 순서를 `"manager"`(운영 `FleetManager.tick`)로 바꿨다.
  `"zone"`은 비교용 선택지로 남겼다(`tick_order="zone"` 또는 `ZONE_TICK_ORDER=zone`).
  (c) `ZoneTraffic.tick` 제거는 운영 코드를 바꾸는 일이라 사용자가 정한다.

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
tick 끝에 한 번에 꺼낸다. 비시험 호출은 없다. 다만 `tick`은 `TrafficApi` 계약(traffic/api.py:92)과
그 시험(test_traffic_api.py:22)에 들어 있다.

운영에서는 입력도 경계다. 매니저의 입력 함수(`on_robot_state`, `on_dock_result`, `on_order_board` 등)는
교통이나 주문 층에 넣은 뒤 바로 `_pump`한다(manager.py:117 등). 그래서 주문 층의 반응 가운데 일부는 tick이
아니라 입력 때 일어난다(아래 "입력 경계와 tick 경계").

시계: 운영은 입력마다 `_input`에서 시각을 얼린다(manager_record.py:37-40). 하네스는 tick 안에서
시각을 바꾸지 않으므로 값이 같다.

## 차이가 결과를 바꿀 수 있는 경로 (코드로 찾음)

운영에서만 일어나고 하네스 순서(`"zone"`)에서는 일어나지 않는다(주문 층이 없다).

- P1 격리 → 주문 되돌림 → 새 호출. 교통이 RobotIsolated를 내면 `orchestrator.on_robot_isolated`
  (orders/faults.py:32)가 묶인 주문을 되돌리고 `_sync_calls`로 DispatchCmd를 낸다. 격리가 tick 안에서
  생기면 그 호출은 같은 tick의 `tick_pick` 전에 교통에 들어간다. 팔이 일하던 로봇이면 ArmStopCmd나
  ArmDropCmd도 같은 `_pump`에서 팔로 간다.
- P2 도착 → 팔 명령, 호출 맞추기. DispatchResult가 `on_dispatch_result`(orchestrator.py:73)로 간다.
  이때 열린 호출이 바뀐다. 실험으로 보니 도킹 결과는 `on_dock_result` 입력 때 주문 층에 닿고, 팔 명령
  (ArmFillCmd)은 다음 tick의 `orchestrator.step`(`_step_robot`)이 낸다.
- P3 팔 timeout → 교통 고장. `arm.check_timeouts`(arm/jobs.py:94-110)의 RobotFaultCmd가 같은 tick의
  `_pump`로 `traffic.on_robot_fault`에 들어간다. 그 로봇은 그 tick의 `tick_progress`, `tick_pick` 전에
  격리되고, 다시 P1로 이어진다. 실험으로 같은 tick 안에서 일어남을 확인했다(아래).
- P4 `_step_robot`의 앞뒤 주문 층. 채우기가 끝나면 `orchestrator.step`이 ReleaseCmd를 내고, 같은 tick의
  `traffic.step`이 로봇을 내보낸다. `traffic.step` 뒤의 `orchestrator.step`은 출고 DispatchCmd를 낼 수 있다.
  주문 층의 진행은 교통 조건(running, 모두 자리에 있음, 가용, stopped 아님)에 묶여 있다.
- 결과를 바꾸지 않는 차이: 출력 목록 안의 순서(`_pump`가 arm, orchestrator, traffic 순으로 꺼낸다),
  goal 번호 발급기(운영은 팔과 공유, 교통 전용 시나리오에서는 팔이 일하지 않아 같은 순서).

## 실험 방법

1. `zone_harness.Harness`에 `tick_order`를 넣었다(`"manager"` 기본, `"zone"` 비교용). 환경 변수
   `ZONE_TICK_ORDER`로 실행 전체의 기본을 바꿀 수 있다.
   - `"manager"`는 운영 `FleetManager`를 그대로 만든다(`TappedManager`). 진짜 OrderOrchestrator, ArmJobs,
     `_pump`, `_route`, `FleetManager.tick`을 쓴다. 덧붙인 것은 기록뿐이고 시계 하나만 바꿨다.
     - `_route`를 지나는 항목을 `tap`과 `seen`(단계 "tick"/"input", tick 번호)에 남긴 뒤 원래 `_route`를 부른다.
       운영에서 주문 층이 먹는 DispatchResult, RobotIsolated도 하네스가 똑같이 볼 수 있게 하려는 것이다.
     - `_frozen`이 시계를 그때그때 읽는다. 기본 하네스는 로봇 답을 매니저 입력 함수를 거치지 않고 교통에
       바로 넣으므로, 운영의 "입력마다 `_input`에서 시각 갱신"을 이렇게 대신했다. tick 안에서는 값이 같다.
   - 로봇의 보고와 답은 기본으로 두 모드 모두 교통에 바로 넣는다. 그래서 순서 비교는 tick 순서만 본다.
   - `manager_inputs=True`(운영 순서에서만)를 주면 로봇의 보고와 답이 매니저 입력 함수와 그 직후의 `_pump`를
     거친다. 입력 경계와 tick 경계를 보는 시험에서만 쓴다.
2. fleet_core 전체 시험을 두 순서로 돌렸다. `conftest.py`에 넣은 기록(`ZONE_TICK_TRACE=<파일>`)이 시험마다
   만들어진 Harness 세계의 log, problems, results, events(상세 포함), commands를 JSON으로 남긴다.
   재현 방법:

   ```
   cd <저장소>
   PYTHONPATH=pc/fleet_core ZONE_TICK_TRACE=/tmp/manager.json python3 -m pytest pc/fleet_core/test -q
   ZONE_TICK_ORDER=zone PYTHONPATH=pc/fleet_core ZONE_TICK_TRACE=/tmp/zone.json \
     python3 -m pytest pc/fleet_core/test -q
   python3 - /tmp/zone.json /tmp/manager.json <<'EOF'
   import json, sys
   a, b = (json.load(open(p)) for p in sys.argv[1:3])
   keys = ("log", "problems", "results", "events", "commands")
   pick = lambda d, t: [[w[k] for k in keys] for w in d.get(t, [])]
   print([t for t in sorted(set(a) | set(b)) if pick(a, t) != pick(b, t)])
   # 환경 변수로 순서가 실제로 바뀐 세계만 센다
   pairs = [(x, y) for t in set(a) & set(b) if len(a[t]) == len(b[t]) for x, y in zip(a[t], b[t])
            if x["order"] == "zone" and y["order"] == "manager"]
   print(len(pairs), sum(any(x[k] != y[k] for k in keys) for x, y in pairs))
   EOF
   ```

   첫 커밋의 387개 비교는 scratchpad의 플러그인으로 했고 저장소에 없었다. 이번에 같은 일을 하는 기록을
   conftest.py에 넣어 위 명령으로 재현할 수 있게 했다.
3. 고정용 새 시험 `pc/fleet_core/test/test_zone_tick_order.py`(12개, 바꾸기 전 전체 745개에 더해 757개)
   - 같음 6개: 공용 시나리오 3개(serve_three, blocked_express, busy_outbound)와 통신 끊김, 고장 격리와
     운영자 해제, 정지와 재개를 두 순서로 돌려 log, problems, 결과가 같은지 본다.
   - 진짜로 `FleetManager.tick`을 부르는지 1개, 잘못된 선택 거부 1개, 기본 순서가 `"manager"`인지 1개
     (`ZONE_TICK_ORDER`를 준 실행에서는 건너뛴다).
   - 입력 경계와 tick 경계 3개(`manager_inputs=True`, `seen`의 단계와 tick 번호로 확인)
     - 주문판의 주문은 `on_order_board` 입력 때 바로 호출(DispatchCmd)이 되어 교통에 닿는다. 로봇은 다음
       tick의 `tick_pick`이 정한다(첫 MoveCmd가 tick 2).
     - WORK 도착 DispatchResult는 `on_dock_result` 입력 때 주문 층에 닿고, ArmFillCmd는 다음 tick에 나온다.
     - 팔 timeout: 앞서 그 로봇에 ArmFillCmd가 있었고(더 이른 tick), 팔의 ArmLost(kind fill),
       RobotFaultCmd(사유 "<server> goal <id> never reported"), 교통의 RobotIsolated가 모두 같은 로봇에 대해
       같은 tick 안에서 나온다. ARM_NO_RESULT 이벤트도 그 로봇이고, 그 뒤 로봇은 가용이 아니다. 같은 tick 안에서
       주문 층·팔이 교통에 답하는 경로(P3)를 실제로 확인한 시험이다.
   - `manager_inputs`는 로봇의 보고와 답에만 해당한다. `call()`은 늘 교통의 `on_dispatch`에 바로 넣고,
     주문판은 시험이 `world.manager.on_order_board`로 따로 넣는다.

## 결과 숫자

fleet_core 전체 시험 (2026-11-16, 후속 커밋 기준)

| 항목 | 기본(manager) | ZONE_TICK_ORDER=zone |
|---|---|---|
| 전체 시험 | 757 passed | 756 passed, 1 skipped |
| 걸린 시간 | 36.8 s | 35.6 s |
| Harness 세계(전부) | 359 | 358 |
| 그중 환경 변수로 순서가 바뀐 세계 | 342 (시험 326개, log 4071줄) | 342 |
| 그중 시험이 순서를 직접 고른 세계 | 16 (+기본 순서 확인 1) | 16 |
| 순서가 바뀐 세계 가운데 log/problems/results/events/commands가 다른 것 | 0 | |

건너뛴 1개는 기본 순서 확인 시험이다(순서를 고른 실행이라). 이 시험이 만드는 세계 1개가 359와 358의 차이다.
`test_zone_tick_order.py`의 시험은 순서를 직접 정하므로 두 실행에서 같은 일을 한다(위의 16개). 전체로
비교하면 358개 세계가 모두 같았지만, 순서 변경의 증거는 342개뿐이다.

첫 커밋(1b3a353) 때의 비교: 하네스를 쓰는 30개 파일 387개 시험, 세계 340개, log 4055줄, 차이 0.

하네스 시험이 보는 안전 조건(세계를 만드는 시험 함수 298개, 첫 커밋 기준. 시험 함수 본문의 낱말로 대충
센 값이다. 도우미 함수 안의 단언은 세지 못했다)

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

## 권고와 근거

(b) 하네스가 운영과 같은 tick 순서 함수(`FleetManager.tick`)를 쓰게 한다. 이 브랜치에서 실행했다.

- 근거는 tick 본문 순서를 두 곳에 손으로 적어 둔 중복(controller.py:34-43, manager.py:218-233)을 시험에서
  없애는 것이다. 한쪽만 바뀌면 시험이 운영과 다른 순서를 조용히 시험하게 되는데, 하네스가 운영 함수를 부르면
  그 위험이 없다.
- 비용이 없다. 전체 시험 결과와 기록이 두 순서에서 같았고 시간도 비슷했다. 단언은 하나도 바꾸지 않았다.
- 한계: 이것으로 주문·팔 경로가 운영과 같게 시험되는 것은 아니다. 기본 하네스는 로봇의 보고와 답을
  매니저 입력 함수와 그 직후의 `_pump` 없이 교통에 넣으므로, 입력 때 일어나는 주문 층의 반응(P2의 도킹 결과,
  주문판)은 운영과 다른 때에 일어날 수 있다. 주문이나 팔이 얽힌 시험은 `manager_inputs=True`로 로봇 보고·답의
  입력 경계를 운영과 같게 하거나, 매니저 전체를 쓰는 sim/fake_robots 세계 시험으로 따로 확인해야 한다.
  `manager_inputs`는 로봇 보고·답에만 해당한다. `Harness.call()`은 주문 층을 거치지 않고 교통에 바로 호출을
  넣으므로, 주문 흐름을 시험하려면 `call()` 대신 `world.manager.on_order_board`로 주문판을 넣어야 한다.
- (a)는 새 시험이 그 몫(같음 6개, 경계 3개)을 한다. 그것만으로는 하네스가 운영 순서를 시험하지 않는 문제가
  남아 (b)를 함께 했다.
- (c) `ZoneTraffic.tick` 제거는 지금은 권하지 않는다. 운영 코드인 `TrafficApi` 계약(api.py:92), 그 시험
  (test_traffic_api.py:22), 설계 문서(docs/IFP_공통설계.md:379)를 함께 바꿔야 한다. 비교용 `"zone"`도 이것을
  쓴다. 운영 코드를 바꾸는 일이라 사용자 결정이다.

## 후속 반영 (Codex 리뷰, 2026-11-16)

- 첫 커밋의 "주문판 주문이 같은 tick에 호출이 된다", "도착 결과와 팔 명령이 같은 tick에 나온다" 시험은
  하네스 기록의 시각만 비교했고 입력을 교통에 바로 넣어 주장을 입증하지 못했다. 매니저 입력 경로와
  단계·tick 번호로 다시 썼다. 실제로는 둘 다 입력 때 일어나고(주문판 호출, 도착 결과), 팔 명령은 다음 tick에
  나왔다. 같은 tick 안의 경로는 팔 timeout(P3)으로 따로 시험했다.
- 하네스 기본 순서를 `"manager"`로 바꿨다.
- 위 "한계"와 결론의 범위를 "교통 전용 시나리오에서 tick 순서만 바꿔도 같다"로 좁혔다.
- 387개 비교의 재현 수단을 conftest.py에 넣었다.
- 두 번째 리뷰(19e36d6): 순서가 실제로 바뀐 세계 수(342)를 따로 적었다. 팔 timeout 시험이 같은 로봇,
  팔 timeout 사유, 앞선 팔 작업까지 단언하게 했다. `manager_inputs`의 범위(로봇 보고·답만, `call()`은 제외)를
  하네스 주석과 권고에 적었다.

## 돌리지 않은 것

- 주문과 팔이 실제로 일하는 상태의 두 순서 비교. 하네스 순서에는 주문 층이 없으므로 비교할 대상이 없다.
  운영 순서의 그 경로는 sim/fake_robots의 세계 시험(FleetManager를 쓴다)이 다룬다. 이번에 돌리지 않았다.
- 기존 하네스 시험 전체를 `manager_inputs=True`로 돌리기. 입력 검사(자세 범위 등)의 차이가 섞인다.
- ruff(저장소의 I 규칙). 설치되어 있지 않아 flake8의 E, F, B만 돌렸다.
- ROS, Gazebo, 실제 로봇.
