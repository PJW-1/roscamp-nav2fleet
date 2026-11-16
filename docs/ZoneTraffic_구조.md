# ZoneTraffic 구조 지도

기준: origin/main 7c7d04b. 코드는 바꾸지 않았다. 이 문서는 구조를 읽는 지도다. 전면 재구성을 권하지 않는다.
표에 든 숫자는 스크립트(ast)로 직접 센 값이다. 센 방법은 8절에 있다. 추측은 [추측]으로 표시했다.
줄 번호는 7c7d04b 기준이다. 코드가 바뀌면 어긋난다.

## 0. 한눈에

- ZoneTraffic은 관제 교통 층의 구현이다. 구역(zone)과 노드를 로봇에게 "허락"하고, 허락받은 구간만 이동 명령으로 내보낸다.
- `pc/fleet_core/fleet_core/traffic/zone/`에 파일 30개, 3349줄. 믹스인 23개(클래스 몸통 합계 2140줄, 메서드 165개) + ZoneBase(공통 상태) + ZoneTraffic(조립과 tick 3개).
- 구역 허락 기록은 믹스인이 아니라 객체 `ZoneGrants`(grants.py, 134줄) 하나가 쥔다. 쓰는 지점은 10곳, 클래스 7개(믹스인 6개와 ZoneBase)다(5절).
- 믹스인이 다른 믹스인의 메서드를 부르는 (부르는 클래스, 제공 클래스) 쌍은 41개다. ZoneBase 도우미를 부르는 믹스인은 22개다. ZoneBase의 13개 도우미는 모두 어느 믹스인이 쓴다.
- 가장 많이 기대는 믹스인은 Blocked, Drive, Results, Watch, MoveOrders다(4절). 가장 많이 기댐을 받는 것은 ZoneBase 도우미(`_event` 21개 믹스인, `_cancel_goal` 8개)이고, 믹스인으로는 Goals(6개 믹스인이 부름)다.
- 서로 맞부르는 쌍이 넷 있다: Goals-MoveOrders, Goals-Turns, Backoff-Results, Backoff-Watch. 여기가 가장 얽힌 곳이다.
- 점진 분리 후보는 6절. 가장 낮은 위험부터 Views, 작은 감시 셋(Capacity, Localization, NoStart), 막힘 표시(BlockMarks+ClearChecks)다.
- 코드를 읽다 찾은 이상한 점 일곱 가지는 7절. 고치지 않았다.

## 1. ZoneTraffic이 무엇인가

`controller.py:31-33`에서 믹스인 23개와 ZoneBase를 다중 상속한다. 앞에 쓴 클래스가 이름 충돌에서 이긴다(MRO). 같은 이름의 메서드를 두 번 정의하지 못하게 `test_zone_mixins.py`가 막는다(예외는 `_event` 하나, Silent가 ZoneBase의 것을 가로채 `super()`로 넘긴다).

믹스인에는 `__init__`이 없다. 모든 상태는 `ZoneBase.__init__`(base.py:23-52)이 만든다. 믹스인은 `self.<속성>`으로 그 상태를 직접 읽고 쓴다. 로봇별 상태는 `Robot` 레코드(robots.py:51, 필드 77개)에 있고, 믹스인은 `m.<필드>`로 쓴다(3절 끝).

| 믹스인 | 정의 위치 | 줄 | 맡은 일 한 줄 | 바깥에서 부르는 입구 |
|---|---|---|---|---|
| Calls | calls.py:24 | 157 | 운행 모드, 배차 받기·취소·놓기, 열린 호출에 응할 로봇 고르기, 단계에 따라 다음 목적지로 돌리기 | on_run_mode, on_dispatch, on_dispatch_cancel, on_release, on_robot_fault, tick_pick |
| Goals | goals.py:18 | 206 | 링크 비용과 경로 찾기, 후보 중 잡을 수 있는 트립 고르기, 다음 구간의 구역·노드·차선을 `grants.take`로 잡기 | (없음) |
| Turns | turns.py:11 | 129 | 도크 대기 줄과 차례, 스테이션의 도크·대기·주차 지점 후보 | (없음) |
| Drive | drive.py:21 | 124 | 로봇 단계(phase)별 한 걸음: 트립 따라 가기, 도착 판정, 이동·도킹·물러나기 명령 보내기 | step |
| MoveOrders | move_orders.py:17 | 139 | order 모델(`use_order_model`)의 주문 조각(base, horizon) 만들기와 갱신 보내기 | (없음) |
| OrderAnswers | order_answers.py:15 | 113 | order 피드백, 결과, 갱신 실패 처리, 통신 끊김 때 base 끝까지 갈 시간 | on_move_feedback |
| Results | results.py:22 | 169 | goal 결과 처리(성공, 실패, 거절, 미전송, 잃음), 도착 확인, 구간 완료 | on_move_result, on_retreat_result, on_dock_result, on_goal_rejected, on_goal_not_sent, on_goal_lost |
| Redock | redock.py:21 | 35 | 도킹 실패 후 물러났다가 다시 도크로 들어오는 트립 | (없음) |
| Progress | progress.py:30 | 133 | 로봇 상태 보고 접수, 트립 진행, 지나온 구역·노드 놓기(`grants.release_*`), 배터리 | on_robot_state |
| Placement | placement.py:17 | 54 | 시작할 때 로봇을 노드와 구역에 `grants.force`로 올려 놓기, 겹치면 거절 | (없음) |
| Blocked | blocked.py:24 | 163 | BLOCKED 로봇 처리, 막힘 기록과 만료, 다른 길로 다시 가기, 자가 복구 기록 | (없음) |
| BlockMarks | block_marks.py:59 | 83 | 막힘 표시(Mark, block_marks.py:16) 만들기, 링크 막기, 만료 | (없음) |
| YardRetreat | yard_retreat.py:24 | 86 | 마당(SHARED)과 급행에서 물러날 길 정하기, 다른 로봇에게 양보 | (없음) |
| Detours | detours.py:33 | 12 | 우회 종류 판정과 DETOUR 이벤트 | (없음) |
| ClearChecks | clear_checks.py:10 | 52 | 막힘 표시 근처 로봇에게 해제 확인 요청, 응답으로 표시 지우기 | on_clear_check |
| Health | health.py:12 | 117 | 시작 보고, 통신 끊김과 복구, 운영자 치우기·되돌리기 | confirm_clear, reinstate_refusal, reinstate |
| Watch | watch.py:22 | 139 | 움직임 없는 goal(GOAL_STALLED), 구역 밖, 가까이 선 두 로봇, 대기 순환(교착) 감시 | (없음) |
| Localization | localization.py:21 | 12 | 보고 자세가 튀면 LOCALIZATION_SUSPECT 경고 | (없음) |
| Backoff | backoff.py:52 | 60 | SHARED 구역에서 BLOCKED일 때 뒤로 물러나기와 그 결과 | (없음) |
| Silent | silent.py:12 | 42 | 목표 없이 이동 단계에 머문 로봇, 소식 없는 로봇 경고. `_event`를 가로챔 | (없음) |
| NoStart | nostart.py:11 | 24 | 목표를 받고도 안 떠나는 로봇 경고(NO_START) | (없음) |
| Capacity | capacity.py:8 | 12 | SHARED 정원 합이 로봇 수보다 적으면 한 번 경고 | (없음) |
| Views | views.py:8 | 79 | 조회(robots, running, zones_view, robot_view, status_view)와 `take_output`. 조회는 읽기만 하지만 `take_output`은 `_out`을 비우고 `zones`는 변경 가능한 `ZoneGrants` 객체 자체를 돌려준다 | robots, everyone_placed, running, zones, available, take_output, zones_view, robot_view, status_view |

"줄"은 클래스 몸통 줄 수다(파일 앞쪽의 모듈 함수와 상수는 뺀다). "바깥에서 부르는 입구"는 `traffic/api.py`의 `TrafficApi` 이름과 맞춘 것이다. `test_traffic_api.py`가 FleetManager와 ROS 노드가 부르는 이름이 모두 API에 있는지 지킨다.

믹스인이 아닌 파일: robots.py(Robot, Goal, MoveOrder, Dispatch 자료형), trips.py(Trip 자료형, 경로 계산, 이동 명령 만들기), grants.py(ZoneGrants), lane_policy.py(LanePolicy, Want), base.py(ZoneBase), controller.py(ZoneTraffic). ZoneBase에는 도우미 13개가 있다: `_event`, `_gap`, `_link_reached`, `_on_node`, `_in_yard`, `_via`, `_priority`, `_goal_priority`, `_cancel_goal`, `_drop_trip`, `_stop_wanting`, `_fail`, `_goal_failed`.

## 2. tick 단계 순서

시험 하네스는 `ZoneTraffic.tick`(controller.py:34-43)을 부른다(`zone_harness.py:89`). 운영은 `FleetManager.tick`(manager.py:218-237)이 같은 부품(`tick_begin`, `tick_progress`, `tick_pick`, `step`)을 직접 부른다. 운영에서는 `ZoneTraffic.tick`을 부르지 않는다.

| 순서 | 단계 | ZoneTraffic.tick (시험) | FleetManager.tick (운영) | 차이 |
|---|---|---|---|---|
| 0 | 입력 기록 | - | `_input("tick")` (시각 `_time`은 항상 갱신, 기록 한 줄은 `_write`가 설정된 때만 씀: manager_record.py:40-45) | 운영만 |
| 1 | 시각 | `self._clock()` | `self._time` (입력 때 한 번 읽어 고정. 코어의 시계도 `_frozen`) | 시계 출처가 다르다 |
| 2 | tick_begin | `_check_capacity`, `_check_startup`, `_check_comm`, `_finish_recoveries` → `everyone_here` | 같다 | 운영은 뒤에 `_pump()` |
| 3 | 팔 시간 초과 | - | `arm.check_timeouts(now)` 뒤 `_pump()` | 운영만 |
| 4 | tick_progress | `_expire_blocked`, `_expire_marks`, `_ask_clear`; 로봇마다 available이면 `_judge_arrival`, `_watch_stall`, `_watch_idle`, `_watch_retreat_wait`; 로봇마다(available과 무관) `_watch_start`, `_watch_silent`; `_watch_near`; `_watch_deadlock` → `stopped` | 같다 (traffic 안의 같은 메서드) | 운영은 뒤에 `_pump()` |
| 5 | 호출 고르기 | `running`이고 `everyone_here`면 `tick_pick(stopped)` | 같다 | 운영은 뒤에 `_pump()` |
| 6 | 로봇 한 걸음 | `running`이고 `everyone_here`인 블록 안에서만(5단계와 같은 `if`), 로봇마다 available이고 `stopped`에 없으면 `self.step(robot)` | 같은 조건(controller.py:38-42, manager.py:228-233)에서 `_step_robot(robot)` = `orchestrator.step`, `_pump`, `traffic.step`, `_pump`, `orchestrator.step`, `_pump` (manager_route.py:19-25) | 운영은 주문 층이 앞뒤로 한 번씩 더 걷는다 |
| 7 | 출력 | `take_output()` | 매니저 자신의 `_out`을 꺼내 `_note("out")`을 부르고 반환 | `_note`는 `_write`가 설정된 때만 쓴다(manager_record.py:77-78) |

`_pump`(manager_route.py:27-33)는 팔, 주문, 교통 세 코어의 출력을 모아 알맞은 코어에 넘기기를 출력이 없을 때까지 되풀이한다. 그래서 운영에서는 한 단계가 낳은 명령이 같은 tick 안에서 다른 코어에 닿고, 그 답이 다음 단계 전에 반영된다. 하네스에는 이 되풀이가 없다. 하네스와 운영의 순서가 같다는 것을 지키는 시험은 찾지 못했다 [추측: 같은 순서를 확인하는 시험이 따로 없다. `grep -rn "tick_begin\|tick_progress" test`가 `test_zone_clear_checks.py`만 찾았다].

`stopped`는 `_watch_deadlock`의 반환값이다. 그 함수는 항상 빈 `frozenset()`을 반환한다(watch.py:136, 7절).

tick 말고 입력으로 들어오는 길도 있다. 가장 중요한 것은 `on_robot_state`(progress.py:31-63)다: `last_seen` 갱신, 배터리, LOCALIZING이면 중단, 자세 튐 감시(`_watch_pose_jump`), 위치 반영, 그다음 아직 배치 전이면 `_place`, 통신 끊김에서 돌아오면 `_begin_recovery`, 정상이면 `_track` → `_release_behind` → `_watch_outside` → `_watch_blocked`, 고장이면 `_return_unreached`. 구역을 놓는 일은 tick이 아니라 이 길에서 일어난다.

## 3. 공유 상태

`ZoneBase.__init__`(base.py:23-52)가 만드는 속성 29개와, 초기화에 없지만 Capacity가 처음 쓸 때 만드는 `_capacity_checked` 하나. 쓰는 곳 = 대입, 인덱스 대입, 삭제, 변경 메서드 호출(`append`, `pop`, `clear`, `add`, `update`, `setdefault` 등), 그리고 `grants`의 변경 메서드(`take`, `force`, `release_zone`, `release_node`, `stop_waiting`, `note_waiting`). 괄호 안은 그 클래스 안의 쓰기·읽기 줄 수다. 지역 변수에 받아 쓰는 경우(`grants = self.grants`)도 센다.

| 속성 | 선언 | 쓰는 곳 | 읽는 곳(쓰는 곳 제외) |
|---|---|---|---|
| `site` | base.py:24 | - | Goals(9), Turns(9), MoveOrders(9), Watch(6), Calls(5), BlockMarks(5), YardRetreat(5), Drive(4), Blocked(4), Redock(3), Progress(3), ZoneBase(3), Placement(2), Backoff(2), Detours(1), Health(1), NoStart(1), Capacity(1), Views(1) |
| `params` | base.py:25 | - | Results(8), Goals(7), BlockMarks(7), YardRetreat(5), Health(5), ZoneBase(5), Progress(4), Blocked(4), Watch(4), Backoff(4), OrderAnswers(3), Drive(2), MoveOrders(2), Placement(2), ClearChecks(2), Localization(2), Silent(2), Redock(1), NoStart(1) |
| `_clock` | base.py:26 | - | Goals(5), MoveOrders(4), Results(4), Progress(2), Health(2), ZoneBase(2), Calls(1), Turns(1), Drive(1), OrderAnswers(1), Redock(1), YardRetreat(1), ClearChecks(1), Backoff(1), ZoneTraffic(1) |
| `_next_goal_id` | base.py:27 | - | Drive(2), MoveOrders(2), Backoff(1) |
| `grants` | base.py:28 | Progress(2), Health(2), Watch(2), Goals(1), Drive(1), Placement(1), ZoneBase(1) | Turns(9), Views(5), Backoff(2), Calls(1) |
| `lanes` | base.py:29 | - | Goals(3) |
| `selector` | base.py:30 | - | Calls(1) |
| `_robots` | base.py:31 | Health(1) | Calls(8), Views(6), Watch(4), Turns(2), Results(2), Blocked(2), Goals(1), Drive(1), OrderAnswers(1), Progress(1), Placement(1), BlockMarks(1), YardRetreat(1), ClearChecks(1), Backoff(1), Silent(1), Capacity(1), ZoneTraffic(1) |
| `_work` | base.py:32 | - | Calls(6) |
| `_outbound` | base.py:33 | - | - |
| `_open_calls` | base.py:34 | Calls(3) | Drive(1) |
| `_out` | base.py:35 | Calls(6), ZoneBase(5), Drive(3), MoveOrders(2), ClearChecks(1), Health(1), Backoff(1), Views(1) | - |
| `_run_mode` | base.py:36 | Calls(1) | Watch(1), Silent(1), NoStart(1), Views(1) |
| `_started` | base.py:37 | Health(1) | - |
| `_startup_warned` | base.py:38 | Health(1) | - |
| `_no_robot_warned` | base.py:39 | Calls(1) | - |
| `_joining` | base.py:40 | Placement(1), Health(1) | Views(1) |
| `_wants` | base.py:41 | Goals(1), MoveOrders(1), Redock(1), Health(1), ZoneBase(1) | Watch(1), Views(1) |
| `_blocked` | base.py:42 | BlockMarks(3), Blocked(2) | Goals(3), Redock(1) |
| `_marks` | base.py:43 | BlockMarks(2) | ClearChecks(2) |
| `_mark_memory` | base.py:44 | BlockMarks(3), ClearChecks(1) | - |
| `_mark_seq` | base.py:45 | - | BlockMarks(1), ClearChecks(1) |
| `_cycle` | base.py:46 | Watch(2) | - |
| `_cycle_since` | base.py:47 | Watch(1) | - |
| `_cycle_acted` | base.py:48 | Watch(2) | - |
| `_near` | base.py:49 | Watch(3) | - |
| `order_fault` | base.py:50 | MoveOrders(1) | - |
| `clear_reason` | base.py:51 | Health(2) | - |
| `station_left` | base.py:52 | - | - |
| `_capacity_checked` | Capacity 안에서 대입 | Capacity(1) | - |

읽는 법과 주의.

- 읽기만 하는 속성(어느 믹스인도 쓰지 않음): `site`, `params`, `_clock`, `_next_goal_id`, `lanes`, `selector`, `_work`, `_mark_seq`. 이 중 `lanes`, `_mark_seq`, `_next_goal_id`는 호출로 안쪽 상태가 바뀐다. `self.lanes.note(...)`(goals.py:204)는 LanePolicy의 `last`를 바꾼다. `self._mark_seq()`는 번호를 하나 올린다. 표는 이런 호출을 쓰기로 세지 않는다.
- 쓰는 믹스인이 하나뿐인 속성: `_robots`(Health), `_open_calls`, `_run_mode`, `_no_robot_warned`(Calls), `_started`, `_startup_warned`, `clear_reason`(Health), `_marks`(BlockMarks), `_cycle`, `_cycle_since`, `_cycle_acted`, `_near`(Watch), `order_fault`(MoveOrders), `_capacity_checked`(Capacity). 소유가 분명하다.
- 여럿이 쓰는 속성: `grants`(7개 클래스), `_out`(8), `_wants`(5), `_blocked`(2), `_mark_memory`(2), `_joining`(2). 얽힘은 여기서 생긴다.
- `_out`은 대부분 `_event()`로 간접히 쓴다. 표에는 `self._out`을 직접 건드리는 곳만 들어 있다. `_event`를 부르는 믹스인은 21개다(4절).
- `_wants`(로봇이 기다리는 구역·노드)는 Goals가 만들고 ZoneBase의 `_stop_wanting`이 지운다. 그런데 MoveOrders, Redock, Health도 직접 쓴다.
- `_blocked`(막힌 링크 → 만료 시각)는 BlockMarks와 Blocked가 쓰고 Goals(경로 찾기 3곳)와 Redock이 읽는다. 막힘 처리의 가운데 매듭이다.
- 쓰는 곳도 읽는 곳도 없는 것: `_outbound`(base.py:33), `station_left`(base.py:52). 7절.
- `grants` 읽기 쪽은 Turns 9곳, Views 5곳이다(`held_by`, `holders`, `blockers`, `node_holder` 등).

로봇별 상태(`Robot`, robots.py:51)도 믹스인이 공유한다. 이것은 위 표에 없다. 필드는 77개다. `m.<필드> =`로 쓰는 믹스인을 필드 이름으로 세면, 대입이 한 번이라도 있는 필드 75개 중 49개는 쓰는 클래스가 하나이고 26개는 둘 이상이다(센 방법: 8절, 필드 이름만 맞춘 근사다). 가장 여러 곳이 쓰는 필드: `goal`(7개 클래스), `stall_from`, `trip`, `node`(각 6), `retry_at`, `phase`(각 5). 즉 "누가 로봇의 goal과 trip을 바꾸는가"는 믹스인 일곱 곳에 흩어져 있다. 분리할 때 가장 조심할 곳이다.

## 4. 믹스인 의존 지도

`self.<이름>`으로 다른 믹스인(또는 ZoneBase)에 정의된 메서드·프로퍼티를 부르는 (부르는 클래스, 제공 클래스) 쌍을 셌다. 믹스인 → 다른 믹스인 41쌍, 믹스인 → ZoneBase 22쌍, ZoneTraffic(tick 조립) → 13쌍, 합계 76쌍.

| 믹스인 | 줄 수 | 메서드 수 | 기대는 믹스인 수 | 부르는 남의 메서드(고유) | 자기를 부르는 믹스인 수 | 자기 메서드 중 남이 부르는 수 |
|---|---|---|---|---|---|---|
| Calls | 157 | 16 | 2 | 7 | 4 | 4 |
| Goals | 206 | 12 | 5 | 8 | 6 | 7 |
| Turns | 129 | 14 | 2 | 4 | 3 | 6 |
| Drive | 124 | 8 | 6 | 15 | 2 | 2 |
| MoveOrders | 139 | 11 | 4 | 12 | 2 | 3 |
| OrderAnswers | 113 | 10 | 1 | 2 | 3 | 6 |
| Results | 169 | 15 | 6 | 15 | 3 | 4 |
| Redock | 35 | 2 | 3 | 6 | 2 | 2 |
| Progress | 133 | 8 | 6 | 8 | 0 | 0 |
| Placement | 54 | 5 | 1 | 2 | 1 | 1 |
| Blocked | 163 | 8 | 5 | 17 | 2 | 2 |
| BlockMarks | 83 | 8 | 2 | 2 | 3 | 8 |
| YardRetreat | 86 | 7 | 3 | 7 | 4 | 6 |
| Detours | 12 | 2 | 1 | 1 | 2 | 2 |
| ClearChecks | 52 | 2 | 2 | 3 | 1 | 1 |
| Health | 117 | 8 | 2 | 5 | 2 | 4 |
| Watch | 139 | 7 | 5 | 14 | 5 | 5 |
| Localization | 12 | 1 | 1 | 1 | 1 | 1 |
| Backoff | 60 | 5 | 3 | 3 | 3 | 4 |
| Silent | 42 | 4 | 1 | 1 | 1 | 2 |
| NoStart | 24 | 1 | 1 | 2 | 1 | 1 |
| Capacity | 12 | 1 | 1 | 1 | 1 | 1 |
| Views | 79 | 10 | 0 | 0 | 2 | 4 |
| ZoneBase | 123 | 14 | 0 | 0 | 22 | 13 |
| ZoneTraffic | 37 | 3 | 13 | 21 | 0 | 0 |

- 열 뜻: "기대는 믹스인 수" = 메서드를 빌려 오는 다른 클래스 수(ZoneBase 포함). "부르는 남의 메서드" = 그 클래스들에서 부르는 고유 메서드 수. "자기를 부르는 믹스인 수" = 이 클래스의 메서드를 부르는 다른 믹스인 수. `_event`는 ZoneBase 것으로 센다.
- 가장 많이 기대는 믹스인(ZoneBase 도우미 제외, 믹스인에서만 빌려 온 고유 메서드 수): Blocked 11, Drive 8, Results 8, Watch 8, MoveOrders 7. ZoneBase를 더하면 Blocked 17, Drive 15, Results 15, Watch 14, MoveOrders 12다.
- 가장 많이 기댐을 받는 것은 ZoneBase(22개 믹스인), 그다음 믹스인 중에서는 Goals(Drive, MoveOrders, Redock, Turns, Watch, BlockMarks가 7개 메서드를 부름), Watch(4개), YardRetreat, Turns, Calls, OrderAnswers, Backoff(각 3개).
- 다른 믹스인에는 기대지 않는 믹스인(ZoneBase 도우미만 부름): OrderAnswers, Placement, Detours, Localization, Silent, NoStart, Capacity. Views는 아무것도 빌리지 않는다. Localization, Detours, Capacity, Silent는 `_event`만, NoStart와 Placement는 `_event`와 `_gap`만 쓴다.
- 아무도 부르지 않는 믹스인: Progress. 바깥(on_robot_state)에서만 들어온다.
- 맞부르는 쌍(서로 상대의 메서드를 부름): Goals-MoveOrders, Goals-Turns, Backoff-Results, Backoff-Watch.

가장 많이 불리는 메서드(부르는 믹스인 수 순):

| 메서드 | 정의 | 부르는 믹스인 수 | 호출 지점 수 |
|---|---|---|---|
| `_event` | Silent/ZoneBase | 21 | 72 |
| `_cancel_goal` | ZoneBase | 8 | 13 |
| `_via` | ZoneBase | 6 | 8 |
| `_drop_trip` | ZoneBase | 6 | 8 |
| `_gap` | ZoneBase | 5 | 12 |
| `_stop_wanting` | ZoneBase | 5 | 8 |
| `_on_node` | ZoneBase | 5 | 7 |
| `_take` | Goals | 4 | 4 |
| `_fail` | ZoneBase | 3 | 4 |
| `_to_parking` | Watch | 3 | 3 |
| `_in_yard` | ZoneBase | 3 | 3 |
| `_goal_failed` | ZoneBase | 3 | 3 |
| `_cost` | Goals | 2 | 4 |
| `available` | Views | 2 | 3 |
| `_goal_priority` | ZoneBase | 2 | 3 |

믹스인마다 무엇을 어디서 빌리는지(전체):

| 믹스인 | 부르는 곳 (제공자: 메서드) |
|---|---|
| Calls | ZoneBase: `_cancel_goal`, `_drop_trip`, `_fail`, `_on_node`, `_stop_wanting`, `_via`; Views: `available` |
| Goals | ZoneBase: `_event`, `_on_node`, `_stop_wanting`; Turns: `_candidates`, `_take_turn`; Detours: `_note_detour`; MoveOrders: `_through`; YardRetreat: `_retreat_held_up` |
| Turns | Goals: `_cost`, `_way`; ZoneBase: `_event`, `_on_node` |
| Drive | ZoneBase: `_cancel_goal`, `_drop_trip`, `_event`, `_goal_priority`, `_on_node`, `_stop_wanting`, `_via`; Goals: `_choose`, `_grab`, `_take`; MoveOrders: `_extend_order`, `_send_order`; Calls: `_head_for`; Redock: `_reapproach`; Turns: `_station_spots` |
| MoveOrders | ZoneBase: `_cancel_goal`, `_drop_trip`, `_event`, `_goal_priority`, `_via`; Goals: `_cost`, `_note_wait`, `_take`; Turns: `_dock_free`, `_is_next`, `_stay`; OrderAnswers: `_end_both` |
| OrderAnswers | ZoneBase: `_cancel_goal`, `_event` |
| Results | ZoneBase: `_drop_trip`, `_event`, `_gap`, `_goal_failed`, `_in_yard`, `_link_reached`, `_via`; OrderAnswers: `_old_goal_result`, `_order_result`, `_update_lost`, `_update_unclear`; Backoff: `_backoff_result`; Calls: `_finish_dispatch`; Redock: `_dock_failed`; YardRetreat: `_waits_behind` |
| Redock | ZoneBase: `_event`, `_goal_failed`, `_stop_wanting`; Goals: `_note_wait`, `_take`; Calls: `_leg_of` |
| Progress | ZoneBase: `_event`, `_gap`, `_on_node`; Blocked: `_watch_blocked`; Health: `_begin_recovery`; Localization: `_watch_pose_jump`; Placement: `_place`; Watch: `_watch_outside` |
| Placement | ZoneBase: `_event`, `_gap` |
| Blocked | BlockMarks: `_hold_key`, `_mark_point`, `_obstacle_point`, `_point_keys`, `_robot_at`, `_yielded_lately`; ZoneBase: `_cancel_goal`, `_drop_trip`, `_event`, `_fail`, `_in_yard`, `_via`; YardRetreat: `_behind_on_the_bent_way`, `_set_retreat`, `_yield_to_robot`; Detours: `_remember_way`; Watch: `_to_parking` |
| BlockMarks | Goals: `_reach`; ZoneBase: `_event` |
| YardRetreat | ZoneBase: `_cancel_goal`, `_drop_trip`, `_event`, `_fail`, `_in_yard`; Backoff: `_stand_aside`; Watch: `_to_parking` |
| Detours | ZoneBase: `_event` |
| ClearChecks | BlockMarks: `_drop_mark`, `_hold_key`; ZoneBase: `_event` |
| Health | ZoneBase: `_cancel_goal`, `_event`, `_link_reached`, `_stop_wanting`; OrderAnswers: `_hold_base` |
| Watch | ZoneBase: `_cancel_goal`, `_event`, `_gap`, `_goal_failed`, `_priority`, `_via`; Backoff: `_awaits_blocked`, `_backoff_ended`, `_stand_aside`; Goals: `_take`, `_way`; Results: `_arrived_anyway`, `_stands_at_end`; Drive: `_send` |
| Localization | ZoneBase: `_event` |
| Backoff | Results: `_active`; Watch: `_to_parking`; ZoneBase: `_event` |
| Silent | ZoneBase: `_event` |
| NoStart | ZoneBase: `_event`, `_gap` |
| Capacity | ZoneBase: `_event` |
| Views | - |
| ZoneBase | - |
| ZoneTraffic | Views: `available`, `robots`, `running`, `take_output`; Health: `_check_comm`, `_check_startup`, `_finish_recoveries`; Watch: `_watch_deadlock`, `_watch_near`, `_watch_stall`; Silent: `_watch_idle`, `_watch_silent`; BlockMarks: `_expire_marks`; Blocked: `_expire_blocked`; Calls: `tick_pick`; Capacity: `_check_capacity`; ClearChecks: `_ask_clear`; Drive: `step`; NoStart: `_watch_start`; Results: `_judge_arrival`; YardRetreat: `_watch_retreat_wait` |

ZoneTraffic 줄에는 tick이 부르는 13개 클래스의 입구가 들어 있다.

## 5. 구역 보유 규칙(grants)과 안전 불변식

### 5.1 누가 쥐고 누가 놓는가

기록은 `ZoneGrants`(grants.py) 하나에 있다. 믹스인은 이 객체의 메서드로만 쥐고 놓는다. `ZoneGrants`의 쓰기 메서드 6개를 부르는 곳을 모두 센 결과(10곳, 7개 클래스):

| 동작 | 메서드 | 부르는 곳 | 뜻 |
|---|---|---|---|
| 쥐기 | `take` | Goals goals.py:176 | 다음 구간의 구역·노드·차선을 모두 한꺼번에. 하나라도 막히면 아무것도 쥐지 않고 `_waiting`에 기록 |
| 억지로 올리기 | `force` | Placement placement.py:32, Watch watch.py:76 | 자리가 있을 때만. 시작 배치와 "쥔 구역 밖에 선 로봇이 서 있는 구역" 잡기 |
| 놓기 | `release_zone`, `release_node` | Progress progress.py:113, 120 (정상), Health health.py:87, 89 (운영자 confirm_clear) | 놓는 때의 규칙은 `docs/관제_구역교통.md` "구역을 놓는 때" |
| 대기 기록 | `note_waiting`, `stop_waiting` | Drive drive.py:95, Watch watch.py:159, ZoneBase base.py:112 | 쥐기에 실패한 로봇이 무엇을 기다리는지(교착 감시와 조회가 읽음) |

쥐기는 Goals 한 곳뿐이다. 놓기는 Progress(정상)와 Health(운영자) 두 곳뿐이다. 통신이 끊긴 로봇과 고장 로봇은 구역을 놓지 않는다(고장은 서 있는 자리에서 벗어난 구역만 `ZONES_RETURNED`로 돌려준다). 이 규칙의 설명은 `docs/관제_구역교통.md`에 있다.

### 5.2 ZoneGrants가 스스로 지키는 것

- 노드는 한 번에 한 로봇만 쥔다(`_nodes`, `blockers`가 다른 주인을 막는다).
- 구역은 정원(`spec.capacity`)까지만 쥔다. 도크 구역의 정원은 1이다(지도 파서가 1로 고정하고 다른 값은 오류로 거절한다, `fleet_map/parse.py:77-82`).
- 한 줄 구역(LANE)은 한 번에 한 방향이다. 방향이 다르면 막고, 같은 방향은 마지막 진입 뒤 `lane_follow_gap_s`가 지나고 정원 안일 때만 따라 들어간다(grants.py `_others_in_the_way`). 방향을 안 주면 `ValueError`다. 단 이미 그 구역을 쥔 로봇은 방향 검사 전에 통과한다(grants.py:33 근처 `if robot in holders: return ()`).
- `take`는 전부 아니면 전무다. 막히면 `_waiting`에 기록만 한다.
- `release_*`는 쥔 로봇만 놓을 수 있고, 구역이 비면 차선 방향을 지운다.
- 알아 둘 점: `force`는 차선 방향과 간격을 보지 않고 정원만 본다(grants.py `force`는 `_grant(..., None)`을 부른다). `force`는 기존 방향도 설정하지 않는다. 그래서 "한 번에 한 방향"은 `take` 경로의 규칙이다. 전체 보유 상태의 불변식이 아니다. 초기 배치(placement.py:23-32)가 노드가 있는 차선에 `force`를 쓸 수 있다.

### 5.3 불변식을 고정하는 시험

| 지키는 것 | 시험 | 방법 |
|---|---|---|
| 위 5.2의 구역·노드·차선 규칙 | `test_zone_grants.py` (시험 13개) | ZoneGrants를 직접 쓴다. 한꺼번에 잡기, 정원, 노드 하나, 차선 한 방향·간격·정원, 놓기, force, `acquired_at`, 모르는 구역은 프로그램 오류 |
| 시험 세계 안에서 (배치되어 있고 고장·통신 이상이 없는 로봇은) 쥔 구역 안에 있다, 한 줄 구역에서 명령 또는 order로 달리는 로봇의 `heading[0]` 반올림 값이 둘 이상 섞이지 않는다, 도크에 둘이 있지 않다 | `zone_harness.py`의 `_check()` (zone_harness.py:256-275, `step()`이 매번 부른다) | 위반하면 `world.problems`에 적는다. 이것을 `assert not world.problems`로 확인하는 시험은 `test_zone_*.py` 19개 파일에 있다(`grep -c problems`로 0이 아닌 파일 수). 하네스가 스스로 실패시키지는 않는다. 시험이 `problems`를 확인해야 지켜진다. 미배치·고장·통신 이상 로봇은 첫 검사에서 빠지고, 방향 검사는 달리지 않는 로봇과 `heading`의 x 성분만 본다. 모든 위반을 잡는다는 보장은 아니다 (zone_harness.py:256-275) |
| 같은 이름의 메서드를 믹스인이 두 번 정의하지 않는다 | `test_zone_mixins.py` (1개) | MRO를 돌며 이름이 겹치는 것을 센다. 허용은 `_event`뿐 |
| 교통 층이 주문·바구니 쪽을 부르지 않는다(import, 이름, 필드) | `test_fms_boundary.py` (16개) | ast로 import를 따라간다. traffic은 commands.py에서 허용 목록의 이름만 가져온다. 이 시험이 실제로 실패할 수 있음도 시험한다(반례 orchestrator) |
| FleetManager와 ROS 노드가 교통 층에 부르는 이름이 모두 `TrafficApi`에 있다 | `test_traffic_api.py` (5개) | ast로 `self.traffic.<이름>`, `core.traffic.<이름>`을 모은다 |
| 놓는 거리와 때 | `test_zone_release.py` (12개) | 몸 전체가 나갈 때까지, 도킹 오차, 마진 |
| 고장·격리된 로봇은 구역을 쥔 채로 호출을 받지 않는다 | `test_zone_isolation.py` (18개)의 `test_an_isolated_robot_keeps_its_zones_and_takes_no_call` 등 | 하네스 |
| 정원 경고 | `test_zone_capacity.py` (2개) | |
| 교착과 대기 순환 | `test_zone_watch.py` (15개), `test_deadlock.py` (4개) | |

`test_zone_*.py`는 33개 파일, `def test_` 373개다(`grep -c "def test_"` 합계. parametrize는 한 번으로 센다).

시험이 비어 있는 곳 [추측]: (a) 하네스의 `_check()`는 SHARED 구역 정원 초과와 노드 중복을 직접 보지 않는다. 그것은 ZoneGrants가 보장하고 `test_zone_grants.py`가 지킨다. 그러나 "믹스인이 `grants`를 거치지 않고 로봇을 움직인다"는 `_check()`의 첫 항목(쥔 구역 안에 있는가)이 있어야 잡힌다. (b) `force`가 차선 방향을 무시하는 것을 막는 시험은 보지 못했다.

## 6. 점진 분리 후보 (위험 낮은 순)

전제: 전면 재구성은 하지 않는다. 한 번에 하나씩 떼고, 떼기 전에 비교 도구(6.0)를 먼저 세운다. 아래 위험 평가는 코드를 읽고 쓴 판단이다. 실제로 떼어 본 것은 아니다.

### 6.0 먼저 세울 것: 같은 입력, 같은 출력 비교

세 가지가 이미 있다. zone 파일, zone_harness.py, zone_scenarios.py에서 `grep -n "random\|time.time\|monotonic"`가 아무것도 찾지 못했다. 시계와 목표 번호 생성기는 밖에서 주입된다(base.py:26-27). 그래서 초기 상태, 주입값, 호출 순서가 같으면 같은 출력이 나올 것이라고 본다 [추측: 실제로 두 번 돌려 비교해 보지는 않았다].

1. 하네스 시나리오: `zone_scenarios.py`의 `play(이름)`(serve_three 등). 떼기 전과 후의 `world.commands`, `world.events`와 매 걸음의 `grants.held_by(로봇)`, `grants.holders(구역)`를 기록해 비교한다. 지금은 이 기록을 남기는 도구가 없다. 새 시험 도구로 추가해야 한다.
2. `tools/replay.py`: FleetManager에 입력 기록(jsonl)을 다시 넣고 기록된 `in`, `inside`, `out` 줄을 순서대로 비교한다(첫 줄에 `"recorded": true`가 있는 기록만. 입력만 있는 기록은 비교하지 않고, 줄 중간에서 끊긴 기록은 끝 뒤를 비교하지 못한 채 남긴다. replay.py 머리말). 기록은 `fleet_manager_node --record`나 `FleetManager.record_to`로 만든다. 저장소에 기록 파일은 없다(`find . -name "*.jsonl"`이 0). 보유 구역은 이 비교에 들지 않는다.
3. `tools/situations.py`: 상황 x 배치 x 씨드 표. 결과 표가 같은지 본다.

분리 후보마다 "비교할 것"은 같다: 같은 입력에서 (가) 명령과 이벤트를 한 줄로 이은 목록과 순서(둘을 따로 비교하면 둘 사이의 순서 변경을 놓친다), (나) 매 tick 끝의 로봇별 보유 구역·노드. 후보 고유의 비교는 후보마다 따로 적었다.

### 6.1 Views → `ZoneViews` (위험 가장 낮음)

- 이유: 조회는 읽기만 한다(79줄). 쓰는 것은 `take_output`의 `_out` 교체 하나뿐이고, `zones`는 변경 가능한 `grants`를 그대로 내준다. `_robots`, `_wants`, `_joining`, `_run_mode`, `grants`, `site`를 읽는다.
- 떼는 법: `ZoneViews(traffic)` 객체를 만들고 `robot_view`, `status_view`, `zones_view`, `everyone_placed`(메서드가 아니라 프로퍼티), 두 뷰가 함께 쓰는 `_route_left`를 옮긴다. ZoneTraffic에는 같은 이름의 한 줄 위임을 남긴다. `available`, `running`, `robots`, `take_output`은 tick과 다른 믹스인이 `self.`로 부르니(2개 믹스인이 `available`) 그대로 둔다.
- 비교할 것: 시나리오 매 걸음의 `robot_view`, `status_view`, `zones_view` 반환값이 전과 같은 dict인지. 여기에 `test_zone_mixins.py`의 이름 중복 검사가 위임 때문에 깨지지 않는지.
- 위험: `robot_view`, `status_view`의 dict 키와 값은 ROS 노드와 매니저가 읽는다(`zones_view`는 dict가 아니라 `list[ZoneView]`). `test_traffic_api.py`는 호출 이름이 `TrafficApi`에 선언됐는지만 보고 ZoneTraffic에 구현이나 위임이 있는지는 보지 않으므로 위임을 빼먹어도 이 시험은 잡지 못한다. 위 비교와 `test_manager.py` 등이 실제로 부르는 곳에서 잡는다 [추측]. 상대적으로 낮다고 보지만 확정은 아니다.

### 6.2 작은 감시 셋: Capacity, Localization, NoStart (위험 낮음)

- 이유: 각각 메서드 하나(12, 12, 24줄). 읽는 것은 `params`, `site`, `_robots`, `_run_mode`와 로봇 레코드. 내는 것은 이벤트뿐이다. 다른 믹스인을 부르지 않고 ZoneBase의 `_event`, `_gap`만 쓴다(4절). 불리는 곳도 한 곳씩이다(controller.py:47, progress.py:48, controller.py:64).
- 떼는 법: 각각 `watch(m, now, emit)` 꼴의 작은 객체(또는 순수 함수와 필드 묶음)로 옮기고, 이벤트는 `emit` 콜백으로 낸다. Localization의 판정 `pose_jump`는 이미 순수 모듈 함수라 따로 시험하기 쉽다.
- 비교할 것: 이벤트 목록(`LOCALIZATION_SUSPECT`, `NO_START`, `ZONE_CAPACITY_LOW`)이 시각과 순서까지 같은지. `emit`은 `Silent._event`를 거쳐야 기존 부작용(`silent_from` 지우기)이 남는다. 로봇 필드 `start_goal`, `start_xy`, `start_since`, `start_done`(NoStart만 씀), `suspect_quiet_until`(Localization만 씀)이 같은지.
- 위험: Localization의 `pose_at`을 Progress도 지운다(progress.py:43, 47). Capacity는 `_capacity_checked`를 `getattr`로 늦게 만든다(7절). NoStart는 `m.move_order`, `m.arrival_by`, `m.goal`을 읽으므로 그 필드 이름이 바뀌면 깨진다. 낮다.
- Silent(42줄)와 Detours(12줄)도 비슷하게 작다. 그러나 Silent는 모든 이벤트를 가로채(`_event` 재정의, silent.py:14-19) 그 로봇의 `silent_from`을 지운다(로봇이 실제 로봇이고 코드가 SILENT_STALL이 아닐 때만). Detours는 Blocked와 Goals가 불러 이벤트 순서에 낀다. 그래서 이 둘은 6.2에서 뺐다.

### 6.3 막힘 표시 책: BlockMarks + ClearChecks → `MarkBook` (위험 중간)

- 이유: `_marks`, `_mark_memory`, `_mark_seq`를 이 두 믹스인만 쓴다(3절). ClearChecks는 `_ask_clear`와 `on_clear_check` 둘뿐이고, BlockMarks의 `_drop_mark`, `_hold_key`만 부른다.
- 떼는 법: `MarkBook`이 세 상태와 두 믹스인의 메서드를 갖는다. 바깥으로 낼 것은 이 네 동작만으로는 모자란다. Blocked가 장애물 위치, 로봇 찾기, 키 계산, 최근 양보 판정, 키 보유를 직접 부르고 `_mark_point`가 돌려준 `Mark`를 `_hold_key`에 다시 넘긴다(blocked.py:48, 103). ClearChecks는 출력 큐에 `CheckClearCmd`를 직접 넣고(clear_checks.py:27), 두 믹스인이 `_mark_memory` 목록을 재할당한다(clear_checks.py:48, block_marks.py:132). 그러니 참조 주입만으로 소유권이 옮겨지지 않는다. 먼저 이 호출들을 `MarkBook`의 공개 메서드로 정리하고 `_mark_memory`는 `MarkBook` 안에서만 바꾸게 한 뒤 떼야 한다.
- 비교할 것: 이벤트 `ZONE_MARKED_BLOCKED`, `BLOCK_CLEARED`, `BLOCK_CONFIRMED`, `REROUTE`, `CheckClearCmd`의 순서와 내용. 매 tick `_blocked` 사전이 같은지(Goals, Redock이 읽는다).
- 위험: `_blocked`는 Blocked도 쓴다(3절)고 Goals와 Redock이 읽는다. 참조를 공유해야 한다. BlockMarks는 Goals의 `_reach`를 부르고(4절) Blocked는 BlockMarks 메서드 6개를 부른다. 경계가 `_blocked`와 이 6개 메서드에 걸린다. 중간.

### 6.4 교착과 근접 감시 → `DeadlockWatch` (위험 중간)

- 이유: `_cycle`, `_cycle_since`, `_cycle_acted`, `_near`를 Watch만 쓴다. `_watch_deadlock`은 `_wants`와 `grants.blockers`로 대기 그래프를 만들어 순환을 찾는다(`_watch_near`는 다르다. 로봇 위치와 목표를 보고 `_cancel_goal`, `_awaits_blocked`(Backoff), `_stand_aside`(Backoff)를 부른다)(`traffic/deadlock.py`의 `find_cycle`, `choose_retreat`는 이미 순수 함수).
- 떼는 법: `_cycle*` 3개와 `_watch_deadlock`을 먼저 옮기고(`_evade`/`_to_parking`은 콜백), `_near`와 `_watch_near`는 Backoff 호출 때문에 따로 둔다. 둘을 한 객체로 묶으려면 `_cancel_goal`, `_awaits_blocked`, `_stand_aside` 콜백도 필요하다.
- 비교할 것: `DEADLOCK_SEEN` 이벤트와 시각, 비키는 로봇의 명령. 보유 구역.
- 위험: `_to_parking`은 Blocked, YardRetreat, Backoff도 부른다(4절 top 표). 그래서 `_to_parking`을 떼지 말고 콜백으로 둬야 한다. Backoff-Watch가 맞부르는 쌍이다. 중간.

### 6.5 하지 말 것

- Goals, MoveOrders, Turns, Drive, Results, Blocked, Backoff, Watch의 `_to_parking` 쪽은 서로 맞물려 있다(4절). 한꺼번에 떼면 보유 구역이 흔들린다.
- `grants`의 쓰기 호출 지점 10곳(클래스 7개)은 합치지 않는다. 이미 객체로 분리되어 있고 시험이 있다. 대신 6.0의 기록용 래퍼로 쓰기 호출을 로그로 남기는 것이 안전하다.
- 로봇 레코드(`Robot`)를 쪼개는 일은 후보에서 뺐다. `goal`, `trip` 같은 필드를 7곳이 쓴다(3절).

## 7. 코드를 읽다 찾은 이상한 점 (고치지 않음)

1. `_watch_deadlock`은 항상 빈 `frozenset()`을 반환한다(watch.py:136). 그래서 `tick_progress`의 반환값 `stopped`는 항상 비어 있고, `tick_pick(stopped)`와 `robot not in stopped`(controller.py:41, manager.py:232)는 아무 일도 하지 않는다. [확정] 의도된 자리표시인지 남은 코드인지는 모른다.
2. `ZoneBase._outbound`(base.py:33)는 zone 패키지 안에서 읽지 않는다. FleetManager와 orchestrator는 자기 `_outbound`를 따로 갖는다. [확정] 죽은 속성으로 보인다.
3. `station_left`(base.py:52)는 FleetManager가 `hasattr`로 보고 값을 넣어 준다(manager.py:46-47). 그러나 traffic 패키지 안에서 읽는 곳이 없다(`grep -rn station_left pc/fleet_core/fleet_core/traffic`이 base.py:52 하나). [확정] `test_traffic_api.py`는 이 이름을 OPTIONAL로 둔다.
4. `_capacity_checked`는 `__init__`에 없고 `getattr(self, "_capacity_checked", False)`로 늦게 만든다(capacity.py:10-12).
5. `_event`는 Silent가 가로챈다(silent.py:14-19). 로봇을 지정한 이벤트는(SILENT_STALL 제외) 그 로봇의 `silent_from`을 지운다(silent.py:14-17). Capacity 경고처럼 로봇이 없는 이벤트는 지우지 않는다. 이벤트 코드를 하나 추가하면 무진척 감시 시각이 리셋되는 숨은 결합이다.
6. `ZoneGrants.force`는 차선 방향과 간격을 보지 않는다(5.2).
7. 하네스의 불변식 검사 결과(`world.problems`)는 시험이 확인해야 효력이 있다(5.3).

## 8. 이 문서에서 센 것과 센 방법

| 숫자 | 방법 |
|---|---|
| 파일 30개, 3349줄 | zone 폴더의 `*.py` 파일 수(`ls`와 `wc -l`), `wc -l` 줄 합계 |
| 믹스인 23개, 클래스 몸통 2140줄, 메서드 165개 | ast로 `controller.py`에서 기반 클래스를 읽고, 각 클래스 `end_lineno - lineno + 1`과 `FunctionDef` 수 합 |
| 속성 29개 | ast로 `ZoneBase.__init__`의 `self.<x> = ...` 대입 |
| 쓰기/읽기 표 | 클래스 몸통마다 `self.<속성>`: 대입 대상(튜플 풀기 포함), AugAssign, Delete, 변경 메서드 호출(위 목록), `grants` 변경 메서드, 지역 별칭을 센다. 나머지 Load는 읽기 |
| 의존 쌍 76개(41+22+13) | 클래스 몸통마다 `self.<이름>` 중 이름이 다른 클래스에 정의된 메서드·프로퍼티인 것의 (부르는 클래스, 제공 클래스) 고유 조합. `_event`는 ZoneBase 것으로 센다 |
| 호출 지점 수(top 표) | 같은 `self.<이름>` 노드 수 |
| Robot 필드 77개, 한 클래스만 쓰는 49개, 둘 이상 26개 | ast로 `Robot`의 `AnnAssign` 이름을 모으고, `self`가 아닌 이름 `X.<필드> =`(AugAssign 포함)를 클래스별로 센다. 변수 이름이 아니라 필드 이름으로만 맞춘 근사다 |
| `grants` 쓰기 10곳 | 쓰기 메서드 6개 이름으로 zone 폴더를 grep해 ast 결과와 맞춤(grants.py 자신은 뺀다) |
| 시험 수 | `grep -c "def test_"` (parametrize 한 번) |

## 9. 비교 요약

위험 낮은 순: 6.1 Views, 6.2 작은 감시 셋, 6.3 막힘 표시 책, 6.4 교착과 근접 감시.
