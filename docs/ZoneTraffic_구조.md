# ZoneTraffic 구조 지도

기준: `refactor/zone-cleanup` 23f2211(2026-11-18 밤 정리 1부 5단계 끝). 정리 전 기준은 origin/main acfa45b다.
표에 든 숫자는 스크립트(ast)로 직접 센 값이다. 센 방법은 8절에 있다. 추측은 [추측]으로 표시했다.
줄 번호는 23f2211 기준이다. 코드가 바뀌면 어긋난다. 정리의 단계별 기록은 `docs/worklog_zone_cleanup_2026-11-18.md`에 있다.

이 브랜치(`refactor/yield-owner`)에서는 그 뒤에 2부 A2로 마당 물러나기를 지웠고, 아래 표에는 반영하지 않았다. 지워진 것: `Backoff`의 `_back_off`, `_awaits_blocked`, `_backoff_result`, `_backoff_ended`와 `backoff_spot`, goal 종류 `BACKOFF`, `Avoid.backoff_from`. `Backoff`에는 `_stand_aside`만 남았다(`docs/worklog_overnight_2026-11-18.md` 4절).

그 뒤 `refactor/remove-station-left`에서 아무도 읽지 않던 `ZoneBase.station_left`를 지웠다. 값을 넣던 `FleetManager.station_left_s`, 그것만 쓰던 `OrderOrchestrator.station_job`과 파라미터 `eta_fill_s_per_item`, `eta_fill_base_s`, `eta_swap_s`도 함께 지웠다. 3.1 표에서는 그 줄을 뺐지만 속성 수(31개)와 줄 번호는 지우기 전 값이다.

## 정리 전후

정리는 동작을 바꾸지 않았다. 명령, 이벤트, 순서, 보유 구역이 기준 기록(trace, situations, store, order_v7, order_d2 다섯 표와 시험별 하네스 기록)과 0줄 차이다.

| 항목 | 정리 전 (acfa45b) | 정리 후 (23f2211) |
|---|---|---|
| ZoneTraffic 기반 클래스 | 24개 (믹스인 23 + ZoneBase) | 17개 (믹스인 16 + ZoneBase) |
| 믹스인 클래스 몸통 줄 / 메서드 | 2292줄 / 178개 | 2032줄 / 157개 |
| 믹스인이 아닌 작은 객체 | 0 | 8 (ZoneViews, CapacityCheck, PoseJumpWatch, StartWatch, SilentWatch, DetourNotes, MarkBook, DeadlockWatch) |
| `Robot` 맨 위 필드 | 81 | 32 (나머지는 묶음 `report`, `health`, `avoid`, `watch`로) |
| `ZoneBase.__init__` 속성 | 30 | 31 (죽은 속성과 옮긴 상태 7개를 빼고 객체 8개를 더함) |
| `.goal =`이 있는 파일 | 7 | 1 (goal_life.py) |
| 가로채기(믹스인이 다른 믹스인의 메서드를 덮어씀) | 1 (`Silent._event`) | 0 |
| `test_zone_mixins.py`의 이름 중복 예외 | 1 (`_event`) | 0 |
| zone 폴더 파일 / 줄 | 30개 / 3512줄 | 32개 / 3674줄 |
| fleet_core 시험 | 1118 passed, 5 xfailed | 1124 passed, 5 xfailed |

파일별 줄 수(`wc -l`):

| 파일 | 전 | 후 | 바뀐 것 |
|---|---|---|---|
| __init__.py | 2 | 2 | |
| backoff.py | 111 | 111 | `.goal =`를 GoalLife로 |
| base.py | 145 | 169 | 조회 속성, `_event_once`, 객체 8개 만들기. `_cancel_goal`·`_outbound` 뺌 |
| block_marks.py | 142 | 163 | 믹스인 BlockMarks → 객체 MarkBook (ClearChecks를 합침) |
| blocked.py | 217 | 270 | BlockMarks의 로봇·지도 도우미 5개를 받음 |
| calls.py | 180 | 180 | |
| capacity.py | 19 | 24 | 믹스인 → CapacityCheck |
| clear_checks.py | 61 | (없음) | MarkBook으로 합침 |
| controller.py | 67 | 83 | 한 줄 위임 5개와 `on_clear_check` |
| deadlock_watch.py | (없음) | 36 | 새 DeadlockWatch |
| detours.py | 44 | 49 | 믹스인 → DetourNotes |
| drive.py | 152 | 150 | |
| episodes.py | (없음) | 38 | 새 Episode, WaitEpisode, BlockedEpisode |
| goal_life.py | (없음) | 31 | 새 GoalLife 믹스인(goal의 시작과 끝) |
| goals.py | 225 | 224 | |
| grants.py | 134 | 134 | |
| health.py | 128 | 131 | |
| lane_policy.py | 57 | 57 | |
| localization.py | 32 | 36 | 믹스인 → PoseJumpWatch |
| move_orders.py | 212 | 211 | |
| nostart.py | 34 | 39 | 믹스인 → StartWatch |
| order_answers.py | 127 | 128 | |
| placement.py | 70 | 70 | |
| progress.py | 168 | 168 | |
| redock.py | 55 | 56 | |
| results.py | 224 | 224 | |
| robots.py | 151 | 155 | `Robot` 필드 묶음 4개 |
| silent.py | 53 | 58 | 믹스인 → SilentWatch, 가로채기 없앰 |
| trips.py | 191 | 191 | |
| turns.py | 142 | 142 | |
| views.py | 94 | 82 | 믹스인 → ZoneViews |
| watch.py | 160 | 147 | 대기 순환 감지를 DeadlockWatch로 |
| yard_retreat.py | 115 | 115 | |
| 합계 | 3512 | 3674 | |

## 0. 한눈에

- ZoneTraffic은 관제 교통 층의 구현이다. 구역(zone)과 노드를 로봇에게 "허락"하고, 허락받은 구간만 이동 명령으로 내보낸다.
- `pc/fleet_core/fleet_core/traffic/zone/`에 파일 32개, 3674줄. 믹스인 16개(클래스 몸통 합계 2032줄, 메서드 157개) + ZoneBase(공통 상태와 도우미) + ZoneTraffic(조립, 위임, tick 3개).
- 믹스인이 아닌 작은 객체 8개가 감지와 조회를 맡는다(1.1절). 객체는 발견만 알리고, 취소·재시도·실패 결정은 믹스인 쪽에 남았다.
- 구역 허락 기록은 객체 `ZoneGrants`(grants.py) 하나가 쥔다. 쓰는 지점은 10곳, 클래스 6개(믹스인 5개와 ZoneBase)다(5절).
- 믹스인이 다른 믹스인의 메서드를 부르는 (부르는 클래스, 제공 클래스) 쌍은 46개다. ZoneBase 도우미를 부르는 믹스인은 15개다.
- goal을 세우고 지우는 곳은 GoalLife 하나다(`.goal =`은 goal_life.py에만 있다, `test_zone_goal_life.py`가 지킨다).
- 서로 맞부르는 쌍은 그대로 넷이다: Goals-MoveOrders, Goals-Turns, Backoff-Results, Backoff-Watch. 여기가 가장 얽힌 곳이다(6.5).

## 1. ZoneTraffic이 무엇인가

`controller.py:24-25`에서 믹스인 16개와 ZoneBase를 다중 상속한다. 앞에 쓴 클래스가 이름 충돌에서 이긴다(MRO). 같은 이름의 메서드를 두 번 정의하지 못하게 `test_zone_mixins.py`가 막는다. 예외는 없다(`_defined_twice(ZoneTraffic) == {}`).

믹스인에는 `__init__`이 없다. 모든 상태와 작은 객체 8개는 `ZoneBase.__init__`(base.py:31-64)이 만든다. 믹스인은 `self.<속성>`으로 그 상태를 직접 읽고 쓴다. 로봇별 상태는 `Robot` 레코드(robots.py)에 있다. 맨 위 필드 32개와 묶음 4개(`report`, `health`, `avoid`, `watch`)로 나뉜다(3.2절).

| 믹스인 | 파일 | 줄 | 맡은 일 한 줄 | 바깥에서 부르는 입구 |
|---|---|---|---|---|
| Calls | calls.py | 157 | 운행 모드, 배차 받기·취소·놓기, 열린 호출에 응할 로봇 고르기, 단계에 따라 다음 목적지로 돌리기 | on_run_mode, on_dispatch, on_dispatch_cancel, on_release, on_robot_fault, tick_pick |
| Goals | goals.py | 207 | 링크 비용과 경로 찾기, 후보 중 잡을 수 있는 트립 고르기, 다음 구간의 구역·노드·차선을 `grants.take`로 잡기 | (없음) |
| Turns | turns.py | 132 | 도크 대기 줄과 차례, 스테이션의 도크·대기·주차 지점 후보 | (없음) |
| Drive | drive.py | 130 | 로봇 단계(phase)별 한 걸음: 트립 따라 가기, 도착 판정, 이동·도킹·물러나기 명령 보내기 | step |
| MoveOrders | move_orders.py | 195 | order 모델(`use_order_model`)의 주문 조각(base, horizon) 만들기와 갱신 보내기 | (없음) |
| OrderAnswers | order_answers.py | 114 | order 피드백, 결과, 갱신 실패 처리, 통신 끊김 때 base 끝까지 갈 시간 | on_move_feedback |
| Results | results.py | 202 | goal 결과 처리(성공, 실패, 거절, 미전송, 잃음), 도착 확인, 구간 완료 | on_move_result, on_retreat_result, on_dock_result, on_goal_rejected, on_goal_not_sent, on_goal_lost |
| Redock | redock.py | 36 | 도킹 실패 후 물러났다가 다시 도크로 들어오는 트립 | (없음) |
| Progress | progress.py | 139 | 로봇 상태 보고 접수, 트립 진행, 지나온 구역·노드 놓기(`grants.release_*`), 배터리 | on_robot_state |
| Placement | placement.py | 54 | 시작할 때 로봇을 노드와 구역에 `grants.force`로 올려 놓기, 겹치면 거절 | (없음) |
| Blocked | blocked.py | 245 | BLOCKED 로봇 처리, 장애물 위치와 그 자리 로봇 찾기, 막을 링크 키, 다른 길로 다시 가기, 자가 복구 기록 | (없음) |
| YardRetreat | yard_retreat.py | 91 | 마당(SHARED)과 급행에서 물러날 길 정하기, 다른 로봇에게 양보 | (없음) |
| Health | health.py | 120 | 시작 보고, 통신 끊김과 복구, 운영자 치우기·되돌리기 | confirm_clear, reinstate_refusal, reinstate |
| Watch | watch.py | 126 | 움직임 없는 goal(GOAL_STALLED), 구역 밖, 가까이 선 두 로봇, 대기 순환에서 비킬 로봇 정하기, 주차 지점으로 보내기 | (없음) |
| Backoff | backoff.py | 60 | SHARED 구역에서 BLOCKED일 때 뒤로 물러나기와 그 결과 | (없음) |
| GoalLife | goal_life.py | 24 | goal 세우기(`_set_goal`), 지우기(`_clear_goal`), 취소 명령(`_cancel_goal`), 멈춤 일화 초기화 | (없음) |

"줄"은 클래스 몸통 줄 수다(파일 앞쪽의 모듈 함수와 상수는 뺀다). "바깥에서 부르는 입구"는 `traffic/api.py`의 `TrafficApi` 이름과 맞춘 것이다. `test_traffic_api.py`가 FleetManager와 ROS 노드가 부르는 이름이 모두 API에 있는지 지킨다.

ZoneTraffic 자신(controller.py)은 조회 위임 5개(`everyone_placed`, `live_block`, `zones_view`, `robot_view`, `status_view`), `on_clear_check`, 그리고 `tick`, `tick_begin`, `tick_progress`를 갖는다. ZoneBase(base.py)에는 조회 속성(`robots`, `running`, `zones`, `available`, `take_output`)과 도우미가 있다: `_event`, `_event_once`, `_gap`, `_link_reached`, `_on_node`, `_in_yard`, `_via`, `_priority`, `_goal_priority`, `_drop_trip`, `_stop_wanting`, `_fail`, `_goal_failed`.

믹스인이 아닌 자료형 파일: robots.py(Robot과 묶음, Goal, MoveOrder, Dispatch), episodes.py(Episode, WaitEpisode, BlockedEpisode), trips.py(Trip, 경로 계산, 이동 명령 만들기), grants.py(ZoneGrants), lane_policy.py(LanePolicy, Want).

### 1.1 작은 객체 8개

모두 `ZoneBase.__init__`이 만든다. 상태는 ZoneTraffic과 같은 사전 객체를 참조로 나눠 쓴다(그 사전들은 다시 대입하지 않는다). 이벤트는 콜백 `emit`(= `ZoneBase._event`)으로 내므로 이벤트 순서와 `SilentWatch.note`의 초기화가 전과 같다.

| 객체 | 파일 | 스스로 갖는 상태 | 나눠 쓰는 것 | 콜백 | 부르는 곳 |
|---|---|---|---|---|---|
| ZoneViews | views.py | 없음 | site, grants, `_robots`, `_joining`, `_wants`, `_blocked`, `_marks` | 없음 | ZoneTraffic의 한 줄 위임 5개 |
| CapacityCheck | capacity.py | `checked` | site | emit | `tick_begin` |
| PoseJumpWatch | localization.py | 없음(로봇의 `health.pose_at`, `health.suspect_quiet_until`을 씀) | params | emit | `on_robot_state`(Progress) |
| StartWatch | nostart.py | 없음(로봇의 `watch.start`를 씀) | params, site | emit | `tick_progress` |
| SilentWatch | silent.py | 없음(로봇의 `watch.idle`, `watch.silent`를 씀) | params, `_robots` | emit, emit_once | `ZoneBase._event`(`note`), `tick_progress`(`watch_idle`, `watch_silent`) |
| DetourNotes | detours.py | 없음(로봇의 `avoid.detour`를 씀) | site | emit | Blocked `_mark_blocked`(`remember`), Goals(`note`) |
| MarkBook | block_marks.py | `_memory`(지운 표시의 기억), `_seq`(표시와 확인 번호) | params, `_blocked`, `_marks`, `_robots` | clock | `tick_progress`(`expire`, `ask`), Blocked `_mark_point`·`_mark_blocked`(`mark`, `hold`), ZoneTraffic `on_clear_check`(`answer`) |
| DeadlockWatch | deadlock_watch.py | `cycle`, `since`, `acted` | params, grants, `_wants`, `_robots` | emit | Watch `_watch_deadlock`(`due`) |

MarkBook은 이벤트를 직접 내지 않는다. `mark`는 표시를 돌려주고, `ask`는 `CheckClearCmd` 목록을, `answer`는 이벤트 한 건(또는 None)을 돌려준다. 내보내기는 ZoneTraffic 쪽(`_mark_point`, `tick_progress`, `on_clear_check`)이 같은 자리에서 한다. DeadlockWatch는 순환을 찾아 `DEADLOCK_SEEN`을 내고, 유예가 지나면 순환을 돌려준다. 어느 로봇을 비키게 할지(`choose_retreat`, `_evade`)는 Watch가 정한다.

## 2. tick 단계 순서

시험 하네스는 기본으로 `FleetManager.tick`을 부른다(`zone_harness.py`의 `TICK_ORDER` 기본값 `"manager"`). `ZONE_TICK_ORDER=zone`이면 `ZoneTraffic.tick`(controller.py:50-59)을 부른다. 운영은 `FleetManager.tick`(manager.py:218-237)이 같은 부품(`tick_begin`, `tick_progress`, `tick_pick`, `step`)을 직접 부른다. 운영에서는 `ZoneTraffic.tick`을 부르지 않는다.

한 tick(ZoneTraffic.tick 기준, 운영의 차이는 오른쪽):

```
now = clock()                                    운영: now = _time(입력 때 고정)
│
├─ tick_begin(now) → everyone_here               운영: 뒤에 _pump()
│    CapacityCheck.check(로봇 수)                  처음 한 번만 ZONE_CAPACITY_LOW
│    Health._check_startup → everyone_here
│    Health._check_comm
│    Health._finish_recoveries
│                                                 운영: arm.check_timeouts(now), _pump()
├─ tick_progress(now) → stopped(늘 빈 집합)       운영: 뒤에 _pump()
│    Blocked._expire_blocked
│    MarkBook.expire                               표시 만료, 기억 정리
│    _out += MarkBook.ask                          CheckClearCmd
│    로봇마다:
│      available이면  Results._judge_arrival
│                     Watch._watch_stall
│                     SilentWatch.watch_idle
│                     YardRetreat._watch_retreat_wait
│      언제나         StartWatch.watch
│                     SilentWatch.watch_silent
│    Watch._watch_near
│    Watch._watch_deadlock ── DeadlockWatch.due → (유예 지남) choose_retreat, _evade
│
├─ running이고 everyone_here이면
│    Calls.tick_pick(stopped)                     운영: 뒤에 _pump()
│    로봇마다 available이고 stopped에 없으면
│      Drive.step(robot)                          운영: orchestrator.step, _pump, traffic.step,
│                                                       _pump, orchestrator.step, _pump
└─ take_output()                                  운영: 매니저의 _out을 꺼내 _note("out")
```

`_pump`(manager_route.py)는 팔, 주문, 교통 세 코어의 출력을 모아 알맞은 코어에 넘기기를 출력이 없을 때까지 되풀이한다. 그래서 운영에서는 한 단계가 낳은 명령이 같은 tick 안에서 다른 코어에 닿는다. `zone` 순서의 하네스에는 이 되풀이가 없다. 두 순서의 비교와 tick 경계 시험은 `test_zone_tick_order.py`에 있다.

`stopped`는 `_watch_deadlock`의 반환값이고 늘 빈 `frozenset()`이다(7절 1).

tick 말고 입력으로 들어오는 길도 있다. 가장 중요한 것은 `on_robot_state`(progress.py:31-63)다: `last_seen` 갱신, 배터리, LOCALIZING이면 중단, 자세 튐 감시(`PoseJumpWatch.watch`), 위치 반영, 그다음 아직 배치 전이면 `_place`, 통신 끊김에서 돌아오면 `_begin_recovery`, 정상이면 `_track` → `_release_behind` → `_watch_outside` → `_watch_blocked`, 고장이면 `_return_unreached`. 구역을 놓는 일은 tick이 아니라 이 길에서 일어난다.

## 3. 공유 상태

### 3.1 ZoneBase 속성

`ZoneBase.__init__`(base.py:31-64)가 만드는 속성 31개. 쓰는 곳 = 대입, 인덱스 대입, 삭제, 변경 메서드 호출(`append`, `extend`, `pop`, `clear`, `add`, `update`, `setdefault` 등), 그리고 `grants`의 변경 메서드(`take`, `force`, `release_zone`, `release_node`, `stop_waiting`, `note_waiting`). 괄호 안은 그 클래스 안의 쓰기·읽기 지점 수다. `__init__`의 첫 대입은 세지 않는다. 지역 변수에 받아 쓰는 경우(`grants = self.grants`)는 이 표에 들지 않는다(그런 쓰기는 5.1에 있다). "객체" 열은 참조를 나눠 받은 작은 객체다.

| 속성 | 쓰는 믹스인 | 읽는 믹스인 | 객체 |
|---|---|---|---|
| `site` | - | MoveOrders(11), Blocked(9), Goals(9), Turns(9), Calls(6), Watch(6), YardRetreat(5), Drive(4), Progress(4), Redock(3), Backoff(2), Placement(2), Health(1) | ZoneViews, CapacityCheck, StartWatch, DetourNotes |
| `params` | - | Blocked(10), Results(8), Goals(7), Health(5), Progress(5), YardRetreat(5), Backoff(4), Drive(3), OrderAnswers(3), Watch(3), MoveOrders(2), Placement(2), GoalLife(1), Redock(1) | PoseJumpWatch, StartWatch, SilentWatch, MarkBook, DeadlockWatch |
| `_clock` | - | Goals(5), MoveOrders(5), Results(4), Health(2), Progress(2), 그 밖 9개 클래스 각 1(ZoneTraffic 포함) | MarkBook |
| `_next_goal_id` | - | Drive(2), MoveOrders(2), Backoff(1) | |
| `grants` | Health(2), Watch(2), Drive(1), Placement(1), ZoneBase(1) | Turns(9), Goals(5), Backoff(2), Progress(2), Calls(1) | ZoneViews, DeadlockWatch |
| `lanes` | - | Goals(3) | |
| `selector` | - | Calls(1) | |
| `_robots` | Health(2) | Calls(8), Watch(3), Blocked(2), Results(2), Turns(2), ZoneTraffic(2), 그 밖 7개 클래스 각 1 | ZoneViews, SilentWatch, MarkBook, DeadlockWatch |
| `_work` | - | Calls(6) | |
| `_open_calls` | Calls(3) | Drive(1) | |
| `_out` | Calls(6), ZoneBase(4), Drive(3), GoalLife(2), MoveOrders(2), Backoff(1), Health(1), ZoneTraffic(1) | - | |
| `_run_mode` | Calls(1) | Watch(1), ZoneBase(`running`) | (StartWatch, SilentWatch는 `running`을 인자로 받는다) |
| `_started` | Health(1) | - | |
| `_startup_warned` | Health(1) | - | |
| `_no_robot_warned` | Calls(1) | - | |
| `_joining` | Health(1), Placement(1) | - | ZoneViews |
| `_wants` | Goals(2), MoveOrders(2), Redock(2), ZoneBase(1), Health(1) | - | ZoneViews, DeadlockWatch |
| `_blocked` | Blocked(4) | Goals(3), Redock(1) | MarkBook(씀), ZoneViews(읽음) |
| `_marks` | - | - | MarkBook(씀), ZoneViews(읽음) |
| `_near` | Watch(6) | - | |
| `order_fault` | MoveOrders(1) | - | |
| `clear_reason` | Health(2) | - | |
| `_views` | - | ZoneTraffic(5) | |
| `_capacity` | - | ZoneTraffic(1) | |
| `_pose_watch` | - | Progress(1) | |
| `_start_watch` | - | ZoneTraffic(1) | |
| `_silent` | - | ZoneTraffic(2), ZoneBase(1) | |
| `_detours` | - | Blocked(1), Goals(1) | |
| `_mark_book` | - | ZoneTraffic(3), Blocked(2) | |
| `_deadlock` | - | Watch(1) | |

정리 전과 달라진 것:

- 없어진 속성: `_outbound`(죽은 속성), `_capacity_checked`(CapacityCheck의 `checked`로), `_mark_memory`와 `_mark_seq`(MarkBook 안으로), `_cycle`, `_cycle_since`, `_cycle_acted`(DeadlockWatch 안으로).
- `_mark_memory`를 두 믹스인이 다시 대입하던 것이 없어졌다. 지금은 MarkBook만 바꾼다.
- `_marks`를 쓰는 곳은 MarkBook 하나다. `_blocked`는 Blocked(링크 막기와 만료)와 MarkBook(표시의 키 보유와 해제)이 쓰고 Goals(경로 찾기)와 Redock이 읽는다. 막힘 처리의 가운데 매듭은 그대로다.
- 여럿이 쓰는 속성: `grants`(5개 클래스, 지역 별칭까지 6개), `_out`(8), `_wants`(5), `_joining`(2), `_blocked`(Blocked와 MarkBook). 얽힘은 여기서 생긴다.
- `_out`은 대부분 `_event()`로 간접히 쓴다. 표에는 `self._out`을 직접 건드리는 곳만 들어 있다.

### 3.2 로봇 레코드: 필드 묶음과 쓰는 모듈

`Robot`(robots.py)의 맨 위 필드는 32개다. 정리 전에는 81개가 한 층에 있었다. 묶음 넷과 goal·trip 쪽 필드를 누가 쓰는지 모듈(파일) 단위로 센 표다. "쓴다" = `X.<묶음>.<필드> =`, `X.<필드> =`(AugAssign 포함). Episode 묶음은 읽기까지 센다(`test_zone_episodes.py`가 이 목록을 지킨다).

| 필드 묶음 | 안의 필드 | 쓰는 모듈 |
|---|---|---|
| `report` (Report) | robot_state, battery, charge, last_seen, local_yield, obstacle | progress.py (health.py `reinstate`는 새 Report를 만든다) |
| `health` (Health) | fault, fault_reason, fault_xy, pose_doubt, comm, recover_at, pose_at, suspect_quiet_until | base.py(`_fail`), health.py, localization.py, placement.py, progress.py |
| `avoid` (Avoid) | yield_until, yielded, retreat_to, retreat_via, retreat_since, retreat_behind, backoff_from, detour, robot_yield_at, behind_since | backoff.py, calls.py, detours.py, health.py, order_answers.py, results.py, yard_retreat.py |
| `watch.start` | Episode | nostart.py |
| `watch.silent`, `watch.idle` | Episode | silent.py |
| `watch.stall` | Episode | goal_life.py(초기화), watch.py |
| `watch.outside` | Episode | watch.py |
| `watch.wait` | WaitEpisode | base.py, goals.py, silent.py, turns.py, views.py |
| `watch.local_episode` | Episode | blocked.py |
| `watch.redock` | Episode | calls.py, deadlock_watch.py, drive.py, redock.py |
| `watch.blocked` | BlockedEpisode | blocked.py, yard_retreat.py |
| `goal` | | goal_life.py |
| `move_order` | | goal_life.py, move_orders.py, results.py |
| `settle_until`, `arrival_by` | | goal_life.py (`arrival_by`는 results.py도) |
| `trip`(과 그 안) | | base.py, calls.py, drive.py, goal_life.py, health.py, move_orders.py, progress.py, redock.py, results.py, watch.py |
| `phase` | | calls.py, drive.py, placement.py, redock.py, results.py |
| `node` | | health.py, order_answers.py, placement.py, progress.py, results.py, watch.py |
| `x`, `y`, `yaw`, `report` 갱신 | | progress.py |

`trip`, `phase`, `node`는 여전히 여러 모듈이 쓴다. goal과 달리 이것들은 한 곳으로 모으지 않았다(6.5).

## 4. 믹스인 의존 지도

`self.<이름>`으로 다른 믹스인(또는 ZoneBase)에 정의된 메서드·프로퍼티를 부르는 (부르는 클래스, 제공 클래스) 쌍을 셌다. 믹스인 → 다른 믹스인 46쌍(GoalLife를 믹스인으로 센다), 믹스인 → ZoneBase 15쌍, ZoneTraffic(tick 조립) → 8쌍, 합계 69쌍. 정리 전 기준 문서(7c7d04b)는 41+22+13=76쌍이었다. ZoneBase를 부르던 작은 믹스인 일곱이 객체가 되었고, 흩어진 goal 쓰기가 GoalLife 호출로 바뀌어 믹스인 사이 쌍은 늘었다.

| 믹스인 | 줄 | 메서드 | 기대는 클래스 수 | 빌리는 메서드 수 | 자기를 부르는 믹스인 수 | 빌리는 것 (제공자: 메서드) |
|---|---|---|---|---|---|---|
| Calls | 157 | 16 | 2 | 8 | 3 | ZoneBase: `_drop_trip`, `_fail`, `_on_node`, `_stop_wanting`, `_via`, `available`; GoalLife: `_cancel_goal`, `_reset_stall` |
| Goals | 207 | 12 | 4 | 9 | 6 | ZoneBase: `_event`, `_event_once`, `_on_node`, `_stop_wanting`; MoveOrders: `_through`, `_way_after`; Turns: `_candidates`, `_take_turn`; YardRetreat: `_retreat_held_up` |
| Turns | 132 | 14 | 2 | 4 | 3 | Goals: `_cost`, `_way`; ZoneBase: `_event`, `_on_node` |
| Drive | 130 | 8 | 7 | 18 | 1 | ZoneBase: `_drop_trip`, `_event`, `_goal_priority`, `_on_node`, `_stop_wanting`, `_via`; MoveOrders: `_extend_order`, `_held_end`, `_send_order`, `_turn_order`; Goals: `_choose`, `_grab`, `_take`; GoalLife: `_cancel_goal`, `_set_goal`; Calls: `_head_for`; Redock: `_reapproach`; Turns: `_station_spots` |
| MoveOrders | 195 | 15 | 5 | 15 | 2 | ZoneBase: `_drop_trip`, `_event`, `_goal_priority`, `_in_yard`, `_on_node`, `_via`; Goals: `_cost`, `_note_wait`, `_take`; Turns: `_dock_free`, `_is_next`, `_stay`; GoalLife: `_cancel_goal`, `_set_goal`; OrderAnswers: `_end_both` |
| OrderAnswers | 114 | 10 | 2 | 3 | 3 | GoalLife: `_cancel_goal`, `_set_goal`; ZoneBase: `_event` |
| Results | 202 | 18 | 7 | 17 | 2 | ZoneBase: `_drop_trip`, `_event`, `_gap`, `_goal_failed`, `_in_yard`, `_link_reached`, `_via`; OrderAnswers: `_old_goal_result`, `_order_result`, `_update_lost`, `_update_unclear`; GoalLife: `_clear_goal`, `_set_goal`; Backoff: `_backoff_result`; Calls: `_finish_dispatch`; Redock: `_dock_failed`; YardRetreat: `_waits_behind` |
| Redock | 36 | 2 | 3 | 6 | 2 | ZoneBase: `_event`, `_goal_failed`, `_stop_wanting`; Goals: `_note_wait`, `_take`; Calls: `_leg_of` |
| Progress | 139 | 8 | 5 | 7 | 0 | ZoneBase: `_event`, `_gap`, `_on_node`; Blocked: `_watch_blocked`; Health: `_begin_recovery`; Placement: `_place`; Watch: `_watch_outside` |
| Placement | 54 | 5 | 1 | 2 | 1 | ZoneBase: `_event`, `_gap` |
| Blocked | 245 | 17 | 5 | 13 | 1 | ZoneBase: `_drop_trip`, `_event`, `_event_once`, `_fail`, `_in_yard`, `_via`; YardRetreat: `_behind_along`, `_behind_on_the_bent_way`, `_set_retreat`, `_yield_to_robot`; GoalLife: `_cancel_goal`; Goals: `_reach`; Watch: `_to_parking` |
| YardRetreat | 91 | 8 | 4 | 7 | 3 | ZoneBase: `_drop_trip`, `_event`, `_fail`, `_in_yard`; Backoff: `_stand_aside`; GoalLife: `_cancel_goal`; Watch: `_to_parking` |
| Health | 120 | 8 | 3 | 6 | 1 | ZoneBase: `_event`, `_link_reached`, `_stop_wanting`; GoalLife: `_cancel_goal`, `_reset_stall`; OrderAnswers: `_hold_base` |
| Watch | 126 | 7 | 6 | 16 | 4 | ZoneBase: `_event`, `_event_once`, `_gap`, `_goal_failed`, `_priority`, `_via`; Backoff: `_awaits_blocked`, `_backoff_ended`, `_stand_aside`; GoalLife: `_cancel_goal`, `_reset_stall`; Goals: `_take`, `_way`; Results: `_arrived_anyway`, `_stands_at_end`; Drive: `_send` |
| Backoff | 60 | 5 | 4 | 5 | 3 | GoalLife: `_clear_goal`, `_set_goal`; Results: `_active`; Watch: `_to_parking`; ZoneBase: `_event` |
| GoalLife | 24 | 4 | 0 | 0 | 10 (+ ZoneBase) | - |
| ZoneBase | 140 | 19 | 1 | 1 | 15 | GoalLife: `_cancel_goal` (`_fail`이 부른다) |
| ZoneTraffic | 60 | 9 | 8 | 16 | 0 | ZoneBase: `_event`, `available`, `robots`, `running`, `take_output`; Health: `_check_comm`, `_check_startup`, `_finish_recoveries`; Watch: `_watch_deadlock`, `_watch_near`, `_watch_stall`; Blocked: `_expire_blocked`; Calls: `tick_pick`; Drive: `step`; Results: `_judge_arrival`; YardRetreat: `_watch_retreat_wait` |

- 열 뜻: "기대는 클래스 수" = 메서드를 빌려 오는 다른 클래스 수(ZoneBase 포함). "빌리는 메서드 수" = 그 클래스들에서 부르는 고유 메서드 수. "자기를 부르는 믹스인 수" = 이 클래스의 메서드를 부르는 다른 믹스인 수(ZoneTraffic 제외).
- 작은 객체 호출(`self._mark_book.mark` 등)은 이 표에 들지 않는다. 그것은 1.1절 표에 있다.
- 가장 많이 기대는 믹스인: Drive 18, Results 17, Watch 16, MoveOrders 15, Blocked 13.
- 가장 많이 기댐을 받는 것: ZoneBase(15개 믹스인), GoalLife(10개 믹스인과 ZoneBase), Goals(6), Watch(4).
- 아무도 부르지 않는 믹스인: Progress. 바깥(on_robot_state)에서만 들어온다.
- 맞부르는 쌍(서로 상대의 메서드를 부름): Goals-MoveOrders, Goals-Turns, Backoff-Results, Backoff-Watch. 정리 전과 같다.

가장 많이 불리는 메서드(부르는 믹스인 수 순):

| 메서드 | 정의 | 부르는 믹스인 수 | 호출 지점 수 |
|---|---|---|---|
| `_event` | ZoneBase | 14 | 64 |
| `_cancel_goal` | GoalLife | 9 | 15 |
| `_drop_trip` | ZoneBase | 6 | 9 |
| `_on_node` | ZoneBase | 6 | 8 |
| `_via` | ZoneBase | 6 | 7 |
| `_stop_wanting` | ZoneBase | 5 | 8 |
| `_set_goal` | GoalLife | 5 | 6 |
| `_gap` | ZoneBase | 4 | 13 |
| `_in_yard` | ZoneBase | 4 | 4 |
| `_take` | Goals | 4 | 4 |
| `_fail` | ZoneBase | 3 | 4 |
| `_reset_stall` | GoalLife | 3 | 4 |
| `_event_once` | ZoneBase | 3 | 3 |
| `_goal_failed` | ZoneBase | 3 | 3 |
| `_to_parking` | Watch | 3 | 3 |

(`_cancel_goal`의 9에는 ZoneBase `_fail`이 든다.)

## 5. 구역 보유 규칙(grants)과 안전 불변식

### 5.1 누가 쥐고 누가 놓는가

기록은 `ZoneGrants`(grants.py) 하나에 있다. 믹스인은 이 객체의 메서드로만 쥐고 놓는다. 작은 객체 중 `grants`를 받는 ZoneViews와 DeadlockWatch는 읽기 메서드(`holders`, `held_by`, `nodes_of`, `acquired_at`, `blockers`)만 부른다. `ZoneGrants`의 쓰기 메서드 6개를 부르는 곳을 모두 센 결과(10곳, 6개 클래스):

| 동작 | 메서드 | 부르는 곳 | 뜻 |
|---|---|---|---|
| 쥐기 | `take` | Goals goals.py:178 | 다음 구간의 구역·노드·차선을 모두 한꺼번에. 하나라도 막히면 아무것도 쥐지 않고 `_waiting`에 기록 |
| 억지로 올리기 | `force` | Placement placement.py:32, Watch watch.py:72 | 자리가 있을 때만. 시작 배치와 "쥔 구역 밖에 선 로봇이 서 있는 구역" 잡기 |
| 놓기 | `release_zone`, `release_node` | Progress progress.py:119, 126 (정상), Health health.py:89, 91 (운영자 confirm_clear) | 놓는 때의 규칙은 `docs/관제_구역교통.md` "구역을 놓는 때" |
| 대기 기록 | `note_waiting`, `stop_waiting` | Drive drive.py:99, Watch watch.py:146, ZoneBase base.py:139 | 쥐기에 실패한 로봇이 무엇을 기다리는지(교착 감시와 조회가 읽음) |

쥐기는 Goals 한 곳뿐이다. 놓기는 Progress(정상)와 Health(운영자) 두 곳뿐이다. 통신이 끊긴 로봇과 고장 로봇은 구역을 놓지 않는다(고장은 서 있는 자리에서 벗어난 구역만 `ZONES_RETURNED`로 돌려준다). 이 규칙의 설명은 `docs/관제_구역교통.md`에 있다.

### 5.2 ZoneGrants가 스스로 지키는 것

- 노드는 한 번에 한 로봇만 쥔다(`_nodes`, `blockers`가 다른 주인을 막는다).
- 구역은 정원(`spec.capacity`)까지만 쥔다. 도크 구역의 정원은 1이다(지도 파서가 1로 고정하고 다른 값은 오류로 거절한다, `fleet_map/parse.py`).
- 한 줄 구역(LANE)은 한 번에 한 방향이다. 방향이 다르면 막고, 같은 방향은 마지막 진입 뒤 `lane_follow_gap_s`가 지나고 정원 안일 때만 따라 들어간다(grants.py `_others_in_the_way`). 방향을 안 주면 `ValueError`다. 단 이미 그 구역을 쥔 로봇은 방향 검사 전에 통과한다(`if robot in holders: return ()`).
- `take`는 전부 아니면 전무다. 막히면 `_waiting`에 기록만 한다.
- `release_*`는 쥔 로봇만 놓을 수 있고, 구역이 비면 차선 방향을 지운다.
- 알아 둘 점: `force`는 차선 방향과 간격은 보지 않는다. 구역 정원과 노드 점유(다른 로봇이 쥔 노드면 거절)는 본다. `_grant(..., None)`을 부르므로 기존 방향도 설정하지 않는다. 그래서 "한 번에 한 방향"은 `take` 경로의 규칙이다. 전체 보유 상태의 불변식이 아니다. 초기 배치가 노드가 있는 차선에 `force`를 쓸 수 있다.

### 5.3 불변식과 구조를 고정하는 시험

| 지키는 것 | 시험 | 방법 |
|---|---|---|
| 위 5.2의 구역·노드·차선 규칙 | `test_zone_grants.py` (13개) | ZoneGrants를 직접 쓴다 |
| 시험 세계 안에서 (배치되어 있고 고장·통신 이상이 없는 로봇은) 쥔 구역 안에 있다, 한 줄 구역의 방향이 섞이지 않는다, 도크에 둘이 있지 않다 | `zone_harness.py`의 `_check()` (`step()`이 매번 부른다) | 위반하면 `world.problems`에 적는다. 이것을 확인하는 시험은 `test_zone_*.py` 20개 파일에 있다. 하네스가 스스로 실패시키지는 않는다 |
| 같은 이름의 메서드를 두 믹스인이 정의하지 않는다. 예외 0개 | `test_zone_mixins.py` (4개) | MRO를 돌며 이름이 겹치는 것을 센다. 기대값은 빈 사전. 주문 코어와 FleetManager도 같은 검사 |
| goal을 바꾸는 곳은 GoalLife(와 `ZoneBase.__init__`)뿐 | `test_zone_goal_life.py` (3개) | ast로 `.goal`에 대한 대입, AugAssign, AnnAssign, for·with·컴프리헨션 대상, 별표 풀기, `setattr`를 찾는다 |
| 각 Episode 묶음(`Robot.watch.<이름>`)을 쓰는 모듈이 정한 목록 안에 있다. `m.watch` 자체를 돌려 쓰지 않는다 | `test_zone_episodes.py` (3개) | ast로 `.watch.<이름>`을 쓰는 파일을 모아 표(3.2)와 비교한다. `Watches`의 필드가 표와 같은지도 본다 |
| 교통 층이 주문·바구니 쪽을 부르지 않는다(import, 이름, 필드) | `test_fms_boundary.py` (20개) | ast로 import를 따라간다 |
| FleetManager와 ROS 노드가 교통 층에 부르는 이름이 모두 `TrafficApi`에 있다 | `test_traffic_api.py` (6개) | ast로 `self.traffic.<이름>`, `core.traffic.<이름>`을 모은다 |
| 놓는 거리와 때 | `test_zone_release.py` (17개) | 몸 전체가 나갈 때까지, 도킹 오차, 마진 |
| 고장·격리된 로봇은 구역을 쥔 채로 호출을 받지 않는다 | `test_zone_isolation.py` (18개) | 하네스 |
| 정원 경고 | `test_zone_capacity.py` (2개) | |
| 교착과 대기 순환 | `test_zone_watch.py` (15개), `test_deadlock.py` (4개) | |

`test_zone_*.py`는 37개 파일, `def test_` 422개다(parametrize는 한 번으로 센다).

시험이 비어 있는 곳 [추측]: (a) 하네스의 `_check()`는 SHARED 구역 정원 초과와 노드 중복을 직접 보지 않는다. 그것은 ZoneGrants가 보장하고 `test_zone_grants.py`가 지킨다. (b) `force`가 차선 방향을 무시하는 것을 막는 시험은 보지 못했다. (c) 작은 객체가 나눠 쓰는 사전(`_robots`, `_wants`, `_blocked`, `_marks`, `_joining`)을 누가 다시 대입하면 객체만 옛 사전을 본다. 지금은 아무도 다시 대입하지 않지만(시험도 제자리에서만 바꾼다) 이것을 막는 시험은 없다.

## 6. 분리 결과와 남긴 것

전제는 그대로다: 전면 재구성은 하지 않았다. 한 번에 하나씩 떼고, 떼기 전에 비교 도구를 먼저 세웠다(`tools/zone_trace.py`, `tools/zone_compare.sh`, 시험별 하네스 기록 `ZONE_TICK_TRACE`). 단계마다의 결과는 `docs/worklog_zone_cleanup_2026-11-18.md`에 있다.

### 6.1 Views → `ZoneViews` (했다)

`robot_view`, `status_view`, `zones_view`, `everyone_placed`, `live_block`, `_route_left`를 ZoneViews로 옮겼다. ZoneTraffic에는 같은 이름의 한 줄 위임이 남았다. `robots`, `running`, `zones`, `available`, `take_output`은 tick과 다른 믹스인이 `self.`로 부르므로 ZoneBase로 옮겼다. ZoneViews는 ZoneBase가 만든다(ZoneTraffic에 `__init__`을 두면 `test_zone_mixins.py`가 ZoneBase의 `__init__`과 겹친다고 본다).

### 6.2 작은 감시: Capacity, Localization, NoStart, Silent, Detours (했다)

각각 CapacityCheck, PoseJumpWatch, StartWatch, SilentWatch, DetourNotes가 되었다. 이벤트는 `emit`(= `ZoneBase._event`)으로 낸다. Silent의 `_event` 가로채기는 2단계에서 없앴고(ZoneBase `_event`가 바로 부름), 5단계에서 그 초기화가 `SilentWatch.note`가 되었다. Detours를 부르는 Blocked와 Goals의 자리는 그대로라 DETOUR 이벤트의 순서가 같다. 실행 모드는 StartWatch와 SilentWatch에 인자 `running`으로 건넨다.

### 6.3 막힘 표시 책: BlockMarks + ClearChecks → `MarkBook` (했다)

MarkBook이 표시 만들기·갱신(`mark`), 키 보유(`hold`), 만료(`expire`), 확인 요청(`ask`), 응답 처리(`answer`)를 갖는다. `_mark_memory`와 번호 생성기는 MarkBook 안에만 있다. 이벤트와 `CheckClearCmd`는 MarkBook이 돌려주고 ZoneTraffic 쪽이 같은 자리에서 내보낸다. 로봇과 지도를 보는 도우미(`_obstacle_point`, `_robot_at`, `_point_keys`, `_yielded_lately`, 이벤트를 내는 `_mark_point`)는 Blocked로 옮겼다. 콜백은 clock 하나다.

### 6.4 대기 순환 감지 → `DeadlockWatch` (했다)

`_cycle*` 세 상태와 대기 그래프·순환 찾기·`DEADLOCK_SEEN`을 옮겼다. 비킬 로봇을 고르고 `_evade`를 부르는 결정은 Watch의 `_watch_deadlock`에 남았다. `_near`, `_watch_near`, `_to_parking`은 Watch에 그대로 있다(Backoff 호출 때문).

### 6.5 하지 말 것 (그대로)

- Goals, MoveOrders, Turns, Drive, Results, Blocked, Backoff, Watch의 `_to_parking` 쪽은 서로 맞물려 있다(4절). 한꺼번에 떼면 보유 구역이 흔들린다.
- `grants`의 쓰기 호출 지점 10곳은 합치지 않는다. 이미 객체로 분리되어 있고 시험이 있다.
- `trip`, `phase`, `node`를 한 곳으로 모으는 일은 하지 않았다. 쓰는 모듈이 5~10개다(3.2).
- Backoff(마당 물러나기)는 2부에서 따로 다룬다. 이 정리에서는 `.goal =`을 GoalLife로 바꾼 것 말고 건드리지 않았다.

## 7. 코드를 읽다 찾은 이상한 점

1. `_watch_deadlock`은 항상 빈 `frozenset()`을 반환한다(watch.py `_watch_deadlock` 끝). 그래서 `tick_progress`의 반환값 `stopped`는 항상 비어 있고, `tick_pick(stopped)`와 `robot not in stopped`(controller.py:57, manager.py:232)는 아무 일도 하지 않는다. [확정] 이유(2026-11-18 확인): 이 모양은 자유바닥 교통 층에서 그대로 옮겨 왔다(1da1e61). 거기서는 대기 순환에 든 로봇 이름을 돌려주어 그 tick에 `tick_pick`이 고르지 않고 `step`이 걷지 않게 했다("robots of a wait cycle stand still (H1)"). 구역 교통(817343e)은 순환을 `_evade`로 한 대를 주차 자리로 보내 풀 뿐 세울 로봇을 고르지 않아 늘 빈 집합을 돌려준다. `tick_pick(stopped)`는 `TrafficApi`의 공개 API라 고치지 않고, 코드에 같은 주석을 달았다.
2. (고침) `ZoneBase._outbound`는 읽는 곳이 없었다. 2단계에서 지웠다.
3. (고침) `station_left`(base.py)는 FleetManager가 `hasattr`로 보고 값을 넣어 주기만 하고 traffic 패키지 안에서 읽는 곳이 없었다. `refactor/remove-station-left`에서 지웠다.
4. (고침) `_capacity_checked`는 acfa45b에서 이미 `__init__`에 있었다. 지금은 CapacityCheck의 `checked`다.
5. (고침) Silent의 `_event` 가로채기. 지금은 `ZoneBase._event`가 `SilentWatch.note`를 직접 부른다. 동작(로봇을 지정한 이벤트가 SILENT_STALL이 아니면 그 로봇의 무진척 일화를 지운다)은 같다.
6. `ZoneGrants.force`는 차선 방향과 간격을 보지 않는다(정원과 노드 점유는 본다, 5.2).
7. 하네스의 불변식 검사 결과(`world.problems`)는 시험이 확인해야 효력이 있다(5.3).

## 8. 이 문서에서 센 것과 센 방법

| 숫자 | 방법 |
|---|---|
| 파일 32개, 3674줄 | zone 폴더의 `*.py` 파일 수와 줄 합계(`git show <커밋>:<파일>`의 줄 수) |
| 믹스인 16개, 클래스 몸통 2032줄, 메서드 157개 | ast로 `controller.py`에서 기반 클래스를 읽고, 각 클래스 `end_lineno - lineno + 1`과 `FunctionDef` 수 합(같은 이름의 자료형 `robots.Health`와 믹스인 `health.Health`를 구별한다) |
| 속성 31개 | ast로 `ZoneBase.__init__`의 `self.<x> = ...` 대입 |
| 쓰기/읽기 표(3.1) | 클래스 몸통마다 `self.<속성>`: 대입 대상(튜플 풀기, 인덱스 포함), AugAssign, Delete, 변경 메서드 호출(위 목록), `grants` 변경 메서드. 나머지 Load는 읽기. `__init__`의 첫 대입은 뺀다 |
| 필드 묶음 표(3.2) | 파일마다 `X.<report/health/avoid>.<필드> =`와 `X.<필드> =`(`self`가 아닌 이름) 대상, Episode는 `.watch.<이름>`이 나오는 파일. 변수 이름이 아니라 필드 이름으로 맞춘 근사다(`trip` 줄은 `m.trip.done =` 같은 안쪽 쓰기도 센다) |
| 의존 쌍 69개(46+15+8) | 클래스 몸통마다 `self.<이름>` 중 이름이 다른 클래스에 정의된 메서드·프로퍼티인 것의 (부르는 클래스, 제공 클래스) 고유 조합 |
| 호출 지점 수(4절 표) | 같은 `self.<이름>` 노드 수(ZoneTraffic 제외) |
| `Robot` 맨 위 필드 32개(전 81개) | ast로 `Robot`의 `AnnAssign` 이름 |
| `grants` 쓰기 10곳 | 쓰기 메서드 6개 이름으로 zone 폴더를 grep(grants.py 자신은 뺀다) |
| 시험 수 | `grep -c "def test_"` (parametrize 한 번), pytest 결과 줄 |

## 9. 비교 요약

정리 1부의 여섯 단계(0 비교 도구, 1 GoalLife, 2 가로채기와 죽은 속성, 3 감시 일화(Episode), 4 `Robot` 필드 묶음, 5 약하게 묶인 믹스인 떼기) 뒤에 믹스인은 23개에서 16개가 되었다. 남은 얽힘은 4절의 맞부르는 쌍 넷과 `_to_parking`, `trip`·`phase`·`node`의 여러 쓰기다.
