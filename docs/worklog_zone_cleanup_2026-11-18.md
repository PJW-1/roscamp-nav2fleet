# ZoneTraffic 정리 작업 기록 (2026-11-18)

브랜치 `refactor/zone-cleanup`, 출발점 origin/main acfa45b. 지시서 "밤샘 정리" 1부의 0~6단계다. 동작은 바꾸지 않는다: 명령, 이벤트, 순서, 보유 구역이 같아야 한다. 구조 설명은 `docs/ZoneTraffic_구조.md`, 밤 전체의 기록은 `docs/worklog_overnight_2026-11-18.md`에 있다.

## 1. 정리 전후

| 항목 | 정리 전 (acfa45b) | 정리 후 | 센 방법 |
|---|---|---|---|
| ZoneTraffic 믹스인 | 23 (+ ZoneBase = 기반 24) | 16 (+ ZoneBase = 기반 17) | ast, `controller.py`의 기반 클래스 |
| 믹스인 클래스 몸통 / 메서드 | 2292줄 / 178개 | 2032줄 / 157개 | ast `end_lineno - lineno + 1`, `FunctionDef` 수 |
| 믹스인이 아닌 작은 객체 | 0 | 8 | ZoneViews, CapacityCheck, PoseJumpWatch, StartWatch, SilentWatch, DetourNotes, MarkBook, DeadlockWatch |
| `Robot` 맨 위 필드 | 81 | 32 | ast, `Robot`의 `AnnAssign` |
| `.goal =`이 있는 파일 | 7 (backoff, base, drive, move_orders, order_answers, results, watch) | 1 (goal_life.py) | `\.goal\s*=[^=]` 정규식, 그리고 `test_zone_goal_life.py` |
| 가로채기 | 1 (`Silent._event`) | 0 | `test_zone_mixins.py`의 이름 중복 예외 수 |
| zone 폴더 | 30개 파일, 3512줄 | 32개 파일, 3674줄 | `git show <커밋>:<파일>`의 줄 수 |
| fleet_core 시험 | 1118 passed, 5 xfailed | 1124 passed, 5 xfailed | pytest |
| fake_robots 시험 | 883 passed, 7 skipped, 27 xfailed | 같음 | pytest (ROS 없이) |
| tools 시험 | 84 passed, 14 failed | 같음 | pytest. 14개는 Windows에서 원래 실패하는 `test_real_pode` |

파일별 줄 수:

| 파일 | 전 | 후 | | 파일 | 전 | 후 |
|---|---|---|---|---|---|---|
| __init__.py | 2 | 2 | | move_orders.py | 212 | 211 |
| backoff.py | 111 | 111 | | nostart.py | 34 | 39 |
| base.py | 145 | 169 | | order_answers.py | 127 | 128 |
| block_marks.py | 142 | 163 | | placement.py | 70 | 70 |
| blocked.py | 217 | 270 | | progress.py | 168 | 168 |
| calls.py | 180 | 180 | | redock.py | 55 | 56 |
| capacity.py | 19 | 24 | | results.py | 224 | 224 |
| clear_checks.py | 61 | (없음) | | robots.py | 151 | 155 |
| controller.py | 67 | 83 | | silent.py | 53 | 58 |
| deadlock_watch.py | (없음) | 36 | | trips.py | 191 | 191 |
| detours.py | 44 | 49 | | turns.py | 142 | 142 |
| drive.py | 152 | 150 | | views.py | 94 | 82 |
| episodes.py | (없음) | 38 | | watch.py | 160 | 147 |
| goal_life.py | (없음) | 31 | | yard_retreat.py | 115 | 115 |
| goals.py | 225 | 224 | | grants.py | 134 | 134 |
| health.py | 128 | 131 | | lane_policy.py | 57 | 57 |
| localization.py | 32 | 36 | | 합계 | 3512 | 3674 |

줄은 조금 늘었다. 작은 객체마다 `__init__`과 위임이 붙었기 때문이다. 줄이 아니라 "누가 무엇을 바꾸는가"의 자리 수를 줄이는 정리였다.

## 2. 비교 방법

- 기준: `~/zone_baseline/acfa45b/`(저장소 밖). trace.txt(`tools/zone_trace.py`, 시나리오 3판 × tick 순서 2 × order 모델 켬·끔), situations.md, store.md(d2만, v7은 원래 `KeyError: 'X_W'`로 죽는다), order_v7.md, order_d2.md, 시험별 하네스 기록 suite_trace.json(`ZONE_TICK_TRACE`, 1123개 시험). 기준은 고치지 않았다.
- 빠른 검사(커밋마다, 약 1.5분): trace, store, order_v7, order_d2 diff, fleet_core 시험, ruff.
- 전체 비교(단계의 마지막 커밋, 약 13~14분, 한 번에 하나): `tools/zone_compare.sh`를 커밋의 `git archive` 사본에서 돌린다. 다섯 표 diff, store v7이 같은 오류로 죽는지, suite_trace(시험별 로그·이벤트·명령), fleet_core, fake_robots, tools 시험.

## 3. 단계별 결과

모든 diff는 0줄이다. suite_trace는 1123개 시험 중 바뀐 시험 0개다.

| 단계 | 한 일 | 마지막 커밋 | trace | situations | store | order_v7 | order_d2 | suite_trace 바뀜 | fleet_core |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 비교 도구 `tools/zone_trace.py` | eff7053 | 0 | 0 | 0 | 0 | 0 | 0 | 1118 passed |
| 1 | GoalLife: goal은 한 모듈에서만 바꾼다 | 5768696 | 0 | 0 | 0 | 0 | 0 | 0 | 1120 passed |
| 2 | Silent 가로채기 없앰, 죽은 `_outbound` 지움, `_watch_deadlock` 주석 | 07338a4 | 0 | 0 | 0 | 0 | 0 | 0 | 1120 passed |
| 3 | 감시 일화를 Episode로 (NoStart, Silent, Watch, 대기, 로봇단 양보, 다시 도킹, 막힘) | 82f805d | 0 | 0 | 0 | 0 | 0 | 0 | 1121 passed |
| 4 | `Robot` 필드 묶음 report, health, avoid, watch | 532a247 | 0 | 0 | 0 | 0 | 0 | 0 | 1121 passed |
| 5 | 약하게 묶인 믹스인 떼기 (아래) | 23f2211 | 0 | 0 | 0 | 0 | 0 | 0 | 1121 passed |
| 6 | 문서와 구조 시험 | aff479c | 0 | 0 | 0 | 0 | 0 | 0 | 1124 passed |

모든 전체 비교에서 store `-l v7 d2`는 기준과 같은 `KeyError: 'X_W'`로 죽었고, fake_robots는 883 passed, 7 skipped, 27 xfailed, tools는 84 passed, 14 failed였다.

### 3.1 5단계: 떼어 낸 것

| 커밋 | 믹스인 | 객체 | 콜백 | tick 안 순서 | 빠른 검사 |
|---|---|---|---|---|---|
| 2b01529 | Views | ZoneViews (조회 5개, ZoneTraffic에 한 줄 위임) | 0 | 그대로 | 0줄, 1121 passed |
| 106b944 | Capacity, Localization, NoStart | CapacityCheck, PoseJumpWatch, StartWatch | emit 1 | 그대로 | 0줄, 1121 passed |
| 718c5f5 | Silent, Detours | SilentWatch, DetourNotes | emit, emit_once 2 | 그대로 | 0줄, 1121 passed |
| 3bf7be3 | BlockMarks, ClearChecks | MarkBook | clock 1 (이벤트는 돌려주고 ZoneTraffic이 낸다) | 그대로 | 0줄, 1121 passed |
| 23f2211 | (Watch에 남음) | DeadlockWatch (감지만, 비킬 로봇 결정은 Watch) | emit 1 | 그대로 | 0줄, 1121 passed |

718c5f5에서 중간 전체 비교도 했다: 다섯 표 0줄, suite_trace 바뀜 0, fleet_core 1121 passed.

멈춤 조건(콜백 4개 이상, tick 순서 변경)에 걸린 묶음은 없다. 다만 MarkBook은 처음 그린 모양(이벤트와 출력 큐를 MarkBook이 직접 냄)이면 `_reach`, `_gaps_note`, `_event`, `_out` 넣기, clock으로 콜백이 다섯이 되어 멈춤 조건에 걸린다. 그래서 MarkBook은 결과를 돌려주고 내보내기는 ZoneTraffic 쪽이 같은 자리에서 하게 했다. 로봇과 지도를 보는 도우미는 Blocked로 옮겼다. 이 모양이면 콜백은 clock 하나다.

Backoff 믹스인과 `_stand_aside`, `_awaits_blocked`는 건드리지 않았다(2부 A2가 같은 곳을 다른 브랜치에서 고친다).

### 3.2 바꾼 시험 (기대값은 그대로, 내부 경로만)

- `test_zone_silent.py`: 출발 감시를 끄는 대치가 `traffic._watch_start` → `traffic._start_watch.watch`.
- `test_zone_failures.py`, `test_zone_watch.py`: `traffic._cycle` → `traffic._deadlock.cycle`.
- `test_zone_mixins.py`: 모듈 설명의 `_note_robot_event`를 `SilentWatch.note`로(설명문만).
- 지운 시험은 없다.

### 3.3 6단계: 문서와 구조 시험

- `docs/ZoneTraffic_구조.md`를 새 상태로 다시 썼다: 맨 앞 "정리 전후" 표와 파일별 줄 수, 작은 객체 표(1.1), tick 순서 그림(2절), 속성 쓰기·읽기 표(3.1), 필드 묶음 → 쓰는 모듈 표(3.2), 의존 지도(4절), 분리 결과(6절), 이상한 점의 고침 여부(7절).
- `docs/관제_구역교통.md`: 막힘 표시 파일 목록에서 없어진 `clear_checks.py`를 MarkBook으로 바꿨다. 규칙과 값은 그대로.
- 새 구조 시험 `test_zone_episodes.py`(3개): 각 Episode 묶음을 쓰는 모듈이 정한 목록과 같다, 목록이 `Watches`의 필드와 같다, `m.watch`를 통째로 돌려 쓰는 곳이 없다. 일부러 어긴 사본(`w = m.watch`, 새 모듈의 `m.watch.stall`)으로 두 검사가 실패하는 것을 확인했다.
- 이미 있던 구조 시험: `.goal =` 위치(`test_zone_goal_life.py`, 3개), 믹스인 이름 중복 예외 0개(`test_zone_mixins.py`의 기대값 `{}`).

## 4. 남긴 것과 물을 것

- `trip`, `phase`, `node`는 여전히 5~10개 모듈이 쓴다. goal처럼 한 곳으로 모으지 않았다.
- 맞부르는 믹스인 쌍 넷(Goals-MoveOrders, Goals-Turns, Backoff-Results, Backoff-Watch)과 `_to_parking`은 그대로다.
- 작은 객체는 ZoneTraffic의 사전을 참조로 나눠 쓴다. 누가 그 사전을 다시 대입하면 객체만 옛 사전을 보게 된다. 지금은 아무도 다시 대입하지 않지만 이를 막는 시험은 없다.
