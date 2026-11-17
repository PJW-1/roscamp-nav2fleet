# 밤샘 작업 기록 (2026-11-18)

지시서 `지시서_밤샘_정리.md`(사용자 PC의 `D:\downloads\`, 저장소에는 없다)를 따라 한 일을 단계가 끝날 때마다
한 줄씩 적는다. 다른 세션은 이 기록과 지시서만 보고 이어서 할 수 있어야 한다.

## 0. 지시서와 다르게 한 것 (사용자의 채팅 지시)

지시서는 "main에 push·병합하지 않는다", "Gazebo와 `fleet_manager` 시험을 돌리지 않는다"고 적는다.
사용자는 자러 가기 전에 채팅으로 이렇게 덧붙였다(지시서보다 나중에 온 지시라 이것을 따랐다).

- "테스트 후에 시뮬까지 해 보고 이상 없으면 main에 push까지 하고, 애매하다 싶으면 브랜치에 따로 빼 놔."
- "다른 세션에서 레포만 보고 알 수 있도록 기록해 놓고."
- "끝나면 컴퓨터 종료까지 하면 돼."

그래서 이렇게 했다.

- Gazebo 세 대 판은 이 PC에서 돌렸다(도메인 61, 실물 로봇 없음, 도메인 12는 쓰지 않았다).
- `fleet_manager` 시험(도메인 50~59)은 main에 넣기 직전의 마지막 확인으로만 돌린다. 돌렸으면 아래 표에 적는다.
- main 병합은 시험과 Gazebo 판에서 이상이 없을 때만 한다. 애매하면 브랜치에만 둔다. 결정과 근거는 8절에 적는다.
- 나머지 금지(실물 로봇 폴더, `deploy.sh`, 로봇 접속, 동작을 바꾸는 변경, 2부 4절 관문과 B단계)는 그대로 지켰다.

## 1. 진행 (단계가 끝날 때마다 한 줄)

| 단계 | 상태 | 브랜치와 마지막 커밋 | 시험 수(fleet_core / fake_robots / tools) | 기준과의 diff |
|---|---|---|---|---|
| 1부 0 기준과 trace 도구 | 끝 | `refactor/zone-cleanup` eff7053 | 1118 passed 5 xfailed / 883 passed 7 skipped 27 xfailed / 84 passed 14 failed | 0줄 |
| 1부 1 goal의 수명 | 끝 | 5768696 | 1120 / 883 / 84 | 0줄 |
| 1부 2 숨은 결합과 죽은 코드 | 끝 | 07338a4 | 1120 / 883 / 84 | 0줄 |
| 1부 3 감시 기록을 Episode로 | 끝 | 82f805d | 1121 / 883 / 84 | 0줄 |
| 1부 4 `Robot` 필드 묶기 | 끝 | 532a247 | 1121 / 883 / 84 | 0줄 |
| 1부 5 약하게 묶인 믹스인 떼기 | 끝 | 23f2211 | 1121 / 883 / 84 | 0줄 |
| 1부 6 문서와 구조 시험 (1부 끝) | 끝 | aff479c | 1124 / 883 / 84 | 0줄 |
| 2부 A1 양보 보고 도구 | 끝 | `tools/yield-report` bf7848c | tools 시험 5개 추가 | 제품 코드 변경 없음 |
| 2부 A3 가짜 로봇의 양보 사다리 | 끝(끔이 기본) | `refactor/yield-a3-fake-ladder` 25664cf | 1118 / 895 / 84, pode_fsm 366 passed 8 skipped | 끔일 때 0줄 |
| 2부 A2 마당 물러나기 지우기 | 끝 | `refactor/yield-owner` 441d0d5 | 1087 passed 5 xfailed / 892 passed 7 skipped 27 xfailed / 89 passed 14 failed | 0줄(trace, situations, store, order_v7, order_d2) |
| 2부 A4 로봇 사다리 하나로(선택) | 하지 않음(이유는 6절) | 변경 없음 | - | - |
| 1부와 2부 합치기 | 끝 | `refactor/yield-owner` ccb7336(병합 커밋) | 1090 passed 5 xfailed / 892 passed 7 skipped 27 xfailed / 89 passed 14 failed | 0줄(다섯 표 모두) |

- 시험 수는 Windows 파이썬으로 돌린 것이다. fake_robots의 7 skipped는 rclpy가 없어 건너뛴 `test_ros_*.py` 일곱 파일이고,
  tools의 14 failed는 `test_real_pode.py`가 Windows에서 bash를 못 찾는 기존 실패다(WSL에서는 통과한다).
- fleet_core가 1118에서 1124로 는 것은 새 구조 시험 `test_zone_goal_life.py`(3개)와 `test_zone_episodes.py`(3개) 때문이다. 지운 시험은 없다.
- 3·4단계는 커밋마다 빠른 검사(trace, store, order 표 diff, fleet_core 시험, ruff)를 하고, 전체 비교(`tools/zone_compare.sh`)는
  19447ed(3단계 첫 커밋), 82f805d(3단계 끝), 532a247(4단계 끝)에서 돌렸다. 모두 0줄이다.
- 5단계도 커밋마다 빠른 검사를 하고, 전체 비교는 718c5f5(중간)와 23f2211(5단계 끝)에서 돌렸다. 모두 0줄, suite_trace 바뀜 0.

## 2. 기준과 비교 방법

- 기준은 `origin/main` acfa45b에 `tools/zone_trace.py`만 더한 상태(eff7053)에서 만들었다. 이 PC의 `~/zone_baseline/acfa45b/`에 있다
  (저장소 밖이다. 없으면 eff7053을 체크아웃해 아래 명령으로 다시 만든다. 모두 `PYTHONUTF8=1`).

| 파일 | 명령 | 걸린 시간 |
|---|---|---|
| `trace.txt` | `python tools/zone_trace.py` | 5 s |
| `situations.md` | `python tools/situations.py -l v7 d2 d c --avoid all --seeds 1 2 3` | 827 s |
| `store.md` | `python tools/situations.py --store -l d2 --seeds 1 2 3` | 46 s |
| `order_v7.md` | `python tools/order_situations.py --seeds 1 2 3 -l v7` | 66 s |
| `order_d2.md` | `python tools/order_situations.py --seeds 1 2 3 -l d2` | 70 s |
| `suite_trace.json` | `cd pc/fleet_core && ZONE_TICK_TRACE=<파일> python -m pytest -q test` | 95 s |

- 한 번에 비교하기: `bash tools/zone_compare.sh [작업 폴더] [기준 폴더]`. 표마다 diff 줄 수와 세 시험 묶음의 요약을 찍는다(11~14분).
- 지시서의 `--store -l v7 d2`는 v7에 X_W 노드가 없어 `KeyError: 'X_W'`로 죽는다(기존 결함, `store_situations.py:58`).
  그래서 store 기준은 d2로만 만들었고, v7에서 같은 오류로 끝나는지만 매번 확인한다.
- Windows 파이썬의 `PYTHONPATH`는 `cygpath -m`으로 바꾼 경로를 `;`로 이어야 한다. `/c/...`를 그대로 주면 fake_robots 시험 28개
  파일이 수집되지 않는다.
- 실물 기록(`~/real_runs/`)은 이 PC에 없어 `tools/replay.py` 비교는 하지 못했다.

## 3. 1부에서 한 일

- **0단계:** `tools/zone_trace.py`. `SCENARIOS` 세 판을 tick 순서 두 가지와 order 모델 켬·끔으로 12번 돌려, tick마다 나온 명령·
  이벤트·결과와 로봇별 보유 구역·노드·`robot_view`·`status_view`를 적는다.
- **1단계:** 새 믹스인 `GoalLife`(`zone/goal_life.py`): `_set_goal(m, goal, *, reset_watch=True)`, `_clear_goal`, `_reset_stall`,
  `_cancel_goal`. `m.goal =` 대입은 이 파일에만 있다(ast 구조 시험 `test_zone_goal_life.py`).
  - 지시서의 13곳 목록은 실제로 12곳이었고 `drive.py`의 `_send`가 빠져 있었다. 그것까지 13곳을 바꿨다.
  - 멈춤 감시(`watch.py`)는 goal을 되살리지 않고 `_arrived_anyway(m, reason, end, goal)`에 goal을 넘긴다.
- **2단계:** `Silent`의 `_event` 가로채기를 없애고 `ZoneBase._event`가 `_note_robot_event`를 부른다(`OVERRIDES` 예외 삭제).
  읽는 곳이 없는 `ZoneBase._outbound`를 지웠다. `_watch_deadlock`이 늘 빈 집합인 이유를 주석과 구조 문서에 적었다.
  `_capacity_checked`는 main에서 이미 `ZoneBase.__init__`이 만들고 있어 고칠 것이 없었다.
- 바꾼 시험: `test_zone_mixins.py`(지시서가 지우라고 한 `OVERRIDES` 예외, 기대값 `{}`). 지운 시험은 없다.
- **3단계:** `zone/episodes.py`의 `Episode`(key, since, xy, noted, count, `clear()`)와 `WaitEpisode`(key 기본 `()`, queued),
  `BlockedEpisode`(goal id, clear_since, out, rear_noted, self_recovery_since). 감시 하나에 커밋 하나로 NoStart, Silent(silent·idle),
  Watch(stall·outside), 대기, 로봇단 양보, 다시 도킹(세 상태를 Episode 하나로), 막힘 순으로 옮겼다. 한 번만 알리는 경고는
  `ZoneBase._event_once`. `asked_at`, `asked_gap`은 호출이 정하고 `_stop_wanting`이 지우지 않아 대기 일화에 넣지 않았다.
  Codex 지적으로 `.goal =` 구조 시험이 setattr, for·내포 대상, `*m.goal`, with ... as도 잡게 했다(82f805d).
- **4단계:** `Robot` 맨 위 필드 81개(acfa45b) → 61개(3단계 뒤) → 32개. 묶음은 `report`(로봇 보고 6개), `health`(고장·통신·자세 의심
  8개), `avoid`(양보·물러나기·우회 10개), `watch`(3단계의 일화 9개). 이름만 옮기는 기계적 수정이고, `reinstate`는 전과 같은
  보고 값 넷만 새 `Report`에 옮긴다. `asked_at`, `asked_gap`, `clear_n`, `dock_error_m`, `sent_priority`는 맨 위에 두었다.
- 3·4단계에서 경로만 바꾼 시험(기대값 그대로): `test_zone_corridor`, `test_zone_failures`, `test_zone_block_marks`,
  `test_zone_local_yield`, `test_zone_isolation`, `test_zone_at_target`, `test_zone_move_orders`, `test_zone_plumbing`,
  `test_zone_silent`, `test_zone_yard_retreat`.
- **5단계:** 믹스인 23개(+GoalLife 24개) → 16개(+ ZoneBase = 기반 17개). 커밋 하나에 한 묶음씩, 모두 `ZoneBase.__init__`이 만드는 작은 객체로.
  - 2b01529 Views → `ZoneViews`. ZoneTraffic에 한 줄 위임 5개. `robots`, `running`, `zones`, `available`, `take_output`은 ZoneBase로.
    객체는 ZoneBase가 만든다(ZoneTraffic에 `__init__`을 두면 이름 중복 시험이 ZoneBase와 겹친다고 본다).
  - 106b944 Capacity, Localization, NoStart → `CapacityCheck`, `PoseJumpWatch`, `StartWatch`. 콜백은 `emit`(= `_event`) 하나.
  - 718c5f5 Silent, Detours → `SilentWatch`(emit, emit_once), `DetourNotes`(emit). `_event`가 `SilentWatch.note`를 부른다.
  - 3bf7be3 BlockMarks + ClearChecks → `MarkBook`. `_mark_memory`와 번호는 MarkBook 안에만 있다. 이벤트와 `CheckClearCmd`는
    돌려주고 ZoneTraffic 쪽이 같은 자리에서 낸다(직접 내게 하면 콜백이 다섯이라 멈춤 조건에 걸린다). 콜백은 clock 하나.
    로봇·지도를 보는 도우미는 Blocked로 옮겼다. `clear_checks.py`는 없어졌다.
  - 23f2211 대기 순환 감지 → `DeadlockWatch`(emit). 비킬 로봇 결정(`choose_retreat`, `_evade`)과 `_near`, `_watch_near`,
    `_to_parking`은 Watch에 남았다.
  - 멈춤 조건에 걸려 남긴 묶음은 없다. tick 안 호출 순서는 하나도 바꾸지 않았다. Backoff, `_stand_aside`, `_awaits_blocked`는
    건드리지 않았다(2부 A2).
  - 경로만 바꾼 시험: `test_zone_silent`(`_start_watch.watch` 대치), `test_zone_failures`·`test_zone_watch`(`_deadlock.cycle`),
    `test_zone_mixins`(설명문만).
- **6단계:** `docs/ZoneTraffic_구조.md`를 새 상태로 다시 썼다(맨 앞 "정리 전후" 표와 파일별 줄 수, 작은 객체 표, tick 순서 그림,
  속성 표, 필드 묶음 → 쓰는 모듈 표, 의존 지도). `docs/관제_구역교통.md`는 없어진 `clear_checks.py` 언급만 고쳤다.
  새 구조 시험 `test_zone_episodes.py`: Episode 묶음마다 쓰는 모듈 목록, `m.watch`를 통째로 돌려 쓰지 않기. 정리 전후 표와
  단계별 diff 결과는 `docs/worklog_zone_cleanup_2026-11-18.md`에 있다. 전체 비교 aff479c: 0줄. 1부는 여기서 끝이다.

## 4. 2부에서 한 일

- **A1:** `tools/yield_report.py`와 시험, 기록 `docs/worklog_yield_report_2026-11-18.md`(브랜치 `tools/yield-report`).
- **A3:** 머리 없는 세계의 가짜 로봇이 `pode_fsm.local_yield`의 순수 함수를 가져다 로봇단 양보 사다리를 흉내 낸다.
  `ZoneWorld(local_yield=True)`, `tools/situations.py --local-yield`로 켠다. 기본은 끔이고 끔일 때 기준과 0줄이다.
  - `sim/fake_robots/package.xml`에 `exec_depend pode_fsm`을 더했고, 순환을 피하려고 `pode/pode_fsm/package.xml`의
    `test_depend fake_robots`를 뺐다(동작은 그대로다. 질문 7절).
  - 실제 사다리와 다른 점: BLOCKED 뒤의 유지 동작(1 s 비움, 3 s 또는 0.15 m)이 없다. 막힘 판정 거리와 진행 문턱은 가짜
    로봇의 값을 쓴다. ROS 가짜 노드(`fake_pode_node.py`)에는 넣지 않았다.
- **A2:** 마당 물러나기(`yard_backoff`, 기본 끔)를 지웠다(6af41a0 시험, 7bb38cc 코드, 8ad2bb1 파라미터, 441d0d5 문서).
  - 지운 것: `backoff.py`의 `backoff_spot`, `Backoff._back_off`, `_awaits_blocked`, `_backoff_result`, `_backoff_ended`,
    `_stand_aside`의 물러나기 갈래, goal 종류 `BACKOFF`와 `Avoid.backoff_from`, 그것을 보던 `blocked.py`, `calls.py`,
    `results.py`(네 곳), `watch.py`의 갈래, 이벤트 `HALL_BACKOFF_DONE`, `HALL_BACKOFF_FAILED`, 파라미터 넷(`params.py`,
    `fleet_params.yaml`). `_stand_aside`의 섬과 "같은 자리에서 다시 비키면 주차로" 갈래는 남겼다.
  - 시험 수: fleet_core 1121 → 1087(-34), fake_robots 895 → 892(-3), tools 89 passed 14 failed 그대로.
  - 지운 시험: `test_zone_backoff.py`의 24개(물러나기만 본다. 기본 경로를 보는
    `test_with_the_switch_off_a_blocked_robot_only_stops`는 기대값 그대로 남겼다), `test_situation_avoid_modes.py`의
    `test_the_yard_back_off_does_not_stand_a_robot_on_a_node_it_does_not_hold` 3개(물러나기를 켜야만 도는 시험).
    `test_params_validate.py`의 10개는 지운 파라미터마다 자동으로 생기던 경우라 함께 줄었다(시험 코드는 그대로).
  - 고치지 않은 곳: `docs/Pode_계약.md` 10번의 "넓은 구역 안의 물러나기"(계약 문서라 건너뜀),
    `docs/ZoneTraffic_구조.md`(1부 5단계가 같은 문서를 고치는 중이라 병합 충돌을 피함), 지난 기록과 결과 문서,
    `tools/yield_report.py`의 `backs ... then stops` 문구(지난 기록을 읽는다). `fleet_interfaces`에는 해당 상수가 없었다.

## 5. 지시서 밖의 일 (같은 밤)

- **"도리도리" 개선(goal 경로 유지):** 브랜치 `feat/goal-plan-keeper`, 기록 `docs/worklog_goal_plan_keeper_2026-11-18.md`
  (그 브랜치에만 있다). 뒤집힘은 줄었지만 실물에서 돌려 보지 않았고 PC 부하가 높던 판에서 복구가 많아 main에 넣지 않았다.
  버린 두 방법은 브랜치 `fix/goal-replan-on-invalid`에 남겼다.
- **실물 SLAM 지도 브랜치(`feat/real-arena-slam-map`)의 Gazebo 판:** 같은 기록 문서 7절에 있다. 네 판 모두 12/12, 위반 0.
- **main의 기준 판:** 같은 기록 문서 4절(mv1~mv4, mr1~mr3).

## 6. 건너뛴 것과 이유

- `station_left` 삭제: 지시서가 묻고 하라고 한 항목이라 지우지 않았다(7절 1번).
- 실물 기록 비교: 이 PC에 `~/real_runs/`가 없다.
- **2부 A4(로봇 사다리 하나로):** 하지 않았다. `yield_moves.py`(고정 경로)와 `yield_nav.py`(Nav2 goal)에서 실제로 같은 것은
  파일마다 4줄쯤이고, 공통 부분(`_backs_up`, `local_yield_step`, `_send_backup`, `blocking_peer` 등)은 이미 한 곳에 있다.
  단계마다 조건이 달라 합치려면 넘길 함수가 8개 이상 필요하고, 한쪽에만 있는 갈래(Nav2의 정지 대기 단계, 고정 경로의
  `block_clear_s`와 `NAV_ABORTED`로 끝나는 경우)를 공통 코드에 조건문으로 넣어야 한다. 지시서의 "합치는 게 오히려 복잡해지면
  하지 말고" 조건에 해당한다. 브랜치 `refactor/yield-a4-one-ladder`는 만들기만 했고 커밋이 없다.
- 2부 4절 관문과 B단계: 지시서대로 하지 않았다.
- `docs/Pode_계약.md` 10번의 "넓은 구역 안의 물러나기" 문구: A2로 그 기능이 없어졌지만 계약 문서라 고치지 않았다(7절 7번).

## 7. 질문 (아침에 사용자에게)

1. **`station_left`를 지울까요?** traffic 안에서 읽는 곳이 없다(`base.py`에서 만들기만 한다). 값을 넣는 곳은 `manager.py`의
   `hasattr` 분기이고 `test_traffic_api.py`에 `OPTIONAL`로 적혀 있다. 지우면 `manager.station_left_s`와 그것만 쓰는 `eta_*`
   파라미터 셋도 함께 죽는다.
2. **`reset_watch=False` 두 자리에서 멈춤 감시 기록을 초기화해야 할까요?** `order_answers._restore`(order update가 실패해
   예전 goal로 되돌릴 때)와 `results._arrived_anyway`(끝에 서 있는 로봇의 goal을 끝 goal로 바꾼 직후 지울 때)다. 지금 동작을
   그대로 두었다. 세 번째 자리였던 `watch.py`의 되살리기는 대입 자체가 없어졌다.
3. **v7 store 상황의 `KeyError: 'X_W'`를 고칠까요?** `tools/situations.py --store -l v7`이 죽는다(main에도 있는 결함).
4. **`pode_fsm`의 `test_depend fake_robots`를 빼도 될까요?** A3의 새 의존(`fake_robots` → `pode_fsm`)과 순환이 되어 뺐다.
   WSL에서 빌드가 되고 `pode_fsm` 시험은 467 passed, 6 skipped다.
5. **가짜 로봇의 BLOCKED도 실제 로봇처럼 유지해야 할까요?** B3에서 정할 일이다.
6. A1 도구의 `--grace` 기본값 10 s는 담당 에이전트가 정한 값이다.
7. **`docs/Pode_계약.md` 10번에서 마당 물러나기 문구를 뺄까요?** 계약 문서라 손대지 않았다.
8. **두 모드의 사다리가 끝나는 방식이 다릅니다(A4를 따져 보다 확인).** 고정 경로는 재시도 끝에 앞이 비어 있으면 `BLOCKED`가
   아니라 `NAV_ABORTED`로 끝나고(`recovery.py`), `blocked_after_s`가 지나도 앞이 비어 있으면 포기하지 않는다(`yield_moves.py`).
   Nav2 goal은 조건 없이 포기한다. `docs/회피_약속.md` 5절은 두 모드 모두 "또 막히면 BLOCKED"라고 적는다. 그대로 둘지 정해
   주십시오(동작을 바꾸는 일이라 손대지 않았다).
9. **goal 경로 유지(`feat/goal-plan-keeper`)를 main에 넣을까요?** 실물에서 한 번 돌려 보고 정하는 것을 권한다.
10. 구조 시험 둘(`test_zone_goal_life.py`, `test_zone_episodes.py`)은 `getattr(m, "watch")` 같은 동적 접근을 잡지 못한다
    (Codex 지적, 지금 코드에는 그런 접근이 없다). 보강할지 정해 주십시오.

11. **A2 뒤로는 그 전에 남긴 관제 기록을 `tools/replay.py`로 재생하지 못합니다.** 기록 첫 줄에 `yard_backoff*` 파라미터 넷이
    있어 `these parameters do not fit this code`로 거절한다. 첫 줄에서 그 넷을 빼면 재생된다(이번 밤의 main 판 일곱 개로
    확인). 도구가 없어진 파라미터를 알려 주고 건너뛰게 할지 정해 주십시오(도구의 판정을 바꾸는 일이라 손대지 않았다).
12. **`fleet_manager` 통합 시험이 가끔 실패합니다(8절).** 실패할 때의 출력을 남기도록 시험을 고칠지 정해 주십시오.

## 8. main에 넣을지의 결정

**넣었다.** `refactor/yield-owner`(1부 전체 + 2부 A1·A2·A3)를 main에 병합 커밋 하나로 넣고 push했다. 근거는 다음과 같다.

- **기준과의 차이 0줄:** 병합본(ccb7336)에서 trace, situations, store, order_v7, order_d2 다섯 표가 모두 0줄이고, 시험별 하네스
  기록은 지운 시험 34개 말고는 바뀐 것이 없다.
- **시험(WSL, ROS 2 Jazzy, 병합본):** fleet_core 1090 passed 5 xfailed, fake_robots 1031 passed 27 xfailed, tools 103 passed,
  fleet_gz_sim 586 passed, pode_fsm 467 passed 6 skipped, pode_nav_config 106, pode_avoidance 26, pode_peer_obstacles 19,
  pode_scan_filter 23, fleet_manager 95 passed 30 skipped(전체 실행 네 번 가운데 두 번이고, 나머지 두 번은 바로 아래에 적었다).
  `fleet_manager` 시험은 지시서가 금지한 것이지만 main에 넣기 전의 마지막 확인으로 돌렸다(0절).
- **`fleet_manager` 통합 시험은 가끔 실패한다(main도 같다):** 전체 실행(95개, 11분)을 병합본에서 네 번, main(acfa45b)에서
  세 번 돌렸다.

| 코드 | 실행 | 결과 | 실패한 시나리오와 이유 |
|---|---|---|---|
| 병합본 | 1 | 2 failed, 93 passed | `shared_lane_failure`, `stock_shortage`(출력을 남기지 못해 이유 모름) |
| 병합본 | 2 | 95 passed | - |
| 병합본 | 3 | 95 passed | - |
| 병합본 | 4 | 1 failed, 94 passed | `stuck_robot`: pode1이 출고 도크에 닿기 전인 70 s에 멈춰 출고가 2건이 아니라 5건 |
| main | 1 | 95 passed | - |
| main | 2 | 95 passed | - |
| main | 3 | 1 failed, 94 passed | `blocked_lane`: `pode2 and pode3 are 0.135 m apart`(설명 안 되는 근접 1건) |

  실패한 둘(`shared_lane_failure`, `stock_shortage`)만 따로 돌린 것은 병합본 일곱 번, main 한 번 모두 통과했다. 병합본 4번
  실행이 남긴 시나리오 기록 아홉 개(`fms.jsonl`, 실패한 `stuck_robot` 포함)를 main 코드에 `tools/replay.py`로 넣으면 아홉 개
  모두 `same as recorded`다. 즉 실패한 판에서도 main 코드가 같은 명령을 낸다. 이 시험이 지나는 코드 가운데 병합으로 바뀐 것은
  fleet_core뿐이다(`fleet_manager`와 ROS 쪽 가짜 로봇 노드는 그대로다). 그래서 병합과 무관하게 시간에 따라 갈리는 시험으로
  판단했다(`stuck_robot.yaml`의 머리 주석과 `docs/worklog_drive_handoff_2026-11-17.md`에도 같은 종류의 기록이 있다).
- **Gazebo 세 대 판(이 PC, GUI + RViz, 병합본):** 다섯 판 모두 12/12, 위반 0.

| 판 | 종류 | 시간 s | 복구 | BLOCKED_WAIT | 다시 잡은 경로 | 배율 |
|---|---|---|---|---|---|---|
| zv1 | 장애물 | 574.8 | 3 | 4 | 3 | 0.97 |
| zv2 | 장애물 | 572.5 | 0 | 0 | 0 | 0.99 |
| zv3 | 장애물 | 573.3 | 0 | 0 | 0 | 0.99 |
| zr1 | 러시 | 247.1 | 0 | 0 | 0 | 0.99 |
| zr2 | 러시 | 246.3 | 1 | 0 | 0 | 1.00 |

  같은 밤 main(acfa45b)의 판은 장애물 571.2~572.3 s(복구 3~5번), 러시 245.4~245.6 s(복구 1~2번)였다.
- **기록 재생(같은 입력, 두 코드):** Gazebo 판마다 남는 관제 기록(`fleet_record.jsonl`)을 `tools/replay.py`로 반대쪽 코드에
  넣었다. 병합본의 다섯 판(zv1~zv3, zr1, zr2)을 main 코드(acfa45b)에 넣으면 다섯 판 모두 `same as recorded`이고, main의
  일곱 판(mv1~mv4, mr1~mr3)을 병합본 코드에 넣어도 일곱 판 모두 `same as recorded`다(판마다 입력 1만~2.7만 줄, 명령
  434~522개). 즉 실제 ROS 입력에서도 두 코드가 내는 명령과 이벤트가 같다. main의 기록은 첫 줄에 지워진 파라미터
  (`yard_backoff*` 넷)가 있어 병합본 코드가 재생을 거절하므로, 그 넷을 첫 줄에서 빼고 넣었다(7절 11번).
- **zv1에서 본 것:** 445 s쯤 pode2가 E-2에서 Q2로 나가다가 50 s쯤 막혀 있었다(로봇단 양보가 BLOCKED로 끝남, Nav2가 "경로
  없음" 208, 관제가 구간을 막힘으로 표시하고 경로를 세 번 다시 잡음, `NO_START` 두 번). 주문은 모두 끝났고 위반은 없다.
  main의 판(mv3)에서도 같은 시각에 관제의 결정 순서는 같았고(pode2가 E-2로, pode1이 P-1에서 WORK를 기다림) 같은 자리에서
  로봇단 양보가 시작됐는데, 거기서는 1 s 만에 풀렸다. 로봇 코드는 이 병합으로 바뀌지 않았고, zv1의 기록을 main 코드에
  넣으면 main 코드도 같은 명령 522개를 낸다(위의 기록 재생). 그러니 그 막힘은 병합 때문이 아니라 그 판에서 로봇이 선
  자리 때문이다. 뒤의 두 판(zv2, zv3)에서는 나오지 않았다. main 코드에서도 같은 일이 생길 수 있다는 뜻이니 실물에서 눈여겨본다.
- **검수:** Codex 읽기 전용 검수를 단계 묶음마다 돌렸다(1부 0~2, 3~4, 5~6, A2, 병합 커밋). 동작 회귀 지적은 없었다. 확인된
  지적은 구조 시험 보강(반영), 문서 한 줄(반영), 7절 10번이다.

되돌리는 법: main의 `Merge refactor/yield-owner` 커밋을 `git revert -m 1 <그 커밋>`으로 되돌린다. 로봇 쪽 코드(`pode/*`)는
`pode/pode_fsm/package.xml`의 한 줄 말고는 바뀌지 않았으니, 이 병합만으로는 로봇에 다시 배포할 필요가 없다.

## 9. 아침 보고

- **끝낸 단계:** 1부 0~6단계 전부, 2부 A1·A2·A3. **하지 않은 단계:** A4(6절), 2부 4절 관문과 B단계(지시서대로).
- **브랜치와 마지막 커밋:** main = 이 기록이 든 병합 커밋. `refactor/yield-owner`(main에 병합됨), `refactor/zone-cleanup`
  d7d3ba8(1부만), `tools/yield-report` bf7848c(A1만), `refactor/yield-a3-fake-ladder` 25664cf(A3만),
  `feat/goal-plan-keeper` 771c971(main 미병합), `fix/goal-replan-on-invalid` b33e426(버린 실험).
- **바뀐 양(acfa45b 대비, 병합 커밋 ccb7336까지):** 파일 65개, +2565줄, -1263줄.
- **시험 수:** 8절. 정리 전 fleet_core 1118 passed → 1부 끝 1124 → A2 뒤 1090(지운 34개: `test_zone_backoff.py`의 24개와
  `test_params_validate.py`가 파라미터마다 만들어 내던 10개). fake_robots는 883 → A3로 895 → A2로 892(Windows 기준).
- **바꾸거나 지운 시험과 이유:** 3절과 4절. 1부에서 지운 시험은 없고, 바꾼 것은 내부 경로 이름뿐이다(기대값 그대로).
  A2에서 지운 것은 켜야만 도는 물러나기 시험이다.
- **전후 비교 표:** `docs/ZoneTraffic_구조.md` 맨 앞과 `docs/worklog_zone_cleanup_2026-11-18.md`. 믹스인 23개 → 16개,
  `Robot` 맨 위 필드 81개 → 32개.
- **사용자가 다음에 할 일:**
  1. 7절의 질문에 답한다.
  2. 실물 기록을 모은다: 관제를 `ros2 launch iponic_bringup pc.launch.py ... record:=<파일>`로 띄워 `~/real_runs/`에 둔다.
     그 기록으로 `python tools/yield_report.py <파일>`을 돌리면 로봇단 양보와 관제 근접 감시 가운데 누가 마주침을 풀었는지 나온다.
  3. 지시서 2부 4절의 관문 둘(실물에서 로봇단 양보의 후진이 동작하는지, 그 기록에서 로봇 사다리가 실제로 마주침을 푸는지)을
     확인한 뒤 B단계를 시작할지 정한다.
  4. `feat/goal-plan-keeper`를 실물에서 한 번 돌려 보고 main에 넣을지 정한다.
