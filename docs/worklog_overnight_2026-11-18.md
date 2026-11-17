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
| 2부 A1 양보 보고 도구 | 끝 | `tools/yield-report` bf7848c | tools 시험 5개 추가 | 제품 코드 변경 없음 |
| 2부 A3 가짜 로봇의 양보 사다리 | 끝(끔이 기본) | `refactor/yield-a3-fake-ladder` 25664cf | 1118 / 895 / 84, pode_fsm 366 passed 8 skipped | 끔일 때 0줄 |

- 시험 수는 Windows 파이썬으로 돌린 것이다. fake_robots의 7 skipped는 rclpy가 없어 건너뛴 `test_ros_*.py` 일곱 파일이고,
  tools의 14 failed는 `test_real_pode.py`가 Windows에서 bash를 못 찾는 기존 실패다(WSL에서는 통과한다).
- fleet_core가 1118에서 1121로 는 것은 새 구조 시험 셋(`test_zone_goal_life.py`) 때문이다. 지운 시험은 없다.
- 3·4단계는 커밋마다 빠른 검사(trace, store, order 표 diff, fleet_core 시험, ruff)를 하고, 전체 비교(`tools/zone_compare.sh`)는
  19447ed(3단계 첫 커밋), 82f805d(3단계 끝), 532a247(4단계 끝)에서 돌렸다. 모두 0줄이다.

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

## 4. 2부에서 한 일

- **A1:** `tools/yield_report.py`와 시험, 기록 `docs/worklog_yield_report_2026-11-18.md`(브랜치 `tools/yield-report`).
- **A3:** 머리 없는 세계의 가짜 로봇이 `pode_fsm.local_yield`의 순수 함수를 가져다 로봇단 양보 사다리를 흉내 낸다.
  `ZoneWorld(local_yield=True)`, `tools/situations.py --local-yield`로 켠다. 기본은 끔이고 끔일 때 기준과 0줄이다.
  - `sim/fake_robots/package.xml`에 `exec_depend pode_fsm`을 더했고, 순환을 피하려고 `pode/pode_fsm/package.xml`의
    `test_depend fake_robots`를 뺐다(동작은 그대로다. 질문 7절).
  - 실제 사다리와 다른 점: BLOCKED 뒤의 유지 동작(1 s 비움, 3 s 또는 0.15 m)이 없다. 막힘 판정 거리와 진행 문턱은 가짜
    로봇의 값을 쓴다. ROS 가짜 노드(`fake_pode_node.py`)에는 넣지 않았다.

## 5. 지시서 밖의 일 (같은 밤)

- 실물 SLAM 지도 브랜치와 "도리도리" 개선은 `feat/goal-plan-keeper` 브랜치의 기록 문서에 적는다(끝나면 여기에 경로를 적는다).

## 6. 건너뛴 것과 이유

- `station_left` 삭제: 지시서가 묻고 하라고 한 항목이라 지우지 않았다(7절 1번).
- 실물 기록 비교: 기록이 없다.

## 7. 질문 (아침에 사용자에게)

1. **`station_left`를 지울까요?** traffic 안에서 읽는 곳이 없다(`base.py`에서 만들기만 한다). 값을 넣는 곳은 `manager.py`의
   `hasattr` 분기이고 `test_traffic_api.py`에 `OPTIONAL`로 적혀 있다. 지우면 `manager.station_left_s`와 그것만 쓰는 `eta_*`
   파라미터 셋도 함께 죽는다.
2. **`reset_watch=False` 두 자리에서 멈춤 감시 기록을 초기화해야 할까요?** `order_answers._restore`(order update가 실패해
   예전 goal로 되돌릴 때)와 `results._arrived_anyway`(끝에 서 있는 로봇의 goal을 끝 goal로 바꾼 직후 지울 때)다. 지금 동작을
   그대로 두었다. 세 번째 자리였던 `watch.py`의 되살리기는 대입 자체가 없어졌다.
3. **v7 store 상황의 `KeyError: 'X_W'`를 고칠까요?** `tools/situations.py --store -l v7`이 죽는다.
4. **`pode_fsm`의 `test_depend fake_robots`를 빼도 될까요?** A3의 새 의존(`fake_robots` → `pode_fsm`)과 순환이 되어 뺐다.
   `test_ros_matches_fake_pode.py`는 `fake_robots`가 없으면 건너뛴다.
5. **가짜 로봇의 BLOCKED도 실제 로봇처럼 유지해야 할까요?** B3에서 정할 일이다.
6. A1 도구의 `--grace` 기본값 10 s는 담당 에이전트가 정한 값이다.

## 8. main에 넣을지의 결정

아직 정하지 않았다(작업 중).
