# 지금 Segment의 남은 시간을 로봇이 보낸 남은 거리로: 다음 작업 순서 4번 (2026-11-21)

Drive 문서 `nav2fleet_다음_작업_순서_2026-11-19`의 4번 단계다. 브랜치 `feat/distance-remaining-2026-11-21`(main a6265b0에서 시작).
코드, 단위 시험, 분석 스크립트를 만들었고, 같은 PC에서 main과 Gazebo 16판을 번갈아 돌려 비교했다(아래 "Gazebo 결과").

## 결론

- **바꾼 것:** 관제가 order의 지금 Segment가 nav_goal일 때, 그 Segment의 남은 시간을 로봇이 MoveTo 피드백으로 보내는 `distance_remaining`을 `nav_speed_mps`로 나눠 잰다. 그 뒤 Segment들은 전처럼 `travel_s`다.
- **안 쓰는 경우:** 값이 없음, 오래됨(1.0 s 넘게), 경로를 새로 짜서 값이 뛴 뒤 2.0 s 안, 로봇 자리에서 Segment 끝까지 곧은 거리보다 0.10 m 넘게 짧음, base 끝에서 기다리는 중, 끝난 seq의 값. 이때는 예전 투영(`left_s`)을 쓴다.
- **고정 경로(follow_path)와 물러나기(retreat) Segment는 투영 그대로다.** 이유는 아래 "로봇이 보내는 값"에 적었다.
- **Gazebo 결과(16판):** 두 쪽 모두 8판 통과, 위반 0. nav_goal Segment의 ETA 오차는 남은 거리를 쓰면 오히려 커졌다(같은 판에서 |오차| 평균 2.81 → 3.15 s, 부호 평균 +1.72 → +2.17 s). 속도 모델(0.12 m/s)이 이미 시간을 길게 보는데, Nav2 길이가 투영 거리보다 8 % 길어서 더 길게 보게 됐다. 급행 판에서는 도착 전 갱신이 0.2 s 늦었다(중앙값 0.50 → 0.30 s). ETA를 길게 본 탓으로 보지만, 판을 나눠 확인하지는 않았다. Q2 정지, Route 시간, 판 통과는 차이가 없었다. nav_goal Segment에서 남은 거리가 0.10 m 넘게 뛴 것은 12번이었고, 11번은 Segment 시작 1 s 안이었다. 주행 중간에 뛴 것은 피드백 하나짜리 한 번(+0.40 m)뿐이었고, 그때는 기준대로 투영을 썼다. **이 상태로는 main에 넣을 이유가 없다고 본다.** 넣을지는 사용자가 정한다. (2026-11-21 오후 갱신: Nav2 goal 끝 보정을 함께 넣은 통합 판에서는 남은 거리가 ETA를 좋게 했다. 맨 아래 "통합 판 Gazebo 비교"를 본다.)
- **시뮬 속도를 0.12 m/s로 맞춘 재비교(한쪽 6판):** 위 16판은 시뮬이 0.20 m/s로 돌아 관제 모델과 맞지 않았다. 0.12 m/s로 맞추자 투영의 치우침은 없어졌지만(|오차| 평균 1.17 s, 부호 −0.12 s), 남은 거리는 여전히 더 나빴고(1.38 s, +0.31 s) 갱신도 0.2 s 늦었다. main 쪽 v7 두 판이 이 브랜치와 상관없는 이유로 실패했다. 아래 "Gazebo 다시 비교" 절.
- **옛 기록 재생:** 남은 거리가 없는 입력은 기록에 예전 다섯 인자 그대로 남는다. 이 PC의 옛 Gazebo 기록 16판을 이 브랜치 코드로 재생했다. 수정 판(9a39efc) 8판은 모두 "same as recorded"다. 기준 판(5e11338) 8판은 22.8~27.7 s에서 갈린다. 이 판들은 2번 수정(`left_s`) 전 코드라 main에서도 갈린다(`fix_9a39efc/replay_fix.md` 참고).

## 로봇이 보내는 값

`pode/pode_fsm`에서 확인한 것이다.

- 피드백은 0.2 s마다(`server_ros.py`의 `FEEDBACK_PERIOD_S`) 나가고 `distance_remaining`, `last_done_seq_id`, `waiting_at_base_end`를 한 번에 담는다. 두 값은 같은 잠금 안에서 읽는다. 코드만 보고는 남은 거리가 seq `last_done_seq_id + 1`의 것이라고 보았다.
  - **Gazebo에서 본 예외:** 로봇이 nav_goal seq N을 끝냈다고 보고한 뒤 0.2~1.0 s 동안은 끝낸 seq가 N인데도 남은 거리는 seq N의 Nav2 goal 값(0.05~0.13 m)이 그대로 왔다. 새 goal의 Nav2 피드백이 오면 그 값으로 바뀐다. 브랜치 16판 가운데 급행 판에서 6번 보였다. 여섯 번 모두 바뀌기 직전 표본은 "곧은 거리보다 짧음" 기준에 걸려 쓰이지 않는 상태였다(곧은 거리 1.20~1.25 m. 아래 "Gazebo 결과" 4).
- **nav_goal:** 로봇은 nav_goal Segment를 하나씩 따로 달린다(`order_queue.mergeable`은 follow_path끼리만 잇는다). 관제가 합친 Segment(E-1 → E-3 → Q2)도 로봇에게는 Segment 하나, Nav2 goal 하나다.
  - 값은 Nav2 피드백의 남은 거리다(`nav_moves.nav_goal_feedback`). 그 Segment 끝까지 Nav2가 짠 길의 길이다.
  - Nav2 피드백이 오기 전이나 Nav2가 0을 보내면 로봇 자리에서 goal까지의 곧은 거리다(`nav_goal.distance_remaining`).
  - Nav2가 길을 다시 짜면 값이 뛸 수 있다. "도리도리" 수정(3번, `feat/goal-plan-keeper`)은 병합되지 않았다.
- **follow_path:** 로봇은 이어지는 follow_path Segment를 한 줄로 이어 달린다. 값은 그 줄 끝까지 남은 길이다(`core.distance_remaining`). 줄이 여러 seq를 덮고, 갱신으로 줄이 늘면 값도 늘어난다. 그래서 지금 Segment 하나의 값이 아니다. 관제의 투영도 같은 고정 경로 위에서 재므로 이 값으로 바꿔 얻는 것이 없다.
- **retreat:** 값이 0이다(`core.distance_remaining`이 Move, NavGoal만 잰다).
- base 끝에서 기다릴 때도 값이 0이다(달리는 goal이 없다).

## 바꾼 것

| 파일 | 내용 |
|---|---|
| `pc/fleet_manager/fleet_manager/adapters.py:131` | `move_feedback`이 `distance_remaining`을 넷째 값으로 돌려준다 |
| `pc/fleet_manager/fleet_manager/fleet_manager_node.py:305` | 피드백을 모두(0.2 s마다) 코어로 넘긴다. 값이 바뀔 때만 넘기던 `FeedbackFilter`(`order_goals.py`)는 지웠다. 서 있는 로봇의 남은 거리가 같은 값으로 이어지면 필터가 걸러 1 s 뒤 "오래됨"이 됐기 때문이다(리뷰 지적) |
| `pc/fleet_core/fleet_core/manager.py:123` | `on_move_feedback(..., distance_remaining=None)`. 값이 있을 때만 기록 줄에 여섯째 인자로 남긴다 |
| `pc/fleet_core/fleet_core/traffic/api.py:43` | 프로토콜에 같은 인자 |
| `pc/fleet_core/fleet_core/traffic/zone/robots.py:26,48` | `LeftReport(seq, m, at, jumped_at)`와 `MoveOrder.left` |
| `pc/fleet_core/fleet_core/traffic/zone/order_answers.py:14-17` | 기준값 `LEFT_STALE_S`, `LEFT_JUMP_M`, `LEFT_CALM_S`, `LEFT_SHORT_M` |
| `order_answers.py:49` `_note_left` | 피드백마다 표본을 갱신한다(지금 goal과 update 중인 이전 goal 모두) |
| `order_answers.py:64` `_reported_end` | 표본을 쓸 수 있으면 그 Segment의 끝 edge 번호, 아니면 None |
| `order_answers.py:82` `_left_s(m, route, end)` | 남은 시간. 표본이 없으면 `left_s` 그대로 |
| `goals.py:170`, `move_orders.py:117,125`, `order_answers.py:175` | `left_s(...)`를 `self._left_s(m, route, ...)`로 |

- 파라미터는 더하지 않았다. 기준값은 `move_orders.py`의 `HORIZON_HOLD_S`처럼 모듈 상수로 두었다. 그래서 파라미터 파일은 바뀌지 않는다.
- `goals._take`는 새로 고른 후보 Route에도 불린다. 이때는 `route is not m.route`라 투영을 쓴다(표본은 달리던 Route의 seq에 붙은 것이다).
- 남은 거리가 `None`이면 관제의 판단은 main과 같다. 이것은 옛 기록 재생(아래)과 시험 묶음 전체로 확인했다.

## 쓰지 않는 기준과 이유

표본은 `MoveOrder`에 붙어 order가 바뀌면 사라진다. ETA를 물을 때 아래를 차례로 본다. 하나라도 걸리면 투영을 쓴다.

1. **없음:** 표본이 없다. 옛 로봇이나 옛 기록(인자 다섯), 첫 피드백 전.
   - 값이 None, NaN, inf, 음수면 그 피드백은 표본으로 받지 않고 앞 표본을 "오래됨"으로 만든다. 앞 표본의 뛴 시각은 남겨 둔다(그 사이 나쁜 값 하나로 2 s 기다림이 풀리지 않게).
2. **base 끝에서 기다림:** `waiting_at_base_end`면 표본을 지운다. 로봇은 이때 0을 보낸다.
3. **다른 seq:** 표본의 seq가 관제의 `done_seq + 1`이 아니다. 또는 그 seq가 로봇에게 준 Segment(`cuts[:given]`)가 아니거나, 시작 edge가 `route.done`이 아니거나, nav_goal이 아니다.
   - 끝난 Segment의 값이 다음 Segment에 쓰이지 않게 하는 조건이다. 늦게 닿은 피드백(끝낸 seq가 관제보다 작음)도 여기서 걸린다.
4. **오래됨(`LEFT_STALE_S = 1.0` s):** 마지막 피드백이 1.0 s보다 오래됐다.
   - 피드백은 0.2 s마다 오므로 다섯 번을 놓친 것이다. 그때는 통신이 이상하다(`comm_timeout_s` 3.0 s보다는 짧게 잡았다).
   - 로봇이 0.12 m/s로 가면 1 s에 0.12 m, 곧 ETA 1 s 차이다(Gazebo처럼 0.20 m/s로 가면 0.20 m라, 0.12 m/s 모델로는 1.7 s 차이다). 투영의 평소 오차(아래 표, 1 s 안 구간 평균 0.89 s)와 비슷한 크기라 그 안에서는 받아 들인다.
5. **경로를 새로 짬(`LEFT_JUMP_M = 0.10` m, `LEFT_CALM_S = 2.0` s):** 같은 seq의 연이은 두 표본에서 값이 0.10 m 넘게 늘면 그때부터 2.0 s 동안 쓰지 않는다. 그 사이 또 뛰면 다시 2.0 s다.
   - 보통 진행은 피드백 사이 0.024~0.04 m 줄어든다(0.12~0.20 m/s × 0.2 s. Gazebo 판은 0.20 m/s). Nav2 costmap 칸이 0.025 m라 다시 짠 길의 잡음은 몇 cm다. 0.10 m는 칸 네 개다.
   - 길이 장애물의 다른 쪽으로 뒤집히면 길이가 그보다 크게 바뀐다. 줄어드는 쪽 뒤집힘은 막지 않는다(짧은 길이 맞으면 그 값이 맞다).
   - 2.0 s는 뒤집힘이 이어지는 동안 계속 투영을 쓰게 하는 시간이다. 왔다 갔다 하면 매번 늘 때마다 다시 센다.
   - 알고 있는 부작용: 첫 Nav2 피드백 전 값은 곧은 거리라, Nav2 길이 곧은 거리보다 0.10 m 넘게 길면 Segment 시작에서 한 번 "뛴 것"으로 보고 2 s 동안 투영을 쓴다. 갱신 판단은 Segment 끝 가까이(2 s 안쪽)에서 나므로 영향이 작다고 보았다.
6. **곧은 거리보다 짧음(`LEFT_SHORT_M = 0.10` m):** 남은 거리 < (로봇 보고 자리에서 Segment 끝 node까지 곧은 거리) − 0.10 m.
   - Nav2 길은 로봇 자리에서 goal까지라 곧은 거리보다 짧을 수 없다. 관제의 보고 자리는 0.1 s쯤 늦어 1~2 cm 어긋나므로 0.10 m를 둔다.
   - 다른 goal의 값이나 잘못된 값을 거른다.
- 그래프보다 **긴** 값(Nav2가 무엇을 피해 돌아가는 길)은 막지 않는다. 그것이 이 변경으로 얻으려는 정보다.

## 시험

시험 먼저 쓰고(빨강 확인) 고쳤다.

- `pc/fleet_core/test/test_zone_distance_left.py`(새 파일, 매개변수 포함 20개)
  - 쓰는 경우: 줄어드는 값을 쓴다, 그래프보다 긴 값을 믿는다.
  - 같은 값이 거듭 와도(서 있는 로봇) 2 s 넘게 쓰고, 거듭 온 대기 피드백에도 BASE_END_WAIT는 한 번이다.
  - 투영으로: 값 없음과 첫 피드백 전, NaN·inf·음수, 뛴 뒤 나쁜 값 하나가 끼어도 2 s 기다림은 이어짐, 오래된 값(경계 앞뒤 0.05 s), 뛴 값(잡음 안쪽은 쓰고, 뛴 뒤 2 s 동안 투영, 그 뒤 다시 씀), 왔다 갔다 하는 값, 곧은 거리보다 짧은 값(경계 앞뒤 0.01 m), base 끝에서 기다림, 고정 경로 Segment, 끝난 Segment의 값(d2에서 O_Q를 지나 3번 nav_goal을 받은 뒤 2번 때의 피드백이 늦게 닿음).
  - 판단에 닿음: v7에서 E-1 → Q2 nav_goal을 달릴 때 남은 거리가 곧은 거리보다 0.15 m 길면 Q1으로 늘리는 ORDER_UPDATED가 1.25 s(0.15 / 0.12) 늦게 나간다(±0.15 s). 시험 속 로봇이 `nav_speed_mps`(0.12 m/s)로 움직이기 때문이다. 로봇이 Gazebo처럼 0.20 m/s로 가면 같은 0.15 m가 0.75 s다.
  - 기록: 남은 거리 없이 돈 코어의 기록은 피드백 줄이 모두 인자 다섯(옛 형식)이고 그대로 재생된다. 남은 거리가 있는 기록은 인자 여섯이고 그대로 재생된다.
- `pc/fleet_manager/test/test_adapters.py`: `move_feedback`이 남은 거리를 float로 넷째 값에 담는다.
- `pc/fleet_manager/test/test_node_order.py`: 같은 피드백이 거듭 와도 모두 코어로 간다. `test_order_goals.py`의 필터 시험 셋은 필터와 함께 지웠다.
- `docs/distance_remaining_2026-11-21/test_eta_error.py`(4개): 분석 스크립트.
- 결과는 아래 "시험 결과" 절에 그대로 옮겼다.

## 분석 스크립트(Gazebo 비교용)

`docs/distance_remaining_2026-11-21/`

- `eta_error.py NAME=RESULT_DIR ... [--every=0.5] [--per-run]`: nav_goal Segment의 남은 시간 오차.
  - 판의 `fleet_record.jsonl`을 **이 브랜치 코드로** 재생한다(`tools/replay.py`와 같은 방법). 위치 보고마다(로봇마다 0.5 s에 하나) 지금 Segment가 nav_goal이면 두 예측을 잰다.
    - `proj`: 투영만(main의 방법).
    - `used`: 이 브랜치가 실제로 쓰는 값(`_left_s`). main 판과 옛 기록에서는 `proj`와 같다.
  - 실제는 그 Segment의 SEGMENT_DONE 시각 − 표본 시각이다. SEGMENT_DONE이 없는 Segment는 뺀다.
  - 실제 남은 시간 구간(0-1, 1-2, 2-4, 4-8, 8+ s)마다 |오차| 평균, p90, 부호 있는 평균, `used_from_distance`(남은 거리에서 나온 표본 비율)를 낸다.
  - 재생이 기록과 갈리면 그 시각을 경고한다. 그 뒤 표본은 그 판의 것이 아니다. main a6265b0 판과 이 브랜치 판은 갈리지 않아야 한다.
  - 판마다 따로 재생하므로 main 판은 `proj`, 브랜치 판은 `used`를 보면 "그 판이 쓴 방법의 오차"다. 브랜치 판의 `proj` 열은 같은 판에서 투영으로 쟀다면의 오차라 같은 조건의 비교가 된다.
- `extra_checks.py BATCH_DIR ...`: 묶음 전체의 갱신 시점 분포, 코어가 남은 거리가 뛰었다고 본 곳(재생, Segment 종류별), 속도를 바꿨다면의 ETA 오차(재생). Gazebo 판을 돌린 뒤 더했다. 시험은 `test_extra_checks.py`(2개).
- `report.sh BATCH_DIR [run ...]`: 한 묶음의 `step1_report.py`(판 통과, Route 시간, Q2 Route의 도착 전 갱신과 Q2에서 선 Route), `q2_dwell.py`(실제로 선 횟수), `eta_error.py`를 차례로 낸다.

옛 묶음에 시험으로 돌린 값(남은 거리가 없는 기록이라 `used` = `proj`):

| 묶음 | 표본 | Segment | |오차| 평균 | p90 | 부호 평균 | 0-1 s 구간 |오차| 평균 |
|---|---|---|---|---|---|---|
| `fix_9a39efc` 8판 | 1923 | 54 | 2.91 s | 5.08 s | +1.81 s | 0.89 s |
| `base_5e11338` 8판 | 499 | 14 | (22.8~27.7 s 뒤 재생이 갈려 그 뒤는 그 판의 값이 아님) | | | |

- 출력은 WSL `~/gz_runs/fix_9a39efc/dist_report.md`, `~/gz_runs/base_5e11338/dist_report.md`.
- **투영은 nav_goal Segment의 남은 시간을 늘 길게 본다(부호 평균 +1.81 s, 마지막 1 s에서도 +0.72 s).** 남은 거리로 바꿔도 같은 속도 모델(`nav_speed_mps` 0.12)로 나누므로 이 치우침은 그대로 남는다고 본다. 짐작되는 원인 둘(확인하지 않음):
  - 로봇은 다음 Segment가 이어지면 goal 0.15 m 앞(`OrderParams.pass_m`)에서 다음 Segment로 넘어간다. 0.15 m는 0.12 m/s로 1.25 s다.
  - Gazebo 판의 로봇은 0.12 m/s보다 빠르다. `gz.launch.py`의 `nav_max_speed` 기본값 0.20 m/s가 RPP `desired_linear_vel`(avoidance.yaml 0.25)을 0.20으로 낮추고, 이 묶음들은 그 값을 바꾸지 않았다. `nav_speed_mps` 0.12는 관제가 ETA와 길 비용에 쓰는 제안값이고 시뮬 속도에 맞춘 적이 없다.
  - 남은 거리가 줄여 줄 수 있는 것은 길이 오차(그래프와 Nav2 길의 차이, 돌아가는 길)뿐이다. 속도 모델은 이 단계에서 고치지 않았다.

## Gazebo 비교 방법(메인 세션이 돌린다)

조건은 `docs/worklog_left_s_projection_2026-11-20.md` §조건과 같다(도메인 61, `env -i`, LOCALHOST, rmw_fastrtps_cpp, 판마다 truth_logger, 25 s 쉼).

### 1. 공유 install을 a6265b0으로

공유 install `~/gg_build/wt-nf-main/install`은 Windows 작업 트리 `C:\Users\pjw\ros2camp\wt-nf-main`을 `wt_env.sh`의 `wt_build`로 빌드한 것이다.

```bash
# Git Bash: 작업 트리가 a6265b0인지 보고, 아니면 맞춘다(그 작업 트리에 다른 일이 없을 때만)
git -C /c/Users/pjw/ros2camp/wt-nf-main status --short
git -C /c/Users/pjw/ros2camp/wt-nf-main checkout --detach a6265b0
# WSL: 전체 빌드
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc 'export WT_NAME=wt-nf-main; source /mnt/c/Users/pjw/AppData/Local/Temp/claude/95f025f67f393570/ed85ca25-6e98-4e0c-9dea-520ff81b1351/scratchpad/nf/wt_env.sh && wt_build'
```

### 2. 이 브랜치의 오버레이

main과 다른 패키지는 `pc/fleet_core`, `pc/fleet_manager` 둘이다(`sim/fleet_gz_sim`, `pode/*`는 그대로). **`fleet_manager`가 빠지면 노드가 남은 거리를 코어에 넘기지 않아 main과 같아진다.**

```bash
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc '
source /opt/ros/jazzy/setup.bash && source ~/pinky_deps_ws/install/local_setup.bash
source ~/gg_build/wt-nf-main/install/setup.bash
O=~/gz_runs/ovl_dist; rm -rf $O; mkdir -p $O/src && cd $O
rsync -a --delete --exclude .git /mnt/c/Users/pjw/ros2camp/nf-distance/ src/
colcon build --base-paths src/pc/fleet_core src/pc/fleet_manager'
```

확인: 오버레이를 source한 셸에서 `python3 -c 'import fleet_core, fleet_manager; print(fleet_core.__file__, fleet_manager.__file__)'`가 둘 다 `~/gz_runs/ovl_dist/install/...`이어야 한다. 판이 끝난 뒤에는 `eta_error.py`의 `used_from_distance`가 0보다 커야 한다(0이면 남은 거리가 코어에 닿지 않은 것이다).

### 3. 묶음 돌리기

```bash
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc '
cd /mnt/c/Users/pjw/ros2camp/nf-distance
export SIM_BATCH_INSTALL=$HOME/gg_build/wt-nf-main/install/setup.bash
export SIM_BATCH_ENV="WAYLAND_DISPLAY=wayland-0 GALLIUM_DRIVER=d3d12" SIM_BATCH_PAUSE_S=25
S=docs/baseline_sim_2026-11-19/sim_batch.sh
R="rush1:gz_v7_rush rush2:gz_v7_rush rush3:gz_v7_rush rush4:gz_v7_rush v7a:gz_v7 v7b:gz_v7 v7c:gz_v7 v7d:gz_v7"
bash $S ~/gz_runs/main_a6265b0 - $R > ~/gz_runs/main_a6265b0.batch.log 2>&1
bash $S ~/gz_runs/dist_<SHA> $HOME/gz_runs/ovl_dist/install/setup.bash $R > ~/gz_runs/dist_<SHA>.batch.log 2>&1'
```

- `<SHA>`는 오버레이를 빌드한 커밋이다. 16판, 한 판 4~5 분이라 1시간 남짓이다. 무거운 계산은 22시~07시 규칙을 따른다.
- `sim_batch.sh`가 쓰는 truth_logger와 시각 규칙은 그대로다.

### 4. 표

```bash
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc '
cd /mnt/c/Users/pjw/ros2camp/nf-distance
for b in main_a6265b0 dist_<SHA>; do bash docs/distance_remaining_2026-11-21/report.sh ~/gz_runs/$b > ~/gz_runs/$b/dist_report.md 2>&1; done'
```

- 볼 지표: nav_goal Segment의 ETA 오차(main은 `proj`, 브랜치는 `used`. 브랜치 판에서는 두 열을 같은 판으로 비교), 도착 전 갱신(`update_before`, `joined_before`, `lead_median_s`), Q2에서 선 Route(`q2_stood`)와 실제로 선 횟수(`q2_dwell`), Route 평균(`route_mean_s`, `q2_route_mean_s`), 판 통과(`pass`).
- 두 묶음 모두 재생 경고가 없어야 한다(main 판을 이 브랜치 코드로 재생해도 같아야 한다. 남은 거리가 없는 입력은 main과 같게 판단하기 때문이다).

## 시험 결과

WSL 전체 시험(`fulltest.sh`). 출력 그대로다.

- dist1(c23c185, 피드백 필터를 지우기 전):
  - `pc/fleet_core: 1193 passed, 7 xfailed`, `pode/pode_fsm: 467 passed, 6 skipped`, `sim/fake_robots: 1120 passed, 1 skipped, 27 xfailed`, `sim/fleet_gz_sim: 600 passed`, `tools: 194 passed`, `pc/fleet_manager: 106 passed, 34 skipped`, 노드 시험 `34 passed, 1 warning`. 나머지 pode 패키지도 모두 통과.
- dist2(fcf3cd0, 최종):
  - `pc/fleet_core: 1200 passed, 7 xfailed`, `pode/pode_fsm: 467 passed, 6 skipped`, `sim/fake_robots: 1120 passed, 1 skipped, 27 xfailed`, `sim/fleet_gz_sim: 600 passed`, `tools: 194 passed`, 노드 시험 `34 passed`.
  - `pc/fleet_manager: 2 failed, 101 passed, 34 skipped`. 실패 둘:
    - `stock_shortage`: 위반 `t=110.6 WORK_DOCK released from pode1 while 0.08 m from it`. 다시 돌려 `1 passed`. main에서도 간헐 실패가 알려진 시나리오다.
    - `shared_lane_failure`: 900 s 시간 초과, 위반 `unexpected ERROR event ROBOT_ISOLATED`, GOAL_FAILED 1(기대 0), O-005 HELD. 다시 네 번 돌려 네 번 다 `1 passed`(약 48 s).
    - 이 실패는 지금까지 알려진 "위반 0의 시간 초과"와 다르다. 처음에는 결과 폴더가 사라진 줄 알았지만, `kept_runs.py`가 실패한 판을 WSL `~/fleet_test_failures/test_scenario_finishes_and_matches_the_contract-shared_lane_failure_20261121_025505/`에 남겨 두었다(2026-11-21 04:50에 확인).
    - **실패한 판에서 본 것:** 시나리오는 50 s부터 pode1에 nav 중단 두 번을 넣는다. 그 두 번은 70.8 s, 73.3 s에 쓰였고 세 번째 시도(75.3 s)로 지나갔다. 그 뒤 127.0 s에 pode1이 WORK 도크에서 물러나다 "뒤에 무언가 있음"(RETREAT_WAIT)으로 멈췄고, 128.9 s에 관제가 대기 자리를 버리고 O_Q로 가는 order(STANDBY_LEFT, pode1-40)를 보냈다. 그때 pode3은 W_Q 쪽(약 (0.18, 0.55))에서 움직이고 있었고 pode1(약 (0.31, 0.85))과 0.3 m쯤 떨어져 있었다. pode1은 129.8, 132.5, 135.2 s에 NAV_ABORTED를 받아 GOAL_FAILED → ROBOT_ISOLATED가 됐다. 이 세 번이 무엇 때문인지는 가려내지 못했다. 가짜 로봇의 길 찾기(`fake_pode_node.py`의 `goal.nav.update`)가 pode3 때문에 막혔을 수도 있고, 넣은 고장이 70 s대에 다 쓰이지 않았을 수도 있다(복구 중 다시 중단되거나, 중단 전에 끝난 목표는 횟수를 돌려받는다. `motion.py`의 `give_back_nav_abort`). 가짜 로봇은 중단 이유를 로그에 남기지 않는다.
    - **남은 거리가 판단을 바꿨는지:** 이 판의 `fms.jsonl`은 이 브랜치 코드로 그대로 재생된다("same as recorded"). 피드백 줄 955개에서 남은 거리만 지우고 재생하면 75.5 s에서 갈린다. 남은 거리가 없으면 pode2의 구역 허가(HALL+J_W+WORK_DOCK → W)가 75.5 s에 나고, 기록(남은 거리 있음)에서는 75.7 s에 났다. 곧 남은 거리를 쓰는 것만으로 이 판의 판단 시각이 0.2 s쯤 바뀐다. 이 재생은 main과 같은 조건이 아니다: 필터를 지운 이 브랜치의 피드백(같은 값의 중복 포함)이 모두 남아 있고, 피드백마다 코어가 `_pump`를 돈다. 그래서 main(필터 있음)과 판단 시각이 어떻게 다른지는 이 재생으로 알 수 없다. 129 s의 막힘이 이 변경 때문인지, main에서도 같은 배치가 오면 나는지도 가려내지 못했다. 다시 만든 기록은 남기지 않았다(scratchpad).
    - **다시 돌림(2026-11-21 04:32~04:45, Gazebo 판이 모두 끝난 뒤):** `shared_lane_failure` 하나만 main(a6265b0 빌드)과 이 브랜치(같은 소스의 빌드)에서 번갈아 8번씩 돌렸다. **16번 모두 `1 passed`**(한 번에 47~54 s). 따로 돌리면 재현되지 않았다. 로그는 WSL `~/nf_logs/lane_reruns_2026-11-21/`.
    - **fleet_manager 시험 전체를 다시 돌림(04:47~05:32):** 실패가 났던 조건(`fulltest.sh`처럼 `ROS_DOMAIN_ID`를 비우고 `pytest test` 전체)으로 main → 브랜치 → main → 브랜치 차례로 두 번씩 돌렸다. main은 두 번 모두 `106 passed, 34 skipped`(671 s, 667 s), 브랜치는 두 번 모두 `103 passed, 34 skipped`(677 s, 668 s)였다. 브랜치가 셋 적은 것은 지운 필터 시험 셋 때문이다. 새로 남은 실패 판은 없다. 로그는 WSL `~/nf_logs/fm_compare_2026-11-21/`.
    - 정리: 브랜치에서 한 번 났던 `shared_lane_failure` 실패는 그 뒤 단독 16번, 전체 4번에서 다시 나지 않았다. 남은 거리가 이 판의 판단 시각을 바꾸는 것은 재생으로 확인했으므로 이 브랜치와 관계있을 수는 있지만, 이 횟수로는 실패가 main보다 잦은지 가려낼 수 없다.
- 로컬(Windows) `docs/distance_remaining_2026-11-21/test_eta_error.py`: `4 passed`.

## Gazebo 결과

2026-11-21 03:09~04:21, 이 PC(WSL), 도메인 61. 위 "Gazebo 비교 방법"대로 돌렸다.

- main은 공유 install(a6265b0), 이 브랜치는 그 위에 fcf3cd0으로 빌드한 오버레이(`fleet_core`, `fleet_manager`)다. fcf3cd0 뒤의 커밋은 문서뿐이다.
- 판 순서는 같은 이름의 판끼리 짝을 지어 main과 브랜치를 번갈아 앞에 두었다(rush1 main → dist, rush2 dist → main, …). 시간대에 따른 차이가 한쪽에만 쌓이지 않게 하려는 것이다. 순서와 시각은 `docs/distance_remaining_2026-11-21/gz_2026-11-21/run_order.log`에 있다.
- 비교 판을 돌리는 동안 Isaac, 학습, 다른 Gazebo는 돌리지 않았다. 판마다 WSL 남은 메모리는 15.1~15.2 GB였다.
- 출력 원본: 같은 폴더의 `report_main_a6265b0.md`, `report_dist_fcf3cd0.md`(`report.sh`), `rtf_runs.md`(실시간 비율), `extra_checks.md`(`extra_checks.py`).

### 표

| 지표 | main a6265b0 | 이 브랜치 |
|---|---|---|
| 판 통과 | 8/8(주문 급행 12/12, v7 8/8) | 8/8(같음) |
| 실시간 비율(truth.csv의 sim 시간 / 벽시계) | 판마다 0.999~1.000 | 판마다 0.999~1.000 |
| 위반 / 로봇 사이 최소 거리 | 0 / 0.203~0.232 m | 0 / 0.206~0.227 m |
| 판 시간 평균(급행 / v7) | 245.0 / 195.2 s | 244.8 / 194.1 s |
| nav_goal Segment ETA \|오차\| 평균 / p90 / 부호 평균 | 2.94 / 5.10 / +1.83 s | **3.15 / 5.87 / +2.17 s**. 같은 판을 투영으로 쟀다면 2.81 / 5.01 / +1.72 s |
| 같은, 실제 남은 시간 0-1 s | 0.91 / 1.55 s | 1.06 / 1.84 s(투영 0.86 / 1.45 s) |
| 같은, 1-2 s | 1.60 / 2.17 s | 1.71 / 2.59 s(투영 1.50 / 2.10 s) |
| 같은, 2-4 s | 2.61 / 3.39 s | 2.74 / 3.90 s(투영 2.48 / 3.32 s) |
| `used_from_distance`(남은 거리에서 나온 표본 비율) | 0 | 0.97 |
| Q2 Route 80개의 갱신: 도착 0.1 s 넘게 전 / 도착 때 / 도착 뒤 | 78 / 0 / 2 | 77 / 1 / 2 |
| 갱신이 도착보다 앞선 시간, 중앙값(급행 / v7) | 0.50 / 0.50 s | **0.30** / 0.50 s |
| 같은, 평균(급행 / v7) | 0.30 / 0.53 s | 0.14 / 0.42 s |
| Q2에서 선 Route / 실제로 선 횟수 | 2 / 3 | 2 / 3 |
| Route 평균(급행 / v7) | 14.04 / 11.77 s | 13.95 / 11.79 s |
| Q2 Route 평균(급행 / v7) | 15.35 / 12.97 s | 15.14 / 12.97 s |
| 판당 MoveTo 피드백 입력 / 기록 줄 | 96~119 / 8,311~10,662 | 1,365~1,860 / 9,567~12,410 |

- ETA 줄의 main 값은 main 판을 투영으로, 브랜치 값은 브랜치 판을 `_left_s`로 잰 것이다. 두 묶음 모두 재생 경고가 없었다(이 브랜치 코드로 재생해도 기록과 같다).
- 갱신 시점은 판마다의 중앙값(`lead_median_s`)이 아니라 묶음의 Q2 Route를 모두 모아 잰 것이다(`extra_checks.py`). 판마다 값은 report 파일에 있다(main은 모두 0.5, 브랜치는 0.3~0.5).

### 읽은 것

1. **ETA가 나빠진 까닭:** 남은 거리로 잰 값은 같은 표본의 투영보다 중앙값으로 8.4 % 길다(남은 거리를 쓴 표본 1,649개, 투영이 0.5 s 넘는 것). 투영은 이미 시간을 길게 보고 있었으므로(+1.72 s), 길이가 길어지자 치우침이 더 커졌다. Nav2 길이 그래프보다 긴 것은 맞는 정보지만, 지금 속도 모델로 나누면 오차를 키운다.
2. **속도를 바꿔 다시 셈(재생만, 판을 다시 돌리지 않음):** 두 예측 모두 nav_goal Segment에서는 거리 / `nav_speed_mps`라 속도만 바꿔 같은 표본을 다시 셀 수 있다(`extra_checks.py`, 브랜치 판 1,975개 표본).

   | 속도 | 투영 \|오차\| 평균 / p90 / 부호 평균 | 남은 거리 \|오차\| 평균 / p90 / 부호 평균 |
   |---|---|---|
   | 0.12 m/s(지금) | 2.81 / 5.01 / +1.72 s | 3.15 / 5.87 / +2.17 s |
   | 0.16 m/s | 1.62 / 2.78 / +0.28 s | 1.86 / 2.91 / +0.61 s |
   | 0.18 m/s | 1.30 / 2.92 / −0.20 s | 1.50 / 2.80 / +0.09 s |
   | 0.20 m/s | 1.12 / 3.08 / −0.59 s | 1.26 / 2.95 / −0.33 s |
   | 평균 \|오차\|가 가장 작은 속도 | 0.212 m/s에서 1.08 s | 0.228 m/s에서 1.12 s |

   - 이 비교의 Gazebo 판은 모두 `nav_max_speed` 기본값 0.20 m/s로 돌았다(`run_info.txt`에 그 인자가 없다). 관제는 0.12 m/s로 ETA를 셌으므로 두 쪽 모두 처음부터 속도가 맞지 않은 조건이었다. 가장 잘 맞는 속도 0.212~0.228 m/s는 이 상한과 가깝다.
   - 속도를 맞추면 두 방법 모두 평균 1.1 s쯤으로 줄고, 남은 거리 쪽이 낫지 않다. Gazebo 경기장에는 돌아갈 장애물이 없어 남은 거리가 줄여 줄 수 있는 "길이 오차"가 작기 때문으로 본다.
   - 이 속도는 같은 표본에 맞춘 값이라 검증된 값이 아니다. 가장 좋은 속도에서도 실제 남은 시간이 짧으면 +, 길면 −로 치우친다(0-1 s +0.16 s, 8 s 넘음 −1.74 s). 속도 하나로는 출발·감속 같은 고정 시간을 담지 못한다.
   - `nav_speed_mps`는 ETA뿐 아니라 길 비용(`goals.py:76`)에도 쓰여서, 바꾸면 Route 선택도 달라질 수 있다. 이 단계에서는 바꾸지 않았다.
3. **갱신이 늦어진 것:** 급행 판에서 Q2로 이어 붙이는 갱신이 도착 0.50 s 전에서 0.30 s 전으로 늦어졌다. ETA를 길게 볼수록 갱신 판단이 늦게 나기 때문으로 본다(위 시험 "판단에 닿음"과 같은 방향). 이번 8판에서는 Q2 정지와 Route 시간이 달라지지 않았지만, 여유가 0.2 s 줄었다.
4. **값이 뛴 경우(`LEFT_JUMP_M`):** 기록을 이 브랜치 코어로 재생하며, 피드백을 받은 뒤 코어가 표본에 새 뛴 시각을 찍은 곳을 셌다(`extra_checks.py`의 `core_jumps`. `_note_left`의 판정 그대로다). 브랜치 8판에서 124번이었다.
   - 112번은 follow_path Segment였다. 고정 경로의 값은 갱신으로 줄이 늘면 커지고(위 "로봇이 보내는 값"), ETA에는 쓰지 않으므로 영향이 없다.
   - nav_goal Segment에서는 12번이었다. 판마다 줄과 그때 로봇 자리에서 Segment 끝까지 곧은 거리, 직전 표본을 쓸 수 있었는지가 `extra_checks.md`에 있다.
     - 5번: seq의 첫 값 뒤 0.2~1.0 s에 곧은 거리와 비슷한 값(곧은 거리와 −0.01~+0.06 m 차이)이 0.10~0.18 m 긴 값으로 바뀜(예: 1.586 → 1.763 m). 첫 Nav2 피드백 전의 곧은 거리가 Nav2 길이로 바뀐 것으로 본다(위 "알고 있는 부작용"). 로봇 쪽 값을 따로 보지는 않았다.
     - 6번: 앞 goal의 값(0.05~0.13 m)이 새 seq에 남아 있다가 바뀜(예: 0.065 → 1.245 m, seq의 첫 값 뒤 0.2~1.0 s). 위 "로봇이 보내는 값"의 예외다. 바뀌기 직전 표본은 여섯 번 모두 쓰이지 않는 상태였다.
     - 1번(v7d 69.2 s, pode2-27 seq 2): Segment 끝 0.06 m 앞에서 피드백 하나만 0.084 → 0.488 m로 뛰고, 다음 피드백(0.2 s 뒤)에서 0.047 m로 돌아왔다. 원인은 확인하지 못했다(Nav2가 goal 가까이에서 길을 한 번 다르게 짰을 수 있다). 뜀 기준이 막으려던 종류이고, 그 뒤 2.0 s는 기준대로 투영을 썼다.
   - "도리도리"처럼 길이 이어서 뒤집히는 경우는 이번 판들에서 나오지 않았다. 그래서 기준값 0.10 m / 2.0 s가 맞는지는 위 한 번과 단위 시험으로만 보았다. 0.10 m 이하로 바뀐 길은 이 판정으로 가려내지 못한다.
5. **Route 시간과 Q2 정지:** 차이가 없다고 본다. Route 평균 차이(급행 −0.09 s, v7 +0.02 s)는 판 사이 흔들림(급행 13.80~14.18 s)보다 작다.

### 판단

- 지금 속도 모델에서는 남은 거리가 ETA를 나쁘게 하고 갱신을 늦춘다. main에 넣지 않는 것을 권한다. 브랜치는 남겨 둔다.
- 다시 볼 순서로는 (1) `nav_speed_mps`나 ETA 모델을 Gazebo와 실물 주행 기록으로 맞추는 일을 먼저 하고, (2) 그다음 장애물로 돌아가는 길이 생기는 실물이나 장면에서 남은 거리를 다시 비교하는 것을 제안한다. 둘 다 사용자 결정이다.
- 피드백을 모두 코어로 넘기게 바꾼 것(필터 삭제)으로 입력이 판당 15배쯤 늘었다(기록 줄은 약 15 % 늚). 판 시간과 실시간 비율에는 차이가 보이지 않았다.

## Gazebo 다시 비교: 시뮬 속도 0.12 m/s (2026-11-21 오전)

위 16판은 Gazebo 기본 속도 0.20 m/s로 돌아 관제 모델(0.12 m/s)과 맞지 않았다. 사용자 지시로 시뮬도 0.12 m/s로 맞추고 판 수를 줄여 다시 돌렸다.

- 조건: main a6265b0와 이 브랜치(오버레이 fcf3cd0, 그 뒤 코드 변경 없음)를 판마다 번갈아. 한쪽에 gz_v7_rush 3판, gz_v7 3판. 위 묶음의 launch 인자에 `rviz:=none gui:=false nav_max_speed:=0.12`를 더했다(`sim_batch.sh`의 `SIM_BATCH_ARGS`, 750cad1). 10:28~11:35, 다른 무거운 작업 없음.
- 속도 확인: 위치 기록(`truth.csv`)에서 로봇 최고 속도가 0.120 m/s였다(지난 묶음은 0.200 m/s).
- 결과 파일: `docs/distance_remaining_2026-11-21/gz_v012_2026-11-21/`(report_*.md, extra_checks.md, rtf_runs.md, run_order.log). 판 원본은 WSL `~/gz_runs/main_a6265b0_v012`, `~/gz_runs/dist_fcf3cd0_v012`.
- `extra_checks.py`는 이제 `summary.json`이 없는 판(끝나지 못한 판)을 건너뛴다. 실패한 판의 ETA는 아래 표에 넣지 않았다.

| 지표 | main a6265b0 | 이 브랜치 |
|---|---|---|
| 통과 | 5/7(v7 두 판 실패, 아래) | 6/6 |
| 실시간 비율 | 0.995~0.999 | 0.980~0.998 |
| nav_goal ETA \|오차\| 평균 / p90 / 부호 평균 | 1.14 / 1.86 / −0.10 s(5판, 1,848개) | 1.38 / 1.98 / +0.31 s(6판, 2,101개). 같은 판을 투영으로 쟀다면 1.17 / 1.97 / −0.12 s |
| 도착 전 갱신 시점 중앙값(급행 / v7) | 0.90 / 0.90 s | 0.70 / 0.70 s |
| Q2에서 선 Route(급행 3판) | 3 | 6 |
| 실제 Q2 정지(위치 기록, 급행 3판) | 6 | 6 |
| Route 평균(급행 / v7) | 20.00 / 17.27 s | 19.88 / 16.91 s |
| 평균 \|오차\|가 가장 작은 속도 | 0.126 m/s에서 1.08 s | 투영 0.127 m/s에서 1.09 s, 남은 거리 0.136 m/s에서 1.13 s |

읽은 것:

1. 속도를 맞추자 투영의 치우침이 없어졌다(+1.72 → −0.12 s). 가장 잘 맞는 속도도 0.126~0.127 m/s로, 모델 0.12와 거의 같다. 지난 비교의 큰 오차는 시뮬 속도가 모델과 다른 것으로 설명된다(지난 판을 재생에서 0.21 m/s로 다시 세었을 때도 1.08 s였다). 다만 판이 서로 달라 다른 원인을 모두 지운 것은 아니다.
2. 그래도 남은 거리는 투영보다 낫지 않다. 남은 거리로 잰 값은 투영보다 중앙값 7.6 % 길고(1,902개), 그만큼 ETA를 길게 본다(+0.31 s). Gazebo 경기장에서는 Nav2 길이 그래프의 곧은 거리보다 조금 길 뿐 돌아가는 길이 없어서, 남은 거리가 줄여 줄 오차가 작다고 본다.
3. 갱신 시점 중앙값이 0.2 s 늦은 것도 지난 비교와 같다(급행·v7 모두 0.90 → 0.70 s). 판이 서로 달라서 남은 거리를 쓴 것이 갱신을 늦춘 원인이라고 확정하지는 않는다. 급행 판에서 Q2에 선 Route(`q2_stood`)가 판마다 1번에서 2번으로 늘었다. 모두 `step1_report.py`가 `stop_q1_busy`로 분류한 것이다(Q2 도착 무렵 BASE_END_WAIT이고 다른 로봇이 Q1 근처에 있음. Q1 때문에 섰다는 것을 따로 확인하는 분류는 아니다). 위치 기록으로 센 실제 Q2 정지는 양쪽 모두 판마다 2번이라, 로봇이 실제로 더 선 것은 확인되지 않았다.
4. 뜀: nav_goal Segment에서 0.10 m 넘게 뛴 것은 18번이었다. 17번은 seq의 첫 값 뒤 0.2~0.8 s(곧은 거리가 Nav2 길이로 바뀜, 앞 goal의 값이 남았다가 바뀜)였고, 주행 중간은 급행 1판 83.9 s의 한 번(Segment 끝 0.12 m 앞에서 0.145 → 0.453 m)뿐이었다. 지난 비교와 같은 모습이다.
5. main 쪽 실패 두 판(이 브랜치 코드가 없는 판이다. 0.20 m/s 16판에서는 없었다). 근거 로그와 위치 기록은 `gz_v012_2026-11-21/failed_runs.md`에 뽑아 두었다:
   - v7b(시뮬 시각 84 s쯤부터): pode2가 출고 도크를 나와 O_OUT 근처 (0.19, 0.74)에 남쪽을 보고 선 뒤 Nav2 planner가 Q2까지 길을 내지 못했다(`compute_path_to_pose` 중단). 세 번 다시 보냈지만 모두 NAV_ABORTED로 격리됐다. 그때 다른 두 로봇은 (1.85, 0.46), O_X에 있어 남쪽 길을 막지 않았다. 위치 추정은 맞았다(실제와 0.02 m 차이).
     - 원인(조사 에이전트가 찾고 직접 확인함): 관제가 출구 고정 경로(O → O_OUT)를 가운데서 끊었다. 주차하러 가던 order(pode2-29) 도중 작업 호출이 와서 route가 stale이 됐고(`calls.py:142-146`), `_extend_order`는 고정 경로라도 "넓은 구역의 노드에 섰으면" 취소한다(`move_orders.py:105-110`). 82.624 s의 보고 위치 (0.181, 0.762)는 HALL 사각형(y 0.765까지, `fleet_map_v7.yaml:87`) 안이고 노드에서 0.15 m(`arrival_tolerance_m`) 안이라 이 조건이 참이었다. 기록에 82.7 s `CancelCmd goal 29`, 82.8 s Q2로 가는 nav_goal order pode2-30이 있다.
     - 그런데 Nav2 keepout은 OUT_DOCK을 0.06 m 넓혀 y 0.705까지 막는다. `arena_v7_keepout.pgm`에서 (0.175, 0.752), (0.19, 0.741)은 막힌 칸(0), O_OUT (0.18, 0.62)은 열린 칸(254)이다. 막힌 칸에서 출발하는 길은 Nav2가 만들지 못해 로봇이 그 자리에 섰다(이 마지막 고리는 Nav2 내부를 보지 않은 추측).
     - 관제의 구역 경계(0.765)와 keepout 경계(0.705) 사이 6 cm 띠를 로봇이 지나는 0.5 s 안에 호출이 와야 나는 경합이다. 48판 가운데 이 판에서만 났다. 이 취소 로직은 이 브랜치에서도 바뀌지 않았다(브랜치의 `move_orders.py` 변경은 `_left_s` 두 줄뿐이다).
   - v7d(v7b를 다시 돌린 판): pode3이 급행 통로를 서쪽으로 가는 동안 실제로는 x 1.52 부근에서 제자리로 방향만 바꾸다가 x 1.32에 섰는데, 관제가 받은 위치는 O_X(0.71)를 지나 O에 닿았다. 출고 도크 붙기가 103 cm 어긋나 세 번 실패하고 격리됐다. 보고 위치가 실제와 0.8~1.0 m 어긋난 것이다.
     - 원인(조사 에이전트가 찾고 기하와 위치 기록은 직접 확인함): 급행 통로 들머리에서 바퀴가 진열대 A의 동쪽 끝에 걸렸다. 시뮬 진열대 A의 급행 쪽 면은 y 0.793, 동쪽 끝은 x 1.5605다(`arena_v7.sdf:54`). 바퀴 충돌 구는 반지름 0.028, 중심이 몸 중심에서 0.04855라(`pinky.urdf.xacro`의 관절 0.04055 + 0.008) 바깥 끝이 0.0766이다. 곧 로봇 중심이 y 0.8696보다 낮으면 바퀴 구가 진열대 면에 닿는다. 급행 줄은 y 0.868이라 줄 위에서도 1.6 mm 겹친다.
     - pode3은 x 1.563, y 0.864, 방향 −2.91에서 시계 반대로 돌기 시작해 1 s 만에 −1.80(남쪽)이 됐고 x는 1.529까지만 갔다(truth.csv 77.0~77.9 s). 관제가 받은 위치는 77.8 s부터 실제와 갈라져 0.105 m/s로 벌어졌다(바퀴 회전을 그대로 더한 odometry로 보이고, AMCL은 통로에서 이를 바로잡지 못했다. 이 부분은 추측).
     - 통로를 지나는 로봇의 y 중앙값(판·로봇마다): 0.12 m/s 판 0.870~0.875, 0.20 m/s 판 0.871~0.881. 0.12 m/s에서는 닿는 선(0.8696)에서 1 mm 안쪽을 지나고, 이 판의 pode3은 들머리에서 0.864까지 내려왔다.
     - 2026-11-20에 사용자가 시뮬 진열대 A 면을 1 cm 물리기로 했지만(0.803 → 0.793, `fleet_map_v7.yaml` sim_shelves), 그 계산은 몸 반폭 0.056 기준이라 바깥으로 더 나온 바퀴 충돌 구(0.0766)를 담지 못했다.
   - 통과한 판까지 합친 스스로 복구(`recovery_probe`의 RECOVERING 시작, 목록은 `gz_v012_2026-11-21/recoveries.txt`) 횟수: main 7판 9번(O_IN 6번, Q2 2번, O_OUT 근처 1번), 이 브랜치 6판 3번. 0.20 m/s 16판에서는 main 1번, 이 브랜치 2번이었다. 열어 본 O_IN의 한 번(main 급행 2판 81.5 s)은 끝 방향(110도)으로 도는 0.9 s짜리였고 판은 이어졌다.
   - 둘 다 이 비교의 지표와는 따로 볼 문제다. 두 원인(취소 경합, 바퀴 걸림) 모두 이 브랜치에도 그대로 있다. 이 브랜치 판이 통과한 것을 남은 거리가 실패를 막은 근거로 볼 수는 없다.

왜 남은 거리가 나쁜가(실제로 달린 거리와 비교, `driven_distance.py`, 결과 `gz_v012_2026-11-21/driven_distance.txt`와 `gz_2026-11-21/driven_distance.txt`). 관제 기록의 시각이 시뮬 시각이라, Segment마다 남은 거리를 쓴 첫 표본 시각부터 Segment가 끝날 때까지 위치 기록으로 실제 달린 거리를 쟀다(이 브랜치 판, 투영 2 s 넘는 Segment).

| 묶음 | Segment | 곧은 거리 / 달린 거리 중앙값(p10~p90) | Nav2 거리 / 달린 거리 | 달린 거리 / 걸린 시간 중앙값 |
|---|---|---|---|---|
| 0.12 m/s 6판 | 93 | 1.072 (1.041~1.131) | 1.130 (1.059~1.194) | 0.101 m/s |
| 0.20 m/s 8판 | 136 | 1.076 (1.050~1.132) | 1.131 (1.067~1.234) | 0.148 m/s |

- 달린 거리는 관제 지도 거리(투영)보다 7 % 짧고, Nav2 길은 달린 거리보다 13 % 길다(지도 거리보다 약 5 % 김). 지도 거리보다 덜 달리는 까닭은 다음 Segment가 이어지면 로봇이 끝 지점 0.15 m 앞(`pass_m`)에서 넘어가기 때문으로 보지만, 넘어갈 때의 위치를 따로 재지는 않았다.
- 실제 평균 속도(달린 거리 합 / 걸린 시간 합)는 0.099 m/s, 0.137 m/s로 상한의 0.69~0.83배다.
- 경계 오차: `driven_distance.py`는 시작과 끝 모두 그 시각 뒤의 첫 위치 기록을 쓴다. truth.csv는 0.072 s마다 한 줄이라 양 끝의 거리 오차는 각각 0.009 m(0.20 m/s 판 0.014 m) 이하로, Segment 길이(1 m 넘음)의 1 % 안이다.
- 고른 표본(Segment 시작 무렵, 0.12 m/s 판)에서는 둘 다 짧게 봤다: 투영 중앙값 −0.89 s, 남은 거리 −0.71 s(|오차| 평균 1.49 / 1.46 s, 남은 거리 쪽이 조금 덜 짧음). 위 표의 0-1 s, 1-2 s 구간에서는 둘 다 길게 봤고 남은 거리 쪽이 더 길다(+0.60 / +0.77 s, +0.26 / +0.47 s). 남은 거리가 전체 평균에서 나빠진 것은 Segment 뒷부분에서다.
- 해석(확인하지 않은 부분 포함): 오차의 큰 부분은 거리보다 "끝까지 고정 속도 0.12 m/s로 간다"는 시간 모델에서 온다. 앞부분은 실제가 더 느려 짧게 보고(출발할 때 돌거나 속도를 올리는 시간으로 짐작), 끝은 0.15 m 앞에서 넘어가 실제가 더 빨리 끝나 길게 본다(0.15 m는 1.25 s). 남은 거리는 거리만 바꾸므로 이 오차는 그대로 남고, Nav2 길이 지도 거리보다 길어 끝 무렵의 "길게 봄"을 키운다.
- 곧 남은 거리가 득이 되려면 ETA 모델에 출발 시간, 넘어가는 거리, 실측 속도를 넣어야 한다. 이 경기장처럼 돌아가는 길이 없으면 지도 거리로도 충분하다.

판단: 시뮬 속도를 모델에 맞춘 판에서도 남은 거리를 쓴 ETA가 같은 판의 투영보다 나빴고, 갱신 시점 중앙값이 0.2 s 늦었다. **main에 넣지 않는다는 권고는 그대로다.** 남은 거리는 돌아가는 길이 생기는 장면(장애물, 실물)에서 다시 볼 일이다.

## 리뷰

Codex(gpt-6.1-terra, high)가 c23c185를 봤다. 지적 넷 가운데 셋을 확인하고 시험 먼저 써서 고쳤다.

- 두 번째 리뷰(4686b4a)는 시험이 덜 덮는다는 지적 둘만 냈다. 거듭 온 피드백 시험 하나를 더했다. 2 s 기다림의 경계 시험을 나쁜 값마다 넣으라는 것은 넣지 않았다(동작 결함이 아님).
- 고침: 같은 값의 피드백을 노드가 걸러 남은 거리가 1 s 뒤 "오래됨"이 됨, 뛴 뒤 나쁜 값 하나가 2 s 기다림을 풂, 음수 값이 음수 ETA를 만듦.
- 고치지 않음: 서 있다 나가는 길(`_leave_standby`)처럼 달리던 Route의 앞부분을 이어 만든 후보 Route에서는 표본을 쓰지 않는다(`route is m.route`가 아님). 이때는 예전 투영이라 main과 같은 판단이다. 쓰게 하려면 후보 Route와 달리던 Route의 cut이 같은지 따져야 해 이번에는 넓히지 않았다.

## 이 기록의 한계

- 기준값 넷(1.0 s, 0.10 m, 2.0 s, 0.10 m)은 Gazebo나 실물에서 잰 값이 아니다. 피드백 주기, Nav2 costmap 칸, 속도에서 정한 추정값이다. Gazebo 16판에서는 주행 중간에 뛴 것이 한 번뿐이라 0.10 m / 2.0 s가 맞는지 충분히 확인하지 못했다(위 "Gazebo 결과" 4).
- 앞 goal의 값이 새 seq에 남는 것은 로봇 쪽(`pode_fsm`) 동작이다. 다음 nav_goal Segment가 짧으면(곧은 거리 0.23 m 안쪽) 남은 값(0.05~0.13 m)이 "곧은 거리보다 짧음" 기준을 지나 1~2 s 쓰일 수 있다. 이번 판들에서는 그런 경우가 없었고, 로봇 코드는 고치지 않았다.
- 노드는 이제 order goal마다 피드백을 0.2 s마다 코어로 넘긴다(전에는 seq나 대기가 바뀔 때만). base 끝에서 기다리는 동안도 같다. 로봇 셋이면 초당 15번 입력과 기록 줄이 는다. 위치 보고가 이미 초당 약 30번이라 감당할 크기로 보았지만 재지는 않았다.
- 투영의 치우침(+1.8 s)은 이 단계에서 다루지 않았다(위).

## 통합 판 Gazebo 비교: 끝 보정과 함께 (2026-11-21 오후)

통합 브랜치 `integrate/fixes-eta-2026-11-21`(이 브랜치 + Nav2 goal 끝 보정 `feat/eta-pass-turn-2026-11-21` + 출구 고정 경로 고침 + 시뮬 진열대 A 0.783)을 기준(main a6265b0 + 시뮬 진열대 고침 0a426fe)과 같은 장면에서 번갈아 돌렸다. 시뮬 속도 0.12 m/s, 화면 없음, 도메인 61, 급행 3판과 v7 3판씩. 통합 쪽은 9612003까지의 코드다(뒤의 STOP 고침은 통신 끊김 때만 쓰여 이 판들과 관계없다). 표는 `distance_remaining_2026-11-21/gz_int_v012_2026-11-21/`.

| | 기준 | 통합 |
|---|---|---|
| 판 통과 | 5 / 6 (v7a 실패) | 6 / 6 |
| 위반, 최소 거리 | 0, 0.205~0.227 m | 0, 0.192~0.221 m |
| Route 시간 평균(급행) | 19.65~19.78 s | 19.70~19.91 s |
| 도착 전 갱신(급행·v7 중앙값) | 0.90 s, 0.90 s | 1.70 s, 1.75 s |
| Q2에 선 횟수(급행 3판) | 2, 3, 2 | 1, 1, 2 |
| nav_goal ETA \|오차\| 평균(전체) | 투영 1.17 s | 남은 거리 + 끝 보정 0.74 s(같은 판 투영 1.22 s) |
| 같은 값, 0~1 / 1~2 / 2~4 s 구간 | 0.70~0.82 / 0.88~0.95 / 1.24~1.34 s | 0.26 / 0.42 / 0.61 s |
| 같은 값, p90(전체) | 2.32 s | 2.35 s |

- 기준 v7a 실패는 v7b와 같은 원인이다. pode2가 O에서 P-1로 나가는 출구 고정 경로를 가다 (0.17, 0.75)에서 경로가 끊겼고(`goal ended: CANCELED`, 새 order 첫 조각이 O_OUT까지의 nav_goal), keepout 안이라 계획이 계속 실패해 `GOAL_FAILED NAV_ABORTED after 3 attempts`, `ROBOT_ISOLATED`로 끝났다. 기준에는 출구 고침이 없다. 통합 6판에서는 나지 않았다.
- 남은 시간이 짧은 구간(갱신 판단이 나는 곳)에서 오차가 절반 아래로 줄었고, 갱신이 도착 0.9 s 전에서 1.7 s 전으로 당겨졌다(`lookahead_s` 2.0). 오전 판에서 0.2 s 늦어졌던 것과 반대다.
- 남은 쪽: 남은 시간이 8 s 넘는 구간에서는 아직 짧게 본다(통합 부호 평균 −0.69 s, |오차| 1.05 s). p90이 투영과 비슷하거나 조금 크다. Segment를 막 시작했을 때 Nav2 길이가 아직 곧은 거리라 그렇다고 본다(위 "알고 있는 부작용").
- 판정: 이 비교로는 남은 거리를 끝 보정과 함께 main에 넣어도 판 통과, Route 시간, 최소 거리가 나빠지지 않았다.
