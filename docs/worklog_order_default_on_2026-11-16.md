# 작업 기록: order 모델을 기본으로 켬 (2026-11-16)

브랜치 `feat/order-default-on-2` (origin/main 71bdca0에서). main에는 넣지 않았다(조정자가 합친다).

## 1. 무엇을, 왜

- 사용자 결정(2026-11-16): "order 모델을 디폴트로 켜". 성능 우선.
- `use_order_model`을 두 곳에서 켰다: `pc/fleet_core/fleet_core/data/fleet_params.yaml:79`, `pc/fleet_core/fleet_core/common/params.py:100`.
  launch(`pc.launch.py`, `gz.launch.py`)는 인자를 비우면 파일 값을 쓰므로 고칠 것이 없다.
- 켠 상태로 시험을 돌려 드러난 관제 빈틈을 먼저 고쳤다(옛 브랜치 `origin/feat/order-default-on`의 뜻을 옮김, cherry-pick 아님).
- 고침마다 시험을 먼저 썼고, 고침을 빼면 그 시험의 order_on 칸이 실패하는 것을 확인했다.

## 2. 커밋 (71bdca0..)

| 커밋 | 내용 | 옮긴 옛 커밋 |
|---|---|---|
| ea0ea54, 93f7ecc, 8cfcdd5, bd16267 | 시험 하네스(`zone_harness.py`)가 order 다리(legs)를 읽고, 로봇 자신의 OrderRun으로 답한다(answer_at). 받은 것이 없으면 거절. raw order 순서 시험(feedback, SUPERSEDED, update feedback, 성공) | 788cbf0 |
| 6fb5a81, 26bb4f0 | 조각 끝에 선 order 로봇을 도착으로 친다. 후보 조각 끝마다 자기 링크와 방식으로 판정하고 가장 가까운 끝을 고른다. 건너뛴 조각은 SEGMENT_DONE(물러나기면 RETREAT_DONE)으로 닫는다. `_link_reached`와 `_short_of_end`/OBSTACLE_BEHIND 보호는 그대로. watch.py도 같은 함수를 쓴다 | 2b48d20 |
| f8dbed5, 23be68c | 줄 순위를 아직 앞에 남은 줄 자리로 매긴다(r.node 대체는 유지). 끄고 켠 두 경우로 돈다 | f9f2509 |
| e675ecd, 2ad2bed, ea3c24f | be046e8의 1~3: 목적이 바뀐 order는 넓은 구역의 노드에서 멈춘다(nav_goal은 바로), 기다리던 주차 칸을 남이 잡으면 다시 고른다, 줄 station의 horizon은 옛 경로에 줄 자리가 하나 이상 있고 새 경로와 줄 자리 목록이 다를 때만 다시 짜지 않는다. 4번은 main에 이미 있다(`order_answers.py:55`) | be046e8 |
| 8d5845a | 기본값 켬 + 시험 고정(아래 3절) | 872ec88 |
| 3473013, f7ea5dd | 느슨해졌던 줄 시험을 원래대로 두고 켠 칸은 strict xfail(M_E로 돈 것만 `Detour`로 받는다), 놓기 시험의 켠 짝 | (Codex 리뷰) |
| efb64b8 | `tools/replay.py` RECORD_DEFAULTS에 `use_order_model: False`. 키 없는 옛 기록(2026-11-12 전)은 끈 관제로 재생 | — |
| (이 문서) | 문서: README, order_모델.md, 주행_계획서.md, 팀레포_옮기기.md를 "기본 켬"으로 | 7bdfc49 |

옮기지 않은 것: 872ec88의 씨드 바꾸기(yard back-off)는 그대로 옮기지 않았다(3절). 7bdfc49는 지금 문서에 맞춰 새로 썼다.

## 3. 시험 기대가 바뀐 곳 (약해진 것이 아닌 이유)

- 끄는 길이 목적인 시험만 `use_order_model=False`로 고정: 사건 기록 golden(`test_zone_order_off.py`, `sim/fake_robots/test/test_world_order_off.py`), RetreatCmd 세부, 놓기 거리 시험.
- `test_zone_release.py`: 보고 위치 3회 규칙 시험은 끈 길로 고정. order는 물러나기 조각을 끝내고 N_W(구역에서 0.149 m)를 지날 때 노드 좌표로 재어 0.129 m에서 놓는다(몸은 구역 밖, 발자국 0.085 m). 켠 짝 시험은 놓기 직전 마지막 표본에서 몸이 구역 밖인지를 본다(놓은 그 걸음의 거리는 재지 않는다).
- `test_zone_edges.py` 목적 바꿈 시험: 끄고 켠 두 경우. order는 같은 W로 가는 다리가 하나 더 있다(follow_path).
- RetreatCmd 위치 대조 시험: 끈 길 고정, order 짝은 strict xfail(SEGMENT_DONE을 보고 위치와 대조하지 않음, 열린 위험).
- `test_zone_block_marks.py`: order는 대기 자리 O_X에서 조각을 끊고 update로 도크까지 늘린다. 다리가 `O_X, O`이고 BASE_END_WAIT가 없음을 본다.
- `test_zone_failures.py`: 로봇이 제자리인데 성공 답을 받는 경우를 `answer_at(..., move=False)`로 그대로 만든다(전에는 MoveCmd 답이 로봇을 옮기지 않았다).
- `test_zone_line_queue.py` 출고 로봇이 칸 입구를 지나는 시험: P-3이 낀 두 칸의 order_on은 strict xfail(열린 위험 5).
- fleet_manager `test_node_order.py`: 인자 없을 때 기본값이 true.
- fake_robots:
  - pose_jumps(faults 9칸, avoid_modes 18칸)는 끈 길로 지키고, 켠 짝을 `xfail(strict=True, raises=PoseJumpsOpen)`으로 더했다. `I3_OUTSIDE_HELD` 말고 다른 실패가 있으면 그냥 실패한다.
  - yard back-off(`order_burst` 11, 4): 켜면 불변식은 다 지키지만 물러나기를 한 번도 안 쓴다(그래서 "the back-off was used"에서 실패). 씨드 1~12를 켜고 돌린 결과: 실패 0, HALL_BACKOFF_DONE은 씨드 12만 1번. 11, 4는 끈 길로 두고 켠 길은 씨드 12를 더했다. 872ec88의 씨드 변경도 같은 이유(켜면 물러나기를 안 씀)라 정당했지만 끈 길 보호를 잃어서 그대로 옮기지 않았다.

## 4. 숫자 (나온 그대로)

fleet_core, `ZONE_TICK_ORDER=manager|zone`:

| 상태 | manager | zone |
|---|---|---|
| main 71bdca0, 끔 | 1048 passed, 1 skipped | 같음 |
| main, 켬만 | 148 failed, 900 passed, 1 skipped | 같음 |
| 고침 모두 + 켬, 시험 옮기기 전 | 108 failed, 954 passed, 1 skipped, 1 xfailed | — |
| 최종(f7ea5dd, 문서 커밋과 같은 코드) | 1074 passed, 1 skipped, 3 xfailed | 1074 passed, 1 skipped, 3 xfailed |

- 고침마다 커밋 뒤 끈 길 전체도 통과(마지막 1062 passed, 1 skipped, 1 xfailed, 두 차례 모두).
- test_zone_at_target order_on: 고치기 전 11개 실패 → 고친 뒤 29개 통과.
- replay 시험 3개(키 없음/false/true): 고치기 전 "키 없음"만 실패, 고친 뒤 3개 통과. 기존 replay 시험 출력은 바뀌지 않았다.
- pc/fleet_manager(두 파일 뺌, 도메인 72): 113 passed, 1 warning.
- sim/fleet_gz_sim(도메인 73): 528 passed.
- sim/fake_robots(도메인 71): 조정자가 켬만 했을 때 30 failed, 970 passed(642 s). 최종(8d5845a 트리): 1001 passed, 27 xfailed, 1 warning(866 s). xfail 27 = pose_jumps 켠 짝 9 + 18, XPASS 없음.
  고친 파일만: 31 passed, 9 xfailed(57 s).

## 5. 열린 위험 (다음 사람이 볼 것)

이번 변경에서 드러났거나 켬과 직접 묶인 것만 적는다. 그 전부터 있던 한계(Gazebo의 고정 경로 충돌 검사 기본 끔, base_end 정책에서 늦은 도착, 머리 없는 세계가 액션 왕복 지연을 재현하지 않음 등)는 `docs/worklog_nav2base_2026-11-12.md`(286, 404행 근처)와 `docs/주행_계획서.md` 7절에 있다.

0. **합치기 전에 정할 것: ROS 가짜 Pode는 order goal을 모른다.** `sim/fake_robots/fake_robots/pode_goals.py`는 MoveTo의 order 칸(조각, seq)을 읽지 않는다. `pc.launch.py use_fake:=true`는 `use_order_model`을 비우면 파일 값(이제 켬)을 쓴다. `test_scenario_integration.py`의 basic_orders를 도메인만 74/75로 바꾼 사본(스크래치)으로 돌린 결과: 켬 **실패**(6건 중 4건 출고, 900 s 시간 초과, GOAL_FAILED, ROBOT_ISOLATED, FIFO 위반 1), `use_order_model:=false` **통과**(46 s). 즉 합치면 CI의 시나리오 시험 9개가 깨질 수 있다(나머지 8개는 돌리지 않음). 고르는 길: (a) `use_fake:=true`일 때 비어 있으면 끔을 넘긴다, (b) 가짜 Pode에 order를 가르친다. 정책 결정이라 이 브랜치에서는 고치지 않았다.
1. **pose_jumps(B7)**: 튄 보고 위치로 로봇이 서 있는 구역을 놓는다(I3). 조정자 측정(창 안 I3 실패): 끔 8/54(배치 c만, 출하값 20 s 시작에서는 0/9), 켬 45/54(18 s부터 모든 배치). 스크립트: `/tmp/claude-1000/-home-ane-ros2camp/255eff68-4c77-42b3-b9fc-325ae2037b61/scratchpad/jumpshift.py`. `docs/주행_계획서.md` 8절에는 사용자 결정 "비전과 엮을 때까지 그대로 둔다, 관제만으로 고치지 않는다"가 있다. 조정자가 새로 낸 관제만의 고침(LOCALIZATION_SUSPECT 뒤 구역을 쥠)은 그 결정을 바꿀지 사용자에게 묻는 중이다. 여기서 넣지 않았다.
2. SEGMENT_DONE을 보고 위치와 대조하지 않는다(strict xfail `test_a_retreat_segment_reported_done_is_checked_against_the_reported_pose`).
3. Gazebo 3대 켠 경기 `g4o3_on2`(`sim/fleet_gz_sim/results/order_gz_results.md:52`, 조정자가 말한 :44는 같은 표의 옛 줄 번호)에서 244.8 s에 두 로봇 몸이 닿고 5/8로 못 끝났다. 그 뒤 켠 3대 Gazebo는 이 브랜치에서 돌리지 않았다(Gazebo 금지).
4. 실물: `tools/real_pode/real_pode.launch.py`는 pode_fsm에 comm_loss_policy를 넘기지 않는다(로봇 기본은 stop). PC의 comm_loss_policy를 바꿔도 로봇은 바뀌지 않는다.
5. order가 빈 칸으로 주차하러 가다 O_OUT 조각 끝(위치 판정, 끈 길보다 0.2 s 이르다)에서 대기 자리로 돌릴 때, E-3의 로봇이 아직 입구를 쥐고 있으면 O_IN, M_E로 돈다(도킹 2.6 s 늦음). strict xfail 2칸.
6. `_line_spot`은 쥔 줄 자리가 없는 로봇을 맨 뒤로 매긴다.
7. 줄 horizon 규칙을 느슨하게 한 부분(줄 자리 목록이 같으면 다시 짬)을 직접 보는 시험이 없다.
8. 목적이 바뀐 order는 nav_goal이면 바로, 고정 경로면 넓은 구역 노드에 섰을 때만 멈춘다. 노드를 떠난 뒤 목적이 바뀌면 이미 허가된 base(도크 앞까지일 수 있음)를 끝까지 간다(Codex c11 [확정]). 끈 길의 MoveCmd도 허가된 끝까지 가므로 main보다 나빠지지는 않았지만 막지도 않는다. TO_PARK 다시 고르기도 같은 판단.
9. 노드 좌표로 재는 놓기는 움직이는 로봇에서 보고보다 이르다(위 3절의 0.129 m). 켠 짝 시험은 놓기 전 마지막 표본만 재고, 놓은 그 걸음의 거리는 재지 않는다(코드로 확인한 한계).

## 6. 하다 겪은 것

- `--ignore=pc/fleet_manager/test/test_scenario_integration.py`가 듣지 않았다. ROS의 launch_testing pytest 플러그인(`/opt/ros/jazzy/lib/python3.12/site-packages/launch_testing/pytest/hooks.py:204`)의 `pytest_ignore_collect`가 늘 `False`를 돌려주는데, 이 훅은 firstresult라 pytest 기본 `--ignore` 검사까지 가지 못한다(`-p no:launch_testing`이면 0개). 그래서 이 시험이 도메인 50, 51에서 pc.launch.py를 띄웠고 조정자가 멈췄다. 이후 파일을 직접 골라 `pc/fleet_manager` 안에서 돌렸고 `pgrep -fa pc.launch.py`로 50~59에 뜨는 것이 없음을 확인했다.
- 돌리는 법: fleet_core는 `ZONE_TICK_ORDER=manager|zone PYTHONPATH=pc/fleet_core python3 -m pytest pc/fleet_core/test -q -p no:cacheprovider`. fleet_manager는 `cd pc/fleet_manager && python3 -m pytest -q $(ls test/test_*.py | grep -v -e test_scenario_integration -e test_fms_restart)`.
