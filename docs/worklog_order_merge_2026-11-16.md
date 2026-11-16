# order 모델 기본 켬 main 병합 기록 (2026-11-16)

사용자 결정(2026-11-16): "결함 고친 뒤 켬으로 병합", 그다음 "a부터 고치고 테스트해서 결과랑 같이 push 해놔. 메인에 키고 끄는 것도 가능하잖아". 이 기록은 그 병합의 근거다. 결과는 모두 Gazebo가 아니라 가짜 로봇(ROS 가짜 Pode, 머리 없는 세계)에서 나왔다. 실물 로봇은 돌리지 않았다.

## 1. 결론

고친 후보를 order 켬으로 main에 합쳤다. 근거:

- ROS 시나리오: 후보 54판(9개 x 6번) 모두 통과, 이른 놓기 0번. 같은 시간에 나란히 돈 main 45판은 44판 통과, 이른 놓기 1번(물러나기를 끝낸 뒤 늦은 보고와 같은 모양). 이 표본에서 후보가 main보다 나쁘지 않았다는 관찰이지, 안전성을 증명한 것은 아니다.
- 관제 시험 1081(두 틱 순서), fake_robots 1019, fleet_manager 113, FMS 재시작 1 모두 통과.
- 머리 없는 세계: 끈 길은 측정한 300칸에서 main과 실패 결과까지 같았다. 켠 길에서 늘어나는 안전 실패는 이미 알려진 B7(pose_jumps의 I3) 24칸이고, 그 밖에 express_obstacle d의 DEADLOCK_SEEN 2칸이 새로 보였다(main 코드에 켜도 같음).

- 끄는 법: `fleet_params.yaml`의 `use_order_model: false`, 또는 launch 인자 `use_order_model:=false`(`pc.launch.py`, `gz.launch.py`).
- 끈 관제: 머리 없는 세계 300칸에서 main과 칸마다 같은 결과였다(4절). 모든 입력에서 같다는 증명은 아니다.

## 2. 합친 것

브랜치 `merge/order-default-on` = origin/main 71bdca0 위에 아래 넷.

| 브랜치 | 내용 | 기록 |
|---|---|---|
| `fix/fake-pode-order` (f9bee09) | order 모델 수정들, 기본 켬(8d5845a), ROS 가짜 Pode가 order goal을 돈다 | `worklog_order_default_on_2026-11-16.md`, `worklog_fake_pode_order_2026-11-16.md` |
| `fix/order-retreat-release` (911172b) | 결함 1: order 물러나기 중 구역을 일찍 놓음 | `worklog_order_retreat_release_2026-11-16.md` |
| `fix/fake-pode-aborting` (3cbc968) | 결함 2: 가짜 Pode의 Nav2 중단 표시가 다음 조각으로 넘어감 | `worklog_fake_pode_order_2026-11-16.md` 끝 절 |
| `test/backoff-seed-order-on` (167eeab) | 결함 1 수정으로 시점이 바뀌어 world 시험 씨드 교체 | 시험 docstring |

### 결함 1: order 물러나기 중 이른 구역 놓기

- 증상: stock_shortage에서 order pode1-38(seq 1 물러나기 → N_W, seq 2 nav_goal → P-2)이 끝났다는 답(SEGMENT_DONE)이 오기 2초 전에 J_W를 놓았다. 참 거리 0.076 m, 판정 반경 0.08 m 안. 수정 전 차례로 돈 두 판에서 같은 자리에서 났다.
- 원인: `progress.py` `_track`이 goal의 마지막이 아닌 RETREAT 링크를 투영 거리만으로 넘겼고, 그 뒤 `_release_behind`가 보고가 노드 0.05 m 안이라며 노드 좌표로 쟀다.
- 수정(d00449b): 보고가 RETREAT 링크 끝 노드에서 `release_margin_m`(0.05 m) 안이면 링크를 넘기지 않고 끝냈다는 답을 기다린다. 더 멀면 예전처럼 넘긴다. 시험 먼저 썼고 수정 전 2 failed를 확인했다.
- 고치지 않은 것: 물러나기를 끝낸 뒤(끈 길은 따로 보낸 RetreatCmd, 켠 길은 seq 끝 답) 늦은 보고가 노드 0.05 m 안에 머물러 노드 좌표로 재는 경우. 켬과 끔 모두 strict xfail(`test_a_late_report_at_the_node_after_backing_out_does_not_free_the_dock[order_off|order_on]`)로 남았다. 정책(판정 반경 0.08 m를 지킬지, 도크를 몇 초 더 쥘지) 결정이 필요하다. 측정에서 이른 놓기는 후보(켬) 54판에서 관찰되지 않았고(늦은 보고 자체가 있었는지는 세지 않았다), main(끔) 45판에서 같은 모양의 이른 놓기가 한 번 났다(3절).

### 결함 2: 가짜 Pode `goal.aborting`이 조각을 넘어감

- 수정(72e2736): `pode_order.py` `_start_segment`에서 `goal.aborting = False`.
- 길이 0인 조각에서만 닿는다(복구 중에는 처음 중단한 자리를 못 넘어 조각이 안 끝난다). 실제 노드 시험(3cbc968)이 그 경우를 본다.

## 3. 시험 결과 (나온 그대로)

| 무엇 | 대상 | 결과 |
|---|---|---|
| fleet_core, `ZONE_TICK_ORDER=manager` | 911172b(이후 병합은 sim만 바꿈) | 1081 passed, 1 skipped, 5 xfailed |
| fleet_core, `ZONE_TICK_ORDER=zone` | 같음 | 1081 passed, 1 skipped, 5 xfailed |
| fake_robots 전체(`sim/fake_robots/test`, ROS 도메인 77) | 463f1ad | 1019 passed, 27 xfailed in 806.82s |
| fleet_manager(시나리오, FMS 재시작 제외) | 463f1ad | 113 passed in 59.45s |
| FMS 재시작(`FMS_RESTART_DOMAIN=78`) | 463f1ad | 1 passed in 120.27s |

### ROS 시나리오 (test_scenario_integration.py, 시나리오 9개 + 판정 시험 2개)

같은 순간에 후보(도메인 80~88)와 main 71bdca0(도메인 90~98)을 한 줄씩, 각 줄 안에서는 CI처럼 차례로 돌렸다. 두 줄이 같은 시간대의 부하를 함께 받는다(프로세스마다 실제 부하가 같았다는 증거는 아니다).

| 회차 | 후보(order 켬, 463f1ad 빌드) | main 71bdca0(order 끔) |
|---|---|---|
| 0 (수정 뒤 첫 판, 911172b 빌드, 혼자) | 11 passed | — |
| 1 | 11 passed (527 s) | 11 passed (529 s) |
| 2 | 11 passed (523 s) | 11 passed (525 s) |
| 3 | 11 passed (526 s) | 11 passed (523 s) |
| 4 | 11 passed (520 s) | **1 failed**, 10 passed (527 s) |
| 5 | 11 passed (540 s) | 11 passed (525 s) |

판(시나리오 하나)마다 `summary.json`(판정), `safety.json`(가까이 감, 최소 간격), `releases.csv`(놓은 순간 몸과 구역의 참 거리)를 셌다. 판정 반경은 0.08 m다.

| | 후보(5회 45판, 0회 9판 따로) | main(5회 45판) |
|---|---|---|
| 판정 통과 | 45/45 (0회 9/9) | 44/45 |
| 이른 놓기(참 거리 < 0.08 m) | 0번 (0회 0번) | 1번 |
| 가까이 감(near) | 0번 | 0번 |
| 로봇 사이 최소 간격 | 0.170 m | 0.170 m |
| 놓은 순간 가장 가까운 참 거리 | 0.092 m (0회 0.100 m) | 0.078 m |

- main의 한 번: 4회 blocked_lane, t=69.4에 pode1의 WORK_DOCK을 0.078 m에서 놓음. 따로 보낸 RetreatCmd가 끝난(RETREAT_DONE t=69.10) 0.3초 뒤다. 물러나기를 끝낸 뒤 늦은 보고와 같은 모양이다(시각과 거리로 본 것이라 같은 원인이라고 확정하지는 않았다). 이번에 고치지 않은 strict xfail이 이 모양을 본다.
- 시나리오 판정에는 이 밖에 계약 검사(contract)와 출고 수도 들어간다. 후보는 모든 판에서 주문을 다 출고했다.

수정 전 기록(같은 방법): 후보 f9bee09 차례로 10/11(stock_shortage 두 번 모두 이른 놓기), main 11/11. 아홉 개를 한꺼번에 돌린 판(부하 평균 약 44)은 main 8/9 실패, f9bee09 5/9 실패였다. 부하 탓이 크다고 보지만 증명하지는 않았다.

## 4. 머리 없는 세계 이상 횟수

상황 10개 x 배치 5개(d2, d, c, v7, d2_shelf) x 씨드 1~6 = 설정마다 300칸, 기본 설정(avoid_mode perfect)에서 order만 바꿨다. 스크립트는 아래 7절.

| 설정 | 실패 칸 | 안전 불변식 칸 | 못 끝냄 | 그 밖 |
|---|---|---|---|---|
| main, order 끔(main 기본) | 24 | 12 (I5_STANDING 30건) | 12 | — |
| 후보, order 끔 | 24 | 12 (I5_STANDING 30건) | 12 | — |
| main 코드에 order 켬 | 47 | 36 (I3_OUTSIDE_HELD 36건, I5_STANDING 30건) | 9 | DEADLOCK_SEEN 2 |
| 후보, order 켬(새 기본) | 44 | 36 (I3_OUTSIDE_HELD 36건, I5_STANDING 30건) | 6 | DEADLOCK_SEEN 2 |

- 끈 두 줄은 300칸 모두 실패 문구까지 같다.
- 네 설정 모두 있는 것(이번 변경과 무관): v7의 front_robot_stuck이 못 끝냄(0/6, 모든 씨드), express_obstacle·outbound_slow의 I5_STANDING. CI 상황 시험은 d2, d, c만 돌아 v7 칸은 안 본다.
- 켜면 생기는 것:
  - pose_jumps의 I3_OUTSIDE_HELD 24칸(v7 빼고 네 배치 모든 씨드). 이미 열린 위험 B7이다(튄 보고 위치로 서 있는 구역을 놓음, `docs/주행_계획서.md` 8절). 시험에서는 `PoseJumpsOpen` xfail로 표시돼 있다. B(+relocalize)는 사용자가 보관했다.
  - express_obstacle d 씨드 5, 6의 DEADLOCK_SEEN 1건씩. main 코드에 켜도 같은 칸에서 난다. 주문은 다 끝냈다. CI 시험은 씨드 1~3만 돌아 걸리지 않는다. 이번에 처음 센 것이고 원인은 보지 않았다.
- pose_jumps v7: 끈 길은 두 코드 모두 0/6(모든 씨드 못 끝냄, main 기본 그대로). main 코드에 켜면 씨드 2, 5, 6이 1/6, 후보는 여섯 씨드 모두 실패 없음. 그래서 켠 후보의 못 끝냄이 끈 main보다 6칸 적다.

## 5. 리뷰 지적과 판단

- Codex(911172b 범위) P1: 물러나기 끝 노드 0.05 m 안에서 서고 끝 답이 안 오면 구역을 계속 쥘 수 있다(교착 가능성, [추측]).
  - 내 확인: 로봇은 base의 seq를 다 끝낸 뒤에만 `waiting_at_base_end`를 켜고, 같은 피드백에 `last_done_seq`를 실는다(`pode/pode_fsm/pode_fsm/order_fsm.py` `_advance`, `order_feedback`; 가짜는 `sim/fake_robots/fake_robots/pode_goals.py:156`). 관제는 그 seq로 SEGMENT_DONE을 낸다(`order_answers.py` `_order_progress`). 그래서 "base 끝에서 기다리는데 물러나기는 안 끝남"은 규약대로 도는 로봇에서는 생기지 않는다. 물러나기 중에 서 버리면 대기 표시가 없으므로, BLOCKED·SELF_RECOVERING 같은 다른 감시 제외 조건이 없는 한 `GOAL_STALLED` 감시(stall_s)가 돈다(`watch.py` `_watch_stall`). BLOCKED와 긴 자기 복구는 따로 처리한다(`blocked.py` `_watch_blocked`). 통신이나 피드백을 잃은 경우의 보류는 이 논리로 배제되지 않는다. 고치기 전에는 같은 자리에서 구역을 일찍 놓았다(안전 위반).
- Codex(병합 0f3e5c3): 가짜 Pode가 앞 조각에 이어 붙는 FOLLOW_PATH에서도 복구 표시를 지운다. 실물은 이어진 조각을 한 run으로 돌린다.
  - 내 확인: 사실이다. 추가된 두 시험도 이어진 조각을 쓴다. 고장 하나를 "조각 하나에 한 번"으로 볼지 "run 하나 동안 계속"으로 볼지의 흉내 모델 선택이고, 길이 0 조각에서만 닿는다. 코드는 그대로 두고 열린 점으로 남긴다.
- Codex(3cbc968): 길이 0 시험은 틱 타이밍에 따라 수정 전에도 통과할 수 있다. 열린 점.
- Codex(167eeab, 463f1ad): 켠 길의 I8 검사는 유지된다. 문제 없음.

## 6. 열린 점

1. 물러나기를 끝낸 뒤 늦은 보고로 이른 놓기(켬·끔 모두 strict xfail). 정책 결정 필요.
2. FOLLOW_PATH에도 같은 모양: v7의 O → O_OUT이 order goal의 마지막 링크가 아니면 보고 거리 0.12 m에서 OUT_DOCK을 놓는다(참 거리 0.08 m 아래는 못 봄).
3. 위 5절의 가짜 Pode 이어진 조각 문제와 길이 0 시험의 타이밍 의존.
4. express_obstacle d 씨드 5, 6 DEADLOCK_SEEN(켠 길, main 코드도 같음).
5. pose_jumps I3(B7). 실물 위치 추정 측정(`docs/주행_계획서.md` 9절 3단계) 뒤 B를 다시 본다.
6. world 시험 `test_the_yard_back_off_...`의 켠 길 칸은 order_burst d2에서 outbound_slow d 씨드 4로 옮겼다. 600칸 가운데 켠 길에서 물러나기를 쓰는 곳이 7칸뿐이라, 시점이 또 바뀌면 다시 옮겨야 할 수 있다.

## 7. 다시 돌리는 법

- fleet_core: `ZONE_TICK_ORDER=manager PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test`, 그리고 같은 명령을 `ZONE_TICK_ORDER=zone`으로 한 번 더.
- 시나리오: `colcon build --packages-select fleet_interfaces fleet_core fleet_manager fake_robots iponic_bringup --build-base <b> --install-base <i>` 뒤 `. <i>/setup.bash; SCENARIO_FIRST_DOMAIN=<첫 도메인> ROS_DOMAIN_ID=<부모> ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -v -p no:cacheprovider --basetemp=<폴더> pc/fleet_manager/test/test_scenario_integration.py`. 도메인 50~59는 다른 사람이 쓰니 피한다. 판마다 `<폴더>/*/summary.json`, `safety.json`, `releases.csv`(놓은 순간 참 거리 `true_m`)를 센다.
- 머리 없는 세계: `run_situation(name, layout, seed, overrides={"use_order_model": on})`를 `fake_robots.world.situations.NAMES` x `fleet_core.fleet_map.KNOWN_LAYOUTS` x 씨드로 돌리고 `outcome.failures`의 첫 낱말(I번호 = 안전 불변식, `finished` = 못 끝냄)로 센다.
