# 작업 기록: HALL Nav2 goal 경로 뒤집힘(도리도리) 해결안 비교 (2026-11-24)

결과와 결론은 `docs/report_2026-11-24_plan_flip.md`에 있다. 이 문서는 무엇을 왜 했고 어디서 막혔는지를
시간 순서로 적는다. 실물(로봇, deploy.sh)은 쓰지 않았고 Gazebo와 시험만 썼다.

## 브랜치와 빌드

- 브랜치 `feat/plan-flip-2026-11-24`: pode-drive main `aff00d7`에서 시작했다.
  1. `468f09c` origin/feat/kit-maps-tape-2026-11-24 merge(1 cm costmap `arena_v7_a1`, `nav_map_variant`)
  2. `90f848a` archive/goal-plan-keeper merge: 해결안 A의 keeper 노드 `pode_plan_keep`. 기본은 꺼짐이다
     (`plan_keep:=false`면 main과 같은 BT, 노드 없음). 실물 키트(tools/real_pode, deploy.sh)는 main 그대로 두었다.
  3. `50f8c24`, `c0e4985`, `f4552c2` 묶음 실행 스크립트 `docs/plan_flip_2026-11-24/run_batch.sh`
  4. `4a10ac1` 지표 스크립트 `flip_metrics.py`, 그림 `plot_flips.py`와 시험(서브에이전트가 만들고 cherry-pick)
  5. `bc3a9d3`, `9583e54` gz.launch.py `nav_rl` 인자(RL+shim 비교용)
  6. `db77f9e` feat/plan-flip-b-2026-11-24 merge: 해결안 B(관제가 Q2 goal을 중간 지점으로 나눔, 기본 꺼짐)
  7. `80a620b` 지표 스크립트 고침(주문 갱신 구간)
  8. `2330bad`~`1ccb258` 도착 지연 `arrival_check.py`, 표 `make_tables.py`, 지표 창 `--until`과 고침
  9. `dae33dc` feat/plan-flip-b-2026-11-24 다시 merge: B가 노드 출발(O_OUT→E-3→Q2)도 나눔(B 에이전트 9fa232b, bb22a46)
- 빌드(모두 `~/gg_build/` 아래, Gazebo 판마다 launch.out의 install 경로로 확인한다)
  - `pd-flip-base`: 468f09c. 기준 판(해결안 없음, RPP).
  - `pd-flip-a`: c0e4985. A(keeper), C(BT 3 s)의 RPP 판.
  - `pd-flip-a2`: bc3a9d3. `nav_rl`이 있는 빌드. RL+shim 판 전부(기준, A, C).
  - `pd-flip-b`: db77f9e로 빌드한 뒤 16:15에 fleet_core만 dae33dc(B 넓힘)로 다시 빌드했다
    (db77f9e..dae33dc는 pc/fleet_core와 docs만 바뀜). B 판은 모두 넓힌 B다.
- Gazebo: ROS_DOMAIN_ID 93, LOCALHOST, Fast DDS, GUI 없음. PC CPU가 판마다 93~97 %라 한 번에 한 판만 돌렸다.
  판 기록은 `~/gz_runs/plan_flip_2026-11-24/<판>/`(launch.out, run_info.txt, result/, truth.csv, bag/, cpu*.csv)에,
  묶음 기록은 같은 폴더의 batch.log에 있다.

## 해결안 셋을 켜는 방법 (Gazebo)

- A: `plan_keep:=true`. BT가 `..._goal_keep.xml`(ComputePathToPose의 server_name=compute_path_keep,
  server_timeout=1000)로 바뀌고, keeper는 sim_stand_in 안에서 로봇마다 돈다(DDS 참여자 추가 없음).
- B: `fleet_params_file:=docs/plan_flip_2026-11-24/fleet_params_plan_split.yaml`(plan_split: true) + B 빌드.
- C: `nav_bt_xml:=navigate_to_pose_w_replanning_and_recovery_goal_replan_3s.xml`(RateController 1 Hz → 0.333 Hz).
- RL+shim: `nav_rl:=rl_shim` + 선택기 `select_controller.py 93 FollowPathRL`(run_batch의 `controller:=FollowPathRL`).

## 시간 순서

- 14:20 무렵 worktree와 브랜치를 만들고 kit-maps, keeper를 합쳤다. keeper는 실물 키트에 넣지 않았다
  (실물에 가져갈 때 필요한 일은 보고서 결론 4).
- 첫 판에서 fleet_manager가 result/fleet_record.jsonl을 못 만들어 죽었다: run_batch가 result/를 먼저 만들게 고쳤다.
- Codex 검수(90f848a)로 상대 경로, 끝나지 않은 판 재사용, bag이 20 s 늦게 시작하던 것을 고쳤다.
  그래서 s1_a1_v1, s1_a_v1 두 판은 bag이 launch 뒤 약 19 s에 시작했다(첫 goal들의 /plan 일부가 없다).
- 1단계 첫 결과: 장애물 판에서 1 cm 7건, 2.5 cm 20건 뒤집힘. 둘 다 남아 있어 2단계로 갔다.
- RL+shim 기준 판이 main의 nav_overrides 막음(plugin 키 거절)에 걸려 죽었다. 막음은 풀지 않고
  `nav_rl` 인자를 새로 만들어 저장소의 RL 설정 파일 둘만 합치게 했다(Codex 지적 둘을 고침).
- 묶음 실행 중인 bash는 반복문을 메모리에 들고 있어 파일을 고쳐도 반영되지 않는다. 큐를 바꿀 때는
  돌고 있는 판의 subshell은 두고 바깥 반복문만 TERM으로 멈춘 뒤, 도메인 93이 빌 때까지 기다리는 새 큐를 띄웠다.
- 15:25 큐 하나가 "domain 93 processes left"로 멈췄다: 내가 도메인 93으로 `ros2 param get`을 돌려서였다.
  판이 도는 동안에는 도메인 93으로 아무 ROS 명령도 쓰지 않는다.
- 지표 스크립트의 Codex 검수에서 확정 결함을 확인했다: 주문 갱신(update_id>0)은 새 goal_id로 새 조각만 보내지만
  로봇은 옛 base의 남은 조각(nav_goal 포함)을 마저 한다(기록에서 갱신 뒤 1.7 s 동안 last_done_seq=1).
  남은 조각을 이어 붙이게 고쳤다(80a620b). 고친 뒤 HALL 회전 값은 0.1 s 안에서만 바뀌었다.
- B 확인 판(s2_a1_B_rpp_r1, 러시)에서 Q2 goal 12건이 하나도 나뉘지 않았다. 원인 추적:
  - B 규칙은 노드에서 벗어난 출발 자리에서만 돈다. 러시 판의 Q2 goal은 O_OUT이나 E-1~E-3 노드에서 출발한다.
  - 장애물 판의 뒤집힘은 충전 뒤 E-1→Q2 goal에서 났고, 그때 장애물 1번 상자(남쪽 통로 (0.946, 0.216),
    60~150 s, 540~630 s)가 있었다. 상자가 만든 동점이라 정적 지도 규칙으로는 보이지 않는다.
  - B 에이전트에게 노드 출발까지 넓히고, E-x→Q2에 관제가 아는 정보만으로 도울 규칙이 있는지 판단을 맡겼다.
    그동안 큐에서는 B 판을 빼고 A, C, 기준 RL을 먼저 돌렸다.

## 지표 정의 (flip_metrics.py)

- flips: 같은 goal(0.05 m 안)의 이웃한 경로 둘을 `pode_plan_keep.route.first_look`으로 견줘 OTHER인 것.
  로봇이 실제로 받은 경로를 센다: A 판은 keeper가 낸 /podeN/plan_kept, 나머지는 /podeN/plan.
  flips_planner는 늘 /podeN/plan(planner가 낸 그대로) 기준이다.
  OFF(로봇이 경로 밖)는 off_route로 따로 센다.
- hall_spins / hall_spin_s: truth 기준 HALL 안, nav_goal 수행 구간 안에서 선속도 < 0.02 m/s이고 각속도가 큰 0.5 s 넘는 구간.
  뒤집힘과 상관없는 정상 방향 돌기도 들어간다.
- recoveries: fleet_record의 on_robot_state MOVING→RECOVERING 수. blocked_wait, path_not_connected는 관제 기록.
- dist_jumps_by_seq: 관제 distance_remaining이 같은 goal, 같은 조각 안에서 0.10 m 넘게 바뀐 수(주 지표).
- shim_spins: cmd_vel이 선속도 ≈ 0, |각속도| 큼인 구간 수(속도 기준 추정치이고 RotationShim 실행 수와 같지 않다).
- arrival_check.py: nav_goal 조각마다 truth로 목적지 0.06 m 안에 든 때부터 관제가 끝을 받은 때까지(지연),
  5 s 넘는 수(slow), 닿은 뒤 실패·취소로 끝난 수(near_fail). C의 11-18 도착 판정 문제 확인용이다.
- collision_ahead: launch.out의 RPP "collision ahead" 줄 수. RL 판은 0이 나와 FollowPathRL이 쓰였다는 방증으로 쓴다.

## 판 실패와 오류 조사 (15:40~15:55)

- `s2_a1_base_rl_v1` 1회차(RL+shim, 1 cm, 장애물 판; 폴더 `s2_a1_base_rl_v1.unfinished-154338`)가 sim 약 540 s에
  `ROBOT_ISOLATED DOCK_FAILED`로 멈췄다. 감시가 판을 끊어 summary가 없고, 큐가 같은 이름으로 다시 돌렸다.
  - sim 430.5~432.4 s에 pode1이 P-3에 서 있던 pode3에 닿았다(중심 거리 0.115 m, truth). 몸은 (0.70, 0.23)에서
    yaw -0.74~-0.78 rad로 거의 멈춰 있었는데, cmd_vel은 각속도 +1.0(포화)을 냈고 Nav2가 아는 yaw는 -18도에서
    +136도로 돌았다. 바퀴 오도메트리만 돈 것이다. 이후 AMCL이 바로잡지 못해 Nav2 위치가 0.4 m → 1.4 m,
    방향 150~180도까지 어긋났다(bag의 navigate_to_pose feedback과 truth 비교, recovery_probe.csv에도 같은 값).
  - 관제는 어긋난 위치를 믿고 Q2, Q1, W 조각 끝을 받았고, 도크 대역은 실제 위치가 W에서 1.73 m 떨어져
    `dock WORK FAILED`를 세 번 낸 뒤 격리했다.
  - RPP 판(s1_*, A, B, C)은 Nav2 위치 오차 최대 0.049 m, 방향 8도 이하였다.
- `worldToMap failed: mx,my: 228..231, 1..80, size_x,size_y: 228,128`(pode1 planner_server)은
  `libnav2_navfn_planner.so`의 로그다. NavFn의 tolerance 0.5 m 창을 Q2(1.66, 0.15) 둘레로 훑으면 x 2.16 m까지 가는데
  지도는 x 2.13 m(1 cm, origin -0.15, 228칸)에서 끝난다. 2.5 cm 지도도 x 2.15 m(92칸)에서 끝나서 같은 일이 날 수 있으므로
  1 cm만의 문제는 아니다. 판별 오류 수: C_rpp_r1 640, base_rl_r1 320, RL 장애물 판 2240~11888, 나머지 0.
  NavFn은 goal 칸 자체에 닿지 못할 때만 이 창을 훑는다고 본다(추측: Q2 근처에 다른 로봇이 있을 때). 오류가 난 판도
  다른 지표는 정상이라 동작 영향은 확인하지 못했다(로그 양만 늘어난다).

## B 넓힘, RL 정리, RPP만으로 줄임 (16:00~17:25)

- B 에이전트 보고(9fa232b, bb22a46, Codex 재검수 결함 없음)를 확인하고 합쳤다(dae33dc). fleet_core 시험 1330 passed,
  7 xfailed를 내가 다시 돌려 확인했다. 기록 재계산(B 에이전트, `two_ways.py`): 러시 판 Q2 goal 12건 중 9건(O_OUT 출발)이
  GAP_S로 나뉘고, 장애물 판 E-1→Q2 goal은 하나도 나뉘지 않는다. s1_a_v1, s1_a1_v1에 ZONE_MARKED_BLOCKED가 0건인 것은
  내가 launch.out에서 다시 셌다(관제가 상자를 알 신호가 없다).
- 좁은 B로 돈 첫 B 판(Q2 goal을 하나도 안 나눔)은 `s2_a1_B0_rpp_r1`로 이름을 바꿔 표에서 뺐다.
- RL+shim 기준 장애물 판(`s2_a1_base_rl_v1`)은 세 번 돌았다. 1회차(`s2_a1_base_rl_v3`로 이름 바꿈)와
  2회차(`s2_a1_base_rl_v4`)는 sim 425~453 s에 로봇끼리 닿은 뒤 위치를 잃고 격리됐고, 3회차(`s2_a1_base_rl_v1`)는
  12/12, 586.1 s로 통과했다. A, B, C의 RL 장애물 판도 각 1판씩 통과했다. 격리가 늘 나는 일은 아니다.
- 17:23 사용자 지시: "RL 쪽은 진행 상황 그대로 두고 최근 시험까지만 올리고, RPP 쪽만 우선". 돌던
  `s2_a1_A_rl_r1`은 내가 끊었고(`s2_a1_A_rl_r1.stopped-172300`, 판 실패가 아니라 표에서 뺐다) 이후 큐(queue8)는 RPP만이다:
  1 cm 2단계 2회차(A, B, C × 두 판), 1단계 나머지(1 cm, 2.5 cm 각 v2, v3, r2, r3).
- `s2_a1_A_rpp_v1`의 path_not_connected 1건(sim 447.7 s, pode1). 처음에는 갱신과 겹친 경합으로 봤으나
  관제 기록(fleet_record의 on_robot_state 추정 위치, MoveOrderCmd)으로 다시 보니 갱신은 원인이 아니었다:
  - 447.39 s Q2 nav_goal을 먼저 넘김. `_passes`는 Q2까지 ≤ pass_m 0.15를 보고, 다음 조각이 follow_path면 그 경로까지
    ≤ connect_tolerance_m 0.15도 본다(다음이 nav_goal이면 경로 거리는 보지 않는다). 이번 다음 조각은 Q2에서 시작하는
    follow_path(seg 3)였고, 추정 위치는 Q2에서 0.144 m였다.
  - Nav2 goal이 아직 달리고 있어 seg 3은 넘겨받기(`handover_since`)로 들어갔다. 447.51 s 갱신(update 2)은 seg 4를 그 경로 끝에
    덧붙였을 뿐이다(`_extend`). 447.64 s 멈춘 뒤 `_hand_over`가 다시 잰 추정 위치는 Q2에서 0.163 m라 PATH_NOT_CONNECTED.
  - 그 사이 truth는 움직이지 않았고(두 때 모두 Q2에서 0.158 m) 추정 위치만 2.2 cm 뒤로 움직였다(AMCL 보정으로 본다). 같은 0.15 m를 두 번 재는 사이의
    위치 추정 흔들림이 원인이다. keeper와 costmap은 판정식에 없지만 접근 경로를 바꾸므로 빈도에는 간접 영향이 있을 수 있다.
  - `~/gz_runs` 아래 남은 모든 판 기록에서 이 실패는 이 1건뿐이다.
- `s2_a1_C_rpp_v1`의 도착 지연 8.14 s(pode3, E-3): truth로 E-3 0.038 m 앞에서 5 s 서서 방향만 돌다가
  RECOVERING → BLOCKED를 거친 뒤 끝을 받았다. 11-18 도착 판정 문제와 같은 꼴이다. 3 s 묵은 경로 때문인지
  AMCL 오차 때문인지는 이 자료로 가를 수 없다.
