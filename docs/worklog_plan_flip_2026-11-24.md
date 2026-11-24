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
  - 5 s의 근거: 1단계 기준 판 4판(s1_a1_r1, s1_a1_v1, s1_a_r1, s1_a_v1)에서 닿은 조각 32개의 지연은 7.2 s 한 건 말고
    모두 3.33 s 이하였다(닿은 뒤 방향 맞추기, yaw 허용치 0.12 rad). 그 위로 여유를 두었다.
  - 창 0.5 s: 관제 feedback 주기 0.2 s보다 넉넉히 잡아 앞 조각 끝 보고가 늦은 만큼 앞당긴다.
  - 이전 order를 하나만 기억해도 되는 근거: 11-24 판들에서 새 명령 뒤에 온 실패 결과는 5건, 가장 늦은 것이
    0.068 s였고 결과를 기다리는 goal이 둘 넘게 쌓인 적은 없었다.
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

## A 위주로 바꿈, 실물 스위치, B 2회차 실패 (17:50~18:20)

- 17:50 사용자: "우리는 거의 무조건 A 쪽으로 갈 것". 남은 큐를 A 먼저로 바꿨다(queue9): 1 cm A 러시 2회차 →
  2.5 cm A 장애물·러시 각 2판과 2.5 cm 기준 2회차 → 1 cm 기준 2회차, 1 cm A 3회차 → B, C 러시 2회차 → 1단계 나머지.
- 실물 키트에 A 스위치를 넣었다(50e0d13): `PLAN_KEEP=1 start.sh ...` → `real_pode.launch.py`의 `plan_keep`(기본 false)
  → `nav2.launch.py`. `deploy.sh`가 `pode/pode_plan_keep`도 복사한다(pode_nav_config가 이미 exec_depend로 기대고 있었는데
  복사 목록에 빠져 있었다). 실행하지 않았다(deploy, 로봇 접속 없음). Codex 검수 결함 없음.
- `s2_a1_B_rpp_v2`(B, RPP, 장애물 판 2회차)는 sim 약 474 s에 `ROBOT_ISOLATED NAV_ABORTED`로 끝났다(summary 없음).
  - pode2가 sim 462 s부터 (1.72, 0.36)에 멈춰, 0.22 m 앞 Q2로 가는 nav_goal을 세 번(461.0 s 갱신 뒤, 466.7 s, 473.6 s)
    NAV_ABORTED로 끝냈다. 제자리에서 방향만 돌았다(yaw -1.8 ~ 0.16 rad).
  - 위치 추정 오차는 2.4 cm 이하였다. 그 시간(460~475 s)에는 장애물 상자가 없고(v7_yard_mix: 480 s 주기, 370~430 s 다음은
    540 s), 다른 로봇은 0.8 m 넘게 떨어져 있었다.
  - NavFn이 goal 칸에 닿지 못할 때 tolerance 창을 훑으며 남기는 `worldToMap failed`가 pode2 planner에서만 5120줄 났다.
  - 이 goal(P-1 쪽 follow_path 뒤 nav_goal Q2)은 B가 나누지 않았다. 원인 미확정. 전역 비용 지도에 남은 표시가 Q2와 로봇
    사이를 막았을 수 있다(추측: 이 판의 bag에 costmap을 남기지 않아 확인하지 못했다).

## 2회차와 3회차, 남은 의문 정리 (18:20~20:50)

- queue9로 2.5 cm A 장애물·러시 각 2판, 2.5 cm 기준 2회차, 1 cm 기준 2회차, 1 cm A 3회차를 돌렸다.
- 19:25 pode_fsm 수정 브랜치(`fix/fsm-pass-ready-2026-11-24`, 작업 기록 `docs/worklog_fsm_pass_ready_2026-11-24.md`)의
  Nav2 늦게 켜기 판을 끼워 넣느라 queue9 바깥 반복문을 멈췄다. 돌던 `s2_a1_A_rpp_v3`는 끝까지 돌았지만 끝 줄이 batch.log에
  남지 않아, summary를 보고 손으로 적었다(batch.log 208행).
- 그 무렵 다른 세션이 도메인 93을 쓰기 시작해, 그 뒤 판(queue10~12)은 도메인 97로 옮겼다. 남의 프로세스는 건드리지 않았다.
- 19:30 무렵부터 다른 세션의 시험과 Gazebo로 PC load가 45~58(12코어)이었다. 판 결과는 모두 끝났지만, 이 도메인이 받은
  CPU 몫(판 평균 550~810 %)과 RTF가 판마다 다르다. stand_in CPU 시간도 이 몫을 따라 흔들린다.
- 두 번째 path_not_connected(`s1_a_v2`, 2.5 cm 기준, keeper 없음, pode3 sim 340.6 s): 추정 위치 Q2에서 0.150 m일 때 넘겼고
  선 뒤 추정 위치만 0.152 m로 물러났다(truth는 0.151 m 그대로). keeper와 costmap과 상관없이 두 판정이 같은 0.15를 쓰는
  데서 난다. pode_fsm 브랜치에서 `pass_m`을 0.12로 내렸고, 그 브랜치는 다른 세션이 main에 병합했다(6133ba9).
- 20:40 사용자 판단: keeper는 갈림길에서 경로가 두 길 사이로 바뀌는 문제를 막으려는 것이므로 경로 뒤집힘으로만 평가한다.
  제자리 회전(도리도리)은 컨트롤러 문제다(11-23 비교에서 같은 경로에 MPPI가 RPP보다 약 3배 덜 흔들림). 보고서 결론을
  이에 맞춰 고쳤다.
- A에서 남은 뒤집힘(장애물 판 판당 0~3건): keeper 로그 `changed the route`의 사유를 셌다. 1 cm는 모두
  "the kept route is blocked"(붙잡은 길이 비용 99 이상 칸에 막힘)이고, 2.5 cm는 판마다 막힘 1건과
  "the new plan is clearly shorter" 1건이다. 둘 다 keeper가 설계대로 길을 놓아준 경우다.
- 러시 판 거리 튐이 A에서 많은 까닭(2회차까지 기준 판당 3~6, A 8~12): 같은 goal·조각 안 튐을 둘로 나눴다(`docs/plan_flip_2026-11-24/dist_jump_split.py`).
  - 조각 첫 값 바로 뒤에 커지는 튐: 기준 0~4(6판), A 6~8(5판). 나머지 튐: 기준 3~12, A 2~5.
  - pode_fsm은 Nav2 피드백의 distance_remaining이 0인 동안 목표까지 직선거리를 보고한다(`NavGoals.distance_remaining`).
    Nav2가 첫 피드백부터 0이 아닌 거리를 줄 때까지 걸린 시간(`nav_first_distance.py`, bag의 navigate_to_pose 피드백, 러시 판 기준 5판, A 5판)은 기준 중앙값 0.01~0.02 s,
    A 0.08~0.16 s(최대 0.50 s)다. keeper를 거치면서 첫 계획이 그만큼 늦어지는 것으로 보고, 그래서 직선거리가 한 번 보고된 뒤 경로 길이(약 0.13 m 더 김)로
    바뀌는 일이 A에서 잦다(`s2_a1_A_rpp_r1` pode1 127.9 s의 첫 값 1.586 m = 그때 추정 위치에서 Q2까지 직선거리).
    관제는 남은 거리가 0.10 m 넘게 커진 보고 뒤 2 s 동안 로봇이 보고한 거리를 쓰지 않고 지도 길이로 ETA를 잰다
    (fleet_core `order_answers.py` `LEFT_JUMP_M`, `LEFT_CALM_S`). 그래서 goal 시작 뒤 2 s 동안 ETA가 지도 기준으로 바뀔 뿐 로봇 움직임은
    바꾸지 않는다. 그 ETA로 정하는 구역 미리 허가 시점에 준 영향은 재지 않았다.
  - 장애물 판의 나머지 튐은 2.5 cm에서 A가 적다(기준 17, 19 → A 6, 12). 뒤집힘이 준 만큼 경로 길이가 덜 바뀐 것으로 본다.

## main 병합 준비 (21:00~)

- 사용자 지시로 main에 병합한다. 다만 ros2camp-13이 진열대 서쪽 끝과 진열대 A 북쪽 면을 고치는 브랜치
  (`fix/shelf-west-ends-2026-11-24`)를 먼저 합친다(사용자 결정: 틀린 실물 지도가 main에 들어가지 않게).
- main(6133ba9)을 먼저 합쳤다. 충돌 11개 파일은 코드는 양쪽 기능을 모두 두고 주석 형식은 main 것을 따랐다.
  브랜치에서 더한 코드 주석의 날짜를 뺐고, `splits` 주석을 두 줄로 줄였다(Codex 검수).
- main의 `nav_pass_m` 0.12에 맞춰 B 비교 파일(`fleet_params_plan_split.yaml`)을 기본 파일에서 다시 만들었다
  (`test_zone_plan_split`이 두 파일 차이를 본다). 오늘 B 판은 0.15일 때 돌았다.
