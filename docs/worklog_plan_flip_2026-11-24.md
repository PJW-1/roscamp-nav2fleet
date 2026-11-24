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
- 빌드(모두 `~/gg_build/` 아래, Gazebo 판마다 launch.out의 install 경로로 확인한다)
  - `pd-flip-base`: 468f09c. 기준 판(해결안 없음, RPP).
  - `pd-flip-a`: c0e4985. A(keeper), C(BT 3 s)의 RPP 판.
  - `pd-flip-a2`: bc3a9d3. `nav_rl`이 있는 빌드. RL+shim 판 전부(기준, A, C).
  - `pd-flip-b`: db77f9e. B 판.
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
