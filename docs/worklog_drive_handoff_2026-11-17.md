# 작업 기록과 인수인계: 주행 다듬기, O_IN/O_OUT 이동, 코드 정리 (2026-11-17)

다음 세션(학원 컴퓨터의 CLI)이 이 문서 하나로 2026-11-16 밤 ~ 11-17 새벽에 바뀐 것을 파악하도록 쓴 기록이다.
먼저 `git fetch origin && git log --oneline -30 origin/main`으로 아래 커밋이 있는지 확인한다.

사용자 지시(요약): 고정 경로 끝에서 방향을 맞추느라 서는 것을 없애고(D), 관제가 목적지를 일찍 정하게 하고,
O_OUT을 1 cm, O_IN을 오른쪽 위로 2~5 cm 옮기고, 시험한 뒤 main에 합치면서 코드를 한 번 정리하고 push한다.
O_OUT을 나온 뒤 뚝뚝 끊기는 것의 개선은 main이 아닌 별도 브랜치에 둔다(6절).

## 1. main에 들어간 것 (08241c0..)

| 묶음 | 커밋 | 내용 |
|---|---|---|
| D: 끝 방향 안 맞추기 | 9edde5c, 1315dec | 고정 경로 다음 조각이 Nav2 goal이면 그 고정 경로 끝에서 방향을 맞추지 않는다(2절) |
| O_OUT 1 cm | 5880fb9 | `fleet_map_v7.yaml` O_OUT x 0.15 -> 0.16(사용자: "딱 1cm만"), 월드 다시 그림 |
| 관제: 목적지 일찍 정하기 | d25dd78, a983efe | 대기(standby) order의 base를 앞쪽 고정 경로 끝에서 끊고, 자리가 나면 취소 없이 이어 붙인다(3절) |
| O_IN 이동 | fa1b3bb | O_IN (0.44, 0.62) -> (0.49, 0.625), yaw 108도(4절) |
| 정리: pode | eb518cc..c657cc7 (6개) | 5절 |
| 정리: sim, tools | 243c3a3..335f8fe (5개) | 5절 |
| 정리: fleet_core, fleet_manager | a2db88e..347b191 (10개), 3dad50f | 5절 |

작업 브랜치는 `feat/oout-smooth`이고 main은 여기로 fast-forward했다. 묶음마다의 브랜치
(`feat/path-end-heading`, `feat/oout-east-1cm`, `feat/standby-early-switch`, `refactor/pode-cleanup`,
`refactor/sim-tools-cleanup`, `refactor/fleet-core-cleanup`)도 원격에 올렸다. 지운 브랜치는 없다.

## 2. D: 고정 경로 끝에서 방향을 맞추지 않기 (로봇 쪽)

- 전에는 O_X -> O_OUT 고정 경로 끝에서 끝 방향(yaw 허용 0.12 rad)을 맞추느라 제자리에서 돌고, 이어서 Nav2 goal이
  시작되면 또 돌았다.
- 규칙: `OrderFsm._free_end()`가 다음 조각(`pending[0]`, 없으면 `horizon[0]`)의 mode가 `nav_goal`이면 참을 준다.
  그때 `FollowPathRunner.send`가 goal checker로 `free_heading_goal_checker`(xy 0.05, yaw 3.2, stateful)를 고른다.
  후진 조각은 후진용 checker가 먼저다. 파라미터 `free_heading_goal_checker_id`를 비우면 옛 동작이다.
- order가 갱신되어 다음 조각이 바뀌면 규칙을 다시 계산하고, 달라졌으면 같은 경로를 다시 보낸다(1315dec, Codex 지적).
- **실물에 올릴 때:** `pode_fsm`과 `pode_nav_config`를 같은 커밋으로 올려야 한다. `nav2_params.yaml`의
  `goal_checker_plugins`에 새 checker가 없으면 Nav2가 그 goal을 거절한다. 로봇별 덮어쓰기 파일에서
  `goal_checker_plugins`를 통째로 바꾸고 있다면 새 이름을 넣는다(`pode/pode_nav_config/README.md`).

## 3. 관제: 대기 order의 목적지를 일찍 정하기

- 대기 order(줄이나 도크가 비기를 기다리며 내보내는 order)의 base를 맨 앞 고정 경로의 끝에서 끊는다. 나머지는
  horizon에 둔다. 자리가 나면 `_leave_standby`가 base 끝에서 새 길을 이어 붙인다(취소하지 않는다).
- 갱신에 로봇이 아직 답하지 않았으면(`mo.before is not None`) 기다린다(a983efe, Codex 지적: 전에는 취소로 빠졌다).
- 주차 조각은 남은 시간이 `standby_lookahead_s`(0.5초) 이하이거나 로봇이 base 끝에서 기다릴 때 붙인다.
  **0.5초는 재지 않은 추정값이다.** 보통 order의 `lookahead_s` 2.0과 order를 끈 경로는 그대로다.
- 문서: `docs/관제_구역교통.md`, `docs/order_모델.md`. 시험: `pc/fleet_core/test/test_zone_standby.py`.

## 4. O_IN, O_OUT 이동

- O_OUT: x 0.15 -> 0.16. 벽에서 1 cm 멀어졌다. 효과는 확인되지 않았다(아래 측정에서 벽 경고가 판마다 61~115로 흔들린다).
- O_IN: (0.44, 0.62) -> (0.49, 0.625). 북쪽은 OUT_DOCK(`[0, 0.765, 0.50, 0.99]`)까지의 놓기 간격
  (0.135 + 0.005 = 0.14) 때문에 y 0.625가 한계이고, 동쪽 5 cm는 도크 동쪽 끝(0.50) 안쪽이다.
  서는 방향은 `out_in` 첫 구간의 방향(108도)이고, O_X와 `out_in`의 간격은 0.252 -> 0.242 m가 되었다.
- 월드와 지도는 `python3 -m fleet_gz_sim.arena --layout v7 --out .`로 다시 만든 것과 바이트까지 같아야 한다
  (`sim/fleet_gz_sim/test/test_arena_v7.py`). `worlds/arena_v7.sdf`만 바뀌었다.
- **확인하지 못한 것:** Gazebo 판(`gz_v7_rush` 여러 판, `gz_v7_varied` + `v7_yard_mix` 한 판)에서 로봇이 O_IN으로 간
  일이 한 번도 없었다. O_IN 이동은 단위·통합 시험(fleet_core, fake_robots, fleet_manager)으로만 확인했다.

## 5. 코드 정리 (동작을 바꾸지 않음)

읽기 전용 점검 세 건(pode, sim/tools, fleet_core/fleet_manager)을 먼저 하고 "안전하고 이득이 있는 것"만 골랐다.
결론: 파일을 나누거나 옮길 곳은 없었다(가장 큰 파일 397줄, 순환 import 없음, 틱마다 파일을 읽는 곳 없음).

- pode: BackUp 발행 블록 세 곳과 시도 횟수 기록을 `SelfRecovery._send_backup`, `_count_attempt`로 묶음. `_Move`,
  `_Dock`, `_Recovery`, `_Retreat`를 공개 이름(`Move`, `Dock`, `Recovery`, `Retreat`)으로. `node.py`가 `commands`,
  `results`에서 직접 import하고 `core.py`에 `__all__`. `_obstacle`이 `_costmap_tf`를 다시 쓰고 `pose_here` 도우미.
  `pode_nav_config/package.xml`에 `python3-yaml` exec 의존.
- sim, tools: `tools/order_situations.py`가 체크아웃에서 실행되지 않던 것을 고침(`ModuleNotFoundError: fake_robots`,
  시험 `tools/test/test_script_help.py`). 쿼터니언 -> yaw 식 다섯 곳을 `fleet_gz_sim/quaternion.py`의 `yaw_of`로.
  시험의 `read_text`/`write_text`에 `encoding`.
- fleet_core, fleet_manager: `zip(x, x[1:])` -> `pairwise`. 물러나는 길 뒤의 로봇 찾기 두 반복문을
  `YardRetreat._behind_along`으로. `_capacity_checked`를 `ZoneBase.__init__`에서 초기화. `orders/records.py`의
  `_Robot`, `_BoundOrder`, `_LostJob`, `_merge`, `_subtract`와 `geometry._cross`를 공개 이름(`RobotRecord`,
  `BoundOrder`, `LostJob`, `merge_items`, `subtract_items`, `segments_cross`)으로. `_order_result`의 안 쓰는 `success`
  삭제. `validate._on_map`. `fleet_manager_node._dispatch`와 `arm/jobs.on_command`를 `match` 문으로.
  믹스인 이름 충돌 시험을 `OrderOrchestrator`, `FleetManager`에도.
- FILL/SWAP: `traffic.mission`은 `fleet_map.model`의 것을 다시 내보낸다. `common.commands`는 자기 정의를 그대로 둔다
  (3dad50f: 정리 중에 `common`이 `fleet_map`을 import하게 된 것을 Codex가 짚어 되돌렸고,
  `test_common_stands_alone_below_the_other_packages`로 고정했다).
- `python -m ruff check .`가 저장소 전체에서 깨끗하다(전에는 37건).

하지 않은 것(점검에서 "나중에"로 분류): zone 믹스인 24개를 합성 구조로 바꾸기, `manager_route._route`의 `match`
(튜플 `ARM_COMMANDS`로 검사해 패턴으로 옮기면 새 명령이 조용히 빠진다), `to_ros_qos`를 fleet_manager로 옮기기,
`from __future__ import annotations` 일괄 삭제, `Robot`에 `slots`, sim의 선분 거리 계산 통합, 노드 `main()` 통합,
`tools/test/test_real_pode.py`의 encoding, pode `_run`의 `match`, YAML 앵커.

## 6. 측정 (Gazebo `gz_v7_rush`, 창과 RViz 켬, 판마다 한 번, 모두 12/12 출고에 위반 0건)

### 6.1 D, 1 cm, 관제 변경

| 판 | 내용 | 경기 시간 | O_OUT을 떠나기 전 뒤집힘 | 떠나는 데 걸린 평균 | O_OUT 근처 벽 경고 |
|---|---|---|---|---|---|
| base | 바꾸기 전 | 264.1 s | 19 (전환 9번) | 2.63 s | 149 |
| d1 | D만 | 259.1 s | 0 (12번) | 1.63 s | 115 |
| dm1 | D + 1 cm | 263.1 s | 1 (12번) | 1.28 s | 61 |
| all1 | D + 1 cm + 관제 | 260.9 s | 0 (12번) | 1.33 s | 102 |

- D는 떠나기 전의 제자리 돌기를 없앴다. 경기 시간은 한 판씩이라 차이를 말하기 어렵다.
- all1에서 `STANDBY_LEFT` 5번 가운데 3번이 O_OUT 도착 전(1.5, 0.9, 2.2초 전)이었다.

### 6.2 O_OUT을 나온 뒤 끊기는 것 (사용자 질문: "왜 뚝뚝 끊기는거야")

- 재는 법: O_OUT을 떠난 뒤 9초 동안 직진 명령이 0이고 회전만 하는 구간을 "선 것"으로 센다.
- all1: 전환 12번에 63번 섰다(전환당 5.2번, 3.3초). 63번 가운데 55번은 벽 경고 없이 RPP의 "방향이 14도 넘게 틀어지면
  서서 돈다"(`rotate_to_heading_min_angle` 0.25) 때문이었고, 선 동안 돈 각의 중앙값은 8도였다.
- 원인: NavFn이 5 cm 격자에서 내는 경로가 지그재그다. 처음 0.6 m에서 방향 변화의 합이 중앙 151도인데 순 변화는
  35도다. 앞보기 거리(0.10~0.16 m)가 격자 두세 칸이라 이 굽이에 그대로 반응한다.
- 후보 비교(이 문서의 main 상태에 `nav_overrides`만 준 판):

| 판 | 바꾼 것 | 경기 시간 | 전환당 선 횟수 | 전환당 선 시간 | 평균 직진 명령 | collision ahead | 최소 거리 |
|---|---|---|---|---|---|---|---|
| sm0 | 없음(NavFn) | 264.9 s | 5.8 | 3.30 s | 0.100 m/s | 133 | 0.207 m |
| smtheta | 플래너를 Theta*로 | 247.2 s | 2.0 | 1.17 s | 0.142 m/s | 118 | 0.239 m |
| smlook | `min_lookahead_dist` 0.10 -> 0.15 | 250.4 s | 4.0 | 2.30 s | 0.133 m/s | 80 | 0.224 m |
| smrot | `rotate_to_heading_min_angle` 0.25 -> 0.45 | 250.0 s | 2.8 | 1.59 s | 0.126 m/s | 131 | 0.226 m |

- **적용은 main에 없다.** 사용자 지시대로 별도 브랜치 `feat/nav-goal-smooth`에 두었고, 그 브랜치의
  `docs/worklog_nav_goal_smooth_2026-11-17.md`에 조합 판과 선택한 값을 적었다.

## 7. 검증

- 단위·통합: fleet_core 1097 passed 5 xfailed(틱 순서 `manager`, `zone`으로도 1096 passed 1 skipped 5 xfailed),
  fleet_manager 통합 125, pode_fsm 455 passed 6 skipped, pode_nav_config 103, pode_avoidance 26, pode_peer_obstacles 18,
  pode_scan_filter 23, fleet_gz_sim 534, fake_robots 1019 passed 27 xfailed, tools 98. `ruff check .` 깨끗함.
- Gazebo: 6절의 판들. 정리까지 모두 합친 뒤의 판 main1(`gz_v7_rush`, 창과 RViz 켬)은 12/12 출고, 261.2 s, 위반 0건,
  최소 거리 0.226 m, keepout 침범 깊이 0, 실시간 배율 1.00, `STANDBY_LEFT` 6번, `BLOCKED_WAIT`와 `UPDATE_FAILED` 0번이다.
- **간헐 실패 한 건(오늘 것이 아님):** fleet_manager 통합 시험을 다른 시험과 함께 돌린 한 번에서 `stock_shortage`가
  `t=110.9 J_W released from pode1 while 0.08 m from it`로 실패했다. 같은 빌드에서 그 앞 실행은 통과했고, 단독으로
  세 번 다시 돌려 세 번 통과했다. 같은 문구의 실패가 `docs/worklog_fake_pode_order_2026-11-16.md` 5절에 이미 있다
  (order의 물러나기 조각 중에 구역을 일찍 놓는 것, 원인 후보는 관제 쪽). 고치지 않았다.
- 시험을 돌릴 때 주의: `sim/fake_robots`와 `pc/fleet_manager`의 노드 시험은 둘 다 환경 변수 `ROS_DOMAIN_ID`를 쓴다.
  같은 도메인으로 동시에 돌리면 서로의 메시지를 받아 실패한다(오늘 한 번 그렇게 돌려 16건이 실패했다. 도메인을
  나눠 다시 돌려 통과).
- Codex 검수(읽기 전용, 배경 실행): 9edde5c(1건 확정 -> 1315dec), d25dd78(1건 확정 -> a983efe), 재검수 둘은 고칠 것 없음,
  fa1b3bb 고칠 것 없음, 정리 세 묶음에서 확정 1건(`common` -> `fleet_map` import -> 3dad50f). 기록은 `~/codex-review.md`.

## 8. 남은 것과 사용자에게 물을 것

- O_OUT 1 cm의 효과는 확인되지 않았다(사용자도 "맘에 안 들긴 하는데"라고 했다). 되돌리려면 5880fb9를 revert하고 월드를 다시 만든다.
- `standby_lookahead_s` 0.5초는 추정값이다. 실물에서 관제 -> 로봇 갱신이 늦으면 base 끝에서 한 번 선다(`BASE_END_WAIT`).
- O_IN은 Gazebo에서 지나간 적이 없다(4절).
- 실물 시험 순서와 잴 값은 `docs/주행_계획서.md` 9절 그대로다. 실물 설정에서는 고정 경로 충돌 검사를 켜고,
  처음에는 `use_order_model:=false`로 시작한다. 오늘 바뀐 것 가운데 실물에서 먼저 볼 것은 O_X -> O_OUT 끝에서
  서지 않고 바로 Nav2 goal로 넘어가는지다.
- `feat/nav-goal-smooth`를 main에 넣을지는 사용자가 정한다. Theta*는 로봇에 `nav2_theta_star_planner` 패키지가
  있어야 한다(`ros2 pkg prefix nav2_theta_star_planner`로 확인).

## 9. 이 컴퓨터의 작업 폴더 (다른 컴퓨터에는 없다)

- `C:\Users\pjw\ros2camp\wt-nf-drive`: `feat/oout-smooth`(= main), 이어서 `feat/nav-goal-smooth`.
- `wt-nf-standby`, `wt-nf-rf-pode`, `wt-nf-rf-sim`, `wt-nf-rf-core`: 묶음별 브랜치. `wt-nf-main`: main.
- Gazebo 판의 기록은 WSL `~/nf_logs/chk/<판 이름>/`에 있다(launch.log, bag, summary.json, fleet_check.json).
