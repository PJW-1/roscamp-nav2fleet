# v7을 기본 배치로 (2026-11-19)

브랜치 `feat/v7-default-layout-2026-11-19`, 바탕 origin/main 3d4254c.

## 1. 왜

- v7은 확정된 실제 경기장 배치다(11-18 밤 지도 확정, main 7b3a619부터). 실제 로봇 시험은 모두 v7로 돈다.
- 그런데 배치를 주지 않을 때 고르는 기본값은 아직 옛 후보 `d2`였다. `map_file`을 안 준 관제(`load_site()`)가 d2를 읽는 식이었다.
- 팀 저장소 내보내기(fleet-v1)는 이미 `fleet_core`의 `DEFAULT_LAYOUT = "v7"`이다. 이 저장소도 맞춘다.
- 옛 배치 `d2`, `d`, `c`, `d2_shelf`는 지우지 않았다. 이름을 주면 전과 같이 돈다. 지도 데이터는 건드리지 않았다.

## 2. 바꾼 것

| 곳 | 전 | 후 |
|---|---|---|
| `pc/fleet_core/fleet_core/fleet_map/fleet_map.py` `DEFAULT_LAYOUT` (`load_fleet_map`, `load_site`의 기본) | `d2` | `v7` |
| `sim/fleet_gz_sim/launch/gz.launch.py` 인자 `layout` | `d2` | `v7`(기본 시나리오도 `gz_v7`) |
| `sim/fleet_gz_sim/launch/one_pode.launch.py` 인자 `layout` | `d2` | `v7`. `start` 기본은 `""`로, drive_check에 그 layout만 주고 plan을 생략했을 때 고르는 계획의 시작(v7은 O_IN, 그 밖은 P-1). 4절 참고 |
| `sim/fleet_gz_sim/fleet_gz_sim/fleet_setup.py` `DEFAULT_LAYOUT` (`map` 없는 시나리오의 배치) | `d2` | `v7` |
| `sim/fleet_gz_sim/fleet_gz_sim/drive_check.py` `--layout`, `--plan` | `d2`, `stage2` | 둘 다 비면 `v7`, `corridor` |

`drive_check`: `stage2`, `cross` 계획은 d2 지점(N_W, P-1→P-3 Nav2 이음)으로 짜여 있어 v7에서는 만들어지지 않는다. 그래서 `--layout`만 v7로 바꾸면 인자 없는 실행이 깨진다. `stage2_plan.pick_layout_plan(layout, plan)`을 더했다.

- 둘 다 비면 `v7`과 v7의 첫 계획 `corridor`.
- 계획만 주면 그 계획의 배치(`plan_layout`: `stage2`, `cross`는 d2, `corridor`, `gap`은 v7). README의 `drive_check --plan cross`는 전처럼 d2로 돈다.
- 배치만 주면 그 배치의 첫 계획, 없으면 `stage2`. `--layout d2`는 전처럼 `stage2`다.

d2에 기대던 시나리오와 문서는 `d2`를 적었다(기대값은 바꾸지 않음).

- `sim/fake_robots/scenarios/` 중 `map`이 없던 9개(basic_orders, order_burst, shared_lane_failure, stock_shortage, stuck_robot, blocked_lane, operator_recovery, reinstate, auto_isolation)에 `map: d2`. 모두 D2 지점과 구역(J_E, HALL 등)으로 쓴 시나리오라, 기본이 v7로 바뀌어도 fakes.launch에서 d2로 돈다.
- README의 `one_pode ... ` + `drive_check --layout d2 --plan stage2` 짝에 `layout:=d2`, sim README의 `obstacles:=d2_yard_express` 예에 `layout:=d2`.
- 기본을 적은 문서: README(배치 절), sim/fleet_gz_sim/README.md, docs/관제_구역교통.md, `fake_robots/scenario.py` docstring.

일부러 그대로 둔 것(배치 기본값이 아니라 d2용 도구의 인자다).

- `tools/order_situations.py -l`, `tools/triage/cli.py -l`, `tools/triage/headless.py run_triaged(layout="d2")`, `fake_robots/world/order_situations.py run_order_situation(layout="d2")`: order 상황은 d2로 짰다(`express_horizon`이 d2의 EXPRESS 구역에 상자를 놓음). v7로 돌려 보면 extend_fail과 express_horizon이 I5_STANDING으로 실패한다(씨드 1, order 켬). 상황을 v7용으로 다시 짜는 것은 따로 할 일이다.
- `stage2_plan.plan_layout`의 기본 `d2`: 계획의 배치이고 `stage2`, `cross`는 d2 계획이다.

## 3. 시험

먼저 쓴 시험(바꾸기 전 실패, 바꾼 뒤 통과).

| 시험 | 바꾸기 전 |
|---|---|
| `pc/fleet_core/test/test_site.py::test_load_site_without_a_layout_reads_the_real_arena_v7` (새) | `AssertionError: assert 'd2' == 'v7'` |
| `pc/fleet_core/test/test_layout_v7.py::test_v7_is_a_candidate_and_the_default` (옛 `..._but_not_the_default`, 기본을 단언하는 시험) | `AssertionError: assert 'arena_d2' == 'arena_v7'` |
| `sim/fleet_gz_sim/test/test_fleet_setup.py::test_a_scenario_without_a_map_is_for_v7` (옛 `..._for_d2`) | `AssertionError: assert 'd2' == 'v7'` |
| `sim/fleet_gz_sim/test/test_launch_wiring.py::test_the_launch_takes_a_layout_and_no_site_argument` | `AssertionError: assert 'd2' == 'v7'` |
| `sim/fleet_gz_sim/test/test_one_pode_wiring.py::test_the_default_layout_is_the_real_arena_v7` (새) | `AssertionError: assert 'd2' == 'v7'` |
| `sim/fleet_gz_sim/test/test_stage2_plan.py::test_without_layout_and_plan_the_check_drives_v7_by_its_corridor_plan` (새) | `AttributeError: ... has no attribute 'pick_layout_plan'` |
| `sim/fleet_gz_sim/test/test_drive_check.py::test_without_arguments_the_check_drives_the_real_arena_v7` (새) | `AttributeError: ... has no attribute 'parse_args'` |

d2 기본에 기대던 시험은 `d2`를 적었다(기대값 그대로).

- `pc/fleet_core/test/test_site.py::test_load_site_reads_the_default_layout_or_a_named_one`: `load_site().name == "arena_d2"` → `load_site("d2")`.
- `pc/fleet_core/test/test_fleet_map.py::test_d2_matches_the_instruction`(옛 `test_d2_is_the_default_and_matches_the_instruction`): `load_fleet_map()` → `load_fleet_map("d2")`.
- `sim/fleet_gz_sim/test/test_fleet_setup.py`: `test_a_scenario_with_robot_faults_is_refused`, `test_a_low_battery_runs_in_the_arena_the_stand_in_reports_it`의 시나리오에 `"map": "d2"`(`check_fits(..., "d2")`와 맞춤).
- `sim/fleet_gz_sim/test/test_launch_wiring.py`: `test_three_podes_start_on_the_parking_nodes_in_order`, `test_nav_map_variant_is_empty_by_default_and_refuses_a_layout_without_it`(d2에서 거절), `test_the_real_pode_fsm_answers_dock_by_default`, `test_obstacles_take_a_link_schedule_checked_against_the_layout`, `test_one_rviz_shows_the_whole_fleet_by_default`, `test_ros_process_budget_leaves_room_under_the_localhost_limit`에 `layout="d2"`.
- `sim/fleet_gz_sim/test/test_one_pode_wiring.py::test_nav_map_variant_is_empty_by_default_and_v7_only`: 거절 확인에 `layout="d2"`.
- `sim/fake_robots/test/test_scenario.py::test_a_scenario_can_choose_a_layout`: `basic_orders`가 이제 `map: d2`를 적으므로 `""` 대신 `"d2"`. `map` 없는 시나리오는 빈 값이라는 확인은 임시 파일로 옮겼다.

결과(같은 환경, `ROS_DOMAIN_ID=70`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, /opt/ros/jazzy와 nav2fleet_ws/install을 source하고 PYTHONPATH에 작업 트리의 pc/fleet_core, sim/fake_robots, sim/fleet_gz_sim, pode/pode_fsm, pc/fleet_manager). 비교용으로 origin/main(3d4254c)을 `git archive`로 풀어 같은 명령으로 돌렸다.

| 패키지 (`python3 -m pytest -q -p no:cacheprovider`) | origin/main | 이 브랜치 |
|---|---|---|
| pc/fleet_core | 1110 passed, 7 xfailed in 47.61s | 1111 passed, 7 xfailed in 47.84s |
| sim/fleet_gz_sim | 12 failed, 575 passed in 19.85s | 12 failed, 578 passed in 21.31s |
| tools | 2 failed, 154 passed in 174.53s | 2 failed, 154 passed in 178.00s |
| sim/fake_robots | 133 failed, 932 passed, 1 skipped, 27 xfailed in 442.08s | 133 failed, 932 passed, 1 skipped, 27 xfailed in 550.88s |

sim/fleet_gz_sim을 install 없이(PYTHONPATH에 pc/fleet_core, sim/fake_robots, sim/fleet_gz_sim, pode/pode_fsm만) 돌리면 `449 passed, 18 skipped in 3.97s`다(ROS 메시지가 필요한 시험은 건너뜀). PYTHONPATH에 pode/pode_fsm이 없으면 `test_timing_agreement.py` 수집이 실패해 그 줄만 남는다.

origin/main에서도 같은 시험이 같은 수로 실패한다(실패 목록 diff 없음). 모두 ROS 메시지를 쓰는 시험이고 `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'fleet_interfaces'`(설치된 fleet_interfaces와 이 셸의 환경 문제)이거나 tools의 `test_real_pode.py` 두 개(설치된 pode_fsm launch)다. 이번 변경과 상관없다.

돌리지 않은 것: `pc/fleet_manager` ROS 시험(도메인 50~59를 스스로 잡음), Gazebo(띄우지 않음).

## 4. 후속: one_pode 기본 시작 지점 (Codex b07ff12 지적)

문제: 인자 없는 `drive_check`는 v7 + corridor라 O_IN에서 시작하는데, `one_pode.launch.py`의 기본 `start`는 P-1이었다. v7에서 두 점은 0.642 m 떨어져 있고 `drive_check`의 시작 검사(`START_M` 0.15 m)에 걸려, 기본 launch 둘을 함께 쓰면 출발 전에 멈췄다. b07ff12 전에는 d2 + stage2라 둘 다 P-1이었다.

고친 것: `one_pode`의 `start` 기본을 빈 값으로 두고, 비면 `start_node(pick_layout_plan(layout, None)[1])`을 쓴다(규칙은 `stage2_plan` 한 곳). v7은 O_IN, d2·d·c·d2_shelf는 stage2의 시작 P-1로 전과 같다. `start:=X`를 주면 그대로 쓴다. docstring, 인자 설명, sim README(`start:=O_IN`을 붙이라던 줄)를 고쳤다. v7 기본 시작이 주차 지점 P-1에서 O_IN으로 바뀌므로 문서에 `start:=P-1`(전처럼 주차 지점)과 `start:=M_E`(v7의 다른 계획 gap의 시작)를 적었다. 기본 시작은 `PLANS`에서 배치마다 첫 계획을 따르므로, 순서를 바꾸면 바뀐다는 주석을 `PLANS` 옆에 달았다. `tools/avoid_regression.sh`는 `layout:=d2 start:=P-1`을 직접 주고, gz.launch는 one_pode를 쓰지 않아 v7 시나리오는 P-1 그대로라 바꿀 것이 없다.

시험(`sim/fleet_gz_sim/test/test_one_pode_wiring.py`, 먼저 씀):

| 시험 | 고치기 전 |
|---|---|
| `test_the_default_layout_is_the_real_arena_v7`(`start` 기본은 빈 값) | `AssertionError: assert 'P-1' == ''` |
| `test_without_layout_and_start_the_pode_stands_where_drive_check_starts`(새, O_IN이고 START_M 안) | `assert (0.12, 0.1) == (0.49, 0.625)` |
| `test_d2_without_start_stands_on_p1`(새) | 통과(지금 동작을 지키는 시험) |
| `test_an_explicit_start_wins`(새) | 통과(지금 동작을 지키는 시험) |

결과: `test_one_pode_wiring.py` 18 passed. sim/fleet_gz_sim 전체(위와 같은 환경, `ROS_DOMAIN_ID=70`, install의 fleet_interfaces는 10:10쯤 새로 빌드됨) `593 passed in 21.57s`. 앞서 실패하던 12개(test_dock_stub, test_recovery_probe, test_stand_in_battery)가 이제 통과한다(세 파일만 돌려 `38 passed`).

그대로 둔 것: fake_robots의 머리 없는 상황 도구(`ZoneWorld()`, `situations.build`, `table.run_situation`, 시험 도우미 `world_helpers.quiet_world`)와 fleet_core 시험 도우미 `zone_harness`의 기본 d2는 바꾸지 않는다(코디네이터 결정). 기존 상황과 시험이 이 기본을 통해 d2 구역과 지점(HALL, EXPRESS 구역, J_E 등)을 쓴다. `situations.build`처럼 v7 분기가 있는 상황도 있으므로 모두 d2 전용은 아니고, "공통 기본은 v7"의 예외로 둔 것이다. 공통 기본 v7은 `load_fleet_map()`과 v7을 명시한 시험이 확인한다.
