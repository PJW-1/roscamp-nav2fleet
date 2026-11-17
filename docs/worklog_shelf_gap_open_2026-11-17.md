# 작업 기록: v7 남쪽 진열대 사이 건넘 틈 다시 열기 (2026-11-17)

- 브랜치 `fix/v7-shelf-gap-open`(origin/main 956140d에서). push하지 않았다.
- 사용자 결정(2026-11-17, 원문): "저틈을 막으면안되지 지름길인데 안되면 고쳐야되는거고".
  건넘 틈은 지름길이라 막지 않는다. 지나가다 실패하면 틈을 막지 말고 주행을 고친다.
- [확정]은 코드, 기록, 문서에서 본 것(파일:줄)이고 [추측]은 확인하지 않은 판단이다.

## 1. 바꾼 것

| 파일 | 무엇 |
|---|---|
| `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml` | `keepout_extra`에서 첫 사각형 `[0.99, 0.365, 1.25, 0.405]`(건넘 틈)을 뺐다. 칸막이 셋 둘레의 사각형은 그대로다. 주석: 틈은 사용자 결정으로 열어 둔다, 4079c40에서 막았던 까닭 |
| `sim/fleet_gz_sim/maps/arena_v7_keepout.pgm` | 저장소의 생성기로 다시 만들었다(`PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena --layout v7 --out <임시>`). 바뀐 칸은 130개(x 0.995~1.245, y 0.365~0.405의 칸 중심, 검정 -> 흰색)뿐이다. 같은 생성으로 나온 월드, 5 cm 지도, 1 cm 지도, keepout yaml은 저장소의 것과 바이트가 같다 |
| `tools/real_pode/maps/arena_v7_keepout.pgm` | 위와 같은 파일(`test_kit_maps_equal_sim_maps`가 두 파일이 같은지 본다) |
| `pc/fleet_core/test/test_keepout_mask.py` | 틈이 열려 있는지 보는 시험을 더하고(`test_v7_the_crossing_gap_stays_open_as_a_shortcut`), 덧대지 않음 시험을 칸막이 사각형으로 옮겼다. 지도 글을 바꿔 넣는 시험 둘은 틈 사각형 글 대신 `keepout_extra: [[`를 기준으로 바꾼다 |
| `sim/fleet_gz_sim/test/test_arena_v7.py` | `test_v7_the_crossing_gap_is_shut_...` 대신: 5 cm costmap에서 서쪽 끝과 동쪽 통로를 다 막아도 틈으로 남북이 이어진다, 커밋된 1 cm 마스크의 틈 안 칸이 하나도 닫히지 않는다 |
| `sim/fake_robots/test/test_world_planner.py` | 머리 없는 세계에서 틈은 바닥이다. P-3에 주차한 로봇이 남쪽 통로를 막으면 E-1 -> Q2는 가운데 통로로 올라갔다가 틈으로 내려온다(틈을 지나는 점은 모두 모서리에서 0.08 이상) |
| 문서 | `worklog_layout_v7_2026-11-13.md`, `주행_계획서.md`에 한 줄씩 |

`pc/fleet_core/test/data/fleet_map_line3.yaml`은 바꾸지 않았다. 이 파일은 v7을 복사해 생성되는 것이
아니라 "자리 셋인 옛 v7"을 그대로 얼려 둔 시험용 지도다(`test_zone_line_queue.py:7`,
`test_zone_yard_retreat.py:150`). 줄 서기와 마당 물러나기 규칙을 보는 시험이 쓰고, keepout 마스크나
Nav2 계획을 보는 시험은 이 파일을 쓰지 않는다. 틈 사각형은 그 시험들의 결과에 들지 않는다. 바꾸면
옛 지도를 고치는 셈이라 그대로 두었다.

## 2. 조사

### 2.1 4079c40의 까닭 (Gazebo 2판, 2026-11-13)

- [확정] 판: v7, 커밋 7b929fd, GUI, ROS_DOMAIN_ID 61, 로봇 3대, 시나리오 `gz_v7_random`.
  기록 `docs/worklog_layout_v7_2026-11-13.md:81`("가운데 틈에서 pinky1이 위치를 잃음 -> 틈을 막음(4079c40)").
  ROS 로그 `~/.ros/log/2026-11-13-18-00-01-891340-*/launch.log`는 남아 있으나 노드 기록이 warn 수준이라
  이 일은 없다. 판 폴더(`.../6f858ace-.../scratchpad/v7ws/gz_v7_run2/`)는 지워졌다. 아래 수치는 그 세션
  기록(`~/.claude/projects/-home-ane-ros2camp/6f858ace-77ac-4537-9adb-52656f20da9d.jsonl`, 2026-11-13
  09:05~09:10 UTC)에 남은 `recovery_probe.csv` 출력과 Codex 판단 리뷰를 옮긴 것이다.
- [확정] 로봇: pinky1. 컨트롤러: 마당의 Nav2 goal이라 `FollowPathGoal`(RPP). 당시 값
  (`git show 7b929fd:pinky/pinky_nav_config/params/avoidance.yaml`): `lookahead_time` 1.5,
  `approach_velocity_scaling_dist` 0.6, `rotate_to_heading_min_angle` 0.25.
- [확정] 속도: 시뮬 기본값 `DEFAULT_SPEED_MPS = 0.12`(7b929fd의 `sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py:129`).
  [추측] 2판도 이 값으로 돌았다. 3판의 실행 명령에 속도 인자가 없었고(`scenario:=gz_v7_random
  scenario_seed:=4 battery_low_pct:=20`), 2판 명령은 남아 있지 않다. 지금 기본값은 0.20이다(`launch_parts.py:130`).
- [확정] 순서(recovery_probe):
  - 66.792 s: 위치 오차 0.014 m, 방향 오차 -0.318°(정상), 실제 (0.648, 0.571).
  - 72.912 s: RECOVERING 시작, 실제 (1.055, 0.421), 방향 오차 +36.5°, 위치 오차 0.023 m. 서쪽 진열대
    모서리 (0.99, 0.405)에서 중심까지 약 0.067 m(몸 원 0.083보다 작다). 74.852 s에 BLOCKED.
  - 133.42 s: 보고 위치가 실제와 0.497 m, 방향 178.8° 어긋남. 136.2 s에는 0.622 m.
  - 201.56 s: pinky1이 쥐지 않은 J_E에 몸이 0.083 m 들어감. 202.75 s: pinky1과 pinky2 중심 사이 0.076 m.
- [확정] 같은 판의 18.584 s에 pinky1은 P-3을 나오다 칸막이 3 위 끝 근처(실제 (0.752, 0.150))에서 이미
  방향 오차 31.4°를 보였다. 66.8 s에는 다시 0.3°였다. 틈 앞의 위치 추정이 그 전에 늘 좋았던 것은 아니다.
- [추측] 원인: "틈에서 진열대 모서리에 몸이 쓸려 바퀴가 미끄러지고 odom이 틀어져 AMCL이 잘못 맞췄다"는
  당시 설명은 입증되지 않았다. Codex 판단 리뷰(같은 세션 09:10 UTC)도 [추측]으로 남겼다: 72.9 s 직전에
  RPP 충돌 예측과 계획 실패가 있었고, 62.7~64.7 s 복구 중 실제 방향이 약 112° 바뀌었으며, 다른 로봇이
  밀었는지는 기록으로 가릴 수 없다. 76.95 s와 133.42 s 사이 언제 크게 틀어졌는지도 표본이 없어 모른다.
- [확정] 그때 틈을 지난 길: 7b929fd의 지도에는 틈을 곧게 건너는 Nav2 이음 N_O-Q1, N_E-E-2, N_E-E-3이
  있었다(4079c40 메시지가 이 셋을 지웠다고 적었다). 지금 지도에는 N_O, N_E가 없다.

### 2.2 틈이 닫혀 있어야 하는 곳이 있나

- [확정] 구역: 틈(x 0.99~1.25, y 0.365~0.405)은 HALL이다. HALL 사각형 `[0.0, 0.0, 1.98, 0.395]`와
  `[0.0, 0.395, 1.78, 0.63]`이 덮는다(`fleet_map_v7.yaml` zones.HALL). 틈 안에 선 로봇도 HALL에 있어
  "어느 구역에도 없는" 상태가 되지 않는다. 새 시험이 틈 안 9점의 `zone_at`이 HALL임을 본다.
  `unheld_zone_pose`, `released_while_off_lane`은 머리 없는 판의 판정이고(`sim/fake_robots/fake_robots/scenario_cfg.py:30,42`)
  `zone_at`으로 구역을 본다(`fleet_map.py:89`). 구역 쪽에서 고칠 것은 없다.
- [확정] keepout_extra는 구역이 아니다: 마스크에만 들어간다(`keepout.py:19`, `keepout.py:77`). 관제의 쥐기,
  정원, fleet_check의 keepout 깊이(비공유 구역만, `fleet_check.py:175`)와 관계없다.
- [확정] 길 그래프: 길 값은 이음의 곧은 선 길이다(`graph.py:42`). 마스크를 보지 않으므로 틈을 열어도
  관제가 고르는 길은 그대로다. 지금 지도에 틈을 건너는 이음은 없다(4079c40에서 지운 셋의 끝점 N_O, N_E가 없다).
- [확정] 생성물: 틈 사각형이 들어가는 것은 keepout 마스크(`arena_v7_keepout.pgm`, 실물 묶음의 같은 파일)와
  머리 없는 세계의 바닥(`sim/fake_robots/fake_robots/world/free_space.py:31`, 코드에서 읽으므로 생성물이 아니다)뿐이다.
  월드(SDF), 5 cm 지도, 1 cm 지도는 진열대만 그리고 바뀌지 않았다(다시 만들어 바이트 비교). 다른 배치(d2, d, c,
  d2_shelf)에는 keepout_extra가 없다(`test_layouts_without_keepout_extra_keep_their_masks`). 생성 마스크와 커밋된
  마스크가 같은지는 `test_it_matches_the_mask_the_simulation_ships`가 본다.
- [추측] 지금 흐름에서 틈을 쓰는 Nav2 goal은 드물다. 머리 없는 planner(NavFn 대신, 반지름 0.08)로 보면 O_IN -> Q2,
  M_E -> E-3, E-3 -> M_E, Q2 -> O_IN은 틈으로 가지만, 관제는 이 쌍을 한 goal로 보내지 않는다(M_E, O_IN, Q2는
  through가 아니라 합치지 않는다, `trips.py:51`). 합쳐 보내는 O_OUT -> E-3 -> Q2는 틈으로 가지 않는다.
  그래도 Nav2는 끝점만 받아 costmap과 장애물로 길을 정하고, 지점 밖에 선 로봇은 선 자리에서 지점까지 임시
  이음을 만든다(`goals.py:62~72`). 그래서 막힘 뒤 다시 계획(예: P-3에 주차한 로봇이 남쪽 통로를 막을 때 E-1 -> Q2)과
  지점 밖 출발에서도 틈으로 갈 수 있다. "드물다"는 판단이고, Gazebo에서 재지 않았다(Codex fddb0a1 검수 반영).
  지름길로 늘 쓰게 하려면 틈을 건너는 이음이 필요하다(아래 3절 마지막 항목, 사용자 결정).

### 2.3 0.26 m 틈을 0.112 m 폭 로봇이 튼튼하게 지나려면 (후보만, 이 브랜치에서 Nav2 값은 바꾸지 않았다)

기하: 틈 가운데로 곧게 지나면 몸 옆(0.056)과 모서리 사이가 양쪽 0.074다. 제자리 회전 반경(뒤 모서리
0.0845, `nav2_params.yaml` local footprint 주석)을 쓰면 가운데에서 돌 때 0.045가 남고, 가운데에서 0.045
벗어나면 돌 자리가 없다.

1. [확정] 전역 계획이 지나는 열: 5 cm global costmap에서 내접 0.09(`footprint_padding` 0.03, 정사각형 0.12)로
   닫고 남는 열은 칸 중심 x 1.075, 1.125, 1.175 셋이다(새 시험이 본다). 1.075와 1.175는 모서리에서 0.085로,
   가로지른 상태에서 돌면 회전 원(0.0845)이 모서리에 닿는다. 후보: 틈 양쪽 가장자리 띠(예: x 0.99~1.06,
   1.19~1.25)만 막아 가운데 열 하나(모서리에서 0.135)만 남기는 keepout_extra. 틈을 닫지 않고 좁힌다.
   [추측] 대신 비스듬히 들어가는 경로가 꺾일 여지가 줄어 막힘이 늘 수 있다.
2. [확정] 대각선 통과: 머리 없는 planner의 P-3 우회는 (1.05, 0.49) -> (1.25, 0.23)으로 틈을 비스듬히 지난다.
   NavFn도 꺾는 점을 모서리 가까이 둔다. RPP의 `use_rotate_to_heading`(`rotate_to_heading_min_angle` 0.25 rad,
   `avoidance.yaml` follow_path_goal)은 경로 방향이 14° 넘게 틀어지면 제자리에서 돈다. 2판의 72.9 s는 틈 북쪽
   어귀에서 남쪽을 보고 회복 중이었다. 후보: 틈 위아래에 틈 가운데 줄(x 1.12)로 곧게 들어가는 점을 두어
   틈 안에서 돌지 않게 한다(이음 또는 고정 경로).
3. [추측] 경로 매끄럽게 하기: NavFn 경로의 계단과 꺾임을 Smac 2D나 smoother로 줄이면 RPP가 모서리를 덜
   자른다. 메모리의 "zigzag fix"(corridor A/B, BLOCKED 고침 뒤에 하기로 함, 사용자 2026-11-17)와 같은 일이다.
4. [확정] 앞보기: 2판 당시 `lookahead_time` 1.5는 2026-11-15에 0.8로 줄었다(`avoidance.yaml` 주석: 0.20 m/s에서
   진열대 동쪽 끝을 스쳐 방향이 AMCL과 26도 어긋남). 지금 값(0.10~0.25 m)은 0.20 m/s에서 0.16 m 앞을 본다.
   [추측] 2판과 지금은 이 값과 속도(0.12 -> 0.20)가 다르므로 2판 결과가 지금도 그대로 나는지는 모른다.
5. [추측] 틈 안 속도 제한: Nav2 SpeedFilter(costmap filter, keepout과 같은 마스크 방식)로 틈 둘레만 느리게 한다.
   많이 쓰는 방식이고 지도 한 장이 는다. 성능 우선(사용자 11-16)과 맞바꾸는 값이라 사용자 결정이다.
6. [확정] 충돌 검사: local costmap 2.5 cm, 다각형 발자국, inflation 0.15/3.0, RPP `use_collision_detection` 켬,
   `max_allowed_time_to_collision_up_to_carrot` 0.6. [추측] 0.074의 옆 여유는 2.5 cm 칸 셋이라 칸 경계에 걸리는
   진열대 칸이 하나 더 칠해지면 0.05로 줄어 "collision ahead"가 잦을 수 있다.
7. [확정] AMCL: 1 cm 지도(`map_topic: map_fine`), `sigma_hit` 0.05, `update_min_d` 0.15, `update_min_a` 0.1,
   alpha1~5 0.2, `recovery_alpha_*` 0. [추측] 모서리에 쓸린 뒤 다시 잡지 못한 것이 2판의 큰 어긋남으로 이어졌다면,
   틈을 막는 것보다 닿은 뒤의 다시 위치 잡기(B, 11-16에 보류)가 근본 대책이다. 실물 위치 추정을 잰 뒤 본다는
   보류 조건(메모리 pose-jump B)이 있다.
8. [추측] 마주 오는 두 대: 틈에는 두 대가 비켜 갈 폭이 없다(0.224 + 여유 > 0.26). HALL은 잠그지 않고 Nav2가
   피한다(peer_obstacles). 틈 안에서 마주치면 한쪽이 물러나야 한다. 한 방향 규칙이나 정원 1 구역은 "급행만
   잠근다"는 사용자 결정과 부딪히므로 사용자 결정이다.
9. 지름길로 늘 쓰게 하기(사용자 결정): 지금 이음으로는 2.2절처럼 평소에는 틈을 쓰지 않는다. 틈 북쪽과 남쪽에
   틈 가운데 줄로 곧게 잇는 지점(또는 through 지점)을 두면 관제 길 값에도 지름길이 들어간다.

## 3. 시험

시험 먼저: 시험만 고치고 지도는 그대로인 상태에서 돌렸다(`ROS_DOMAIN_ID=81`).

```
$ PYTHONPATH=pc/fleet_core:sim/fleet_gz_sim:sim/fake_robots python3 -m pytest -q -p no:cacheprovider \
    pc/fleet_core/test/test_keepout_mask.py sim/fleet_gz_sim/test/test_arena_v7.py sim/fake_robots/test/test_world_planner.py
FAILED pc/fleet_core/test/test_keepout_mask.py::test_keepout_extra_closes_its_rectangle_without_padding_and_is_no_zone
FAILED pc/fleet_core/test/test_keepout_mask.py::test_v7_the_crossing_gap_stays_open_as_a_shortcut
FAILED sim/fleet_gz_sim/test/test_arena_v7.py::test_v7_the_crossing_gap_is_open_for_nav2_on_the_5cm_costmap
FAILED sim/fleet_gz_sim/test/test_arena_v7.py::test_v7_the_crossing_gap_has_no_keepout_cell_in_the_fine_mask
FAILED sim/fake_robots/test/test_world_planner.py::test_v7_keepout_extra_is_not_floor
FAILED sim/fake_robots/test/test_world_planner.py::test_v7_keepout_extra_closes_every_cell_it_touches_like_the_mask
FAILED sim/fake_robots/test/test_world_planner.py::test_v7_a_robot_parked_at_p3_sends_others_through_the_crossing_gap
7 failed, 120 passed in 4.33s
```

(실패 까닭: keepout_extra에 틈 사각형이 있음, 1 cm 마스크에서 검사한 틈 안 칸 104개(26열 x 4줄)가
모두 닫힘(첫 칸 (0.995, 0.37)), 머리 없는 세계에서 (1.12, 0.385)가 바닥이 아님, P-3 우회가 동쪽 통로(x 1.65)로 돎.)

지도와 마스크를 고친 뒤:

```
$ PYTHONPATH=pc/fleet_core:sim/fleet_gz_sim:sim/fake_robots python3 -m pytest -q -p no:cacheprovider \
    pc/fleet_core/test/test_keepout_mask.py sim/fleet_gz_sim/test/test_arena_v7.py \
    sim/fake_robots/test/test_world_planner.py tools/test/test_real_pode.py
170 passed in 25.88s
```

전체(차례로, 한 번에 한 프로세스):

```
$ for t in manager zone; do ZONE_TICK_ORDER=$t PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test; done
1097 passed, 1 skipped, 5 xfailed in 101.37s (0:01:41)    # manager
1097 passed, 1 skipped, 5 xfailed in 92.31s (0:01:32)     # zone
```

지도, 마스크, 경기장을 보는 sim 시험(fleet_gz_sim 9개 파일, fake_robots 2개, tools/test/test_real_pode.py).
처음에는 작업 공간을 source하지 않아 `test_launch_wiring.py::test_the_test_layout_has_its_default_scenario_and_it_names_that_layout`
하나가 `PackageNotFoundError: package 'fleet_gz_sim' not found`로 실패했다(1 failed, 446 passed, 61 skipped).
지도와 관계없는 환경 문제다. /opt/ros/jazzy, pinky_deps_ws, nav2fleet_ws의 install을 source하고(PYTHONPATH는 이
worktree가 먼저) 다시 돌렸다:

```
$ PYTHONPATH=pc/fleet_core:sim/fleet_gz_sim:sim/fake_robots python3 -m pytest -q -rs -p no:cacheprovider \
    sim/fleet_gz_sim/test/{test_arena_v7,test_arena,test_fleet_view,test_fleet_check,test_fleet_setup,test_stage2_plan,test_launch_wiring,test_sim_nav_params,test_obstacles}.py \
    sim/fake_robots/test/test_world_planner.py sim/fake_robots/test/test_world_nav_goal.py tools/test/test_real_pode.py
508 passed in 55.87s
```

`pc/fleet_manager/test`는 돌리지 않았다(지시). Gazebo와 launch 파일은 띄우지 않았다.

## 4. Gazebo로만 정할 수 있는 것

- 틈을 실제로 지나는 판: 지금 흐름에서는 틈을 쓰는 일이 드물다고 본다(2.2절 [추측]). 일부러 지나게 하려면
  P-3에 로봇을 세워 두고 E-1 -> Q2를 보내거나, 장애물 일정으로 남쪽 통로를 막는다.
- 0.20 m/s와 지금 RPP 값(앞보기 0.8초)에서 2판처럼 모서리 0.067까지 붙는지, 방향 오차가 커지는지.
  `recovery_probe`의 방향 오차와 fleet_check의 진열대 거리로 본다.
- NavFn이 틈 가장자리 열(x 1.075, 1.175)을 쓰는지, 가운데 열(1.125)을 쓰는지.
- 틈 안에서 두 대가 마주쳤을 때 풀리는지.
