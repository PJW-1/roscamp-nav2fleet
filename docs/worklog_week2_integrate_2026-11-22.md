# 2주차 통합 브랜치와 주행 속도 0.12 통일 (2026-11-22)

사용자 계획의 2단계(통합 브랜치 `integrate/week2`)와 3단계(모든 속도 상한을 0.12 m/s 한 곳에서)를 한 기록이다.
이제부터 저장소는 `PJW-1/roscamp-pode-drive` 하나이고, `roscamp-nav2fleet`의 main도 여기로 합쳤다.
`main`에는 합치지 않았고 push도 하지 않았다. `main`에 넣을지는 세 컨트롤러 비교(5단계) 뒤에 사용자가 정한다.

## 1. 합친 것

브랜치 `integrate/week2`는 `integrate/week2-2026-11-21`(b6c2fda)에서 시작했다. b6c2fda에는 이미 다음이 들어 있다.

- `--no-ff` 병합 셋: `feat/store-arena-2026-11-21`(6510857), `feat/mppi-controller-2026-11-21`(bd3ef24),
  `feat/follow-path-rl-2026-11-21`(4d0f3d5)
- 통합 수정 셋: d072514(nav_bench에서 RPP·MPPI·RL을 나란히 돌림), 8a41559(시뮬 속도 상한 하나), ef6fbd6(리뷰 수정)
- RL 담당의 초안 기록 둘: ec4e787, b6c2fda

그 위에 올린 것:

| 커밋 | 내용 |
|---|---|
| e2258e1 | `feat/store-arena-2026-11-21` 병합. 그때의 머리는 9eea912 |
| 20dcbe6 | `nav2fleet/main` 병합(b9e86c5). 공통 조상은 a6265b0이고 그 뒤로 33커밋이다. distance_remaining을 ETA에 넣기, nav_goal 끝 모델, 마당 지점 옆 출구 경로 자르기, 시뮬 진열대 바퀴 여유(진열대 A 면 0.793 → 0.783)가 들어 있다 |
| db852f7 | `feat/store-arena-2026-11-21` 다시 병합(9eb8f3c). Isaac 담당이 끝낸 머리다. 6510857 뒤로 8커밋: timeout 분류기, 로봇 물러나기·링크 잠금 선택지(기본은 둘 다 꺼짐), 양보 안 하는 이동체를 섞는 선택지, 후속 기록 |
| 1130a1a | 사용자 결정 1(아래 3절): 배포할 정책의 onnx만 커밋 |
| 0eb7d6b | 병합 뒤 시험 하나 고침(아래 "충돌") |
| 00d15bb | 사용자 결정 3, 4: MPPI는 시뮬에서만, 속도 0.12 통일(2절, 3절) |
| d4eb043 | `feat/store-arena-2026-11-21` 다시 병합(5b55b75): 대본 로봇 기본(경로 12개, 물러나기 켬), 노드 예약·통행 우선·마주 오기 평가 묶음, alone timeout 집계 |
| 2f5783b | `feat/follow-path-rl-2026-11-21` 병합(fa138aa): RotationShim 변형 덮어쓰기 파일, 후진 막기 기본 끔 |
| e0183c5 | 병합 뒤 고침: shim 변형에도 시뮬 속도 상한, 그 controller 목록에 FollowPathMPPI |
| ba2f156 | `feat/follow-path-rl-2026-11-21` 다시 병합(790c4b0): RL 기록의 결정 21~27과 진단(문서만, 충돌 없음). 후진 막기 기본 끔(`rear_stop_range` 0.0)은 이미 2f5783b로 들어와 있다 |

`feat/follow-path-rl-2026-11-21`은 처음 병합할 때 4d0f3d5 그대로여서 새로 합칠 것이 없었다. 그 뒤에 RL 담당이 올린
8621c0e, fa138aa를 2f5783b로, 마지막 머리 790c4b0을 ba2f156으로 합쳤다. 두 쪽 값이 부딪히는 곳은 RL 브랜치 값을 따랐다. `integrate/week2-2026-11-21`은 b6c2fda 뒤로 새 커밋이 없었다.
Isaac 담당은 `feat/store-arena-2026-11-21`에 계속 올리고 있어, 5b55b75 뒤의 커밋은 조정자가 합친다.

### 충돌

- 글자 충돌: 셋 다 없었다. nav2fleet가 a6265b0 뒤로 바꾼 파일 59개와 이쪽이 바꾼 파일 260개 가운데 겹치는 파일은
  하나도 없었다(`git diff --name-only`로 셈).
- 의미 충돌 하나: nav2fleet가 `fleet_map_v7.yaml`의 `sim_shelves` 첫 줄을 0.793에서 0.783으로 바꿨다. Isaac 쪽
  `learning/store_arena/test_arena_spec.py`의 `test_boxes_are_read_from_the_yaml_at_run_time`은 지도 글에서 그 줄을
  찾아 바꿔 "Isaac은 sim_shelves를 읽지 않는다"를 보이는데, 찾는 글이 0.793이라 실패했다. 찾는 글만 0.783으로
  고쳤다(0eb7d6b). 어느 담당의 동작도 바뀌지 않는다. Isaac 경기장은 원래부터 실측 `shelves`만 읽으므로
  nav2fleet의 진열대 변경은 Isaac 학습에 닿지 않는다.
- d4eb043(store-arena 5b55b75): `arena_spec.py`에서 Isaac 쪽 새 기본(`robot_retreat` 1)과 이쪽의 시간 늘리기
  (`robot_wait_s`, `robot_backoff_s` = `ts.travel_time(...)`)가 같은 줄에서 부딪혔다. 둘 다 살렸다. 시험 파일 둘
  (`test_task_spec.py`, `test_arena_spec.py`)은 양쪽이 끝에 더한 시험을 모두 남겼다.
- 2f5783b(follow-path-rl fa138aa): `follow_path_rl.hpp`에서 그쪽의 후진 막기 기본 0과 이쪽의 `max_v_`,
  `speed_mismatch_`를 모두 남겼다. README 표는 두 설명을 합쳤다.
- 의미 충돌: 새 `follow_path_rl_shim.yaml`은 FollowPathRL을 RotationShim의 `primary_controller`로 감싸서, `plugin`만
  보던 시뮬 속도 상한(8a41559)을 비켜 갔다. 그리고 그 controller 목록에 FollowPathMPPI가 없어 그쪽 시험(두 파일의
  목록이 같아야 함)이 깨졌다. 상한이 `primary_controller`도 보게 하고 목록에 MPPI를 넣었다(e0183c5). 세 controller를
  같은 상한으로 견주려던 8a41559의 뜻을 지키는 고침이다. 바뀌는 것은 하나: `nav_max_speed`를 출처보다 낮게(예 0.10)
  주면 shim 변형의 RL도 그 값으로 잘린다(전에는 잘리지 않았다). 기본(출처 0.12)에서는 그대로다.

## 2. 속도의 단일 출처

출처는 `pode/pode_nav_config/params/nav_speed.yaml`의 `nav_speed_mps: 0.12` 한 줄이다.

이 자리를 고른 까닭: Isaac(Windows, ROS 없음), Gazebo launch(WSL), 실물 로봇(Pi4, `deploy.sh`), FollowPathRL
플러그인(C++)이 모두 닿을 수 있는 곳이어야 했다. `pode_nav_config`는 로봇에 배포되는 패키지이고(`deploy.sh`의 목록),
`params` 폴더가 share에 설치되며, 저장소 경로로도 읽힌다. fleet_core는 로봇에 가지 않으므로 출처로 쓸 수 없었다.

### 따르는 곳(지도)

"읽음"은 실행할 때 출처를 읽어 따라가는 곳, "사본+시험"은 값이 파일에 적혀 있고 다르면 시험이 실패하는 곳이다.
사본은 모두 `pode/pode_nav_config/test/test_nav_config.py`의 `test_every_copy_of_the_top_speed_is_the_one_source`
하나가 본다.

| 곳 | 값 | 방식 | 보는 시험 |
|---|---|---|---|
| Isaac `learning/goal_reach/task_spec.py` `ActionLimits.v_max` | 0.12 | 읽음(`NAV_SPEED_MPS`, import 때) | `test_task_spec.py::test_the_top_speed_is_the_one_source` |
| Isaac `learning/store_arena/robot_spec.py` `ROBOT.speed`(대본 로봇) | 0.12 × (0.67~1.33) | 읽음(`ts.NAV_SPEED_MPS`) | `test_arena_spec.py::test_episode_time_and_travel_waits_follow_the_one_top_speed` |
| Isaac `learning/store_arena/play.py` 성공 시간 비율의 "최고 속도" | `ACTION_LIMITS.v_max` | 읽음(전에는 0.25 고정) | 없음(Isaac 실행 스크립트) |
| Gazebo `fleet_gz_sim/launch_parts.py` `speed_of("")`, `pode(nav_max_speed=None)` | 0.12 | 읽음(`nav_speed_mps()`, share의 파일) | `test_sim_nav_params.py::test_the_launch_speed_default_is_the_one_source` |
| Gazebo launch 인자 `nav_max_speed`(gz, one_pode) | 기본 빈 값 = 출처 | 읽음 | `test_launch_wiring.py` |
| Gazebo가 받는 범위 | 0.05 ~ 출처 | 위 끝은 읽음 | `test_sim_nav_params.py` |
| 실물 `tools/real_pode/make_params.py` `MAX_SPEED`(`--speed` 상한) | 0.12 | 읽음(저장소 또는 로봇 `~/nav2fleet_ws/src`) | `tools/test/test_real_pode.py` |
| FollowPathRL 플러그인 속도 상한과 검사 | 0.12 | 읽음(share의 파일, 아래 2.1) | gtest `NavSpeed.PolicyCheckedAgainstTheOneSource` |
| `nav2_params.yaml` FollowPath, FollowPathThrough `desired_linear_vel` | 0.12 | 사본+시험 | 위 사본 시험 |
| `pode_avoidance/params/avoidance.yaml` FollowPathGoal `desired_linear_vel` | 0.12 | 사본+시험 | 위 사본 시험 |
| `nav2_params.yaml` FollowPathMPPI `vx_max` | 0.12 | 사본+시험 | 위 사본 시험 |
| `fleet_params.yaml` `nav_speed_mps`, `follow_speed_mps` | 0.12(전부터) | 사본+시험 | 위 사본 시험 |
| `fleet_core/common/params.py` 같은 두 기본값 | 0.12(전부터) | 사본+시험 | 위 사본 시험 |
| `pode_fsm/params.py` `horizon_cap_mps` 기본 | 0.12(전에는 0.25) | 사본+시험 | 위 사본 시험, `pode_fsm` `test_the_speed_cap_is_not_above_the_nav2_desired_speed` |
| `tools/real_pode/start.sh` 속도 기본 | 0.12(전부터) | 사본+시험 | 위 사본 시험 |
| nav_bench `scenarios.yaml` 대역 로봇(`robot`) 속도 | 0.12(전에는 0.15) | 사본+시험 | 위 사본 시험 |

따르지 않는 것: 후진 FollowPathReverse 0.05, pode_fsm 후진·물러나기 속도, MPPI `vx_min` -0.05,
사람(0.10)·카트(0.06, 0.05) 같은 움직이는 장애물, Isaac 이동체의 명목 속도. 사람과 카트는 로봇이 아니라 제 속도로 간다.
그리고 아래 둘은 이번에 바꾸지 않았다(Codex 리뷰가 짚음, 7절 남은 일):

- Isaac 대본 로봇의 속도 무작위 배율 `robot_speed_scale` (0.67, 1.33): 기준은 0.12를 읽지만 배율을 곱하면 한 대가
  최대 0.16 m/s로 간다. 지시가 "0.15 × 0.67~1.33을 같은 값으로"였으므로 배율은 그대로 두었다. 1.0을 넘지 않게 할지는
  4단계 재학습에서 정할 일이다.
- 가짜 로봇(`sim/fake_robots`)의 `speed_mps`: 기본 0.2, 시나리오 파일 12개에 0.18~0.20. 관제 시험용 대역이고(시간은
  `speed_factor`로 당김) 바꾸면 그 시험들의 시각이 모두 움직인다. 출처를 따르지 않고 사본 시험에도 없다.

Nav2에 velocity_smoother는 없다(설정에 없음). behavior_server에는 회전 속도만 있다.

### 2.1 FollowPathRL의 정책 v_max 검사

플러그인은 configure 때 `share/pode_nav_config/params/nav_speed.yaml`을 읽어 정책 명세의 `v_max`와 견준다.

- 다르면 configure 때 ERROR 로그를 남기고, 주행하는 동안 10초마다 같은 ERROR를 다시 남긴다.
- 명령 속도는 둘 가운데 작은 값(지금 0.12)으로 자른다. 관측의 속도 정규화는 정책 명세의 `v_max`(학습 때 값)를
  그대로 쓴다.
- configure를 거부하지는 않는다. 4단계에서 0.12 정책으로 바꿀 때까지 배포 정책 `store_arena_4b`(0.25)가 다르고,
  거부하면 시뮬 비교와 진단이 막힌다. 검사를 끄는 파라미터는 두지 않았다. 4b를 띄우는 동안 ERROR는 계속 나온다.
- 패키지 의존: `pode_rl_controller`가 `pode_nav_config`를 `exec_depend`로 둔다.

### 2.2 Isaac 에피소드 시간

규칙: 0.25 m/s에서 맞춘 시간은 `0.25 / v`를 곱한다(`task_spec.travel_time`, `TIMES_TUNED_AT_MPS = 0.25`).
로봇이 같은 거리를 가는 데 드는 시간이 늘어나는 만큼 늘린다. 0.12에서는 ×2.083이다.

| 값 | 0.25 때 | 0.12 때 |
|---|---|---|
| goal_reach `episode_s` | 20 s | 41.7 s |
| store_arena `episode_s` | 30 s | 62.5 s |
| store_arena `mover_wait_s`(사람이 로봇을 기다리다 돌아서는 시간) | 2 s | 4.17 s |
| store_arena `robot_wait_s`(대본 로봇이 우리 로봇을 기다리는 시간) | 3 s | 6.25 s |
| store_arena `robot_backoff_s`(대본 로봇이 시간으로 물러날 때의 시간. 기본 `robot_retreat=1`에서는 쓰지 않고 지난 노드까지 물러난다. `robot_retreat=0`일 때만) | 2 s | 4.17 s |

늘리지 않은 것: 멈춤 시간(`mover_pause_s`, `robot_pause_s`)은 이동과 상관없다. `timeouts.py`의 기본
`episode_s=30`은 `outcome`이 없는 예전 롤아웃(0.25 m/s, 30 s)에만 쓰이므로 그대로 두었다. 보상 무게(`w_time` 등)와
이동체 양보 섞기(사용자 결정 B48), 벽 스침 대책은 4단계 재학습의 몫이라 손대지 않았다.
`w_progress`의 정규화(`v_max × dt`)는 v_max를 읽으므로 저절로 따라간다.

## 3. 결정 목록

| # | 결정 | 누가 | 한 일 |
|---|---|---|---|
| 1 | 배포할 정책만 `policy.onnx`를 커밋한다(명세와 출처 기록과 함께) | 사용자 결정 2026-11-22 | `goal_reach_base_4096`의 onnx와 parity 픽스처를 지움. 그 `policy_spec.json`은 `test/fixtures/goal_reach_base_4096`으로 옮김(`test_observation`이 path_end 규칙을 읽음). CMake는 `store_arena_4b`만 돈다. `.gitignore` 예외는 `store_arena_4b/policy.onnx` 하나로 좁힘 |
| 2 | `store_arena_4b`는 4단계에서 0.12 정책으로 바뀔 때까지 둔다 | 사용자 결정 2026-11-22 | 그대로 |
| 3 | MPPI는 시뮬에서 계속 띄운다. 실물 Nav2는 nav2_mppi_controller 없이 떠야 한다 | 사용자 결정 2026-11-22 | `nav2_params.yaml`의 `controller_plugins`에서 FollowPathMPPI를 뺌(블록은 남김). 시뮬은 `sim_nav_params`가 블록이 있으면 목록 끝에 더함. `make_params.py`는 vx_max를 더 바꾸지 않음. 대안: Pi에 `ros-jazzy-nav2-mppi-controller`를 깔고 목록에 다시 넣기 |
| 4 | 모든 주행 속도 상한을 0.12 m/s 한 곳에서 | 사용자 결정 2026-11-22 | 2절 |
| 5 | 단일 출처는 `pode_nav_config/params/nav_speed.yaml` | 이 작업의 선택 | 2절 "이 자리를 고른 까닭" |
| 6 | 정책 v_max가 다르면 ERROR(거부하지 않음), 명령은 출처 값으로 자름 | 이 작업의 선택(지시의 기본 가운데 하나) | 2.1 |
| 7 | Isaac 시간은 ×0.25/v | 이 작업의 선택(지시의 예) | 2.2 |
| 8 | 대본 Pode(nav_bench `robot`, Isaac `ROBOT`)도 출처를 따른다. 사람·카트는 제 속도 | 지시 | 2절 |
| 9 | Gazebo `nav_max_speed`는 0.05부터 출처까지 받는다(전에는 0.12~0.20) | 이 작업의 선택 | 출처보다 빠르게는 돌 수 없게 위 끝을 출처로. 아래 끝은 더 느린 시험을 위해 0.05로 내림(느슨해진 막대, 4절) |

## 4. 느슨해진 막대

- `pode_rl_controller`의 `test_parity_goal_reach_base_4096`(ONNX Runtime C++와 PyTorch 추론 비교)을 지웠다.
  결정 1로 그 onnx가 없어졌기 때문이다. 관측·행동 픽스처는 `store_arena_4b` 쪽에 같은 경우가 남아 있다(Codex 확인).
- Gazebo `nav_max_speed`의 아래 끝을 0.12에서 0.05로 내렸다. 위 끝이 출처(0.12)가 되어 예전 범위로는 0.12 하나만
  남기 때문이다. 시험들은 "출처보다 느린 상한" 경우를 0.10으로 본다(전에는 0.14, 0.15, 0.20).
- FollowPathRL은 정책 v_max가 출처와 달라도 configure를 거부하지 않는다(2.1). 대신 ERROR가 계속 나온다.

## 5. 알아야 할 포인트

- 0.12는 이제 실물과 시뮬의 기본이다. 예전 Gazebo 기본 0.20으로 잰 결과(MPPI 벤치마크 등)와 시간·속도 지표를 바로
  견주면 안 된다.
- MPPI의 `time_steps: 40`(2초 앞)은 0.12에서 0.24 m 앞만 본다(0.25 때 0.5 m). 0.12에서 다시 맞추지 않았다.
- `store_arena_4b`는 0.25로 학습됐다. 시뮬에서 FollowPathRL을 띄우면 ERROR가 나오고 명령은 0.12로 잘린다.
  그 상태의 결과는 "0.12 정책"의 성능이 아니다. 4단계 재학습 뒤 세 컨트롤러 비교를 한다.
- `export_policy.py`는 명세의 `v_max`를 지금의 `task_spec`에서 쓴다. 0.25로 학습한 체크포인트를 지금 다시 내보내면
  명세에 0.12가 적힌다(틀린 명세). 예전 체크포인트를 다시 내보낼 때는 조심한다.
- 실물: `make_params.py`는 import 때 출처를 찾는다. 로봇의 `~/nav2fleet_ws/src/pode/pode_nav_config/params/`에
  `nav_speed.yaml`이 없으면(이 커밋 전의 배포) `start.sh`가 바로 실패한다. 배포를 새로 하면 된다.
- 실물 Nav2의 controller 목록에는 이제 MPPI가 없다. 로봇에서 MPPI를 고르면(controller_selector) 실패한다.
- Isaac 쪽은 `yaml`(PyYAML)을 import한다. Isaac Lab 환경에는 들어 있다.

## 6. 확인한 것

모두 e0183c5 위에서 돌렸다(그 뒤 ba2f156은 문서만 더함). WSL 시험은 도메인 48, 49.

| 묶음 | 결과 |
|---|---|
| fleet_core | 1229 passed, 7 xfailed |
| fleet_gz_sim | 607 passed |
| pode_fsm | 467 passed, 6 skipped |
| pode_nav_config | 111 passed |
| tools/test(test_nav_bench, test_real_pode 포함) | 267 passed. test_nav_bench만 따로 26 passed |
| fleet_manager | 137 passed |
| fake_robots | 1120 passed, 1 skipped, 27 xfailed |
| pode_rl_controller colcon test(gtest) | 20 tests, 0 errors, 0 failures |
| Windows learning(goal_reach, store_arena) | 61 passed |
| ruff check(learning tools pc pode sim) | 내 변경에서 나온 것 없음. 남은 E501 다섯은 이 작업 전부터 있던 줄(arena.py:144, drive_check.py:20, fleet_setup.py:3, one_pode.launch.py:11, test_arena_v7.py:16) |

첫 회 시험(속도 커밋 전)에서 깨진 것과 고친 것: pode_fsm 7개(잠금 경계 감속 곡선 시험이 상한 0.25를 가정 →
`horizon_cap_mps=0.25`를 시험에 명시), pode_nav_config 1개(composition 시험의 controller 목록), fleet_gz_sim 1개
(시뮬이 바꾸는 것이 AMCL만이라는 시험 → MPPI 목록 항목도 더함).

Gazebo 짧은 판(도메인 61, 무거운 작업 잠금 아래, 00:42~00:44, `run_bench.sh` a_hall 1판씩, 속도 인자 없음 = 출처 0.12):

| 컨트롤러 | 성공 | 시간 s | 주행 중 평균 vx m/s | 몸 최소 거리 m |
|---|---|---|---|---|
| RPP(FollowPathGoal) | 1/1 | 13.8 | 0.110 | 0.100 |
| MPPI(FollowPathMPPI) | 1/1 | 15.7 | 0.110 | 0.106 |

0.20 기본일 때 RPP 주행 중 평균은 0.176이었다. 둘 다 0.12 상한 아래로 달렸다. 한 판씩이라 두 컨트롤러 비교로는 쓰지 않는다.
세 컨트롤러 비교(5단계)는 돌리지 않았다.

Codex 리뷰: 병합 범위(b6c2fda..0eb7d6b)는 결함 없음(지적 하나는 parity 시험 삭제 = 4절). 속도 변경(0eb7d6b..e0183c5)은
둘을 짚었다. 둘 다 고치지 않고 2절 끝과 7절에 적었다(Isaac 속도 배율, 가짜 로봇 속도).

## 7. 남은 일

- `feat/store-arena-2026-11-21`의 5b55b75 뒤 커밋(Isaac 담당이 계속 올림)은 조정자가 합친다.
- Isaac 대본 로봇 배율(0.67~1.33)이 0.12를 넘게 한다(최대 0.16). 1.0 상한을 둘지 4단계에서 정한다.
- 가짜 로봇(`sim/fake_robots`) 속도 0.18~0.20은 출처 밖이다. 관제 시험 시각이 움직이므로 따로 정한다.
- MPPI `time_steps`를 0.12에 맞게 다시 볼지(지금 0.24 m 앞).
- 4단계: 0.12 정책으로 재학습하고 `store_arena_4b`를 바꾼다. 그러면 FollowPathRL의 ERROR가 사라진다.
- 로봇에 새로 배포해야 `make_params.py`가 `nav_speed.yaml`을 찾는다.

## 8. 직접 해 볼 것

1. `pode/pode_nav_config/params/nav_speed.yaml`의 0.12를 0.15로 바꾼다.
2. Windows에서 `python -m pytest -q learning/goal_reach learning/store_arena`: 통과한다(Isaac은 읽어서 따른다).
   `python -c "import sys; sys.path.insert(0,'learning/goal_reach'); import task_spec as t; print(t.ACTION_LIMITS.v_max, t.SPEC.episode_s)"`가
   `0.15 33.33...`을 낸다.
3. WSL에서 `pode_nav_config` 시험: `test_every_copy_of_the_top_speed_is_the_one_source`가 실패하고, 맞지 않는 사본
   이름(nav2 FollowPath, avoidance FollowPathGoal, fleet_params …)을 모두 보여 준다. 그 사본들을 0.15로 맞추면 통과한다.
4. 다시 0.12로 되돌린다.
