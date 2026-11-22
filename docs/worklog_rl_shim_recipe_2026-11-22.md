# 작업 기록: RL+shim 기본, shim이 넘겨준 직후의 상태, policy_spec format 3 (2026-11-22)

브랜치 `feat/rl-shim-recipe-2026-11-22`(기준 `main` 092fdc4). 오늘 낮 작업이라 Gazebo는 돌리지 않았다(오늘 밤은 Isaac 차례).
분석 스크립트와 출력은 `docs/rl_shim_recipe_2026-11-22/`에 있다.

## 결정 목록

사용자 결정(2026-11-22 낮):

1. **RL 기본은 RL+shim이다.** 시뮬 덮어쓰기 `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml`의 `FollowPathRL`이
   이제 RotationShim으로 감싼 정책이고, 감싸지 않은 같은 정책은 비교용 `FollowPathRLAlone`으로 남는다.
   `follow_path_rl_shim.yaml`은 같은 내용이 되어 지웠다. 실물 `pode/pode_nav_config/params/nav2_params.yaml`은
   Q2 → E-3 벽 접촉의 원인을 안 뒤에만 바꾼다. 이번에는 바꾸지 않았다.
2. **후진 막기(`rear_stop_range`)는 기본 끔(0.0)이고 벽 스침 대책이 아니다.** 두 RL 블록 모두 0.0이다.

내가 정한 것(바꿔도 된다):

3. **format 3 기억이 처음으로 돌아가는 때.** 학습 에피소드는 제자리에서 시작하고 prev_action 0, 라이다 기록의 모든 칸이
   현재 프레임이다. 플러그인은 다음 때 이 상태로 되돌린다:
   - activate와 `reset()`(Nav2가 controller를 다시 쓸 때);
   - 새 계획의 마지막 점이 1 mm보다 옮겨졌을 때(새 goal). 같은 goal로 1 Hz마다 다시 짠 계획에서는 지우지 않는다.
     학습 에피소드 안의 기억은 끊기지 않기 때문이다;
   - 정책이 앞 주기에 돌지 않았을 때(마지막 정책 호출에서 제어 주기 2개, 0.10 s보다 오래 지남). shim이 도는 동안이나
     goal에서 제자리 회전하는 동안이 그렇다.
   shim이 넘겨준 직후에 prev_action을 shim의 마지막 명령(0, ±1.0)으로 두지 않은 까닭: 정책이 그 명령을 내지 않았고, 학습에서
   "정책이 내지 않은 앞 명령"이 나오는 상태는 에피소드 시작(0)뿐이다. 돌던 동안의 라이다 프레임도 정책이 겪은 과거가 아니어서
   현재 프레임으로 채운다. 그래서 넘겨받은 순간이 학습의 에피소드 시작과 가장 가깝다. 남는 차이는 기본 관측의 v, w다.
   넘겨받을 때 w는 아직 약 1.0 rad/s다(아래 1절). 이것은 학습의 출발 분포로 메운다.
4. **speed_cap에 넣는 값은 플러그인이 지금 자르는 값과 같다.** `nav_speed.yaml`이고, 시뮬이 `max_linear_vel`로 더 낮추면
   그 값이다. 학습 범위로 자른 값을 정책에 주고 a0을 그 값에 맞춰 바꾼다. 범위 밖이면 configure에서 ERROR를 내고, 달리는
   동안 10 s마다 다시 낸다. costmap 속도 필터(`setSpeedLimit`)는 명령만 자르고 관측에는 넣지 않는다(계약이 원천 값이라 했다).
5. **format 3 명세에서 기본 v, w 배율과 다른 블록 배율은 거부한다.** 기본 관측의 v, w는 지금처럼 `action.v_max`,
   `action.w_max`로 나눈다. 계약은 prev_action과 speed_cap이 같은 상수(0.12, 1.0)를 쓴다고 했다. 그래서
   `prev_action.v_scale/w_scale`이나 `speed_cap.obs_scale`이 `action.v_max/w_max`와 다르면 configure에서 멈춘다.
   Isaac의 내보내기(f3b5352 `export_policy.block_entries`)는 `action.v_max` = 0.12와 같은 배율을 적으므로 이 검사를 통과한다.
6. **compare_012 조건으로 다시 잴 때는 `results/2026-11-22/compare_012/rl_and_shim.yaml`을 쓴다.** 그 파일에서는
   `FollowPathRL`이 RL 혼자이고 `FollowPathRLShim`이 RL+shim이라, `report.py`의 이름표("RL", "RL+shim")가 맞는다.
   새 기본 덮어쓰기로 재면 `FollowPathRL`이 RL+shim인데 표에는 "RL"로 나온다(`FollowPathRLAlone`은 "RL alone").

사용자가 정할 것:

7. **shim 쪽 선택지**(아래 1.4절에 장단점). 실물 `nav2_params.yaml`은 이것과 원인 확인 뒤에 바꾼다.
8. **원인 확인 끝**(1.6절): 이른 넘김은 모두 회전 충돌 검사의 예외였다. shim을 실물에 넣을지는 새 레시피 정책으로
   item 8 비교를 한 뒤 정한다(1.6절의 뜻).

## 1. shim이 넘겨준 직후의 상태 (항목 1)

### 1.1 Jazzy RotationShim의 동작 (설치된 1.3.13, 소스로 [확정])

WSL에 설치된 판은 `ros-jazzy-nav2-rotation-shim-controller` 1.3.13이다. 소스는 ros-navigation/navigation2 태그 1.3.13의
`nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp`이고, jazzy 브랜치와 같다. 줄 번호는 그 파일 기준이다.

- 새 계획마다(`setPlan`, BT가 약 1 Hz로 다시 짬) `path_updated_`가 참이 된다(`rotate_to_heading_once`가 기본 거짓, :395).
- 오차는 계획 첫 점에서 0.15 m 이상 떨어진 첫 점까지의 방향이다(:262-287). 처음에는 0.785 rad보다 크면 돌기 시작하고,
  돌던 중에는 `angular_disengage_threshold`보다 크면 계속 돈다. 이 값은 설정하지 않아 기본 0.785 / 2 = 0.39 rad다(:57, :229-230).
- 회전 명령: ±1.0 rad/s, 앞 각속도에서 2.0 rad/s² × 0.05 s씩 바뀌고, 멈출 거리에 맞춰 √(2·2.0·|오차|)로 줄인다(:315-336).
  0.39 rad에서 이 값은 1.25 rad/s라, 넘겨줄 때 w는 아직 1.0 rad/s다.
- 회전 명령마다 `isCollisionFree`가 앞으로 `simulate_ahead_time` 1.0 s를 발자국으로 돌려 본다. local costmap 칸 값이
  `max_cost_threshold`(기본 LETHAL 254, :65) 이상이면 `NoValidControl`을 던진다(:372, :377).
- `NoValidControl`과 TF 실패(`ControllerTFError`, :306)와 짧은 경로(:266)는 모두 `std::runtime_error`이다. 셋 다
  :245의 `catch`에서 잡혀 `path_updated_ = false`(:251)가 된다. 그러면 같은 주기에 `in_rotation_ = false`가 되고 정책이
  불린다(:256-257). 로그는 `RCLCPP_DEBUG` 한 줄뿐이다(:246). warn 수준으로 띄운 bench에는 남지 않는다.
  **회전이 일찍 끝나는 길은 이 셋과 정상 종료(오차 0.39 rad 이하)뿐이다.** goal 근처의 방향 맞추기(:166-204)는 출발에는 해당이 없다.
- 덤으로 본 것: 회전을 미리 돌려 볼 때 문턱에서 멈추는 조건(:352-361)이
  `shortest_angular_distance(yaw, initial_yaw) = −w·t`를 쓰고, 우변은 `|오차| − 0.785`다. 오차가 0.785 rad보다 크면
  오른쪽 회전(w < 0)만 문턱에서 멈추고, 왼쪽 회전(w > 0)은 1.0 s를 다 돌려 본다. 오차가 0.785 rad 이하(돌던 중
  0.39~0.785 rad)면 우변이 음수라, 오른쪽은 첫 칸에서 바로 멈추고 왼쪽도 0.785 − |오차|가 w × 0.05 s보다 크면 첫 칸에서
  멈춘다. 즉 그 구간에서는 사실상 검사를 하지 않는다. Q2의 회전은 왼쪽이고 넘길 때 오차가 2.4~2.8 rad라 앞의 경우(1 s 전체)다.

### 1.2 Gazebo 기록에서 본 것

compare_012의 RL+shim 판(`docs/rl_shim_recipe_2026-11-22/handover.py`, `shim_cycles.py`):

| 판 | shim이 넘긴 때 | 넘긴 자세 | 넘길 때 w | 넘길 때 오차 |
|---|---|---|---|---|
| Q2 → E-3 0 (벽 접촉) | 1.17 s | (1.660, 0.150, 0.67 rad) | 1.00 | +2.43 rad |
| Q2 → E-3 1 | 0.42 s | (1.660, 0.150, 0.11 rad) | 0.54 | +2.73 rad |
| Q2 → E-3 2 | 0.58 s | (1.660, 0.150, 0.17 rad) | 0.60 | +2.84 rad |
| O_IN → M_E 8판(a_hall, c_moving, d_robot, e_corridor 각 2) | 2.36~2.53 s | (0.490, 0.625, −0.03~−0.11) | −1.00 | −0.37~−0.44 rad |

- O_IN에서는 여덟 판 모두 오차 0.39 rad 부근에서 넘겼다(정상 종료).
- Q2에서는 세 판 모두 오차 2.4~2.8 rad, 아직 돌던 중에 넘겼다. 계획이 바뀌지 않은 때다(0 판: 계획은 0.0 s와 1.02 s, 회전은
  0.03~1.13 s). 소스상 이것은 위의 예외 셋 중 하나다.
- 명령 기록에서 v가 정확히 0인 구간(shim이 도는 주기)은 모두 새 계획 직후에 시작한다. Q2 0 판은 2.04 s 계획 뒤 2.07~2.27 s에
  다시 돌다가 다시 일찍 끝났다. Q2 1, 2 판도 계획마다 다시 돌고 일찍 끝나기를 되풀이했다.
- [추측] 셋 중 회전 충돌 검사일 가능성이 크다. 일찍 끝나는 일이 자리에 매여 있다(Q2 3/3, O_IN 0/8). local costmap은
  라이다 장애물 층뿐이고 0.025 m 칸이다. Gazebo 라이다 잡음 σ 0.02 m(최솟값이 약 0.04 m 짧게 읽힘)로 남쪽 벽 칸이 안쪽에
  찍히면, 앞으로 1 s 돌려 본 발자국 둘레에 LETHAL 칸이 걸릴 수 있다. 실제 기하로는 그 회전의 발자국이 벽에서 0.065 m 떨어져
  있다. TF 실패는 자리에 매이지 않고, 짧은 경로는 아니다(계획 점 수십 개). DEBUG 로그나 costmap 기록이 있어야 [확정]할 수 있다.

### 1.3 528 출발점 검사 (shim 모형 포함)

`docs/rl_shim_recipe_2026-11-22/shim_sweep.py`:
- 경로: 학습 경로 44개 × 출발 방향 12개, 운동학 모형(유니사이클 + 가속 제한, 학습의 측정 진열대);
- 정책: ppo_k `model_998.pt`. 묶음 계산이라 체크포인트를 썼다. `store_arena_4b/policy.onnx`와의 차이는 무작위 관측 64개에서
  최대 1.25e-06이다;
- shim: 위 1.1의 논리를 근사로 옮기고 1 s마다 새 계획을 낸다. 다른 점 둘: 0.15 m 표본점을 계획 첫 점이 아니라 로봇 위치에서
  재고, costmap 충돌 검사는 구현하지 않았다(그 대신 아래 "회전 끊김" 가정);
- "회전 끊김 T s"는 shim이 T s 돈 뒤 오차와 상관없이 정책에 넘기고 다시 돌지 않는 경우다. 충돌 검사가 계속 거부하는 상황을 흉내 낸다;
- 스침 = 몸(URDF base_link 상자) 거리 0.005 m 이하.

| 경우 | 성공 | 평균 시간 s | 스침 | 닿음(≤ 0) | 몸 최소 m | 넘긴 상태 |
|---|---|---|---|---|---|---|
| RL 혼자 | 528/528 | 11.25 | 0 | 0 | 0.010 | - |
| shim 정상 | 528/528 | 10.80 | 0 | 0 | 0.031 | 405판, \|오차\| 20~22°, w 1.0(경로 쪽) |
| 회전 끊김 0.25 s | 528/528 | 11.19 | 0 | 0 | 0.005 | w 0.5 |
| 회전 끊김 0.5 s | 528/528 | 10.89 | 0 | 0 | 0.008 | \|오차\| 40/94/150° (p10/50/90), w 1.0 |
| 회전 끊김 0.75 s | 528/528 | 10.63 | 0 | 0 | 0.007 | |
| 회전 끊김 1.0 s | 527/528 | 10.58 | 1 | 1 | 0.000 | \|오차\| 22/65/121°, w 1.0 |
| 회전 끊김 1.25 s | 528/528 | 10.60 | 0 | 0 | 0.009 | |

닿은 한 판: M_E → E-3, 출발 +90°. 1.0 s에 방향 46°, w −1.0, 오차 −109°로 넘겨받았고, 2.80 s에 (1.594, 0.482, 94°)에서
shelf_3에 닿았다. 닿은 곳은 몸 뒤 왼쪽 모서리 (−0.074, +0.047)로, Gazebo Q2 접촉과 0.25 정책의 스침과 같은 모서리다.

Gazebo 판을 운동학으로 다시 돌린 것(Gazebo 계획 그대로):
- 0 판이 넘겨받은 상태 (1.660, 0.150, 0.67 rad, w 1.0)에서 정책만: 몸 최소 0.026 m. 같은 자세에 w 0이면 0.062 m.
  라이다 잡음 0.02 m 50번: 스침 0, 최소 0.031 m;
- 1, 2 판의 상태(w 0.54, 0.60)에서는 0.082 m 이상. shim 정상과 RL 혼자는 0.062 m, 0.082 m.

읽는 법:
- 정상적으로 넘긴 상태는 스침을 만들지 않는다. 일찍 넘긴 상태(큰 오차, 아직 도는 중)는 벽과 진열대 가까이에서 여유를
  2~3 cm 깎고 한 판에서 닿게 했다.
- 같은 자세에서 w만 1.0이냐 0이냐로 3.6 cm가 갈린다. 운동학 모형은 Gazebo가 닿은 0 판을 0.026 m로만 재현한다. 남은 차이는
  모형 밖의 것이다(Gazebo의 바퀴·관성, 실제 라이다 잡음 모양).

**Isaac 출발 분포 제안**(조정자에게 보냄, 오늘 밤 공통 기본으로 전달됨):
- v0 = 0;
- w0 = ±1.0 rad/s(±0.5도 섞음), 부호는 경로 쪽으로 도는 방향;
- 경로 방향과의 오차는 두 갈래: |21°|(정상 종료)와 40°~160°(회전 끊김);
- 출발점은 벽·진열대 옆 모서리 노드(Q2, M_E, E-3 등);
- format 3이면 prev_action 0, 라이다 기록은 현재 프레임(플러그인이 shim 뒤 기억을 지우므로).

### 1.4 shim 쪽 선택지 (사용자 결정, 실물 설정은 그대로)

| 선택지 | 효과 | 대가 |
|---|---|---|
| `simulate_ahead_time` 1.0 → 0.3~0.5 s | 미리 돌려 보는 각도가 줄어 벽 옆에서 거부가 준다 | 진짜 회전 충돌을 늦게 본다. RL은 제자리 회전에 충돌 검사가 없다 |
| `max_cost_threshold` 254 그대로 / 255 | 255면 알 수 없는 칸만 거부한다(사실상 검사 끔). 254보다 낮추면 거부가 늘어난다 | 끄면 shim이 벽에 대고 돌 수 있다 |
| local costmap 해상도 0.025 → 0.01 m | 잡음으로 안쪽에 찍히는 칸이 얇아져 거부가 준다 | 계산이 늘고 팀 설정(costmap 해상도)과 엮인다. 실물 라이다 잡음은 아직 모른다 |
| `angular_dist_threshold` 0.785 → 더 크게(예: 1.2) | shim이 덜 끼어들어 정책이 큰 오차를 스스로 다룬다(학습한 상황) | RL 혼자의 약점(O_IN 북쪽으로 휨, d_robot 1/6)이 돌아온다 |
| `rotate_to_heading_once: true` | 같은 goal의 재계획마다 다시 돌지 않는다. 일찍 끝나기의 되풀이가 준다 | 경로가 크게 바뀌어도 다시 돌지 않는다 |
| `angular_disengage_threshold`를 적어 둠(0.39) | 동작은 같고 기본값에 기대지 않게 된다 | 없음. 사용자 결정 2026-11-23: 적용. `follow_path_rl.yaml`과 8번 비교 설정 `item8_overrides.yaml`의 shim 셋에 0.39. compare_012는 기본값 0.785 / 2 = 0.3925를 썼으므로 8번은 그보다 0.0025 rad 작다. 나머지 shim 선택지는 8번 비교 뒤에 정한다 |
| 학습 출발 분포(1.3절) | shim이 어떻게 넘기든 정책이 그 상태를 겪어 본다 | 재학습 한 번. 오늘 밤 Isaac 기본에 들어감 |

### 1.5 원인을 [확정]하는 Gazebo 실행

- RL+shim으로 Q2 → E-3를 3판, O_IN → M_E를 2판 돌린다(compare_012와 같은 조건).
- controller_server만 DEBUG로 띄우고(`--ros-args --log-level <ns>.controller_server:=debug`), local costmap을 bag으로 남긴다.
- 실행(WSL, `plug.sh` 방식으로 `~/nf_src/pd-plugin2`에 맞추고 빌드한 뒤):
  `bash ~/nf_src/pd-plugin2/docs/rl_shim_recipe_2026-11-22/debug_run.sh [OUT]`.
  - 잠금 `.heavy_run.lock`을 한 번 잡고(owner 줄), Gazebo 한 대(도메인 61)에서 `bench_node`를 두 번 돈다:
    `q2e3.yaml` 3판, `scenarios.yaml` a_hall 2판, 모두 `--controllers FollowPathRLShim`(되돌아가는 길도 RL+shim).
  - Nav2 설정은 `debug_overrides.yaml` = compare_012 `rl_and_shim.yaml` + local costmap `publish_frequency` 5.0
    (갱신 주기 5 Hz와 같게, 발행만 바뀌고 주행에는 영향 없음). compare_012와 다른 점은 이것과 로그 수준뿐이다.
    `nav_log_level:=pode1.controller_server:=debug`라 다른 Nav2 노드는 warn 대신 기본 info로 찍는다.
  - `ros2 bag record --use-sim-time`으로 `/rosout`, local costmap, odom, cmd_vel, plan, sim truth를 남긴다(시각이 판 기록과
    같은 sim 시간).
  - Gazebo가 끝나면 바로 잠금을 놓고 `gazebo done` 줄을 `runs.txt`에 쓴다.
- 분석: (bag이 강제 종료됐으면 먼저 `ros2 bag reindex OUT/bag -s mcap`) `python3 docs/rl_shim_recipe_2026-11-22/debug_extract.py OUT`,
  그다음 이른 넘김마다 닿는 칸을 세계 좌표로 보는 `debug_cells.py OUT`. 판마다 제자리 회전(v = 0, w ≠ 0)을 찾아,
  끝날 때의 오차로 이른 끝(EARLY, |오차| > 0.44 rad), 정상 끝, goal 방향 맞추기(goal)를 가르고, 그 사이의 :246 줄의
  `what()`으로 원인(collision / tf / short_path)을 붙인다. 로봇 상태(실제 자세, 오차, w)와, 그때의 local costmap과 odom
  자세로 shim의 충돌 검사(1 s, 0.05 s 칸, LETHAL 칸이 발자국 외곽에 닿는지)를 다시 돌린 결과(닿은 시각과 칸)를 붙인다.
  결과는 `OUT/debug_extract.json`.
- 셀 것: catch의 DEBUG 줄(:246)이 회전이 일찍 끝난 때와 맞는지, 그 순간 발자국 둘레의 LETHAL 칸 위치. 이 줄의 앞부분 문구는
  세 예외에 공통이라, 원인은 끝의 `what():` 내용으로 가른다("detected collision ahead!" 또는 "detected a potential collision
  ahead!"면 충돌 검사, "Failed to transform"이면 TF, "Path is too short"면 짧은 경로).
- 무거운 작업 잠금 아래에서 조정자가 "go"를 보내면 돌린다.

### 1.6 결과: 이른 넘김은 회전 충돌 검사다 [확정]

2026-11-22 16:35:46~16:39:57에 돌렸다(잠금을 잡고 놓음, 도메인 61, WSL `~/rl_gz_check/shimdbg_1122/`). 다섯 판 모두
SUCCEEDED: Q2 → E-3 16.8 / 36.5 / 20.6 s, O_IN → M_E 12.5 / 12.5 s. 판마다 출발점으로는 순간이동했다
(RL+shim으로 되돌아가기가 실패해 bench가 옮김). 기록한 판은 그대로 compare_012와 같은 출발이다.

- **원인 [확정]**: Q2 → E-3 세 판의 이른 넘김 11번(판별 3 / 1 / 7번) 모두 바로 그 주기에 :246 줄이 있고, `what()`이
  전부 `RotationShimController detected collision ahead!`이다. 이 줄은 15번 찍혔다(한 회전에 두 번 찍힌 때 포함).
  controller_server 로그 파일(`roslog/controller_server_*.log`)에도 같은 15줄이 있다. TF 실패와 짧은 경로는 0번이다.
  즉 `isCollisionFree`(:377)가 local costmap의 LETHAL 칸을 보고 예외를 던졌고, shim이 그걸 삼키고 정책에 넘겼다.
- 이른 넘김 때의 상태: 오차 +2.4~+3.0 rad 또는 −2.8~−2.9 rad, |w| 0.04~0.94 rad/s(오차 쪽으로 도는 중, 회전을 막 시작했거나 다시 시작한 때),
  자세 (1.46~1.66, 0.11~0.15). 남쪽 벽까지 실제 몸 거리는 0.051~0.092 m였다.
- 그 근처(넘김 −0.6~+0.2 s)의 local costmap과 넘김 때의 odom 자세로, 기록된 w와 ±1 rad/s 두 가지로 shim 검사를 다시
  돌리면 11번 중 10번에서 발자국 외곽이 LETHAL 칸에 닿는다(`debug_cells.py`, LETHAL만 셈, 지도 밖은 0번). 이것은
  그 조건에서의 재현이지, 예외 당시의 정확한 칸과 명령은 아니다. 닿은 칸은 세계 좌표 y = 0.024~0.073 m, 즉 남쪽 벽(y = 0) 선보다 2.4~7.3 cm 안쪽이다.
  벽을 잡은 라이다 점이 잡음(σ 0.02 m)과 0.025 m 칸으로 안쪽에 찍힌 것으로 본다(왜 7 cm까지 들어오는지는 [추측]:
  잡음만으로는 크다. Gazebo 벽 두께나 라이다 위치 차이일 수 있다). 재현되지 않은 1번은 bag의 5 Hz costmap 사이에 칸이 있었을
  것으로 본다([추측]).
- O_IN 출발 두 판은 예외 없이 정상 종료(오차 −0.40, −0.44 rad, "is at the new path's rough heading" 줄)였다.
- 결과 지표(Q2 → E-3 3판): 몸 최소 0.036 ± 0.013 m, 충돌 0, BLOCKED 1(36.5 s 판, E-3에서 방향 맞추기를 여러 번 함).
  compare_012의 같은 칸은 0.033 ± 0.034 m, 충돌 1이었다. 판 수가 3이라 차이를 말할 수는 없다.
- 실행의 흠: bag 기록기가 백그라운드라 SIGINT를 무시해 마지막에 강제 종료됐다. `ros2 bag reindex bag -s mcap`으로
  색인을 다시 만들어 전부 읽었다(29331 메시지). 그 뒤 `debug_run.sh`는 기록기에 SIGTERM을 보낸다(WSL에서 따로 확인함:
  SIGTERM이면 `metadata.yaml`까지 쓰고 끝난다).

실물 `nav2_params.yaml`에 shim을 넣는 일에 대한 뜻(실물 파일은 그대로, 사용자 결정):

- 벽에서 몸이 5~9 cm인 자리(Q2 같은 남쪽 진열대 앞)에서 shim은 제자리 회전을 자주 거부하고, 그때마다 정책이
  큰 오차와 0이 아닌 w로 출발한다. "shim이 벽 옆 회전을 대신 맡는다"는 기대는 이 자리에서 맞지 않는다.
- 거부가 실제 위험을 막은 것인지는 모른다([추측]). costmap의 벽 칸이 실제 벽보다 2~7 cm 안쪽이라, 실제 몸 거리
  5~9 cm에서 거부했다. 실제 벽에 닿았을지는 이 실행으로 알 수 없다. 그래도 검사를 끄면(`max_cost_threshold` 255)
  진짜로 막힌 회전도 그대로 돌게 되므로, 끄는 쪽은 권하지 않는다.
- 넣는다면 정책이 그 넘김 상태를 학습한 뒤여야 한다(1.3절 출발 분포, 오늘 Isaac 기본에 들어감). 그 정책으로 item 8
  비교를 한 뒤 정하는 것이 순서다. 거부를 줄이는 손잡이는 1.4절의 `simulate_ahead_time`(더 짧게)과
  `rotate_to_heading_once`(같은 goal의 재계획마다 다시 돌지 않음)다. 둘 다 아직 Gazebo로 재 보지 않았다.
- 실물 라이다 잡음과 실물 벽에서의 costmap은 모른다. 실물에서도 같은 빈도일지는 실물 시험 전까지 [추측]이다.

## 2. policy_spec format 3 지원 (항목 8 준비)

계약(조정자가 전달, Isaac `feat/recipe-2026-11-22`의 `task_spec.py`, 아직 커밋 전)에 맞춰 만들었다:
- 관측 = 기본 41개 그대로 ++ prev_action (2) ++ lidar_history (36 × (frames − 1)) ++ speed_cap (1). 이 순서이고 고른 것만 들어간다;
- `intent`가 적힌 명세는 configure에서 "obs block intent is not supported by this FollowPathRL yet"으로 멈춘다;
- format 2(ppo_k, `store_arena_4b`)는 바뀌지 않는다: 블록이 없고, 행동 매핑은 `v_top = action.v_max`, 속도 ERROR 검사도 그대로다.

| 파일 | 바뀐 것 |
|---|---|
| `policy_spec.hpp/.cpp` | format 2와 3을 읽는다. `obs_blocks`의 순서와 중복, 블록 필드, 관측 이름(`prev_v`, `prev_w`, `lidar_hist{f}_sector_{i}`, `speed_cap`)과 차원을 검사한다. `speed_cap_value`(범위로 자르고 밖이면 오류 문장) |
| `observation.hpp/.cpp` | `ObsMemory`(blocks: 앞 명령, 기록 칸 f = f × stride 주기 전, 모자라면 에피소드 첫 프레임 / step / reset), `action_to_target(..., v_top)`, `memory_stale`, `new_episode` |
| `follow_path_rl.hpp/.cpp` | 기억을 붙이고 결정 3, 4대로 지운다. a0은 speed_cap 값에 맞춰 바꾼다. 정책 target(가드·속도 제한·가속 제한 전)을 prev_action으로 남긴다. 설정 로그에 format, 블록, speed_cap을 더함 |
| `test/test_format3.cpp` | gtest 12개 |

시험(실제 출력):
```
[       OK ] Format3Spec.Format2LoadsUnchangedWithoutBlocks (1 ms)
[       OK ] Format3Spec.ReadsTheBlocksInTheirFixedOrder (6 ms)
[       OK ] Format3Spec.RefusesWhatThePluginCannotBuild (24 ms)
[       OK ] ObsMemory.EpisodeStartHasZeroPrevActionAndTheCurrentFrameEverywhere (3 ms)
[       OK ] ObsMemory.HistorySlotsAreStrideStepsApartNewestFirst (3 ms)
[       OK ] ObsMemory.PrevActionIsTheScaledTargetBeforeTheRateLimit (1 ms)
[       OK ] ObsMemory.ResetReturnsToTheEpisodeStart (3 ms)
[       OK ] ObsMemory.Format2HasNoExtraEntries (0 ms)
[       OK ] SpeedCap.ClampsTheSourceToTheTrainedRangeAndSaysWhenOutside (1 ms)
[       OK ] SpeedCap.ActionMapsOntoTheCapInPlaceOfVMax (0 ms)
[       OK ] EpisodeRules.MemoryIsStaleWhenThePolicyMissedAStep (0 ms)
[       OK ] EpisodeRules.ANewEpisodeStartsOnlyWhenTheGoalMoves (0 ms)
[  PASSED  ] 12 tests.
```
`colcon test --packages-select pode_rl_controller`: `Summary: 33 tests, 0 errors, 0 failures, 0 skipped`(기존 parity와
관측 시험 포함). pytest `sim/fleet_gz_sim/test`, `tools/test/test_nav_bench.py`, `pode/pode_nav_config/test`: 747 통과.

### 2.1 관측 parity (Isaac f3b5352, ONNX 없이)

`tools/rl_fixtures/make_format3_cases.py`가 레시피 코드로 고정값을 만든다. `task_spec.build_observation`,
`task_spec.action_to_target`, `export_policy.block_entries`, `obs_names`를 그대로 부르고, 기록 칸은
`store_arena_env.py`의 `lidar_history` 메서드 본문을 그대로 떼어 실행한다(Isaac Lab 없이). 순서는 env와 같다: 초기화
때 target 0과 새 기록, 관측, 행동 -> target. 입력은 19단계(에피소드 13단계 + 초기화 후 6단계, stride 4 기록이 다 차고
다시 비는 것까지), 행동은 [-1.3, 1.3]이라 자르기도 지난다. 단계마다 네 섹터는 광선이 모두 2 m를 넘어 상한 자르기도
지난다. 블록 묶음 넷(prev_action, lidar_history, speed_cap(0.06~0.12),
셋 다)의 관측과 (v, w)를 `pode/pode_rl_controller/test/fixtures/format3/format3_cases.json`(159 KB)에 적었다. f3b5352의
파일은 `git archive`로 꺼내 썼다(Isaac 작업 트리의 HEAD는 그 뒤 cf473cc였지만 이 세 파일은 같다).

```bash
git archive -o recipe.tar f3b5352 learning pode/pode_nav_config/params && mkdir recipe && tar -xf recipe.tar -C recipe
PYTHONUTF8=1 python tools/rl_fixtures/make_format3_cases.py recipe pode/pode_rl_controller/test/fixtures/format3/format3_cases.json --commit f3b5352
```

gtest `Format3Parity.ObservationAndTargetMatchTheRecipe`는 각 묶음의 명세를 base 명세 + 고정값의 블록 항목·`obs_layout`·
`obs_dim`으로 읽고(configure 검사 통과), `ObsMemory`와 `action_to_target`을 단계마다 1e-6 안에서 비교한다:
```
[       OK ] Format3Parity.ObservationAndTargetMatchTheRecipe (66 ms)
[  PASSED  ] 13 tests.
```
`colcon test --packages-select pode_rl_controller`: `Summary: 34 tests, 0 errors, 0 failures, 0 skipped`. 기록 칸 번호를
일부러 하나 밀면(`f * stride - 1` -> `f * stride`) 이 시험이 `lidar_history step 5 entry 41`부터 646건 실패한다(첫 고정값 기준).
섹터의 2 m 상한을 없애면 608건 실패한다.

### 2.2 레시피 후보 둘의 ONNX parity

Isaac `feat/recipe-2026-11-22` f178e50의 내보내기 둘(그 브랜치 worklog B79):
- `store_arena_c`: run `t_c/model_1197.pt`, 비대칭 critic만, actor 입력은 ppo_k와 같은 41개(format 2);
- `store_arena_acd`: run `t_combo_acd/model_1197.pt`, format 3, `obs_blocks` [prev_action, speed_cap], cap 0.08~0.16
  (44개).

`policy_spec.json`(출처 `source` 포함)과 parity 고정값(`test/fixtures/<이름>/`)을 커밋했다. `policy.onnx` 둘은 작업
트리에만 둔다. `.gitignore`가 배포 정책(store_arena_4b)의 onnx만 받는 것이 사용자 결정(2026-11-22)이라, 고른 정책만
그 예외를 옮겨 커밋한다. 그래서 이 브랜치를 새로 받으면 두 parity 시험은 onnx를 복사하기 전까지 실패한다.

`test_parity.cpp`는 이제 format 3 블록을 각 사례의 `prev_target`/`lidar_history`/`speed_cap`으로 `ObsMemory`에 채워
관측을 만들고, 행동 사례의 `speed_cap`을 매핑 상한으로 쓴다. format 2 정책은 전과 같은 길을 탄다. 결과(WSL):
```
store_arena_4b : obs 4.47e-08, action 3.58e-08, onnxruntime C++ vs torch 2.62e-06 (bound margin 1.01e-05), 7 tests PASSED
store_arena_c  : obs 4.47e-08, action 3.58e-08, onnxruntime C++ vs torch 2.86e-06 (bound margin 1.07e-05), 7 tests PASSED
store_arena_acd: obs 1.19e-07, action 7.15e-08, onnxruntime C++ vs torch 3.81e-06 (bound margin 1.01e-05), 7 tests PASSED
[parity] speed_cap range [0.080, 0.160], nav speed 0.120 -> 0.120
```
acd 사례는 세 관측 모두 prev_target과 speed_cap이 있고, 행동 사례는 cap 0.08과 0.16 끝을 지난다. speed_cap 원천은
플러그인 configure에서 min(max_linear_vel, nav_speed.yaml)이고, 시뮬에서는 max_linear_vel이 0.12(`sim_nav_params`)라 0.12다.
범위 밖이면 잘라서 ERROR를 낸다는 것은 `speed_cap_value`의 시험(0.07 -> 0.08, 0.17 -> 0.16, 둘 다 오류 글)으로 확인했다.
`colcon test --packages-select pode_rl_controller`: `Summary: 51 tests, 0 errors, 0 failures, 0 skipped`.

### 2.3 item 8 다섯 컨트롤러 비교 준비

- 실행(WSL, `plug.sh`로 맞추고 빌드한 뒤): `bash ~/nf_src/pd-plugin2/docs/rl_shim_recipe_2026-11-22/item8_run.sh [OUT]`.
- 컨트롤러: RPP(`FollowPathGoal`), MPPI, RL+shim(store_arena_4b, 지금 기본), RL+shim c, RL+shim acd.
  Nav2 설정 `item8_overrides.yaml` = compare_012 `rl_and_shim.yaml` + 같은 shim 설정에 정책만 바꾼 둘. 합친 설정을 미리
  만들어 확인함: 세 shim 모두 max_linear_vel 0.12, MPPI vx_max 0.12와 time_steps 83(compare_012와 같음).
- 묶음: Q2 → E-3 칸마다 3판, 그다음 O_IN → M_E 네 시나리오(a_hall e_corridor d_robot c_moving) 칸마다 6판. compare_012와
  같은 설정 파일, 판 수, 도메인 규칙이다. 잠금은 `run_bench.sh`가 시나리오마다 잡고 놓는다(사이 180 s).
- 끝나면 `report.py` 표 둘(성공, 충돌, 몸 최소 거리, 도리도리 /분, 시간 등 compare_012와 같은 지표)과 shim 셋의 이른
  넘김 수(`early_count.py`)를 만든다. 이른 넘김은 로그 없이 판 기록만으로 센다(오차 0.44 rad 넘게 남기고 끝난 제자리 회전).
  1.6절 DEBUG 실행의 판에서 이 셈은 로그로 확인한 11번과 같았다. 그래도 이 실행에는 DEBUG 로그가 없으므로 추정치다.
- 실행 스크립트를 멈추려면 그 PID에 TERM을 보낸다. 자식 `run_bench.sh`에 TERM을 넘겨 Gazebo를 끄고 잠금을 놓는다.
- 예상 시간: compare_012의 네 컨트롤러가 Q2 → E-3 8분, 본 비교 74분(사이 쉼 제외)이었다. 다섯이면 ×1.25로 10분 + 92분,
  쉼 4번 12분을 더해 약 1시간 55분이다.
- compare_012와 다른 점: 기준 코드가 이 브랜치다(format 3 지원, 시뮬 기본 RL 옵션 변경). format 2 정책의 길은 parity와
  단위 시험으로 같다는 것을 확인했고, 비교에 쓰는 이름(`FollowPathRLShim` 등)은 모두 overrides 파일에서 정한다.
  Williams 순서는 다섯이면 10판이 완전 균형이라 6판은 compare_012처럼 완전하지 않다.

## 주의

- 1.3의 검사는 운동학 모형이다. Isaac 물리도 Gazebo도 아니다. Gazebo가 닿은 상태를 0.026 m로만 재현했다.
- 1.6절의 DEBUG 실행에서 그 실행의 이른 넘김 11번이 회전 충돌 검사라는 것은 로그 줄로 [확정]했다. 로그를 남기지
  않은 compare_012(1.2절)의 넘김도 같은 원인이라는 것은 같은 자리·같은 모양이라 그렇게 보는 [추측]이다. LETHAL 칸이
  벽 선보다 최대 7 cm 안쪽에 찍히는 까닭도 [추측]이다.
- 플러그인의 기억 규칙(결정 3)은 단위 시험으로만 확인했다. ROS 안에서 shim과 같이 돈 확인은 다음 Gazebo 밤에 한다.

## 알아야 할 포인트

- **shim은 예외를 삼킨다.** 회전이 위험하다고 판단해도 멈추지 않고 감싼 controller에 넘긴다. 그래서 "shim이 막아 준다"고
  기대한 자리(벽 옆)에서 오히려 정책이 큰 오차와 큰 각속도를 안고 출발한다.
- **넘겨줄 때 w는 0이 아니다.** 멈출 거리에 맞춘 감속은 오차 0에 맞춰져 있어, 0.39 rad에서 넘길 때 여전히 1.0 rad/s로 돈다.
  학습 에피소드는 늘 w = 0에서 시작했다.
- **prev_action과 기록은 "정책이 겪은 과거"다.** 정책이 돌지 않은 동안(shim, goal 회전)의 과거는 학습에 없던 것이라,
  에피소드 시작처럼 비운다.

## 직접 해 볼 것

1. 운동학 검사를 다시 돌린다(Windows의 Git Bash, 저장소 `C:/Users/pjw/ros2camp/pd-plugin2`, 체크포인트가 D:에 있어야 함). Gazebo 판
   JSON 세 개를 WSL `~/rl_gz_check/cmp012_025415/q2e3/legs/a_hall_FollowPathRLShim_{0,1,2}.json`에서 가져온다:

   ```bash
   PYTHONUTF8=1 python docs/rl_shim_recipe_2026-11-22/shim_sweep.py shim0.json shim1.json shim2.json shim_out.json
   ```

   첫 줄의 체크포인트-ONNX 차이가 1e-5 아래인지, "shim abort 1.0 s" 줄에 M_E->E-3 한 판이 나오는지 본다.
2. `shim_sweep.py`의 `DISENGAGE`를 0.785로 바꿔(문턱을 하나로) 다시 돌린다. 넘길 때의 |오차|가 45° 부근으로 커지고
   "shim nominal"의 몸 최소가 어떻게 바뀌는지 본다.
3. format 3 시험만 돌린다(WSL, 빌드 뒤): `~/<빌드>/build/pode_rl_controller/test_format3`. `test_format3.cpp`의
   `format3({"lidar_history"}, 3, 4)`를 `(4, 2)`로 바꾸고 기대값을 고치기 전에 어떤 시험이 실패하는지 본다.
