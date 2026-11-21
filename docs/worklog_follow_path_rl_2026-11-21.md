# RL 정책을 Nav2 controller로: FollowPathRL 뼈대 — 2026-11-21

계획 7번. 빈 바닥 목표점 가기 정책(`learning/goal_reach`, base_4096 `model_299.pt`)을 ONNX Runtime으로 돌리는 Nav2 controller 플러그인 `pode_rl_controller::FollowPathRL`(C++, `nav2_core::Controller`)을 만들고, Gazebo(one_pode, 도메인 62)에서 이 플러그인으로 짧은 Nav2 goal을 몰아 보았다. 성능은 아직 보지 않는다. 나중에 매장 경기장 정책을 이 플러그인에 올려 RPP, MPPI와 견주고 Pi4에서 돌린다.

**한 줄 결론:** 된다. 관측·행동 변환은 `task_spec`과 5e-8 안에서 같고, ONNX Runtime C++ 출력은 PyTorch 체크포인트와 허용 범위(1e-5 + 1e-5·|값|) 안에서 같다. Gazebo에서 controller_server가 FollowPathRL을 띄웠고, `controller_selector`로 고른 뒤 0.77 m goal을 정책이 약 4 s에 몰아 SUCCEEDED로 끝났다. FollowPathGoal로 되돌린 goal도 SUCCEEDED였다(아래 3절). 한 번 추론은 Gazebo 안에서 평균 36~57 µs(WSL x86, 1스레드)였다. **단, 빈 바닥 정책은 2 m 안에 무엇이든 보이면 관측이 학습 범위를 크게 벗어난다**(아래 5절). 그래서 Gazebo 확인은 로봇을 경기장 밖 빈 바닥으로 옮겨서 했다.

## 1. 만든 것

| 파일 | 하는 일 |
|---|---|
| `learning/goal_reach/export_policy.py` | rsl_rl ONNX(외부 데이터 파일 둘)를 한 파일 `policy.onnx`로 묶고, `task_spec`에서 `policy_spec.json`을 쓰고, 패리티 fixture 셋을 만든다. onnxruntime(python)과 체크포인트(PyTorch)의 출력을 견주어 허용 범위를 넘으면 멈춘다 |
| `pode/pode_rl_controller/` (첫 C++ 패키지) | `src/policy_spec.cpp`(명세 읽기, 관측 배치가 다르면 거부), `src/observation.cpp`(스캔 → 학습 광선 360개 → 관측 41개, 행동 → (v, ω), 가속 제한), `src/onnx_policy.cpp`(ORT 세션), `src/follow_path_rl.cpp`(플러그인), `follow_path_rl.xml`(pluginlib), `policies/goal_reach_base_4096/`(`policy.onnx` 99 KB, `policy_spec.json`), `test/test_parity.cpp` + `test/fixtures/`(gtest 6개), `README.md` |
| `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml` | one_pode `nav_overrides:=`로 FollowPathRL을 더 띄우는 파일. controller 넷을 모두 다시 적고 FollowPathRL을 더한다. 플러그인 값은 모두 여기 |
| `sim/fleet_gz_sim/test/test_sim_nav_params.py` | 시험 하나: 기본 설정에는 FollowPathRL이 없고, 덮어쓰기 파일은 기본 controller를 하나도 빼지 않고 FollowPathRL만 더한다(nav2_params.yaml에 controller가 늘면 이 시험이 실패해 알려 준다) |
| `.gitignore` | `*.onnx` 무시에 `pode/pode_rl_controller/policies/*/policy.onnx`만 예외 |

### 1.1 플러그인 한 주기(20 Hz)

1. 부모 노드(controller_server)로 직접 받은 최신 `scan`을 꺼낸다. 없거나 0.5 s보다 오래되면 `ControllerException`.
2. 경로를 TF로 `base_footprint` 틀로 옮겨 목표점을 정한다(`goal_rule`: 경로 끝 또는 경로 위 `lookahead_dist` 앞의 점).
3. 경로 끝이 goal checker의 xy 허용치 안이면 정책을 부르지 않고 끝 방향으로 제자리 회전(ω = 1.5 × 방향 오차, ±w_max).
4. 아니면 스캔 → 광선 360개 → 관측 41개 → ORT → 행동 2개 → `action_to_target` → 속도 제한(`setSpeedLimit`) → `rate_limit`(학습과 같은 dt 0.05 s).
5. 1 s마다 INFO 한 줄: 호출 수, 추론·호출 시간(평균, 최대), 목표 거리·각도, 행동, 명령.

### 1.2 스캔을 학습 광선에 맞추는 규칙

학습(`goal_reach_env.make_lidar_cfg`): 광선 360개, k번 광선은 base 틀에서 −180 + k도(180번이 정면), 라이다 중심 base_link (−0.017, 0, 0.097)에서 잰 거리, 0.05 m 밖에서 출발, 안 맞으면 inf → 관측에서 2.0 m.
Gazebo(`pode_sim.urdf.xacro`): 640빔, −π~π(첫 빔과 끝 빔이 같은 방향, 간격 2π/639 = 0.009833 rad), 0.05~12 m, 10 Hz, 틀 `rplidar_link`. 그 틀은 base에서 yaw π로 돌아 있다(pinky_description `rplidar_link_fixed_joint` rpy 0 0 π). Gazebo에서 TF로 읽은 값: `rplidar_link in base_footprint at (-0.017, 0.000) m, yaw 3.142 rad`.

| 항목 | 규칙 |
|---|---|
| 개수·방향 | k번 광선의 base 각도 θ_k에서 스캔 빔 번호 i = floor((θ_k − lidar_yaw − angle_min) mod 2π / angle_increment + 0.5). 가장 가까운 빔 하나. 640빔이면 빔 간격 0.56°라 1° 광선마다 빔 하나가 0.28° 안에 있다 |
| 각도 원점·방향 | lidar_yaw는 TF(base ← scan 틀)의 yaw. 처음 한 번 읽어 둔다(고정 관절) |
| 끝 빔 넘어 | 마지막 빔을 넘으면 첫 빔에서 반 간격 안일 때만 첫 빔, 아니면 자료 없음(일부만 보는 스캔) |
| 위치 차 | 쓰지 않는다. 학습과 Gazebo 모두 라이다가 base 기준 (−0.017, 0)에 있고 거리를 라이다 중심에서 잰다. 다른 로봇에 올리면 TF 위치를 로그로 확인한다 |
| 안 맞음 | inf, −inf, NaN, range_min 미만(실물 RPLIDAR의 0 포함), range_max 초과, 자료 없음은 모두 "안 맞음" → 2.0 m(학습의 inf와 같다) |
| 최대 | 2.0 m에서 자른다(학습 `lidar_obs_max`) |

참조 구현은 `export_policy.scan_to_rays`(python)이고 C++ `scan_to_rays`가 그것과 같은지 fixture로 본다.

## 2. 결정 대기 목록

사용자가 바꿀 값은 두 곳에만 있다: 플러그인 값은 `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml`, 정책의 숫자는 정책 폴더의 `policy_spec.json`(손으로 고치지 않고 `export_policy.py`로 다시 만든다).

| # | 항목 | 후보 | 고른 것 | 이유 |
|---|---|---|---|---|
| 1 | 띄우는 방법 | (a) `nav2_params.yaml` controller_plugins에 더함 (b) 덮어쓰기 파일 | (b) **기본으로는 띄우지 않고 덮어쓰기 파일(`follow_path_rl.yaml`)로만 띄운다** | 다른 트랙이 같은 목록을 고치고 있다(FollowPathMPPI). 실물 설정을 바꾸지 않는다. 대가: 기본 목록이 바뀌면 이 파일도 고쳐야 한다(시험이 알려 줌). gz.launch.py는 덮어쓰기의 `plugin` 키를 거부하므로 세 대 경기에는 못 쓴다 |
| 2 | 관측 숫자의 출처 | (a) C++에 상수로 다시 적음 (b) `task_spec`에서 만든 명세 파일 | (b) `policy_spec.json`. 관측 배치 이름 목록까지 적어 C++가 자기가 만드는 배치와 다르면 시작하지 않는다 | 학습 쪽을 바꾸고 다시 내보내면 플러그인이 따라간다. 배치 순서 자체는 코드(양쪽)라 다르면 거부로 막는다 |
| 3 | 명세 파일 형식 | YAML, JSON | JSON(표준 라이브러리로 씀), C++는 yaml-cpp로 읽는다(JSON은 YAML) | python 쪽에 PyYAML이 필요 없고 C++ 의존은 Nav2가 이미 쓰는 yaml-cpp 하나 |
| 4 | 목표점 규칙 | 경로 끝, 경로 위 앞보기 점 | 파라미터 `goal_rule`, **이번 기본 `path_end`**, `lookahead_dist` 0.4 m | 빈 바닥 정책은 1~2 m 떨어진 목표점으로 학습했다. 매장 정책은 경로 위 0.3~0.5 m 앞 점으로 학습할 예정이라 그때 `lookahead`로 바꾼다 |
| 5 | 끝 방향 맞추기 | (a) goal checker 방향 허용치를 덮어써 끈다 (b) 플러그인이 마지막에 제자리 회전 | (b) goal checker xy 허용치 안에서 ω = `rotate_gain`(1.5) × 방향 오차 | 정책은 위치만 배웠고 BT는 `general_goal_checker`(방향 0.12 rad)를 쓴다. (a)는 되돌린 FollowPathGoal에도 영향을 준다. 허용치는 goal checker에서 읽으므로 2 cm checker에도 맞다 |
| 6 | ORT 실행 설정 | CPU / XNNPACK·ACL EP, 스레드 수, 순차·병렬, spinning | **CPU EP, intra_op 1(파라미터 `intra_op_threads`), inter_op 1, 순차, spinning 끔, 그래프 최적화 전부** | 41-128-128-2 MLP는 한 번에 수십 µs라 스레드를 늘려도 이득이 없고, 20 Hz 호출 사이에 도는(spin) 스레드는 Pi4 CPU만 먹는다 |
| 7 | 정책 파일 커밋 | (a) `*.onnx` 무시 유지, 내보내기 스크립트로 만듦 (b) 예외로 커밋 | (b) `policies/*/policy.onnx`만 예외(99 KB) | 플러그인 기본값과 추론 패리티 시험이 그 파일을 연다. 새 체크아웃·Pi에서 바로 돈다. 저장소 규칙(모델은 커밋 안 함)을 좁게 푼 것이라 사용자 확인 필요 |
| 8 | 가속 제한의 dt | 실제 호출 간격, 학습의 0.05 s | 0.05 s(명세 `control_dt`) | 학습과 같다. controller_frequency가 20 Hz라 같다 |
| 9 | 가속 제한의 시작 상태 | 0, 측정 속도 | 첫 호출이나 2 주기 넘게 쉬었다가 다시 부를 때 odom 측정 속도 | 학습은 정지에서 0으로 시작했다. 움직이는 중에 controller를 바꾸면 0으로 시작하면 급정거한다 |
| 10 | 속도 입력 | 명령, 측정 | 측정(controller_server가 주는 odom 속도) | 학습 관측이 몸체 속도(`root_link_lin_vel_b`)다 |
| 11 | 스캔 받기 | costmap, 스캔 직접 | 부모 노드로 `scan` 직접(SensorDataQoS) | 관측은 원래 거리값이다. costmap은 0.025 m 칸과 inflation으로 바뀐다 |
| 12 | 추론 비교 허용치 | 절대만, 절대+상대 | atol 1e-5 + rtol 1e-5(numpy.allclose) | 아래 3.1. 학습 범위 밖 입력에서 행동이 100을 넘어 절대 허용치만으로는 float32 반올림을 못 덮는다 |

## 3. 결과

### 3.1 빌드와 패리티 시험

WSL Ubuntu 24.04, ROS 2 Jazzy, ORT 1.24.3(`ros-jazzy-onnxruntime-vendor` .deb를 `dpkg -x`로 풀어 `CMAKE_PREFIX_PATH`에 둠, 실행 때는 RPATH). `colcon build --packages-select pode_rl_controller` 경고 없음, gtest 6개 통과(colcon 요약의 7은 ctest 결과 하나를 더 센 것):

```
[       OK ] Gap.NonFiniteNeverPasses
[       OK ] PolicySpec.ReadsTheExportedTaskSpec
[parity] rays max |diff| 5.91e-08, obs max |diff| 4.47e-08
[       OK ] Observation.MatchesTaskSpec
[       OK ] Observation.NoReturnBecomesObsMax
[parity] action -> (v, w) max |diff| 3.58e-08
[       OK ] Action.MatchesTaskSpec
[parity] onnxruntime C++ vs torch max |diff| 1.53e-05 (bound margin 9.99e-06), vs onnxruntime python 0
[parity] inference 7.4 us per call (mean of 2000, 1 thread)
[       OK ] Inference.MatchesTorch
Summary: 7 tests, 0 errors, 0 failures, 0 skipped
```

- 관측 fixture 셋: Gazebo 배치(640빔, yaw π), 학습 배치 그대로(360빔, yaw 0, 항등이어야 함), 일부만 보는 어긋난 스캔(500빔, −2.5 rad부터 4.8 rad, yaw 0.3). 각 스캔의 1/8에 inf, −inf, NaN, 0, 0.03, 13.0을 넣었다. 허용치 1e-6: 값이 1 이하인 관측을 float32(PyTorch)와 double(C++)로 계산한 차이(float32 한 칸 6e-8)를 덮는다.
- 행동 fixture: 시작 (0, 0)과 (0.2, −0.8)에서 40스텝씩, ±1을 넘는 행동(3.0, −3.0) 포함.
- 추론 fixture 32개: 빈 바닥처럼 라이다가 모두 1.0인 것 16개, 라이다 무작위 12개, 범위 밖 극단 4개. **ORT C++와 ORT python은 비트까지 같고(0)**, PyTorch와는 최대 1.53e-5. 이 최대는 행동 크기가 34인 곳에서 났다(상대 4.5e-7). 학습 범위 안(행동 O(1))에서는 1e-6 이하다.
- `export_policy.py`가 같은 비교를 python에서 먼저 한다: `onnxruntime 1.24.4 vs torch 2.11.0+cpu: max |diff| 1.53e-05, max |action| 126`.
- `sim/fleet_gz_sim` 시험(test_sim_nav_params, test_launch_wiring, test_arena, test_one_pode_wiring) 203개 통과, `learning/goal_reach` 9개 통과.

### 3.2 Gazebo

명령(`ROS_DOMAIN_ID=62`, `GZ_PARTITION=rl_plugin_62`, 무거운 실행 잠금을 잡고):

```bash
ros2 launch fleet_gz_sim one_pode.launch.py gui:=false rviz:=false fsm:=false keepout:=false \
  scan_filter:=false start:=P-3 nav_log_level:=info nav_overrides:=<follow_path_rl.yaml + AMCL 멈춤>
gz service -s /world/arena/set_pose ... 'name: "pode1", position: {x: 0.64, y: -4.0}'   # 경기장 밖
ros2 topic pub --qos-durability transient_local --qos-reliability reliable -r 2 \
  /pode1/controller_selector std_msgs/msg/String "{data: FollowPathRL}"
ros2 action send_goal /pode1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.40, y: 0.20}, orientation: {w: 1.0}}}}"
```

경기장 밖으로 옮긴 까닭과 방법: v7 경기장은 안쪽이 1.9 × 0.9 m라 어디서든 2 m 안에 벽이 보인다. 빈 바닥 정책은 그런 관측을 본 적이 없다(5절). 그래서 Nav2가 P-3(0.64, 0.10)에서 떠서 위치를 잡은 뒤 Gazebo에서 로봇만 (0.64, −4.0)으로 옮겼다. 바퀴 odom은 순간 이동을 모르므로 Nav2는 계속 P-3 둘레에 있다고 믿고 지도 위에서 경로를 짜고, 실제 로봇은 벽이 3.9 m 떨어진 빈 바닥을 같은 모양으로 달린다. AMCL이 옮겨진 스캔을 쫓지 않게 이번 확인에만 `update_min_d`, `update_min_a`를 100으로 덮었다(이 값은 저장소에 넣지 않았다). 옮긴 뒤 스캔: `frame rplidar_link n 640 angle_min -3.1416 inc 0.009833`, 맞은 빔 53개, 최솟값 3.93 m(모두 2.0 m로 들어감).

결과(로그 `~/rl_gz_check/161841/` in WSL):

| 확인 | 근거 |
|---|---|
| controller_server가 FollowPathRL을 띄움 | `Created controller : FollowPathRL of type pode_rl_controller::FollowPathRL`, `Controller Server has FollowPath FollowPathThrough FollowPathReverse FollowPathGoal FollowPathRL  controllers available.` |
| 라이다 TF | `FollowPathRL: lidar rplidar_link in base_footprint at (-0.017, 0.000) m, yaw 3.142 rad` |
| goal 1(FollowPathRL, 0.77 m, 시작 방향과 목표 방향이 −1.44 rad 어긋남) | sim 시각 537.99 s에 받음 → 542.19 s `Reached the goal!`, `Goal succeeded`, 결과 `SUCCEEDED`(error_code 0). 약 4.2 s |
| 정책이 몬 것 | 1 s 로그 넷: 목표 0.767 → 0.701 → 0.481 → 0.232 m, 각도 −1.44 → −0.84 → −0.09 → −0.01 rad. 마지막 0.026 m에서 `rotate`(끝 방향 맞추기) |
| cmd_vel(goal 1 구간 84개) | v −0.050~0.250 m/s, ω −1.000~0.279 rad/s. 첫 명령 (−0.035, −0.100): 목표가 오른쪽 옆이라 살짝 뒤로 빼며 돈다. 그 뒤 v는 0.25(v_max)에 붙는다 |
| 실제 위치(Gazebo truth) | (0.640, −4.000) → (1.374, −3.882), 끝 yaw −0.087 rad. Nav2가 믿는 목표에 해당하는 실제 점은 (1.40, −3.90)이라 3.2 cm 앞에서 멈췄다(xy 허용치 5 cm). 가는 길에 북쪽으로 최대 0.16 m 부풀었다가 돌아왔다 |
| 추론 시간(20 Hz, 1 s 평균/최대) | 첫 호출 297 µs, 그 뒤 57/169, 50/83, 50/74, 36/78 µs. 한 주기 전체(TF, 관측, 추론) 82~123 µs 평균, 최대 233 µs |
| FollowPathGoal로 되돌리기 | `controller_selector`에 FollowPathGoal을 내고 goal 2((0.90, 0.20), 방향 π) → 547.56 s 받음, 553.86 s `Reached the goal!`, `SUCCEEDED`. 이 구간에 FollowPathRL 로그 0줄(RPP가 몰았다). cmd_vel 126개, v 0~0.200(시뮬 속도 상한 0.20), 실제 위치 (0.943, −3.929) |
| 정리 | 끝난 뒤 gz sim, 브리지, Nav2 노드가 남지 않은 것을 확인하고 잠금을 풀었다 |

### 3.3 추론 시간 읽는 법

- gtest의 7.4 µs는 같은 관측을 2000번 이어 부른 평균이다(캐시가 데워진 상태). Gazebo 안 20 Hz 호출은 평균 36~57 µs, 최대 169 µs, 첫 호출 297 µs다. 50 ms마다 한 번씩 부르면 그 사이 캐시와 CPU 클럭이 식어서 더 느리다. Pi4에서 비교할 숫자는 Gazebo 쪽(20 Hz로 띄엄띄엄 부른 값)이다.
- "call"은 TF로 경로를 옮기고 관측을 만들고 추론하는 한 주기 전체다(평균 82~123 µs). controller 주기 50 ms의 0.2 %다.

## 4. 막힌 것

- 하네스가 이 작업 폴더의 일부 셸 명령(`.git`이라는 글자가 든 rsync 인자)을 막아서, WSL 빌드·시험 명령을 scratchpad 스크립트로 옮겨 실행했다.
- gz.launch.py는 nav_overrides에 controller `plugin` 키가 있으면 거부한다(속도 상한을 RPP 키로 거는 장치). 그래서 FollowPathRL 덮어쓰기는 one_pode에서만 쓰고, 그 시험은 `test_sim_nav_params.py`에 두었다.
- 처음에는 추론 비교를 절대 허용치 1e-5로만 두었다가, 학습 범위 밖 입력(라이다 무작위)에서 1.53e-5가 나와 멈췄다. 원인은 아래 5절의 정규화 폭주라 상대 항을 더했다(결정 12).

### 4.1 검토와 고친 것

Codex 검토(0aeffaa..3ffb097)에서 [확정] 넷:
1. `*.onnx`가 무시되어 `policy.onnx`가 커밋에 없었다 → 좁은 예외로 커밋(결정 7). 맞는 지적.
2. 제자리 회전 반경이 5 cm로 고정이라 2 cm checker를 못 맞춘다 → goal checker의 `getTolerances` xy 값을 쓴다. 맞는 지적.
3. 시험이 `std::max`로 차이를 모아 NaN이 지나갔다 → `gap()`이 유한하지 않은 값을 +inf로 바꾼다(시험 `Gap.NonFiniteNeverPasses` 먼저). 맞는 지적.
4. 패리티 시험이 플러그인의 TF·속도 연결(실제 `computeVelocityCommands` 경로)은 보지 않는다 → 맞다. 이번에는 고치지 않았다: Gazebo 확인(라이다 TF yaw 3.142 로그, 목표 쪽으로 가는 주행)이 그 연결을 한 번 보았을 뿐이다. 플러그인 단위 시험(가짜 TF·스캔)은 남은 일이다.
고친 커밋 b67f172의 재검토는 "결함을 발견하지 못했습니다".

## 5. 배운 것

- **빈 바닥 정책의 라이다 입력은 사실상 쓰이지 않는다.** 학습 중 2 m 안에 아무것도 없어 라이다 36구역이 늘 1.0이었고, rsl_rl 관측 정규화의 표준편차가 0(두 구역만 1e-5)이 되었다. ONNX 안의 정규화는 (x − 1.0) / (0 + 0.01)이라 벽이 1 m에 보이면 −50이 들어간다. 그 입력에 붙은 첫 층 가중치는 학습 동안 입력이 늘 0이라 기울기가 없어 초기값 그대로다. 그래서 벽이 보이면 정책 출력은 무작위로 커진다(무작위 라이다 입력에서 행동 크기 최대 126). 매장 정책은 장애물을 보며 학습하므로 이 문제는 거기서 사라지지만, **같은 문제가 매장에서 한 번도 안 본 입력(예: 다른 로봇)으로 다시 올 수 있다.** 학습 데이터의 관측 분포(정규화 평균·표준편차)를 정책과 같이 확인하는 습관이 필요하다.
- rsl_rl의 ONNX 내보내기는 관측 정규화를 그래프 안에 넣지만 행동 자르기(`clip_actions`)는 넣지 않는다(env 래퍼가 한다). 배포 쪽이 [-1, 1] 자르기를 해야 한다. `task_spec.action_to_target`도 자르므로 C++도 같게 했다.
- Nav2 `ControllerSelector`는 transient_local + reliable로 구독한다. `ros2 topic pub --once`(volatile, 곧 끝남)로는 닿지 않을 수 있어 QoS를 맞추고 계속 내보냈다.
- ORT를 sudo 없이 쓰려면 `ros-jazzy-onnxruntime-vendor` .deb를 풀어 `CMAKE_PREFIX_PATH`에 두면 된다. 그 패키지의 cmake extras가 자기 위치에서 상대 경로로 라이브러리를 찾고 RPATH를 넣어 주므로 `LD_LIBRARY_PATH`도 필요 없었다.

## 6. Pi4 ONNX Runtime 조사 (책상 조사만, 실측 없음)

2026-11-21 조사. "확인함"은 직접 본 것, "미확인"은 하위 조사가 가져왔지만 확인하지 않은 것이다.

확인함:
- `ros-controls/onnxruntime_vendor` master의 CMakeLists: ORT_VERSION 1.24.3. aarch64에서는 `onnxruntime-linux-aarch64-1.24.3.tgz`를 고르고, 빌드할 때 GitHub releases에서 받는다(FetchContent).
- Jazzy apt: `ros-jazzy-onnxruntime-vendor` 0.1.1이 amd64(WSL apt-cache, "Vendor package for ONNX Runtime 1.24.3", libc6 >= 2.34 필요)와 arm64(packages.ros.org noble binary-arm64 색인, 0.1.1-1noble.20261026) 모두 있다. Ubuntu 24.04의 glibc는 2.39다. → Pi에서는 `apt install ros-jazzy-onnxruntime-vendor`로 되고 빌드 때 내려받기가 없다.
- 이 작업에서 amd64 .deb로 빌드·실행까지 확인했다(위 3.1). arm64에서는 아직 안 해 봤다.
- 조사 때 github.com/microsoft/onnxruntime에 닿지 못해(HTTP 404) 릴리스 파일 목록은 확인하지 못했다.

미확인(하위 조사):
- CMake는 `find_package(onnxruntime_vendor)`, 대상 `onnxruntime::onnxruntime`, package.xml에 depend. (이 부분은 이번 빌드에서 그대로 확인됨)
- 스레드: intra 1, inter 1, 순차, 20 Hz 호출이면 `allow_spinning=0`(ORT 스레딩 문서).
- CPU EP, 그래프 최적화 ORT_ENABLE_ALL.
- 내보내기: `torch.onnx.export(dynamo=False, opset 17/18, 배치 1, 동적 축 없음)`. 새 torch는 기본 내보내기가 dynamo로 바뀌었으니 명시해 고정한다. (이번 정책은 rsl_rl이 opset 18, 입력 [1, 41]로 내보냈다)
- 패리티: `onnx.checker`, ORT python 대 torch allclose atol 1e-5, python ORT를 1.24.3에 맞추기, 관측 정규화와 출력 자르기가 그래프 밖에 있는지 확인. (이번에는 python ORT 1.24.4였고 C++ 1.24.3과 비트까지 같았다)
- 대안: Eigen 3 행렬곱으로 2×256 MLP(곱셈 약 9만 번)를 직접 계산하면 수십 µs로 추정되고 60 MB짜리 .so가 필요 없다. 견줘 볼 만하다.
- ONNX를 감싸는 기존 Nav2 controller 플러그인은 찾지 못했다.

## 7. 알아야 할 포인트 (계획 개념 E: ARM64의 ONNX와 ONNX Runtime, Nav2 controller 플러그인 구조)

**무엇을 왜 골랐나.**
- ONNX: 신경망을 "계산 그래프 + 가중치" 파일 하나로 적는 표준 형식이다. 학습은 PyTorch(GPU, 파이썬)에서 하고 로봇에서는 파이썬·PyTorch 없이 C++로 돌리고 싶어서 쓴다. 대안: libtorch(TorchScript, 수백 MB, ARM 빌드가 무겁다), 가중치를 꺼내 Eigen으로 직접 곱하기(가장 가볍지만 층 구조가 바뀔 때마다 C++를 고친다), TFLite(PyTorch에서 바로 안 나온다).
- ONNX Runtime: ONNX 파일을 읽어 CPU(또는 다른 가속기, "실행 공급자 EP")에서 돌리는 엔진. ROS Jazzy apt에 amd64·arm64 모두 같은 1.24.3이 있어 WSL과 Pi에서 같은 코드가 돈다. 작은 MLP에는 과하지만, 매장 정책이 커지거나(합성곱, 순환층) 모양이 바뀌어도 C++를 고치지 않는다.
- Nav2 controller 플러그인: controller_server가 pluginlib으로 `nav2_core::Controller`를 상속한 클래스를 이름으로 불러 20 Hz로 `computeVelocityCommands(pose, velocity, goal_checker)`를 부른다. BT의 `ControllerSelector`가 `controller_selector` 토픽으로 어떤 controller를 쓸지 바꾼다. 그래서 RPP·MPPI·RL을 같은 BT, 같은 경로, 같은 goal checker로 견줄 수 있다. 대안(별도 노드가 cmd_vel을 바로 냄)은 Nav2의 진행 감시, 취소, 복구, goal 판정을 다시 만들어야 한다.

**코드 세 곳.**
1. `pode/pode_rl_controller/src/observation.cpp` `scan_to_rays`, `build_observation`: 학습 정의를 C++로 옮긴 곳. 학습 쪽 정의가 바뀌면 여기와 fixture가 같이 바뀌어야 한다.
2. `pode/pode_rl_controller/src/onnx_policy.cpp` 생성자: ORT 세션 설정(스레드, spinning, 최적화)과 입력·출력 모양 확인.
3. `pode/pode_rl_controller/src/follow_path_rl.cpp` `computeVelocityCommands`: Nav2가 매 주기 부르는 곳. 목표점, 끝 회전, 정책, 가속 제한이 이 순서로 있다.

**숫자 읽는 법.**
- 패리티 "max |diff|": 같은 입력에 두 구현이 낸 값의 가장 큰 차이. float32 한 칸은 1 근처에서 1.2e-7이라 관측의 5e-8은 "반올림만 다르다"는 뜻이다. 추론 1.5e-5는 행동 크기 34에서 난 것이라 상대로는 4.5e-7이다.
- 로그 한 줄 `20 calls, infer mean 50 max 83 us, call mean 116 max 202 us | policy goal 0.481 m at -0.09 rad ... act (1.31, -0.25) cmd v 0.250 w -0.316`: 지난 1 s 동안 20번 불렸고, 목표까지 0.48 m, 거의 정면(−0.09 rad), 정책 행동 v 쪽 1.31은 자르면 1.0 → v_max 0.25 m/s다. ω가 −1.0이 아니라 −0.316인 것은 가속 제한(한 주기 0.1 rad/s) 때문에 목표를 따라가는 중이어서다.
- `act`가 ±1을 넘으면(예: −2.24) 정책이 자르기 경계 밖을 내고 있다는 뜻이다. 빈 바닥(라이다 모두 2.0)에서도 −2.24가 나왔으니 이 정책은 원래 경계 밖을 내고 자르기에 기대어 "최대로"를 표현한다. 라이다 입력이 학습 범위를 벗어나면 이 값이 수십~백으로 커진다(5절).

**스스로 점검.**
1. Gazebo 스캔의 0번 빔은 로봇의 어느 쪽을 가리키고, 학습 광선 0번(−180°)은 어느 쪽인가? 그 둘을 잇는 데 TF yaw가 왜 필요한가?
2. 빈 바닥 정책을 v7 경기장 안에서 그대로 돌리면 왜 엉뚱하게 움직일 수 있는가? ONNX 그래프의 어느 연산이 그 원인을 키우는가?
3. FollowPathRL은 goal checker의 xy 허용치 안에서 정책을 끄고 제자리 회전한다. 정책을 끝까지 쓰면 Nav2 goal이 왜 안 끝날 수 있나?

## 8. 직접 해 볼 것

`goal_rule`을 `lookahead`로 바꿔 같은 goal을 다시 몰아 보고, 로그의 `goal ... m`이 경로 끝 거리(`end`)와 어떻게 다른지 본다. 덮어쓰기 파일의 `goal_rule: "path_end"`를 `"lookahead"`로 고치고(`lookahead_dist: 0.4`) 3.2의 명령을 그대로 쓴다. 빈 바닥 정책은 1~2 m 목표로 배웠으니 0.4 m 앞 점을 쫓으면 늘 "가까운 목표"를 보게 된다. 속도(`cmd v`)가 경로 끝 규칙 때와 어떻게 달라지는지 적어 둔다. 그 차이가 매장 정책을 앞보기 점으로 학습해야 하는 까닭이다.

## 9. 남은 일

- 플러그인 단위 시험(가짜 TF·스캔·경로로 `computeVelocityCommands`의 연결을 보는 것). 지금 패리티는 관측·행동·추론 함수만 본다.
- 충돌 검사 없음. 매장 정책을 올릴 때 local costmap으로 앞을 보는 안전장치(RPP의 `use_collision_detection`과 같은 것)를 더할지 정한다.
- 경기장 안에서 빈 바닥 정책을 돌린 기록은 없다(5절 때문에 의미가 적다).
- Pi4(arm64)에서 빌드·추론 시간 재기. Eigen 직접 계산과 비교.
- 매장 정책을 `export_policy.py`로 내보내 `policies/<이름>/`에 넣고 RPP, MPPI와 같은 goal로 견주기.
