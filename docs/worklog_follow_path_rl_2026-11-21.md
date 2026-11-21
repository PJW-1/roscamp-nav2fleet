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
2. 경로를 TF로 `base_footprint` 틀로 옮겨 목표점을 정한다(`goal_rule`: 경로 끝, 또는 로봇에서 가장 가까운 경로 점부터 앞으로 가며 로봇에서 직선거리가 `lookahead_dist` 이상인 첫 점. 그런 점이 없으면 경로 끝).
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
| 4 | 목표점 규칙 | 경로 끝, 경로 위 앞보기 점 | ~~파라미터 `goal_rule`, 이번 기본 `path_end`, `lookahead_dist` 0.4 m~~ → 결정 14로 바뀜(규칙은 정책 명세에서 읽는다) | 빈 바닥 정책은 1~2 m 떨어진 목표점으로 학습했다. 매장 정책은 경로 위 0.40 m 앞 점으로 학습했다 |
| 5 | 끝 방향 맞추기 | (a) goal checker 방향 허용치를 덮어써 끈다 (b) 플러그인이 마지막에 제자리 회전 | (b) goal checker xy 허용치 안에서 ω = `rotate_gain`(1.5) × 방향 오차 | 정책은 위치만 배웠고 BT는 `general_goal_checker`(방향 0.12 rad)를 쓴다. (a)는 되돌린 FollowPathGoal에도 영향을 준다. 허용치는 goal checker에서 읽으므로 2 cm checker에도 맞다 |
| 6 | ORT 실행 설정 | CPU / XNNPACK·ACL EP, 스레드 수, 순차·병렬, spinning | **CPU EP, intra_op 1(파라미터 `intra_op_threads`), inter_op 1, 순차, spinning 끔, 그래프 최적화 전부** | 41-128-128-2 MLP는 한 번에 수십 µs라 스레드를 늘려도 이득이 없고, 20 Hz 호출 사이에 도는(spin) 스레드는 Pi4 CPU만 먹는다 |
| 7 | 정책 파일 커밋 | (a) `*.onnx` 무시 유지, 내보내기 스크립트로 만듦 (b) 예외로 커밋 | (b) `policies/*/policy.onnx`만 예외(99 KB) | 플러그인 기본값과 추론 패리티 시험이 그 파일을 연다. 새 체크아웃·Pi에서 바로 돈다. 저장소 규칙(모델은 커밋 안 함)을 좁게 푼 것이라 사용자 확인 필요 |
| 8 | 가속 제한의 dt | 실제 호출 간격, 학습의 0.05 s | 0.05 s(명세 `control_dt`) | 학습과 같다. controller_frequency가 20 Hz라 같다 |
| 9 | 가속 제한의 시작 상태 | 0, 측정 속도 | 첫 호출이나 2 주기 넘게 쉬었다가 다시 부를 때 odom 측정 속도 | 학습은 정지에서 0으로 시작했다. 움직이는 중에 controller를 바꾸면 0으로 시작하면 급정거한다 |
| 10 | 속도 입력 | 명령, 측정 | 측정(controller_server가 주는 odom 속도) | 학습 관측이 몸체 속도(`root_link_lin_vel_b`)다 |
| 11 | 스캔 받기 | costmap, 스캔 직접 | 부모 노드로 `scan` 직접(SensorDataQoS) | 관측은 원래 거리값이다. costmap은 0.025 m 칸과 inflation으로 바뀐다 |
| 12 | 추론 비교 허용치 | 절대만, 절대+상대 | atol 1e-5 + rtol 1e-5(numpy.allclose) | 아래 3.1. 학습 범위 밖 입력에서 행동이 100을 넘어 절대 허용치만으로는 float32 반올림을 못 덮는다 |
| 13 | 매장 정책 ONNX를 만드는 곳 | (a) Isaac에서 rsl_rl 내보내기(play, GPU 잠금 필요) (b) 체크포인트에서 `export_policy.py`가 직접 `torch.onnx.export` | (b) 정규화 + ELU MLP를 같은 모양(opset 18, 입력 `obs` [1, 41], 출력 `actions`, 자르기 없음)으로 만든다. `policy_spec.json` `source.onnx_from`에 적힌다 | Isaac과 잠금이 필요 없다. 같은 그래프인지는 PyTorch 대 ORT python 2.86e-6, ORT python 대 ORT C++ 0(비트까지 같음)으로 본다. 대가: rsl_rl 쪽 내보내기 코드가 바뀌어도 따라가지 않는다(정책 모양이 바뀌면 `CheckpointPolicy`도 고친다) |
| 14 | 목표점 규칙의 출처 | (a) 덮어쓰기 YAML의 `goal_rule`, `lookahead_dist` (b) 정책 명세(`goal.rule`, `lookahead_m`, `follow_window`), 파라미터는 덮어쓰기만 | (b) 명세 형식 2. 파라미터 `goal_rule`, `lookahead_dist`를 적으면 명세를 덮고 **WARN 로그**(`... OVERRIDES the policy spec ...`)를 남긴다. 덮어쓰기 YAML에서는 두 줄을 뺐다 | YAML이 틀리면 정책이 배우지 않은 목표를 보면서도 조용히 돈다. 명세는 학습 쪽에서 만들어지므로 정책과 함께 다닌다. 덮어쓰기는 시험(예: 규칙을 바꿔 보기)에 싸게 남겼다. 형식 1 명세는 거부한다(다시 내보내라는 메시지) |
| 15 | 앞보기 점 정의 | (a) 이전 뼈대: 가장 가까운 점부터 로봇과의 직선거리가 0.4 m 이상인 첫 점 (b) 학습의 `route_spec.follow` 그대로 | (b) 진행 번호는 [진행, 진행 + 20점] 안에서 가장 가까운 점으로만 앞으로 가고(뒤로 안 감), 앞보기 점은 진행 점에서 **경로 길이로** 0.40 m 이상 떨어진 첫 점(없으면 끝점) | 정책이 본 목표와 같아야 한다. 굽은 경로에서 (a)는 직선거리라 경로를 질러 더 먼 점을 고른다. fixture로 학습 코드와 견준다(아래 10.2) |
| 16 | 경로를 따라가는 틀 | (a) 매 주기 경로를 `base_footprint`로 옮겨 계산 (b) 경로 틀(map) 그대로, 로봇 위치를 TF로 그 틀에서 읽음 | (b) | 학습도 map 틀에서 계산한다. (a)는 로봇 자세에 따라 누적 길이가 반올림만큼 바뀌어, 고르게 놓인 경로에서 앞보기 점이 이웃 점과 번갈아 바뀔 수 있다(Codex 검토 지적, 고침) |
| 17 | 새 경로를 받을 때 진행 번호 | (a) 0으로 되돌림 (b) 새 경로에 로봇을 투영해 이어감 | (a) `setPlan`과 `reset`에서 0 | BT가 1 Hz로 다시 계획하고 NavFn 경로는 로봇 자리에서 시작하므로 0 근처가 맞다. 학습도 에피소드 시작에 0이다. 대가: 경로 시작이 로봇보다 20점(0.5 m) 넘게 뒤에 있으면 진행이 한 주기 늦게 따라간다 |
| 18 | `follow_window` 단위 | 점 개수(학습 그대로), 미터 | 점 개수 20(명세) | 학습 경로는 0.025 m 칸이라 약 0.5 m. 실물 global costmap도 0.025 m라 같다. 시뮬 지도 해상도가 다르면 미터로는 달라진다(10.3에서 시뮬 경로 점 간격을 적었다) |
| 19 | 정책마다 fixture | 한 폴더, 정책 폴더마다 | `test/fixtures/<정책>/`, 패리티 시험은 정책마다 한 번씩 빌드(`test_parity_<정책>`) | 관측·행동 fixture는 정책과 무관하지만 추론 fixture와 앞보기 fixture는 정책마다 다르다. 정책을 더하면 CMake 목록에 이름 하나만 더한다 |
| 20 | 플러그인 기본 정책 | `goal_reach_base_4096`, `store_arena_4b` | `store_arena_4b`(플러그인 기본값과 덮어쓰기 YAML 모두) | 빈 바닥 정책은 경기장 안에서 관측이 학습 범위를 벗어난다(5절). 배포할 것은 매장 정책이다 |

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

재현 순서. 모든 터미널에 `export ROS_DOMAIN_ID=62 GZ_PARTITION=rl_plugin_62 GZ_IP=127.0.0.1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`을 두고, 무거운 실행 잠금을 잡은 뒤 한다.

1. 확인용 덮어쓰기 파일 `/tmp/rl_check.yaml`을 만든다: `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml`을 그대로 복사하고 끝에 아래를 더한다.
   ```yaml
   amcl:
     ros__parameters:
       update_min_d: 100.0
       update_min_a: 100.0
   ```
2. 터미널 A(계속 돈다):
   ```bash
   ros2 launch fleet_gz_sim one_pode.launch.py gui:=false rviz:=false fsm:=false keepout:=false \
     scan_filter:=false start:=P-3 nav_log_level:=info nav_overrides:=/tmp/rl_check.yaml
   ```
3. 터미널 B, 로그에 `Managed nodes are active`가 나온 뒤 로봇을 경기장 밖으로 옮긴다:
   ```bash
   gz service -s /world/arena/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
     --req 'name: "pode1", position: {x: 0.64, y: -4.0, z: 0.02}, orientation: {z: 0.7071068, w: 0.7071068}'
   ```
4. 터미널 C(계속 돈다, goal 동안 켜 둔다): 
   ```bash
   ros2 topic pub --qos-durability transient_local --qos-reliability reliable -r 2 \
     /pode1/controller_selector std_msgs/msg/String "{data: FollowPathRL}"
   ```
5. 터미널 B:
   ```bash
   ros2 action send_goal /pode1/navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: 1.40, y: 0.20}, orientation: {w: 1.0}}}}"
   ```
6. 되돌리기: 터미널 C를 Ctrl-C로 끄고 같은 명령을 `{data: FollowPathGoal}`로 다시 띄운 뒤, 터미널 B에서 goal `{x: 0.90, y: 0.20}`, 방향 `{z: 1.0, w: 0.0}`을 보낸다. 끝나면 모든 터미널을 끄고 잠금을 푼다.

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
- 로그 한 줄 `20 calls, infer mean 50 max 83 us, call mean 116 max 202 us | policy goal 0.481 m at -0.09 rad ... act (1.31, -0.25) cmd v 0.250 w -0.316`: 지난 1 s 동안 20번 불렸고, 목표까지 0.48 m, 거의 정면(−0.09 rad), 정책 행동 v 쪽 1.31은 자르면 1.0 → v_max 0.25 m/s다. ω 쪽 −0.25는 목표 ω −0.25 rad/s이고, 명령이 −0.316인 것은 1 s 전의 −1.0에서 가속 제한(한 주기 0.1 rad/s)으로 −0.25 쪽으로 다가가는 중이어서다.
- `act`가 ±1을 넘으면(예: −2.24) 정책이 자르기 경계 밖을 내고 있다는 뜻이다. 빈 바닥(라이다 모두 2.0)에서도 −2.24가 나왔으니 이 정책은 원래 경계 밖을 내고 자르기에 기대어 "최대로"를 표현한다. 라이다 입력이 학습 범위를 벗어나면 이 값이 수십~백으로 커진다(5절).

**스스로 점검.**
1. Gazebo 스캔의 0번 빔은 로봇의 어느 쪽을 가리키고, 학습 광선 0번(−180°)은 어느 쪽인가? 그 둘을 잇는 데 TF yaw가 왜 필요한가?
2. 빈 바닥 정책을 v7 경기장 안에서 그대로 돌리면 왜 엉뚱하게 움직일 수 있는가? ONNX 그래프의 어느 연산이 그 원인을 키우는가?
3. FollowPathRL은 goal checker의 xy 허용치 안에서 정책을 끄고 제자리 회전한다. 정책을 끝까지 쓰면 Nav2 goal이 왜 안 끝날 수 있나?

## 8. 직접 해 볼 것

`goal_rule`을 `lookahead`로 바꿔 같은 goal을 다시 몰아 보고, 로그의 `goal ... m`이 경로 끝 거리(`end`)와 어떻게 다른지 본다. 덮어쓰기 파일의 `goal_rule: "path_end"`를 `"lookahead"`로 고치고(`lookahead_dist: 0.4`, 직선거리 기준이라 굽은 경로에서는 경로를 따라 0.4 m 앞선 점과 다를 수 있다) 3.2의 재현 순서를 그대로 따른다. 빈 바닥 정책은 1~2 m 목표로 배웠으니 0.4 m 앞 점을 쫓으면 늘 "가까운 목표"를 보게 된다. 속도(`cmd v`)가 경로 끝 규칙 때와 어떻게 달라지는지 적어 둔다. 그 차이가 매장 정책을 앞보기 점으로 학습해야 하는 까닭이다.

## 9. 남은 일

- 플러그인 단위 시험(가짜 TF·스캔·경로로 `computeVelocityCommands`의 연결을 보는 것). 지금 패리티는 관측·행동·추론 함수만 본다.
- 충돌 검사 없음. 매장 정책을 올릴 때 local costmap으로 앞을 보는 안전장치(RPP의 `use_collision_detection`과 같은 것)를 더할지 정한다.
- 경기장 안에서 빈 바닥 정책을 돌린 기록은 없다(5절 때문에 의미가 적다).
- Pi4(arm64)에서 빌드·추론 시간 재기. Eigen 직접 계산과 비교.
- ~~매장 정책을 `export_policy.py`로 내보내 `policies/<이름>/`에 넣기~~ 끝(10절). RPP, MPPI와 같은 goal로 견주기는 10.5.

## 10. 매장 정책 4b를 플러그인에 올리기 (2026-11-21 밤)

매장 경기장 4b 정책(사람·카트가 움직이는 경기장에서 학습, `D:/isaac/runs/store_arena/ppo_g/c1/model_499.pt`, 평가 성공 94.6 %)을 `policies/store_arena_4b/`로 내보내고, 학습 때 본 목표(경로 위 0.40 m 앞 점)를 플러그인이 똑같이 만들게 했다. Gazebo one_pode(도메인 62)에서 HALL goal 셋을 세 번씩, 그리고 nav_bench e_corridor를 두 번 몰았다.

**한 줄 결론:** 22판 모두 Nav2 SUCCEEDED, 실제 위치가 goal에서 0.10 m 안이다(RL 11판, RPP 11판). RL은 O_IN → M_E를 평균 6.3 s(RPP 9.9 s)에 갔고 제자리 돌기나 좌우 흔들림은 보이지 않았다. **단, Q2 → E-3 세 판 모두 RL 몸이 남쪽 벽이나 칸막이 2 끝에 0~3 mm까지 붙었다**(nav_bench 판정으로 충돌 1, 2, 1번. RPP는 같은 goal에서 0.065 m). 한 번 추론은 Gazebo 안에서 1 s 평균의 가운데값 50 µs다.

### 10.1 학습 설정이 goal_reach와 같은지

학습 commit은 `feat/store-arena-2026-11-21` d2abc2c다(체크포인트 `model_0.pt`가 18:53, 그 전 마지막 commit이 18:42 d2abc2c, 다음 commit은 19:30). 그 commit과 이 브랜치를 견주었다.

| 항목 | 같은가 | 근거 |
|---|---|---|
| `task_spec.py`(관측 배치·축척, 라이다 360광선·36구역·0.05~2.0 m, 행동 한계 v −0.05~0.25, ω ±1.0, 가속 2.0, 주기 0.05 s) | 같다 | 차이는 `parse_overrides`(문자열 필드 받기) 하나. 관측·행동 함수는 글자까지 같다 |
| 라이다 설정(`goal_reach_env.make_lidar_cfg`) | 같다 | `goal_reach_env.py` 차이는 주석과 시작점 고르기(`sample_start`)뿐 |
| 관측 | 같다 | `store_arena_env._get_observations`가 `ts.build_observation(lidar, la_dist, la_ang, body_velocity, spec)`를 부른다. `--set` 덮어쓰기 없음(`run.json` `spec_overrides: {}`) |
| 정책 망과 행동 자르기 | 같다 | `agent_cfg`: ELU 128-128, 관측 정규화, `clip_actions` 1.0 |
| 정규화 통계 | **다르다(좋은 쪽)** | 4b: 라이다 구역 std 0.095~0.150(0인 칸 없음), `goal_dist` 평균 0.118(0.35 m) std 0.032(0.096 m). goal_reach: 라이다 std 0(5절의 문제). 그래서 4b는 벽이 보이는 관측에서 행동 크기가 최대 10.7(추론 fixture 32개)로 goal_reach의 126보다 작다 |
| 목표 | **다르다** | 경로 위 앞보기 점(`ArenaSpec.lookahead_m` 0.40, `follow_window` 20). 명세 `goal`에 적었다(결정 14, 15) |

### 10.2 만든 것과 시험

| 파일 | 바뀐 것 |
|---|---|
| `learning/goal_reach/export_policy.py` | `--onnx` 없이 체크포인트에서 ONNX를 만든다(결정 13). `--goal-rule lookahead --lookahead-m --follow-window --route-spec --source-branch --source-commit`. 명세 형식 2: `goal.rule`, `lookahead_m`, `follow_window`, `source.checkpoint_path`, `onnx_from`, `branch`, `commit`. `follow_cases.json`: 학습 commit의 `route_spec.py`(`git show d2abc2c:learning/store_arena/route_spec.py`로 꺼낸 파일)의 `follow`와 `task_spec`으로 만든다 |
| `pode/pode_rl_controller/src/observation.cpp` | `follow_progress`, `lookahead_index`, `goal_in_robot_frame`, `path_goal`(학습 정의를 옮김) |
| `pode/pode_rl_controller/src/follow_path_rl.cpp` | 규칙을 명세에서 읽고, 파라미터는 덮어쓰기와 WARN만. 경로를 경로 틀(map)에 두고 로봇 자세를 TF로 그 틀에서 읽는다(결정 16). 진행 번호는 `setPlan`, `reset`에서 0 |
| `pode/pode_rl_controller/policies/store_arena_4b/` | `policy.onnx`(88 KB), `policy_spec.json` |
| `test/fixtures/<정책>/`, `test/test_observation.cpp` | 정책마다 fixture(결정 19). 새 시험 9개 |
| `sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml` | `policy: store_arena_4b`, `goal_rule`·`lookahead_dist` 줄을 뺌(시험도 그렇게 바꿈) |

시험(WSL, `colcon test --packages-select pode_rl_controller`): `Summary: 24 tests, 0 errors, 0 failures, 0 skipped`(gtest 21개 + ctest 3개).

```
== test_parity_store_arena_4b
[parity] rays max |diff| 5.91e-08, obs max |diff| 4.47e-08
[parity] action -> (v, w) max |diff| 3.58e-08
[parity] onnxruntime C++ vs torch max |diff| 2.86e-06 (bound margin 9.33e-06), vs onnxruntime python 0
[parity] inference 6.8 us per call (mean of 2000, 1 thread)
== test_observation
[follow] 81 steps, lookahead point max |diff| 0 m
[follow] path_goal over 81 steps, goal max |diff| 5.55e-17
[goal] polar max |diff| 5.55e-17, goal and velocity obs max |diff| 2.96e-08
```

- 허용치는 그대로다: 관측 1e-6, 추론 atol 1e-5 + rtol 1e-5, 기하 1e-9. 늦춘 것 없음.
- 앞보기 fixture 네 경로: 곧은 2 m(0.025 m 간격), 0.40 m보다 짧은 0.2 m, ㄱ자(1 m 뒤 0.6 m 꺾임), 0.05 m 간격. 각 경로를 무작위로 돌리고 옮긴 뒤 점마다 ±2 mm 흔들었다(고르게 놓인 점에서 앞보기 거리가 정확히 점에 걸리는 반올림 경계를 피하려고). 로봇은 경로를 따라가며 옆으로 ±3 cm 벗어나고, 한 번은 창(20점)보다 멀리 뛰고, 끝에서 세 번 선다.
- 손으로 쓴 시험: 짧은 경로와 경로 끝에서는 끝점, 진행은 창만큼만 앞으로 가고 뒤로 안 감, `path_end` 규칙은 끝점, yaw π 라이다: 0번 빔(라이다 뒤 = 로봇 정면)이 광선 180, 160번 빔이 광선 270(왼쪽), 320번 빔(라이다 정면 = 로봇 뒤)이 광선 0.
- `goal_reach_base_4096`도 형식 2로 다시 내보냈다(`rule: path_end`). `policy.onnx`와 fixture 셋은 바이트까지 그대로다.
- `sim/fleet_gz_sim/test/test_sim_nav_params.py` 36개 통과(WSL), `learning/goal_reach` 9개 통과, ruff check 통과.

Codex 검토(df4c2e3..ed79583) [확정] 둘: (1) 매 주기 경로를 base 틀로 옮겨 누적 길이를 다시 재면 반올림으로 앞보기 점이 이웃 점과 번갈아 바뀔 수 있다 → 맞다. 경로 틀에서 계산하게 고쳤다(결정 16). (2) 새 시험이 빈 fixture에서도 통과한다 → 맞다. 걸음 수가 40을 넘는지 본다. 시험을 먼저 쓰고(빌드 실패 확인) 고친 63b8107의 재검토는 "결함 없음".

### 10.3 Gazebo

설정: one_pode(`gui:=false rviz:=false fsm:=false start:=O_IN nav_log_level:=info nav_overrides:=sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml`), 도메인 62, LOCALHOST. 판을 모는 것은 MPPI 브랜치의 `tools/nav_bench/bench_node.py`를 그 worktree에서 읽기만 하며 돌렸다(10.5). 설정 파일은 그쪽 `scenarios.yaml`을 복사해 controller를 `[FollowPathGoal, FollowPathRL]`, 판 수 3(e_corridor 2), 판 시간 상한 60 s, 출발·goal만 바꿨다. 판마다 RL과 RPP를 번갈아(ABBA) 돌고, 판 사이에 RPP로 출발점에 돌아간다. 성공 = Nav2 SUCCEEDED이면서 `/pode1/sim/truth`가 goal에서 0.10 m 안. 기록 `~/rl_gz_check/4b_215805/`(WSL). 22:24 잠금을 잡고 22:33에 풀었다(끝난 뒤 남은 Gazebo·Nav2 프로세스 0개, 여유 메모리 15.3 GB 그대로).

controller_server 로그: `FollowPathRL FollowPathRL: policy .../policies/store_arena_4b (obs 41, dt 0.050 s, v -0.05..0.25 m/s, w 1.00 rad/s), goal_rule lookahead (lookahead 0.40 m, window 20 points)`, `lidar rplidar_link in base_footprint at (-0.017, 0.000) m, yaw 3.142 rad`. `OVERRIDES` 경고 0줄.

| goal | controller | 성공 | 시간 s(판마다) | 끝 오차 m | 라이다 최소 m | 몸 최소 m | 충돌 | ω 부호 바뀜 | 제자리 회전 s |
|---|---|---|---|---|---|---|---|---|---|
| O_IN → M_E (1.645, 0.55) | RL | 3/3 | 6.2, 6.5, 6.1 | 0.033~0.051 | 0.073 | 0.072 | 0 | 10, 5, 6 | 0.0~0.1 |
| | RPP | 3/3 | 9.2, 10.7, 9.9 | 0.035~0.053 | 0.097 | 0.100 | 0 | 11, 11, 7 | 2.4~3.4 |
| M_E → Q2 (1.66, 0.15) | RL | 3/3 | 6.5, 6.5, 6.7 | 0.021~0.027 | 0.078 | 0.074 | 0 | 1, 1, 1 | 3.1~3.2 |
| | RPP | 3/3 | 6.4, 6.0, 8.3 | 0.052~0.055 | 0.085 | 0.084 | 0 | 3, 1, 3 | 3.2~4.6 |
| Q2 → E-3 (0.64, 0.24, 방향 90°) | RL | 3/3 | 8.0, 11.4, 8.4 | 0.029~0.079 | 0.050 | **0.000** | **1, 2, 1** | 5, 4, 3 | 0.0~1.6 |
| | RPP | 3/3 | 11.2, 10.9, 10.3 | 0.034~0.043 | 0.069 | 0.065 | 0 | 13, 13, 3 | 3.8~4.7 |
| e_corridor(O_IN → M_E, 진열대 A 밑 상자와 오가는 카트) | RL | 2/2 | 6.1, 6.1 | 0.036~0.039 | 0.050 | 0.048 | 0 | 5, 5 | 0.0 |
| | RPP | 2/2 | 8.8, 9.3 | 0.037~0.049 | 0.074 | 0.082 | 0 | 6, 8 | 2.3~2.4 |

본 것:
- **Q2 → E-3에서 벽에 붙는다.** 세 판 중 두 판은 출발 2.2 s에 (1.60, 0.085), 방향 1.93 rad에서 남쪽 벽(y 0)과 몸 사이가 0.000, 0.002 m였다. Q2에서 동쪽을 보고 서서 서쪽 goal을 받으면, 정책은 첫 명령부터 뒤로 빼며(로그 첫 줄 `act (-1.75, -0.52) cmd v -0.050`) 왼쪽으로 돈다. 그사이 몸 뒤가 벽 쪽(0.15 m 남쪽)으로 간다. 남은 한 판은 E-3 앞에서 칸막이 2 끝과 0.003 m였다. 학습에서는 접촉 힘 1 N 넘게 닿아야 충돌이고, nav_bench는 몸과 면의 실제 거리 0.005 m 이하를 충돌로 센다(스치기도 센다). 그래도 실물에서는 벽을 긁을 수 있다. 시뮬 라이다 최소 0.050 m(측정 하한)도 같은 판에서 나왔다.
- **제자리 돌기와 흔들림은 없었다.** RL은 출발할 때 서서 돌지 않고 돌면서 나간다(O_IN → M_E 제자리 회전 0~0.1 s, RPP 2.4~3.4 s). M_E → Q2의 3.1 s는 끝에서 goal 방향 0°로 맞추는 플러그인의 회전(결정 5)이다. ω 부호 바뀜은 RL 1~10번으로 RPP(1~13번)보다 적다.
- **시간 비교는 공정하지 않다.** RL은 v 상한 0.25 m/s(명세)로 달리고, RPP는 시뮬 덮어쓰기로 0.20 m/s다. 달린 구간 평균 속도는 RL 0.10~0.18, RPP 0.14~0.18 m/s로 비슷하고, 시간 차이의 대부분은 RPP의 출발 제자리 돌기다. RL 길은 RPP보다 0.07~0.33 m 길다(돌면서 나가 부푼다).
- RL은 진열대 A에 0.072 m까지 붙어 간다(RPP 0.100 m). 학습한 경로가 inflation 0.15 m 비용으로 짠 길인데도 정책은 경로 위 0.40 m 앞 점을 쫓아 모서리를 조금 자른다.
- 시뮬 NavFn 경로 점 간격은 0.0125 m였다(판마다 처음 세 경로의 가운데값). 학습 경로는 0.025 m라 `follow_window` 20점은 시뮬에서 0.25 m다(학습 0.5 m). 앞보기 0.40 m는 경로 길이로 재므로 영향이 없고, 창은 한 주기(5 cm 미만 이동)에 넉넉하다.
- 판마다 RPP로 출발점에 돌아온 뒤 nav_bench가 Gazebo에서 출발점으로 옮겼다(goal 하나에 4~6번). 돌아온 자리가 nav_bench의 출발 허용치(0.03 m, 5°)보다 넓은 goal checker(0.05 m)로 끝나서다. 판은 모두 정확한 출발점에서 시작했다.

### 10.4 추론 시간

controller_server 1 s 로그에서 정책을 20번 가까이 부른 59개 구간: 1 s 평균의 p10 / p50 / p90 = 46 / 50 / 55 µs, 1 s 최대의 가운데값 67 µs, 가장 큰 값 167 µs. 한 주기 전체(TF, 관측, 추론) 평균 가운데값 102 µs. goal을 받은 첫 호출은 1592 µs(한 번, 첫 호출만 느리다). gtest의 이어 부르기 평균은 6.8 µs다(3.3절과 같은 까닭으로 띄엄띄엄 부르면 느리다).

### 10.5 nav_bench 다시 쓰기와 RL·RPP·MPPI 비교에 필요한 것

- **nav_bench는 고치지 않고 쓸 수 있었다.** `bench_node.py`는 `--config`로 아무 설정 파일이나 받고 controller 이름, 출발, goal이 설정에 있다. 이 브랜치의 install을 source하고 MPPI worktree의 `tools/nav_bench/bench_node.py`를 `PYTHONDONTWRITEBYTECODE=1`로 돌렸다(그쪽 파일을 바꾸지 않음). `run_bench.sh`는 `--config "$HERE/scenarios.yaml"`이 박혀 있고 그 설정의 controller가 `[FollowPathGoal, FollowPathMPPI]`라, 이 브랜치 install(MPPI 없음)로는 그대로 못 쓴다. 그래서 launch와 잠금은 내 스크립트가 하고 bench_node만 불렀다. 지표는 `metrics.leg_metrics`를 그대로 불렀다.
- **세 controller를 한 판에 견주려면** 두 브랜치를 합쳐야 한다(`feat/follow-path-rl-2026-11-21` + `feat/mppi-controller-2026-11-21`). `git merge-tree`로 보면 글자 충돌은 없다. 하지만 합친 뒤 `nav2_params.yaml`의 `controller_plugins`에 `FollowPathMPPI`가 들어가고, `follow_path_rl.yaml`은 예전 넷 + FollowPathRL을 적고 있어서, RL을 띄우면 MPPI가 빠지고 `test_sim_nav_params.py`의 `test_the_rl_controller_comes_only_from_its_overrides_file_and_keeps_the_base_ones`(`[*base, "FollowPathRL"]`)가 실패한다. 합칠 때 `follow_path_rl.yaml` 목록에 `FollowPathMPPI`를 더하면 된다. 그 뒤 nav_bench 설정의 `controllers`를 `[FollowPathGoal, FollowPathMPPI, FollowPathRL]`로 둔다(`run_order`가 셋을 받는지, `report.py`의 범례가 "RPP"/"MPPI" 둘로 박혀 있는 것은 그때 고친다).
- 공정하게 견주려면 속도 상한을 맞춰야 한다(RPP 시뮬 0.20, RL 0.25 m/s).

### 10.6 알아야 할 포인트 (계획 개념: 학습 정의를 배포에 똑같이 옮기기, 경로 따라가기, 패리티 시험)

**쓴 개념.**
- 앞보기 점(lookahead point): 경로 끝 대신 경로 위 조금 앞의 점을 목표로 보여 주면 정책은 "가까운 점 쫓기"만 배우면 되고, 길 고르기(벽·진열대 돌기)는 전역 계획(NavFn)이 맡는다. RPP의 lookahead와 같은 생각이다.
- 진행 번호(progress index): 로봇이 경로의 어디까지 왔는지. 뒤로 가지 않고 한 번에 창만큼만 앞으로 가게 해, 경로가 자기와 가까이 지나는 곳(U자)에서 엉뚱한 점으로 뛰지 않는다.
- 경로 길이로 잰 거리와 직선거리: 굽은 경로에서는 다르다. 학습은 경로 길이(누적 길이 `cum`)로 쟀다.
- 학습·배포 일치(train-deploy parity): 같은 입력에 같은 관측이 나와야 한다. 이번에는 fixture를 학습 코드(`route_spec.follow`, `task_spec`)가 직접 만들어 C++가 그것과 같은지만 본다. 공식을 시험에 다시 적지 않는다.
- 부동소수점 동점(tie): "0.40 m 이상인 첫 점"이 정확히 0.40 m에 걸리면 반올림 한 칸이 답을 바꾼다. 같은 틀에서 같은 순서로 더해야 학습과 같은 점이 나온다(결정 16).

**먼저 읽을 코드 세 곳.**
1. `pode/pode_rl_controller/src/observation.cpp` `follow_progress`, `lookahead_index`, `path_goal`: 학습 `route_spec.follow`를 옮긴 곳.
2. `pode/pode_rl_controller/src/follow_path_rl.cpp` `configure`(명세의 규칙, 덮어쓰기 WARN)와 `goals`(경로 틀에서 로봇 자세 읽기).
3. `learning/goal_reach/export_policy.py` `follow_cases`, `CheckpointPolicy`: fixture와 ONNX를 만드는 곳.

**숫자 읽는 법.**
- 로그 `policy goal 0.386 m at -2.05 rad, end 1.159 m`: 정책이 보는 목표(앞보기 점)는 0.39 m 앞, 거의 뒤쪽(−2.05 rad), 경로 끝은 1.16 m. 학습에서 이 거리는 평균 0.35 m, 표준편차 0.096 m였다. 0.6 m를 넘게 자주 보이면 로봇이 경로에서 멀리 벗어났다는 뜻이다.
- 표의 "몸 최소": 발자국 다각형과 벽·진열대 면의 실제(Gazebo) 거리. 0.005 m 이하면 nav_bench가 충돌로 센다. "라이다 최소"는 스캔에서 가장 가까운 유효 거리이고 0.050이면 측정 하한에 닿은 것이다.
- "ω 부호 바뀜": 0.02 rad/s 넘는 ω 명령의 부호가 바뀐 횟수. 크면 좌우로 흔들린다. 판 길이가 다르니 같은 goal끼리만 견준다.
- 추론 시간: 1 s 평균의 가운데값 50 µs가 controller 주기 50 ms의 0.1 %다. 첫 호출 1.6 ms는 ORT가 처음 한 번 준비하는 값이다.

**스스로 점검.**
1. 곧은 경로에서 점 간격이 0.025 m이면 진행 점에서 0.40 m 앞은 몇 번째 점인가? 간격이 0.0125 m(시뮬)면? 창 20점은 각각 몇 m인가?
2. 플러그인이 경로를 매 주기 `base_footprint`로 옮겨 앞보기 점을 고르면 무엇이 흔들릴 수 있나? 경로 틀(map)에서 고르면 왜 괜찮은가?
3. Q2 → E-3에서 정책은 왜 뒤로 빼며 도는가? 학습 경기장에서 Q2처럼 벽에 0.15 m 붙은 출발점이 얼마나 자주 나왔을지, 그리고 접촉 힘 1 N 판정이 스치기를 놓치는지 생각해 본다.
4. YAML에 `lookahead_dist: 0.3`을 적으면 무슨 로그가 나오고, 정책이 보는 `goal_dist` 관측은 학습 분포(평균 0.35 m)와 어떻게 달라지나?

### 10.7 직접 해 볼 것

Q2 → E-3의 벽 붙기를 직접 본다(RL만, 세 판, 약 3분). WSL에서 무거운 실행 잠금을 잡고 한다. `SP`는 이번 설정 파일을 둔 곳으로, 아래처럼 `g3_q2_e3.yaml`을 만들어 두면 된다(MPPI worktree `tools/nav_bench/scenarios.yaml`을 복사하고 `controllers: [FollowPathGoal, FollowPathRL]`, `repeats: 3`, `start: {x: 1.66, y: 0.15, yaw_deg: 0}`, `goal: {x: 0.64, y: 0.24, yaw_deg: 90}`, `scenarios: {a_hall: []}`로 바꾼다). 아래를 파일로 저장해 `bash 파일`로 돌린다(잠금이 잡혀 있으면 주인을 보여 주고 끝난다).

```bash
SP=$HOME/rl_cfg   # g3_q2_e3.yaml을 둔 폴더
LOCK=/mnt/c/Users/pjw/ros2camp/.heavy_run.lock
mkdir "$LOCK" && echo "me $$ rl-q2-e3 $(date -Is)" > "$LOCK/owner" || { cat "$LOCK/owner"; exit; }
trap 'rm -rf "$LOCK"' EXIT
set +u; source /opt/ros/jazzy/setup.bash; source ~/ros2_deps_ws/install/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash; source ~/gg_build/pd-rl-plugin/install/setup.bash
export ROS_DOMAIN_ID=62 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=me_62 GZ_IP=127.0.0.1
ros2 launch fleet_gz_sim one_pode.launch.py gui:=false rviz:=false fsm:=false start:=Q2 nav_log_level:=info \
  nav_overrides:=$HOME/nf_src/pd-rl-plugin/sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml > /tmp/me_launch.log 2>&1 &
# bt_navigator가 active가 되면(ros2 lifecycle get /pode1/bt_navigator)
PYTHONDONTWRITEBYTECODE=1 python3 /mnt/c/Users/pjw/ros2camp/pd-mppi/tools/nav_bench/bench_node.py \
  --config "$SP/g3_q2_e3.yaml" --scenario a_hall --out /tmp/me_legs
grep "FollowPathRL:.*policy goal" /tmp/me_launch.log | head -5
kill %1; sleep 10   # 남은 gz·Nav2 프로세스가 없는지 ps로 확인한다
```

볼 것: 첫 몇 줄의 `act`(v 쪽이 −1 아래면 뒤로 뺀다)와 `goal ... at ... rad`. 판 기록 `/tmp/me_legs/*.json`의 `truth`에서 y가 0.09 아래로 내려가는 때를 찾는다. 출발 방향을 서쪽(`yaw_deg: 180`)으로 바꾸면 벽 붙기가 사라지는지 보면, 원인이 "뒤로 빼며 돌기"인지 가릴 수 있다.

### 10.8 남은 일

- Q2 → E-3 벽 붙기: 정책을 다시 학습할지(벽에 붙은 출발점과 출발 방향을 더 섞기, 스치기 판정 강화), 플러그인에 뒤로 빼기 제한이나 costmap 충돌 검사를 더할지 정한다.
- RL·RPP·MPPI를 같은 속도 상한으로 한 판에 견주기(10.5의 브랜치 합치기와 목록 고치기가 먼저).
- Pi4에서 빌드·추론 시간 재기(6절).
- 플러그인 단위 시험(가짜 TF·경로로 `computeVelocityCommands`의 연결 보기)은 여전히 없다. 이번에 그 연결 중 목표점 계산은 `path_goal`로 빼서 시험한다.
