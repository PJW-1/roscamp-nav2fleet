# 2026-11-24 RL 실물 준비 점검 (작업 3)

질문: 실물 Pode(Raspberry Pi 4, aarch64, ROS 2 Jazzy, RPLIDAR)에서 FollowPathRL(정책 `store_arena_4b`,
Nav2 RotationShim으로 감쌈)을 돌리기 전에 무엇이 비어 있는가. 코드는 바꾸지 않았고 로봇에도 닿지 않았다.
기준은 `origin/main`=b2aea4d, shim 쪽은 `origin/feat/rl-shim-recipe-2026-11-22`(미병합),
실물 자료는 `origin/data/real-2026-11-23`의 pode1 rosbag(`1123_ds_off2`, 387.6 s, scan 3828개)이다.

표시: **[확정]** 코드·자료·공개 문서에서 직접 본 것(file:line 또는 URL), **[추측]** 확인하지 않은 판단.
bag 분석 스크립트는 `docs/rl_real_prep_2026-11-24/`에 있다(맨 끝 "다시 재는 법").

## 한눈에

- **ONNX Runtime은 막히지 않는다.** Jazzy apt에 arm64용 `ros-jazzy-onnxruntime-vendor` 0.1.1(ORT 1.24.3)이 있다.
  .deb 5.6 MB, 설치 19.3 MB, 빌드가 필요 없다 [확정, packages.ros.org arm64 색인]. 로봇에 인터넷이 없으면
  PC에서 .deb 하나를 받아 scp로 옮기면 된다.
- **배포 흐름에 RL이 없다.** `deploy.sh`의 패키지 목록에 `pode/pode_rl_controller`가 없고 [확정 tools/real_pode/deploy.sh:9-10],
  `start.sh`는 실행 때마다 `make_params.py`로 `params/`를 새로 써서 손으로 넣은 FollowPathRL 블록을 지운다
  [확정 tools/real_pode/start.sh:37]. 실물 nav2.launch.py에는 시뮬의 `nav_overrides` 같은 덮어쓰기 인자가 없다.
  코드 변경 없이 하려면 패키지를 손으로 rsync하고, 고친 params 사본으로 `real_pode.launch.py`를 직접 띄워야 한다(2.3절).
- **실물 스캔에는 학습에 없던 "몸 뒤 왼쪽 7.5 cm 물체"가 늘 보인다.** 빔 300~306(base 기준 약 150~153°)이
  scan의 92~99 %에서 0.075 m다 [확정, bag]. 이 값은 range_min 0.05보다 커서 플러그인이 그대로 쓰고,
  섹터 33이 늘 약 0.037(=0.075/2.0)이 된다. Isaac 스캔 표본에는 0.2 m 안의 점이 없다 [확정, newton_pinky_check].
  같은 bag 스캔을 정책에 넣어 보면 이 점 하나로 목표 ω가 중앙값 0.044, 95 % 0.34, 최대 0.84 rad/s 달라진다
  [확정, 오프라인 재생, 목표를 정면 0.4 m로 고정한 조건]. 코드 변경(빔 가리기) 또는 재학습 판단이 필요하다.
- **Pi4는 이미 바쁘다.** RPP로 달린 11-23 판의 pode1 CPU 평균이 74~81 %, 합류 구간 81~92 %였고 [확정 results/2026-11-23/checklist_2026-11-23_filled.md:207],
  scan이 최대 0.64 s 끊겼다 [확정 results/2026-11-23/measure_2026-11-23.md:63]. 플러그인은 scan이 `scan_timeout` 0.5 s보다 오래되면
  그 주기에 ControllerException을 던진다 [확정 follow_path_rl.cpp:226-228]. 추론 자체(Gazebo 안 PC에서 평균 36~57 µs)는 작다고 보지만 Pi4 값은 아직 없다.
- **실물 기본 launch는 log_level warn이라 FollowPathRL의 시간 로그(INFO)가 안 보인다** [확정 tools/real_pode/real_pode.launch.py:119, follow_path_rl.cpp:287].
  잴 때는 `log_level:=info`로 띄운다.
- **RL은 Nav2 goal(navigate_to_pose)에만 쓰인다.** 고정 경로 셋(FollowPath 등)은 pode_fsm이 RPP로 바로 부른다
  [확정 nav2_params.yaml:66-68, 274-275]. 되돌리기는 `controller_selector`에 `FollowPathGoal`을 한 번 보내는 것이다.

## 사용자가 정할 것

1. **몸 뒤 왼쪽 7.5 cm 점을 어떻게 다룰지.** (a) 플러그인에 "가릴 빔 각도 범위" 파라미터를 더해 inf로 바꾼다(코드 변경, 작음),
   (b) 정책 쪽(재학습 때 몸 가림 무작위화), (c) 그대로 시험. 먼저 그 점이 로봇의 무엇인지(케이블, 손잡이, 배터리 등) 눈으로 본다.
2. **실물에 RL을 띄우는 길.** (a) 코드 변경 없이: 손 rsync + 직접 만든 params 파일 + `real_pode.launch.py` 직접 실행(2.3절),
   (b) `deploy.sh` 목록에 `pode/pode_rl_controller`를 더하고 `make_params.py`(또는 nav2.launch.py)에 RL 덮어쓰기 옵션을 더한다(코드 변경).
3. **실물 첫 시험을 RL 혼자로 할지 RL+shim으로 할지.** main의 시뮬 덮어쓰기는 RL 혼자가 기본이고 shim은 별도 파일이다
   [확정 sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml, follow_path_rl_shim.yaml]. RL+shim 기본과
   `angular_disengage_threshold` 0.39는 미병합 브랜치에만 있다 [확정 feat/rl-shim-recipe-2026-11-22의 follow_path_rl.yaml].
   그 브랜치는 "실물 nav2_params.yaml에는 아직 넣지 않는다(Q2 -> E-3 벽 접촉의 원인을 안 뒤)"고 적었다.
4. **Pi4에서 gtest까지 돌릴지.** `ament_cmake_gtest`가 로봇에 있어야 하고 빌드 시간이 는다. 돌리지 않으면 PC(WSL)의 parity와 같은 ORT 1.24.3이라는 것에 기댄다.
5. **로봇 인터넷.** apt로 바로 받을지, PC에서 .deb를 옮길지.
6. **scan 끊김과 `scan_timeout`.** 0.5 s를 그대로 두면 CPU가 바쁜 구간에서 FollowPathRL이 실패를 낼 수 있다. 올리면 오래된 스캔으로 달린다(기준 완화).

## 알아야 할 포인트

- **onnxruntime_vendor**: ORT를 ROS 패키지로 감싼 것. 소스로 빌드해도 ORT를 컴파일하지 않고 GitHub 릴리스의 미리 빌드된 tgz를 받아 설치만 한다. apt 바이너리는 그 결과물을 담고 있어 받기가 없다.
- **RotationShim**: Nav2 controller를 감싸, 경로 방향과 크게 어긋나면 먼저 제자리에서 돌고 그다음 안쪽 controller(여기서는 FollowPathRL)에 넘긴다. 돌기 전 costmap으로 회전 충돌을 검사한다.
- **controller_selector**: BT의 ControllerSelector가 듣는 토픽. 여기에 controller 이름을 보내면 그 뒤 Nav2 goal의 FollowPath가 그 controller로 간다(기본 FollowPathGoal = RPP).
- **섹터**: 정책은 1° 간격 학습 광선 360개를 10개씩 묶은 36개 섹터의 최솟값(÷2.0 m)을 본다. 섹터 안에 가까운 점 하나만 있어도 그 섹터 전체가 가깝다고 본다.

## 1. ONNX Runtime을 aarch64에서 얻는 법

### 1.1 지금 코드가 요구하는 것

- `find_package(onnxruntime_vendor REQUIRED)`, 대상 `onnxruntime::onnxruntime`, package.xml `<depend>onnxruntime_vendor</depend>`
  [확정 pode/pode_rl_controller/CMakeLists.txt:16,35, package.xml:17].
- 그 밖에 `yaml_cpp_vendor`, Nav2 헤더(nav2_core, nav2_costmap_2d, nav2_util), 시험에 `ament_cmake_gtest` [확정 CMakeLists.txt:9-25,52].
- 정책 파일 `policies/store_arena_4b/policy.onnx`(89,901 B)와 `policy_spec.json`이 share로 설치된다 [확정 CMakeLists.txt:49].
  LFS가 아니라 일반 git 파일이다 [확정 .gitattributes].
- RotationShim으로 감싸려면 `ros-jazzy-nav2-rotation-shim-controller`. arm64 `ros-jazzy-navigation2` 1.3.14가 이것을 의존한다 [확정 packages.ros.org arm64 색인].
  로봇에 그 패키지가 깔렸는지, 버전이 `angular_disengage_threshold`를 읽는지는 모른다 [추측]. 로봇에서 `ros2 pkg prefix nav2_rotation_shim_controller`와
  `apt-cache policy ros-jazzy-nav2-rotation-shim-controller`로 본다(shim 브랜치 기록은 WSL 1.3.13 소스를 읽었다).

### 1.2 선택지

| 길 | 로봇이 받는 것 | 네트워크 | Pi4 빌드 부담 | 비고 |
|---|---|---|---|---|
| **A. apt** `sudo apt install ros-jazzy-onnxruntime-vendor` | arm64 .deb 5,587,572 B, 설치 19,330 KB, 버전 `0.1.1-1noble.20261026.232151`, 의존 libc6≥2.34, libgcc-s1, libstdc++6, ros-jazzy-ros-workspace [확정 http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-arm64/Packages.gz] | packages.ros.org | 없음 | 가장 쉽다. 로봇의 apt 색인이 오래됐으면 `apt update` 필요 |
| **B. 오프라인 .deb** | PC에서 `http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-jazzy-onnxruntime-vendor/ros-jazzy-onnxruntime-vendor_0.1.1-1noble.20261026.232151_arm64.deb`를 받아 scp. `sudo apt install ./<deb>` 또는 sudo 없이 `dpkg -x <deb> ~/ort`에 풀고 `CMAKE_PREFIX_PATH`에 `~/ort/opt/ros/jazzy` | 로봇은 없음 | 없음 | README의 sudo 없는 길 [확정 pode/pode_rl_controller/README.md:24-26]. amd64에서는 RPATH로 실행 때 라이브러리를 찾았다 [확정 docs/worklog_follow_path_rl_2026-11-21.md:175]. arm64에서는 안 해 봤다. 받은 파일은 색인의 SHA256 `62c7c56c47f898c58f9f40698268259c6ab322d7fa91ecf2fe960c6967ed2906`으로 확인 |
| C. vendor 소스 빌드 | FetchContent가 `https://github.com/microsoft/onnxruntime/releases/download/v1.24.3/onnxruntime-linux-aarch64-1.24.3.tgz`를 받는다 [확정 https://raw.githubusercontent.com/ros-controls/onnxruntime_vendor/master/CMakeLists.txt] | github.com | 작음(받아서 풀고 설치만) | ORT를 컴파일하지 않는다. tgz 크기는 확인 못 했다(이번에도 GitHub의 microsoft/onnxruntime 릴리스 API가 404, 11-21 조사와 같음) |
| D. 시스템 ORT | `AMENT_VENDOR_POLICY=NEVER_VENDOR`면 vendor가 받지 않고 시스템 `onnxruntime`에 기댄다 [확정 같은 CMakeLists] | — | — | Ubuntu 24.04 apt에 C++ ORT 패키지가 있는지는 확인 안 함 [추측: 없음] |
| E. pip wheel | `onnxruntime` aarch64 wheel | PyPI | — | Python용이다. C++ 헤더와 CMake 설정이 없어 이 플러그인에는 맞지 않는다 [추측] |
| F. 교차 빌드 | — | — | — | A/B가 미리 빌드된 것을 주므로 이득이 없다 [추측] |
| G. ORT 없이 Eigen 직접 계산 | — | — | — | 코드 변경. 11-21 기록의 대안 [확정 docs/worklog_follow_path_rl_2026-11-21.md:194] |

### 1.3 Pi4에서 플러그인 빌드

- ORT 쪽 빌드는 A/B면 없다. 남는 것은 `pode_rl_controller` 자체(소스 4개, gtest 2종 3개)다 [확정 CMakeLists.txt:27-66].
- 시간과 메모리는 재지 않았다 [추측: Nav2 헤더를 읽는 번역 단위 하나가 수백 MB~1 GB, 몇 분]. 11-23 끔 다시 판 중 pode1의 남은 메모리는 최소 1.6 GB였다
  [확정 results/2026-11-23/robot/1123_ds_off2/pode1/vmstat_1123_ds_off2.txt, free 열 최소 1,618,868 KB]. 빌드는 Nav2를 내린 뒤
  `MAKEFLAGS=-j1 colcon build --packages-select pode_rl_controller --parallel-workers 1`로 하고 `/usr/bin/time -v`로 잰다.
- gtest의 NAV_SPEED_FILE은 소스 기준 `../pode_nav_config/params/nav_speed.yaml`이다 [확정 CMakeLists.txt:59,67].
  로봇에서 `~/nav2fleet_ws/src/pode/pode_rl_controller`에 두면 deploy.sh가 놓는 `src/pode/pode_nav_config`와 맞는다.

## 2. 실물 nav2_params와 시뮬 덮어쓰기의 차이, 실물에 넣는 법

### 2.1 차이

| 항목 | 실물(`pode/pode_nav_config/params/nav2_params.yaml`, main) | 시뮬 덮어쓰기 |
|---|---|---|
| controller_plugins | `FollowPath, FollowPathThrough, FollowPathReverse, FollowPathGoal` [확정 :101] | 그 넷 + `FollowPathMPPI` + `FollowPathRL`(main) [확정 sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml], shim 브랜치는 `FollowPathRLAlone`도 |
| FollowPathRL 블록 | 없음 | main `follow_path_rl.yaml`: RL 혼자. main `follow_path_rl_shim.yaml`과 shim 브랜치: `RotationShimController` + `primary_controller: pode_rl_controller::FollowPathRL`, `angular_dist_threshold 0.785`, (브랜치만) `angular_disengage_threshold 0.39`, `forward_sampling_distance 0.15`, `rotate_to_heading_angular_vel 1.0`, `max_angular_accel 2.0`, `simulate_ahead_time 1.0`, `rotate_to_goal_heading true` |
| controller_frequency | 20.0 [확정 :87], 정책 control_dt 0.05와 같다 [확정 policy_spec.json]. 11-23 bag의 cmd_vel 간격 중앙값 0.050 s [확정, bag] | 같음 |
| 속도 | `nav_speed.yaml` 0.12 = 정책 v_max 0.12, 어긋남 ERROR 없음 [확정 nav_speed.yaml, policy_spec.json, follow_path_rl.cpp:74-84] | 같음 |
| scan_topic | 플러그인 기본 `scan`(상대 이름 → `/pode1/scan`, 거르지 않은 스캔) [확정 follow_path_rl.cpp:47] | 같음 |
| FollowPathMPPI | 실물 목록에 없음. 블록만 있다(Pi에 nav2_mppi_controller 없음) [확정 tools/real_pode/make_params.py:10, docs/report_2026-11-22_night.md] | 목록에 있음 |

**주의**: 시뮬 파일의 `controller_plugins`를 그대로 실물에 쓰면 `FollowPathMPPI`가 들어가 controller_server가 플러그인을 못 찾고 설정에 실패한다
[추측: Pi에 MPPI가 없다는 11-22 기록이 맞다면]. 실물용은 MPPI를 빼고 `FollowPathRL`만 더한 다섯이어야 한다.

### 2.2 실물에서 params가 만들어지는 흐름 [확정]

1. PC `deploy.sh pinky@<주소>`: 커밋된 HEAD의 7개 패키지를 `~/nav2fleet_ws/src/`에, `tools/real_pode`를 `~/nav2fleet_real/`에 rsync. 빌드는 하지 않는다 [deploy.sh:1-30]. **`pode_rl_controller`는 목록에 없다** [deploy.sh:9-10].
2. 로봇 `start.sh NS X Y YAW [SPEED]`: 매번 `make_params.py --speed S --out params`로 `nav2_params.yaml`, `avoidance.yaml` 사본을 새로 쓴다 [start.sh:37]. 사본은 원본을 그대로 베끼고 속도 줄(과 고정 경로 충돌 검사 줄)만 바꾼다 [make_params.py:1-16].
3. `real_pode.launch.py`의 `params_file` 기본은 `~/nav2fleet_real/params/nav2_params.yaml` [real_pode.launch.py:102]. start.sh는 `params_file`도 `log_level`도 넘기지 않는다 [start.sh:42-45].
4. `nav2.launch.py`가 avoidance(FollowPathGoal), keepout, scan_filter, peer 층을 덮은 뒤 RewrittenYaml로 namespace를 붙인다 [pode/pode_nav_config/launch/nav2.launch.py:66-106]. 사용자 덮어쓰기 인자는 없다(`nav_overrides`는 시뮬 gz.launch.py/one_pode.launch.py에만 있다 [sim/fleet_gz_sim/launch/gz.launch.py:90]).

### 2.3 코드 변경 없이 실물에 넣는 순서 (제안)

1. PC에서 `rsync -a pode/pode_rl_controller/ pinky@<주소>:~/nav2fleet_ws/src/pode/pode_rl_controller/`(deploy.sh는 그대로 쓰고 이 패키지만 손으로 더한다).
2. 로봇에서 ORT(1.2절 A 또는 B) 뒤 `colcon build --packages-select pode_rl_controller`.
3. 로봇에서 `python3 make_params.py --speed 0.12 --out params_rl`로 별도 폴더에 사본을 만들고, `params_rl/nav2_params.yaml`의
   `controller_server.ros__parameters`에 FollowPathRL 블록을 더하고 `controller_plugins`를 MPPI 없는 다섯으로 고친다.
4. `start.sh` 대신 `source env.sh && ros2 launch ./real_pode.launch.py namespace:=pode1 initial_x:=… initial_y:=… initial_yaw:=… params_file:=$PWD/params_rl/nav2_params.yaml avoidance_params_file:=$PWD/params_rl/avoidance.yaml log_level:=info`.
   start.sh가 하는 잠금·pid·로그 파일은 빠진다(stop.sh가 이 프로세스를 못 찾을 수 있다 [추측]). Ctrl+C로 내린다.

길게 쓸 것이면 코드 변경(사용자 결정 2)이 낫다: deploy.sh 목록에 더하기, make_params.py에 덮어쓰기 yaml을 깊게 병합하는 옵션(MPPI가 목록에 있으면 거절) 같은 것.

## 3. 실물 스캔과 학습 관측

### 3.1 실물 스캔 (11-23 pode1 bag, [확정] 재현: `scan_sum.py`, `scan_beams.py`)

| 항목 | 실물 | Gazebo | Isaac 학습 |
|---|---|---|---|
| 드라이버 | sllidar_node, DenseBoost, 5 kHz, 10 Hz, max_distance 40 m, FW 1.02, HW 18 [확정 results/2026-11-23/robot/1123_ds_off2/pode1/launch_165948.log] | gpu_lidar | Newton ray caster |
| 빔 수, 각도 | 720, angle_min −π, angle_max π, 간격 0.5007°(=2π/719) | 640, 360° [확정 tools/real_pode/README.md:176] | 360, −180°부터 1°, base 기준 [확정 policy_spec.json, learning/goal_reach/goal_reach_env.py:53-66] |
| range_min / max | 0.05 / 40.0 | 0.05 / 12 | 시작 0.05, 2.0에서 자름 [확정 learning/goal_reach/task_spec.py:80-81] |
| 주기 | 9.92 Hz, 간격 중앙값 0.101 s, 95 % 0.109, 최대 0.635 s | 10 Hz | 매 제어 단계(20 Hz) |
| 0 / inf / NaN | 0이 0개, inf 10.1 %(277,441/2,756,160), NaN 0 | 맞지 않으면 inf | 비유한 값 → 2.0 [확정 task_spec.py:230-234] |
| 틀 | `rplidar_link`, `rplidar_mount`에서 yaw π, mount는 base_link (−0.017, 0) [확정 bag tf_static] | 같음 | 같은 위치, 광선은 base 기준 |
| 가장 먼 유한 값 | 15.01 m | — | — |

### 3.2 플러그인이 섹터를 만드는 법 [확정 pode/pode_rl_controller/src/observation.cpp:67-114]

- 학습 광선 k(base 각도 −180+k°)마다 `i = floor(((θ_k − lidar_yaw − angle_min) mod 2π) / angle_increment + 0.5)`로 **가장 가까운 빔 하나**를 고른다.
  lidar_yaw는 처음 한 번 TF에서 읽어 둔다 [follow_path_rl.cpp:169-186].
- 값이 비유한, `< range_min`, `> range_max`면 2.0 m(=빈 곳). 그다음 2.0에서 자른다.
- 10개씩 묶어 최솟값 ÷ 2.0. 정책 명세의 `lidar.min_range` 0.05는 읽기만 하고 관측에는 쓰지 않는다 [확정 policy_spec.cpp:36, observation.cpp에 사용 없음].
  대신 스캔의 range_min(실물 0.05)을 쓴다. 지금은 값이 같아 문제는 없다.

### 3.3 어긋남과 할 일

| # | 어긋남 | 근거 | 영향 | 할 일 |
|---|---|---|---|---|
| S1 | **몸 뒤 왼쪽 0.075 m의 고정 점.** 빔 300~306(base 약 150.2~153.2°)이 scan의 92~99 %에서 중앙값 0.075~0.076 m. 학습 광선 330~333 → 섹터 33의 평균이 0.0746 m(관측 약 0.037) | [확정 bag, scan_beams.py, replay.py]. Isaac 스캔 표본에는 0.2 m 안 점 없음 [확정 data/real-2026-11-23의 results/2026-11-21/newton_pinky_check/sensors_scans.json] | 오프라인 재생(3828 스캔, 목표 정면 0.4 m, v 0.1, w 0): 빔 300~306을 inf로 바꾸면 섹터 33 평균이 0.453 m가 되고, 목표 ω 평균 −0.005 → −0.079 rad/s, |Δω| 중앙값 0.044, 95 % 0.34, 최대 0.84. |Δv| 95 % 0.011 m/s. 후진 명령은 둘 다 0 [확정, replay.py] | 그 물체가 무엇인지 본다. 플러그인에 빔 가리기(각도 범위 → inf) 파라미터와 test_observation 사례 추가(코드 변경). 점의 base 좌표는 약 (−0.084, +0.036) m로 발자국 다각형 뒤 왼쪽 모서리(−0.076, 0.037) 바로 뒤다 [확정 계산, footprint nav2_params.yaml:389]. 좌우는 TF의 각도 방향이 맞다는 가정 아래다 [추측: AMCL이 이 TF로 돈다] |
| S2 | **그 옆 그림자 빔이 inf.** 빔 297~298, 309(약 148.7~149.2°, 154.7°)와 432, 440(약 −143.7°, −139.7°)이 scan의 90 % 넘게 inf | [확정 bag] | inf는 2.0 m(빈 곳)이 된다. 몸 뒤 대각선의 진짜 장애물을 그 빔으로는 못 본다. 섹터 안 다른 빔이 메워 주는지는 섹터마다 다르다 | 시험: 뒤 대각선에 상자를 두고 섹터 값을 본다(4.2절 2단계) |
| S3 | **빔 절반만 쓴다.** 0.5° 빔에서 1° 광선마다 가장 가까운 하나라, 짝수 번호 빔만 쓰인다(296, 298, …) | [확정 replay.py 출력] | 0.5°보다 좁은 물체를 놓칠 수 있다 [추측, 1 m에서 0.5°는 약 0.9 cm] | 바꾸려면 "광선 ±0.5° 안 빔의 최솟값" 같은 코드 변경. 학습과 다른 규칙이 되므로 S1부터 |
| S4 | **가까운 물체가 inf로 올 수 있다.** 학습에서는 2 m 안의 면이면 늘 맞는다. 실물은 inf 10 % | inf 비율 [확정 bag]. 원인(먼 곳, 검은 면, 그림자) 구분은 [추측] | 가까운 벽이 inf면 정책은 빈 곳으로 본다 | 경기장 벽·진열대 재질에서 섹터 값을 본다. 필요하면 inf 메우기(코드 변경) |
| S5 | **스캔 주기와 지연.** 실물 10 Hz라 20 Hz 제어의 두 주기가 같은 스캔을 쓴다. 학습은 매 단계 새 스캔. CPU가 바쁠 때 0.64 s 끊김 | [확정 bag, measure_2026-11-23.md:63] | `scan_timeout` 0.5 s를 넘으면 ControllerException [확정 follow_path_rl.cpp:226-228] | Pi4 실측(4.1절)에서 scan age 로그를 본다. timeout을 올리는 것은 기준 완화라 사용자 결정 6 |
| S6 | **속도 관측은 odom.** 플러그인은 controller_server가 준 velocity(odom)를 v, w 관측으로 쓴다 [확정 follow_path_rl.cpp:261-262]. 실물 odom 28.5 Hz [확정 bag 11045개/387.6 s] | — | odom 잡음·지연이 학습의 참값과 다르다 [추측] | 1단계 반응 측정(11-23 미실시) 결과와 같이 본다 |
| S7 | 실물 range_max 40 m vs 학습 2 m 자르기 | [확정] | 2.0에서 잘라 문제 없음 | 없음 |

`range_min` 아래(0 아닌) 값은 4개뿐이고 0은 없었다. 11-21 기록이 대비한 "실물 RPLIDAR의 0"은 이 드라이버 설정에서는 나오지 않았다
[확정 bag]. sllidar가 0을 inf로 바꿔 내는 것으로 보인다 [추측].

## 4. Pi4에서 잴 것과 시험 순서

### 4.1 잴 것

| 항목 | 방법 | 기준 |
|---|---|---|
| ORT 설치 | `ros2 pkg prefix onnxruntime_vendor`, 그 아래 `opt/onnxruntime_vendor/lib` | `libonnxruntime.so` 있음 |
| 빌드 시간·최대 메모리 | `/usr/bin/time -v colcon build --packages-select pode_rl_controller --parallel-workers 1`(Nav2 내린 상태) | 기록만. 남은 메모리가 빠듯하면 `-j1` 유지 |
| gtest parity | `colcon test --packages-select pode_rl_controller --event-handlers console_direct+`, 그다음 `colcon test-result --verbose` | test_parity_store_arena_4b, test_observation 통과. `[parity] … vs onnxruntime python` 값(x86 python 대 arm64 C++)과 `inference … us per call`을 적는다. 판정은 torch 기준 허용 범위뿐이다 [확정 test_parity.cpp:130-170] |
| 추론 시간(제어 중) | `log_level:=info`로 띄우고 FollowPathRL 1초 로그의 `infer mean/max`, `call mean/max`, `scan age` | 비교 기준은 Gazebo 20 Hz의 평균 36~57 µs, 최대 169 µs [확정 docs/worklog_follow_path_rl_2026-11-21.md:152]. call max가 5 ms(주기의 10 %)를 넘으면 원인을 본다 [추측 기준] |
| 제어 주기 | controller_server의 "Control loop missed its desired rate" WARN 횟수, bag의 cmd_vel 간격 | 11-23 RPP 끔 다시 판 로그에는 그 WARN이 없었다 [확정 launch_165948.log grep] |
| CPU | `vmstat 1`(11-23과 같게), 가능하면 `pidstat -p <container pid> 1` | RPP 판 74~92 %와 견준다 |
| 실물 스캔 | `scan_info.py --ns pode1 --seconds 10`(11-23에 못 함) | 3.1절 표와 같으면 됨 |

### 4.2 한 대 시험 순서 (제안)

0. **가만히**: Nav2 없이 bringup만(`bringup_only.launch.py`)으로 `scan_info.py`, 몸 뒤 왼쪽 물체 확인(S1).
1. **바퀴 띄우고 빌드·parity**: 1.3절 빌드, gtest.
2. **바퀴 띄운 채 RL 띄우기**: 2.3절 순서, `log_level:=info`. `controller_selector`에 FollowPathRL을 보내고(4.3절과 같은 QoS 옵션) 1 m 앞 Nav2 goal.
   로그에서 `infer … us`, 라이다 TF 한 줄(`lidar rplidar_link in base_footprint … yaw 3.142` 꼴 [확정 follow_path_rl.cpp:182-184]),
   scan age, 명령 v ≤ 0.12를 본다. 몸 둘레 8방향에 상자를 대 보고 명령이 그쪽을 피하는지 본다(S1, S2).
3. **바닥, 빈 곳, 짧은 직선**: 경기장 빈 자리에서 0.5 m 앞 goal, 옆에서 끌 준비. 더 느리게 하려면 `max_linear_vel` 파라미터(예 0.06) [확정 follow_path_rl.cpp:55,84].
4. **방향 틀어진 goal**: 뒤쪽 goal로 회전(shim이면 shim 회전, 아니면 정책의 후진·회전). 벽에서 0.3 m 떨어진 곳에서.
5. **좁은 곳**: 시뮬에서 문제였던 Q2 → E-3처럼 벽 옆 출발. 몸 뒤 왼쪽 모서리를 본다.
6. **RPP와 번갈아**: 같은 출발·goal을 FollowPathGoal과 FollowPathRL로 번갈아, 로봇 bag을 켠다.
7. 그다음에만 관제(pode_fsm nav_goal 모드)로 넘어간다.

### 4.3 RPP로 되돌리기

- **실행 중, 가장 빠름**: `ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /pode1/controller_selector std_msgs/msg/String "{data: FollowPathGoal}"` [확정 pode/pode_nav_config/bt/navigate_to_pose_w_replanning_and_recovery_goal.xml:23].
  QoS 두 옵션을 빼면 안 된다: Jazzy `ros2 topic pub`의 기본 durability는 volatile이고 ControllerSelector는 transient_local로 구독해 짝이 맞지 않는다.
  그러면 `--once`가 구독자를 기다리며 멈추고 아무것도 보내지 않는다 [확정 /opt/ros/jazzy ros2topic/verb/pub.py:86-95,113; 저장소의 발행 예 tools/nav_bench/bench_node.py:88-90].
  그 뒤의 goal부터 RPP다. 달리고 있는 goal 안에서 바로 바뀌는지는 [추측]이다. 확실히 하려면 goal을 취소하고 다시 보낸다.
- **멈추기**: goal 취소. 그래도 움직이면 launch Ctrl+C(하나라도 끝나면 전체가 내려가고 bringup이 모터를 끈다 [확정 real_pode.launch.py:11-12]).
  cmd_watchdog는 움직이는 명령이 0.4 s 끊기면 0을 보낸다 [확정 tools/real_pode/cmd_watchdog.py:12].
- **설정으로**: 기본 start.sh로 띄우면 FollowPathRL이 아예 없다. 고정 경로 셋은 원래 RL을 쓰지 않는다.

## 체크리스트

| # | 할 일 | 상태 | 막힘 정도 |
|---|---|---|---|
| 1 | 로봇에 `pode_rl_controller` 소스 옮기기(deploy.sh 밖) | 없음 [확정 deploy.sh:9-10] | 막음(손 rsync로 풀림) |
| 2 | ORT arm64 설치(apt 또는 .deb) | 안 함. 패키지 있음 [확정] | 작음 |
| 3 | 실물용 params에 FollowPathRL(MPPI 없이) 넣고 띄우는 길 | 없음. start.sh가 params를 덮어씀 [확정 start.sh:37] | 막음(직접 launch로 풀림) |
| 4 | shim 패키지·`angular_disengage_threshold` 지원 확인 | 모름 [추측] | 중간(shim을 쓰면) |
| 5 | 몸 뒤 왼쪽 0.075 m 점 처리 | 없음 [확정 bag] | 높음(정책 회전이 바뀜) |
| 6 | inf 그림자·가까운 inf 확인 | 없음 | 중간 |
| 7 | Pi4 빌드 시간·메모리 | 안 잼 | 작음 |
| 8 | Pi4 gtest parity | 안 함 | 작음 |
| 9 | Pi4 추론 시간·CPU, `log_level:=info` | 안 잼. 기본은 warn [확정] | 중간(이미 CPU 74~92 %) |
| 10 | scan 끊김 0.64 s vs `scan_timeout` 0.5 | 결정 대기 | 중간 |
| 11 | 1단계 반응 측정(odom, 회전) | 11-23 미실시 [확정 checklist_2026-11-23_filled.md:308] | 중간 |
| 12 | RL 기본(혼자/shim)과 shim 브랜치 병합 | 사용자 결정 | 결정 |

## 다시 재는 법

`docs/rl_real_prep_2026-11-24/`(bag은 `data/real-2026-11-23`의 `.mcap.zst`를 zstd로 풀고 같은 폴더에 `metadata.yaml`이 없으면 `storage_id='mcap'`로 연다):

- `scan_sum.py <mcap>`: 빔 수, 각도, 거리 범위, 주기, inf/0/NaN, tf_static, cmd_vel 간격(WSL, `source /opt/ros/jazzy/setup.bash`).
- `scan_beams.py <mcap>`: base 기준 10° 칸별 짧은 값·inf 비율, 늘 짧은 빔과 늘 inf인 빔.
- `dump_scans.py <mcap> <out.npz>` 뒤 `replay.py <npz> pode/pode_rl_controller/policies/store_arena_4b`: 섹터 33과 빔 가리기 전후 정책 행동(onnxruntime python 1.24.4, 목표 고정 조건).
