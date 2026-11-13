# fleet_gz_sim

Gazebo 시뮬레이션. launch가 둘 있다. 둘 다 이 컴퓨터 안에서만 돈다(아래 "격리").

| launch | 내용 |
|---|---|
| `gz.launch.py` | 핑키 3대 + 진짜 관제. `fleet_manager`·`pinky_fsm`·Nav2는 진짜, central·팔은 가짜(`fake_robots`). 3대 주행을 눈으로 확인하는 용도 |
| `one_pinky.launch.py` | 핑키 한 대 + Nav2(`pinky_nav_config`) + `pinky_fsm`. 관제는 띄우지 않는다(`drive_check`로 관제 흉내) |

## 준비 (한 번)

ROS 2 Jazzy가 깔린 Ubuntu 24.04(WSL2도 된다)에서 한다.

1. Gazebo, Nav2 등 apt 패키지

   ```bash
   sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-xacro \
     ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp
   ```

2. 로봇 모델: 제조사 `pinky_description`(`iponic.repos`의 pinky_pro 014a09f). 따로 빌드해 둔다.

   ```bash
   mkdir -p ~/pinky_deps_ws/src && cd ~/pinky_deps_ws/src
   git clone https://github.com/pinklab-art/pinky_pro.git && git -C pinky_pro checkout 014a09f
   source /opt/ros/jazzy/setup.bash
   cd ~/pinky_deps_ws && colcon build --packages-select pinky_description
   ```

3. 이 저장소: `pinky_description`을 source한 셸에서 빌드한다(`fleet_gz_sim`이 그것을 찾는다).

   ```bash
   mkdir -p ~/fleet_ws/src && cp -r <저장소> ~/fleet_ws/src/roscamp-repo-2 && cd ~/fleet_ws
   source /opt/ros/jazzy/setup.bash
   source ~/pinky_deps_ws/install/local_setup.bash
   colcon build --packages-up-to fleet_gz_sim
   ```

## 격리

실물과 같은 `/pinkyN/...` 이름을 쓰므로 다음이 아니면 시작하지 않는다(`fleet_gz_sim/isolation.py`). 두 launch 모두 무엇을 띄우기 전에 검사한다.

- `ROS_DOMAIN_ID` 43~101(로봇 네트워크의 6은 안 된다)
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS` 없음
- RMW는 `rmw_fastrtps_cpp`, Fast DDS가 따로 읽는 설정 없음(막는 변수와 파일은 `isolation.py`의 `FAST_DDS_VARIABLES`, `FAST_DDS_FILES`)

Gazebo 통신도 이 컴퓨터 안에 둔다(`GZ_IP=127.0.0.1`, `GZ_PARTITION`이 없으면 `fleet_gz_<도메인>`). 다른 사람이나 다른 시뮬과 같은 도메인을 쓰지 않는다.

## 핑키 3대 (`gz.launch.py`)

### 실행

새 터미널마다 같은 환경을 만든다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash
source ~/fleet_ws/install/local_setup.bash
export ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

```bash
# 기본: D2 배치, Gazebo 화면 + 3대를 한 RViz에, 시나리오 gz_d2(주문 8개), 도킹은 진짜 pinky_fsm(실패)
ros2 launch fleet_gz_sim gz.launch.py

# 임무를 끝까지(도킹 대역), 마당과 급행에 상자를 놓았다 치우는 일정
ros2 launch fleet_gz_sim gz.launch.py sim_dock_stub:=true obstacles:=d2_yard_express

# 다른 배치, 한 대만, 화면 없이
ros2 launch fleet_gz_sim gz.launch.py layout:=c scenario:=gz_c_one gui:=false rviz:=none
```

배치(`layout`)는 `d2`(기본), `d`, `c`이고 구역 지도는 `fleet_core`의 `load_fleet_map(<배치>)`다. 배치마다 월드 `worlds/arena_<배치>.sdf`, Nav2 지도 `maps/arena_<배치>[_fine].yaml`, keepout 마스크 `maps/arena_<배치>_keepout.yaml`을 쓴다. 시나리오의 `map`은 배치와 같아야 한다. 핑키는 시나리오의 `start`(지점 이름)에, 없으면 주차 지점(P-1, P-2, P-3)에 차례로 놓인다. 관제는 같은 배치를 `map_file`로 받고 교통 방식은 `zones`다.

WSL2에서 화면이 느리면 GPU를 쓰게 한다: `export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`(그래픽 카드 이름).

뜨는 순서: Gazebo 월드(`arena`) → 브리지 하나 → 로봇마다 모델, Nav2(프로세스 하나), `pinky_fsm` → 세 대가 모두 IDLE이 되면(`wait_ready`, 보통 1분 안) 관제, 가짜 작업셀·비전·출고 팔·central, `fleet_check`, 장애물 노드, 시나리오 모니터. 3분 안에 IDLE이 안 되면 launch가 끝난다. 시나리오 주문 시각은 Gazebo 시계 0부터 센다.

끝내기: Ctrl-C 한 번, 다 내려갈 때까지(10~30 s) 기다린다. 남은 것이 있는지 본다.

```bash
pgrep -af "gz sim|ruby|component_container|pinky_fsm_node|fleet_manager_node"   # 아무것도 없어야 한다
```

### 옵션

| 인자 | 기본 | 뜻 |
|---|---|---|
| `layout` | `d2` | 배치: `d2`, `d`, `c` |
| `scenario` | `gz_<배치>` | 시나리오 이름(`fleet_gz_sim/scenarios`, 다음에 `fake_robots/scenarios`에서 찾음)이나 파일. 로봇 고장이 든 시나리오는 거부한다. `gz_<배치>`는 3대와 주문 8개, `gz_<배치>_one`은 한 대 |
| `gui` | `true` | Gazebo 화면 |
| `rviz` | `fleet` | `fleet`: 3대를 한 RViz에(로봇, 쥔 구역, 주문). 로봇 이름(`pinky2`)이면 그 로봇의 Nav2 화면. `none`이면 없음 |
| `sim_dock_stub` | `false` | 시뮬 전용 도킹 대역(아래 "도킹") |
| `obstacles` | 없음 | 지점 사이 연결(`link`)에 상자를 놓았다 치우는 일정(`fleet_gz_sim/obstacles`의 이름이나 파일). `d2_yard_express`는 300 s마다 마당 하나, 급행 하나를 120 s씩 |
| `nav_max_speed` | `0.12` | 앞으로 가는 controller들의 최고 속도(m/s). 0.12~0.20만 받는다(지시서의 경기장 속도는 0.12~0.15). 관제의 이동 시간 추정 속도(`nav_speed_mps`, `follow_speed_mps`)는 따라 바뀌지 않으므로 `fleet_params_file`로 같이 맞춘다 |
| `keepout` | `true` | Nav2 global costmap의 keepout 필터(shared가 아닌 구역) |
| `fleet_check` | `true` | 주행 지표를 `result_dir/fleet_check.json`에(아래 "결과 확인") |
| `monitor` | `true` | 시나리오 판정. 시나리오가 끝나면 `summary.json`을 쓰고 launch를 끝낸다 |
| `result_dir` | `gz_result` | 결과 폴더(launch를 띄운 폴더 기준) |
| `fleet_params_file` | 설치된 `fleet_params.yaml` | 관제 파라미터 |
| `heartbeat_timeout_s` | `1.5` | `pinky_fsm`의 heartbeat 끊김 판정 |
| `nav_log_level` | `warn` | Nav2 로그 수준 |
| `scan_filter` | `true` | 스캔 거르개(`pinky_scan_filter`)를 띄우고 배치의 shared가 아닌 구역 사각형(경기장 벽에 닿은 변은 `wall_band_m`만큼 벽 너머까지)과 진열대 끝면마다 `wall_band_m`만큼 넓힌 작은 상자를 좁은 띠로 준다. `false`면 costmap이 원래 스캔을 읽는다(비교용) |
| `wall_band_m` | `0.05` | 거르개가 벽·진열대에서 지우는 띠 폭(m). D2 측정에서 0.04는 도크 진입을 두 번 중 한 번 멈췄고 0.05는 세 번 지나갔다(`results/fixed_line_results.md`) |
| `peer_obstacles` | `true` | 로봇마다 다른 로봇의 몸을 costmap에 찍는다(`pinky_peer_obstacles`, `pinky_nav_config`의 `params/peer_obstacles.yaml`, 임시 구현): 다른 로봇이 보고한 자세에 반지름 0.08 m 원을 찍어 Nav2가 피해서 길을 짠다. 노드는 `sim_stand_in` 프로세스 안에서 돈다(프로세스를 늘리지 않는다). `false`면 찍지 않는다(비교용). 기본을 켠 근거는 작업 기록 11절(D2 세 판) |
| `nav_composition` | `true` | 로봇마다 Nav2 노드 9개를 프로세스 하나로. `false`면 DDS 참여자가 LOCALHOST 한도(약 32)를 넘어 pinky3이 안 보인다 |

`true`/`false` 인자에 오타가 있으면 launch가 바로 멈춘다.

### 도킹

- 기본(`sim_dock_stub:=false`): 진짜 `pinky_fsm`이 `/pinkyN/dock`을 받고 `DOCK_UNAVAILABLE`로 실패시킨다(`pinky_dock`이 아직 없다). 임무가 도크에서 멈추고 관제가 모두 세 번 시도한 뒤(`move_retry_limit`) 그 로봇을 격리한다. 주행 확인에는 이대로 충분하다.
- `sim_dock_stub:=true`: 임무를 끝까지 돌려 보고 싶을 때. `pinky_fsm`의 dock 서버를 `/pinkyN/dock_fsm`으로 비켜 두고 `sim_stand_in`이 `/pinkyN/dock`을 받는다. 로봇을 움직이지 않고, 관제가 데려다 놓은 로봇의 실제 위치가 도크 지점 0.15 m 안이면 시나리오의 `dock.duration_s` 뒤 성공, 아니면 실패로 끝낸다.
- 대역은 `use_sim_time`이 켜져 있고 격리 검사를 통과했을 때만 뜬다. Gazebo 시계(`/clock`)가 없거나 5 s(벽시계) 넘게 멈추면 도킹을 실패로 끝내고, 뜬 뒤 `use_sim_time`을 끄는 것도 거부한다. 실물 bringup(`iponic_bringup`, `pinky_fsm.launch.py`)에는 이 옵션이 없고, `pc.launch.py`에 `sim_dock_stub:=true`를 줘도 아무것도 바뀌지 않는다(`test/test_dock_stub.py`가 고정한다).

### 결과 확인

`result_dir`(기본 `./gz_result`)에 남는다.

- `fleet_check.json`: 10 s마다, 끝낼 때 한 번 더 쓴다. launch 로그에도 1분마다 한 줄(`[fleet_check]: min distance ...`).

  | 값 | 뜻 |
  |---|---|
  | `min_distance_m`, `min_distance_pair`, `min_distance_at_s` | 두 로봇 중심 사이 최소 거리(Gazebo 실제 위치, 0.1 s 안에 잰 표본끼리), 그 쌍, 시뮬 시각 |
  | `stops`, `stops_while_moving` | 0.05 m/s 넘게 달린 뒤 0.02 m/s 밑으로 0.5 s 넘게 선 횟수(제자리 회전은 정지가 아니다). 그중 로봇이 MOVING을 보고하던 중에 선 횟수 |
  | `outside_held_samples` / `zone_samples` | 구역을 쥔 로봇의 위치 표본 중 쥔 구역 바닥 밖인 수 / 관제가 구역을 낸 뒤의 전체 표본 수 |
  | `no_zone_held_samples` | 아무 구역도 쥐지 않은 동안의 표본 수(주차 칸에서 기다릴 때 등) |
  | `worst_outside_m` | 쥔 구역 바닥에서 가장 멀리 벗어난 거리 |
  | `sim_span_s`, `wall_span_s`, `real_time_factor` | 잰 시뮬 시간, 벽시계 시간, 그 비 |

- 시나리오가 끝나면(`monitor:=true`) `summary.json`(판정, 출고 수), `safety.json`, `events.csv`, `zones.csv`, `robots.csv` 등. 중간에 Ctrl-C로 끝내면 이것들은 남지 않는다. 그때는 launch 로그의 `[fleet_manager]` 줄(배정, `DOCKED`, `ROBOT_ISOLATED` 등)을 본다.
- 돌아가는 동안 다른 터미널(같은 환경)에서:

  ```bash
  ros2 topic echo /fleet/robots --once      # 로봇마다 상태, 작업, 고장
  ros2 topic echo /fleet/zones --once       # 구역마다 쥔 로봇
  ros2 run fleet_gz_sim fleet_check --ros-args -p output:=/tmp/check.json   # launch 밖에서 따로 잴 때
  ```

### 알려진 한계

- 무겁다: Gazebo + Nav2 3대 + 관제로 16코어 PC에서도 CPU를 많이 쓴다. 다른 무거운 작업과 같이 돌리지 않는다. 실시간 비율(`fleet_check.json`의 `real_time_factor`)이 1보다 많이 낮으면 Nav2 controller가 주기를 놓치고(`Control loop missed its desired rate`) 로봇이 고정 경로에서 벗어날 수 있다.
- DDS 참여자: LOCALHOST 탐색에서는 32개 정도만 서로 보인다. 노드를 더하면 `test_launch_wiring.py`의 프로세스 수 시험이 알려 준다. 돌아가는 동안 `ros2` 명령을 여러 개 띄우면 그 수도 들어간다.

## 핑키 한 대 (`one_pinky.launch.py`)

### 준비

위 "준비"와 같다.

### 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash   # pinky_description
source ~/fleet_ws/install/local_setup.bash
export ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch fleet_gz_sim one_pinky.launch.py                          # D2, P-1, Gazebo 화면 + RViz
ros2 launch fleet_gz_sim one_pinky.launch.py gui:=false rviz:=false   # 화면 없이
```

인자: `namespace`(pinky1), `layout`(d2), `start`(P-1, 지점 이름이고 방향은 그 지점의 yaw), `gui`, `rviz`, `fsm`, `heartbeat_timeout_s`(1.5), `nav_log_level`(warn), `nav_max_speed`(0.12), `keepout`(true), `still_pinky`(`x,y,yaw`를 주면 Nav2 없이 서 있기만 하는 `pinky_still`을 놓는다).

로봇이 IDLE이 되면(약 10 s) 다른 터미널에서 같은 환경으로 주행시킨다.

```bash
ros2 run fleet_gz_sim drive_check --layout d2 --plan stage2 --json stage2.json
ros2 run fleet_gz_sim drive_check --plan cross     # P-1 -> P-3 -> P-1 Nav2 goal(둘째 핑키 확인용)
```

`drive_check`는 관제 없이 로봇에 바로 이동을 보낸다(`fleet_gz_sim/stage2_plan.py`). `stage2`는 고정 선을 한 바퀴 도는 계획이다(추가지시서3 2-1): P-1에서 Nav2 goal로 N_W, `work_in`으로 W, 후진으로 N_W, 급행 동쪽으로 N_E, `out_in`으로 O, 후진으로 N_E, 급행 서쪽으로 N_W, Nav2 goal로 P-1. 고정 선의 꺾는 점도 경로에 넣는다. 결과는 `results/fixed_line_results.md`. 구간마다 결과, 걸린 시간(벽시계, 시뮬), 끝 위치와 방향 오차, 고정 경로에서 벗어난 최대 거리, Nav2 계획이 지난 keepout 구역을 적는다. `/fleet/heartbeat`는 2 Hz로 보낸다.

RViz의 2D Goal Pose로도 보낼 수 있다. 이것은 `bt_navigator`로 가고 `pinky_fsm`을 거치지 않는다.

### 구성

| 것 | 이름 |
|---|---|
| 월드 | `worlds/arena_<배치>.sdf`. 월드 좌표 = 지도 좌표 = 구역 지도 좌표. 안쪽 바닥 1.98 x 0.99 m를 벽이 두르고, 선반은 두께 1.5 cm, 높이 0.15 m 이상. 고정 경로는 바닥 그림 |
| 지도 | `maps/arena_<배치>.yaml`(5 cm, costmap), `maps/arena_<배치>_fine.yaml`(1 cm, AMCL), `maps/arena_<배치>_keepout.yaml`(1 cm, shared가 아닌 구역을 몸 반폭 0.06 m만큼 넓힘) |
| 로봇 모델 | `urdf/pinky_sim.urdf.xacro`: 제조사 모델 + DiffDrive, 라이다, 실제 위치. 제조사 `pinky_gz.urdf.xacro`의 램프 플러그인과 카메라는 뺐다 |
| 토픽 | `/pinky1/scan`, `/pinky1/odom`, `/pinky1/cmd_vel`, `/pinky1/joint_states`, `/pinky1/tf`, `/pinky1/tf_static`, `/clock` |
| 실제 위치 | `/pinky1/sim/truth`(`nav_msgs/Odometry`, map 좌표). 시험 도구만 읽는다 |
| 프레임 | `map`, `odom`, `base_footprint`, `rplidar_link`(접두사 없음, TF가 namespace로 나뉨) |

월드와 지도는 구역 지도에서 만든다(`fleet_gz_sim/arena.py`, `arena_maps.py`, `arena_world.py`). 구역 지도를 바꾸면 다시 만든다. 시험이 저장소 파일과 생성 결과가 같은지 본다.

```bash
cd sim/fleet_gz_sim && PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena            # 세 배치 모두
cd sim/fleet_gz_sim && PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena --layout d2
```
