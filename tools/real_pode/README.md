# 실물 Pode 한 대 실행 묶음 (v7 배치)

2026-11-10(free-floor 시절) 실물 시험의 묶음을 지금 패키지에 맞춰 옮긴 것이다. ROS 패키지가 아니라
그냥 폴더다. 로봇에서는 `~/nav2fleet_real/`에 두고, 패키지는 `~/nav2fleet_ws`에서 빌드한다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `real_pode.launch.py` | 제조사 bringup(`/<ns>` 안, `/tf` remap), `cmd_watchdog.py`, `pode_nav_config` `nav2.launch.py`, `pode_fsm`을 띄운다. 프로세스 하나라도 끝나면 전체를 내린다. 이 규칙은 프로세스만 본다: Nav2 노드는 컨테이너 한 프로세스 안의 컴포넌트라, 컴포넌트 하나가 뜨지 못해도 launch는 계속된다. 달리기 전에 기록에서 "Failed to load"나 lifecycle이 active가 되지 못한 노드가 없는지 본다 |
| `cmd_watchdog.py` | 움직이는 cmd_vel이 0.4 s 넘게 새로 오지 않으면 속도 0을 보낸다(0.05 s 타이머로 보므로 마지막 움직이는 명령에서 약 0.40~0.45 s 뒤). 제조사 bringup은 마지막 명령을 끝없이 지킨다 |
| `make_params.py --speed S --out DIR` | 저장소의 `nav2_params.yaml`과 `avoidance.yaml`을 베끼고 앞으로 가는 속도(FollowPath, FollowPathThrough, FollowPathGoal)만 S로 바꾼다. 0 < S ≤ 0.25를 반올림 전에 보고, 소수 넷째 자리로 반올림해 쓴다(반올림해서 0이 되는 값은 거부). FollowPathReverse(0.05)는 그대로. 또 기본으로 고정 경로 컨트롤러 셋(FollowPath, FollowPathThrough, FollowPathReverse)의 `use_collision_detection`을 false로 쓰고, `--fixed-line-check`를 주면 true로 쓴다(마당 goal FollowPathGoal의 검사는 원본 값 그대로다. 저장소 원본은 true) |
| `start.sh NS X Y YAW [SPEED]` | `params/`를 다시 만들고(SPEED 기본 0.12) launch를 뒤에서 띄운다. 기록은 `logs/launch_*.log`, 프로세스 그룹은 `logs/launch.pid`. `logs/start.lock`(flock)으로 동시에 두 번 시작하지 않는다. 5 s 안에 pid 파일이 생기지 않으면 그 launch에 TERM을 보내고 약 2 s 기다린 뒤 실패로 끝난다(그래도 남아 있으면 그렇다고 알린다. KILL은 쓰지 않는다) |
| `stop.sh` | start.sh가 띄운 그룹에 SIGTERM. 그룹 안에 `real_pode.launch.py` 명령이 없으면 내리지 않는다 |
| `env.sh` | 로봇용: ROS, `~/pinky_pro`, `~/nav2fleet_ws`를 읽고 통신 설정은 `net.sh`를 읽는다. `start.sh`가 이 파일을 읽으므로 로봇 명령은 그대로다 |
| `net.sh` | source 전용. 통신 설정을 한곳에 모은다. `env.sh`와 `pc_env.sh`가 둘 다 읽는다. `ROS_DOMAIN_ID=12`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`을 정하고, 먼저 `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `ROS_LOCALHOST_ONLY`, `ROS_STATIC_PEERS`를 지운다(다시 source해도 이 네 변수의 이전 값은 남지 않는다. `SUPER`, `DISCOVERY_SERVER`는 `SUPER=1 source ...`처럼 앞에 붙여 쓰고 `export`하지 않는다. export하면 다음 source에도 적용된다). `DISCOVERY_SERVER`로 서버 주소를 정한다(기본 `192.168.0.252:11811`, 관제 PC 유선 주소. 다른 주소는 `DISCOVERY_SERVER=IP:PORT`(포트를 바꾸면 `discovery_server.sh`는 늘 11811이므로 서버를 `fastdds discovery -i 0 -p PORT`로 직접 띄운다). `off`면 서버 없이 예전처럼 멀티캐스트). `SUPER=1`이면 `ROS_SUPER_CLIENT=TRUE`(ros2 명령, RViz처럼 전체 그래프를 봐야 하는 터미널용. `off`일 때는 무시). 도메인 번호는 팀이 아직 정하지 않았다(2026-11-19). 정해지면 `net.sh` 한 줄만 바꾼다 |
| `pc_env.sh` | PC 터미널에서 source한다. `/opt/ros/jazzy/setup.bash`와 PC 작업공간 `~/ros2camp/nav2fleet_ws/install/setup.bash`를 읽고(`ROS_SETUP`, `NAV2FLEET_WS_SETUP`로 바꿀 수 있다) `net.sh`를 읽는다. 관제 터미널은 `source tools/real_pode/pc_env.sh`, ros2 명령 터미널은 `SUPER=1 source tools/real_pode/pc_env.sh`. 작업공간 setup 파일이 없으면 경고만 내고 ROS만 읽은 채 계속한다. 그 경고가 보이면 PC 작업공간을 먼저 빌드한다(ROS setup이 없거나 실패, 작업공간 setup이 있는데 실패, `net.sh`를 못 읽으면 `return 1`). 새 bash 터미널에서 쓴다(PC 작업공간 setup이 `/opt/ros/jazzy`, `~/pinky_pro`, `~/ros2_ws`를 같이 읽어서, 다른 작업공간을 읽은 터미널이면 섞일 수 있다) |
| `discovery_server.sh` | PC에서 실행한다. ROS를 읽고 `fastdds discovery -i 0 -p 11811`을 앞에서 돌린다. 끌 때는 Ctrl+C(이 도구는 sh, python, 서버 프로그램이 겹쳐 있어서 PID 하나에 TERM을 보내면 서버가 남을 수 있다) |
| `rviz_pc.sh` | PC에서 `tools/real_pode/rviz_pc.sh pode1 pode2`로 RViz를 띄운다. 안에서 `SUPER=1`로 `pc_env.sh`를 읽는다. `DISCOVERY_SERVER=off tools/real_pode/rviz_pc.sh ...`도 된다 |
| `deploy.sh ROBOT_HOST` | PC에서: 커밋된 HEAD의 패키지를 `ROBOT:~/nav2fleet_ws/src/<저장소 경로>/`에, 이 폴더(지도 포함)를 `ROBOT:~/nav2fleet_real/`에 복사한다. 빌드와 실행은 하지 않는다 |
| `maps/` | `arena_real_slam`: 실물 경기장 SLAM 지도(2.5 cm, 2026-11-17 확정). AMCL(`map_fine`)의 기본이다. 나머지는 `sim/fleet_gz_sim/maps/`의 v7 지도 사본이다: `arena_v7_a`(전역 costmap 2.5 cm, 2026-11-17 결정), `arena_v7_keepout`, `arena_v7_fine`(1 cm 도면 지도, 이제 기본 아님). `arena_v7`(5 cm)은 비교용으로 남긴다 |

## 왜 속도 사본이 따로 있나

Gazebo는 `nav_max_speed`로 컨트롤러 속도를 낮추지만 실물 launch에는 그런 상한이 없다. 그리고 넓은 구역
Nav2 goal을 모는 컨트롤러(FollowPathGoal)의 속도는 `nav2_params.yaml`이 아니라 회피 담당의
`avoidance.yaml`에 있다. 그래서 천천히 달리는 첫 시험에는 두 사본이 모두 필요하다. `pode_fsm`의
`horizon_cap_mps`는 비워 두면 사본의 FollowPath 속도와 같게 둔다.

## 첫 실행 순서 (한 대, 0.12 m/s)

1. PC에서 `tools/real_pode/deploy.sh pinky@<로봇 주소>` (커밋된 것만 간다).
2. 로봇에서 `cd ~/nav2fleet_ws && colcon build`.
3. 로봇을 아는 자리에 놓는다. 예: P-1 = (0.12, 0.10), 북쪽(yaw 1.5708).
4. 로봇 옆에 들어 올릴 사람을 둔다.
5. PC에서 디스커버리 서버를 먼저 켜 둔다(`tools/real_pode/discovery_server.sh`). 기본 설정의 `start.sh`는 서버에 붙으므로 서버가 없으면 로봇과 PC가 서로 못 찾는다. 서버를 안 쓰려면 로봇과 PC 양쪽 모두 `DISCOVERY_SERVER=off`로 한다. 자세한 것은 아래 "디스커버리 서버 (기본)".
6. `~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708` (속도 0.12 기본). 기록을 본다:
   `tail -f ~/nav2fleet_real/logs/launch_*.log`.
7. PC에서 `source tools/real_pode/pc_env.sh`(도메인 12와 디스커버리 서버 설정이 들어 있다)로 관제를 띄워 시험한다. 아래 "디스커버리 서버 (기본)"을 본다.
8. 끝나면(또는 이상하면) `~/nav2fleet_real/stop.sh`.

### 옛 이름(pinky)으로 배포된 로봇에 다시 배포할 때 (2026-11-16 이름 변경)

- 배포 전에 옛 launch가 돌고 있지 않은지 본다. 새 `stop.sh`는 `real_pode.launch.py`만 찾으므로
  옛 `real_pinky.launch.py`는 내리지 못한다. 돌고 있으면 배포 전에 옛 `stop.sh`로 내린다.
- `deploy.sh`는 옛 `~/nav2fleet_ws/src/pinky/`와 `build/`, `install/`의 `pinky_*`를 지우지 않는다.
  옛 것은 `~/nav2fleet_ws` 밖의 보관 폴더로 옮기고 `build/`, `install/`을 비운 뒤 다시 빌드한다. 그러지 않으면 옛
  패키지가 함께 남는다.

## 디스커버리 서버 (기본)

실물 시험은 PC의 Fast DDS 디스커버리 서버로 서로를 찾는다. PC 명령의 `tools/real_pode/...`는 저장소 루트(`~/ros2camp/nav2fleet_ws/src/roscamp-nav2fleet`)에서 친다. 시험 날 순서는 이렇다.

1. PC: `tools/real_pode/discovery_server.sh`를 먼저 켜 둔다(또는 `source /opt/ros/jazzy/setup.bash` 뒤 `fastdds discovery -i 0 -p 11811`).
2. PC 관제: `source tools/real_pode/pc_env.sh` 뒤 평소처럼 `ros2 launch iponic_bringup pc.launch.py ...`.
3. 로봇: 예전 그대로 `~/nav2fleet_real/start.sh pode1 ...`.
4. RViz: `tools/real_pode/rviz_pc.sh pode1 pode2`.
5. 확인(ros2 명령 터미널): `SUPER=1 source tools/real_pode/pc_env.sh`, `ros2 daemon stop`, `ros2 daemon start`, 몇 초 뒤 `ros2 node list`, `ros2 topic echo /pode1/state --qos-reliability best_effort --once --timeout 10`.

### 서버 없이 돌리기

- 로봇은 `DISCOVERY_SERVER=off ~/nav2fleet_real/start.sh ...`, PC는 `DISCOVERY_SERVER=off source tools/real_pode/pc_env.sh`.
- 모드를 바꿀 때는 돌던 노드를 모두 다시 띄운다. 옛 설정 터미널에서 `ros2 daemon stop`을 한 뒤 새 터미널에서 daemon을 다시 띄운다.
- 서버를 끄기만 하면 이미 서로 찾은 노드는 계속 통신한다(2026-11-19 PC 한 대에서 본 결과, Wi-Fi와 로봇에서는 아직 확인하지 않았다). off로 돌아가려면 로봇을 `stop.sh` 뒤 다시 start한다.

### 알아 둘 것

아래 첫째와 둘째 항목의 시험 결과는 모두 2026-11-19 PC 한 대에서 본 것이다. Wi-Fi와 로봇에서는 아직 확인하지 않았다.

- 서버가 꺼진 동안 새로 뜨거나 다시 뜬 노드는 상대를 못 찾는다. 서버를 다시 켜면 회복한다. 늦게 켠 서버에도 붙는다.
- 서버 아래에서는 도메인 번호로 갈라지지 않는다. 같은 서버에 붙은 다른 도메인끼리도 보인다.
- `ROS_DISCOVERY_SERVER`를 `~/.bashrc`에 넣지 않는다. Gazebo 시뮬은 이 변수가 있으면 띄우지 않는다(`sim/fleet_gz_sim/fleet_gz_sim/isolation.py`의 `FAST_DDS_VARIABLES`).
- 로봇에서 ros2 명령을 손으로 칠 때는 먼저 `source ~/nav2fleet_real/env.sh`.
- `deploy.sh`는 패키지도 보내므로 배포 뒤 로봇마다 `cd ~/nav2fleet_ws && colcon build`(위 "첫 실행 순서" 2번).

## Gazebo와 다른 실물 기본값

- peer 층 꺼짐(`peer_obstacles:=false`). 두 대 이상이면 `PEER_ROBOTS=pode1,pode2 ./start.sh pode1 ...`(start.sh가 `peer_obstacles:=true peer_robots:=...`를 붙인다). 두 로봇 모두 같은 목록(두 이름 다)을 줘도 된다: 자기 이름은 건너뛴다(`pode_peer_obstacles/body.py:19`, `node.py:60`).
- 고정 경로 충돌 검사 꺼짐(시뮬 기본과 같다, 사용자 결정 2026-11-18). `make_params.py`가 `params/nav2_params.yaml`의
  FollowPath, FollowPathThrough, FollowPathReverse에 `use_collision_detection: false`를 쓴다. 마당 goal(FollowPathGoal,
  `avoidance.yaml`)의 검사는 원본 값 그대로다(저장소 원본은 켬). 켜 두려면 `FIXED_LINE_CHECK=1 ./start.sh ...`(`--fixed-line-check`). 안 주거나 `0`이면 끔,
  다른 값은 start.sh가 거절한다(종료 2). 위험: 고정 경로에서는 앞의 충돌을 미리 알아채는 곳이 없어진다. 로봇 사이 간격은 관제에 맡긴다.
- 속도 상한 없음: `params/` 사본의 값이 곧 최고 속도다.
- keepout은 `keepout_mask`를 줄 때만 켜진다. 이 묶음은 기본으로 `maps/arena_v7_keepout.yaml`을 준다.
- 스캔 거르개 꺼짐(`narrow_strips` 비움).
- 도킹 대역: `DOCK_STUB=1 ./start.sh ...`(`dock_aside:=true`)면 `pode_fsm`의 Dock 서버가
  `/<ns>/dock_fsm`으로 비킨다. 이때 PC에서 도킹 대역 노드가 `/<ns>/dock`을 맡아야 한다. 없으면 도킹
  goal을 받는 서버가 없다. 안 주거나 `0`이면 아래 "도킹 없음"과 같다. 다른 값은 start.sh가 거절한다(종료 2).
- 도킹 없음(DOCK_STUB 없이): `pode_fsm` dock()은 DOCK_UNAVAILABLE을 돌려준다. 관제는 도킹을 3번 다시 시도한 뒤 WORK
  도크 구역을 쥔 채로 로봇을 격리한다. 시험은 거기서 멈춘다.
- 배터리 토픽 비움(`battery_topic`): 배터리를 모름(-1)으로 알린다.

## 알려진 한계

- `stop.sh`는 그룹 안 어느 프로세스의 명령줄에 `real_pode.launch.py`가 있는지만 본다. pid가 다시 쓰인 그룹에 그 글자가 든 다른 명령이 있으면 그 그룹도 내린다.
- `deploy.sh`의 `rsync --delete`는 경로 글자만 패키지 폴더로 묶는다. 로봇의 `~/nav2fleet_ws/src/pode`나 패키지 폴더가 다른 곳을 가리키는 심볼릭 링크면 그곳에서 지운다.
- 위 시간 초과에서 launch가 pid 파일을 쓰기 전에 죽고 그 번호가 2 s 안에 다른 프로세스에 다시 쓰이면 그 프로세스가 TERM을 받을 수 있다(드물다).

## 시험

```bash
python3 -m pytest -q tools/test/test_real_pode.py   # ROS 필요 없음
```
