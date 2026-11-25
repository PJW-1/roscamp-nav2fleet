# 실물 Pode 한 대 실행 묶음 (v7 배치)

2026-11-10(free-floor 시절) 실물 시험의 묶음을 지금 패키지에 맞춰 옮긴 것이다. ROS 패키지가 아니라
그냥 폴더다. 로봇에서는 `~/nav2fleet_real/`에 두고, 패키지는 `~/nav2fleet_ws`에서 빌드한다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `real_pode.launch.py` | 제조사 bringup(`/<ns>` 안, `/tf` remap), `cmd_watchdog.py`, `pode_nav_config` `nav2.launch.py`, `pode_fsm`을 띄운다. 프로세스 하나라도 끝나면 전체를 내린다. 이 규칙은 프로세스만 본다: Nav2 노드는 컨테이너 한 프로세스 안의 컴포넌트라, 컴포넌트 하나가 뜨지 못해도 launch는 계속된다. 달리기 전에 기록에서 "Failed to load"나 lifecycle이 active가 되지 못한 노드가 없는지 본다 |
| `cmd_watchdog.py` | 움직이는 cmd_vel이 0.4 s 넘게 새로 오지 않으면 속도 0을 보낸다(0.05 s 타이머로 보므로 마지막 움직이는 명령에서 약 0.40~0.45 s 뒤). 제조사 bringup은 마지막 명령을 끝없이 지킨다 |
| `make_params.py --speed S --out DIR` | 저장소의 `nav2_params.yaml`과 `avoidance.yaml`을 베끼고 앞으로 가는 속도(FollowPath, FollowPathThrough, FollowPathGoal)만 S로 바꾼다. 0 < S ≤ 단일 출처(`pode_nav_config/params/nav_speed.yaml`, 0.12)를 반올림 전에 보고, 소수 넷째 자리로 반올림해 쓴다(반올림해서 0이 되는 값은 거부). 비교용 MPPI(FollowPathMPPI)는 실물 controller 목록에 없어 뜨지 않는다(로봇에 nav2_mppi_controller가 없어도 Nav2가 뜬다, 사용자 결정 2026-11-22). FollowPathReverse(0.05)는 그대로. 또 기본으로 고정 경로 컨트롤러 셋(FollowPath, FollowPathThrough, FollowPathReverse)의 `use_collision_detection`을 false로 쓰고, `--fixed-line-check`를 주면 true로 쓴다(마당 goal FollowPathGoal의 검사는 원본 값 그대로다. 저장소 원본은 true) |
| `start.sh NS X Y YAW [SPEED]` | `params/`를 다시 만들고(SPEED 기본 0.12) launch를 뒤에서 띄운다. 기록은 `logs/launch_*.log`, 프로세스 그룹은 `logs/launch.pid`. `logs/start.lock`(flock)으로 동시에 두 번 시작하지 않는다. 5 s 안에 pid 파일이 생기지 않으면 그 launch에 TERM을 보내고 약 2 s 기다린 뒤 실패로 끝난다(그래도 남아 있으면 그렇다고 알린다. KILL은 쓰지 않는다). `PLAN_KEEP=1 start.sh ...`이면 Nav2 goal 길 유지 노드(`pode_plan_keep`, 경로 뒤집힘 해결안 A)와 `_goal_keep` 행동 트리를 쓴다. 기본은 꺼짐이고, 켜면 로봇마다 프로세스 하나(DDS 참여자 하나)가 늘고 그 노드가 전역 비용 지도를 받는다 |
| `stop.sh` | start.sh가 띄운 그룹에 SIGTERM. 그룹 안에 `real_pode.launch.py` 명령이나 동작 없음(`NO_MOTION=1`)의 `pode_fsm.launch.py` 명령이 없으면 내리지 않는다 |
| `bag_record.sh start NS NAME` / `bag_record.sh stop` | 로봇 SD 카드에 rosbag(mcap)을 `bags/NAME`으로 남긴다. 토픽은 `/NS/` 아래 scan, odom, cmd_vel, amcl_pose, particle_cloud, plan, tf, tf_static, state. 새 토픽을 잡는 데 몇 초 걸리므로 `start.sh`보다 먼저 켠다. 녹화기가 bag 파일을 열 때까지 `BAG_START_TIMEOUT_S`초(기본 20, 양의 정수) 기다리고 못 열면 멈춘다. stop은 SIGINT로 bag을 마무리한다. 실물 판마다 켠다(2026-11-23: 이것이 없어서 벽 충돌 뒤 위치를 잃은 원인을 가르지 못했다) |
| `env.sh` | 로봇용: ROS(`ROS_SETUP`, 기본 `/opt/ros/jazzy/setup.bash`), `~/pinky_pro`, `~/nav2fleet_ws`를 읽고 통신 설정은 `net.sh`를 읽는다. `start.sh`가 이 파일을 읽으므로 로봇 명령은 그대로다 |
| `net.sh` | source 전용. 통신 설정을 한곳에 모은다. `env.sh`와 `pc_env.sh`가 둘 다 읽는다. `ROS_DOMAIN_ID=6`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`을 정하고, 먼저 `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `ROS_LOCALHOST_ONLY`, `ROS_STATIC_PEERS`를 지운다(다시 source해도 이 네 변수의 이전 값은 남지 않는다. `SUPER`, `DISCOVERY_SERVER`는 `SUPER=1 source ...`처럼 앞에 붙여 쓰고 `export`하지 않는다. export하면 다음 source에도 적용된다). `DISCOVERY_SERVER`로 서버 주소를 정한다(기본 `192.168.0.252:11811`, 관제 PC 유선 주소. 다른 주소는 `DISCOVERY_SERVER=IP:PORT`(포트를 바꾸면 `discovery_server.sh`는 늘 11811이므로 서버를 `fastdds discovery -i 0 -p PORT`로 직접 띄운다). `off`면 서버 없이 예전처럼 멀티캐스트). `SUPER=1`이면 `ROS_SUPER_CLIENT=TRUE`(ros2 명령, RViz처럼 전체 그래프를 봐야 하는 터미널용. `off`일 때는 무시). 도메인 번호는 팀 통신 규칙 IR-81의 6이다(2026-11-24에 12에서 바꿈). 바꿀 때는 `net.sh` 한 줄만 바꾼다 |
| `pc_env.sh` | PC 터미널에서 source한다. `/opt/ros/jazzy/setup.bash`와 PC 작업공간 `~/ros2camp/nav2fleet_ws/install/setup.bash`를 읽고(`ROS_SETUP`, `NAV2FLEET_WS_SETUP`로 바꿀 수 있다) `net.sh`를 읽는다. 관제 터미널은 `source tools/real_pode/pc_env.sh`, ros2 명령 터미널은 `SUPER=1 source tools/real_pode/pc_env.sh`. 작업공간 setup 파일이 없으면 경고만 내고 ROS만 읽은 채 계속한다. 그 경고가 보이면 PC 작업공간을 먼저 빌드한다(ROS setup이 없거나 실패, 작업공간 setup이 있는데 실패, `net.sh`를 못 읽으면 `return 1`). 새 bash 터미널에서 쓴다(PC 작업공간 setup이 `/opt/ros/jazzy`, `~/pinky_pro`, `~/ros2_ws`를 같이 읽어서, 다른 작업공간을 읽은 터미널이면 섞일 수 있다) |
| `discovery_server.sh` | PC에서 실행한다. ROS를 읽고 `fastdds discovery -i 0 -p 11811`을 앞에서 돌린다. 끌 때는 Ctrl+C(이 도구는 sh, python, 서버 프로그램이 겹쳐 있어서 PID 하나에 TERM을 보내면 서버가 남을 수 있다) |
| `rviz_pc.sh` | PC에서 `tools/real_pode/rviz_pc.sh pode1 pode2`로 RViz를 띄운다. 안에서 `SUPER=1`로 `pc_env.sh`를 읽는다. `DISCOVERY_SERVER=off tools/real_pode/rviz_pc.sh ...`도 된다. `RVIZ_MODE`로 창을 고른다: 비우면 `all`(전체 창과 로봇별 창), `overview`는 전체 창만, `robots`는 로봇별 창만 |
| `deploy.sh ROBOT_HOST` | PC에서: 커밋된 HEAD의 패키지를 `ROBOT:~/nav2fleet_ws/src/<저장소 경로>/`에, 이 폴더(지도 포함)를 `ROBOT:~/nav2fleet_real/`에 복사한다. 빌드와 실행은 하지 않는다 |
| `maps/` | `arena_real_slam`: 실물 경기장 SLAM 지도(0.5 cm, 2026-11-17 확정). AMCL(`map_fine`)의 기본이다. 나머지는 실측 진열대로 그린 v7 지도다(`python3 -m fleet_gz_sim.arena --layout v7 --real --out /tmp/real_v7`로 만들고 `/tmp/real_v7/maps/`의 지도를 복사한다): `arena_v7_a`(전역 costmap 2.5 cm, 2026-11-17 결정), `arena_v7_keepout`, `arena_v7_fine`(1 cm 도면 지도, 이제 기본 아님, AMCL 후보), `arena_v7_a1`(1 cm costmap 후보, `arena_v7_a`와 같은 칠하기 규칙이라 그림은 `arena_v7_fine`과 같다. 2026-11-24 추가, launch 기본값은 그대로). `arena_v7`(5 cm)은 비교용으로 남긴다. 진열대는 11-24 줄자 값(A 서쪽 끝 0.65, 북쪽 면 0.775(lidar 높이), 남쪽 서쪽 끝 0.70)으로 그렸다(`docs/report_2026-11-24_kit_maps.md`). 세 진열대 모두 두께 4 cm이고 아래 통로 둘은 각 0.3475 m다(A y 0.735~0.775, 남쪽 둘 y 0.3475~0.3875, `docs/report_2026-11-25_shelf_depth.md`) |

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
6. 먼저 bag을 켠다: `~/nav2fleet_real/bag_record.sh start pode1 <판 이름>`(새 토픽을 잡는 데 몇 초 걸린다). 그다음
   `~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708` (속도 0.12 기본). 기록을 본다:
   `tail -f ~/nav2fleet_real/logs/launch_*.log`.
7. PC에서 `source tools/real_pode/pc_env.sh`(도메인 6과 디스커버리 서버 설정이 들어 있다)로 관제를 띄워 시험한다. 아래 "디스커버리 서버 (기본)"을 본다.
8. 끝나면(또는 이상하면) `~/nav2fleet_real/stop.sh`, 그다음 `~/nav2fleet_real/bag_record.sh stop`(bag을 마무리한다).

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

- peer 층 꺼짐(`peer_obstacles:=false`). 두 대 이상이면 `PEER_ROBOTS=pode1,pode2 ./start.sh pode1 ...`(start.sh가 `peer_obstacles:=true peer_robots:=...`를 붙인다). 두 로봇 모두 같은 목록(두 이름 다)을 줘도 된다: 자기 이름은 건너뛴다(`pode_peer_obstacles/body.py`의 `others()`, `node.py`의 `PeerObstacles.__init__`).
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
- 동작 없음(설계 4.5): `NO_MOTION=1 ./start.sh pode2`는 bringup과 Nav2 없이 `pode_fsm`만 `no_motion:=true`로
  띄운다. 바퀴는 돌지 않고, 목표는 정해진 시간 뒤 성공한다. 자세를 안 주면 로봇별 주차 자리(v7 P-1~P-3)를 쓰고,
  `NO_MOTION=1 ./start.sh pode2 X Y YAW`로 바꿀 수 있다. `PEER_ROBOTS`, `DOCK_STUB`, `FIXED_LINE_CHECK`, `PLAN_KEEP`과 함께
  주면 거절한다(종료 2). 내릴 때는 `./stop.sh`.

## 알려진 한계

- `stop.sh`는 그룹 안 어느 프로세스의 명령줄에 `real_pode.launch.py`나, `NO_MOTION=1 start.sh`가 쓰는 인자 순서의 `pode_fsm.launch.py namespace:=… no_motion:=true initial_x:=…`가 있는지만 본다. pid가 다시 쓰인 그룹에 그런 명령이 있으면 그 그룹도 내린다.
- `deploy.sh`의 `rsync --delete`는 경로 글자만 패키지 폴더로 묶는다. 로봇의 `~/nav2fleet_ws/src/pode`나 패키지 폴더가 다른 곳을 가리키는 심볼릭 링크면 그곳에서 지운다.
- 위 시간 초과에서 launch가 pid 파일을 쓰기 전에 죽고 그 번호가 2 s 안에 다른 프로세스에 다시 쓰이면 그 프로세스가 TERM을 받을 수 있다(드물다).

## 시험

```bash
python3 -m pytest -q tools/test/test_real_pode.py   # ROS 필요 없음
python3 -m pytest -q tools/test/test_step_probe.py  # 응답 측정 도구. rclpy가 있으면 도메인 63에서 안전 장치도 본다
```

## 응답 측정 도구 (sim2real, 2026-11-21)

Isaac Lab의 로봇 응답과 무작위화 범위를 정하려고 실물 Pode의 cmd_vel 응답을 잰다(계획 문서 5-3 가).

| 파일 | 하는 일 |
|---|---|
| `cmd_step_probe.py` | Nav2 없이 `/<ns>/cmd_vel`에 계단 입력을 20 Hz로 내고, 보낸 명령과 `/<ns>/odom`(속도, 자세)을 CSV에 적는다(명령마다, odom마다 한 줄, monotonic과 ROS 시각). 같은 이름 뒤에 `.json`을 붙인 파일에 인자, 일정, 끝난 까닭(exit)을 적는다 |
| `step_report.py` | probe CSV로 `report.md`와 그림(PNG)을 만든다. 계단마다 지연(명령에서 odom이 목표의 10%에 닿기까지), 상승 시간(10→90%), 정상 비(odom÷명령, 계단 뒤쪽 절반 평균), 최대 가속·감속. 작은 회전에서 데드밴드(평균 비 0.5 이상인 가장 작은 ω). 원호에서 실제 반지름(odom 궤적의 원 맞춤, odom v÷ω)과 명령 반지름 v÷ω, 왼쪽·오른쪽 차이 |
| `path_overlay.py` | `record`: Nav2와 관제가 모는 동안 `/<ns>/odom`, `/<ns>/amcl_pose`, `/<ns>/tf`의 map→odom을 CSV에 적는다. `overlay`: 기록을 v7 계획 경로 W→W_X→O_X→O(구역 지도 `fleet_map_v7.yaml`에서 읽음) 위에 그려 PNG로 저장하고, 모서리마다 바깥쪽으로 가장 많이 벗어난 거리(cm)와 경로 어디서든 가장 큰 옆 거리를 낸다. map→odom이 있으면 odom 자세를 그것으로 옮긴 촘촘한 궤적을, 없으면 amcl_pose를 쓴다 |
| `scan_info.py` | `/<ns>/scan`을 몇 초 받아 빔 수, 각도 범위, 거리 범위, 주기(헤더 시각으로 셈), 유효 거리 비율을 JSON으로 낸다. Gazebo 기준값도 같이 적는다 |
| `bringup_only.launch.py` | Nav2·pode_fsm 없이 제조사 bringup(`/<ns>` 안)과 `cmd_watchdog.py`만 띄운다. probe는 이것과 함께 쓴다 |
| `gz_probe.launch.py`, `gz_step_probe.sh` | Gazebo Pode 한 대(Nav2 없음, 경기장 밖 빈 바닥 (5, 5))에 probe 네 묶음을 모두 돌리고 보고서를 `results/<날짜>/step_probe_gz/`에 쓴다 |

### probe 묶음과 거리 안전

한 번 실행에 한 묶음(`--set`)만 돈다. 각 묶음은 3번 되풀이하고(`--repeats`), 계단마다 정지 2 s → 명령 → 정지 2 s → 출발 자리로 돌아오기 순서다. 돌아오기는 odom으로 닫힌 고리다: 출발 자리가 뒤에 있으면 후진, 0.10 m/s 이하, 방향을 먼저 맞추고(0.3~0.6 rad/s), 3 cm 안에 들면 처음 방향으로 돌아선다. 돌아오는 동안 출발 자리까지의 거리가 그 돌아오기에서 가장 가까웠던 거리보다 5 cm 넘게 늘면 바로 멈춘다(종료 6). 그래서 돌아오기는 그 계단이 끝난 자리보다 5 cm 넘게 더 멀어지지 않는다(거리 한도 `--limit`도 계속 걸린다). 돌아오기 줄은 `phase=home`이고 분석에서 뺀다(`--no-home`으로 끈다).

| `--set` | 명령 | 한 계단의 가장 먼 거리 |
|---|---|---|
| `linear` | v 0.05, 0.10, 0.15, 0.20, 0.25 m/s, 각 2 s | 0.50 m |
| `angular` | ω 0.5, 1.0, 1.5 rad/s, 각 2 s | 제자리 |
| `small_rot` | ω 0.05, 0.1, 0.2, 0.3 rad/s를 v 0과 v 0.12에서, 각 3 s(`--small-s`) | 0.36 m |
| `arc` | v 0.10, ω ±0.5(R 0.20 m) 왼쪽·오른쪽, 각 4 s(`--arc-s`, `--arc-v`, `--arc-w`) | 0.34 m |

안전 장치:

- odom 자세가 출발 자리에서 `--limit`(기본 0.6 m, 직선 거리)보다 멀어지면 바로 멈춘다(종료 4). 일정 안의 계단이 한도를 넘을 수 있으면 시작하지 않는다(종료 3). `--dry-run`은 일정과 계단마다 가장 먼 거리만 찍는다.
- 시작 전에(자기 publisher를 만들기 전에) cmd_vel의 publisher를 센다. `cmd_watchdog` 말고 다른 publisher(Nav2 controller, 이미 돌고 있는 다른 probe 등)가 있으면 시작하지 않는다(종료 3, `--allow`로 이름을 더할 수 있다. 허용은 노드 이름으로만 본다). 자기 publisher를 만든 뒤 첫 명령 전에 한 번 더 세어 새 publisher가 있으면 시작하지 않는다(종료 3). 도는 중에는 자기 publisher 하나만(이름이 아니라 endpoint gid로) 빼고 세며, 새 publisher가 생기면 멈춘다(종료 4). probe 노드 이름은 `cmd_step_probe_<pid>`로 실행마다 다르다.
- odom이 1 s(`--odom-timeout`) 넘게 안 오면 멈춘다. 돌아오기가 30 s(`--home-s`) 안에 안 끝나면 멈춘다.
- odom의 속도, 위치, 방향 쿼터니언에 NaN·inf가 하나라도 오면 멈춘다(종료 5). 유한하지 않은 명령은 보내지 않는다.
- Ctrl+C(종료 130), SIGTERM, 안전 정지, 오류 등 어떻게 끝나든 속도 0을 5번 보낸다. 다섯 번을 먼저 다 보내고 CSV에는 그 뒤에 적으며(CSV 쓰기가 실패해도 0은 다 나간다), Ctrl+C·SIGTERM은 처음 한 번만 멈춤으로 받고, 그 뒤(정리하는 동안, 안전 정지로 끝나는 중에도)는 무시한다. 프로세스가 통째로 죽어도 `cmd_watchdog`이 0.4 s 뒤 0을 보낸다.

### 실물에서 재는 순서 (한 대)

1. 넓고 빈 곳(사방 1 m 이상, 경기장 밖 평평한 바닥)에 로봇을 놓는다. 옆에 들어 올릴 사람을 둔다. 배터리를 채운다.
2. 로봇에 이 폴더가 있어야 한다(`deploy.sh`로 보냄). Nav2 묶음이 돌고 있으면 `~/nav2fleet_real/stop.sh`로 내린다.
3. 로봇 터미널 1(Nav2 없이): `source ~/nav2fleet_real/env.sh && ros2 launch ~/nav2fleet_real/bringup_only.launch.py namespace:=pode1`
4. 로봇 터미널 2(probe는 로봇에서 돌린다. 실제 운용에서도 Nav2가 로봇 안에서 명령을 내므로 Wi-Fi 지연을 섞지 않는다):
   ```bash
   source ~/nav2fleet_real/env.sh && cd ~/nav2fleet_real && mkdir -p probe
   python3 cmd_step_probe.py --set linear --dry-run          # 일정 확인
   python3 cmd_step_probe.py --set linear --out probe/step_linear.csv
   python3 cmd_step_probe.py --set angular --out probe/step_angular.csv
   python3 cmd_step_probe.py --set small_rot --out probe/step_small_rot.csv
   python3 cmd_step_probe.py --set arc --out probe/step_arc.csv
   python3 scan_info.py --seconds 10 --out probe/scan_real.json
   ```
   묶음 사이에 로봇을 빈 곳 가운데로 다시 옮겨도 된다(출발 자리는 실행마다 새로 잡는다).
5. PC로 가져와 보고서를 만든다: `scp -r pinky@<로봇 주소>:~/nav2fleet_real/probe results/<날짜>/step_probe_real` 뒤
   `python3 tools/real_pode/step_report.py results/<날짜>/step_probe_real/step_*.csv --out results/<날짜>/step_probe_real`.
6. 경로 겹쳐 보기(Nav2 켬, 관제가 W→W_X→O_X→O로 몰 때): 평소대로 `start.sh`로 띄우고 PC에서
   `SUPER=1 source tools/real_pode/pc_env.sh && python3 tools/real_pode/path_overlay.py record --ns pode1 --out results/<날짜>/path_pode1.csv`
   (Ctrl+C로 끝냄), 그 뒤 `python3 tools/real_pode/path_overlay.py overlay results/<날짜>/path_pode1.csv`(PNG와 `_overlay.txt`가 옆에 생긴다).

주의:

- 실물 odom 속도는 제조사 bringup이 30 Hz마다 엔코더 차이로 계산한 값이다(바퀴 미끄러짐은 안 보인다). 바퀴 최고 100 rpm(약 0.28 m/s)에서 명령이 비율대로 잘린다.
- `small_rot`은 양(+)의 ω만 잰다.
- 실물 토픽 이름(`/<ns>/cmd_vel`, `/<ns>/odom`, `/<ns>/scan`, `/<ns>/amcl_pose`, `/<ns>/tf`)은 제조사 코드와 launch에서 읽은 것이고 실물에서 아직 확인하지 않았다. 처음에 `ros2 topic list`로 본다.

### Gazebo에서 같은 측정

작업공간을 빌드하고 source한 셸에서(도메인은 43~101, 6·12·48·50~59·61은 안 됨):

```bash
export ROS_DOMAIN_ID=63 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
tools/real_pode/gz_step_probe.sh                 # 화면 없이, 네 묶음 모두, results/<날짜>/step_probe_gz/
GUI=true SETS="linear" PROBE_ARGS="--repeats 1" tools/real_pode/gz_step_probe.sh   # 짧게, 화면 켬
```

probe는 `--use-sim-time`으로 Gazebo 시계에 맞춰 돈다. 다른 무거운 작업이 돌면 실시간 비율이 떨어져 걸리는 시간이 늘고, 메시지 전달 지연이 시뮬 시각으로 커 보일 수 있으므로 PC가 한가할 때 잰다. Gazebo 모델은 DiffDrive 속도 ±1.0, 가속 ±2.0(이 상한은 선속도 m/s와 각속도 rad/s 모두에 걸린다. 그래서 angular 1.5 계단은 1.0에서 멈춘다), odom 30 Hz이고, 라이다 기준값은 640빔, 360°, 0.05~12 m, 10 Hz다(`sim/fleet_gz_sim/urdf/pode_sim.urdf.xacro`). 실물 값은 `scan_info.py`로 잰다.
