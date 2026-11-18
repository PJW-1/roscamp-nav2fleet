# 실물 Pode 한 대 실행 묶음 (v7 배치)

2026-11-10(free-floor 시절) 실물 시험의 묶음을 지금 패키지에 맞춰 옮긴 것이다. ROS 패키지가 아니라
그냥 폴더다. 로봇에서는 `~/nav2fleet_real/`에 두고, 패키지는 `~/nav2fleet_ws`에서 빌드한다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `real_pode.launch.py` | 제조사 bringup(`/<ns>` 안, `/tf` remap), `cmd_watchdog.py`, `pode_nav_config` `nav2.launch.py`, `pode_fsm`을 띄운다. 프로세스 하나라도 끝나면 전체를 내린다. 이 규칙은 프로세스만 본다: Nav2 노드는 컨테이너 한 프로세스 안의 컴포넌트라, 컴포넌트 하나가 뜨지 못해도 launch는 계속된다. 달리기 전에 기록에서 "Failed to load"나 lifecycle이 active가 되지 못한 노드가 없는지 본다 |
| `cmd_watchdog.py` | 움직이는 cmd_vel이 0.4 s 넘게 새로 오지 않으면 속도 0을 보낸다(0.05 s 타이머로 보므로 마지막 움직이는 명령에서 약 0.40~0.45 s 뒤). 제조사 bringup은 마지막 명령을 끝없이 지킨다 |
| `make_params.py --speed S --out DIR` | 저장소의 `nav2_params.yaml`과 `avoidance.yaml`을 베끼고 앞으로 가는 속도(FollowPath, FollowPathThrough, FollowPathGoal)만 S로 바꾼다. 0 < S ≤ 0.25를 반올림 전에 보고, 소수 넷째 자리로 반올림해 쓴다(반올림해서 0이 되는 값은 거부). FollowPathReverse(0.05)는 그대로 |
| `start.sh NS X Y YAW [SPEED]` | `params/`를 다시 만들고(SPEED 기본 0.12) launch를 뒤에서 띄운다. 기록은 `logs/launch_*.log`, 프로세스 그룹은 `logs/launch.pid`. `logs/start.lock`(flock)으로 동시에 두 번 시작하지 않는다. 5 s 안에 pid 파일이 생기지 않으면 그 launch에 TERM을 보내고 약 2 s 기다린 뒤 실패로 끝난다(그래도 남아 있으면 그렇다고 알린다. KILL은 쓰지 않는다) |
| `stop.sh` | start.sh가 띄운 그룹에 SIGTERM. 그룹 안에 `real_pode.launch.py` 명령이 없으면 내리지 않는다 |
| `env.sh` | ROS, `~/pinky_pro`, `~/nav2fleet_ws`를 읽고 `ROS_DOMAIN_ID=12`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (11-10에 로봇과 PC가 쓴 값) |
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
5. `~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708` (속도 0.12 기본). 기록을 본다:
   `tail -f ~/nav2fleet_real/logs/launch_*.log`.
6. PC에서 같은 도메인(12)으로 관제를 띄워 시험한다.
7. 끝나면(또는 이상하면) `~/nav2fleet_real/stop.sh`.

### 옛 이름(pinky)으로 배포된 로봇에 다시 배포할 때 (2026-11-16 이름 변경)

- 배포 전에 옛 launch가 돌고 있지 않은지 본다. 새 `stop.sh`는 `real_pode.launch.py`만 찾으므로
  옛 `real_pinky.launch.py`는 내리지 못한다. 돌고 있으면 배포 전에 옛 `stop.sh`로 내린다.
- `deploy.sh`는 옛 `~/nav2fleet_ws/src/pinky/`와 `build/`, `install/`의 `pinky_*`를 지우지 않는다.
  옛 것은 `~/nav2fleet_ws` 밖의 보관 폴더로 옮기고 `build/`, `install/`을 비운 뒤 다시 빌드한다. 그러지 않으면 옛
  패키지가 함께 남는다.

## Gazebo와 다른 실물 기본값

- peer 층 꺼짐(`peer_obstacles:=false`). 두 대 이상이면 `PEER_ROBOTS=pode1,pode2 ./start.sh pode1 ...`(start.sh가 `peer_obstacles:=true peer_robots:=...`를 붙인다). 두 로봇 모두 같은 목록(두 이름 다)을 줘도 된다: 자기 이름은 건너뛴다(`pode_peer_obstacles/body.py:19`, `node.py:60`).
- 고정 경로 충돌 검사 켬(저장소 값 그대로. Gazebo의 `fixed_line_collision_check` 같은 끄기는 없다).
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
