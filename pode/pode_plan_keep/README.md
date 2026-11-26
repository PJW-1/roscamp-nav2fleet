# pode_plan_keep

Nav2 goal(MoveTo mode `nav_goal`)의 행동 트리와 planner 사이에서 길을 지키는 노드 `plan_keep`이 있습니다. 행동 트리는 1 Hz로 다시 계획하는데, 길이가 비슷한 두 길이 있으면 NavFn이 1초마다 두 길을 번갈아 내서 로봇이 제자리에서 방향만 바꿨습니다(Gazebo 장애물 판, 목적지 Q2, 두 길 0.93~1.07 m). 이 노드는 그런 때 지금 가던 길을 그대로 돌려줍니다.

| 파일 | 내용 |
|---|---|
| `pode_plan_keep/route.py` | 길 유지 규칙(순수 함수). ROS 없이 시험합니다 |
| `pode_plan_keep/node.py` | action server `compute_path_keep`(`nav2_msgs/action/ComputePathToPose`). 요청마다 planner의 `compute_path_to_pose`를 부르고 규칙으로 고른 경로를 돌려줍니다 |

## 규칙

- 지켜 둔 경로가 없거나, goal 위치가 지켜 둔 경로의 goal과 0.05 m 이상 다르거나, planner가 실패하면 planner의 결과를 그대로 돌려줍니다. 실패면 오류 코드도 그대로 돌려주고 지켜 둔 경로를 지웁니다.
- 지켜 둔 경로에서 새 경로의 첫 점(로봇 위치)에 가장 가까운 점부터 끝까지를 남은 길로 봅니다.
- 새 경로의 모든 점이 남은 길에서 `same_route_m`(0.20 m) 안이면 같은 길로 보고 새 경로를 돌려주며 그것을 지켜 둡니다.
- 다른 길이면 남은 길이 막혔는지 planner(NavFn)와 같은 기준으로 봅니다. 노드는 전역 비용 지도 `global_costmap/costmap`(`OccupancyGrid`, 1 Hz 전체 격자)을 받아 두고, 남은 길의 점(끝 점은 빼고)이 놓인 칸 가운데 값이 `blocked_cost`(99) 이상인 칸이 있으면 막힌 것으로 봅니다. 99는 INSCRIBED(253), 100은 LETHAL(254)이 바뀐 값이고 NavFn은 253 이상을 지나가지 못합니다. 모르는 칸(-1)은 `allow_unknown: true`라 NavFn이 지나가므로 막힘이 아닙니다. 격자 밖의 점은 막힘으로 봅니다. 끝 점은 NavFn이 goal 칸의 비용을 보지 않고 거기서부터 퍼지므로 보지 않습니다. 격자를 아직 받지 못했거나, 받은 지 `costmap_timeout_s`(3.0 s, 벽시계)가 지났거나, 격자 원점이 돌아가 있으면 판정하지 않고 새 경로를 씁니다. 3.0 s는 Gazebo가 느릴 때(실시간 배율 0.75) 1 Hz 격자가 벽시계로 1.3 s쯤마다 오는 것을 감안한 값입니다.
- 막히지 않았고 새 경로가 `max(shorter_m, shorter_ratio × 남은 길이)`(0.30 m, 0.2)만큼 확실히 짧지 않으면 남은 길을 돌려줍니다. 이때 기억하는 goal은 그 길을 만든 goal 그대로입니다(goal이 5 cm 안에서 조금씩 움직여도 오차가 쌓이지 않게).
- 남은 길을 돌려줄 때는 경로와 모든 자세의 시각을 지금으로 새로 찍습니다. Jazzy의 `controller_server`는 도착 판정에서 경로 끝 자세를 그 경로의 시각 TF로 옮기므로, 묵은 시각을 그대로 두면 목적지 바로 앞에서 도착으로 인정받지 못했습니다.
- 로봇이 남은 길에서 `off_route_m`(0.30 m)보다 멀리 벗어나 있으면 새 경로를 씁니다.
- planner가 실패하거나 처리 중 예외가 나면 지켜 둔 경로를 지웁니다.
- 길을 지킨 때와 바꾼 때만 INFO 로그를 한 줄 남깁니다(`kept the route: ...`, `changed the route: ...`). 막혀서 바꾼 때는 막힌 첫 점과 그 칸의 값을 적습니다(예: `the kept route is blocked at (1.12, 0.31) cost 99`).

## 파라미터

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `keep` | `true` | `false`면 늘 planner의 결과를 그대로 돌려줍니다. launch는 이 값을 주지 않습니다(끄려면 `plan_keep:=false`로 노드와 길 유지 트리를 함께 뺍니다) |
| `same_route_m` | `0.20` | 같은 길로 보는 거리 |
| `off_route_m` | `0.30` | 로봇이 길에서 벗어났다고 보는 거리 |
| `shorter_m`, `shorter_ratio` | `0.30`, `0.2` | 새 길이 이보다 더 짧아야 바꿉니다(둘 중 큰 쪽) |
| `same_goal_m` | `0.05` | goal이 같다고 보는 거리 |
| `blocked_cost` | `99` | 전역 비용 지도 값이 이 이상인 칸을 막힌 칸으로 봅니다(99 = INSCRIBED, 100 = LETHAL) |
| `costmap_timeout_s` | `3.0` | 받은 지 이보다 오래된 전역 비용 지도로는 판정하지 않습니다(벽시계, 로그 이유 `the global costmap is older than 3 s`) |

돌려준 경로는 토픽 `plan_kept`(`nav_msgs/Path`)로도 냅니다. planner가 내는 `plan`과 달리 로봇이 실제로 따라가는 길이라, RViz에서 보거나 경로 뒤집힘을 셀 때 씁니다.

기본으로 켜져 있습니다(11-26 실물 두 대 판에서 켠 채로 문제없이 돈 뒤 기본값을 바꿈). `pode_nav_config`의 `nav2.launch.py`는 `plan_keep`(기본 true)이 켜져 있으면 로봇 namespace 안에 이 노드를 띄우고 행동 트리를 `_keep` 갈래(`server_name="compute_path_keep"`)로 바꿉니다. 끄려면 `plan_keep:=false`, 실물 묶음은 `PLAN_KEEP=0 start.sh ...`입니다(`real_pode.launch.py`의 `plan_keep`, 기본 true). `deploy.sh`가 이 패키지도 복사합니다. 실물 Pi4에서 켠 채로 주행할 때 Nav2 컨테이너 109~113 %, pode_fsm 55~57 %였습니다(11-26, 끈 판과의 비교는 없음). Gazebo 세 대(`fleet_gz_sim` `gz.launch.py`)는 DDS 참여자 수를 늘리지 않으려고 `sim_stand_in` 프로세스 안에 로봇마다 하나씩 둡니다(`make_fleet`).

## 한계

- 남은 길은 지켜 둔 경로에서 로봇에 가장 가까운 꼭짓점부터 자릅니다. 선분에 투영하지 않으므로 꼭짓점이 듬성한 경로에서는 로봇 뒤의 구간이 조금 남거나 앞의 구간이 조금 빠질 수 있고, 가까이 되돌아오는 경로에서는 다른 쪽 구간을 고를 수 있습니다(NavFn 경로는 칸 크기만큼 촘촘합니다).
- 막힘은 경로 점이 놓인 칸만 보고, 점 사이 선분이 지나는 칸은 보지 않습니다(NavFn 경로는 칸 크기만큼 촘촘합니다).
- 같은 요청을 동시에 둘까지 받습니다. 세 번째는 거절합니다. 동시에 처리 중인 두 요청이 끝나는 순서에 따라 먼저 온 요청의 경로가 나중에 기억될 수 있습니다(행동 트리는 한 번에 하나만 보냅니다).
