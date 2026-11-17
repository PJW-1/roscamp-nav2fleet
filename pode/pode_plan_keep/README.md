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
- 다른 길이면 planner의 `is_path_valid`로 남은 길이 막혔는지 묻습니다. 막혔거나 `valid_timeout_s`(0.2 s) 안에 답이 없으면 새 경로를 씁니다. 막히지 않았고 새 경로가 `max(shorter_m, shorter_ratio × 남은 길이)`(0.30 m, 0.2)만큼 확실히 짧지 않으면 남은 길을 돌려줍니다.
- 남은 길을 돌려줄 때는 경로와 모든 자세의 시각을 지금으로 새로 찍습니다. Jazzy의 `controller_server`는 도착 판정에서 경로 끝 자세를 그 경로의 시각 TF로 옮기므로, 묵은 시각을 그대로 두면 목적지 바로 앞에서 도착으로 인정받지 못했습니다.
- 로봇이 남은 길에서 `off_route_m`(0.30 m)보다 멀리 벗어나 있으면 새 경로를 씁니다.
- 길을 지킨 때와 바꾼 때만 INFO 로그를 한 줄 남깁니다(`kept the route: ...`, `changed the route: ...`).

## 파라미터

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `keep` | `true` | `false`면 늘 planner의 결과를 그대로 돌려줍니다(비교용). 행동 트리가 이 노드를 부르므로 노드는 늘 떠 있어야 합니다 |
| `same_route_m` | `0.20` | 같은 길로 보는 거리 |
| `off_route_m` | `0.30` | 로봇이 길에서 벗어났다고 보는 거리 |
| `shorter_m`, `shorter_ratio` | `0.30`, `0.2` | 새 길이 이보다 더 짧아야 바꿉니다(둘 중 큰 쪽) |
| `same_goal_m` | `0.05` | goal이 같다고 보는 거리 |
| `valid_timeout_s` | `0.2` | `is_path_valid` 답을 기다리는 시간 |

`pode_nav_config`의 `nav2.launch.py`가 로봇 namespace 안에 띄웁니다(`plan_keep:=false`면 `keep`이 거짓). Gazebo 세 대(`fleet_gz_sim` `gz.launch.py`)는 DDS 참여자 수를 늘리지 않으려고 `sim_stand_in` 프로세스 안에 로봇마다 하나씩 둡니다(`make_fleet`).
