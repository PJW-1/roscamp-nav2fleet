# pinky_avoidance

넓은 구역(HALL)의 회피를 맡는 임시 패키지입니다. 회피 담당의 패키지가 오기 전까지 지금의 회피 쪽 설정과 노드를 한곳에 모아 둡니다. 회피 담당의 패키지로 바꿀 때는 launch 인자 `avoidance_package:=<패키지>` 하나만 바꿉니다(`docs/회피_약속.md` 6.1절).

| 파일 | 내용 |
|---|---|
| `params/avoidance.yaml` | 회피 담당의 설정입니다. 키는 `follow_path_goal`(`FollowPathGoal` 안의 값), `vision_obstacles`(비전 관측원의 값), `peer_layer`(다른 로봇의 몸을 찍는 층) 셋입니다. 값은 `pinky_nav_config`에 있던 그대로입니다 |
| `launch/avoidance.launch.py` | 로봇 한 대의 회피 쪽 노드를 띄웁니다. `pinky_nav_config`의 `nav2.launch.py`가 로봇 namespace 안에서 부릅니다. `peer_obstacles:=true`이고 `peer_node:=true`면 `pinky_peer_obstacles` 노드를, `vision_stand_in:=true`면 비전 대역 노드를 띄웁니다 |
| `pinky_avoidance/vision_stand_in.py` | 시뮬레이션 전용 비전 대역 노드입니다(기본 꺼짐) |
| `pinky_avoidance/vision_cone.py` | 비전 대역의 기하 계산입니다. ROS 없이 돕니다 |

## 비전 대역

- 받는 것: `/sim/obstacle_markers`(`MarkerArray`, map 틀, `CUBE` 표시만 씁니다)와 `/<로봇>/sim/truth`(`Odometry`, Gazebo의 실제 자세, map 틀).
- 내는 것: `vision_obstacles`(로봇 namespace 안, `PointCloud2`, 필드 `x`, `y`, `z` float32). 시각은 그 점을 계산한 자세의 시각입니다. 새 자세가 올 때만 냅니다.
- 점: 상자의 옆면 가운데 로봇 쪽을 보는 면 위의 점(간격 `step_m`)을 내고, 그 가운데 앞쪽 카메라 원뿔 안의 것만 남깁니다. 다른 상자에 가려지는지는 보지 않습니다. 점구름이 비지 않게 먼 점 하나(1000, 1000)를 늘 함께 냅니다.
- 파라미터: `fov_deg`(70), `range_m`(1.0), `step_m`(0.02), `rate_hz`(10, 5 이상), `frame_id`(`map` 또는 `base_footprint`, 기본 `base_footprint`), `markers_topic`, `pose_topic`(비면 `/<로봇>/sim/truth`).
- 켜는 법: `gz.launch.py vision_stand_in:=true`(로봇마다 프로세스 하나가 늘어납니다). 켜면 비전 관측원도 켜집니다.
- 틀: 기본은 `base_footprint`입니다. costmap은 점까지의 거리를 점구름 틀의 원점에서 재므로, `map` 틀로 내면 `obstacle_max_range`(1.5 m)를 지도 원점에서 재서 경기장 대부분의 점이 버려집니다(2026-11-14에 기본을 바꿈).

## 시험

```bash
python3 -m pytest test -q   # test_vision_cone.py는 ROS 없이, 노드 시험은 rclpy가 있을 때만
```
