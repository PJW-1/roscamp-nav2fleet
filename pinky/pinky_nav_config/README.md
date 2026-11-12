# pinky_nav_config

핑키 한 대의 Nav2 설정(파라미터, BT, launch, RViz). 핑키 Pi에서 namespace `/{pinky}`로 돈다. 지킬 약속은 Drive `핑키_FSM_Nav2_구현_안내_2026-11-07.md`와 `핑키_제어파트_전달사항_2026-11-06`(R4, R12~R17)이다.

| 파일 | 내용 |
|---|---|
| `params/nav2_params.yaml` | 제조사 `pinky_navigation` 설정(pinky_pro 014a09f) 위에 전달사항의 nav2_overrides 값을 덮었다. 바꾼 줄마다 이유가 있다 |
| `bt/navigate_to_pose_w_replanning_and_recovery_goal.xml` | `navigate_to_pose`용 BT(MoveTo mode `nav_goal`, 계약 0.11-draft 제안). Jazzy 기본 BT를 옮기고 controller를 `FollowPathGoal`로 바꿨다. 1 Hz 재계획. 복구는 지도 지우기와 Wait뿐이다: Spin과 BackUp은 뺐다(추가 지시서 3의 7-4, 기준(사용자)). 물러나기는 `pinky_fsm`의 로봇단 교착 처리만 한다 |
| `params/keepout.yaml` | keepout 덧붙임(제안). `keepout_mask`가 있을 때만 launch가 덧붙인다: global costmap의 keepout 필터와 두 서버 설정 |
| `params/avoidance.yaml` | 회피 담당의 설정(추가 지시서 3의 7-1). 키는 `follow_path_goal`(`FollowPathGoal` 컨트롤러 안의 값)과 `vision_obstacles`(비전 관측원의 값) 둘뿐이다. 다른 키가 있으면 launch가 뜨지 않고 시험이 실패한다. 나머지 설정은 사용자의 것이고 `nav2_params.yaml`에 있다 |
| `launch/pinky_nav_params.py` | 두 파일을 모으는 순수 함수(`with_avoidance`, `foreign_keys`). ROS 없이 돈다 |
| `launch/nav2.launch.py` | 지도 서버 둘, AMCL, controller, planner, behavior, bt_navigator, lifecycle manager 둘. `keepout_mask`가 있으면 keepout 마스크 서버, 필터 정보 서버, 그 lifecycle manager를 더한다 |
| `rviz/pinky_nav.rviz` | 지도, 로봇, 라이다, costmap, AMCL 입자, 따라가는 경로. `<robot_namespace>`는 띄울 때 바꾼다 |

## 구성

- 모든 노드가 `/{namespace}` 안에서 돈다. TF는 `/{namespace}/tf`, `/{namespace}/tf_static`이고 프레임 이름(`map`, `odom`, `base_footprint`, `rplidar_link`)에는 접두사가 없다(R17).
- controller_server(`follow_path`, RPP)가 `cmd_vel`로 바로 낸다. controller는 넷이다: `FollowPath`, 잠금 경계 경로용 `FollowPathThrough`, 물러나기 경로용 `FollowPathReverse`(제안, 팀 합의 전. 0.05 m/s 후진, 제자리 회전 없음), Nav2 goal용 `FollowPathGoal`(제안. 지금은 `FollowPath`와 같은 RPP, 충돌 검사 켬. 팀원이 MPPI나 DWB로 바꿀 수 있고 이름만 고정이다). 넷 다 늘 뜬다. velocity_smoother와 collision_monitor는 없다. `pinky_fsm`이 mode `follow_path`에서는 `follow_path`를 바로 부르고, `nav_goal`에서는 `navigate_to_pose`를 부른다.
- 고정 선(추가지시서3 2-1): `FollowPath`, `FollowPathThrough`, `FollowPathReverse`는 lookahead를 0.10 m로 고정하고(속도에 따라 늘리지 않음) `rotate_to_heading_min_angle`을 0.70 rad로 두었다. 핑키 FSM이 꺾이는 곳을 반지름 0.10 m 호로 바꿔 보내므로 호에서는 제자리 회전을 하지 않는다. `FollowPathGoal`은 바꾸지 않았다. local costmap 발자국은 몸 모양 다각형 14점(몸통과 바퀴, 폭 0.112 m, 회전 중심에서 가장 먼 점 약 0.083 m)이다. global costmap은 마당 NavFn 계획이 바뀌지 않게 옛 정사각형 그대로다(가정).
- planner는 NavFn이다. global costmap의 inflation만 0.25 m, 6.0으로 넓혔다(local은 0.15 m, 3.0 그대로). 0.38 m 통로에서도 가운데 칸이 INSCRIBED보다 낮아 길이 나오고 가운데가 가장 싸며, 넓은 마당에서는 경로가 벽과 진열대에서 약 0.25 m 떨어진다. Smac 2D로 바꿀 이유는 찾지 못했다(점 로봇 계획으로 충분하고, 몸은 INSCRIBED 반지름 0.09 m로 본다).
- behavior_server는 `backup`, `wait`를 띄운다(`spin`은 7-4에서 뺐다). `backup`은 `pinky_fsm`의 로봇단 교착 처리, 끈 때의 자가 복구, 물러나기가 쓴다.
- keepout(제안): `keepout_mask`를 주면 급행과 도크 구역을 global costmap에서 막는다. Nav2 goal 경로(NavFn)만 영향을 받고, FollowPath 셋은 local costmap만 쓰므로 그대로다. 필터는 inflation 뒤에 얹히므로 마스크는 몸 반폭만큼 넓혀 만든다. 마스크는 site 도구가 만든다.
- AMCL은 1 cm 지도(`map_fine`), costmap은 5 cm 지도(`map`)를 읽는다(R15).
- local costmap은 0.025 m이고 `always_send_full_costmap: true`다. `pinky_fsm`이 `local_costmap/costmap`으로 BLOCKED를 판단한다.
- 버전 2를 막지 않으려고 둔 것(안내 3절): planner_server(`compute_path_to_pose`, `is_path_valid`), 전역 costmap의 라이다 층, controller의 `speed_limit` 토픽.
- costmap 층은 Jazzy에서 상대 토픽을 costmap 노드 안으로 풀기 때문에, 파라미터 파일에 `<robot_namespace>/scan`처럼 쓰고 launch가 `/pinky1/scan`으로 바꾼다.

## 실행

```bash
ros2 launch pinky_nav_config nav2.launch.py namespace:=pinky1 \
    map:=<5cm 지도.yaml> map_fine:=<1cm 지도.yaml> \
    initial_x:=0.35 initial_y:=1.55 initial_yaw:=1.5708
```

`keepout_mask:=<마스크.yaml>`(map_server 형식, 막힌 칸이 keepout)을 주면 keepout을 켠다. 비면(기본) 노드와 파라미터가 keepout 전과 같다.

`vision_obstacles:=true`를 주면 비전 장애물 관측원을 더한다. 기본(false)은 노드와 파라미터가 전과 같다. 마스크는 `python3 tools/make_keepout.py --layout d2 --map <지도.yaml>`로 실제 지도에 맞춰 만든다.

`use_composition:=true`를 주면 노드 아홉(keepout이면 열둘)을 컨테이너(`component_container_isolated`) 한 프로세스에 올린다. 한 PC에 Gazebo 핑키 여럿을 띄우면 LOCALHOST discovery가 볼 수 있는 프로세스 수(약 32)를 넘기 때문이다. 이름, 파라미터, TF remap은 같고 기본값(false)은 노드마다 프로세스다.

실물 지도는 아직 없다(SLAM 뒤에 넣는다). Gazebo 지도와 한 번에 띄우는 방법은 `sim/fleet_gz_sim/README.md`.

## 시험

```bash
cd pinky/pinky_nav_config && python3 -m pytest -q test   # 설정 값 검사, ROS 필요 없음
```

`test/test_nav2_launch.py`(launch 구성 검사)는 ROS 2와 설치된 이 패키지가 있어야 돌고, 없으면 건너뛴다. `colcon test`로 돌린다.

`params/`의 시작값은 0.33 m 통로 Gazebo 연구에서 온 값이므로 실제 지도에서 다시 맞춘다.
