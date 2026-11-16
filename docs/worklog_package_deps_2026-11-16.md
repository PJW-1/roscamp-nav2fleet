# package.xml 의존 선언 보충 (2026-11-16)

## 결론
쓰는데 선언이 없던 의존을 8개 package.xml에 보충했다. 한 가지는 순환이라 빼야 했다: fleet_manager의 `test_depend iponic_bringup`을 지웠다(아래 "순환"). 코드 변경은 없다.

## 바꾼 파일과 추가한 의존
| 파일 | 변경 | 근거 |
|---|---|---|
| bringup/iponic_bringup | exec_depend: launch, launch_ros, ament_index_python, fake_robots, fleet_manager | launch/pc.launch.py:13-18(launch, launch_ros import), :25(ament_index_python), :26-27(fake_robots import와 get_package_share_directory), :54(`package="fleet_manager"`), :71(`FindPackageShare("fake_robots")`). 다른 launch 파일 없음 |
| pode/pode_fsm | exec_depend: builtin_interfaces | pode_fsm/backup_ros.py:9 |
| pode/pode_peer_obstacles | exec_depend: std_msgs | pode_peer_obstacles/node.py:36 |
| pc/fleet_manager | test_depend: launch. test_depend iponic_bringup 삭제 | test/test_launch_args.py:13-15가 launch만 씀(src는 안 씀) |
| pc/fleet_manager (후속) | test_depend: launch_ros | test/test_launch_args.py:7이 launch_ros를 직접 import (Codex 지적) |
| pode/pode_avoidance | test_depend: builtin_interfaces | test/test_vision_stand_in_node.py:10 (src는 안 씀) |
| pode/pode_scan_filter | test_depend: geometry_msgs | test/test_scan_filter_node.py:15 (src는 안 씀) |
| sim/fleet_gz_sim | exec_depend: rosgraph_msgs (test_depend가 아님). tf2_ros를 tf2_ros_py로 교체 | rosgraph_msgs: fleet_gz_sim/launch_parts.py:86이 브리지 인자 `/clock@rosgraph_msgs/msg/Clock[...`를 만들어 실행 때 필요. test_stand_in_battery.py:50도 씀. tf2_ros: fleet_view_node.py:21이 파이썬 모듈을 import하므로 jazzy rosdep 키는 tf2_ros_py(`rosdep resolve`: tf2_ros_py는 ros-jazzy-tf2-ros-py) |
| pc/fleet_core | exec_depend: rclpy | fleet_core/common/contract.py:122(to_ros_qos 안의 지연 import). 시험(test_*)도 rclpy 사용 |

## 순환
iponic_bringup이 fleet_manager를 exec_depend하면, fleet_manager의 `test_depend iponic_bringup`과 겹쳐 colcon이 "Unable to order packages topologically"로 실패했다(colcon은 test_depend도 순서에 넣는다). 실행 방향(bringup -> fleet_manager)이 실제 의존이라 fleet_manager 쪽 test_depend를 지웠다. 영향: test/test_scenario_integration.py:59는 iponic_bringup이 없으면 skip하므로 시험은 안 깨지고, 그 시험을 돌리려면 iponic_bringup도 같이 빌드해 두어야 한다.

남은 문제(사용자 결정, 고치지 않음): fleet_manager의 test_depend iponic_bringup을 빼서, iponic_bringup을 같이 빌드하지 않으면 test_scenario_integration.py가 건너뛰고, test_launch_args.py:20은 그 패키지의 소스 경로를 직접 읽는다.

## 확인
- `xmllint --noout` 8개 파일 모두 통과.
- `colcon graph --packages-up-to iponic_bringup`: 순환 없음, fake_robots, fleet_manager가 앞에 잡힘. `colcon graph` 전체에도 오류 없음.
- `colcon build --packages-up-to iponic_bringup pode_fsm pode_peer_obstacles pode_avoidance pode_scan_filter fleet_core`(산출물은 scratchpad/deps): 9개 패키지 성공.
- `rosdep check --from-paths bringup pode pc sim --ignore-src`: 바꾼 키에 대한 오류 없음. 남은 오류는 `fleet_interfaces`(interfaces 경로를 안 줘서 로컬 패키지로 못 찾음)뿐.

## 돌리지 않은 것
- fleet_gz_sim 빌드(pinky_description 등 외부 패키지 필요)와 pytest. 
- Gazebo 실행, 시험 실행.

## 보고만 (건드리지 않음)
- arm_control, arm_vision, central, pode_dock: 소스가 빈 껍데기(setuptools만 import)라 `rclpy`, `fleet_interfaces` 선언이 쓰이지 않음.
- pode_avoidance: launch, launch_ros는 launch/avoidance.launch.py에서 launch만 확인(launch_ros 사용 여부는 따로 확인 안 함).
