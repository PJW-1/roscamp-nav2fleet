"""실물 Pode 한 대. 모두 /<namespace> 안에서 돈다:
제조사 bringup(모터, 라이다, odom), cmd_vel 감시, Nav2 + AMCL(pode_nav_config), pode_fsm.

    source ~/nav2fleet_real/env.sh
    ros2 launch ~/nav2fleet_real/real_pode.launch.py namespace:=pode1 \
        initial_x:=0.12 initial_y:=0.10 initial_yaw:=1.5708

- bringup_robot.launch.xml에는 namespace 인자가 없다. PushRosNamespace 아래에서 /tf, /tf_static을
  namespace 안으로 돌려 띄운다(틀 이름에는 앞머리가 없다). pode_nav_config가 그렇게 읽는다.
- 제조사 bringup은 마지막 cmd_vel을 끝없이 지키므로 cmd_watchdog.py가 속도 0을 보낸다.
- 이 launch의 프로세스 하나라도 끝나면 전체를 내린다(그러면 bringup이 모터를 끈다):
  pode_fsm 없이 Nav2 goal로 계속 달리면 안 된다.
- 속도는 params/의 사본(make_params.py, start.sh가 만든다)이 정한다. pode_fsm의
  horizon_cap_mps는 비우면 그 사본의 FollowPath 속도와 같게 둔다.
- 도킹은 없다(pode_fsm dock()은 DOCK_UNAVAILABLE). 시험은 거기서 멈춘다.
  dock_aside:=true(start.sh는 DOCK_STUB=1)면 pode_fsm의 Dock 서버를 /<namespace>/dock_fsm으로
  비킨다. 그 자리(/<namespace>/dock)는 PC의 도킹 대역 노드가 맡는다.
"""
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from make_params import horizon_cap  # noqa: E402

MAPS = HERE / "maps"
PARAMS = HERE / "params"
# nav2.launch.py에 넘기는 인자(이 launch의 같은 이름 인자, 아래 FIXED는 고정값)
NAV2_ARGS = ("namespace", "use_sim_time", "map", "map_fine", "keepout_mask", "params_file",
             "avoidance_params_file", "initial_x", "initial_y", "initial_yaw",
             "peer_obstacles", "peer_robots", "log_level", "use_composition")
# pode_fsm.launch.py에 넘기는 인자
FSM_ARGS = ("namespace", "use_sim_time", "heartbeat_timeout_s", "through_controller_id",
            "horizon_cap_mps", "battery_topic")
FIXED = {"use_sim_time": "false", "use_composition": "true",
         "through_controller_id": "FollowPathThrough"}


def _fsm(context):
    # ---- horizon_cap_mps: 비면 params_file의 FollowPath 속도 ----
    def value(name):
        return LaunchConfiguration(name).perform(context)

    cap = horizon_cap(value("horizon_cap_mps"), value("params_file"))
    given = {name: FIXED.get(name) or LaunchConfiguration(name) for name in FSM_ARGS}
    given["horizon_cap_mps"] = cap
    fsm = Path(get_package_share_directory("pode_fsm"))
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(fsm / "launch" / "pode_fsm.launch.py")),
        launch_arguments=given.items())
    out = [LogInfo(msg=f"real_pode: pode_fsm horizon_cap_mps {cap}")]
    if value("dock_aside") == "true":
        # 서버 이름은 계약의 절대 이름(/<ns>/dock)이라 절대 이름으로 비킨다(시뮬과 같다)
        ns = value("namespace")
        out += [LogInfo(msg=f"real_pode: dock_aside on, pode_fsm Dock -> /{ns}/dock_fsm"
                            f" (PC dock stub must serve /{ns}/dock)"),
                GroupAction([SetRemap(f"/{ns}/dock", f"/{ns}/dock_fsm"), include])]
    else:
        out.append(include)
    return out


def generate_launch_description():
    arg = LaunchConfiguration
    ns = arg("namespace")
    nav = Path(get_package_share_directory("pode_nav_config"))
    bringup = Path(get_package_share_directory("pinky_bringup"))
    nav2_given = {name: FIXED.get(name) or arg(name) for name in NAV2_ARGS}

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pode1"),
        DeclareLaunchArgument("initial_x", default_value="0.12"),
        DeclareLaunchArgument("initial_y", default_value="0.10"),
        DeclareLaunchArgument("initial_yaw", default_value="1.5708"),
        # 2.5 cm 지도(11-17 결정)
        DeclareLaunchArgument("map", default_value=str(MAPS / "arena_v7_a.yaml")),
        # 위치 추정은 실물 SLAM 지도(2026-11-17 확정)
        DeclareLaunchArgument("map_fine", default_value=str(MAPS / "arena_real_slam.yaml")),
        DeclareLaunchArgument("keepout_mask", default_value=str(MAPS / "arena_v7_keepout.yaml"),
                              description="비우면 keepout을 끈다"),
        DeclareLaunchArgument("params_file", default_value=str(PARAMS / "nav2_params.yaml"),
                              description="make_params.py가 만든 사본"),
        DeclareLaunchArgument("avoidance_params_file",
                              default_value=str(PARAMS / "avoidance.yaml"),
                              description="make_params.py가 만든 사본(FollowPathGoal 속도)"),
        DeclareLaunchArgument("peer_obstacles", default_value="false",
                              description="두 대 이상이면 true와 peer_robots"),
        DeclareLaunchArgument("peer_robots", default_value="",
                              description="예: pode1,pode2"),
        DeclareLaunchArgument("horizon_cap_mps", default_value="",
                              description="비면 params_file의 FollowPath 속도"),
        DeclareLaunchArgument("heartbeat_timeout_s", default_value="1.5",
                              description="0이면 관제 heartbeat가 끊겨도 서지 않는다"),
        DeclareLaunchArgument("battery_topic", default_value="",
                              description="BatteryState 토픽. 비면 배터리를 모름으로 알린다"),
        DeclareLaunchArgument("dock_aside", default_value="false",
                              description="true면 pode_fsm Dock 서버를 /<ns>/dock_fsm으로 비킨다"),
        DeclareLaunchArgument("log_level", default_value="warn"),
        # ---- 제조사 bringup ----
        GroupAction([
            PushRosNamespace(ns),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            # bringup 안의 upload_robot.launch.py에도 namespace 인자가 있다(틀 앞머리와 두 번째
            # namespace를 붙인다). 우리 값이 그리로 새면 안 된다
            SetLaunchConfiguration("namespace", ""),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(
                str(bringup / "launch" / "bringup_robot.launch.xml"))),
        ]),
        # ---- cmd_vel 감시 ----
        ExecuteProcess(cmd=["python3", str(HERE / "cmd_watchdog.py"), ["/", ns, "/cmd_vel"]],
                       name="cmd_watchdog", output="screen"),
        # ---- Nav2 + AMCL ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / "launch" / "nav2.launch.py")),
            launch_arguments=nav2_given.items()),
        # ---- pode_fsm ----
        OpaqueFunction(function=_fsm),
        # ---- 하나라도 끝나면 모두 내린다 ----
        RegisterEventHandler(OnProcessExit(on_exit=[
            LogInfo(msg="real_pode: a process ended, shutting everything down"),
            Shutdown(reason="a process of the robot launch ended")])),
    ])
