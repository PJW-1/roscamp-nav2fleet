"""실물 Pode 한 대를 /<namespace> 안에서 띄운다: 제조사 bringup, cmd_vel 감시, Nav2 + AMCL, pode_fsm.

하나라도 끝나면 전체를 내린다(bringup이 모터를 끈다).
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
             "peer_obstacles", "peer_robots", "log_level", "use_composition", "plan_keep")
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
        # costmap 지도(2.5 cm)
        DeclareLaunchArgument("map", default_value=str(MAPS / "arena_v7_a.yaml")),
        # 위치 추정은 도면 1 cm 지도(시뮬과 같다). 옛 SLAM 지도는 map_fine:=.../arena_real_slam.yaml
        DeclareLaunchArgument("map_fine", default_value=str(MAPS / "arena_v7_fine.yaml")),
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
        DeclareLaunchArgument("plan_keep", default_value="true",
                              description="true(기본)면 길 유지 노드(pode_plan_keep)와 _goal_keep 행동 트리를 쓴다"),
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
