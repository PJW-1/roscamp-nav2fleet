from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap

HERE = Path(__file__).resolve().parent


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    bringup = Path(get_package_share_directory("pinky_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pode1"),
        # ---- 제조사 bringup(real_pode.launch.py와 같은 묶음, Nav2·pode_fsm 없음) ----
        GroupAction([
            PushRosNamespace(ns),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetLaunchConfiguration("namespace", ""),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(
                str(bringup / "launch" / "bringup_robot.launch.xml"))),
        ]),
        # ---- cmd_vel 감시 ----
        ExecuteProcess(cmd=["python3", str(HERE / "cmd_watchdog.py"), ["/", ns, "/cmd_vel"]],
                       name="cmd_watchdog", output="screen"),
        RegisterEventHandler(OnProcessExit(on_exit=[
            LogInfo(msg="bringup_only: a process ended, shutting everything down"),
            Shutdown(reason="a process of the bringup ended")])),
    ])
