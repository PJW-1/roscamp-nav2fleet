import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ARGS = {
    "namespace": ("pode1", "로봇 이름"),
    "layout": ("v7", "월드 배치"),
    "gui": ("false", "Gazebo 화면"),
    "x": ("5.0", "놓는 자리 x(경기장 밖 빈 바닥, 바닥은 원점 가운데 20 m)"),
    "y": ("5.0", "놓는 자리 y"),
    "yaw": ("0.0", "방향(rad)"),
}


def _start(context):
    from fleet_gz_sim import launch_parts
    from fleet_gz_sim.fleet_setup import arena_for, layout_of
    from fleet_gz_sim.isolation import require_isolated
    from rclpy import get_rmw_implementation_identifier

    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())

    def value(name):
        return LaunchConfiguration(name).perform(context)

    ns = value("namespace")
    arena = arena_for(layout_of(value("layout")), launch_parts.share("fleet_gz_sim"))
    # ---- 월드, 브리지, 모델만(Nav2·pode_fsm 없음) ----
    return [*launch_parts.world(value("gui").lower() in ("true", "1"), arena.world),
            launch_parts.bridge([ns]),
            *launch_parts.model(ns, float(value("x")), float(value("y")), float(value("yaw")))]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
