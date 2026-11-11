"""Gazebo에서 핑키 한 대를 Nav2, pinky_fsm과 함께 돌린다. 관제(fleet_manager)는 띄우지 않는다.

    ros2 launch fleet_gz_sim one_pinky.launch.py                          # D2, P-1, 화면 + RViz
    ros2 launch fleet_gz_sim one_pinky.launch.py layout:=c start:=P-2 gui:=false rviz:=false

주행은 따로 `ros2 run fleet_gz_sim drive_check --layout d2 --plan stage2`(관제 대신 이동을 바로
보냄)로 시키거나 RViz의 2D Goal Pose(bt_navigator)로 시킨다. 격리 검사(isolation.py)를 통과해야
무엇이든 띄운다. 월드와 핑키는 gz.launch.py와 같은 조각(fleet_gz_sim.launch_parts)으로 만든다.
still_pinky:="x,y,yaw"를 주면 Nav2 없이 서 있기만 하는 핑키(pinky_still)를 그 자리에 놓는다
(다른 핑키가 라이다에 보이는지 재기).
"""
import os
from pathlib import Path

from fleet_gz_sim import launch_parts
from fleet_gz_sim.isolation import require_isolated
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rclpy import get_rmw_implementation_identifier

ARGS = {
    "namespace": ("pinky1", "로봇 이름이자 namespace"),
    "layout": ("d2", "배치: d2(기본), d, c"),
    "start": ("P-1", "시작 지점(구역 지도의 지점 이름). 방향은 그 지점의 yaw"),
    "gui": ("true", "Gazebo 화면"),
    "rviz": ("true", "RViz"),
    "fsm": ("true", "pinky_fsm(move_to, retreat, dock, state)"),
    "heartbeat_timeout_s": ("1.5", "pinky_fsm heartbeat 끊김 판정. 0이면 서지 않는다"),
    "nav_log_level": ("warn", "Nav2 노드 로그 수준"),
    "nav_max_speed": ("0.12", "controller들의 최고 속도(m/s), 0.12~0.15"),
    "keepout": ("true", "false면 global costmap keepout 필터를 띄우지 않는다(비교용)"),
    "still_pinky": ("", "x,y,yaw: 서 있기만 하는 둘째 핑키(pinky_still)를 놓는다. 비면 없음"),
    "nav_overrides": ("", "Nav2 설정 위에 덮을 yaml 파일(시뮬 전용, 비교용). 비면 없음"),
    "obstacles": ("", "상자를 놓았다 치우는 일정(fleet_gz_sim/obstacles의 이름이나 파일)"),
    "lidar_noise": ("", "라이다 거리 잡음 표준편차(m, 비교용). 비면 모델 기본값 0.02"),
}
STILL = "pinky_still"


def still_pose(text: str) -> tuple[float, float, float] | None:
    if not text.strip():
        return None
    try:
        x, y, yaw = (float(v) for v in text.split(","))
    except ValueError:
        raise ValueError(f"still_pinky must be x,y,yaw, got {text!r}") from None
    return x, y, yaw


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다(gz.launch.py와 같은 검사)
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    import yaml
    from fleet_core.fleet_map import load_fleet_map
    from fleet_gz_sim.fleet_setup import arena_for, layout_of, obstacles_path, start_poses
    from fleet_gz_sim.obstacles import load_schedule
    from launch_ros.actions import Node

    def value(name):
        return LaunchConfiguration(name).perform(context)

    on = {name: value(name).lower() in ("true", "1") for name in ("gui", "rviz", "fsm", "keepout")}
    layout = layout_of(value("layout"))
    site = load_fleet_map(layout)
    arena = arena_for(layout, launch_parts.share("fleet_gz_sim"))
    ns = value("namespace")
    if value("start") not in site.nodes:
        raise ValueError(f"start {value('start')!r} is not a node of {site.name}")
    x, y, yaw = start_poses(site, {ns: value("start")})[ns]
    overrides = None
    if value("nav_overrides"):
        overrides = yaml.safe_load(Path(value("nav_overrides")).read_text(encoding="utf-8"))
    obstacles_file = obstacles_path(value("obstacles"), launch_parts.share("fleet_gz_sim"))
    if obstacles_file:
        load_schedule(obstacles_file, site)  # 틀린 일정이면 Gazebo를 띄우기 전에 멈춘다
    actions = [*launch_parts.world(on["gui"], arena.world),
               launch_parts.bridge([ns]),
               *launch_parts.pinky(ns, x, y, yaw, nav_map=arena.nav_map,
                                   nav_map_fine=arena.nav_map_fine,
                                   keepout_mask=arena.keepout_mask if on["keepout"] else "",
                                   heartbeat_timeout_s=value("heartbeat_timeout_s"),
                                   nav_log_level=value("nav_log_level"),
                                   nav_max_speed=launch_parts.speed_of(value("nav_max_speed")),
                                   fsm=on["fsm"], rviz=on["rviz"], nav_overrides=overrides,
                                   lidar_noise=(float(value("lidar_noise"))
                                                if value("lidar_noise") else None))]
    if obstacles_file:
        actions.append(Node(package="fleet_gz_sim", executable="sim_obstacles",
                            name="sim_obstacles", output="screen",
                            parameters=[{"use_sim_time": True, "robots": [ns],
                                         "obstacles_file": obstacles_file, "map_file": layout}]))
    still = still_pose(value("still_pinky"))
    if still is not None:
        actions += launch_parts.model(STILL, *still)
    return actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
