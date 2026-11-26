"""Gazebo에서 Pode 한 대를 Nav2, pode_fsm과 함께 돌린다. 관제(fleet_manager)는 띄우지 않는다.
  ros2 launch fleet_gz_sim one_pode.launch.py layout:=c start:=P-2 gui:=false rviz:=false
주행은 drive_check나 RViz 2D Goal Pose로 시킨다. 격리 검사(isolation.py)를 통과해야 뜬다. still_pode:="x,y,yaw"는 서 있기만 하는 Pode를 놓는다.
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
    "namespace": ("pode1", "로봇 이름이자 namespace"),
    "layout": ("v7", "배치: v7(기본, 실제 경기장), d2, d, c, 시험용 d2_shelf"),
    "start": ("", "시작 지점(구역 지도의 지점 이름). 방향은 그 지점의 yaw. 비면 인자 없는"
                  " drive_check가 그 배치에서 고르는 계획의 시작(v7은 O_IN, 그 밖은 P-1)"),
    "gui": ("true", "Gazebo 화면"),
    "rviz": ("true", "RViz"),
    "fsm": ("true", "pode_fsm(move_to, retreat, dock, state)"),
    "heartbeat_timeout_s": ("1.5", "pode_fsm heartbeat 끊김 판정. 0이면 서지 않는다"),
    "nav_log_level": ("warn", "Nav2 노드 로그 수준"),
    "nav_max_speed": ("", "controller들의 최고 속도(m/s), 0.05~단일 출처."
                          " 비면 pode_nav_config/params/nav_speed.yaml(0.12)"),
    "keepout": ("true", "false면 global costmap keepout 필터를 띄우지 않는다(비교용)"),
    "still_pode": ("", "x,y,yaw: 서 있기만 하는 둘째 Pode(pode_still)를 놓는다. 비면 없음"),
    "nav_overrides": ("", "Nav2 설정 위에 덮을 yaml 파일(시뮬 전용, 비교용). 비면 없음"),
    "obstacles": ("", "상자를 놓았다 치우는 일정(fleet_gz_sim/obstacles의 이름이나 파일)"),
    "lidar_noise": ("", "라이다 거리 잡음 표준편차(m, 비교용). 비면 모델 기본값 0.02"),
    "scan_filter": ("true", "좁은 띠 안에서 벽·진열대에 붙은 라이다 점을 costmap에서 지우는 거르개"
                            "(pode_scan_filter). false면 띄우지 않는다(비교용)"),
    "wall_band_m": ("0.05", "스캔 거르개: 벽·진열대에서 이만큼 안의 점을 지운다(m)."
                            " 0.05는 D2 측정으로 고른 값(fixed_line_results.md)"),
    "plan_keep": ("true", "true(기본)면 Nav2 goal을 다시 계획할 때 길이가 비슷한 다른 길로"
                           " 바꾸지 않고 지금 길을 지킨다(pode_plan_keep). false면 main과 같다"),
    "fixed_line_collision_check": ("false", "시뮬 전용. true면 고정 경로 controller 셋도 충돌"
                                           " 검사를 켠다. 마당 goal(FollowPathGoal)은 늘 켬"),
    "nav_map_variant": ("", "global costmap 지도 갈래(v7만): 비면 배치 기본(v7은 a, 2.5 cm),"
                            " base는 5 cm 지도, b는 칸 중심 규칙, ab는 둘 다, a1은 a 규칙의 1 cm."
                            " 1 cm 지도와 keepout 마스크는 그대로"),
}
STILL = "pode_still"


def still_pose(text: str) -> tuple[float, float, float] | None:
    if not text.strip():
        return None
    try:
        x, y, yaw = (float(v) for v in text.split(","))
    except ValueError:
        raise ValueError(f"still_pode must be x,y,yaw, got {text!r}") from None
    return x, y, yaw


def _start(context):
    # 노드는 launch의 작업 폴더를 물려받는다(gz.launch.py와 같은 검사)
    require_isolated(os.environ, rmw=get_rmw_implementation_identifier(), workdir=Path.cwd())
    import yaml
    from fleet_core.fleet_map import load_fleet_map
    from fleet_gz_sim.fleet_setup import (
        arena_for,
        layout_of,
        narrow_strips,
        obstacles_path,
        start_poses,
    )
    from fleet_gz_sim.obstacles import load_schedule
    from fleet_gz_sim.stage2_plan import pick_layout_plan, start_node
    from launch_ros.actions import Node

    def value(name):
        return LaunchConfiguration(name).perform(context)

    on = {name: value(name).lower() in ("true", "1")
          for name in ("gui", "rviz", "fsm", "keepout", "scan_filter")}
    fixed_check = value("fixed_line_collision_check").strip().lower()
    if fixed_check not in ("true", "false", "1", "0"):  # 오타가 '끔'으로 바뀌지 않게
        raise ValueError(f"fixed_line_collision_check must be true or false, got {fixed_check!r}")
    keep = value("plan_keep").strip().lower()
    if keep not in ("true", "false", "1", "0"):
        raise ValueError(f"plan_keep must be true or false, got {keep!r}")
    layout = layout_of(value("layout"))
    site = load_fleet_map(layout)
    arena = arena_for(layout, launch_parts.share("fleet_gz_sim"),
                      value("nav_map_variant").strip())
    ns = value("namespace")
    # 비면 drive_check 계획의 시작 지점
    start = value("start").strip() or start_node(pick_layout_plan(layout, None)[1])
    if start not in site.nodes:
        raise ValueError(f"start {start!r} is not a node of {site.name}")
    x, y, yaw = start_poses(site, {ns: start})[ns]
    overrides = None
    if value("nav_overrides"):
        overrides = yaml.safe_load(Path(value("nav_overrides")).read_text(encoding="utf-8"))
    obstacles_file = obstacles_path(value("obstacles"), launch_parts.share("fleet_gz_sim"))
    if obstacles_file:
        load_schedule(obstacles_file, site)  # 틀린 일정이면 Gazebo를 띄우기 전에 멈춘다
    actions = [*launch_parts.world(on["gui"], arena.world),
               launch_parts.bridge([ns]),
               *launch_parts.pode(ns, x, y, yaw, nav_map=arena.nav_map,
                                   nav_map_fine=arena.nav_map_fine,
                                   keepout_mask=arena.keepout_mask if on["keepout"] else "",
                                   heartbeat_timeout_s=value("heartbeat_timeout_s"),
                                   nav_log_level=value("nav_log_level"),
                                   nav_max_speed=launch_parts.speed_of(value("nav_max_speed")),
                                   fsm=on["fsm"], rviz=on["rviz"], nav_overrides=overrides,
                                   fixed_line_check=fixed_check in ("true", "1"),
                                   narrow_strips=narrow_strips(site, float(value("wall_band_m")))
                                   if on["scan_filter"] else "",
                                   wall_band_m=float(value("wall_band_m")),
                                   plan_keep=keep in ("true", "1"),
                                   lidar_noise=(float(value("lidar_noise"))
                                                if value("lidar_noise") else None))]
    if obstacles_file:
        actions.append(Node(package="fleet_gz_sim", executable="sim_obstacles",
                            name="sim_obstacles", output="screen",
                            parameters=[{"use_sim_time": True, "robots": [ns],
                                         "obstacles_file": obstacles_file, "map_file": layout}]))
    still = still_pose(value("still_pode"))
    if still is not None:
        actions += launch_parts.model(STILL, *still)
    return actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGS.items()),
        OpaqueFunction(function=_start),
    ])
