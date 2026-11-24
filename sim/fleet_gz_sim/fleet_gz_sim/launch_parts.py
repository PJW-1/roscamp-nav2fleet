"""one_pode.launch.py와 gz.launch.py가 같이 쓰는 launch 조각: Gazebo 월드, 브리지, Pode.
LOCALHOST 탐색에서 DDS 참여자는 32개 정도만 서로 보이므로 브리지는 하나, 3대일 때 Nav2는 로봇마다 프로세스 하나로 묶는다.
"""
from __future__ import annotations

import copy
import math
import os
import tempfile
from pathlib import Path

import yaml
from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from fleet_gz_sim.arena import WORLD

TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def share(package: str) -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def bridge_args(ns: str) -> tuple[list[str], list[tuple[str, str]]]:
    """gz 토픽 /model/<ns>/... ↔ ROS 토픽 /<ns>/... (/clock은 bridge()가 한 번만 넣는다)."""
    pairs = [
        ("scan", "scan", "sensor_msgs/msg/LaserScan", "[", "gz.msgs.LaserScan"),
        ("odom", "odom", "nav_msgs/msg/Odometry", "[", "gz.msgs.Odometry"),
        ("tf", "tf", "tf2_msgs/msg/TFMessage", "[", "gz.msgs.Pose_V"),
        ("joint_states", "joint_states", "sensor_msgs/msg/JointState", "[", "gz.msgs.Model"),
        ("cmd_vel", "cmd_vel", "geometry_msgs/msg/Twist", "]", "gz.msgs.Twist"),
        ("truth", "sim/truth", "nav_msgs/msg/Odometry", "[", "gz.msgs.Odometry"),
    ]
    args, remaps = [], []
    for gz_name, ros_name, ros_type, direction, gz_type in pairs:
        gz_topic = f"/model/{ns}/{gz_name}"
        args.append(f"{gz_topic}@{ros_type}{direction}{gz_type}")
        remaps.append((gz_topic, f"/{ns}/{ros_name}"))
    return args, remaps


def world(gui: bool, world_file: str) -> list:
    """Gazebo 서버(끝나면 launch도 끝)와 화면. 월드 이름은 WORLD다."""
    server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v", "2", world_file],
                            name="gz_server", output="screen")
    actions = [
        # gz-transport도 이 컴퓨터 안에서만 쓰고, 다른 시뮬과 섞이지 않게 칸막이를 나눈다
        SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
        SetEnvironmentVariable("GZ_PARTITION", os.environ.get(
            "GZ_PARTITION", f"fleet_gz_{os.environ['ROS_DOMAIN_ID']}")),
        # 제조사 메시(package://pinky_description/...)를 Gazebo가 찾게 한다
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share("pinky_description").parent)),
        server,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[Shutdown()])),
    ]
    if gui:
        actions.append(ExecuteProcess(cmd=["gz", "sim", "-g", "-v", "2"], name="gz_gui",
                                      output="screen"))
    return actions


def bridge(robots) -> Node:
    """Gazebo 시계와 모든 로봇의 토픽을 잇는 브리지 프로세스 하나. /clock은 한 번만 넣는다."""
    args, remaps = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], []
    for ns in robots:
        robot_args, robot_remaps = bridge_args(ns)
        args += robot_args
        remaps += robot_remaps
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
                output="screen", arguments=args, remappings=remaps,
                parameters=[{"use_sim_time": True}])


def _rviz_config(ns: str) -> str:
    text = (share("pode_nav_config") / "rviz" / "pode_nav.rviz").read_text(encoding="utf-8")
    out = Path(tempfile.gettempdir()) / f"pode_nav_{ns}.rviz"
    out.write_text(text.replace("<robot_namespace>", f"/{ns}"), encoding="utf-8")
    return str(out)


# 시뮬에서만 쓰는 AMCL 값(실물 설정은 그대로). 지도와 맞는 빔을 더 믿고 빔 수와 갱신 빈도를 올린다.
SIM_AMCL = {"z_hit": 0.95, "z_rand": 0.05, "max_beams": 120, "laser_max_range": 12.0,
            "update_min_d": 0.05, "update_min_a": 0.05}

# Nav2 controller 넷(pode_nav_config): 잠금 경계 경로, Retreat 경로, nav_goal(bt_navigator)용.
THROUGH_CONTROLLER = "FollowPathThrough"
REVERSE_CONTROLLER = "FollowPathReverse"
GOAL_CONTROLLER = "FollowPathGoal"
# 비교용 Nav2 goal MPPI(ControllerSelector로 고를 때만 씀). 속도 상한은 vx_max
MPPI_CONTROLLER = "FollowPathMPPI"
# RL 주행 정책(nav_overrides로만 띄움). 속도 상한은 max_linear_vel
RL_PLUGIN = "pode_rl_controller::FollowPathRL"
# ---- 회피 패키지 ----
AVOIDANCE_PACKAGE = "pode_avoidance"
AVOIDANCE_FILE = Path("params") / "avoidance.yaml"
GOAL_CONTROLLER_KEY = "follow_path_goal"
CONTROLLERS = ("FollowPath", THROUGH_CONTROLLER, REVERSE_CONTROLLER, GOAL_CONTROLLER)
# 고정 경로를 모는 controller. fixed_line_collision_check가 거짓이면 이 셋만 충돌 검사를 끈다(시뮬 전용, launch 인자 기본은 끔).
# 마당 goal(FollowPathGoal)은 늘 켠다.
FIXED_LINE_CONTROLLERS = ("FollowPath", THROUGH_CONTROLLER, REVERSE_CONTROLLER)
# 경기장 속도 상한. 기본과 위 끝은 pode_nav_config/params/nav_speed.yaml이 단일 출처다.
NAV_SPEED_FILE = Path("params") / "nav_speed.yaml"
SLOWEST_SPEED_MPS = 0.05
# sim_stand_in이 내는 로봇별 시뮬 배터리 토픽. low_battery가 있으면 pode_fsm이 본다
BATTERY_TOPIC = "/{robot}/sim/battery"


def nav_speed_mps(path: Path | None = None) -> float:
    """단일 출처의 주행 최고 속도(m/s)."""
    path = share("pode_nav_config") / NAV_SPEED_FILE if path is None else Path(path)
    value = float(yaml.safe_load(path.read_text(encoding="utf-8"))["nav_speed_mps"])
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{path}: nav_speed_mps must be above 0, got {value!r}")
    return value


def speed_of(text) -> float:
    """launch 인자 nav_max_speed. 비면 nav_speed_mps(). 오타가 최고 속도로 바뀌지 않게 막는다."""
    top = nav_speed_mps()
    if not str(text).strip():
        return top
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"nav_max_speed must be a number in m/s, got {text!r}") from None
    low, high = min(SLOWEST_SPEED_MPS, top), top
    if not (math.isfinite(value) and low <= value <= high):
        raise ValueError(f"nav_max_speed must be from {low} to {high} m/s, got {text!r}")
    return value


def merge(into: dict, extra: dict) -> dict:
    """extra를 into에 깊이 덮는다(dict끼리는 안으로, 나머지는 바꿔 넣음). into를 돌려준다."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(into.get(key), dict):
            merge(into[key], value)
        else:
            into[key] = value
    return into


# ---- 덮어쓰기가 매핑을 통째로 바꾸는 곳 ----
def replaced_mappings(into: dict, extra: dict, prefix: str = "") -> list[str]:
    found = []
    for key, value in extra.items():
        if not isinstance(into.get(key), dict):
            continue
        if isinstance(value, dict):
            found += replaced_mappings(into[key], value, f"{prefix}{key}.")
        else:
            found.append(f"{prefix}{key}")
    return found


def avoidance_file(package: str = AVOIDANCE_PACKAGE) -> Path:
    return share(package) / AVOIDANCE_FILE


def sim_nav_params(base: Path, out: Path, amcl: dict | None = None,
                   max_speed: float | None = None, overrides: dict | None = None,
                   fixed_line_check: bool = True, avoidance: Path | None = None) -> Path:
    """base(실물 Nav2 설정)에 시뮬 값을 덮어 out에 쓴다. base는 바꾸지 않는다.
    덮는 것: FollowPathGoal(회피 파일), AMCL(SIM_AMCL), overrides, controller 충돌 검사(fixed_line_check가 거짓일 때만 고정 경로 셋을 끔), max_speed 속도 상한.
    """
    data = yaml.safe_load(base.read_text(encoding="utf-8"))
    if GOAL_CONTROLLER not in data["controller_server"]["ros__parameters"] and avoidance:
        data["controller_server"]["ros__parameters"][GOAL_CONTROLLER] = yaml.safe_load(
            Path(avoidance).read_text(encoding="utf-8"))[GOAL_CONTROLLER_KEY]
    data["amcl"]["ros__parameters"].update(SIM_AMCL if amcl is None else amcl)
    replaced = replaced_mappings(data, overrides or {})
    if replaced:
        raise ValueError("nav_overrides may not replace a mapping with a non-mapping: "
                         + ", ".join(replaced))
    merge(data, copy.deepcopy(overrides or {}))
    controllers = data["controller_server"]["ros__parameters"]
    if MPPI_CONTROLLER in controllers and MPPI_CONTROLLER not in controllers["controller_plugins"]:
        controllers["controller_plugins"] = [*controllers["controller_plugins"], MPPI_CONTROLLER]
    missing = [name for name in CONTROLLERS if name not in controllers]
    if missing:
        raise ValueError(f"{base} has no controller {', '.join(missing)}")
    # FollowPathGoal은 RPP가 아닐 수 있어, 아래 두 RPP 키는 다른 플러그인에 넣지 않는다(속도 상한은 회피 파일이 지킨다)
    names = [name for name in CONTROLLERS
             if name != GOAL_CONTROLLER or "desired_linear_vel" in controllers[name]]
    rpps = [controllers[name] for name in names]
    for name in names:
        controllers[name]["use_collision_detection"] = (
            fixed_line_check or name not in FIXED_LINE_CONTROLLERS)
    if max_speed is not None:
        top = max(rpp["desired_linear_vel"] for rpp in rpps)
        if (isinstance(max_speed, bool) or not isinstance(max_speed, int | float)
                or not 0 < max_speed <= top):
            raise ValueError(f"max_speed {max_speed!r} must be a number above 0 and at most "
                             f"the robot's {top} m/s")
        for rpp in rpps:
            rpp["desired_linear_vel"] = min(float(rpp["desired_linear_vel"]), float(max_speed))
        if MPPI_CONTROLLER in controllers:
            mppi = controllers[MPPI_CONTROLLER]
            own = float(mppi["vx_max"])
            capped = {**mppi, "vx_max": min(own, float(max_speed))}
            if "time_steps" in mppi:
                capped["time_steps"] = round(int(mppi["time_steps"]) * own / capped["vx_max"])
            controllers[MPPI_CONTROLLER] = capped
        for name, rl in list(controllers.items()):
            if isinstance(rl, dict) and RL_PLUGIN in (rl.get("plugin"),
                                                      rl.get("primary_controller")):
                own = float(rl.get("max_linear_vel", 0.0))
                top = min(own, float(max_speed)) if own > 0 else float(max_speed)
                controllers[name] = {**rl, "max_linear_vel": top}
    out.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return out


def model(ns: str, x, y, yaw, lidar_noise: float | None = None) -> list:
    """Pode 모델만(robot_state_publisher와 Gazebo에 놓기). Nav2와 pode_fsm은 없다.
    lidar_noise: 라이다 거리 잡음 표준편차(m). None이면 모델 기본값(0.02).
    """
    me = share("fleet_gz_sim")
    noise = "" if lidar_noise is None else f" lidar_noise:={float(lidar_noise)}"
    description = ParameterValue(
        Command(["xacro ", str(me / "urdf" / "pode_sim.urdf.xacro"), f" name:={ns}{noise}"]),
        value_type=str)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=ns, output="screen", remappings=TF_REMAPS,
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", namespace=ns, output="screen",
             arguments=["-world", WORLD, "-name", ns, "-topic", f"/{ns}/robot_description",
                        "-x", str(x), "-y", str(y), "-z", "0.01", "-Y", str(yaw)]),
    ]


def pode(ns: str, x, y, yaw, *, nav_map: str, nav_map_fine: str, keepout_mask: str = "",
          heartbeat_timeout_s="1.5", nav_log_level="warn", fsm=True, rviz=False,
          dock_aside=False, nav_composition=False, through_controller_id=THROUGH_CONTROLLER,
          nav_max_speed: float | None = None, battery_topic: str = "",
          nav_overrides: dict | None = None, lidar_noise: float | None = None,
          fixed_line_check: bool = True, narrow_strips: str = "",
          wall_band_m: float = 0.05, peer_layer: bool = False,
          avoidance_package: str = AVOIDANCE_PACKAGE, vision_stand_in: bool = False,
          navigation_delay_s: float = 0.0) -> list:
    """Pode 한 대: 모델, Nav2, pode_fsm, RViz. 브리지는 bridge()에 이 로봇 이름을 넣는다.
    nav_map, nav_map_fine: 배치의 5 cm, 1 cm 지도. keepout_mask: 비면 keepout 없음. dock_aside: dock 서버를 /<ns>/dock_fsm으로 비킴.
    nav_max_speed: controller 최고 속도(m/s), None이면 nav_speed_mps(). fixed_line_check: 거짓이면 고정 경로 셋의 충돌 검사를 끈다.
    narrow_strips, wall_band_m: 스캔 거르개 좁은 띠(비면 거르개 없음). peer_layer: 다른 로봇 몸을 costmap에 찍는 층(임시).
    navigation_delay_s: 시험용, 0보다 크면 주행 쪽 서버만 이만큼 늦게 켠다.
    """
    nav = share("pode_nav_config")
    nav_max_speed = nav_speed_mps() if nav_max_speed is None else nav_max_speed
    x, y, yaw = str(x), str(y), str(yaw)
    actions = [
        *model(ns, x, y, yaw, lidar_noise),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / "launch" / "nav2.launch.py")),
            launch_arguments={
                "namespace": ns, "use_sim_time": "true",
                "map": nav_map, "map_fine": nav_map_fine, "keepout_mask": keepout_mask,
                "initial_x": x, "initial_y": y, "initial_yaw": yaw,
                "log_level": nav_log_level,
                "narrow_strips": narrow_strips, "wall_band_m": str(wall_band_m),
                "peer_obstacles": "true" if peer_layer else "false", "peer_node": "false",
                "avoidance_package": avoidance_package,
                "vision_stand_in": "true" if vision_stand_in else "false",
                "use_composition": "true" if nav_composition else "false",
                "navigation_delay_s": str(float(navigation_delay_s)),
                "params_file": str(sim_nav_params(
                    nav / "params" / "nav2_params.yaml",
                    Path(tempfile.gettempdir()) / f"nav2_params_sim_{ns}_{os.getpid()}.yaml",
                    max_speed=nav_max_speed, overrides=nav_overrides,
                    fixed_line_check=fixed_line_check,
                    avoidance=avoidance_file(avoidance_package))),
            }.items()),
    ]
    if fsm:
        fsm_args = {"namespace": ns, "use_sim_time": "true",
                    "heartbeat_timeout_s": str(heartbeat_timeout_s),
                    "through_controller_id": through_controller_id,
                    "horizon_cap_mps": str(nav_max_speed)}
        if battery_topic:
            fsm_args["battery_topic"] = battery_topic
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share("pode_fsm") / "launch" /
                                              "pode_fsm.launch.py")),
            launch_arguments=fsm_args.items())
        actions.append(GroupAction([SetRemap(f"/{ns}/dock", f"/{ns}/dock_fsm"), include])
                       if dock_aside else include)
    if rviz:
        actions.append(Node(package="rviz2", executable="rviz2", namespace=ns, output="log",
                            arguments=["-d", _rviz_config(ns)], remappings=TF_REMAPS,
                            parameters=[{"use_sim_time": True}]))
    return actions
