"""one_pinky.launch.py와 gz.launch.py가 같이 쓰는 launch 조각: Gazebo 월드 하나, 브리지 하나,
핑키 한 대씩.

월드 좌표 = 지도 좌표 = 구역 지도 좌표다(fleet_gz_sim.arena).
ROS 프로세스 하나가 DDS 참여자 하나다. LOCALHOST 탐색에서는 32개 정도만 서로 보이므로
브리지는 모든 로봇을 하나로 맡고, 3대일 때는 Nav2도 로봇마다 프로세스 하나로 묶는다.
"""
from __future__ import annotations

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
    """gz 토픽 /model/<ns>/... ↔ ROS 토픽 /<ns>/... /clock은 bridge()가 한 번만 넣는다."""
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
    """Gazebo 서버(끝나면 launch도 끝)와 화면. 브리지는 bridge()가 따로 만든다.
    어느 배치든 월드 이름은 WORLD다(arena.py)."""
    server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v", "2", world_file],
                            name="gz_server", output="screen")
    actions = [
        # Gazebo 통신(gz-transport)도 이 컴퓨터 안에서만. 셸에 남은 GZ_IP(LAN 주소일 수 있다)는
        # 쓰지 않는다. 다른 시뮬과 섞이지 않게 칸막이를 나눈다
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
    """Gazebo 시계와 모든 로봇의 토픽을 잇는 브리지 프로세스 하나.

    /clock을 여럿이 내면 표본 순서가 섞여 시간이 뒤로 갈 수 있으므로 한 번만 넣는다.
    토픽 이름이 모두 절대 이름이라 namespace는 필요 없다."""
    args, remaps = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], []
    for ns in robots:
        robot_args, robot_remaps = bridge_args(ns)
        args += robot_args
        remaps += robot_remaps
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
                output="screen", arguments=args, remappings=remaps,
                parameters=[{"use_sim_time": True}])


def _rviz_config(ns: str) -> str:
    text = (share("pinky_nav_config") / "rviz" / "pinky_nav.rviz").read_text(encoding="utf-8")
    out = Path(tempfile.gettempdir()) / f"pinky_nav_{ns}.rviz"
    out.write_text(text.replace("<robot_namespace>", f"/{ns}"), encoding="utf-8")
    return str(out)


# 시뮬에서만 쓰는 AMCL 값. 실물 설정(pinky_nav_config/params/nav2_params.yaml)은 그대로 둔다.
# 창고 지도 A4->A3 통로(양쪽 랙까지 0.25 m)에서 실물 값(z_hit 0.5, z_rand 0.5, 빔 60, 0.15 m나
# 0.1 rad마다 갱신)은 AMCL 위치가 실제 위치에서 옆으로 0.13~0.29 m 떨어진 채 머물러, 로봇이
# 랙에 붙고 격리됐다(2026-11-08). 지도와 맞는 빔을 더 믿고, 빔을 늘리고, 더 자주 갱신한다.
SIM_AMCL = {"z_hit": 0.95, "z_rand": 0.05, "max_beams": 120, "laser_max_range": 12.0,
            "update_min_d": 0.05, "update_min_a": 0.05}

# Nav2 controller 넷(pinky_nav_config). pinky_fsm은 잠금 경계 경로에 FollowPathThrough,
# Retreat path에 FollowPathReverse, mode nav_goal(bt_navigator)에 FollowPathGoal을 쓴다.
THROUGH_CONTROLLER = "FollowPathThrough"
REVERSE_CONTROLLER = "FollowPathReverse"
GOAL_CONTROLLER = "FollowPathGoal"
CONTROLLERS = ("FollowPath", THROUGH_CONTROLLER, REVERSE_CONTROLLER, GOAL_CONTROLLER)
# 경기장 속도 상한(지시서 3절: 0.12~0.15 m/s). 앞으로 가는 controller 셋이 이 값으로 간다
SPEED_RANGE_MPS = (0.12, 0.15)
DEFAULT_SPEED_MPS = 0.12
# sim_stand_in이 내는 로봇마다의 시뮬 배터리(BatteryState). low_battery가 있으면 pinky_fsm이 본다
BATTERY_TOPIC = "/{robot}/sim/battery"


def speed_of(text) -> float:
    """launch 인자 nav_max_speed. 비면 DEFAULT_SPEED_MPS. 오타가 최고 속도로 바뀌지 않게 막는다."""
    if not str(text).strip():
        return DEFAULT_SPEED_MPS
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"nav_max_speed must be a number in m/s, got {text!r}") from None
    low, high = SPEED_RANGE_MPS
    if not (math.isfinite(value) and low <= value <= high):
        raise ValueError(f"nav_max_speed must be from {low} to {high} m/s, got {text!r}")
    return value


def sim_nav_params(base: Path, out: Path, amcl: dict | None = None,
                   max_speed: float | None = None) -> Path:
    """base(실물 Nav2 설정)에 시뮬 값을 덮어 out에 쓴다. base는 바꾸지 않는다.

    - AMCL: SIM_AMCL(또는 amcl).
    - controller 넷(CONTROLLERS) 모두 충돌 검사를 켠다(지시서 3절: 모든 실행에서 켬).
    - max_speed가 있으면 controller마다 desired_linear_vel을 max_speed와 원래 값 중 작은 것으로
      (후진 FollowPathReverse 0.05 m/s는 그대로 남는다)."""
    data = yaml.safe_load(base.read_text(encoding="utf-8"))
    data["amcl"]["ros__parameters"].update(SIM_AMCL if amcl is None else amcl)
    controllers = data["controller_server"]["ros__parameters"]
    missing = [name for name in CONTROLLERS if name not in controllers]
    if missing:
        raise ValueError(f"{base} has no controller {', '.join(missing)}")
    rpps = [controllers[name] for name in CONTROLLERS]
    for rpp in rpps:
        rpp["use_collision_detection"] = True
    if max_speed is not None:
        top = max(rpp["desired_linear_vel"] for rpp in rpps)
        if (isinstance(max_speed, bool) or not isinstance(max_speed, int | float)
                or not 0 < max_speed <= top):
            raise ValueError(f"max_speed {max_speed!r} must be a number above 0 and at most "
                             f"the robot's {top} m/s")
        for rpp in rpps:
            rpp["desired_linear_vel"] = min(float(rpp["desired_linear_vel"]), float(max_speed))
    out.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return out


def model(ns: str, x, y, yaw) -> list:
    """핑키 모델만(robot_state_publisher와 Gazebo에 놓기). Nav2와 pinky_fsm은 없다.
    서 있는 다른 핑키(one_pinky.launch.py still_pinky)도 이것으로 놓는다."""
    me = share("fleet_gz_sim")
    description = ParameterValue(
        Command(["xacro ", str(me / "urdf" / "pinky_sim.urdf.xacro"), f" name:={ns}"]),
        value_type=str)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=ns, output="screen", remappings=TF_REMAPS,
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", namespace=ns, output="screen",
             arguments=["-world", WORLD, "-name", ns, "-topic", f"/{ns}/robot_description",
                        "-x", str(x), "-y", str(y), "-z", "0.01", "-Y", str(yaw)]),
    ]


def pinky(ns: str, x, y, yaw, *, nav_map: str, nav_map_fine: str, keepout_mask: str = "",
          heartbeat_timeout_s="1.5", nav_log_level="warn", fsm=True, rviz=False,
          dock_aside=False, nav_composition=False, through_controller_id=THROUGH_CONTROLLER,
          nav_max_speed: float = DEFAULT_SPEED_MPS, battery_topic: str = "") -> list:
    """핑키 한 대: 모델, Nav2, pinky_fsm, RViz. 브리지는 bridge()에 이 로봇 이름을 넣는다.

    nav_map, nav_map_fine: Nav2 지도 둘(배치의 5 cm, 1 cm 지도, fleet_setup.Arena).
    keepout_mask: global costmap keepout 마스크(pinky_nav_config keepout_mask). 비면 keepout 없음.
    dock_aside: pinky_fsm의 dock 서버를 /<ns>/dock_fsm으로 비켜 둔다(시뮬 전용 도크 대역용).
    nav_composition: Nav2 노드들을 프로세스 하나(component container)로 띄운다.
    through_controller_id: 잠금 경계 경로에 쓸 controller(pinky_fsm). 비우면 모든 경로가
    FollowPath로 간다. Nav2는 controller 넷(CONTROLLERS)을 늘 띄우고 모두 충돌 검사를 켠다.
    nav_max_speed: controller들의 최고 속도(m/s). pinky_fsm이 잠금 경계에서 내는 속도 제한의
    상한(horizon_cap_mps)도 이 값으로: RPP는 그 제한을 속도로 쓴다.
    battery_topic: pinky_fsm이 볼 BatteryState 토픽(sim_stand_in의 BATTERY_TOPIC). 비면 없음."""
    nav = share("pinky_nav_config")
    x, y, yaw = str(x), str(y), str(yaw)
    actions = [
        *model(ns, x, y, yaw),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / "launch" / "nav2.launch.py")),
            launch_arguments={
                "namespace": ns, "use_sim_time": "true",
                "map": nav_map, "map_fine": nav_map_fine, "keepout_mask": keepout_mask,
                "initial_x": x, "initial_y": y, "initial_yaw": yaw,
                "log_level": nav_log_level,
                "use_composition": "true" if nav_composition else "false",
                "params_file": str(sim_nav_params(
                    nav / "params" / "nav2_params.yaml",
                    Path(tempfile.gettempdir()) / f"nav2_params_sim_{ns}_{os.getpid()}.yaml",
                    max_speed=nav_max_speed)),
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
            PythonLaunchDescriptionSource(str(share("pinky_fsm") / "launch" /
                                              "pinky_fsm.launch.py")),
            launch_arguments=fsm_args.items())
        actions.append(GroupAction([SetRemap(f"/{ns}/dock", f"/{ns}/dock_fsm"), include])
                       if dock_aside else include)
    if rviz:
        actions.append(Node(package="rviz2", executable="rviz2", namespace=ns, output="log",
                            arguments=["-d", _rviz_config(ns)], remappings=TF_REMAPS,
                            parameters=[{"use_sim_time": True}]))
    return actions
