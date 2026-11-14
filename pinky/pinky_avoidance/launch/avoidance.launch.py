# ---- 로봇 한 대의 회피 쪽 노드를 띄우는 launch 파일 ----
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue

ARGS = {
    "namespace": "pinky1",
    "use_sim_time": "false",
    "log_level": "info",
    "peer_obstacles": "false",
    "peer_node": "true",
    "peer_robots": "",
    "vision_obstacles": "false",
    "vision_stand_in": "false",
}


# ---- 스위치와 로봇 목록 검사 ----
class Flag(Substitution):
    def __init__(self, name, value):
        super().__init__()
        self._name = name
        self._value = normalize_to_list_of_substitutions(value)

    def perform(self, context) -> str:
        text = perform_substitutions(context, self._value).strip().lower()
        if text not in ("true", "false"):
            raise ValueError(f"{self._name} must be true or false, got {text!r}")
        return text


class PeerRobots(Substitution):
    def __init__(self, on, robots):
        super().__init__()
        self._on = normalize_to_list_of_substitutions(on)
        self._robots = normalize_to_list_of_substitutions(robots)

    def perform(self, context) -> str:
        robots = perform_substitutions(context, self._robots).strip()
        if perform_substitutions(context, self._on).lower() == "true" and not robots:
            raise ValueError("peer_obstacles:=true needs peer_robots (pinky1,pinky2,pinky3),"
                             " or peer_node:=false when another process sends the bodies")
        return robots


# ---- 노드 구성 ----
def generate_launch_description():
    arg = LaunchConfiguration
    namespace = arg("namespace")
    log = ["--ros-args", "--log-level", arg("log_level")]
    peer_node = AndSubstitution(Flag("peer_obstacles", arg("peer_obstacles")),
                                Flag("peer_node", arg("peer_node")))
    peer_obstacles = Node(
        package="pinky_peer_obstacles", executable="peer_obstacles", name="peer_obstacles",
        output="screen", arguments=log, condition=IfCondition(peer_node),
        parameters=[{"robot_id": ParameterValue(namespace, value_type=str),
                     "robots": ParameterValue(PeerRobots(peer_node, arg("peer_robots")),
                                              value_type=str)}])
    vision_stand_in = Node(
        package="pinky_avoidance", executable="vision_stand_in", name="vision_stand_in",
        output="screen", arguments=log,
        condition=IfCondition(Flag("vision_stand_in", arg("vision_stand_in"))),
        parameters=[{"robot_id": ParameterValue(namespace, value_type=str)}])
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in ARGS.items()),
        SetParameter("use_sim_time", arg("use_sim_time")),
        peer_obstacles,
        vision_stand_in,
    ])
