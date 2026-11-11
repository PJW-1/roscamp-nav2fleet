"""핑키 한 대의 Nav2를 로봇 namespace 안에 띄운다.

    ros2 launch pinky_nav_config nav2.launch.py map:=<지도.yaml> map_fine:=<1cm 지도.yaml> \
        initial_x:=0.35 initial_y:=1.55 initial_yaw:=1.5708 [keepout_mask:=<마스크.yaml>] \
        [vision_obstacles:=true]

- 노드: map_server(5 cm, map), map_server_fine(1 cm, map_fine), amcl, controller_server,
  planner_server, behavior_server, bt_navigator와 lifecycle manager 둘.
- 모두 /{namespace} 안에서 돌고 TF는 /{namespace}/tf, /{namespace}/tf_static을 쓴다(R17).
  프레임 이름(map, odom, base_footprint)에는 접두사를 붙이지 않는다.
- controller_server는 cmd_vel로 바로 낸다(velocity_smoother, collision_monitor 없음).
- AMCL은 1 cm 지도를, costmap은 5 cm 지도를 쓴다(R15).
- bt_navigator는 MoveTo mode nav_goal(계약 0.11-draft 제안)의 navigate_to_pose다. BT는 Nav2 기본
  BT(1 Hz 재계획, 복구)에서 controller만 FollowPathGoal로 바꾼 것이다.
- keepout_mask(map_server 형식의 마스크 yaml)를 주면 급행·도크 구역을 Nav2 goal 경로에서 막는다:
  params/keepout.yaml을 파라미터에 덧붙여 global costmap에 keepout 필터를 걸고,
  keepout_mask_server, costmap_filter_info_server와 lifecycle_manager_keepout을 더 띄운다.
  비면(기본) 아무것도 바뀌지 않는다.
- vision_obstacles:=true면 params/vision_obstacles.yaml을 덧붙여 두 costmap의 장애물 층에
  /{namespace}/vision_obstacles(PointCloud2)를 관측원으로 더한다(회피 담당의 자리, 기본 꺼짐).
- use_composition:=true면 같은 노드들을 컨테이너(component_container_isolated) 한 프로세스에
  올린다. 한 PC에 로봇 여럿을 띄울 때 DDS 참가자(프로세스) 수를 줄인다.
  기본은 false(노드마다 프로세스).
"""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, NotEqualsSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml

SHARE = Path(get_package_share_directory("pinky_nav_config"))
LOCALIZATION = ["map_server", "map_server_fine", "amcl"]
NAVIGATION = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
KEEPOUT = ["keepout_mask_server", "costmap_filter_info_server"]
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def _merge(base: dict, extra: dict) -> None:
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


class WithOverlays(Substitution):
    """파라미터 파일 경로. 켜진 덧붙임을 얹은 임시 파일이고, 켜진 것이 없으면 그대로다.

    mask가 있으면 params/keepout.yaml을, vision이 true면 params/vision_obstacles.yaml을 얹는다.
    """

    def __init__(self, source, mask, vision):
        super().__init__()
        self._source = normalize_to_list_of_substitutions(source)
        self._mask = normalize_to_list_of_substitutions(mask)
        self._vision = normalize_to_list_of_substitutions(vision)

    def perform(self, context) -> str:
        source = perform_substitutions(context, self._source)
        overlays = []
        if perform_substitutions(context, self._mask):
            overlays.append("keepout.yaml")
        if perform_substitutions(context, self._vision).lower() == "true":
            overlays.append("vision_obstacles.yaml")
        if not overlays:
            return source
        data = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
        for name in overlays:
            _merge(data, yaml.safe_load((SHARE / "params" / name).read_text(encoding="utf-8")))
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", prefix="nav2_overlay_",
                                         delete=False, encoding="utf-8") as out:
            yaml.safe_dump(data, out, sort_keys=False, allow_unicode=True)
        return out.name


def generate_launch_description():
    arg = LaunchConfiguration
    namespace = arg("namespace")
    keepout = IfCondition(NotEqualsSubstitution(arg("keepout_mask"), ""))
    # costmap 층의 절대 토픽 이름(<robot_namespace>/scan)을 채운 뒤
    # 노드 이름 키를 namespace 아래에 넣는다
    params = ParameterFile(RewrittenYaml(
        source_file=ReplaceString(source_file=WithOverlays(arg("params_file"),
                                                           arg("keepout_mask"),
                                                           arg("vision_obstacles")),
                                  replacements={"<robot_namespace>": ("/", namespace)}),
        root_key=namespace, param_rewrites={}, convert_types=True), allow_substs=True)
    log = ["--ros-args", "--log-level", arg("log_level")]

    def manager_params(nodes):
        return {"autostart": ParameterValue(arg("autostart"), value_type=bool),
                "node_names": nodes}

    initial_pose = {
        "set_initial_pose": True,
        "initial_pose.x": ParameterValue(arg("initial_x"), value_type=float),
        "initial_pose.y": ParameterValue(arg("initial_y"), value_type=float),
        "initial_pose.yaw": ParameterValue(arg("initial_yaw"), value_type=float),
    }
    # (패키지, 실행 파일, 컴포넌트 플러그인, 노드 이름, 덧붙일 파라미터 목록, 조건)
    nav_nodes = [
        ("nav2_map_server", "map_server", "nav2_map_server::MapServer", "map_server",
         [{"yaml_filename": arg("map")}], None),
        ("nav2_map_server", "map_server", "nav2_map_server::MapServer", "map_server_fine",
         [{"yaml_filename": arg("map_fine")}], None),
        ("nav2_amcl", "amcl", "nav2_amcl::AmclNode", "amcl", [initial_pose], None),
        ("nav2_controller", "controller_server", "nav2_controller::ControllerServer",
         "controller_server", [], None),
        ("nav2_planner", "planner_server", "nav2_planner::PlannerServer", "planner_server", [],
         None),
        ("nav2_behaviors", "behavior_server", "behavior_server::BehaviorServer",
         "behavior_server", [], None),
        ("nav2_bt_navigator", "bt_navigator", "nav2_bt_navigator::BtNavigator", "bt_navigator",
         [{"default_nav_to_pose_bt_xml": arg("bt_xml")}], None),
    ]
    keepout_nodes = [
        ("nav2_map_server", "map_server", "nav2_map_server::MapServer", "keepout_mask_server",
         [{"yaml_filename": arg("keepout_mask")}], keepout),
        ("nav2_map_server", "costmap_filter_info_server",
         "nav2_map_server::CostmapFilterInfoServer", "costmap_filter_info_server", [], keepout),
    ]
    # (이름, 노드 이름들, 조건)
    managers = [("lifecycle_manager_localization", LOCALIZATION, None),
                ("lifecycle_manager_navigation", NAVIGATION, None)]
    keepout_manager = [("lifecycle_manager_keepout", KEEPOUT, keepout)]

    def processes(nodes):
        return [Node(package=package, executable=executable, name=name, output="screen",
                     parameters=[params, *extra], remappings=TF_REMAPS, arguments=log,
                     condition=cond)
                for package, executable, _, name, extra, cond in nodes]

    def manager_processes(entries):
        return [Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                     name=name, output="screen", arguments=log,
                     parameters=[manager_params(nodes)], condition=cond)
                for name, nodes, cond in entries]

    def components(nodes):
        return [ComposableNode(package=package, plugin=plugin, name=name,
                               parameters=[params, *extra], remappings=TF_REMAPS,
                               condition=cond)
                for package, _, plugin, name, extra, cond in nodes]

    def manager_components(entries):
        return [ComposableNode(package="nav2_lifecycle_manager",
                               plugin="nav2_lifecycle_manager::LifecycleManager", name=name,
                               parameters=[manager_params(nodes)], condition=cond)
                for name, nodes, cond in entries]

    # 컴포넌트의 namespace와 SetParameter(use_sim_time)는 LoadNode 요청을 만들 때 이 GroupAction
    # 범위에서 읽힌다(launch_ros가 execute 안에서 요청을 만든다). costmap처럼 노드 안에서 만드는
    # 하위 노드는 요청이 아니라 프로세스 인자로 파라미터를 받으므로 컨테이너에도 params를 준다
    # (nav2_bringup과 같다). keepout 필터도 그래서 컨테이너의 params로 global costmap에 닿는다.
    container = ComposableNodeContainer(
        name="nav2_container", namespace="", package="rclcpp_components",
        executable="component_container_isolated", output="screen", arguments=log,
        parameters=[params], remappings=TF_REMAPS,
        composable_node_descriptions=(components(nav_nodes) + manager_components(managers)
                                      + components(keepout_nodes)
                                      + manager_components(keepout_manager)))
    use_composition = arg("use_composition")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pinky1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", description="5 cm 지도 yaml(costmap)"),
        DeclareLaunchArgument("map_fine", default_value=arg("map"),
                              description="1 cm 지도 yaml(AMCL, R15). 없으면 map"),
        DeclareLaunchArgument("params_file",
                              default_value=str(SHARE / "params" / "nav2_params.yaml")),
        DeclareLaunchArgument(
            "bt_xml", default_value=str(
                SHARE / "bt" / "navigate_to_pose_w_replanning_and_recovery_goal.xml")),
        DeclareLaunchArgument("keepout_mask", default_value="",
                              description="급행·도크 구역 keepout 마스크 yaml(map_server 형식)."
                                          " 비면 keepout 필터와 그 서버를 띄우지 않는다"),
        DeclareLaunchArgument("vision_obstacles", default_value="false",
                              description="true면 두 costmap의 장애물 층에"
                                          " /<로봇>/vision_obstacles(PointCloud2)를 더한다"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_composition", default_value="false",
                              description="true면 노드들을 컨테이너 한 프로세스로 띄운다"),
        GroupAction([
            PushRosNamespace(namespace),
            SetParameter("use_sim_time", arg("use_sim_time")),
            *processes(nav_nodes), *manager_processes(managers),
            *processes(keepout_nodes), *manager_processes(keepout_manager),
        ], condition=UnlessCondition(use_composition)),
        GroupAction([
            PushRosNamespace(namespace),
            SetParameter("use_sim_time", arg("use_sim_time")),
            container,
        ], condition=IfCondition(use_composition)),
    ])
