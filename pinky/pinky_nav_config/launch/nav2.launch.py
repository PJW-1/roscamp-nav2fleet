# ---- Nav2 노드들을 띄우는 launch 파일 ----
import sys
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitution import Substitution
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile, ParameterValue
from nav2_common.launch import ReplaceString, RewrittenYaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pinky_nav_params import merge, with_avoidance, with_peers  # noqa: E402

# ---- 경로와 노드 그룹 상수 ----
SHARE = Path(get_package_share_directory("pinky_nav_config"))
LOCALIZATION = ["map_server", "map_server_fine", "amcl"]
NAVIGATION = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
KEEPOUT = ["keepout_mask_server", "costmap_filter_info_server"]
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


# ---- 파라미터 읽기와 launch 치환 클래스 ----
def _read(path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


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


class WithOverlays(Substitution):
    def __init__(self, source, avoidance, mask, vision, strips, peers):
        super().__init__()
        self._source = normalize_to_list_of_substitutions(source)
        self._avoidance = normalize_to_list_of_substitutions(avoidance)
        self._mask = normalize_to_list_of_substitutions(mask)
        self._vision = normalize_to_list_of_substitutions(vision)
        self._strips = normalize_to_list_of_substitutions(strips)
        self._peers = normalize_to_list_of_substitutions(peers)

    def perform(self, context) -> str:
        data = _read(perform_substitutions(context, self._source))
        avoidance = perform_substitutions(context, self._avoidance)
        vision = perform_substitutions(context, self._vision).lower() == "true"
        if avoidance:
            with_avoidance(data, _read(avoidance), vision)
        elif vision:
            raise ValueError("vision_obstacles:=true needs avoidance_params_file")
        if perform_substitutions(context, self._mask):
            merge(data, _read(SHARE / "params" / "keepout.yaml"))
        if perform_substitutions(context, self._strips).strip():
            merge(data, _read(SHARE / "params" / "scan_filter.yaml"))
        if perform_substitutions(context, self._peers) == "true":
            with_peers(data, _read(SHARE / "params" / "peer_obstacles.yaml"))
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", prefix="nav2_overlay_",
                                         delete=False, encoding="utf-8") as out:
            yaml.safe_dump(data, out, sort_keys=False, allow_unicode=True)
        return out.name


# ---- launch 인자와 노드 구성을 만드는 함수 ----
def generate_launch_description():
    arg = LaunchConfiguration
    namespace = arg("namespace")
    keepout = IfCondition(NotEqualsSubstitution(arg("keepout_mask"), ""))
    filtered = IfCondition(NotEqualsSubstitution(arg("narrow_strips"), ""))
    peers_on = Flag("peer_obstacles", arg("peer_obstacles"))
    peer_node = AndSubstitution(peers_on, Flag("peer_node", arg("peer_node")))
    params = ParameterFile(RewrittenYaml(
        source_file=ReplaceString(source_file=WithOverlays(arg("params_file"),
                                                           arg("avoidance_params_file"),
                                                           arg("keepout_mask"),
                                                           arg("vision_obstacles"),
                                                           arg("narrow_strips"), peers_on),
                                  replacements={"<robot_namespace>": ("/", namespace)}),
        root_key=namespace, param_rewrites={}, convert_types=True), allow_substs=True)
    log = ["--ros-args", "--log-level", arg("log_level")]

    # ---- 라이프사이클 매니저 파라미터와 초기 위치 ----
    def manager_params(nodes):
        return {"autostart": ParameterValue(arg("autostart"), value_type=bool),
                "node_names": nodes}

    initial_pose = {
        "set_initial_pose": True,
        "initial_pose.x": ParameterValue(arg("initial_x"), value_type=float),
        "initial_pose.y": ParameterValue(arg("initial_y"), value_type=float),
        "initial_pose.yaw": ParameterValue(arg("initial_yaw"), value_type=float),
    }
    # ---- Nav2·keepout 노드와 라이프사이클 매니저 목록 ----
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
    managers = [("lifecycle_manager_localization", LOCALIZATION, None),
                ("lifecycle_manager_navigation", NAVIGATION, None)]
    keepout_manager = [("lifecycle_manager_keepout", KEEPOUT, keepout)]

    # ---- 노드를 프로세스·컴포넌트로 만드는 헬퍼 ----
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

    # ---- 컴포넌트 컨테이너와 보조 노드 ----
    container = ComposableNodeContainer(
        name="nav2_container", namespace="", package="rclcpp_components",
        executable="component_container_isolated", output="screen", arguments=log,
        parameters=[params], remappings=TF_REMAPS,
        composable_node_descriptions=(components(nav_nodes) + manager_components(managers)
                                      + components(keepout_nodes)
                                      + manager_components(keepout_manager)))
    scan_filter = Node(
        package="pinky_scan_filter", executable="scan_filter", name="scan_filter",
        output="screen", arguments=log, remappings=TF_REMAPS, condition=filtered,
        parameters=[{"wall_band_m": ParameterValue(arg("wall_band_m"), value_type=float),
                     "strips": ParameterValue(arg("narrow_strips"), value_type=str)}])
    peer_obstacles = Node(
        package="pinky_peer_obstacles", executable="peer_obstacles", name="peer_obstacles",
        output="screen", arguments=log, condition=IfCondition(peer_node),
        parameters=[{"robot_id": ParameterValue(namespace, value_type=str),
                     "robots": ParameterValue(PeerRobots(peer_node, arg("peer_robots")),
                                              value_type=str)}])
    use_composition = arg("use_composition")
    # ---- LaunchDescription 반환 ----
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pinky1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", description="5 cm 지도 yaml(costmap)"),
        DeclareLaunchArgument("map_fine", default_value=arg("map"),
                              description="1 cm 지도 yaml(AMCL, R15). 없으면 map"),
        DeclareLaunchArgument("params_file",
                              default_value=str(SHARE / "params" / "nav2_params.yaml"),
                              description="사용자의 Nav2 설정"),
        DeclareLaunchArgument("avoidance_params_file",
                              default_value=str(SHARE / "params" / "avoidance.yaml"),
                              description="회피 담당의 설정(FollowPathGoal 값, 비전 관측원 값)."
                                          " 사용자의 키가 들어 있으면 뜨지 않는다"),
        DeclareLaunchArgument(
            "bt_xml", default_value=str(
                SHARE / "bt" / "navigate_to_pose_w_replanning_and_recovery_goal.xml")),
        DeclareLaunchArgument("keepout_mask", default_value="",
                              description="급행·도크 구역 keepout 마스크 yaml(map_server 형식)."
                                          " 비면 keepout 필터와 그 서버를 띄우지 않는다"),
        DeclareLaunchArgument("vision_obstacles", default_value="false",
                              description="true면 두 costmap의 장애물 층에"
                                          " /<로봇>/vision_obstacles(PointCloud2)를 더한다"),
        DeclareLaunchArgument("narrow_strips", default_value="",
                              description="좁은 띠 사각형 \"x0,y0,x1,y1; ...\"(map 좌표)."
                                          " 주면 스캔 거르개를 띄우고"
                                          " costmap이 거른 스캔을 읽는다"),
        DeclareLaunchArgument("wall_band_m", default_value="0.05",
                              description="스캔 거르개: 벽·진열대에서 이만큼 안의 점을 지운다"),
        DeclareLaunchArgument("peer_obstacles", default_value="false",
                              description="true면(임시, 주행 시험용) 다른 로봇의 몸을 두 costmap에"
                                          " 찍는 peer 층과 pinky_peer_obstacles 노드를 띄운다"),
        DeclareLaunchArgument("peer_node", default_value="true",
                              description="false면 peer 층만 넣고 pinky_peer_obstacles 노드는"
                                          " 띄우지 않는다(다른 프로세스가 토픽을 낸다)"),
        DeclareLaunchArgument("peer_robots", default_value="",
                              description="peer_obstacles:=true일 때 로봇 이름들"
                                          "(pinky1,pinky2,pinky3). 나를 뺀 로봇의 몸을 찍는다"),
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
        GroupAction([
            PushRosNamespace(namespace),
            SetParameter("use_sim_time", arg("use_sim_time")),
            scan_filter,
            peer_obstacles,
        ]),
    ])
