"""PC에서 플릿 매니저를 띄운다. 가짜와 실물 장치를 섞어 쓸 수 있다.

모두 가짜:  ros2 launch iponic_bringup pc.launch.py use_fake:=true scenario:=shared_lane_failure
섞기:       ros2 launch iponic_bringup pc.launch.py use_fake:=true fake_robots:="pode2 pode3"
실물 팔:    ros2 launch iponic_bringup pc.launch.py use_fake:=true fake_arms:="arm_ship"
모두 실물:  ros2 launch iponic_bringup pc.launch.py use_fake:=false
실물 두 대 시험(로봇만 실물, 도킹은 PC가 받고 로봇은 DOCK_STUB=1):
            ros2 launch iponic_bringup pc.launch.py use_fake:=true use_sim_clock:=false \
                fake_robots:=none robots:="pode1 pode2" map_file:=v7 \
                scenario:=real_two_orders monitor:=false dock_wait_robots:="pode1 pode2"
세 대는 robots와 dock_wait_robots에 pode3을 더하고 scenario:=real_three_orders로 둔다.

record:=PATH는 tools/replay.py용 FMS 기록을 남긴다.
fake_arms: arm_left와 arm_right는 함께 적는다(fill_basket 서버 하나가 둘을 맡는다). arm_ship은 가짜 출고 팔이고,
none은 가짜 팔 없음(셋 모두 실물)이다.
monitor는 시뮬 전용 자세(/sim/truth)를 읽으므로 실물 시험에서는 false로 둔다.
dock_wait_robots: 실물 로봇의 도킹을 PC가 대신 받는다(fake_robots real_dock_wait_node).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _scenario_map(scenario: str) -> str:
    """시나리오 파일의 `map`이 고른 지도. 없으면 빈 값."""
    import os

    from ament_index_python.packages import get_package_share_directory
    from fake_robots.scenario import scenario_map_file
    share = get_package_share_directory("fake_robots")
    path = scenario if os.path.isfile(scenario) else os.path.join(
        share, "scenarios", scenario if scenario.endswith(".yaml") else scenario + ".yaml")
    return scenario_map_file(path) if os.path.isfile(path) else ""


def _order_params(get) -> dict:
    """order 모델 인자. 비우면 params 파일의 값이 쓰인다."""
    out = {}
    if get("use_order_model"):
        out["use_order_model"] = get("use_order_model").lower() == "true"
    if get("lookahead_s"):
        out["lookahead_s"] = float(get("lookahead_s"))
    return out


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    use_fake = get("use_fake").lower() == "true"
    use_sim_clock = use_fake and get("use_sim_clock").lower() == "true"
    record = get("record")
    map_file = get("map_file")
    if use_fake and not map_file:
        map_file = _scenario_map(get("scenario"))

    actions = [
        Node(
            package="fleet_manager",
            executable="fleet_manager_node",
            name="fleet_manager",
            output="screen",
            arguments=["--record", record] if record else [],
            parameters=[{
                "use_sim_time": use_sim_clock,
                "robots": get("robots").split(),
                "map_file": map_file,
                "params_file": get("params_file"),
                "contract_file": get("contract_file"),
                **_order_params(lambda name: LaunchConfiguration(name, default="").perform(
                    context)),
            }],
        ),
    ]
    if use_fake:
        share = FindPackageShare("fake_robots").perform(context)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{share}/launch/fakes.launch.py"),
            launch_arguments={
                "scenario": get("scenario"),
                "fake_robots": get("fake_robots"),
                "fake_arms": get("fake_arms"),
                "fake_central": get("fake_central"),
                "use_sim_clock": str(use_sim_clock).lower(),
                "result_dir": get("result_dir"),
                "map_file": map_file,
                "params_file": get("params_file"),
                "contract_file": get("contract_file"),
                "monitor": LaunchConfiguration("monitor", default="true").perform(context),
                "dock_wait_robots": LaunchConfiguration(
                    "dock_wait_robots", default="").perform(context),
            }.items(),
        ))
    return actions


def generate_launch_description():
    defaults = {
        "use_fake": "true",
        "scenario": "basic_orders",
        "robots": "pode1 pode2 pode3",
        "fake_robots": "pode1 pode2 pode3",
        "fake_arms": "arm_left arm_right arm_ship",
        "fake_central": "true",
        "use_sim_clock": "true",
        "result_dir": "/tmp/iponic_result",
        "map_file": "",
        "params_file": "",
        "contract_file": "",
        "record": "",
        "use_order_model": "",  # true/false
        "lookahead_s": "",  # 초
        "monitor": "true",  # 시나리오 감시 노드
        "dock_wait_robots": "",  # 도킹을 PC가 받을 실물 로봇
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    return LaunchDescription([*declared, OpaqueFunction(function=_setup)])
