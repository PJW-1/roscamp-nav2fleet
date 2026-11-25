"""bag 재생용 AMCL: map_server_fine, amcl, lifecycle 관리자만 sim time으로 띄운다.

AMCL 값은 저장소 pode_nav_config/params/nav2_params.yaml의 amcl 절을 그대로 쓴다.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = Path(__file__).resolve().parents[3]
PARAMS = REPO / "pode" / "pode_nav_config" / "params" / "nav2_params.yaml"


def _flat(d: dict, pre: str = "") -> dict:
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flat(v, f"{pre}{k}."))
        else:
            out[f"{pre}{k}"] = v
    return out


def _nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    ns = arg("ns")
    tf = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]
    amcl = _flat(yaml.safe_load(PARAMS.read_text())["amcl"]["ros__parameters"])
    amcl.update({"use_sim_time": True, "set_initial_pose": True,
                 "initial_pose.x": float(arg("x")), "initial_pose.y": float(arg("y")),
                 "initial_pose.z": 0.0, "initial_pose.yaw": float(arg("yaw"))})
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server_fine",
             namespace=ns, remappings=tf, output="screen",
             parameters=[{"use_sim_time": True, "yaml_filename": arg("map_fine"),
                          "topic_name": "map_fine", "frame_id": "map"}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", namespace=ns,
             remappings=tf, output="screen", parameters=[amcl]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_replay", namespace=ns, output="screen",
             parameters=[{"use_sim_time": True, "autostart": True, "bond_timeout": 0.0,
                          "node_names": ["map_server_fine", "amcl"]}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="pode2"),
        DeclareLaunchArgument("map_fine", description="AMCL 지도 yaml"),
        DeclareLaunchArgument("x"),
        DeclareLaunchArgument("y"),
        DeclareLaunchArgument("yaw", description="라디안"),
        OpaqueFunction(function=_nodes),
    ])
