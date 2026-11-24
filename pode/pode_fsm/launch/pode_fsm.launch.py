"""Pode 한 대의 FSM을 띄운다. namespace가 robot_id가 된다.

  ros2 launch pode_fsm pode_fsm.launch.py namespace:=pode1

TF는 Nav2와 같이 namespace의 tf, tf_static을 쓴다.

동작 없음(설계 4.5): no_motion:=true면 Nav2 없이 혼자 돈다. 목표는 정해진 시간 뒤 성공하고
자세는 목표로 옮긴다. 시작 자세는 initial_x/y/yaw.

  ros2 launch pode_fsm pode_fsm.launch.py namespace:=pode1 no_motion:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pode_fsm.core import FsmParams


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pode1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("heartbeat_timeout_s", default_value="1.5",
                              description="0이면 heartbeat가 끊겨도 서지 않는다"
                                          "(관제 없이 시험할 때)"),
        DeclareLaunchArgument("comm_loss_policy", default_value=FsmParams().comm_loss_policy,
                              description="order 모델에서 heartbeat가 끊기면: stop(바로 선다)"
                                          " 또는 base_end(받은 base 끝까지 가서 선다)"),
        DeclareLaunchArgument("through_controller_id", default_value="FollowPathThrough",
                              description="잠금 경계에서 끝나는 경로에 쓸 Nav2 controller"
                                          "(pode_nav_config가 함께 띄운다)."
                                          " 비우면 모두 controller_id(접근 감속 그대로)"),
        DeclareLaunchArgument("horizon_cap_mps", default_value=str(FsmParams().horizon_cap_mps),
                              description="잠금 경계 속도 제한의 상한(m/s). RPP는 그 제한을"
                                          " 속도로 쓰므로 Nav2 desired_linear_vel 이하로"),
        DeclareLaunchArgument("battery_topic", default_value="",
                              description="BatteryState 토픽(percentage 0~1). 비면 배터리를"
                                          " 모름(-1)으로 알린다. 시뮬은 시뮬 배터리를 준다"),
        # TODO(no-motion): 실주행으로 돌아가면 no_motion은 기본(false) 그대로 둔다
        DeclareLaunchArgument("no_motion", default_value="false",
                              description="true면 주행하지 않고 정해진 시간 뒤 목표를 성공시키고"
                                          " 자세를 목표로 옮긴다(설계 4.5)"),
        DeclareLaunchArgument("initial_x", default_value="0.12",
                              description="no_motion일 때 시작 자세 x(m)"),
        DeclareLaunchArgument("initial_y", default_value="0.10",
                              description="no_motion일 때 시작 자세 y(m)"),
        DeclareLaunchArgument("initial_yaw", default_value="1.5708",
                              description="no_motion일 때 시작 방향(rad)"),
        Node(
            package="pode_fsm", executable="pode_fsm_node", name="pode_fsm",
            namespace=namespace, output="screen",
            parameters=[{
                "robot_id": namespace,
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"),
                                               value_type=bool),
                "heartbeat_timeout_s": ParameterValue(
                    LaunchConfiguration("heartbeat_timeout_s"), value_type=float),
                "comm_loss_policy": ParameterValue(
                    LaunchConfiguration("comm_loss_policy"), value_type=str),
                "through_controller_id": ParameterValue(
                    LaunchConfiguration("through_controller_id"), value_type=str),
                "horizon_cap_mps": ParameterValue(
                    LaunchConfiguration("horizon_cap_mps"), value_type=float),
                "battery_topic": ParameterValue(
                    LaunchConfiguration("battery_topic"), value_type=str),
                "no_motion": ParameterValue(LaunchConfiguration("no_motion"), value_type=bool),
                "initial_x": ParameterValue(LaunchConfiguration("initial_x"), value_type=float),
                "initial_y": ParameterValue(LaunchConfiguration("initial_y"), value_type=float),
                "initial_yaw": ParameterValue(LaunchConfiguration("initial_yaw"),
                                              value_type=float),
            }],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]),
    ])
