"""recovery_probe 노드: RECOVERING 구간마다 AMCL 자세 오차를 Gazebo 실제 위치와 견준다.
  ros2 run fleet_gz_sim recovery_probe --ros-args -p use_sim_time:=true -p result_dir:=gz_result
<result_dir>/recovery_probe.csv에 쓴다(열은 recovery_probe.py). result_dir이 비면 로그만 낸다.
"""
from __future__ import annotations

import rclpy
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_interfaces.msg import RobotState
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from fleet_gz_sim.quaternion import yaw_of
from fleet_gz_sim.recovery_probe import COLUMNS, CsvSink, RecoveryProbe


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class RecoveryProbeNode(Node):
    def __init__(self, robots=None, result_dir=None, settle_s=None, **node_kwargs):
        """인자(fleet_check가 제 파라미터를 넘김)가 노드 파라미터보다 먼저다.
        fleet_check 안에서는 프로세스 인자를 무시한다(launch의 -r __node:= 때문에 이 노드 이름도 바뀐다).
        """
        hosted = {} if robots is None else {
            "use_global_arguments": False,
            "parameter_overrides": [Parameter("use_sim_time", value=True)]}
        super().__init__("recovery_probe", **node_kwargs, **hosted)
        if robots is None:
            robots = list(self.declare_parameter("robots", ["pode1", "pode2", "pode3"]).value)
            result_dir = self.declare_parameter("result_dir", "").value
            settle_s = float(self.declare_parameter("settle_s", 2.0).value)
        contract_file = self.declare_parameter("contract_file", "").value
        if not contract_file:
            from ament_index_python.packages import get_package_share_directory
            contract_file = get_package_share_directory("fleet_interfaces") + "/contract.yaml"
        contract = load_contract(contract_file)
        self.probe = RecoveryProbe(robots, settle_s)
        self.sink = CsvSink(f"{result_dir}/recovery_probe.csv" if result_dir else "")
        for robot in robots:
            self.create_subscription(Odometry, f"/{robot}/sim/truth",
                                     lambda msg, r=robot: self._on_truth(r, msg),
                                     qos_profile_sensor_data)
            self.create_subscription(RobotState, contract.name("robot_state", pode=robot),
                                     lambda msg, r=robot: self._on_state(r, msg),
                                     to_ros_qos(contract.qos("robot_state")))
        self.get_logger().info(f"recovery probe for {robots} -> {result_dir or 'log only'}")

    def _on_truth(self, robot: str, msg: Odometry) -> None:
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = yaw_of(q)
        self.probe.on_truth(robot, _seconds(msg.header.stamp), p.x, p.y, yaw)

    def _on_state(self, robot: str, msg: RobotState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        for row in self.probe.on_state(robot, now, msg.state, msg.pose.x, msg.pose.y,
                                       msg.pose.theta, _seconds(msg.stamp)):
            self.sink.write(row)
            self.get_logger().info(" ".join(f"{k}={_fmt(row[k])}" for k in COLUMNS
                                            if row[k] != ""))


def _fmt(value) -> str:
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryProbeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.sink.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
