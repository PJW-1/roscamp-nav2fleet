"""관제 화면 노드(시뮬레이션 전용): 함대 전체를 RViz 하나에 마커로 그린다.
함대 상태, 구역, 로봇 상태, 계획 경로, 실제 위치를 받아 /fleet_view/markers에 5 Hz로 낸다.
네임스페이스 없는 RViz에도 map 프레임이 있도록 전역 /tf_static에 map -> fleet_view를 낸다.
"""
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from fleet_core.common.contract import load_contract, to_ros_qos
from fleet_core.traffic.site import load_site
from fleet_interfaces.msg import RobotState, RobotStatus, RobotStatusArray, ZoneStateArray
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from fleet_gz_sim.fleet_view import FleetView
from fleet_gz_sim.quaternion import yaw_of

FRAME = "map"
PERIOD_S = 0.2


def to_marker(spec, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.header.stamp = stamp
    m.ns, m.id = spec.ns, spec.id
    m.type = Marker.TEXT_VIEW_FACING if spec.kind == "TEXT" else getattr(Marker, spec.kind)
    m.action = Marker.ADD
    x, y, z, yaw = spec.pose
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in spec.scale)
    m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in spec.color)
    m.points = [Point(x=float(px), y=float(py)) for px, py in spec.points]
    m.text = spec.text
    return m


class FleetViewNode(Node):
    def __init__(self):
        super().__init__("fleet_view")
        self.declare_parameter("robots", ["pode1", "pode2", "pode3"])
        self.declare_parameter("map_file", "")  # 배치 이름이나 파일, 비면 기본(v7)
        self.declare_parameter("pose_source", "truth")
        self.declare_parameter("contract_file", "")
        robots = list(self.get_parameter("robots").value)
        self.pose_source = self.get_parameter("pose_source").value
        if self.pose_source not in ("truth", "fleet"):
            raise ValueError(f"pose_source must be truth or fleet, not {self.pose_source}")
        contract = load_contract(self.get_parameter("contract_file").value or (
            get_package_share_directory("fleet_interfaces") + "/contract.yaml"))
        self.view = FleetView(load_site(self.get_parameter("map_file").value or None))

        def sub(msg_type, key, callback):
            self.create_subscription(msg_type, contract.name(key), callback,
                                     to_ros_qos(contract.qos(key)))

        sub(RobotStatusArray, "robots", self._on_robots)
        sub(ZoneStateArray, "zones", lambda msg: self.view.set_zones(msg.zones))
        for robot in robots:  # 각 pode가 지금 가는 곳(current_goal)
            self.create_subscription(
                RobotState, contract.name("robot_state", pode=robot),
                lambda msg, robot=robot: self.view.set_goal(robot, msg.current_goal),
                to_ros_qos(contract.qos("robot_state")))
            self.create_subscription(
                Path, f"/{robot}/plan",
                lambda msg, robot=robot: self._on_plan(robot, msg), 5)
        if self.pose_source == "truth":
            for robot in robots:
                self.create_subscription(
                    Odometry, f"/{robot}/sim/truth",
                    lambda msg, robot=robot: self._on_truth(robot, msg),
                    qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, "/fleet_view/markers", 5)
        self.tf_static = StaticTransformBroadcaster(self)
        frame = TransformStamped()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.header.frame_id = FRAME
        frame.child_frame_id = "fleet_view"
        frame.transform.rotation.w = 1.0
        self.tf_static.sendTransform(frame)
        self.create_timer(PERIOD_S, self._draw)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_robots(self, msg):
        # 팔(kind ARM)은 위치가 없어 그리지 않는다
        podes = [r for r in msg.robots if r.kind != RobotStatus.KIND_ARM]
        self.view.set_statuses(podes)
        if self.pose_source == "fleet":
            now = self._now()
            for r in podes:
                self.view.set_pose(r.robot_id, r.x, r.y, r.yaw, now)

    def _on_plan(self, robot, msg):
        self.view.set_plan(robot, [(p.pose.position.x, p.pose.position.y) for p in msg.poses],
                           self._now())

    def _on_truth(self, robot, msg):
        p = msg.pose.pose
        self.view.set_pose(robot, p.position.x, p.position.y, yaw_of(p.orientation), self._now())

    def _draw(self):
        stamp = self.get_clock().now().to_msg()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = FRAME
        out = MarkerArray(markers=[clear])
        out.markers += [to_marker(spec, stamp) for spec in self.view.specs(self._now())]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FleetViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
