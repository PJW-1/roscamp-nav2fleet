"""다른 핑키의 몸을 이 로봇의 costmap에 넣는 노드(임시, 주행 시험용. 회피 담당이 바꿀 수 있다).

라이다 빔은 다른 핑키의 몸 위로 지나가 costmap에 작은 점으로만 찍힌다. 그래서 Nav2가 다른 로봇을
피해 길을 짜지 못하고 마당에서 몸이 닿았다. 이 노드는 다른 로봇이 보고한 자세로 그 몸 자리를
점구름으로 내고, costmap의 peer 층(회피 패키지 avoidance.yaml의 peer_layer)이 그것을 찍는다.

- 받는 것: 다른 로봇마다 `/<로봇>/state`(RobotState, 10 Hz, map 좌표 자세).
- 내는 것(로봇 namespace 안, rate_hz마다):
  `peer_obstacles`(PointCloud2, map 틀): 다른 로봇마다 몸을 덮는 원 안의 점들.
  `peer_sweep`(LaserScan, base_footprint 틀): 모든 빔이 inf인 한 바퀴 스캔. peer 층은 inf_is_valid로
  이 빔을 따라 둘레를 다 지운 뒤 지금 자리만 다시 찍는다. 그래서 지나간 자리가 남지 않는다.
- 조용해진 로봇의 자리는 지우지 않는다: 통신이 끊긴 로봇은 그 자리에 서 있다(관제 heartbeat가
  끊기면 로봇은 선다는 계약에 기댄다).
- 위치를 모르는 보고(LOCALIZING이거나 자세가 0, 0, 0: pinky_fsm은 위치를 모르면 자세 칸을 비워 둔다)
  는 쓰지 않는다. 그 전에 알던 자리는 그대로 둔다.
- 점구름은 비지 않는다: Nav2의 점구름 반복자는 빈 data에서 front()를 부른다. 늘 먼 점(FAR_POINT)
  하나를 함께 낸다. costmap은 obstacle_max_range 밖이라 찍지 않는다.
- 자세는 그 로봇의 위치 추정이다. 추정이 틀어지면 찍는 자리도 틀린다.
- 파라미터: `robot_id`(비면 namespace), `robots`("pinky1,pinky2,pinky3"), `radius_m`(0.08: 몸 가운데
  에서 가장 먼 모서리 0.073 m에 보고 지연 몫을 더함), `centre_m`(-0.018: 바퀴 축에서 몸 가운데까지),
  `step_m`, `rate_hz`, `sweep_beams`, `sweep_range_m`.
- 실행 파일 `peer_obstacles`는 로봇 한 대의 노드다(로봇에서). 시뮬레이션은 make_fleet으로 로봇마다의
  노드를 sim_stand_in 프로세스 안에 둔다(한 PC의 DDS 참가자 수를 늘리지 않으려고).
"""
import math

import rclpy
from fleet_interfaces.msg import RobotState
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .body import body_points, others

STATE_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)  # 계약의 STATE
POINT_Z_M = 0.05  # costmap 관측원의 높이 창(0~2 m) 안
FAR_POINT = (1000.0, 1000.0, POINT_Z_M)  # 구름이 비지 않게 늘 함께 내는 점(찍히지 않는다)
# sweep의 시각을 이만큼 지난 때로 적는다: costmap은 스캔 시각의 TF(odom <- base_footprint)로 빔을
# 놓는데 지금 시각의 TF는 아직 오지 않았을 수 있다. 둘레를 지우는 원이라 0.2 s 전 자리여도 된다
SWEEP_DELAY = Duration(seconds=0.2)


class PeerObstacles(Node):
    def __init__(self, **kwargs):
        super().__init__("peer_obstacles", **kwargs)
        declare = self.declare_parameter
        self.robot_id = declare("robot_id", "").value or self.get_namespace().strip("/")
        self.radius_m = float(declare("radius_m", 0.08).value)
        self.centre_m = float(declare("centre_m", -0.018).value)
        self.step_m = float(declare("step_m", 0.02).value)
        self.map_frame = declare("map_frame", "map").value
        self.base_frame = declare("base_frame", "base_footprint").value
        beams = int(declare("sweep_beams", 1440).value)
        sweep_range_m = float(declare("sweep_range_m", 4.0).value)
        peers = others(declare("robots", "").value, self.robot_id)
        self.poses = {}  # 로봇 이름 -> 마지막으로 보고한 (x, y, yaw)
        for name in peers:
            self.create_subscription(RobotState, f"/{name}/state",
                                     lambda msg, name=name: self.on_state(name, msg), STATE_QOS)
        self.cloud_pub = self.create_publisher(PointCloud2, "peer_obstacles", 5)
        self.sweep_pub = self.create_publisher(LaserScan, "peer_sweep", 5)
        self.sweep = LaserScan()
        self.sweep.header.frame_id = self.base_frame
        self.sweep.angle_min = -math.pi
        self.sweep.angle_increment = 2.0 * math.pi / beams
        self.sweep.angle_max = math.pi - self.sweep.angle_increment
        self.sweep.range_min = 0.0
        self.sweep.range_max = sweep_range_m
        self.sweep.ranges = [math.inf] * beams
        self.create_timer(1.0 / float(declare("rate_hz", 5.0).value), self.publish)
        self.get_logger().info(f"{self.robot_id}: bodies of {peers}, radius {self.radius_m} m")

    def on_state(self, name: str, msg: RobotState):
        pose = (msg.pose.x, msg.pose.y, msg.pose.theta)
        known = msg.state != RobotState.STATE_LOCALIZING and pose != (0.0, 0.0, 0.0)
        if known and all(math.isfinite(value) for value in pose):
            self.poses[name] = pose

    def publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        points = [(x, y, POINT_Z_M) for pose in self.poses.values()
                  for x, y in body_points(*pose, self.radius_m, self.step_m, self.centre_m)]
        points.append(FAR_POINT)
        header = Header(stamp=stamp, frame_id=self.map_frame)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        if now.nanoseconds > SWEEP_DELAY.nanoseconds:  # 시뮬레이션 시계가 막 0에서 시작했을 때
            self.sweep.header.stamp = (now - SWEEP_DELAY).to_msg()
        else:
            self.sweep.header.stamp = stamp
        self.sweep_pub.publish(self.sweep)


def make_fleet(robots: str, use_sim_time: bool = False, context=None) -> list:
    """robots의 로봇마다 그 namespace 안의 PeerObstacles 노드 하나. 프로세스 인자(launch의 노드 이름
    바꾸기)는 받지 않는다: 받으면 노드들의 이름이 모두 같아진다."""
    return [PeerObstacles(namespace=f"/{name}", context=context, use_global_arguments=False,
                          parameter_overrides=[Parameter("robot_id", value=name),
                                               Parameter("robots", value=robots),
                                               Parameter("use_sim_time", value=use_sim_time)])
            for name in others(robots, "")]


def main():
    rclpy.init()
    node = PeerObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
