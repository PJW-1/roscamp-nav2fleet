"""로봇마다 Nav2 goal용 controller를 고른다(행동 트리 ControllerSelector의 controller_selector 토픽).

사용: python3 select_controller.py DOMAIN NAME [로봇 수]
transient_local로 한 번 내고 끝날 때까지 떠 있다: 나중에 뜬 bt_navigator도 마지막 값을 받는다.
"""
import signal
import sys

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DOM, NAME = int(sys.argv[1]), sys.argv[2]
COUNT = int(sys.argv[3]) if len(sys.argv) > 3 else 3
QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
rclpy.init(domain_id=DOM)
node = rclpy.create_node("plan_flip_select_controller")
pubs = [node.create_publisher(String, f"/pode{i}/controller_selector", QOS)
        for i in range(1, COUNT + 1)]
for pub in pubs:
    pub.publish(String(data=NAME))
node.get_logger().info(f"controller_selector -> {NAME} for {COUNT} robots")
signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    rclpy.try_shutdown()
