#!/usr/bin/env python3
"""움직이는 명령이 새로 오지 않으면 속도 0을 보낸다.

    cmd_watchdog.py /pode1/cmd_vel
"""
import sys
import time

WATCHDOG_S = 0.4


def overdue(moving_since, now: float) -> bool:
    """움직이는 명령이 WATCHDOG_S보다 오래 새로 오지 않았는가."""
    return moving_since is not None and now - moving_since > WATCHDOG_S


def main(argv) -> int:
    if len(argv) < 2 or not argv[1].startswith("/"):
        print(__doc__)
        return 2
    # ---- ROS는 여기서만 읽는다(시험은 ROS 없이 돈다) ----
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    class Watchdog(Node):
        def __init__(self, topic: str):
            super().__init__("cmd_watchdog")
            self.pub = self.create_publisher(Twist, topic, 10)
            self.moving_since = None
            self.create_subscription(Twist, topic, self._on_cmd, 10)
            self.create_timer(0.05, self._tick)
            self.get_logger().info(f"watching {topic}")

        def _on_cmd(self, msg) -> None:
            moving = msg.linear.x != 0.0 or msg.angular.z != 0.0
            self.moving_since = time.monotonic() if moving else None

        def _tick(self) -> None:
            if overdue(self.moving_since, time.monotonic()):
                self.pub.publish(Twist())
                self.get_logger().warn("command not renewed: zero velocity sent")

    rclpy.init()
    node = Watchdog(argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
