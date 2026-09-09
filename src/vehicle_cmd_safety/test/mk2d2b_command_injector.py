#!/usr/bin/env python3
"""Independent, qualification-only CM command injector for P5A-MK2D2B.

This is intentionally not installed or referenced by a production launch.  It
is run in its own OS process so stopping a sensor publisher cannot affect the
continuous isolated command stream being measured.
"""

from __future__ import annotations

import argparse
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Injector(Node):
    def __init__(self, topic: str, rate_hz: float, linear_x: float) -> None:
        super().__init__("mk2d2b_independent_command_injector")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, topic, qos)
        self._message = Twist()
        self._message.linear.x = linear_x
        self._count = 0
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(1.0, self._report)

    def _tick(self) -> None:
        self._publisher.publish(self._message)
        self._count += 1

    def _report(self) -> None:
        self.get_logger().info(
            f"published={self._count} subscriptions={self._publisher.get_subscription_count()} "
            f"monotonic_ns={time.monotonic_ns()}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/phase5/cm_test/cmd_vel_in")
    parser.add_argument("--rate-hz", type=float, default=25.0)
    parser.add_argument("--linear-x", type=float, default=0.250)
    args = parser.parse_args()
    if args.rate_hz < 20.0:
        raise SystemExit("P5A-MK2D2B requires an injector rate of at least 20 Hz")
    rclpy.init()
    node = Injector(args.topic, args.rate_hz, args.linear_x)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
