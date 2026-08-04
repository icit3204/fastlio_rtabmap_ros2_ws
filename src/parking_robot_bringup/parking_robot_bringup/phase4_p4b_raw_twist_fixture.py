"""Deterministic raw Twist publisher for Phase 4 P4-B Collision Monitor preflight."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Phase4P4BRawTwistFixture(Node):
    """Publish one deterministic nonzero raw command stream."""

    def __init__(self) -> None:
        super().__init__("phase4_p4b_raw_twist_fixture")
        self.declare_parameter("enabled", True)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("linear_x", 0.20)
        self.declare_parameter("angular_z", 0.20)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz < 20.0:
            raise ValueError("publish_rate_hz must be at least 20.0")
        self._enabled = bool(self.get_parameter("enabled").value)
        self._linear_x = float(self.get_parameter("linear_x").value)
        self._angular_z = float(self.get_parameter("angular_z").value)

        self._pub = self.create_publisher(Twist, "/cmd_vel_nav_raw", 10)
        self.add_on_set_parameters_callback(self._set_parameters_cb)
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_cb)

    def _set_parameters_cb(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "enabled":
                self._enabled = bool(param.value)
            elif param.name == "linear_x":
                self._linear_x = float(param.value)
            elif param.name == "angular_z":
                self._angular_z = float(param.value)
        return SetParametersResult(successful=True)

    def _timer_cb(self) -> None:
        if not self._enabled:
            return
        msg = Twist()
        msg.linear.x = self._linear_x
        msg.angular.z = self._angular_z
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Phase4P4BRawTwistFixture()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
