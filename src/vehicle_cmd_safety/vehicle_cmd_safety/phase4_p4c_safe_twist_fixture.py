"""Deterministic /cmd_vel_nav_safe fixture for P4-C tests."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SafeTwistFixture(Node):
    def __init__(self) -> None:
        super().__init__("phase4_p4c_safe_twist_fixture")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("linear_x", 0.10)
        self.declare_parameter("angular_z", 0.0)
        self.declare_parameter("mode", "VALID")
        self._linear_x = float(self.get_parameter("linear_x").value)
        self._angular_z = float(self.get_parameter("angular_z").value)
        self._mode = str(self.get_parameter("mode").value).upper()
        self._pub = self.create_publisher(Twist, "/cmd_vel_nav_safe", 10)
        self.add_on_set_parameters_callback(self._set_parameters_cb)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self._timer_cb)

    def _set_parameters_cb(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "linear_x":
                self._linear_x = float(param.value)
            elif param.name == "angular_z":
                self._angular_z = float(param.value)
            elif param.name == "mode":
                self._mode = str(param.value).upper()
        return SetParametersResult(successful=True)

    def _timer_cb(self) -> None:
        if self._mode == "SILENT":
            return
        msg = Twist()
        msg.linear.x = self._linear_x
        msg.angular.z = self._angular_z
        if self._mode == "NAN":
            msg.linear.x = float("nan")
        elif self._mode == "INF":
            msg.linear.x = float("inf")
        elif self._mode == "UNSUPPORTED_AXIS":
            msg.linear.y = 0.1
        elif self._mode == "REVERSE":
            msg.linear.x = -0.1
        elif self._mode == "IN_PLACE":
            msg.linear.x = 0.0
            msg.angular.z = 0.2
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafeTwistFixture()
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
