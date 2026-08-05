"""Deterministic Bool permission fixture for P4-C tests."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


def enabled_permission_topics(
    *,
    publish_localization: bool = True,
    publish_controller: bool = True,
    publish_collision: bool = True,
) -> tuple[str, ...]:
    topics = []
    if publish_localization:
        topics.append("/system/localization_valid")
    if publish_controller:
        topics.append("/system/controller_valid")
    if publish_collision:
        topics.append("/system/collision_monitor_valid")
    return tuple(topics)


class PermissionFixture(Node):
    def __init__(self) -> None:
        super().__init__("phase4_p4c_permission_fixture")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("localization_valid", True)
        self.declare_parameter("controller_valid", True)
        self.declare_parameter("collision_valid", True)
        self.declare_parameter("publish_localization", True)
        self.declare_parameter("publish_controller", True)
        self.declare_parameter("publish_collision", True)
        self._localization_valid = bool(self.get_parameter("localization_valid").value)
        self._controller_valid = bool(self.get_parameter("controller_valid").value)
        self._collision_valid = bool(self.get_parameter("collision_valid").value)
        self._publish_localization = bool(self.get_parameter("publish_localization").value)
        self._publish_controller = bool(self.get_parameter("publish_controller").value)
        self._publish_collision = bool(self.get_parameter("publish_collision").value)
        self._localization_pub = (
            self.create_publisher(Bool, "/system/localization_valid", 10)
            if self._publish_localization
            else None
        )
        self._controller_pub = (
            self.create_publisher(Bool, "/system/controller_valid", 10)
            if self._publish_controller
            else None
        )
        self._collision_pub = (
            self.create_publisher(Bool, "/system/collision_monitor_valid", 10)
            if self._publish_collision
            else None
        )
        self.add_on_set_parameters_callback(self._set_parameters_cb)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self._timer_cb)

    def _set_parameters_cb(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "localization_valid":
                self._localization_valid = bool(param.value)
            elif param.name == "controller_valid":
                self._controller_valid = bool(param.value)
            elif param.name == "collision_valid":
                self._collision_valid = bool(param.value)
            elif param.name == "publish_localization":
                self._publish_localization = bool(param.value)
            elif param.name == "publish_controller":
                self._publish_controller = bool(param.value)
            elif param.name == "publish_collision":
                self._publish_collision = bool(param.value)
        return SetParametersResult(successful=True)

    def _timer_cb(self) -> None:
        if self._publish_localization:
            msg = Bool()
            msg.data = self._localization_valid
            if self._localization_pub is not None:
                self._localization_pub.publish(msg)
        if self._publish_controller:
            msg = Bool()
            msg.data = self._controller_valid
            if self._controller_pub is not None:
                self._controller_pub.publish(msg)
        if self._publish_collision:
            msg = Bool()
            msg.data = self._collision_valid
            if self._collision_pub is not None:
                self._collision_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PermissionFixture()
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
