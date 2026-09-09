#!/usr/bin/env python3
from __future__ import annotations

import json
import time

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class CanonicalChainRunner(Node):
    def __init__(self) -> None:
        super().__init__("mk2e5_canonical_chain_runner")
        self.declare_parameter("linear_x", 0.25)
        self.declare_parameter("angular_z", 0.0)
        self.declare_parameter("publishing_enabled", True)
        self._publisher = self.create_publisher(Twist, "/phase5/cm_test/cmd_vel_in", 10)
        self._latest = {}
        self._counts = {}
        self.create_subscription(Twist, "/phase5/cm_test/cmd_vel_out", self._cm, 10)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self._gate, 10)
        self.create_subscription(
            Float32MultiArray, "/wheelchair_control_command", self._bridge, 10
        )
        self.create_subscription(
            Float32MultiArray, "/phase5/mk2e4/mock_can_decoded", self._backend, 100
        )
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._valid, 10)
        self.create_subscription(
            DiagnosticStatus, "/phase5/gate_test/state", self._state, 10
        )
        self.create_timer(0.04, self._publish)
        self.create_timer(1.0, self._report)

    def _set(self, name, value) -> None:
        self._latest[name] = value
        self._counts[name] = self._counts.get(name, 0) + 1

    def _cm(self, message) -> None:
        self._set("cm", [message.linear.x, message.angular.z])

    def _gate(self, message) -> None:
        self._set(
            "gate",
            [message.twist.linear.x, message.twist.angular.z, message.header.frame_id],
        )

    def _bridge(self, message) -> None:
        self._set("bridge", list(message.data))

    def _backend(self, message) -> None:
        self._set("backend", list(message.data))

    def _valid(self, message) -> None:
        self._set("required_valid", bool(message.data))

    def _state(self, message) -> None:
        self._set("gate_state", message.message)

    def _publish(self) -> None:
        if not bool(self.get_parameter("publishing_enabled").value):
            return
        message = Twist()
        message.linear.x = float(self.get_parameter("linear_x").value)
        message.angular.z = float(self.get_parameter("angular_z").value)
        self._publisher.publish(message)
        self._counts["upstream"] = self._counts.get("upstream", 0) + 1

    def _report(self) -> None:
        report = {
            "monotonic_ns": time.monotonic_ns(),
            "command": {
                "enabled": bool(self.get_parameter("publishing_enabled").value),
                "linear_x": float(self.get_parameter("linear_x").value),
                "angular_z": float(self.get_parameter("angular_z").value),
            },
            "latest": self._latest,
            "counts": self._counts,
        }
        self.get_logger().info("MK2E5_SNAPSHOT " + json.dumps(report, sort_keys=True))


def main() -> None:
    rclpy.init()
    node = CanonicalChainRunner()
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
