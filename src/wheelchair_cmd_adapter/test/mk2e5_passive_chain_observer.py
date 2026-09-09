#!/usr/bin/env python3
"""Passive MK2E5 observer: subscribes only and never creates a publisher."""

from __future__ import annotations

import json
import time

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class PassiveChainObserver(Node):
    def __init__(self) -> None:
        super().__init__("mk2e5_passive_chain_observer")
        self.latest = {}
        self.counts = {}
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
        self.create_timer(1.0, self._report)

    def _set(self, name, value) -> None:
        self.latest[name] = value
        self.counts[name] = self.counts.get(name, 0) + 1

    def _cm(self, msg: Twist) -> None:
        self._set("cm", [msg.linear.x, msg.angular.z])

    def _gate(self, msg: TwistStamped) -> None:
        self._set("gate", [msg.twist.linear.x, msg.twist.angular.z, msg.header.frame_id])

    def _bridge(self, msg: Float32MultiArray) -> None:
        self._set("bridge", list(msg.data))

    def _backend(self, msg: Float32MultiArray) -> None:
        self._set("backend", list(msg.data))

    def _valid(self, msg: Bool) -> None:
        self._set("required_valid", bool(msg.data))

    def _state(self, msg: DiagnosticStatus) -> None:
        self._set("gate_state", msg.message)

    def _report(self) -> None:
        self.get_logger().info(
            "MK2E5_PASSIVE_SNAPSHOT "
            + json.dumps(
                {"monotonic_ns": time.monotonic_ns(), "latest": self.latest, "counts": self.counts},
                sort_keys=True,
            )
        )


def main() -> None:
    rclpy.init()
    node = PassiveChainObserver()
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
