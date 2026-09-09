"""Fail-closed AND authority for the two required obstacle sensors."""

from __future__ import annotations

import time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class RequiredPerceptionValidity(Node):
    def __init__(self) -> None:
        super().__init__("required_perception_validity")
        self.declare_parameter("mid360_topic", "/phase5/perception/mid360_valid")
        self.declare_parameter("tmini_topic", "/phase5/perception/tmini_valid")
        self.declare_parameter("output_topic", "/system/collision_monitor_valid")
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("recovery_consecutive_ticks", 3)
        self.declare_parameter("heartbeat_hz", 20.0)
        self._timeout = float(self.get_parameter("input_timeout_sec").value)
        self._needed = int(self.get_parameter("recovery_consecutive_ticks").value)
        self._samples = {"mid360": (False, None), "tmini": (False, None)}
        self._healthy_ticks = 0
        self._pub = self.create_publisher(Bool, str(self.get_parameter("output_topic").value), 10)
        self._diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Bool, str(self.get_parameter("mid360_topic").value), lambda m: self._set("mid360", m), 10)
        self.create_subscription(Bool, str(self.get_parameter("tmini_topic").value), lambda m: self._set("tmini", m), 10)
        self.create_timer(1.0 / float(self.get_parameter("heartbeat_hz").value), self._tick)

    def _set(self, name: str, msg: Bool) -> None:
        self._samples[name] = (bool(msg.data), time.monotonic())

    def _tick(self) -> None:
        now = time.monotonic()
        healthy = all(value and stamp is not None and now - stamp <= self._timeout for value, stamp in self._samples.values())
        self._healthy_ticks = self._healthy_ticks + 1 if healthy else 0
        valid = healthy and self._healthy_ticks >= self._needed
        self._pub.publish(Bool(data=valid))
        status = DiagnosticStatus(name="vehicle_cmd_safety/required_perception_validity", hardware_id="dual_obstacle_sources")
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR
        status.message = "VALID" if valid else "REQUIRED_SOURCE_INVALID"
        status.values = [KeyValue(key="mid360_valid", value=str(self._samples["mid360"][0]).lower()), KeyValue(key="tmini_valid", value=str(self._samples["tmini"][0]).lower()), KeyValue(key="recovery_ticks", value=str(self._healthy_ticks))]
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = [status]
        self._diag.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args); node = RequiredPerceptionValidity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
