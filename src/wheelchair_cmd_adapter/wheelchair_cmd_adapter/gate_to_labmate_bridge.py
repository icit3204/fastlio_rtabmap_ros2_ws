from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .labmate_bridge_core import (
    LabmateBridgeConfig,
    SafeTwist,
    convert_safe_twist,
)


class GateToLabmateBridge(Node):
    def __init__(self) -> None:
        super().__init__("gate_to_labmate_bridge")
        self.declare_parameter("input_topic", "/vehicle_cmd_safe")
        self.declare_parameter("output_topic", "/wheelchair_control_command")
        self.declare_parameter("expected_frame", "base_footprint")
        self.declare_parameter("heartbeat_hz", 20.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("max_forward_velocity_mps", 0.25)
        self.declare_parameter("max_reverse_velocity_mps", 0.25)
        self.declare_parameter("max_angular_velocity_rps", 0.50)
        self.declare_parameter("minimum_radius_m", 1.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if input_topic != "/vehicle_cmd_safe" or output_topic != "/wheelchair_control_command":
            raise RuntimeError("E4 production bridge topics are fixed safety contracts")

        self._config = LabmateBridgeConfig(
            frame_id=str(self.get_parameter("expected_frame").value),
            input_timeout_sec=float(self.get_parameter("input_timeout_sec").value),
            max_forward_velocity_mps=float(
                self.get_parameter("max_forward_velocity_mps").value
            ),
            max_reverse_velocity_mps=float(
                self.get_parameter("max_reverse_velocity_mps").value
            ),
            max_angular_velocity_rps=float(
                self.get_parameter("max_angular_velocity_rps").value
            ),
            minimum_radius_m=float(self.get_parameter("minimum_radius_m").value),
        )
        self._last_command: SafeTwist | None = None
        self._last_receipt_monotonic: float | None = None
        self._output = self.create_publisher(Float32MultiArray, output_topic, 10)
        self._diagnostics = self.create_publisher(
            DiagnosticArray, "/gate_to_labmate_bridge/diagnostics", 10
        )
        self._input = self.create_subscription(
            TwistStamped, input_topic, self._command_callback, 10
        )
        frequency = float(self.get_parameter("heartbeat_hz").value)
        self._timer = self.create_timer(1.0 / frequency, self._heartbeat)

    def _command_callback(self, message: TwistStamped) -> None:
        self._last_command = SafeTwist(
            frame_id=message.header.frame_id,
            stamp_sec=float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9,
            linear_x=message.twist.linear.x,
            linear_y=message.twist.linear.y,
            linear_z=message.twist.linear.z,
            angular_x=message.twist.angular.x,
            angular_y=message.twist.angular.y,
            angular_z=message.twist.angular.z,
        )
        self._last_receipt_monotonic = time.monotonic()

    def _heartbeat(self) -> None:
        monotonic_now = time.monotonic()
        receipt_age = (
            None
            if self._last_receipt_monotonic is None
            else monotonic_now - self._last_receipt_monotonic
        )
        result = convert_safe_twist(
            self._last_command,
            now_ros_sec=self.get_clock().now().nanoseconds * 1e-9,
            receipt_age_sec=receipt_age,
            config=self._config,
        )
        output = Float32MultiArray()
        output.data = list(result.output)
        self._output.publish(output)

        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "gate_to_labmate_bridge"
        status.hardware_id = "mkmini_development_chassis"
        status.level = DiagnosticStatus.OK if result.valid else DiagnosticStatus.WARN
        status.message = result.reason.value
        status.values = [
            KeyValue(key="input_topic", value="/vehicle_cmd_safe"),
            KeyValue(key="output_topic", value="/wheelchair_control_command"),
            KeyValue(key="fail_closed", value=str(not result.valid).lower()),
        ]
        diagnostic.status = [status]
        self._diagnostics.publish(diagnostic)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GateToLabmateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
