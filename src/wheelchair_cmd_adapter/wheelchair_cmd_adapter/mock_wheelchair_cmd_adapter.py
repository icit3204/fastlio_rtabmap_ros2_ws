from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray

from wheelchair_cmd_adapter.conversion_core import (
    AdapterConfig,
    AdapterState,
    CommandValues,
    ConversionResult,
    evaluate_command,
    validate_topic_contract,
)


class MockWheelchairCmdAdapter(Node):
    def __init__(self):
        super().__init__("mock_wheelchair_cmd_adapter")
        self._config = self._load_config()

        self._heartbeat_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._graph_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._receipt_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self._latest_values: CommandValues | None = None
        self._latest_receipt_ns: int | None = None
        self._input_publishers = 0
        self._output_publishers = 1
        self._last_result: ConversionResult | None = None

        self.create_subscription(TwistStamped, self._config.input_topic, self._input_cb, 10)
        self._output_pub = self.create_publisher(Float32MultiArray, self._config.output_topic, 10)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, "/wheelchair_cmd_adapter/diagnostics", 10)

        heartbeat_period = 1.0 / self._config.heartbeat_hz
        self._heartbeat_timer = self.create_timer(heartbeat_period, self._heartbeat_cb, clock=self._heartbeat_clock)
        self._graph_timer = self.create_timer(0.1, self._graph_cb, clock=self._graph_clock)

    def _load_config(self) -> AdapterConfig:
        self.declare_parameter("input_topic", "/vehicle_cmd_safe")
        self.declare_parameter("output_topic", "/wheelchair_control_command_mock")
        self.declare_parameter("expected_frame", "base_footprint")
        self.declare_parameter("heartbeat_hz", 20.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("max_forward_velocity", 0.20)
        self.declare_parameter("max_angular_velocity", 0.50)
        self.declare_parameter("unsupported_axis_epsilon", 1e-6)
        self.declare_parameter("reverse_epsilon", 1e-6)
        self.declare_parameter("in_place_linear_epsilon", 0.01)
        self.declare_parameter("in_place_angular_epsilon", 0.02)
        self.declare_parameter("minimum_turn_radius_m", 1.0)
        self.declare_parameter("straight_radius_m", 10.0)
        config = AdapterConfig(
            input_topic=str(self.get_parameter("input_topic").value),
            output_topic=str(self.get_parameter("output_topic").value),
            expected_frame=str(self.get_parameter("expected_frame").value),
            heartbeat_hz=float(self.get_parameter("heartbeat_hz").value),
            input_timeout_sec=float(self.get_parameter("input_timeout_sec").value),
            max_forward_velocity=float(self.get_parameter("max_forward_velocity").value),
            max_angular_velocity=float(self.get_parameter("max_angular_velocity").value),
            unsupported_axis_epsilon=float(self.get_parameter("unsupported_axis_epsilon").value),
            reverse_epsilon=float(self.get_parameter("reverse_epsilon").value),
            in_place_linear_epsilon=float(self.get_parameter("in_place_linear_epsilon").value),
            in_place_angular_epsilon=float(self.get_parameter("in_place_angular_epsilon").value),
            minimum_turn_radius_m=float(self.get_parameter("minimum_turn_radius_m").value),
            straight_radius_m=float(self.get_parameter("straight_radius_m").value),
        )
        validate_topic_contract(config.input_topic, config.output_topic)
        return config

    def _input_cb(self, msg: TwistStamped) -> None:
        self._latest_values = CommandValues(
            frame_id=msg.header.frame_id,
            linear_x=float(msg.twist.linear.x),
            linear_y=float(msg.twist.linear.y),
            linear_z=float(msg.twist.linear.z),
            angular_x=float(msg.twist.angular.x),
            angular_y=float(msg.twist.angular.y),
            angular_z=float(msg.twist.angular.z),
        )
        self._latest_receipt_ns = self._receipt_clock.now().nanoseconds

    def _graph_cb(self) -> None:
        self._input_publishers = len(self.get_publishers_info_by_topic(self._config.input_topic))
        self._output_publishers = len(self.get_publishers_info_by_topic(self._config.output_topic))

    def _heartbeat_cb(self) -> None:
        result = self._evaluate()
        self._last_result = result
        out = Float32MultiArray()
        out.data = list(result.output)
        self._output_pub.publish(out)
        self._publish_diagnostics(result)

    def _evaluate(self) -> ConversionResult:
        age = None
        if self._latest_receipt_ns is not None:
            age = (self._receipt_clock.now().nanoseconds - self._latest_receipt_ns) / 1e9
        return evaluate_command(
            self._latest_values,
            AdapterState(
                has_input=self._latest_values is not None,
                input_age_sec=age,
                input_publisher_count=self._input_publishers,
                output_publisher_count=self._output_publishers,
            ),
            self._config,
        )

    def _publish_diagnostics(self, result: ConversionResult) -> None:
        values = self._latest_values or CommandValues()
        details = result.details
        status = DiagnosticStatus()
        status.name = "mock_wheelchair_cmd_adapter"
        status.hardware_id = "mock_only"
        status.level = DiagnosticStatus.OK if result.valid else DiagnosticStatus.WARN
        status.message = result.reason.value
        fields = {
            "condition": result.reason.value,
            "input_age_sec": details.get("input_age_sec"),
            "input_publisher_count": self._input_publishers,
            "output_publisher_count": self._output_publishers,
            "expected_frame": self._config.expected_frame,
            "observed_frame": values.frame_id,
            "linear_x": values.linear_x,
            "angular_z": values.angular_z,
            "computed_radius_m": details.get("computed_radius_m"),
            "output_array": list(result.output),
            "heartbeat_hz": self._config.heartbeat_hz,
            "input_timeout_sec": self._config.input_timeout_sec,
            "steady_clock": True,
        }
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in fields.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diagnostic_pub.publish(array)


def main() -> None:
    rclpy.init()
    node = MockWheelchairCmdAdapter()
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
