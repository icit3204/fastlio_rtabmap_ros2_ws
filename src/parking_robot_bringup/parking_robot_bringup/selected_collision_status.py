"""Mode-neutral observability for the selected R23 collision-filter output."""

from __future__ import annotations

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


VALID_MODES = ("fixed_qualified", "motion_aware_experimental")


def selected_validity(state):
    """A selected filter is healthy in risk states; STALE is fail-closed."""
    return state in {"CLEAR", "SLOW", "STOP"}


def classify_selected(input_twist, output_twist, *, input_age, output_age,
                      timeout=0.25, epsilon=1.0e-6):
    if (input_twist is None or output_twist is None or
            input_age > timeout or output_age > timeout):
        return "STALE"
    values = (
        input_twist.linear.x, input_twist.angular.z,
        output_twist.linear.x, output_twist.angular.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return "STALE"
    input_norm = abs(input_twist.linear.x) + abs(input_twist.angular.z)
    output_norm = abs(output_twist.linear.x) + abs(output_twist.angular.z)
    if input_norm <= epsilon:
        return "CLEAR" if output_norm <= epsilon else "STALE"
    if output_norm <= epsilon:
        return "STOP"
    ratios = []
    for requested, filtered in (
            (input_twist.linear.x, output_twist.linear.x),
            (input_twist.angular.z, output_twist.angular.z)):
        if abs(requested) > epsilon:
            ratios.append(abs(filtered / requested))
    return "SLOW" if ratios and min(ratios) < 0.99 else "CLEAR"


class SelectedCollisionStatus(Node):
    def __init__(self):
        super().__init__("selected_collision_status")
        self.declare_parameter("active_mode", "fixed_qualified")
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("selected_output_topic", "/cmd_vel_collision_selected")
        self.declare_parameter("experimental_state_topic", "/motion_aware_collision_mock/state")
        self.declare_parameter("status_topic", "/collision_filter_selected/state")
        self.declare_parameter("publish_validity", False)
        self.declare_parameter("validity_topic", "/system/collision_monitor_valid")
        self.declare_parameter("timeout_sec", 0.25)
        self.mode = str(self.get_parameter("active_mode").value)
        if self.mode not in VALID_MODES:
            raise RuntimeError(f"invalid collision mode {self.mode!r}")
        self.timeout = float(self.get_parameter("timeout_sec").value)
        self.input_twist = self.output_twist = None
        self.input_time = self.output_time = None
        self.experimental_state = None
        self.experimental_time = None
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._input, 10)
        self.create_subscription(
            Twist, str(self.get_parameter("selected_output_topic").value), self._output, 10)
        self.create_subscription(
            String, str(self.get_parameter("experimental_state_topic").value),
            self._experimental, 10)
        self.publisher = self.create_publisher(
            DiagnosticStatus, str(self.get_parameter("status_topic").value), 10)
        self.validity_publisher = None
        if bool(self.get_parameter("publish_validity").value):
            if self.mode != "motion_aware_experimental":
                raise RuntimeError("selected validity is exclusive to experimental mode")
            self.validity_publisher = self.create_publisher(
                Bool, str(self.get_parameter("validity_topic").value), 10)
        self.get_logger().warning(f"ACTIVE_COLLISION_MODE={self.mode}")
        self.create_timer(0.05, self._publish)

    def _input(self, message):
        self.input_twist = message
        self.input_time = time.monotonic()

    def _output(self, message):
        self.output_twist = message
        self.output_time = time.monotonic()

    def _experimental(self, message):
        try:
            self.experimental_state = json.loads(message.data)
            self.experimental_time = time.monotonic()
        except (TypeError, ValueError):
            self.experimental_state = {"state": "STALE", "fallback": True}
            self.experimental_time = time.monotonic()

    def _publish(self):
        now = time.monotonic()
        input_age = math.inf if self.input_time is None else now - self.input_time
        output_age = math.inf if self.output_time is None else now - self.output_time
        state = classify_selected(
            self.input_twist, self.output_twist,
            input_age=input_age, output_age=output_age, timeout=self.timeout)
        if self.mode == "motion_aware_experimental":
            experimental_age = (
                math.inf if self.experimental_time is None else now - self.experimental_time)
            if state == "STALE":
                # Never let a diagnostic message override a missing/stale
                # selected Twist at the actual Generic Gate boundary.
                pass
            elif experimental_age > self.timeout or not self.experimental_state:
                state = "STALE"
            elif self.experimental_state.get("fallback", False):
                state = "STALE"
            else:
                state = str(self.experimental_state.get("state", "STALE"))
        status = DiagnosticStatus()
        status.name = "collision_filter_selected"
        status.hardware_id = "r23_r5_mock_integration"
        status.level = (DiagnosticStatus.ERROR if state in {"STOP", "STALE"}
                        else DiagnosticStatus.WARN if state == "SLOW"
                        else DiagnosticStatus.OK)
        status.message = state
        status.values = [
            KeyValue(key="active_collision_mode", value=self.mode),
            KeyValue(key="collision_state", value=state),
            KeyValue(key="input_age_sec", value=str(input_age)),
            KeyValue(key="selected_output_age_sec", value=str(output_age)),
        ]
        self.publisher.publish(status)
        if self.validity_publisher is not None:
            validity = Bool()
            validity.data = selected_validity(state)
            self.validity_publisher.publish(validity)


def main(args=None):
    rclpy.init(args=args)
    node = SelectedCollisionStatus()
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
