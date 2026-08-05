"""ROS wrapper for the Phase 4 generic guarded vehicle command gate."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from vehicle_cmd_safety.gate_core import AuthoritySnapshot, GateConfig, GateCore, Twist6


class GuardedVehicleCmdGate(Node):
    def __init__(self) -> None:
        super().__init__("guarded_vehicle_cmd_gate")
        self._declare_parameters()
        self._core = GateCore(self._config_from_parameters())

        self._output_pub = self.create_publisher(TwistStamped, "/vehicle_cmd_safe", 10)
        self._state_pub = self.create_publisher(DiagnosticStatus, "/vehicle_cmd_safety/state", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe_cb, 10)
        self.create_subscription(Bool, "/system/localization_valid", self._localization_cb, 10)
        self.create_subscription(Bool, "/system/controller_valid", self._controller_cb, 10)
        self.create_subscription(Bool, "/system/collision_monitor_valid", self._collision_cb, 10)
        self.create_service(SetBool, "/vehicle_cmd_safety/arm", self._arm_cb)

        self._heartbeat_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._graph_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._heartbeat_timer = self.create_timer(
            1.0 / self._core.config.heartbeat_hz,
            self._heartbeat_cb,
            clock=self._heartbeat_clock,
        )
        self._graph_timer = self.create_timer(0.5, self._graph_cb, clock=self._graph_clock)
        self._graph_cb()

    def _declare_parameters(self) -> None:
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("heartbeat_hz", 20.0)
        self.declare_parameter("safe_twist_timeout_sec", 0.25)
        self.declare_parameter("localization_timeout_sec", 0.50)
        self.declare_parameter("controller_timeout_sec", 0.50)
        self.declare_parameter("collision_valid_timeout_sec", 0.50)
        self.declare_parameter("authority_stability_sec", 1.0)
        self.declare_parameter("max_forward_velocity", 0.0)
        self.declare_parameter("max_angular_velocity", 0.0)
        self.declare_parameter("max_linear_increase_rate", 0.0)
        self.declare_parameter("max_angular_increase_rate", 0.0)
        self.declare_parameter("unsupported_axis_epsilon", 1e-6)
        self.declare_parameter("reverse_epsilon", 1e-6)
        self.declare_parameter("in_place_linear_epsilon", 0.01)
        self.declare_parameter("in_place_angular_epsilon", 0.02)
        self.declare_parameter("max_slew_dt_sec", 0.10)

    def _config_from_parameters(self) -> GateConfig:
        return GateConfig(
            frame_id=str(self.get_parameter("frame_id").value),
            heartbeat_hz=float(self.get_parameter("heartbeat_hz").value),
            safe_twist_timeout_sec=float(self.get_parameter("safe_twist_timeout_sec").value),
            localization_timeout_sec=float(self.get_parameter("localization_timeout_sec").value),
            controller_timeout_sec=float(self.get_parameter("controller_timeout_sec").value),
            collision_valid_timeout_sec=float(self.get_parameter("collision_valid_timeout_sec").value),
            authority_stability_sec=float(self.get_parameter("authority_stability_sec").value),
            max_forward_velocity=float(self.get_parameter("max_forward_velocity").value),
            max_angular_velocity=float(self.get_parameter("max_angular_velocity").value),
            max_linear_increase_rate=float(self.get_parameter("max_linear_increase_rate").value),
            max_angular_increase_rate=float(self.get_parameter("max_angular_increase_rate").value),
            unsupported_axis_epsilon=float(self.get_parameter("unsupported_axis_epsilon").value),
            reverse_epsilon=float(self.get_parameter("reverse_epsilon").value),
            in_place_linear_epsilon=float(self.get_parameter("in_place_linear_epsilon").value),
            in_place_angular_epsilon=float(self.get_parameter("in_place_angular_epsilon").value),
            max_slew_dt_sec=float(self.get_parameter("max_slew_dt_sec").value),
        )

    def _now_steady(self) -> float:
        return time.monotonic()

    def _safe_cb(self, msg: Twist) -> None:
        command = Twist6(
            linear_x=float(msg.linear.x),
            linear_y=float(msg.linear.y),
            linear_z=float(msg.linear.z),
            angular_x=float(msg.angular.x),
            angular_y=float(msg.angular.y),
            angular_z=float(msg.angular.z),
        )
        self._core.set_safe_command(command, self._now_steady())

    def _localization_cb(self, msg: Bool) -> None:
        self._core.set_permission("localization", bool(msg.data), self._now_steady())

    def _controller_cb(self, msg: Bool) -> None:
        self._core.set_permission("controller", bool(msg.data), self._now_steady())

    def _collision_cb(self, msg: Bool) -> None:
        self._core.set_permission("collision", bool(msg.data), self._now_steady())

    def _arm_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        success, reason = self._core.request_arm(bool(request.data), self._now_steady())
        response.success = success
        response.message = reason
        if not request.data:
            self._publish_zero_now()
        return response

    def _graph_cb(self) -> None:
        now = self._now_steady()
        authority = AuthoritySnapshot(
            safe_input_publishers=len(self.get_publishers_info_by_topic("/cmd_vel_nav_safe")),
            output_publishers=len(self.get_publishers_info_by_topic("/vehicle_cmd_safe")),
            localization_publishers=len(self.get_publishers_info_by_topic("/system/localization_valid")),
            controller_publishers=len(self.get_publishers_info_by_topic("/system/controller_valid")),
            collision_valid_publishers=len(self.get_publishers_info_by_topic("/system/collision_monitor_valid")),
        )
        self._core.set_authority(authority, now)

    def _heartbeat_cb(self) -> None:
        status = self._core.tick(self._now_steady())
        self._publish_output(status.output)
        self._publish_status(status)

    def _publish_zero_now(self) -> None:
        self._publish_output(Twist6())

    def _publish_output(self, command: Twist6) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._core.config.frame_id
        msg.twist.linear.x = command.linear_x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = command.angular_z
        self._output_pub.publish(msg)

    def _publish_status(self, status) -> None:
        diag = DiagnosticStatus()
        diag.name = "vehicle_cmd_safety/guarded_vehicle_cmd_gate"
        diag.hardware_id = "vehicle_cmd_safety"
        if status.state == "ARMED":
            diag.level = DiagnosticStatus.OK
        elif status.state == "FAULT":
            diag.level = DiagnosticStatus.ERROR
        else:
            diag.level = DiagnosticStatus.WARN
        diag.message = status.reason_code
        diag.values = [KeyValue(key=str(key), value=str(value)) for key, value in status.diagnostics.items()]
        self._state_pub.publish(diag)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diag]
        self._diag_pub.publish(array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GuardedVehicleCmdGate()
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
