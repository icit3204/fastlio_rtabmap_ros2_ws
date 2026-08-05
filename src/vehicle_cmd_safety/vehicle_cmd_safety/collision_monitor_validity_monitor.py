"""ROS wrapper for the Phase 4 Collision Monitor validity permission."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool

from vehicle_cmd_safety.validity_core import (
    CollisionMonitorState,
    CollisionMonitorValidityCore,
    ValidityConfig,
)


class CollisionMonitorValidityMonitor(Node):
    def __init__(self) -> None:
        super().__init__("collision_monitor_validity_monitor")
        self._declare_parameters()
        self._core = CollisionMonitorValidityCore(self._config_from_parameters())

        self._valid_pub = self.create_publisher(Bool, "/system/collision_monitor_valid", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._state_client = self.create_client(
            GetState,
            f"/{self._core.config.collision_monitor_node_name}/get_state",
        )
        self._params_client = self.create_client(
            GetParameters,
            f"/{self._core.config.collision_monitor_node_name}/get_parameters",
        )
        self._cached_state = CollisionMonitorState()
        self._state_future = None
        self._params_future = None

        if self._core.config.source_type == "scan":
            self.create_subscription(LaserScan, self._core.config.source_topic, self._scan_cb, 10)
        elif self._core.config.source_type == "pointcloud":
            self.create_subscription(PointCloud2, self._core.config.source_topic, self._cloud_cb, 10)
        else:
            raise ValueError("source_type must be scan or pointcloud")

        self._timer = self.create_timer(1.0 / self._core.config.heartbeat_hz, self._timer_cb)

    def _declare_parameters(self) -> None:
        self.declare_parameter("source_type", "scan")
        self.declare_parameter("source_topic", "/phase4/synthetic_scan")
        self.declare_parameter("expected_frame", "base_footprint")
        self.declare_parameter("source_freshness_sec", 0.50)
        self.declare_parameter("recovery_stability_sec", 0.50)
        self.declare_parameter("heartbeat_hz", 20.0)
        self.declare_parameter("collision_monitor_node_name", "collision_monitor")
        self.declare_parameter("expected_observation_source_name", "scan")
        self.declare_parameter("expected_observation_source_type", "scan")
        self.declare_parameter("expected_observation_source_topic", "/phase4/synthetic_scan")

    def _config_from_parameters(self) -> ValidityConfig:
        return ValidityConfig(
            source_type=str(self.get_parameter("source_type").value),
            source_topic=str(self.get_parameter("source_topic").value),
            expected_frame=str(self.get_parameter("expected_frame").value),
            source_freshness_sec=float(self.get_parameter("source_freshness_sec").value),
            recovery_stability_sec=float(self.get_parameter("recovery_stability_sec").value),
            heartbeat_hz=float(self.get_parameter("heartbeat_hz").value),
            collision_monitor_node_name=str(self.get_parameter("collision_monitor_node_name").value),
            expected_observation_source_name=str(self.get_parameter("expected_observation_source_name").value),
            expected_observation_source_type=str(self.get_parameter("expected_observation_source_type").value),
            expected_observation_source_topic=str(self.get_parameter("expected_observation_source_topic").value),
        )

    def _now_steady(self) -> float:
        return time.monotonic()

    def _scan_cb(self, msg: LaserScan) -> None:
        valid = (
            msg.angle_increment > 0.0
            and msg.range_max > msg.range_min > 0.0
            and len(msg.ranges) > 0
        )
        self._core.set_observation(msg.header.frame_id, valid, self._now_steady())

    def _cloud_cb(self, msg: PointCloud2) -> None:
        valid = msg.point_step > 0 and msg.row_step >= msg.point_step * msg.width and len(msg.data) >= msg.row_step
        self._core.set_observation(msg.header.frame_id, valid, self._now_steady())

    def _timer_cb(self) -> None:
        try:
            self._update_collision_monitor_state()
        except Exception as exc:  # noqa: BLE001 - heartbeat must survive service-query failures.
            self.get_logger().error(f"collision monitor state update failed: {exc!r}")
            self._state_future = None
            self._params_future = None
        self._core.set_valid_publisher_count(len(self.get_publishers_info_by_topic("/system/collision_monitor_valid")))
        status = self._core.tick(self._now_steady())
        msg = Bool()
        msg.data = bool(status.valid)
        self._valid_pub.publish(msg)
        self._publish_diag(status)

    def _update_collision_monitor_state(self) -> None:
        state = CollisionMonitorState(
            lifecycle_reachable=self._cached_state.lifecycle_reachable,
            lifecycle_active=self._cached_state.lifecycle_active,
            config_reachable=self._cached_state.config_reachable,
            configured_source_present=self._cached_state.configured_source_present,
            configured_source_type_matches=self._cached_state.configured_source_type_matches,
            configured_source_topic_matches=self._cached_state.configured_source_topic_matches,
        )
        state.lifecycle_reachable = self._state_client.service_is_ready()
        state.config_reachable = self._params_client.service_is_ready()
        if state.lifecycle_reachable and self._state_future is None:
            self._state_future = self._state_client.call_async(GetState.Request())
        if state.config_reachable and self._params_future is None:
            req = GetParameters.Request()
            src = self._core.config.expected_observation_source_name
            req.names = ["observation_sources", f"{src}.type", f"{src}.topic"]
            self._params_future = self._params_client.call_async(req)
        if self._state_future is not None and self._state_future.done():
            result = self._state_future.result()
            if result is not None:
                state.lifecycle_active = result.current_state.label.lower() == "active"
            self._state_future = None
        if self._params_future is not None and self._params_future.done():
            result = self._params_future.result()
            if result is not None:
                values = result.values
                if len(values) >= 3:
                    sources = list(values[0].string_array_value)
                    src = self._core.config.expected_observation_source_name
                    state.configured_source_present = src in sources
                    state.configured_source_type_matches = values[1].string_value == self._core.config.expected_observation_source_type
                    state.configured_source_topic_matches = values[2].string_value == self._core.config.expected_observation_source_topic
            self._params_future = None
        self._cached_state = state
        self._core.set_collision_monitor_state(state)

    def _publish_diag(self, status) -> None:
        diag = DiagnosticStatus()
        diag.name = "vehicle_cmd_safety/collision_monitor_validity_monitor"
        diag.hardware_id = "vehicle_cmd_safety"
        diag.level = DiagnosticStatus.OK if status.valid else DiagnosticStatus.ERROR
        diag.message = status.reason_code
        diag.values = [KeyValue(key=str(key), value=str(value)) for key, value in status.diagnostics.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diag]
        self._diag_pub.publish(array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CollisionMonitorValidityMonitor()
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
