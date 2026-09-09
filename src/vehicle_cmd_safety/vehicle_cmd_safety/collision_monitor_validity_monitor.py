"""ROS wrapper for the Phase 4 Collision Monitor validity permission."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool

from vehicle_cmd_safety.validity_core import (
    CollisionMonitorState,
    CollisionMonitorValidityCore,
    ValidityConfig,
)


@dataclass
class ServiceQueryTracker:
    """Bound one service future and make retirement/retry explicit."""

    name: str
    watchdog_sec: float
    retry_interval_sec: float
    future: Any = None
    generation: int = 0
    started_at: float | None = None
    next_attempt_at: float = 0.0
    status: str = "WAITING_FOR_SERVICE"
    retry_count: int = 0
    last_success_at: float | None = None

    def can_issue(self, now: float) -> bool:
        return self.future is None and now >= self.next_attempt_at

    def begin(self, future: Any, now: float) -> int:
        if self.future is not None:
            raise RuntimeError(f"overlapping {self.name} request")
        self.generation += 1
        self.future = future
        self.started_at = now
        self.status = "IN_FLIGHT"
        return self.generation

    def complete(self, now: float) -> Any:
        future = self.future
        self.future = None
        self.started_at = None
        self.status = "VALID_RESPONSE"
        self.last_success_at = now
        self.next_attempt_at = now + self.retry_interval_sec
        return future

    def expire(self, now: float) -> Any | None:
        if self.future is None or self.started_at is None:
            return None
        if now - self.started_at <= self.watchdog_sec:
            return None
        future = self.future
        self.future = None
        self.started_at = None
        self.generation += 1
        self.retry_count += 1
        self.status = "TIMED_OUT"
        self.next_attempt_at = now + self.retry_interval_sec
        cancel = getattr(future, "cancel", None)
        if callable(cancel):
            cancel()
        return future

    def fail(self, now: float, status: str) -> None:
        self.future = None
        self.started_at = None
        self.generation += 1
        self.retry_count += 1
        self.status = status
        self.next_attempt_at = now + self.retry_interval_sec

    def mark_waiting(self, now: float) -> None:
        if self.future is None:
            self.status = "WAITING_FOR_SERVICE"
            self.next_attempt_at = max(self.next_attempt_at, now)

    def last_success_age(self, now: float) -> str:
        if self.last_success_at is None:
            return "none"
        return f"{max(0.0, now - self.last_success_at):.6f}"


def parse_state_response(result: Any) -> bool:
    if result is None or not hasattr(result, "current_state"):
        raise ValueError("malformed GetState response")
    label = getattr(result.current_state, "label", None)
    if not isinstance(label, str) or not label:
        raise ValueError("malformed lifecycle state label")
    return label.lower() == "active"


def parse_parameters_response(result: Any, source_name: str, expected_type: str, expected_topic: str) -> tuple[bool, bool, bool]:
    if result is None or not hasattr(result, "values") or len(result.values) < 3:
        raise ValueError("malformed GetParameters response")
    values = result.values
    sources = getattr(values[0], "string_array_value", None)
    actual_type = getattr(values[1], "string_value", None)
    actual_topic = getattr(values[2], "string_value", None)
    if not isinstance(sources, (list, tuple)) or not isinstance(actual_type, str) or not isinstance(actual_topic, str):
        raise ValueError("malformed parameter value types")
    return source_name in sources, actual_type == expected_type, actual_topic == expected_topic


class CollisionMonitorValidityMonitor(Node):
    def __init__(self) -> None:
        super().__init__("collision_monitor_validity_monitor")
        self._declare_parameters()
        self._core = CollisionMonitorValidityCore(self._config_from_parameters())

        self._validity_output_topic = str(self.get_parameter("validity_output_topic").value)
        self._valid_pub = self.create_publisher(Bool, self._validity_output_topic, 10)
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
        self._query_watchdog_sec = float(self.get_parameter("query_watchdog_sec").value)
        self._query_retry_interval_sec = float(self.get_parameter("query_retry_interval_sec").value)
        if self._query_watchdog_sec <= 0.0 or self._query_retry_interval_sec < 0.0:
            raise ValueError("query watchdog must be positive and retry interval nonnegative")
        self._state_query = ServiceQueryTracker(
            "state", self._query_watchdog_sec, self._query_retry_interval_sec
        )
        self._params_query = ServiceQueryTracker(
            "parameters", self._query_watchdog_sec, self._query_retry_interval_sec
        )
        self._last_state_service_ready = False
        self._last_params_service_ready = False

        if self._core.config.source_type == "scan":
            self.create_subscription(LaserScan, self._core.config.source_topic, self._scan_cb, qos_profile_sensor_data)
        elif self._core.config.source_type == "pointcloud":
            self.create_subscription(PointCloud2, self._core.config.source_topic, self._cloud_cb, qos_profile_sensor_data)
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
        self.declare_parameter("query_watchdog_sec", 0.25)
        self.declare_parameter("query_retry_interval_sec", 0.10)
        self.declare_parameter("validity_output_topic", "/system/collision_monitor_valid")

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

    def _now_ros(self) -> float:
        now = self.get_clock().now()
        return float(now.nanoseconds) * 1e-9

    @staticmethod
    def _header_stamp_sec(msg: LaserScan | PointCloud2) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _scan_cb(self, msg: LaserScan) -> None:
        valid = (
            msg.angle_increment > 0.0
            and msg.range_max > msg.range_min > 0.0
            and len(msg.ranges) > 0
        )
        self._core.set_observation(
            msg.header.frame_id,
            valid,
            self._now_steady(),
            self._header_stamp_sec(msg),
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        valid = msg.point_step > 0 and msg.row_step >= msg.point_step * msg.width and len(msg.data) >= msg.row_step
        self._core.set_observation(
            msg.header.frame_id,
            valid,
            self._now_steady(),
            self._header_stamp_sec(msg),
        )

    def _timer_cb(self) -> None:
        now = self._now_steady()
        ros_now = self._now_ros()
        try:
            self._update_collision_monitor_state(now)
        except Exception as exc:  # noqa: BLE001 - heartbeat must survive service-query failures.
            self.get_logger().error(f"collision monitor state update failed: {exc!r}")
            self._state_future = None
            self._params_future = None
        self._core.set_valid_publisher_count(len(self.get_publishers_info_by_topic(self._validity_output_topic)))
        status = self._core.tick(now, ros_now)
        msg = Bool()
        msg.data = bool(status.valid)
        self._valid_pub.publish(msg)
        self._publish_diag(status)

    def _retire_expired_queries(self, now: float) -> None:
        for tracker in (self._state_query, self._params_query):
            tracker.expire(now)

    def _update_collision_monitor_state(self, now: float | None = None) -> None:
        now = self._now_steady() if now is None else now
        self._retire_expired_queries(now)
        state = CollisionMonitorState(
            lifecycle_reachable=self._cached_state.lifecycle_reachable,
            lifecycle_active=self._cached_state.lifecycle_active,
            config_reachable=self._cached_state.config_reachable,
            configured_source_present=self._cached_state.configured_source_present,
            configured_source_type_matches=self._cached_state.configured_source_type_matches,
            configured_source_topic_matches=self._cached_state.configured_source_topic_matches,
        )
        if self._state_query.status in {"TIMED_OUT", "EXCEPTION"}:
            state.lifecycle_active = False
        if self._params_query.status in {"TIMED_OUT", "EXCEPTION"}:
            state.configured_source_present = False
            state.configured_source_type_matches = False
            state.configured_source_topic_matches = False
        state.lifecycle_reachable = self._state_client.service_is_ready()
        state.config_reachable = self._params_client.service_is_ready()
        self._last_state_service_ready = state.lifecycle_reachable
        self._last_params_service_ready = state.config_reachable
        if not state.lifecycle_reachable:
            self._state_query.mark_waiting(now)
            state.lifecycle_active = False
        elif self._state_query.can_issue(now):
            self._state_future = self._state_client.call_async(GetState.Request())
            self._state_query.begin(self._state_future, now)
        if not state.config_reachable:
            self._params_query.mark_waiting(now)
            state.config_reachable = False
            state.configured_source_present = False
            state.configured_source_type_matches = False
            state.configured_source_topic_matches = False
        elif self._params_query.can_issue(now):
            req = GetParameters.Request()
            src = self._core.config.expected_observation_source_name
            req.names = ["observation_sources", f"{src}.type", f"{src}.topic"]
            self._params_future = self._params_client.call_async(req)
            self._params_query.begin(self._params_future, now)
        if self._state_query.future is not None and self._state_query.future.done():
            try:
                result = self._state_query.future.result()
                if result is None:
                    raise RuntimeError("empty GetState response")
                state.lifecycle_active = parse_state_response(result)
                self._state_query.complete(now)
            except Exception as exc:  # noqa: BLE001 - query failure is fail-closed and recoverable.
                self.get_logger().warning(f"collision monitor state query failed: {exc!r}")
                self._state_query.fail(now, "EXCEPTION")
                state.lifecycle_active = False
            self._state_future = None
        if self._params_query.future is not None and self._params_query.future.done():
            try:
                result = self._params_query.future.result()
                src = self._core.config.expected_observation_source_name
                (
                    state.configured_source_present,
                    state.configured_source_type_matches,
                    state.configured_source_topic_matches,
                ) = parse_parameters_response(
                    result,
                    src,
                    self._core.config.expected_observation_source_type,
                    self._core.config.expected_observation_source_topic,
                )
                self._params_query.complete(now)
            except Exception as exc:  # noqa: BLE001 - query failure is fail-closed and recoverable.
                self.get_logger().warning(f"collision monitor parameter query failed: {exc!r}")
                self._params_query.fail(now, "EXCEPTION")
                state.configured_source_present = False
                state.configured_source_type_matches = False
                state.configured_source_topic_matches = False
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
        now = self._now_steady()
        diag.values.extend([
            KeyValue(key="state_query_status", value=self._state_query.status),
            KeyValue(key="state_query_retry_count", value=str(self._state_query.retry_count)),
            KeyValue(key="state_query_last_success_age", value=self._state_query.last_success_age(now)),
            KeyValue(key="state_service_ready", value=str(self._last_state_service_ready).lower()),
            KeyValue(key="params_query_status", value=self._params_query.status),
            KeyValue(key="params_query_retry_count", value=str(self._params_query.retry_count)),
            KeyValue(key="params_query_last_success_age", value=self._params_query.last_success_age(now)),
            KeyValue(key="params_service_ready", value=str(self._last_params_service_ready).lower()),
        ])
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
