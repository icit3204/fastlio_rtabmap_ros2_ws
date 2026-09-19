"""Publish controller permission only while Nav2's controller is lifecycle-active.

This is a small physical-bringup authority source, not a command fixture. It
queries the real ``controller_server`` lifecycle state and fails false when
the service or its response becomes stale.
"""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class ControllerLifecycleHealth:
    """Pure lifecycle-state validity with a bounded response age."""

    def __init__(self, timeout_sec: float = 0.50) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and positive")
        self.timeout_sec = timeout_sec
        self.state_id: int | None = None
        self.state_label: str | None = None
        self.last_response_at: float | None = None

    def observe(self, *, state_id: int, state_label: str, now: float) -> None:
        if not math.isfinite(now) or not state_label:
            self.state_id = None
            self.state_label = None
            self.last_response_at = None
            return
        self.state_id = state_id
        self.state_label = state_label
        self.last_response_at = now

    def query_failed(self) -> None:
        self.state_id = None
        self.state_label = None
        self.last_response_at = None

    def valid(self, now: float) -> bool:
        return bool(
            math.isfinite(now)
            and self.state_id == State.PRIMARY_STATE_ACTIVE
            and self.state_label == "active"
            and self.last_response_at is not None
            and 0.0 <= now - self.last_response_at <= self.timeout_sec
        )


class Nav2ControllerValidityMonitor(Node):
    def __init__(self) -> None:
        super().__init__("nav2_controller_validity_monitor")
        self.declare_parameter("controller_node_name", "controller_server")
        self.declare_parameter("validity_output_topic", "/system/controller_valid")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("query_rate_hz", 5.0)
        self.declare_parameter("response_timeout_sec", 0.50)

        node_name = str(self.get_parameter("controller_node_name").value).strip("/")
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self._query_period = 1.0 / float(self.get_parameter("query_rate_hz").value)
        timeout = float(self.get_parameter("response_timeout_sec").value)
        if not node_name or not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("controller node name and positive publish rate are required")
        if not math.isfinite(self._query_period) or self._query_period <= 0.0:
            raise ValueError("query_rate_hz must be finite and positive")

        self._health = ControllerLifecycleHealth(timeout)
        self._state_client = self.create_client(GetState, f"/{node_name}/get_state")
        self._future = None
        self._last_query_at: float | None = None
        self._query_started_at: float | None = None
        self._last_query_status = "WAITING_FOR_CONTROLLER_SERVICE"
        self._valid_pub = self.create_publisher(
            Bool, str(self.get_parameter("validity_output_topic").value), 10
        )
        self._diag_pub = self.create_publisher(DiagnosticStatus, "/diagnostics", 10)
        self.create_timer(1.0 / publish_rate, self._timer_cb)

    def _complete_query(self, now: float) -> None:
        if self._future is None:
            return
        # A DDS service response can be lost while the controller itself is
        # still active.  It must revoke validity for this interval, but must
        # not leave this monitor permanently wedged behind one unfinished
        # future.  The next periodic query re-establishes authority only from
        # a fresh real lifecycle response.
        if not self._future.done():
            if (self._query_started_at is not None
                    and now - self._query_started_at > self._health.timeout_sec):
                self._future = None
                self._query_started_at = None
                self._health.query_failed()
                self._last_query_status = "GET_STATE_TIMEOUT"
            return
        future, self._future = self._future, None
        self._query_started_at = None
        try:
            response = future.result()
            state = response.current_state
            if not isinstance(state.label, str) or not state.label:
                raise ValueError("controller GetState returned a malformed label")
            self._health.observe(state_id=state.id, state_label=state.label.lower(), now=now)
            self._last_query_status = state.label.upper()
        except Exception as exc:  # service loss is permission loss
            self._health.query_failed()
            self._last_query_status = f"GET_STATE_FAILED:{type(exc).__name__}"

    def _timer_cb(self) -> None:
        now = time.monotonic()
        self._complete_query(now)
        if (
            self._future is None
            and (self._last_query_at is None or now - self._last_query_at >= self._query_period)
        ):
            if self._state_client.service_is_ready():
                self._last_query_at = now
                self._future = self._state_client.call_async(GetState.Request())
                self._query_started_at = now
                self._last_query_status = "GET_STATE_IN_FLIGHT"
            else:
                self._last_query_status = "WAITING_FOR_CONTROLLER_SERVICE"

        valid = self._health.valid(now)
        self._valid_pub.publish(Bool(data=valid))
        diag = DiagnosticStatus()
        diag.name = "vehicle_cmd_safety/nav2_controller_validity_monitor"
        diag.hardware_id = "nav2_controller_lifecycle"
        diag.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        diag.message = "CONTROLLER_ACTIVE" if valid else self._last_query_status
        age = (None if self._health.last_response_at is None
               else max(0.0, now - self._health.last_response_at))
        diag.values = [
            KeyValue(key="controller_state", value=self._health.state_label or "unknown"),
            KeyValue(key="state_response_age_sec", value=str(age)),
            KeyValue(key="controller_valid", value=str(valid).lower()),
            KeyValue(key="state_query_status", value=self._last_query_status),
        ]
        self._diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2ControllerValidityMonitor()
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
