"""ROS shadow controller-validity publisher for software-only qualification."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .health import HealthAssessment
from .validity import (
    MkminiControllerValidityCore,
    ValidityEvidence,
    ValidityResult,
    ValidityState,
)


SHADOW_VALID_TOPIC = "/phase5a/mkmini/controller_valid"
SHADOW_DIAGNOSTIC_TOPIC = "/phase5a/mkmini/controller_valid/state"


@dataclass(frozen=True)
class ValidityFeedbackSnapshot:
    """Normalized provider output; no raw CAN or ROS messages are required."""

    ctrl_health: HealthAssessment | None
    diagnostic_health: HealthAssessment | None
    ctrl_age_sec: float | None
    diagnostic_age_sec: float | None

    def evidence(self) -> ValidityEvidence:
        return ValidityEvidence(
            ctrl_health=self.ctrl_health,
            diagnostic_health=self.diagnostic_health,
            ctrl_age_sec=self.ctrl_age_sec,
            diagnostic_age_sec=self.diagnostic_age_sec,
        )


class ValidityFeedbackProvider(Protocol):
    def snapshot(self, now_monotonic_sec: float) -> ValidityFeedbackSnapshot:
        """Return normalized, explicitly age-assessed feedback evidence."""


class NoValidityFeedbackProvider:
    """Default provider while no physical feedback transport exists."""

    def snapshot(self, now_monotonic_sec: float) -> ValidityFeedbackSnapshot:
        del now_monotonic_sec
        return ValidityFeedbackSnapshot(None, None, None, None)


def _qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class MkminiControllerValidityNode(Node):
    """Publishes only the Phase-5A shadow controller-validity authority."""

    def __init__(
        self,
        *,
        feedback_provider: ValidityFeedbackProvider | None = None,
        monotonic_clock: Callable[[], float] | None = None,
    ) -> None:
        super().__init__("mkmini_controller_validity_node")
        self._feedback_provider = feedback_provider or NoValidityFeedbackProvider()
        self._monotonic = monotonic_clock or time.monotonic
        self._core: MkminiControllerValidityCore | None = None
        self._last_result: ValidityResult | None = None

        # Safety-critical thresholds remain explicitly unresolved by default.
        self.declare_parameter("feedback_source_timeout_sec", -1.0)
        self.declare_parameter("recovery_stability_sec", -1.0)
        self.declare_parameter("heartbeat_hz", 20.0)

        self._valid_pub = self.create_publisher(Bool, SHADOW_VALID_TOPIC, _qos())
        self._diagnostic_pub = self.create_publisher(
            DiagnosticStatus, SHADOW_DIAGNOSTIC_TOPIC, _qos(10)
        )
        heartbeat_hz = float(self.get_parameter("heartbeat_hz").value)
        if not math.isfinite(heartbeat_hz) or heartbeat_hz <= 0.0:
            heartbeat_hz = 20.0
        self._timer = self.create_timer(1.0 / heartbeat_hz, self._tick)

    @property
    def last_result(self) -> ValidityResult | None:
        return self._last_result

    @staticmethod
    def _configured(value: object) -> float | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) and number > 0.0 else None

    def _make_core(self) -> MkminiControllerValidityCore:
        return MkminiControllerValidityCore(
            self._configured(self.get_parameter("feedback_source_timeout_sec").value),
            self._configured(self.get_parameter("recovery_stability_sec").value),
        )

    def _tick(self) -> None:
        now = self._monotonic()
        configured_core = self._make_core()
        if self._core is None or (
            self._core.source_timeout_sec != configured_core.source_timeout_sec
            or self._core.recovery_stability_sec != configured_core.recovery_stability_sec
        ):
            self._core = configured_core
        snapshot = self._feedback_provider.snapshot(now)
        result = self._core.evaluate(snapshot.evidence(), now)
        self._last_result = result

        valid = Bool()
        valid.data = result.valid
        self._valid_pub.publish(valid)

        diagnostic = DiagnosticStatus()
        diagnostic.name = "mkmini_controller_validity/state"
        diagnostic.hardware_id = "mkmini_controller_validity"
        diagnostic.level = DiagnosticStatus.OK if result.valid else DiagnosticStatus.WARN
        diagnostic.message = result.reason
        values = {
            "validity_state": result.state.value,
            "valid": str(result.valid).lower(),
            "reason": result.reason,
            "contributing_reasons": ",".join(result.contributing_reasons),
            "ctrl_feedback_age_sec": "none" if result.ctrl_age_sec is None else f"{result.ctrl_age_sec:.6f}",
            "diagnostic_feedback_age_sec": "none" if result.diagnostic_age_sec is None else f"{result.diagnostic_age_sec:.6f}",
            "recovery_stable_for_sec": f"{result.stable_for_sec:.6f}",
            "recovery_stability_sec": str(self.get_parameter("recovery_stability_sec").value),
            "feedback_source_timeout_sec": str(self.get_parameter("feedback_source_timeout_sec").value),
            "configuration": "configured" if result.configured else "UNRESOLVED",
            "publisher_authority_mode": "SHADOW_PHASE5A",
            "ctrl_fb_required": "true",
            "veh_fb_diag_required": "true",
            "wheel_feedback_required_for_controller_valid": "false",
        }
        diagnostic.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        self._diagnostic_pub.publish(diagnostic)


def main(args: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = MkminiControllerValidityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
