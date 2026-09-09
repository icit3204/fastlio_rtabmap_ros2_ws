"""ROS-facing mock-only MK-mini adapter.

The node is deliberately restricted to the existing safe command boundary and
an in-memory MockTransport.  It has no real-CAN implementation or transport
selection path.  Clock reads and ROS message conversion stay in this wrapper;
the D0 AdapterCore remains timestamp-injected and ROS-free.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .adapter import AdapterConfig, AdapterReason, AdapterResult, AdapterState, GateContext, MkminiAdapterCore, StationaryGearPolicy
from .codec import AliveCounter, CtrlFeedback, MkminiCanCodec, verify_checksum
from .health import HealthAssessment, HealthReason, HealthStatus
from .model import CanFrame, CtrlCommand
from .transport import MockTransport
from .codec import WheelFeedback


GATE_DIAGNOSTIC_NAME = "vehicle_cmd_safety/guarded_vehicle_cmd_gate"
GATE_HARDWARE_ID = "vehicle_cmd_safety"
GATE_TOPIC = "/vehicle_cmd_safety/state"
SAFE_COMMAND_TOPIC = "/vehicle_cmd_safe"
ADAPTER_DIAGNOSTIC_TOPIC = "/mkmini_cmd_adapter/state"
UNSUPPORTED_COMPONENT_EPSILON = 1.0e-9


@dataclass(frozen=True)
class FeedbackSnapshot:
    health: HealthAssessment
    ctrl_feedback: CtrlFeedback | None = None
    left_wheel_feedback: WheelFeedback | None = None
    right_wheel_feedback: WheelFeedback | None = None


class FeedbackProvider(Protocol):
    def snapshot(self, now_monotonic_sec: float) -> FeedbackSnapshot:
        """Return an explicitly timestamp-assessed feedback snapshot."""


class NoFeedbackProvider:
    """Production/default provider while no physical receive path exists."""

    def snapshot(self, now_monotonic_sec: float) -> FeedbackSnapshot:
        del now_monotonic_sec
        return FeedbackSnapshot(
            HealthAssessment(
                HealthStatus.UNKNOWN,
                False,
                unknown_reasons=(HealthReason.NO_FEEDBACK_SAMPLE,),
            )
        )


def _qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _bool_value(values: dict[str, str], key: str) -> bool | None:
    value = values.get(key)
    if value is None:
        return None
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    return None


class MkminiCmdAdapterNode(Node):
    """Standalone ROS wrapper whose only output sink is MockTransport."""

    def __init__(
        self,
        *,
        feedback_provider: FeedbackProvider | None = None,
        transport: MockTransport | None = None,
        monotonic_clock: Callable[[], float] | None = None,
    ) -> None:
        super().__init__("mkmini_cmd_adapter_node")
        self._feedback_provider = feedback_provider or NoFeedbackProvider()
        self._transport = transport or MockTransport()
        self._monotonic = monotonic_clock or time.monotonic
        self._alive = AliveCounter(0)
        self._core: MkminiAdapterCore | None = None
        self._last_result: AdapterResult | None = None
        self._last_reason = "STARTUP_NO_CONTEXT"
        self._last_frame: CanFrame | None = None
        self._last_frame_alive: int | None = None
        self._command: tuple[float, float] | None = None
        self._command_received: float | None = None
        self._command_generation = 0
        self._command_error: str | None = None
        self._gate: GateContext | None = None
        self._gate_received: float | None = None
        self._gate_error: str | None = None
        self._recovery_baseline_generation: int | None = None

        self.declare_parameter("command_timeout_sec", -1.0)
        self.declare_parameter("gate_state_timeout_sec", -1.0)
        self.declare_parameter("standstill_speed_threshold_mps", -1.0)
        self.declare_parameter("standstill_stability_sec", -1.0)
        # Defaults are the canonical production-boundary contract.  The
        # stationary dry-run launch remaps only these explicit interfaces;
        # it cannot select a hardware transport because this node owns only
        # MockTransport.
        self.declare_parameter("safe_command_topic", SAFE_COMMAND_TOPIC)
        self.declare_parameter("gate_state_topic", GATE_TOPIC)
        self.declare_parameter("diagnostic_topic", ADAPTER_DIAGNOSTIC_TOPIC)

        safe_command_topic = str(self.get_parameter("safe_command_topic").value)
        gate_state_topic = str(self.get_parameter("gate_state_topic").value)
        diagnostic_topic = str(self.get_parameter("diagnostic_topic").value)

        self._command_sub = self.create_subscription(
            TwistStamped, safe_command_topic, self._command_cb, _qos()
        )
        self._gate_sub = self.create_subscription(DiagnosticStatus, gate_state_topic, self._gate_cb, _qos())
        self._diagnostic_pub = self.create_publisher(
            DiagnosticStatus, diagnostic_topic, _qos(10)
        )
        self._frame_timer = self.create_timer(0.010, self._frame_tick)
        self._diagnostic_timer = self.create_timer(0.050, self._diagnostic_tick)

    @property
    def transport(self) -> MockTransport:
        return self._transport

    @property
    def core(self) -> MkminiAdapterCore | None:
        return self._core

    def _read_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _configuration(self) -> AdapterConfig | None:
        values = {
            "input_timeout_sec": self._read_float("command_timeout_sec"),
            "gate_timeout_sec": self._read_float("gate_state_timeout_sec"),
            "standstill_threshold": self._read_float("standstill_speed_threshold_mps"),
            "standstill_stability": self._read_float("standstill_stability_sec"),
        }
        if any(not math.isfinite(value) or value <= 0.0 for value in values.values()):
            return None
        return AdapterConfig(
            input_timeout_sec=values["input_timeout_sec"],
            standstill_speed_threshold_mps=values["standstill_threshold"],
            standstill_stability_sec=values["standstill_stability"],
            stationary_gear_policy=StationaryGearPolicy.UNRESOLVED,
        )

    def _command_cb(self, msg: TwistStamped) -> None:
        now = self._monotonic()
        self._command_generation += 1
        values = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        )
        if any(not math.isfinite(value) for value in values):
            self._command = (0.0, 0.0)
            self._command_error = "NONFINITE_TWIST"
        elif any(abs(value) > UNSUPPORTED_COMPONENT_EPSILON for value in (values[1], values[2], values[3], values[4])):
            self._command = (0.0, 0.0)
            self._command_error = "UNSUPPORTED_TWIST_COMPONENT"
        else:
            self._command = (float(values[0]), float(values[5]))
            self._command_error = None
        self._command_received = now

    def _gate_cb(self, msg: DiagnosticStatus) -> None:
        now = self._monotonic()
        self._gate_received = now
        self._gate = None
        self._gate_error = None
        if msg.name != GATE_DIAGNOSTIC_NAME or msg.hardware_id != GATE_HARDWARE_ID:
            self._gate_error = "GATE_IDENTITY_INVALID"
            return
        values = {item.key: item.value for item in msg.values}
        state = values.get("state")
        arm_pending = _bool_value(values, "arm_pending")
        fault_latched = _bool_value(values, "fault_latched")
        safe_zero_quiescent = _bool_value(values, "safe_zero_quiescent")
        if state not in {"DISARMED", "ARMED", "FAULT"}:
            self._gate_error = "GATE_STATE_INVALID"
            return
        if arm_pending is None or fault_latched is None or safe_zero_quiescent is None:
            self._gate_error = "GATE_FIELDS_INCOMPLETE"
            return
        self._gate = GateContext(
            state=state,
            arm_pending=arm_pending,
            fault_latched=fault_latched,
            safe_zero_quiescent=safe_zero_quiescent,
            reason=msg.message,
        )

    def _fresh_gate(self, now: float, timeout: float) -> GateContext | None:
        if self._gate is None or self._gate_received is None:
            return None
        if now - self._gate_received > timeout:
            return None
        return self._gate

    def _frame_tick(self) -> None:
        now = self._monotonic()
        configuration = self._configuration()
        if configuration is None:
            self._last_result = None
            self._last_reason = "UNRESOLVED_SAFETY_PARAMETERS"
            return
        if self._core is None:
            self._core = MkminiAdapterCore(configuration)
        elif self._core.config != configuration:
            # Parameter changes are accepted only as test-time configuration;
            # resetting the pure core prevents stale policy from carrying over.
            self._core = MkminiAdapterCore(configuration)

        feedback = self._feedback_provider.snapshot(now)
        command = self._command
        if command is None or self._command_received is None:
            self._last_result = None
            self._last_reason = "NO_SAFE_COMMAND"
            return
        gate = self._fresh_gate(now, self._read_float("gate_state_timeout_sec"))
        gate_eligible = (
            gate is not None
            and gate.state == "ARMED"
            and not gate.arm_pending
            and not gate.fault_latched
            and not gate.safe_zero_quiescent
        )
        if (
            self._core.requires_fresh_command
            and feedback.health.status is HealthStatus.VALID
            and gate_eligible
            and self._command_error is None
            and self._recovery_baseline_generation is None
        ):
            self._recovery_baseline_generation = self._command_generation
        recovery_waiting_for_new_command = (
            self._core.requires_fresh_command
            and self._recovery_baseline_generation is not None
            and self._command_generation <= self._recovery_baseline_generation
        )
        step_command = (0.0, 0.0) if recovery_waiting_for_new_command else command
        result = self._core.step(
            step_command[0],
            step_command[1],
            command_timestamp_sec=self._command_received,
            now_sec=now,
            feedback_health=feedback.health,
            ctrl_feedback=feedback.ctrl_feedback,
            left_wheel_feedback=feedback.left_wheel_feedback,
            right_wheel_feedback=feedback.right_wheel_feedback,
            gate=gate,
        )
        self._last_result = result
        self._last_reason = self._command_error or result.reason.value
        if result.requires_fresh_command:
            if feedback.health.status is not HealthStatus.VALID or not gate_eligible or self._command_error is not None:
                self._recovery_baseline_generation = None
        else:
            self._recovery_baseline_generation = None
        if result.command.gear is None:
            return
        try:
            alive = self._alive.next()
            frame = MkminiCanCodec.encode(CtrlCommand(
                result.command.gear,
                result.command.speed_magnitude_mps,
                result.command.inner_wheel_steering_deg,
                alive,
            ))
        except (TypeError, ValueError):
            self._last_reason = "PHYSICAL_COMMAND_NOT_ENCODABLE"
            return
        self._transport.send(frame, timestamp=now)
        self._last_frame = frame
        self._last_frame_alive = alive

    def _diagnostic_tick(self) -> None:
        now = self._monotonic()
        result = self._last_result
        diag = DiagnosticStatus()
        diag.name = "mkmini_cmd_adapter/state"
        diag.hardware_id = "mkmini_cmd_adapter"
        if result is not None and result.state is AdapterState.FAULTED:
            diag.level = DiagnosticStatus.ERROR
        elif result is not None and result.valid:
            diag.level = DiagnosticStatus.OK
        else:
            diag.level = DiagnosticStatus.WARN
        diag.message = self._last_reason
        command_age = None if self._command_received is None else max(0.0, now - self._command_received)
        gate_age = None if self._gate_received is None else max(0.0, now - self._gate_received)
        values = {
            "adapter_state": "NONE" if result is None else result.state.value,
            "reason": self._last_reason,
            "motion_eligible": str(bool(result and result.valid and result.command.speed_magnitude_mps > 0.0)).lower(),
            "command_receipt_age_sec": "none" if command_age is None else f"{command_age:.6f}",
            "gate_receipt_age_sec": "none" if gate_age is None else f"{gate_age:.6f}",
            "gate_public_state": "none" if self._gate is None else str(self._gate.state),
            "arm_pending": "unknown" if self._gate is None else str(self._gate.arm_pending).lower(),
            "fault_latched": "unknown" if self._gate is None else str(self._gate.fault_latched).lower(),
            "safe_zero_quiescent": "unknown" if self._gate is None else str(self._gate.safe_zero_quiescent).lower(),
            "feedback_health": "unknown" if result is None else ("valid" if result.valid else "ineligible"),
            "kinematic_valid": "unknown" if result is None or result.kinematics is None else str(result.kinematics.valid).lower(),
            "kinematic_reason": "none" if result is None or result.kinematics is None else result.kinematics.reason.value,
            "direction": "none" if result is None or result.kinematics is None else result.kinematics.direction.value,
            "requested_v_mps": "none" if result is None or result.kinematics is None else f"{result.kinematics.requested_v_mps:.6f}",
            "requested_w_radps": "none" if result is None or result.kinematics is None else f"{result.kinematics.requested_w_radps:.6f}",
            "physical_speed_mps": "none" if result is None else f"{result.command.speed_magnitude_mps:.6f}",
            "physical_steering_deg": "none" if result is None else f"{result.command.inner_wheel_steering_deg:.6f}",
            "stationary_policy": StationaryGearPolicy.UNRESOLVED.value,
            "last_frame_emitted": str(self._last_frame is not None).lower(),
            "mock_frame_count": str(self._transport.count),
            "last_alive": "none" if self._last_frame_alive is None else str(self._last_frame_alive),
            "gate_error": self._gate_error or "none",
            "command_error": self._command_error or "none",
            "transport_mode": "MOCK",
            "last_frame_can_id": "none" if self._last_frame is None else f"0x{self._last_frame.can_id:08X}",
            "last_frame_extended": "none" if self._last_frame is None else str(self._last_frame.is_extended).lower(),
            "last_frame_dlc": "none" if self._last_frame is None else str(len(self._last_frame.data)),
        }
        if self._last_frame is not None:
            word = int.from_bytes(self._last_frame.data[:7], "little")
            steering_raw = (word >> 20) & 0xFFFF
            if steering_raw & 0x8000:
                steering_raw -= 0x10000
            values.update({
                "last_frame_gear": str(word & 0x0F),
                "last_frame_speed_mps": f"{((word >> 4) & 0xFFFF) * 0.001:.3f}",
                "last_frame_steering_deg": f"{steering_raw * 0.01:.2f}",
                "last_frame_alive": str((word >> 52) & 0x0F),
                "last_frame_reserved_bits_36_51": str((word >> 36) & 0xFFFF),
                "last_frame_checksum_valid": str(verify_checksum(self._last_frame)).lower(),
            })
        diag.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        self._diagnostic_pub.publish(diag)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MkminiCmdAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
