"""Pure physical-keyboard commissioning architecture.

This module is deliberately ROS-free and clock-free.  It prepares the
standalone keyboard path for a later supervised hardware qualification while
making the current unresolved protocol authority fail closed before any
transmit-capable factory can be constructed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, Protocol

from .adapter import AdapterState, PhysicalCommandModel
from .codec import ManufacturerCommandValidator, MkminiCanCodec, RunningMode
from .kinematics import KinematicsResult, MkminiKinematicsCore
from .model import CanFrame, CtrlCommand, Gear
from .transport import FrameSink, MockTransport, ReadOnlyTransport, TransportMode


class RealCanAuthorityError(RuntimeError):
    """Raised before a transmit-capable object can be constructed."""


@dataclass(frozen=True)
class RealTransportAuthority:
    """Independent authorities required before a future physical TX path."""

    protocol_profile_qualified: bool = False
    brake_profile_qualified: bool = False
    stationary_policy_qualified: bool = False
    feedback_path_qualified: bool = False
    real_transport_qualified: bool = False
    # K3D separates static receive-path capability from live feedback health.
    # ``feedback_path_qualified`` is retained for compatibility with K2 callers;
    # TX qualification requires both the explicit capability and live-health
    # fields below as well.
    feedback_transport_qualified: bool = False
    runtime_feedback_healthy: bool = False

    @classmethod
    def current_unqualified(cls) -> "RealTransportAuthority":
        return cls()

    @property
    def tx_qualified(self) -> bool:
        return all(
            (
                self.protocol_profile_qualified,
                self.brake_profile_qualified,
                self.stationary_policy_qualified,
                self.feedback_path_qualified,
                self.feedback_transport_qualified,
                self.runtime_feedback_healthy,
                self.real_transport_qualified,
            )
        )

    def block_reasons(self) -> tuple[str, ...]:
        reasons: list[str] = []
        if not self.protocol_profile_qualified:
            reasons.append("REAL_CAN_BLOCKED_PROTOCOL_PROFILE_UNQUALIFIED")
        if not self.brake_profile_qualified:
            reasons.append("REAL_CAN_BLOCKED_BRAKE_PROFILE_UNRESOLVED")
        if not self.stationary_policy_qualified:
            reasons.append("REAL_CAN_BLOCKED_STATIONARY_POLICY_UNRESOLVED")
        if not self.feedback_path_qualified:
            reasons.append("REAL_CAN_BLOCKED_FEEDBACK_PATH_UNQUALIFIED")
        if not self.feedback_transport_qualified:
            reasons.append("REAL_CAN_BLOCKED_FEEDBACK_TRANSPORT_UNQUALIFIED")
        if not self.runtime_feedback_healthy:
            reasons.append("REAL_CAN_BLOCKED_RUNTIME_FEEDBACK_UNHEALTHY")
        if not self.real_transport_qualified:
            reasons.append("REAL_CAN_BLOCKED_TRANSPORT_UNQUALIFIED")
        return tuple(reasons)

    def require_tx(self) -> None:
        reasons = self.block_reasons()
        if reasons:
            raise RealCanAuthorityError(";".join(reasons))


class TransportFactory:
    """Construct only memory/read-only surfaces in the current phase.

    A future transmit constructor is dependency-injected deliberately.  The
    authority check runs before that callback, so the current unresolved
    profile cannot reach a physical transport constructor.
    """

    @staticmethod
    def create(
        mode: TransportMode,
        *,
        authority: RealTransportAuthority | None = None,
        tx_constructor: Callable[[], FrameSink] | None = None,
    ) -> FrameSink | ReadOnlyTransport:
        if mode is TransportMode.MOCK:
            return MockTransport()
        if mode is TransportMode.READ_ONLY:
            return ReadOnlyTransport()
        if mode is not TransportMode.TX_CAPABLE:
            raise ValueError("unknown transport mode")
        (authority or RealTransportAuthority.current_unqualified()).require_tx()
        if tx_constructor is None:
            raise RealCanAuthorityError("REAL_CAN_BLOCKED_TRANSPORT_IMPLEMENTATION_UNAVAILABLE")
        return tx_constructor()


@dataclass(frozen=True)
class KeyboardFeedbackEvidence:
    """Normalized feedback evidence supplied by a future provider."""

    ctrl_current: bool = False
    diagnostic_current: bool = False
    ctrl_checksum_valid: bool = False
    diagnostic_checksum_valid: bool = False
    ctrl_alive_contiguous: bool = False
    diagnostic_alive_contiguous: bool = False
    mode: RunningMode | int | None = None
    emergency_stop_asserted: bool = False
    blocking_fault: bool = False

    @property
    def eligible(self) -> bool:
        return not self.failure_reasons

    @property
    def failure_reasons(self) -> tuple[str, ...]:
        reasons: list[str] = []
        if not self.ctrl_current:
            reasons.append("CTRL_FB_STALE_OR_MISSING")
        if not self.diagnostic_current:
            reasons.append("VEH_FB_DIAG_STALE_OR_MISSING")
        if not self.ctrl_checksum_valid or not self.diagnostic_checksum_valid:
            reasons.append("FEEDBACK_CHECKSUM_INVALID")
        if not self.ctrl_alive_contiguous or not self.diagnostic_alive_contiguous:
            reasons.append("FEEDBACK_ALIVE_INVALID")
        if self.mode != RunningMode.AUTO:
            if self.mode == RunningMode.REMOTE:
                reasons.append("MODE_REMOTE")
            elif self.mode == RunningMode.STOP:
                reasons.append("MODE_STOP")
            else:
                reasons.append("MODE_UNKNOWN")
        if self.emergency_stop_asserted:
            reasons.append("ESTOP_ASSERTED")
        if self.blocking_fault:
            reasons.append("DIAGNOSTIC_FAULT")
        return tuple(reasons)


class KeyboardState(str, Enum):
    DISARMED = "DISARMED"
    WAITING_FOR_FEEDBACK = "WAITING_FOR_FEEDBACK"
    ARM_READY = "ARM_READY"
    ARMED_ZERO = "ARMED_ZERO"
    MOVING = "MOVING"
    STOPPING = "STOPPING"
    FAULTED = "FAULTED"


@dataclass(frozen=True)
class KeyboardStateResult:
    state: KeyboardState
    accepted: bool
    reason: str
    kinematics: KinematicsResult | None
    command: PhysicalCommandModel | None
    feedback_eligible: bool
    requires_fresh_command: bool


def _zero_command(gear: Gear | None = None, steering_deg: float = 0.0) -> PhysicalCommandModel:
    return PhysicalCommandModel(gear, 0.0, steering_deg)


class KeyboardCommissioningCore:
    """Explicit keyboard state machine with caller-supplied timestamps."""

    def __init__(
        self,
        *,
        keyboard_deadman_sec: float = 0.25,
        transport_mode: TransportMode = TransportMode.MOCK,
        authority: RealTransportAuthority | None = None,
        kinematics: MkminiKinematicsCore | None = None,
    ) -> None:
        if not math.isfinite(keyboard_deadman_sec) or keyboard_deadman_sec <= 0.0:
            raise ValueError("keyboard_deadman_sec must be finite and positive")
        self.keyboard_deadman_sec = keyboard_deadman_sec
        self.transport_mode = transport_mode
        self.authority = authority or RealTransportAuthority.current_unqualified()
        self.kinematics = kinematics or MkminiKinematicsCore()
        self.state = KeyboardState.WAITING_FOR_FEEDBACK
        self._feedback: KeyboardFeedbackEvidence | None = None
        self._last_input_sec: float | None = None
        self._last_motion: PhysicalCommandModel | None = None
        self._requires_fresh_command = False

    @property
    def requires_fresh_command(self) -> bool:
        return self._requires_fresh_command

    @property
    def feedback_eligible(self) -> bool:
        return self._feedback is not None and self._feedback.eligible

    def _result(
        self,
        accepted: bool,
        reason: str,
        kinematics: KinematicsResult | None = None,
        command: PhysicalCommandModel | None = None,
    ) -> KeyboardStateResult:
        return KeyboardStateResult(
            self.state,
            accepted,
            reason,
            kinematics,
            command,
            self.feedback_eligible,
            self._requires_fresh_command,
        )

    def _fault(self, reason: str, kinematics: KinematicsResult | None = None) -> KeyboardStateResult:
        self.state = KeyboardState.FAULTED
        self._requires_fresh_command = True
        return self._result(False, reason, kinematics)

    def observe_feedback(self, evidence: KeyboardFeedbackEvidence) -> KeyboardStateResult:
        if not isinstance(evidence, KeyboardFeedbackEvidence):
            raise TypeError("feedback must be KeyboardFeedbackEvidence")
        self._feedback = evidence
        if not evidence.eligible:
            if self.state in (KeyboardState.ARMED_ZERO, KeyboardState.MOVING, KeyboardState.STOPPING):
                return self._fault(evidence.failure_reasons[0])
            if self.state is not KeyboardState.FAULTED:
                self.state = KeyboardState.WAITING_FOR_FEEDBACK
            return self._result(False, evidence.failure_reasons[0])
        if self.state is KeyboardState.WAITING_FOR_FEEDBACK:
            self.state = KeyboardState.ARM_READY
        # Healthy feedback never clears FAULTED or arms the controller.
        return self._result(True, "FEEDBACK_ELIGIBLE")

    def arm(self) -> KeyboardStateResult:
        if self.state is KeyboardState.FAULTED:
            return self._result(False, "FAULT_RESET_REQUIRED")
        if not self.feedback_eligible:
            self.state = KeyboardState.WAITING_FOR_FEEDBACK
            return self._result(False, "FEEDBACK_INELIGIBLE")
        if self.state is not KeyboardState.ARM_READY:
            return self._result(False, "EXPLICIT_ARM_REQUIRES_ARM_READY")
        self.state = KeyboardState.ARMED_ZERO
        self._requires_fresh_command = False
        return self._result(True, "EXPLICIT_ARMED_ZERO", command=_zero_command())

    def acknowledge_fault(self) -> KeyboardStateResult:
        """Explicit local recovery; healthy feedback still requires a new arm."""

        if self.state is not KeyboardState.FAULTED:
            return self._result(False, "NO_FAULT_TO_ACKNOWLEDGE")
        if not self.feedback_eligible:
            return self._result(False, "FEEDBACK_INELIGIBLE")
        self.state = KeyboardState.ARM_READY
        self._requires_fresh_command = False
        return self._result(True, "FAULT_ACKNOWLEDGED_ARM_READY")

    def disarm(self) -> KeyboardStateResult:
        self.state = KeyboardState.DISARMED
        self._last_input_sec = None
        self._last_motion = None
        self._requires_fresh_command = False
        return self._result(True, "EXPLICIT_DISARMED", command=_zero_command())

    def set_command(self, v_mps: float, w_radps: float, now_sec: float) -> KeyboardStateResult:
        if not math.isfinite(now_sec):
            return self._fault("COMMAND_TIMESTAMP_INVALID")
        kinematics = self.kinematics.compute(v_mps, w_radps)
        if not kinematics.valid:
            return self._fault(kinematics.reason.value, kinematics)
        if kinematics.direction.value == "STATIONARY":
            self._last_input_sec = now_sec
            if self.state in (KeyboardState.MOVING, KeyboardState.STOPPING) and self._last_motion is not None:
                self.state = KeyboardState.STOPPING
                return self._result(
                    True,
                    "STOPPING_ZERO_SPEED",
                    kinematics,
                    _zero_command(self._last_motion.gear, self._last_motion.inner_wheel_steering_deg),
                )
            if self.state is KeyboardState.ARMED_ZERO:
                return self._result(True, "ARMED_ZERO", kinematics, _zero_command())
            return self._result(False, "NOT_ARMED", kinematics)

        if not self.feedback_eligible:
            return self._result(False, "FEEDBACK_INELIGIBLE", kinematics)
        if self.state not in (KeyboardState.ARMED_ZERO, KeyboardState.STOPPING):
            if self.state is KeyboardState.FAULTED:
                return self._result(False, "FAULT_RESET_REQUIRED", kinematics)
            return self._result(False, "EXPLICIT_ARM_REQUIRED", kinematics)
        if self.transport_mode is TransportMode.TX_CAPABLE:
            # K3D keeps the general keyboard path unavailable. The only
            # separately preparable first-TX surface is ZeroOnlyFirstTxSession.
            return self._fault("GENERAL_KEYBOARD_TX_BLOCKED_ZERO_ONLY_SESSION_REQUIRED", kinematics)
        if self.transport_mode is not TransportMode.MOCK:
            try:
                self.authority.require_tx()
            except RealCanAuthorityError as exc:
                return self._fault(str(exc), kinematics)
        gear = Gear.D if kinematics.direction.value == "FORWARD" else Gear.R
        command = PhysicalCommandModel(gear, kinematics.speed_magnitude_mps, kinematics.inner_steering_deg)
        errors = ManufacturerCommandValidator.validate(CtrlCommand(gear, command.speed_magnitude_mps, command.inner_wheel_steering_deg, 0))
        if errors:
            return self._fault("MANUFACTURER_ENVELOPE_INVALID", kinematics)
        self._last_input_sec = now_sec
        self._last_motion = command
        self._requires_fresh_command = False
        self.state = KeyboardState.MOVING
        return self._result(True, "MOVING", kinematics, command)

    def tick(self, now_sec: float) -> KeyboardStateResult:
        if not math.isfinite(now_sec):
            return self._fault("COMMAND_TIMESTAMP_INVALID")
        if self.state is KeyboardState.MOVING and (
            self._last_input_sec is None or now_sec - self._last_input_sec > self.keyboard_deadman_sec
        ):
            self._requires_fresh_command = True
            self.state = KeyboardState.STOPPING
            command = _zero_command(
                self._last_motion.gear if self._last_motion else None,
                self._last_motion.inner_wheel_steering_deg if self._last_motion else 0.0,
            )
            return self._result(True, "KEYBOARD_DEADMAN_EXPIRED", command=command)
        if self.state is KeyboardState.MOVING and not self.feedback_eligible:
            return self._fault("FEEDBACK_INELIGIBLE")
        return self._result(True, self.state.value)

    def fail_safe_shutdown(self, *, exception: bool = False) -> KeyboardStateResult:
        self.state = KeyboardState.FAULTED if exception else KeyboardState.DISARMED
        self._last_input_sec = None
        self._last_motion = None
        self._requires_fresh_command = exception
        return self._result(True, "EXCEPTION_FAIL_SAFE" if exception else "SHUTDOWN_DISARMED", command=_zero_command())


@dataclass(frozen=True)
class FrameEmission:
    frame: CanFrame
    timestamp_sec: float
    alive_counter: int


class KeyboardFrameScheduler:
    """Caller-ticked frame synthesis independent of keyboard event frequency."""

    def __init__(self, transport: FrameSink, *, output_hz: float = 100.0, alive_initial: int = 0) -> None:
        if not math.isfinite(output_hz) or output_hz <= 0.0:
            raise ValueError("output_hz must be finite and positive")
        if isinstance(alive_initial, bool) or not isinstance(alive_initial, int) or not 0 <= alive_initial <= 15:
            raise ValueError("alive_initial must be in [0, 15]")
        self.transport = transport
        self.period_sec = 1.0 / output_hz
        self._alive = alive_initial
        self._command: PhysicalCommandModel | None = None
        self._next_due_sec: float | None = None

    @property
    def alive_next(self) -> int:
        return self._alive

    def update(self, command: PhysicalCommandModel | None) -> None:
        self._command = command

    def clear(self) -> None:
        self._command = None

    def emit(self, now_sec: float) -> FrameEmission | None:
        if not math.isfinite(now_sec):
            raise ValueError("frame timestamp must be finite")
        command = self._command
        if command is None or command.gear is None:
            return None
        wire_command = CtrlCommand(
            command.gear,
            command.speed_magnitude_mps,
            command.inner_wheel_steering_deg,
            self._alive,
        )
        errors = ManufacturerCommandValidator.validate(wire_command)
        if errors:
            raise ValueError("invalid physical command: " + ";".join(errors))
        frame = MkminiCanCodec.encode(wire_command)
        alive = self._alive
        self._alive = (self._alive + 1) & 0x0F
        self.transport.send(frame, timestamp=now_sec)
        return FrameEmission(frame, now_sec, alive)

    def emit_due(self, now_sec: float) -> FrameEmission | None:
        """Emit at most one frame for a caller-owned nominal 100 Hz tick."""

        if not math.isfinite(now_sec):
            raise ValueError("frame timestamp must be finite")
        if self._next_due_sec is None:
            self._next_due_sec = now_sec
        # Timer schedules commonly represent 10 ms as a repeating binary
        # float.  A tiny comparison tolerance prevents a nominally-on-time
        # caller tick from being lost to representation noise.
        if now_sec + 1.0e-12 < self._next_due_sec:
            return None
        emission = self.emit(now_sec)
        self._next_due_sec = now_sec + self.period_sec
        return emission
