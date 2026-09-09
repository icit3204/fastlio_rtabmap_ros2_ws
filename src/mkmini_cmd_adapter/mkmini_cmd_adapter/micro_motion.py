"""Fail-closed, deterministic wheels-off-ground commissioning preparation.

This module is intentionally narrower than the production commissioning and
keyboard paths.  It contains no CLI and accepts no arbitrary velocity or
steering input.  A caller must inject an already-authorized transport; tests
use only fake transports.  Production motion eligibility is never derived
from the contained-bench result.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol

from .codec import MkminiCanCodec, RunningMode
from .model import CanFrame, CtrlCommand, Gear


class MicroMotionTransport(Protocol):
    def send_frame(self, frame: CanFrame, timestamp: float | None = None) -> object:
        ...


@dataclass(frozen=True)
class PhysicalContainment:
    """Operator-supplied physical prerequisites for the contained tier."""

    all_four_wheels_clear: bool = False
    area_clear: bool = False
    hands_clear: bool = False
    estop_or_poweroff_accessible: bool = False


@dataclass(frozen=True)
class BenchFeedbackSnapshot:
    """Current decoded evidence required before contained actuation."""

    ctrl_current: bool = False
    diagnostic_current: bool = False
    ctrl_checksum_valid: bool = False
    diagnostic_checksum_valid: bool = False
    ctrl_alive_contiguous: bool = False
    diagnostic_alive_contiguous: bool = False
    can_state: str = "UNKNOWN"
    mode: RunningMode | int | None = None
    auto_can_error: bool = False
    auto_io_error: bool = False
    remote_off_warning: bool = False
    remote_receiver_loss: bool = False
    vehicle_fault_level: int = 0
    emergency_stop: bool = False
    eps_fault: bool = False
    left_drive_fault: int = 0
    right_drive_fault: int = 0
    bms_can_loss: bool = False
    unexpected_diagnostic: bool = False


@dataclass(frozen=True)
class ContainedBenchEligibilityResult:
    """Decision for the contained bench tier only.

    ``production_motion_eligible`` is deliberately fixed false.  The two
    authorities cannot be accidentally promoted by copying this result.
    """

    eligible: bool
    hard_abort_reasons: tuple[str, ...] = ()
    baseline_warnings: tuple[str, ...] = ()
    unexpected_warnings: tuple[str, ...] = ()
    production_motion_eligible: bool = False


class ContainedBenchEligibilityPolicy:
    """Evaluate the K3E-known baseline without weakening production health."""

    @staticmethod
    def evaluate(
        containment: PhysicalContainment,
        feedback: BenchFeedbackSnapshot,
        *,
        heartbeat_established: bool = True,
    ) -> ContainedBenchEligibilityResult:
        hard: list[str] = []
        warnings: list[str] = []
        unexpected: list[str] = []

        if not containment.all_four_wheels_clear:
            hard.append("CONTAINMENT_ALL_FOUR_WHEELS_NOT_CLEAR")
        if not containment.area_clear:
            hard.append("CONTAINMENT_AREA_NOT_CLEAR")
        if not containment.hands_clear:
            hard.append("CONTAINMENT_HANDS_NOT_CLEAR")
        if not containment.estop_or_poweroff_accessible:
            hard.append("CONTAINMENT_ESTOP_OR_POWEROFF_NOT_ACCESSIBLE")

        if not heartbeat_established:
            hard.append("COMMAND_HEARTBEAT_NOT_ESTABLISHED")
        if not feedback.ctrl_current:
            hard.append("CTRL_FB_STALE_OR_MISSING")
        if not feedback.diagnostic_current:
            hard.append("VEH_FB_DIAG_STALE_OR_MISSING")
        if not feedback.ctrl_checksum_valid or not feedback.diagnostic_checksum_valid:
            hard.append("FEEDBACK_CHECKSUM_INVALID")
        if not feedback.ctrl_alive_contiguous or not feedback.diagnostic_alive_contiguous:
            hard.append("FEEDBACK_ALIVE_INVALID")
        if feedback.can_state != "ERROR-ACTIVE":
            hard.append("CAN_NOT_ERROR_ACTIVE")
        if feedback.mode != RunningMode.AUTO:
            hard.append("MODE_NOT_AUTO")
        if feedback.auto_can_error:
            hard.append("AUTO_CAN_ERROR")
        if feedback.emergency_stop:
            hard.append("ESTOP_ASSERTED")
        if feedback.eps_fault:
            hard.append("EPS_FAULT")
        if feedback.left_drive_fault != 0:
            hard.append("LEFT_DRIVE_FAULT")
        if feedback.right_drive_fault != 0:
            hard.append("RIGHT_DRIVE_FAULT")
        if feedback.bms_can_loss:
            hard.append("BMS_CAN_LOSS")
        if feedback.remote_receiver_loss:
            hard.append("REMOTE_RECEIVER_LOSS")
        if not isinstance(feedback.vehicle_fault_level, int) or isinstance(feedback.vehicle_fault_level, bool):
            hard.append("VEHICLE_FAULT_LEVEL_UNKNOWN")
        elif feedback.vehicle_fault_level >= 2 or feedback.vehicle_fault_level < 0:
            hard.append("VEHICLE_FAULT_LEVEL_2_OR_HIGHER")

        if feedback.unexpected_diagnostic:
            unexpected.append("UNEXPECTED_DIAGNOSTIC")

        # These are tolerated only as the exact bounded K3E baseline.  A clear
        # value is an improvement and is also accepted; no new warning is.
        if feedback.vehicle_fault_level == 1:
            warnings.append("KNOWN_BASELINE_FAULT_LEVEL_1")
        if feedback.auto_io_error:
            warnings.append("KNOWN_BASELINE_AUTO_IO_ERROR")
        if feedback.remote_off_warning:
            warnings.append("KNOWN_BASELINE_REMOTE_OFF_WARNING")

        return ContainedBenchEligibilityResult(
            eligible=not hard and not unexpected,
            hard_abort_reasons=tuple(dict.fromkeys(hard)),
            baseline_warnings=tuple(warnings),
            unexpected_warnings=tuple(unexpected),
        )


class MicroMotionPhase(str, Enum):
    ZERO_HANDSHAKE = "ZERO_HANDSHAKE"
    FORWARD_PULSE = "FORWARD_PULSE"
    FORWARD_STOPPING = "FORWARD_STOPPING"
    FORWARD_NEUTRAL = "FORWARD_NEUTRAL"
    REVERSE_PULSE = "REVERSE_PULSE"
    REVERSE_STOPPING = "REVERSE_STOPPING"
    REVERSE_NEUTRAL = "REVERSE_NEUTRAL"
    STEER_POSITIVE = "STEER_POSITIVE"
    STEER_POSITIVE_CENTER = "STEER_POSITIVE_CENTER"
    STEER_NEGATIVE = "STEER_NEGATIVE"
    STEER_NEGATIVE_CENTER = "STEER_NEGATIVE_CENTER"
    FINAL_NEUTRAL = "FINAL_NEUTRAL"


@dataclass(frozen=True)
class MicroMotionEmission:
    phase: MicroMotionPhase
    frame: CanFrame
    alive_counter: int
    timestamp_sec: float


class MicroMotionRunner:
    """Fixed K3G1 command sequence; no arbitrary command surface exists."""

    OUTPUT_HZ = 100.0
    PERIOD_SEC = 0.010
    NONZERO_SPEED_MPS = 0.05
    STEERING_ALPHA_DEG = 2.0
    NONZERO_PULSE_SEC = 0.10
    STEERING_PULSE_SEC = 0.10
    MAX_NONZERO_INTERVAL_SEC = 0.20
    STOPPING_MAX_SEC = 2.0
    STEERING_GEAR = Gear.D

    def __init__(self, transport: MicroMotionTransport) -> None:
        self.transport = transport

    @classmethod
    def command_for(cls, phase: MicroMotionPhase) -> CtrlCommand:
        commands = {
            MicroMotionPhase.ZERO_HANDSHAKE: (Gear.N, 0.0, 0.0),
            MicroMotionPhase.FORWARD_PULSE: (Gear.D, cls.NONZERO_SPEED_MPS, 0.0),
            MicroMotionPhase.FORWARD_STOPPING: (Gear.D, 0.0, 0.0),
            MicroMotionPhase.FORWARD_NEUTRAL: (Gear.N, 0.0, 0.0),
            MicroMotionPhase.REVERSE_PULSE: (Gear.R, cls.NONZERO_SPEED_MPS, 0.0),
            MicroMotionPhase.REVERSE_STOPPING: (Gear.R, 0.0, 0.0),
            MicroMotionPhase.REVERSE_NEUTRAL: (Gear.N, 0.0, 0.0),
            MicroMotionPhase.STEER_POSITIVE: (cls.STEERING_GEAR, 0.0, cls.STEERING_ALPHA_DEG),
            MicroMotionPhase.STEER_POSITIVE_CENTER: (cls.STEERING_GEAR, 0.0, 0.0),
            MicroMotionPhase.STEER_NEGATIVE: (cls.STEERING_GEAR, 0.0, -cls.STEERING_ALPHA_DEG),
            MicroMotionPhase.STEER_NEGATIVE_CENTER: (cls.STEERING_GEAR, 0.0, 0.0),
            MicroMotionPhase.FINAL_NEUTRAL: (Gear.N, 0.0, 0.0),
        }
        try:
            gear, speed, steering = commands[phase]
        except KeyError as exc:
            raise ValueError("unknown micro-motion phase") from exc
        return CtrlCommand(gear, speed, steering, 0)

    @classmethod
    def phase_duration_limit(cls, phase: MicroMotionPhase) -> float:
        if phase in (MicroMotionPhase.FORWARD_PULSE, MicroMotionPhase.REVERSE_PULSE):
            return cls.MAX_NONZERO_INTERVAL_SEC
        if phase in (MicroMotionPhase.STEER_POSITIVE, MicroMotionPhase.STEER_NEGATIVE):
            return cls.MAX_NONZERO_INTERVAL_SEC
        if phase in (MicroMotionPhase.FORWARD_STOPPING, MicroMotionPhase.REVERSE_STOPPING):
            return cls.STOPPING_MAX_SEC
        return 1.0

    @classmethod
    def _frame_count(cls, phase: MicroMotionPhase, duration_sec: float) -> int:
        if isinstance(duration_sec, bool) or not math.isfinite(duration_sec) or duration_sec <= 0.0:
            raise ValueError("duration must be finite and positive")
        if duration_sec > cls.phase_duration_limit(phase):
            raise ValueError("phase duration exceeds fixed commissioning limit")
        count = int(math.ceil((duration_sec - 1e-12) / cls.PERIOD_SEC))
        return max(1, count)

    @classmethod
    def synthesize_phase(
        cls,
        phase: MicroMotionPhase,
        duration_sec: float,
        *,
        alive_initial: int = 0,
        timestamp_initial: float = 0.0,
    ) -> tuple[MicroMotionEmission, ...]:
        if not isinstance(phase, MicroMotionPhase):
            raise TypeError("phase must be MicroMotionPhase")
        if isinstance(alive_initial, bool) or not 0 <= alive_initial <= 15:
            raise ValueError("alive_initial must be in [0, 15]")
        if not math.isfinite(timestamp_initial):
            raise ValueError("timestamp_initial must be finite")
        count = cls._frame_count(phase, duration_sec)
        base = cls.command_for(phase)
        emissions: list[MicroMotionEmission] = []
        for index in range(count):
            alive = (alive_initial + index) & 0x0F
            command = CtrlCommand(base.gear, base.speed_magnitude_mps, base.inner_wheel_steering_deg, alive)
            frame = MkminiCanCodec.encode(command)
            emissions.append(MicroMotionEmission(phase, frame, alive, timestamp_initial + index * cls.PERIOD_SEC))
        return tuple(emissions)

    def emit_phase(
        self,
        phase: MicroMotionPhase,
        duration_sec: float,
        *,
        alive_initial: int = 0,
        timestamp_initial: float = 0.0,
    ) -> tuple[MicroMotionEmission, ...]:
        emissions = self.synthesize_phase(
            phase,
            duration_sec,
            alive_initial=alive_initial,
            timestamp_initial=timestamp_initial,
        )
        for emission in emissions:
            self.transport.send_frame(emission.frame, timestamp=emission.timestamp_sec)
        return emissions

    @classmethod
    def fixed_sequence(cls) -> tuple[MicroMotionPhase, ...]:
        """Return the only permitted phase ordering for K3G1 preparation."""

        return (
            MicroMotionPhase.ZERO_HANDSHAKE,
            MicroMotionPhase.FORWARD_PULSE,
            MicroMotionPhase.FORWARD_STOPPING,
            MicroMotionPhase.FORWARD_NEUTRAL,
            MicroMotionPhase.REVERSE_PULSE,
            MicroMotionPhase.REVERSE_STOPPING,
            MicroMotionPhase.REVERSE_NEUTRAL,
            MicroMotionPhase.STEER_POSITIVE,
            MicroMotionPhase.STEER_POSITIVE_CENTER,
            MicroMotionPhase.STEER_NEGATIVE,
            MicroMotionPhase.STEER_NEGATIVE_CENTER,
            MicroMotionPhase.FINAL_NEUTRAL,
        )


__all__ = [
    "BenchFeedbackSnapshot",
    "ContainedBenchEligibilityPolicy",
    "ContainedBenchEligibilityResult",
    "MicroMotionEmission",
    "MicroMotionPhase",
    "MicroMotionRunner",
    "PhysicalContainment",
]
