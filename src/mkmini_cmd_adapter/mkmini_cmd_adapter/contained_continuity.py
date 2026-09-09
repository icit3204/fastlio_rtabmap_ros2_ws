"""Contained-bench-only feedback continuity authority.

    This module does not alter production feedback health.  It encodes the narrow
K4 supervisor decision allowing one isolated missing feedback frame while a
robot is securely wheels-off-ground and every independent safety channel is
healthy.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class ContainedContinuityReason(str, Enum):
    FIRST_SAMPLE = "FIRST_SAMPLE"
    NORMAL_CONTINUITY = "NORMAL_CONTINUITY"
    SINGLE_FRAME_OMISSION_WARNING = "SINGLE_FRAME_OMISSION_WARNING"
    TIMING_COHERENT_OMISSION_WARNING = "TIMING_COHERENT_OMISSION_WARNING"
    STALE_FEEDBACK = "STALE_FEEDBACK"
    MULTI_FRAME_LOSS = "MULTI_FRAME_LOSS"
    OUT_OF_ORDER = "OUT_OF_ORDER"
    ALIVE_SEQUENCE_INCONSISTENT = "ALIVE_SEQUENCE_INCONSISTENT"
    CHECKSUM_INVALID = "CHECKSUM_INVALID"
    CAN_DEGRADED = "CAN_DEGRADED"
    CTRL_FEEDBACK_STALE = "CTRL_FEEDBACK_STALE"
    DIAGNOSTIC_FEEDBACK_STALE = "DIAGNOSTIC_FEEDBACK_STALE"
    OPPOSITE_WHEEL_FEEDBACK_STALE = "OPPOSITE_WHEEL_FEEDBACK_STALE"
    HARD_DIAGNOSTIC = "HARD_DIAGNOSTIC"
    COMMAND_OUTSIDE_CONTAINED_ENVELOPE = "COMMAND_OUTSIDE_CONTAINED_ENVELOPE"
    UNEXPECTED_OVERSPEED = "UNEXPECTED_OVERSPEED"
    CONTRADICTORY_PULSE_SPEED = "CONTRADICTORY_PULSE_SPEED"


@dataclass(frozen=True)
class ContainedContinuityContext:
    can_state: str = "UNKNOWN"
    can_error_count: int = 0
    socket_overflow_count: int = 0
    ctrl_current: bool = False
    diagnostic_current: bool = False
    opposite_wheel_current: bool = False
    hard_diagnostic: bool = False
    command_inside_contained_envelope: bool = False
    unexpected_overspeed: bool = False
    contradictory_pulse_speed: bool = False
    current_frame_age_sec: float = 0.0


@dataclass(frozen=True)
class ContainedContinuityResult:
    accepted: bool
    reason: ContainedContinuityReason
    alive_delta: int | None
    inter_arrival_sec: float | None
    warning: bool = False


@dataclass(frozen=True)
class StationaryJitterResult:
    """Contained-only classification of a one-count stationary bounce."""

    accepted: bool
    reason: str
    max_abs_speed_mps: float
    pulse_excursion: int
    net_pulse_delta: int
    odometer_delta_m: float


@dataclass(frozen=True)
class StationaryDitherResult:
    """Contained-only result for the supervisor-authorized one-count dither."""

    accepted: bool
    reason: str
    speed_mps: float
    pulse_delta: int
    following_pulse_delta: int
    odometer_delta_m: float


@dataclass(frozen=True)
class StationaryBoundaryDitherResult:
    """Contained-only stationary adjacent-count cluster classification."""

    accepted: bool
    reason: str
    pulse_span: int
    odometer_delta_m: float
    speed_max_abs_mps: float


def classify_stationary_one_count_jitter(
    *,
    speed_samples_mps: tuple[float, ...],
    pulse_samples: tuple[int, ...],
    odometer_samples: tuple[float, ...],
    can_state: str,
    can_error_count: int,
    visible_wheel_rotation: bool,
    hard_diagnostic: bool,
) -> StationaryJitterResult:
    """Classify only the exact supervisor-approved zero-net pulse bounce."""

    if not speed_samples_mps or not pulse_samples or not odometer_samples:
        return StationaryJitterResult(False, "INSUFFICIENT_STATIONARY_EVIDENCE", 0.0, 0, 0, 0.0)
    max_speed = max(abs(value) for value in speed_samples_mps)
    excursion = max(pulse_samples) - min(pulse_samples)
    net = pulse_samples[-1] - pulse_samples[0]
    odo_delta = odometer_samples[-1] - odometer_samples[0]
    accepted = (
        excursion <= 1
        and net == 0
        and max_speed <= 0.005
        and odo_delta == 0.0
        and not visible_wheel_rotation
        and can_state == "ERROR-ACTIVE"
        and can_error_count == 0
        and not hard_diagnostic
    )
    return StationaryJitterResult(
        accepted,
        "STATIONARY_ONE_COUNT_JITTER_WARNING" if accepted else "STATIONARY_MOTION_EVIDENCE_HARD_ABORT",
        max_speed,
        excursion,
        net,
        odo_delta,
    )


def classify_stationary_encoder_one_count_dither(
    *,
    speed_mps: float,
    pulse_before: int,
    pulse_current: int,
    pulse_following: int,
    odometer_before: float,
    odometer_current: float,
    ctrl_feedback_stationary: bool,
    opposite_wheel_coherent_motion: bool,
    can_state: str,
    can_error_count: int,
    socket_overflow_count: int,
    visible_wheel_rotation: bool,
    hard_diagnostic: bool,
) -> StationaryDitherResult:
    """Apply K4E's narrow contained one-count dither authority.

    This is an event classifier, not a speed deadband.  The following sample
    is required so a same-direction pulse accumulation cannot be hidden.
    Production/ground eligibility does not use this function.
    """

    pulse_delta = pulse_current - pulse_before
    following_delta = pulse_following - pulse_current
    odo_delta = odometer_current - odometer_before
    accepted = (
        math.isfinite(speed_mps)
        and abs(speed_mps) <= 0.030
        and abs(pulse_delta) == 1
        and abs(following_delta) <= 1
        and (following_delta == 0 or (pulse_delta * following_delta) < 0)
        and odo_delta == 0.0
        and ctrl_feedback_stationary
        and not opposite_wheel_coherent_motion
        and can_state == "ERROR-ACTIVE"
        and can_error_count == 0
        and socket_overflow_count == 0
        and not visible_wheel_rotation
        and not hard_diagnostic
    )
    return StationaryDitherResult(
        accepted,
        "STATIONARY_ENCODER_ONE_COUNT_DITHER_WARNING" if accepted else "STATIONARY_DITHER_HARD_ABORT",
        speed_mps,
        pulse_delta,
        following_delta,
        odo_delta,
    )


def classify_stationary_encoder_boundary_dither(
    *,
    pulse_anchor: int,
    pulse_samples: tuple[int, ...],
    speed_samples_mps: tuple[float, ...],
    odometer_anchor: float,
    odometer_samples: tuple[float, ...],
    opposite_wheel_pulse_samples: tuple[int, ...],
    ctrl_feedback_stationary: bool,
    can_state: str,
    can_error_count: int,
    socket_overflow_count: int,
    visible_wheel_rotation: bool,
    hard_diagnostic: bool,
) -> StationaryBoundaryDitherResult:
    """Classify K4F's bounded stationary encoder boundary chatter.

    Speed magnitude is deliberately not used as an independent rejection
    criterion.  Position must remain in the two-count cluster established at
    stationary entry, with no odometer movement or opposite-wheel progression.
    """

    pulses = (pulse_anchor,) + tuple(pulse_samples)
    odos = (odometer_anchor,) + tuple(odometer_samples)
    span = max(pulses) - min(pulses) if pulses else 0
    odo_delta = odos[-1] - odometer_anchor if odos else 0.0
    speed_max = max((abs(value) for value in speed_samples_mps), default=0.0)
    opposite_span = max(opposite_wheel_pulse_samples) - min(opposite_wheel_pulse_samples) if opposite_wheel_pulse_samples else 0
    opposite_steps = tuple(
        current - previous
        for previous, current in zip(opposite_wheel_pulse_samples, opposite_wheel_pulse_samples[1:])
    )
    opposite_monotonic = bool(opposite_steps) and (
        all(step > 0 for step in opposite_steps) or all(step < 0 for step in opposite_steps)
    )
    accepted = (
        bool(pulse_samples)
        and span <= 1
        and opposite_span <= 1
        and not opposite_monotonic
        and all(math.isfinite(value) for value in speed_samples_mps)
        and odo_delta == 0.0
        and ctrl_feedback_stationary
        and can_state == "ERROR-ACTIVE"
        and can_error_count == 0
        and socket_overflow_count == 0
        and not visible_wheel_rotation
        and not hard_diagnostic
    )
    return StationaryBoundaryDitherResult(
        accepted,
        "STATIONARY_ENCODER_BOUNDARY_DITHER" if accepted else "HARD_ABORT_ACCUMULATED_POSITION",
        span,
        odo_delta,
        speed_max,
    )


class ContainedSingleFrameLossPolicy:
    """Per-channel stateful continuity decision for contained tests only."""

    MAX_SINGLE_OMISSION_INTERVAL_SEC = 0.035
    STALE_INTERVAL_SEC = 0.035
    NOMINAL_PERIOD_SEC = 0.010
    TIMING_COHERENCE_TOLERANCE_SEC = 0.006

    def __init__(self) -> None:
        self._last: dict[str, tuple[int, float]] = {}

    @staticmethod
    def _validate(alive: int, timestamp_sec: float) -> None:
        if isinstance(alive, bool) or not isinstance(alive, int) or not 0 <= alive <= 15:
            raise ValueError("alive must be an integer in [0, 15]")
        if not math.isfinite(timestamp_sec):
            raise ValueError("timestamp_sec must be finite")

    @staticmethod
    def _context_failure(context: ContainedContinuityContext) -> ContainedContinuityReason | None:
        if context.can_state != "ERROR-ACTIVE" or context.can_error_count != 0 or context.socket_overflow_count != 0:
            return ContainedContinuityReason.CAN_DEGRADED
        if not context.ctrl_current:
            return ContainedContinuityReason.CTRL_FEEDBACK_STALE
        if not context.diagnostic_current:
            return ContainedContinuityReason.DIAGNOSTIC_FEEDBACK_STALE
        if not context.opposite_wheel_current:
            return ContainedContinuityReason.OPPOSITE_WHEEL_FEEDBACK_STALE
        if context.hard_diagnostic:
            return ContainedContinuityReason.HARD_DIAGNOSTIC
        if not context.command_inside_contained_envelope:
            return ContainedContinuityReason.COMMAND_OUTSIDE_CONTAINED_ENVELOPE
        if context.unexpected_overspeed:
            return ContainedContinuityReason.UNEXPECTED_OVERSPEED
        if context.contradictory_pulse_speed:
            return ContainedContinuityReason.CONTRADICTORY_PULSE_SPEED
        if not math.isfinite(context.current_frame_age_sec) or context.current_frame_age_sec < 0.0:
            return ContainedContinuityReason.STALE_FEEDBACK
        if context.current_frame_age_sec > ContainedSingleFrameLossPolicy.STALE_INTERVAL_SEC:
            return ContainedContinuityReason.STALE_FEEDBACK
        return None

    def observe(
        self,
        channel: str,
        alive: int,
        timestamp_sec: float,
        *,
        checksum_valid: bool,
        context: ContainedContinuityContext,
    ) -> ContainedContinuityResult:
        if not channel:
            raise ValueError("channel must be non-empty")
        self._validate(alive, timestamp_sec)
        if not checksum_valid:
            return ContainedContinuityResult(False, ContainedContinuityReason.CHECKSUM_INVALID, None, None)
        context_failure = self._context_failure(context)
        if context_failure is not None:
            return ContainedContinuityResult(False, context_failure, None, None)

        previous = self._last.get(channel)
        if previous is None:
            self._last[channel] = (alive, timestamp_sec)
            return ContainedContinuityResult(True, ContainedContinuityReason.FIRST_SAMPLE, None, None)

        previous_alive, previous_timestamp = previous
        interval = timestamp_sec - previous_timestamp
        if interval <= 0.0:
            return ContainedContinuityResult(False, ContainedContinuityReason.OUT_OF_ORDER, None, interval)
        if interval > self.STALE_INTERVAL_SEC:
            return ContainedContinuityResult(False, ContainedContinuityReason.STALE_FEEDBACK, None, interval)

        delta = (alive - previous_alive) & 0x0F
        if delta == 1:
            self._last[channel] = (alive, timestamp_sec)
            return ContainedContinuityResult(True, ContainedContinuityReason.NORMAL_CONTINUITY, delta, interval)
        if delta > 1:
            expected_elapsed = delta * self.NOMINAL_PERIOD_SEC
            if abs(interval - expected_elapsed) > self.TIMING_COHERENCE_TOLERANCE_SEC:
                return ContainedContinuityResult(
                    False, ContainedContinuityReason.ALIVE_SEQUENCE_INCONSISTENT, delta, interval
                )
            self._last[channel] = (alive, timestamp_sec)
            return ContainedContinuityResult(
                True,
                ContainedContinuityReason.TIMING_COHERENT_OMISSION_WARNING,
                delta,
                interval,
                warning=True,
            )
        return ContainedContinuityResult(False, ContainedContinuityReason.OUT_OF_ORDER, delta, interval)


__all__ = [
    "ContainedContinuityContext",
    "ContainedContinuityReason",
    "ContainedContinuityResult",
    "ContainedSingleFrameLossPolicy",
    "StationaryJitterResult",
    "StationaryDitherResult",
    "StationaryBoundaryDitherResult",
    "classify_stationary_one_count_jitter",
    "classify_stationary_encoder_one_count_dither",
    "classify_stationary_encoder_boundary_dither",
]
