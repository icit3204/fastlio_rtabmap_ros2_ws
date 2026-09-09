"""Pure, timestamp-injected feedback health primitives.

These classes do not read a clock, publish ROS state, or create a controller
authority. Unknown manufacturer semantics remain non-eligible.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .codec import RunningMode, VehicleDiagnosticFeedback


class HealthStatus(str, Enum):
    VALID = "VALID"
    INVALID = "INVALID"
    UNKNOWN = "UNKNOWN"


class HealthReason(str, Enum):
    NO_FEEDBACK_SAMPLE = "NO_FEEDBACK_SAMPLE"
    FEEDBACK_CHECKSUM_INVALID = "FEEDBACK_CHECKSUM_INVALID"
    FEEDBACK_ALIVE_DISCONTINUITY = "FEEDBACK_ALIVE_DISCONTINUITY"
    FEEDBACK_STALE = "FEEDBACK_STALE"
    FEEDBACK_TIMESTAMP_INVALID = "FEEDBACK_TIMESTAMP_INVALID"
    FEEDBACK_TIMESTAMP_REVERSAL = "FEEDBACK_TIMESTAMP_REVERSAL"
    MODE_REMOTE = "MODE_REMOTE"
    MODE_STOP = "MODE_STOP"
    MODE_UNKNOWN = "MODE_UNKNOWN"
    ESTOP_ASSERTED = "ESTOP_ASSERTED"
    AUTO_CAN_FAULT = "AUTO_CAN_FAULT"
    AUTO_IO_CAN_FAULT = "AUTO_IO_CAN_FAULT"
    EPS_FAULT = "EPS_FAULT"
    LEFT_DRIVE_FAULT = "LEFT_DRIVE_FAULT"
    RIGHT_DRIVE_FAULT = "RIGHT_DRIVE_FAULT"
    BMS_CAN_FAULT = "BMS_CAN_FAULT"
    REMOTE_RECEIVER_FAULT = "REMOTE_RECEIVER_FAULT"
    REMOTE_OFF_WARNING = "REMOTE_OFF_WARNING"
    VEHICLE_FAULT_LEVEL = "VEHICLE_FAULT_LEVEL"
    VEHICLE_FAULT_LEVEL_UNKNOWN = "VEHICLE_FAULT_LEVEL_UNKNOWN"


@dataclass(frozen=True)
class HealthAssessment:
    status: HealthStatus
    eligible: bool
    reasons: tuple[HealthReason, ...] = ()
    unknown_reasons: tuple[HealthReason, ...] = ()


class FeedbackFreshnessState:
    """Receipt-time state machine; callers supply all timestamps and timeout."""

    def __init__(self) -> None:
        self.last_timestamp_sec: float | None = None
        self.timestamp_reversal = False

    def observe(self, timestamp_sec: float) -> None:
        if not math.isfinite(timestamp_sec):
            raise ValueError("feedback timestamp must be finite")
        if self.last_timestamp_sec is not None and timestamp_sec < self.last_timestamp_sec:
            self.timestamp_reversal = True
            return
        self.last_timestamp_sec = timestamp_sec

    def assess(self, now_sec: float, timeout_sec: float) -> HealthAssessment:
        if not math.isfinite(now_sec) or not math.isfinite(timeout_sec) or timeout_sec < 0:
            return HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.FEEDBACK_TIMESTAMP_INVALID,))
        if self.timestamp_reversal:
            return HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.FEEDBACK_TIMESTAMP_REVERSAL,))
        if self.last_timestamp_sec is None:
            return HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.NO_FEEDBACK_SAMPLE,))
        age = now_sec - self.last_timestamp_sec
        if age < 0:
            return HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.FEEDBACK_TIMESTAMP_INVALID,))
        if age > timeout_sec:
            return HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.FEEDBACK_STALE,))
        return HealthAssessment(HealthStatus.VALID, True)


def assess_mode(mode: RunningMode | int) -> HealthAssessment:
    if mode == RunningMode.AUTO:
        return HealthAssessment(HealthStatus.VALID, True)
    if mode == RunningMode.REMOTE:
        return HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.MODE_REMOTE,))
    if mode == RunningMode.STOP:
        return HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.MODE_STOP,))
    return HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.MODE_UNKNOWN,))


class DiagnosticState:
    """Maps only explicit manufacturer diagnostic semantics to reasons."""

    @staticmethod
    def assess(feedback: VehicleDiagnosticFeedback) -> HealthAssessment:
        reasons: list[HealthReason] = []
        unknown: list[HealthReason] = []
        if feedback.vehicle_fault_level not in (0, 1, 2, 3):
            unknown.append(HealthReason.VEHICLE_FAULT_LEVEL_UNKNOWN)
        elif feedback.vehicle_fault_level != 0:
            reasons.append(HealthReason.VEHICLE_FAULT_LEVEL)
        if feedback.auto_can_communication_error:
            reasons.append(HealthReason.AUTO_CAN_FAULT)
        if feedback.auto_io_can_communication_error:
            reasons.append(HealthReason.AUTO_IO_CAN_FAULT)
        if feedback.eps_fault_code != 0:
            reasons.append(HealthReason.EPS_FAULT)
        if feedback.left_drive_fault != 0:
            reasons.append(HealthReason.LEFT_DRIVE_FAULT)
        if feedback.right_drive_fault != 0:
            reasons.append(HealthReason.RIGHT_DRIVE_FAULT)
        if feedback.bms_can_communication_loss:
            reasons.append(HealthReason.BMS_CAN_FAULT)
        if feedback.emergency_stop_asserted:
            reasons.append(HealthReason.ESTOP_ASSERTED)
        if feedback.remote_receiver_loss:
            reasons.append(HealthReason.REMOTE_RECEIVER_FAULT)
        if feedback.remote_off_warning:
            reasons.append(HealthReason.REMOTE_OFF_WARNING)
        status = HealthStatus.INVALID if reasons else HealthStatus.UNKNOWN if unknown else HealthStatus.VALID
        return HealthAssessment(status, status is HealthStatus.VALID, tuple(reasons), tuple(unknown))


def assess_feedback_health(
    *,
    checksum_valid: bool,
    alive_contiguous: bool,
    freshness: HealthAssessment,
    mode: HealthAssessment,
    diagnostics: HealthAssessment,
) -> HealthAssessment:
    """Combine explicit receive-side evidence without creating controller_valid."""

    reasons: list[HealthReason] = []
    unknown: list[HealthReason] = []
    if not checksum_valid:
        reasons.append(HealthReason.FEEDBACK_CHECKSUM_INVALID)
    if not alive_contiguous:
        reasons.append(HealthReason.FEEDBACK_ALIVE_DISCONTINUITY)
    reasons.extend(freshness.reasons)
    reasons.extend(mode.reasons)
    reasons.extend(diagnostics.reasons)
    unknown.extend(freshness.unknown_reasons)
    unknown.extend(mode.unknown_reasons)
    unknown.extend(diagnostics.unknown_reasons)
    if reasons:
        return HealthAssessment(HealthStatus.INVALID, False, tuple(dict.fromkeys(reasons)), tuple(dict.fromkeys(unknown)))
    if unknown:
        return HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=tuple(dict.fromkeys(unknown)))
    return HealthAssessment(HealthStatus.VALID, True)
