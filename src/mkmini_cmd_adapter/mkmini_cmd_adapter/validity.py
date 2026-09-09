"""Pure controller-validity shadow authority for the MK-mini adapter.

This module deliberately contains no ROS, clock, CAN, or transport code. A
caller supplies normalized feedback evidence and explicit ages/timestamps.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple

from .health import HealthAssessment, HealthStatus


class ValidityState(str, Enum):
    INVALID = "INVALID"
    STABILITY_WAIT = "STABILITY_WAIT"
    VALID = "VALID"


class ValidityReason(str, Enum):
    PARAMETERS_UNRESOLVED = "PARAMETERS_UNRESOLVED"
    NO_CTRL_FEEDBACK = "NO_CTRL_FEEDBACK"
    NO_DIAGNOSTIC_FEEDBACK = "NO_DIAGNOSTIC_FEEDBACK"
    CTRL_FEEDBACK_UNKNOWN = "CTRL_FEEDBACK_UNKNOWN"
    DIAGNOSTIC_FEEDBACK_UNKNOWN = "DIAGNOSTIC_FEEDBACK_UNKNOWN"
    CTRL_FEEDBACK_INVALID = "CTRL_FEEDBACK_INVALID"
    DIAGNOSTIC_FEEDBACK_INVALID = "DIAGNOSTIC_FEEDBACK_INVALID"
    CTRL_FEEDBACK_STALE = "CTRL_FEEDBACK_STALE"
    DIAGNOSTIC_FEEDBACK_STALE = "DIAGNOSTIC_FEEDBACK_STALE"
    INVALID_TIMESTAMP = "INVALID_TIMESTAMP"
    RECOVERY_STABILITY_WAIT = "RECOVERY_STABILITY_WAIT"
    HEALTHY = "HEALTHY"


@dataclass(frozen=True)
class ValidityEvidence:
    """Normalized required-source evidence supplied by a feedback provider.

    ``ctrl_health`` represents manufacturer ``ctrl_fb`` evidence and
    ``diagnostic_health`` represents ``Veh_fb_Diag`` evidence. Wheel feedback
    is intentionally not required here; it remains an AdapterCore/standstill
    concern.
    """

    ctrl_health: Optional[HealthAssessment]
    diagnostic_health: Optional[HealthAssessment]
    ctrl_age_sec: Optional[float]
    diagnostic_age_sec: Optional[float]


@dataclass(frozen=True)
class ValidityResult:
    valid: bool
    state: ValidityState
    reason: str
    contributing_reasons: Tuple[str, ...]
    ctrl_age_sec: Optional[float]
    diagnostic_age_sec: Optional[float]
    stable_for_sec: float
    configured: bool


class MkminiControllerValidityCore:
    """Deterministic, fail-closed controller-validity state machine."""

    def __init__(
        self,
        source_timeout_sec: Optional[float],
        recovery_stability_sec: Optional[float],
    ) -> None:
        self.source_timeout_sec = source_timeout_sec
        self.recovery_stability_sec = recovery_stability_sec
        self._healthy_since: Optional[float] = None

    @staticmethod
    def _positive_finite(value: Optional[float]) -> bool:
        return value is not None and math.isfinite(value) and value > 0.0

    @staticmethod
    def _age_ok(age: Optional[float], timeout: float) -> bool:
        return age is not None and math.isfinite(age) and age >= 0.0 and age <= timeout

    @staticmethod
    def _health_reasons(assessment: HealthAssessment) -> Tuple[str, ...]:
        values = []
        for reason in assessment.reasons + assessment.unknown_reasons:
            values.append(reason.value if hasattr(reason, "value") else str(reason))
        return tuple(values) or ("HEALTH_ASSESSMENT_NOT_ELIGIBLE",)

    def evaluate(self, evidence: ValidityEvidence, now_sec: float) -> ValidityResult:
        """Evaluate evidence at caller-supplied monotonic time."""

        ctrl_age = evidence.ctrl_age_sec
        diag_age = evidence.diagnostic_age_sec
        if not math.isfinite(now_sec):
            self._healthy_since = None
            return ValidityResult(
                False, ValidityState.INVALID, ValidityReason.INVALID_TIMESTAMP.value,
                (ValidityReason.INVALID_TIMESTAMP.value,), ctrl_age, diag_age, 0.0, False
            )

        configured = self._positive_finite(self.source_timeout_sec) and self._positive_finite(
            self.recovery_stability_sec
        )
        if not configured:
            self._healthy_since = None
            return ValidityResult(
                False, ValidityState.INVALID, ValidityReason.PARAMETERS_UNRESOLVED.value,
                (ValidityReason.PARAMETERS_UNRESOLVED.value,), ctrl_age, diag_age, 0.0, False
            )

        timeout = float(self.source_timeout_sec)
        reasons = []
        if evidence.ctrl_health is None:
            reasons.append(ValidityReason.NO_CTRL_FEEDBACK.value)
        elif evidence.ctrl_health.status is not HealthStatus.VALID or not evidence.ctrl_health.eligible:
            if evidence.ctrl_health.status is HealthStatus.UNKNOWN:
                reasons.append(ValidityReason.CTRL_FEEDBACK_UNKNOWN.value)
            else:
                reasons.append(ValidityReason.CTRL_FEEDBACK_INVALID.value)
            reasons.extend(self._health_reasons(evidence.ctrl_health))
        if not self._age_ok(ctrl_age, timeout):
            reasons.append(ValidityReason.CTRL_FEEDBACK_STALE.value)

        if evidence.diagnostic_health is None:
            reasons.append(ValidityReason.NO_DIAGNOSTIC_FEEDBACK.value)
        elif (
            evidence.diagnostic_health.status is not HealthStatus.VALID
            or not evidence.diagnostic_health.eligible
        ):
            if evidence.diagnostic_health.status is HealthStatus.UNKNOWN:
                reasons.append(ValidityReason.DIAGNOSTIC_FEEDBACK_UNKNOWN.value)
            else:
                reasons.append(ValidityReason.DIAGNOSTIC_FEEDBACK_INVALID.value)
            reasons.extend(self._health_reasons(evidence.diagnostic_health))
        if not self._age_ok(diag_age, timeout):
            reasons.append(ValidityReason.DIAGNOSTIC_FEEDBACK_STALE.value)

        unique_reasons = tuple(dict.fromkeys(reasons))
        if unique_reasons:
            self._healthy_since = None
            return ValidityResult(
                False, ValidityState.INVALID, unique_reasons[0], unique_reasons,
                ctrl_age, diag_age, 0.0, True
            )

        if self._healthy_since is None:
            self._healthy_since = now_sec
        stable_for = max(0.0, now_sec - self._healthy_since)
        stability = float(self.recovery_stability_sec)
        if stable_for < stability:
            return ValidityResult(
                False, ValidityState.STABILITY_WAIT, ValidityReason.RECOVERY_STABILITY_WAIT.value,
                (ValidityReason.RECOVERY_STABILITY_WAIT.value,), ctrl_age, diag_age,
                stable_for, True
            )
        return ValidityResult(
            True, ValidityState.VALID, ValidityReason.HEALTHY.value,
            (ValidityReason.HEALTHY.value,), ctrl_age, diag_age, stable_for, True
        )

    def reset(self) -> None:
        self._healthy_since = None
