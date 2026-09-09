"""Pure MK-mini adapter state machine.

This module owns no clock, ROS object, CAN transport, or controller authority.
It converts already-authorized body v,w commands into physical command models
and makes zero/stopping policy explicit without selecting a production
stationary gear.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .codec import ManufacturerCommandValidator, MkminiFeedbackCodec
from .health import HealthAssessment, HealthStatus
from .kinematics import Direction, KinematicsReason, KinematicsResult, MkminiKinematicsCore
from .model import CtrlCommand, Gear


class AdapterState(str, Enum):
    WAITING_FOR_FEEDBACK = "WAITING_FOR_FEEDBACK"
    STATIONARY_UNCONFIRMED = "STATIONARY_UNCONFIRMED"
    STATIONARY_HOLD = "STATIONARY_HOLD"
    MOVING_FORWARD = "MOVING_FORWARD"
    MOVING_REVERSE = "MOVING_REVERSE"
    STOPPING_FORWARD = "STOPPING_FORWARD"
    STOPPING_REVERSE = "STOPPING_REVERSE"
    FAULTED = "FAULTED"


class StationaryGearPolicy(str, Enum):
    UNRESOLVED = "UNRESOLVED"
    P = "P"
    N = "N"
    DISABLE = "DISABLE"
    HOLD_LAST_DIRECTION = "HOLD_LAST_DIRECTION"


class AdapterReason(str, Enum):
    WAITING_FOR_FEEDBACK = "WAITING_FOR_FEEDBACK"
    STATIONARY_UNCONFIRMED = "STATIONARY_UNCONFIRMED"
    STATIONARY_POLICY_UNRESOLVED = "STATIONARY_POLICY_UNRESOLVED"
    STATIONARY_HOLD = "STATIONARY_HOLD"
    MOVING_FORWARD = "MOVING_FORWARD"
    MOVING_REVERSE = "MOVING_REVERSE"
    STOPPING_FORWARD = "STOPPING_FORWARD"
    STOPPING_REVERSE = "STOPPING_REVERSE"
    KINEMATICS_INVALID = "KINEMATICS_INVALID"
    FEEDBACK_UNHEALTHY = "FEEDBACK_UNHEALTHY"
    GATE_CONTEXT_MISSING = "GATE_CONTEXT_MISSING"
    GATE_NOT_ARMED = "GATE_NOT_ARMED"
    MANUFACTURER_ENVELOPE_INVALID = "MANUFACTURER_ENVELOPE_INVALID"
    INPUT_DEADMAN_EXPIRED = "INPUT_DEADMAN_EXPIRED"
    RECOVERY_REQUIRES_FRESH_COMMAND = "RECOVERY_REQUIRES_FRESH_COMMAND"
    STANDSTILL_EVIDENCE_UNAVAILABLE = "STANDSTILL_EVIDENCE_UNAVAILABLE"
    STANDSTILL_STABILITY_PENDING = "STANDSTILL_STABILITY_PENDING"


@dataclass(frozen=True)
class AdapterConfig:
    """None means deliberately not selected as a production policy."""

    standstill_speed_threshold_mps: float | None = None
    standstill_stability_sec: float | None = None
    input_timeout_sec: float | None = None
    stationary_gear_policy: StationaryGearPolicy = StationaryGearPolicy.UNRESOLVED

    def __post_init__(self) -> None:
        for name in ("standstill_speed_threshold_mps", "standstill_stability_sec", "input_timeout_sec"):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite and non-negative when configured")
        if self.standstill_speed_threshold_mps is not None and self.standstill_speed_threshold_mps == 0.0:
            raise ValueError("standstill threshold must be positive when configured")
        if self.standstill_stability_sec is not None and self.standstill_stability_sec == 0.0:
            raise ValueError("standstill stability must be positive when configured")


@dataclass(frozen=True)
class GateContext:
    """Read-only caller-supplied Gate context; this is not a ROS message."""

    state: str | None
    arm_pending: bool = False
    fault_latched: bool = False
    safe_zero_quiescent: bool = False
    reason: str = ""


@dataclass(frozen=True)
class PhysicalCommandModel:
    gear: Gear | None
    speed_magnitude_mps: float
    inner_wheel_steering_deg: float

    @property
    def is_zero_speed(self) -> bool:
        return self.speed_magnitude_mps == 0.0


@dataclass(frozen=True)
class AdapterResult:
    state: AdapterState
    reason: AdapterReason
    valid: bool
    command: PhysicalCommandModel
    kinematics: KinematicsResult | None
    requires_fresh_command: bool
    standstill_evidence: bool
    standstill_stable_sec: float | None


def _zero(gear: Gear | None = None, steering_deg: float = 0.0) -> PhysicalCommandModel:
    return PhysicalCommandModel(gear=gear, speed_magnitude_mps=0.0, inner_wheel_steering_deg=steering_deg)


class MkminiAdapterCore:
    """Stateful, timestamp-injected adapter policy with no transport side effects."""

    def __init__(self, config: AdapterConfig | None = None, kinematics: MkminiKinematicsCore | None = None) -> None:
        self.config = config or AdapterConfig()
        self.kinematics = kinematics or MkminiKinematicsCore()
        self.state = AdapterState.WAITING_FOR_FEEDBACK
        self._last_steering_deg = 0.0
        self._last_motion_gear: Gear | None = None
        self._standstill_since: float | None = None
        self._requires_fresh_command = False

    @property
    def requires_fresh_command(self) -> bool:
        return self._requires_fresh_command

    def step(
        self,
        v_mps: float,
        w_radps: float,
        command_timestamp_sec: float,
        now_sec: float,
        feedback_health: HealthAssessment | None,
        *,
        ctrl_feedback=None,
        left_wheel_feedback=None,
        right_wheel_feedback=None,
        gate: GateContext | None = None,
    ) -> AdapterResult:
        if not math.isfinite(command_timestamp_sec) or not math.isfinite(now_sec):
            return self._fault(AdapterReason.INPUT_DEADMAN_EXPIRED)
        kinematics = self.kinematics.compute(v_mps, w_radps)
        is_zero = abs(v_mps) <= self.kinematics.config.numerical_epsilon and abs(w_radps) <= self.kinematics.config.numerical_epsilon

        if self.config.input_timeout_sec is not None and now_sec - command_timestamp_sec > self.config.input_timeout_sec:
            return self._fault(AdapterReason.INPUT_DEADMAN_EXPIRED, kinematics)

        if is_zero:
            return self._zero_step(now_sec, feedback_health, ctrl_feedback, left_wheel_feedback, right_wheel_feedback, kinematics)

        if not kinematics.valid:
            return self._fault(AdapterReason.KINEMATICS_INVALID, kinematics)
        if feedback_health is None or feedback_health.status is not HealthStatus.VALID or not feedback_health.eligible:
            if self.state in (AdapterState.MOVING_FORWARD, AdapterState.MOVING_REVERSE, AdapterState.STOPPING_FORWARD, AdapterState.STOPPING_REVERSE, AdapterState.FAULTED):
                return self._fault(AdapterReason.FEEDBACK_UNHEALTHY, kinematics)
            self.state = AdapterState.WAITING_FOR_FEEDBACK
            return self._result(AdapterReason.FEEDBACK_UNHEALTHY, False, _zero(), kinematics, False, False, None)
        gate_reason = self._gate_motion_reason(gate)
        if gate_reason is not None:
            if self.state in (AdapterState.MOVING_FORWARD, AdapterState.MOVING_REVERSE, AdapterState.STOPPING_FORWARD, AdapterState.STOPPING_REVERSE, AdapterState.FAULTED):
                return self._fault(gate_reason, kinematics)
            self.state = AdapterState.WAITING_FOR_FEEDBACK
            return self._result(gate_reason, False, _zero(), kinematics, False, False, None)

        gear = Gear.D if kinematics.direction is Direction.FORWARD else Gear.R
        command = PhysicalCommandModel(gear, kinematics.speed_magnitude_mps, kinematics.inner_steering_deg)
        envelope_errors = ManufacturerCommandValidator.validate(
            CtrlCommand(gear, command.speed_magnitude_mps, command.inner_wheel_steering_deg, 0)
        )
        if envelope_errors:
            return self._fault(AdapterReason.MANUFACTURER_ENVELOPE_INVALID, kinematics)
        self._last_motion_gear = gear
        self._last_steering_deg = command.inner_wheel_steering_deg
        self._requires_fresh_command = False
        self.state = AdapterState.MOVING_FORWARD if gear is Gear.D else AdapterState.MOVING_REVERSE
        self._standstill_since = None
        return self._result(
            AdapterReason.MOVING_FORWARD if gear is Gear.D else AdapterReason.MOVING_REVERSE,
            True,
            command,
            kinematics,
            False,
            False,
            None,
        )

    def _zero_step(self, now, health, ctrl_feedback, left_feedback, right_feedback, kinematics):
        if self.state is AdapterState.MOVING_FORWARD:
            self.state = AdapterState.STOPPING_FORWARD
            self._last_motion_gear = Gear.D
            self._standstill_since = None
        elif self.state is AdapterState.MOVING_REVERSE:
            self.state = AdapterState.STOPPING_REVERSE
            self._last_motion_gear = Gear.R
            self._standstill_since = None

        if self.state in (AdapterState.STOPPING_FORWARD, AdapterState.STOPPING_REVERSE):
            evidence = self._standstill_evidence(ctrl_feedback, left_feedback, right_feedback)
            stable_sec = self._update_standstill(now, evidence)
            if self._standstill_ready(stable_sec):
                self.state = AdapterState.STATIONARY_HOLD
                gear = self._stationary_gear()
                reason = AdapterReason.STATIONARY_POLICY_UNRESOLVED if gear is None else AdapterReason.STATIONARY_HOLD
                return self._result(reason, gear is not None, _zero(gear), kinematics, self._requires_fresh_command, evidence, stable_sec)
            reason = AdapterReason.STANDSTILL_STABILITY_PENDING if evidence else AdapterReason.STANDSTILL_EVIDENCE_UNAVAILABLE
            return self._result(
                AdapterReason.STOPPING_FORWARD if self.state is AdapterState.STOPPING_FORWARD else AdapterReason.STOPPING_REVERSE,
                True,
                _zero(self._last_motion_gear, self._last_steering_deg),
                kinematics,
                self._requires_fresh_command,
                evidence,
                stable_sec,
            )

        if self.state is AdapterState.FAULTED:
            return self._result(AdapterReason.RECOVERY_REQUIRES_FRESH_COMMAND, False, _zero(), kinematics, True, False, None)

        if health is None:
            self.state = AdapterState.WAITING_FOR_FEEDBACK
            return self._result(AdapterReason.WAITING_FOR_FEEDBACK, False, _zero(), kinematics, False, False, None)
        if health.status is not HealthStatus.VALID or not health.eligible:
            self.state = AdapterState.STATIONARY_UNCONFIRMED
            return self._result(AdapterReason.FEEDBACK_UNHEALTHY, False, _zero(), kinematics, False, False, None)

        evidence = self._standstill_evidence(ctrl_feedback, left_feedback, right_feedback)
        stable_sec = self._update_standstill(now, evidence)
        if self._standstill_ready(stable_sec):
            self.state = AdapterState.STATIONARY_HOLD
            gear = self._stationary_gear()
            reason = AdapterReason.STATIONARY_POLICY_UNRESOLVED if gear is None else AdapterReason.STATIONARY_HOLD
            return self._result(reason, gear is not None, _zero(gear), kinematics, False, evidence, stable_sec)
        self.state = AdapterState.STATIONARY_UNCONFIRMED
        reason = AdapterReason.STANDSTILL_STABILITY_PENDING if evidence else AdapterReason.STANDSTILL_EVIDENCE_UNAVAILABLE
        return self._result(reason, True, _zero(), kinematics, False, evidence, stable_sec)

    def _standstill_evidence(self, ctrl_feedback, left_feedback, right_feedback) -> bool:
        threshold = self.config.standstill_speed_threshold_mps
        if threshold is None:
            return False
        speeds = []
        if ctrl_feedback is not None:
            speeds.append(abs(ctrl_feedback.speed_magnitude_mps))
        if left_feedback is not None:
            speeds.append(abs(left_feedback.speed_mps))
        if right_feedback is not None:
            speeds.append(abs(right_feedback.speed_mps))
        return bool(speeds) and all(speed <= threshold for speed in speeds)

    def _update_standstill(self, now: float, evidence: bool) -> float | None:
        if not evidence:
            self._standstill_since = None
            return None
        if self._standstill_since is None:
            self._standstill_since = now
        return max(0.0, now - self._standstill_since)

    def _standstill_ready(self, stable_sec: float | None) -> bool:
        return stable_sec is not None and self.config.standstill_stability_sec is not None and stable_sec >= self.config.standstill_stability_sec

    def _stationary_gear(self) -> Gear | None:
        policy = self.config.stationary_gear_policy
        return {
            StationaryGearPolicy.P: Gear.P,
            StationaryGearPolicy.N: Gear.N,
            StationaryGearPolicy.DISABLE: Gear.DISABLE,
            StationaryGearPolicy.HOLD_LAST_DIRECTION: self._last_motion_gear,
        }.get(policy)

    @staticmethod
    def _gate_motion_reason(gate: GateContext | None) -> AdapterReason | None:
        if gate is None or gate.state is None:
            return AdapterReason.GATE_CONTEXT_MISSING
        if gate.state != "ARMED":
            return AdapterReason.GATE_NOT_ARMED
        if gate.arm_pending or gate.fault_latched or gate.safe_zero_quiescent:
            return AdapterReason.GATE_NOT_ARMED
        return None

    def _fault(self, reason: AdapterReason, kinematics: KinematicsResult | None = None) -> AdapterResult:
        self.state = AdapterState.FAULTED
        self._requires_fresh_command = True
        return self._result(reason, False, _zero(), kinematics, True, False, None)

    def _result(self, reason, valid, command, kinematics, requires_fresh, evidence, stable_sec):
        return AdapterResult(self.state, reason, valid, command, kinematics, requires_fresh, evidence, stable_sec)
