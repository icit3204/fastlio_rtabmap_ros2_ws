"""Deterministic core for the Phase 4 generic vehicle command gate."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, Optional


STATE_DISARMED = "DISARMED"
STATE_ARMED = "ARMED"
STATE_FAULT = "FAULT"
MODE_MOCK = "MOCK"


@dataclass(frozen=True)
class Twist6:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def fields(self) -> tuple[float, float, float, float, float, float]:
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )

    def is_zero(self, epsilon: float = 1e-9) -> bool:
        return all(abs(value) <= epsilon for value in self.fields())


ZERO_TWIST = Twist6()


@dataclass
class PermissionSample:
    value: bool = False
    stamp: Optional[float] = None

    def age(self, now: float) -> Optional[float]:
        if self.stamp is None:
            return None
        return max(0.0, now - self.stamp)

    def fresh_true(self, now: float, timeout: float) -> bool:
        age = self.age(now)
        return bool(self.value) and age is not None and age <= timeout


@dataclass
class GateConfig:
    frame_id: str = "base_footprint"
    heartbeat_hz: float = 20.0
    safe_twist_timeout_sec: float = 0.25
    localization_timeout_sec: float = 0.50
    controller_timeout_sec: float = 0.50
    collision_valid_timeout_sec: float = 0.50
    authority_stability_sec: float = 1.0
    max_forward_velocity: float = 0.0
    max_angular_velocity: float = 0.0
    max_linear_increase_rate: float = 0.0
    max_angular_increase_rate: float = 0.0
    candidate_c_max_forward_velocity: float = 0.25
    candidate_c_max_angular_velocity: float = 0.8
    unsupported_axis_epsilon: float = 1e-6
    reverse_epsilon: float = 1e-6
    in_place_linear_epsilon: float = 0.01
    in_place_angular_epsilon: float = 0.02
    max_slew_dt_sec: float = 0.10

    def validation_error(self) -> Optional[str]:
        if self.frame_id != "base_footprint":
            return "INVALID_FRAME"
        values = [
            self.heartbeat_hz,
            self.safe_twist_timeout_sec,
            self.localization_timeout_sec,
            self.controller_timeout_sec,
            self.collision_valid_timeout_sec,
            self.authority_stability_sec,
            self.max_forward_velocity,
            self.max_angular_velocity,
            self.max_linear_increase_rate,
            self.max_angular_increase_rate,
        ]
        if any(not math.isfinite(value) for value in values):
            return "INVALID_LIMIT_CONFIGURATION"
        if self.heartbeat_hz <= 0.0:
            return "INVALID_HEARTBEAT"
        if self.max_forward_velocity <= 0.0:
            return "MISSING_EXPLICIT_LINEAR_LIMIT"
        if self.max_angular_velocity <= 0.0:
            return "MISSING_EXPLICIT_ANGULAR_LIMIT"
        if self.max_linear_increase_rate <= 0.0:
            return "MISSING_EXPLICIT_LINEAR_SLEW_LIMIT"
        if self.max_angular_increase_rate <= 0.0:
            return "MISSING_EXPLICIT_ANGULAR_SLEW_LIMIT"
        if self.max_forward_velocity > self.candidate_c_max_forward_velocity:
            return "LINEAR_LIMIT_EXCEEDS_CANDIDATE_C"
        if self.max_angular_velocity > self.candidate_c_max_angular_velocity:
            return "ANGULAR_LIMIT_EXCEEDS_CANDIDATE_C"
        return None


@dataclass
class AuthoritySnapshot:
    safe_input_publishers: int = 0
    output_publishers: int = 1
    localization_publishers: int = 0
    controller_publishers: int = 0
    collision_valid_publishers: int = 0

    def correct(self) -> bool:
        return (
            self.safe_input_publishers == 1
            and self.output_publishers == 1
            and self.localization_publishers == 1
            and self.controller_publishers == 1
            and self.collision_valid_publishers == 1
        )

    def conflict_reason(self) -> Optional[str]:
        if self.safe_input_publishers != 1:
            return "SAFE_INPUT_AUTHORITY_CONFLICT"
        if self.output_publishers != 1:
            return "OUTPUT_AUTHORITY_CONFLICT"
        if self.localization_publishers != 1:
            return "LOCALIZATION_AUTHORITY_CONFLICT"
        if self.controller_publishers != 1:
            return "CONTROLLER_AUTHORITY_CONFLICT"
        if self.collision_valid_publishers != 1:
            return "COLLISION_VALID_AUTHORITY_CONFLICT"
        return None


@dataclass
class GateStatus:
    state: str
    reason_code: str
    detail: str
    output: Twist6
    fault_latched: bool
    contributing_faults: tuple[str, ...] = ()
    diagnostics: Dict[str, str] = field(default_factory=dict)


class GateCore:
    def __init__(self, config: GateConfig) -> None:
        self.config = config
        self.state = STATE_DISARMED
        self.safe_command = ZERO_TWIST
        self.safe_command_stamp: Optional[float] = None
        self.localization = PermissionSample()
        self.controller = PermissionSample()
        self.collision_valid = PermissionSample()
        self.authority = AuthoritySnapshot()
        self.authority_correct_since: Optional[float] = None
        self.last_output = ZERO_TWIST
        self.last_output_time: Optional[float] = None
        self.fault_reason: Optional[str] = None
        self.contributing_faults: list[str] = []
        self.last_arm_request = "none"

    def set_safe_command(self, command: Twist6, now: float) -> None:
        self.safe_command = command
        self.safe_command_stamp = now

    def set_permission(self, name: str, value: bool, now: float) -> None:
        sample = PermissionSample(value=value, stamp=now)
        if name == "localization":
            self.localization = sample
        elif name == "controller":
            self.controller = sample
        elif name == "collision":
            self.collision_valid = sample
        else:
            raise ValueError(f"unknown permission {name}")

    def set_authority(self, authority: AuthoritySnapshot, now: float) -> None:
        was_correct = self.authority.correct()
        self.authority = authority
        if authority.correct():
            if not was_correct or self.authority_correct_since is None:
                self.authority_correct_since = now
        else:
            self.authority_correct_since = None

    def request_arm(self, enable: bool, now: float) -> tuple[bool, str]:
        self.last_arm_request = "arm=true" if enable else "arm=false"
        if not enable:
            if self.state == STATE_FAULT:
                if self._fault_causes_present(now):
                    return False, self.fault_reason or "FAULT_CAUSE_PRESENT"
            self.state = STATE_DISARMED
            self.fault_reason = None
            self.contributing_faults.clear()
            self.authority_correct_since = now if self.authority.correct() else None
            self.last_output = ZERO_TWIST
            self.last_output_time = now
            return True, "DISARMED_FAULT_CLEARED"

        if self.state == STATE_FAULT:
            return False, self.fault_reason or "FAULT_LATCHED"
        reason = self.first_blocking_reason(now)
        if reason is not None:
            return False, reason
        self.state = STATE_ARMED
        self.last_output_time = now
        return True, "ARMED"

    def first_blocking_reason(self, now: float) -> Optional[str]:
        config_error = self.config.validation_error()
        if config_error is not None:
            return config_error
        command_reason = self._command_invalid_reason(self.safe_command)
        if command_reason is not None:
            return command_reason
        conflict = self.authority.conflict_reason()
        if conflict is not None:
            return conflict
        if self.authority_correct_since is None:
            return "AUTHORITY_NOT_STABLE"
        if now - self.authority_correct_since < self.config.authority_stability_sec:
            return "AUTHORITY_NOT_STABLE"
        if not self.localization.fresh_true(now, self.config.localization_timeout_sec):
            return "LOCALIZATION_PERMISSION_INVALID"
        if not self.controller.fresh_true(now, self.config.controller_timeout_sec):
            return "CONTROLLER_PERMISSION_INVALID"
        if not self.collision_valid.fresh_true(now, self.config.collision_valid_timeout_sec):
            return "COLLISION_MONITOR_VALID_INVALID"
        return None

    def tick(self, now: float) -> GateStatus:
        if self.state == STATE_ARMED:
            for reason in self._armed_fault_reasons(now):
                self._latch_fault(reason)
        output = ZERO_TWIST
        reason = "DISARMED_ZERO"
        detail = "disarmed"
        if self.state == STATE_ARMED:
            output = self._limited_output(now)
            reason = "ARMED_COMMAND"
            detail = "passing validated command"
        elif self.state == STATE_FAULT:
            output = ZERO_TWIST
            reason = self.fault_reason or "FAULT"
            detail = "fault latched"
        self.last_output = output
        self.last_output_time = now
        return self._status(now, output, reason, detail)

    def _latch_fault(self, reason: str) -> None:
        if self.fault_reason is None:
            self.fault_reason = reason
        elif reason not in self.contributing_faults and reason != self.fault_reason:
            self.contributing_faults.append(reason)
        self.state = STATE_FAULT

    def _armed_fault_reasons(self, now: float) -> Iterable[str]:
        config_error = self.config.validation_error()
        if config_error is not None:
            yield config_error
        command_reason = self._command_invalid_reason(self.safe_command)
        if command_reason is not None:
            yield command_reason
        if not self._safe_command_fresh(now):
            yield "SAFE_TWIST_STALE"
        if not self.localization.fresh_true(now, self.config.localization_timeout_sec):
            yield "LOCALIZATION_PERMISSION_INVALID"
        if not self.controller.fresh_true(now, self.config.controller_timeout_sec):
            yield "CONTROLLER_PERMISSION_INVALID"
        if not self.collision_valid.fresh_true(now, self.config.collision_valid_timeout_sec):
            yield "COLLISION_MONITOR_VALID_INVALID"
        conflict = self.authority.conflict_reason()
        if conflict is not None:
            yield conflict

    def _fault_causes_present(self, now: float) -> bool:
        return any(True for _ in self._armed_fault_reasons(now))

    def _safe_command_fresh(self, now: float) -> bool:
        return (
            self.safe_command_stamp is not None
            and now - self.safe_command_stamp <= self.config.safe_twist_timeout_sec
        )

    def _command_invalid_reason(self, command: Twist6) -> Optional[str]:
        if any(not math.isfinite(value) for value in command.fields()):
            return "NONFINITE_COMMAND"
        eps = self.config.unsupported_axis_epsilon
        if (
            abs(command.linear_y) > eps
            or abs(command.linear_z) > eps
            or abs(command.angular_x) > eps
            or abs(command.angular_y) > eps
        ):
            return "UNSUPPORTED_AXIS"
        if command.linear_x < -self.config.reverse_epsilon:
            return "REVERSE_COMMAND"
        if (
            abs(command.linear_x) < self.config.in_place_linear_epsilon
            and abs(command.angular_z) > self.config.in_place_angular_epsilon
        ):
            return "IN_PLACE_ROTATION"
        return None

    def _limited_output(self, now: float) -> Twist6:
        target_linear = min(max(self.safe_command.linear_x, 0.0), self.config.max_forward_velocity)
        target_angular = max(
            -self.config.max_angular_velocity,
            min(self.safe_command.angular_z, self.config.max_angular_velocity),
        )
        if self.last_output_time is None:
            return Twist6(linear_x=target_linear, angular_z=target_angular)
        dt = max(0.0, min(now - self.last_output_time, self.config.max_slew_dt_sec))
        linear = self._slew_component(
            self.last_output.linear_x,
            target_linear,
            self.config.max_linear_increase_rate * dt,
        )
        angular = self._slew_component(
            self.last_output.angular_z,
            target_angular,
            self.config.max_angular_increase_rate * dt,
        )
        return Twist6(linear_x=linear, angular_z=angular)

    @staticmethod
    def _slew_component(previous: float, target: float, max_increase: float) -> float:
        if abs(target) <= abs(previous):
            return target
        sign = 1.0 if target >= 0.0 else -1.0
        allowed = abs(previous) + max(0.0, max_increase)
        return sign * min(abs(target), allowed)

    def _status(self, now: float, output: Twist6, reason: str, detail: str) -> GateStatus:
        authority_stable = 0.0
        if self.authority_correct_since is not None:
            authority_stable = max(0.0, now - self.authority_correct_since)
        diagnostics = {
            "mode": MODE_MOCK,
            "state": self.state,
            "reason_code": reason,
            "detail": detail,
            "safe_twist_age_sec": self._age_text(self.safe_command_stamp, now),
            "localization_age_sec": self._age_text(self.localization.stamp, now),
            "controller_age_sec": self._age_text(self.controller.stamp, now),
            "collision_monitor_valid_age_sec": self._age_text(self.collision_valid.stamp, now),
            "localization_valid": str(self.localization.value).lower(),
            "controller_valid": str(self.controller.value).lower(),
            "collision_monitor_valid": str(self.collision_valid.value).lower(),
            "safe_input_publisher_count": str(self.authority.safe_input_publishers),
            "output_publisher_count": str(self.authority.output_publishers),
            "localization_publisher_count": str(self.authority.localization_publishers),
            "controller_publisher_count": str(self.authority.controller_publishers),
            "collision_valid_publisher_count": str(self.authority.collision_valid_publishers),
            "authority_stable_sec": f"{authority_stable:.6f}",
            "configured_frame": self.config.frame_id,
            "max_forward_velocity": f"{self.config.max_forward_velocity:.6f}",
            "max_angular_velocity": f"{self.config.max_angular_velocity:.6f}",
            "max_linear_increase_rate": f"{self.config.max_linear_increase_rate:.6f}",
            "max_angular_increase_rate": f"{self.config.max_angular_increase_rate:.6f}",
            "last_arm_request": self.last_arm_request,
            "fault_latched": str(self.state == STATE_FAULT).lower(),
        }
        return GateStatus(
            state=self.state,
            reason_code=reason,
            detail=detail,
            output=output,
            fault_latched=self.state == STATE_FAULT,
            contributing_faults=tuple(self.contributing_faults),
            diagnostics=diagnostics,
        )

    @staticmethod
    def _age_text(stamp: Optional[float], now: float) -> str:
        if stamp is None:
            return "none"
        return f"{max(0.0, now - stamp):.6f}"
