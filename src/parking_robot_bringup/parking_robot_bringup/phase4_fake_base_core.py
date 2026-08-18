"""Pure validation for the Phase 4 generic fake-motion sink."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
from parking_robot_bringup.phase2_fake_base_math import Twist2D, yaw_from_quaternion

class FakeBaseCondition(str, Enum):
    STARTUP_ZERO = "STARTUP_ZERO"
    VALID = "VALID"
    INPUT_STALE = "INPUT_STALE"
    INPUT_AUTHORITY_INVALID = "INPUT_AUTHORITY_INVALID"
    FRAME_INVALID = "FRAME_INVALID"
    NUMERICAL_INVALID = "NUMERICAL_INVALID"
    UNSUPPORTED_AXES = "UNSUPPORTED_AXES"

@dataclass(frozen=True)
class FakeBaseConfig:
    expected_frame: str = "base_footprint"
    command_timeout_sec: float = 0.50
    unsupported_axis_epsilon: float = 1.0e-6

@dataclass(frozen=True)
class TwistValues:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

@dataclass(frozen=True)
class ValidationResult:
    applied: Twist2D
    valid: bool
    condition: FakeBaseCondition
    reason: str
    details: dict[str, object]

def validate_config(config: FakeBaseConfig) -> None:
    if not config.expected_frame:
        raise ValueError("expected_frame must not be empty")
    if not math.isfinite(config.command_timeout_sec) or config.command_timeout_sec < 0.0:
        raise ValueError("command_timeout_sec must be finite and non-negative")
    if not math.isfinite(config.unsupported_axis_epsilon) or config.unsupported_axis_epsilon < 0.0:
        raise ValueError("unsupported_axis_epsilon must be finite and non-negative")

def validate_input_topic(topic: str) -> None:
    if topic != "/vehicle_cmd_safe":
        raise ValueError("input_topic is frozen to /vehicle_cmd_safe")

def evaluate_command(values: TwistValues | None, observed_frame: str, publisher_count: int,
                     receipt_age_sec: float | None, config: FakeBaseConfig) -> ValidationResult:
    """Return the command safe to integrate, without ROS or clock access."""
    validate_config(config)
    details = {"command_age_sec": receipt_age_sec, "command_publisher_count": publisher_count,
               "expected_frame": config.expected_frame, "observed_frame": observed_frame}
    def reject(condition: FakeBaseCondition, reason: str) -> ValidationResult:
        return ValidationResult(Twist2D(0.0, 0.0), False, condition, reason, details)
    if publisher_count != 1:
        return reject(FakeBaseCondition.INPUT_AUTHORITY_INVALID, "exactly one input publisher is required")
    if values is None or receipt_age_sec is None:
        return reject(FakeBaseCondition.STARTUP_ZERO, "no command has been received")
    if not math.isfinite(receipt_age_sec) or receipt_age_sec < 0.0:
        return reject(FakeBaseCondition.NUMERICAL_INVALID, "command receipt age is invalid")
    if receipt_age_sec > config.command_timeout_sec:
        return reject(FakeBaseCondition.INPUT_STALE, "command receipt age exceeds timeout")
    if observed_frame != config.expected_frame:
        return reject(FakeBaseCondition.FRAME_INVALID, "command frame does not match expected frame")
    fields = (values.linear_x, values.linear_y, values.linear_z, values.angular_x,
              values.angular_y, values.angular_z)
    if not all(math.isfinite(value) for value in fields):
        return reject(FakeBaseCondition.NUMERICAL_INVALID, "Twist contains a non-finite value")
    unsupported = (values.linear_y, values.linear_z, values.angular_x, values.angular_y)
    if any(abs(value) > config.unsupported_axis_epsilon for value in unsupported):
        return reject(FakeBaseCondition.UNSUPPORTED_AXES, "unsupported Twist axis exceeds epsilon")
    return ValidationResult(Twist2D(values.linear_x, values.angular_z), True,
                            FakeBaseCondition.VALID, "fresh valid unique-authority input", details)

def validate_initial_pose(position: tuple[float, float, float],
                          quaternion: tuple[float, float, float, float], frame_id: str,
                          planar_epsilon: float = 1.0e-6,
                          normalization_tolerance: float = 1.0e-6) -> tuple[float, float, float]:
    """Validate an odom-frame planar pose and return x, y, yaw."""
    if frame_id != "odom":
        raise ValueError("initial pose frame must be odom")
    if not all(math.isfinite(value) for value in position + quaternion):
        raise ValueError("initial pose must be finite")
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1.0) > normalization_tolerance:
        raise ValueError("initial pose quaternion must be normalized")
    if abs(qx) > planar_epsilon or abs(qy) > planar_epsilon:
        raise ValueError("initial pose quaternion must represent planar yaw")
    return float(position[0]), float(position[1]), yaw_from_quaternion(qx, qy, qz, qw)
