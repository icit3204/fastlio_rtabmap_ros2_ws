from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


ZERO_COMMAND = (0.0, 0.0, 0.0)


class BridgeReason(str, Enum):
    VALID = "VALID"
    STARTUP = "STARTUP"
    STALE = "STALE"
    INVALID_TIMESTAMP = "INVALID_TIMESTAMP"
    WRONG_FRAME = "WRONG_FRAME"
    NONFINITE = "NONFINITE"
    UNSUPPORTED_AXIS = "UNSUPPORTED_AXIS"
    IN_PLACE_ROTATION = "IN_PLACE_ROTATION"
    BELOW_ZERO_THRESHOLD = "BELOW_ZERO_THRESHOLD"
    VELOCITY_LIMIT = "VELOCITY_LIMIT"
    ANGULAR_LIMIT = "ANGULAR_LIMIT"
    RADIUS_LIMIT = "RADIUS_LIMIT"


@dataclass(frozen=True)
class LabmateBridgeConfig:
    frame_id: str = "base_footprint"
    input_timeout_sec: float = 0.25
    future_tolerance_sec: float = 0.10
    zero_velocity_threshold_mps: float = 0.001
    zero_angular_threshold_rps: float = 0.001
    max_forward_velocity_mps: float = 0.25
    max_reverse_velocity_mps: float = 0.25
    max_angular_velocity_rps: float = 0.50
    # Rear-axle radius for 30 deg inner steering: L/tan(30 deg) + T/2.
    minimum_radius_m: float = 1.298230485
    straight_radius_m: float = 10.0
    unsupported_axis_epsilon: float = 1e-6


@dataclass(frozen=True)
class SafeTwist:
    frame_id: str
    stamp_sec: float
    linear_x: float
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class BridgeResult:
    valid: bool
    reason: BridgeReason
    output: tuple[float, float, float]


def zero(reason: BridgeReason) -> BridgeResult:
    return BridgeResult(False, reason, ZERO_COMMAND)


def convert_safe_twist(
    command: SafeTwist | None,
    *,
    now_ros_sec: float,
    receipt_age_sec: float | None,
    config: LabmateBridgeConfig = LabmateBridgeConfig(),
) -> BridgeResult:
    if command is None or receipt_age_sec is None:
        return zero(BridgeReason.STARTUP)
    if not math.isfinite(receipt_age_sec) or receipt_age_sec > config.input_timeout_sec:
        return zero(BridgeReason.STALE)
    if not math.isfinite(command.stamp_sec) or command.stamp_sec <= 0.0:
        return zero(BridgeReason.INVALID_TIMESTAMP)
    stamp_age = now_ros_sec - command.stamp_sec
    if stamp_age > config.input_timeout_sec or stamp_age < -config.future_tolerance_sec:
        return zero(BridgeReason.INVALID_TIMESTAMP)
    if command.frame_id != config.frame_id:
        return zero(BridgeReason.WRONG_FRAME)

    values = (
        command.linear_x,
        command.linear_y,
        command.linear_z,
        command.angular_x,
        command.angular_y,
        command.angular_z,
    )
    if not all(math.isfinite(value) for value in values):
        return zero(BridgeReason.NONFINITE)
    if any(
        abs(value) > config.unsupported_axis_epsilon
        for value in (command.linear_y, command.linear_z, command.angular_x, command.angular_y)
    ):
        return zero(BridgeReason.UNSUPPORTED_AXIS)

    velocity = command.linear_x
    yaw_rate = command.angular_z
    if velocity > config.max_forward_velocity_mps or velocity < -config.max_reverse_velocity_mps:
        return zero(BridgeReason.VELOCITY_LIMIT)
    if abs(yaw_rate) > config.max_angular_velocity_rps:
        return zero(BridgeReason.ANGULAR_LIMIT)
    if abs(velocity) < config.zero_velocity_threshold_mps:
        if abs(yaw_rate) >= config.zero_angular_threshold_rps:
            return zero(BridgeReason.IN_PLACE_ROTATION)
        return zero(BridgeReason.BELOW_ZERO_THRESHOLD)

    if abs(yaw_rate) < config.zero_angular_threshold_rps:
        return BridgeResult(
            True,
            BridgeReason.VALID,
            (config.straight_radius_m * 1000.0, velocity * 1000.0, 0.0),
        )

    # The reference backend negates non-straight radius before Ackermann conversion.
    # Therefore bridge radius = -v/w preserves ROS yaw sign through that backend.
    bridge_radius_m = -velocity / yaw_rate
    if abs(bridge_radius_m) < config.minimum_radius_m:
        return zero(BridgeReason.RADIUS_LIMIT)
    if abs(bridge_radius_m) >= config.straight_radius_m:
        radius_mm = config.straight_radius_m * 1000.0
    else:
        radius_mm = bridge_radius_m * 1000.0
    return BridgeResult(
        True,
        BridgeReason.VALID,
        (radius_mm, velocity * 1000.0, 0.0),
    )
