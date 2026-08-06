from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Mapping


STOP_ARRAY = (0.0, 0.0, 0.0)
SAFE_INPUT_TOPIC = "/vehicle_cmd_safe"
MOCK_OUTPUT_TOPIC = "/wheelchair_control_command_mock"


class AdapterReason(str, Enum):
    STARTUP_ZERO = "STARTUP_ZERO"
    VALID = "VALID"
    INPUT_STALE = "INPUT_STALE"
    INPUT_AUTHORITY_INVALID = "INPUT_AUTHORITY_INVALID"
    OUTPUT_AUTHORITY_INVALID = "OUTPUT_AUTHORITY_INVALID"
    FRAME_INVALID = "FRAME_INVALID"
    NUMERICAL_INVALID = "NUMERICAL_INVALID"
    UNSUPPORTED_AXES = "UNSUPPORTED_AXES"
    REVERSE_UNSUPPORTED = "REVERSE_UNSUPPORTED"
    IN_PLACE_ROTATION_UNSUPPORTED = "IN_PLACE_ROTATION_UNSUPPORTED"
    TURN_RADIUS_UNSUPPORTED = "TURN_RADIUS_UNSUPPORTED"
    OVER_LIMIT = "OVER_LIMIT"


@dataclass(frozen=True)
class AdapterConfig:
    input_topic: str = "/vehicle_cmd_safe"
    output_topic: str = "/wheelchair_control_command_mock"
    expected_frame: str = "base_footprint"
    heartbeat_hz: float = 20.0
    input_timeout_sec: float = 0.25
    max_forward_velocity: float = 0.20
    max_angular_velocity: float = 0.50
    unsupported_axis_epsilon: float = 1e-6
    reverse_epsilon: float = 1e-6
    in_place_linear_epsilon: float = 0.01
    in_place_angular_epsilon: float = 0.02
    minimum_turn_radius_m: float = 1.0
    straight_radius_m: float = 10.0


@dataclass(frozen=True)
class CommandValues:
    frame_id: str = ""
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class AdapterState:
    has_input: bool
    input_age_sec: float | None
    input_publisher_count: int
    output_publisher_count: int


@dataclass(frozen=True)
class ConversionResult:
    valid: bool
    reason: AdapterReason
    output: tuple[float, float, float]
    details: Mapping[str, object] = field(default_factory=dict)


def validate_topic_contract(input_topic: str, output_topic: str) -> None:
    if input_topic != SAFE_INPUT_TOPIC:
        raise ValueError(f"input_topic must be {SAFE_INPUT_TOPIC}")
    if output_topic != MOCK_OUTPUT_TOPIC:
        raise ValueError(f"output_topic must be {MOCK_OUTPUT_TOPIC}")


def zero_result(reason: AdapterReason, details: Mapping[str, object] | None = None) -> ConversionResult:
    return ConversionResult(valid=False, reason=reason, output=STOP_ARRAY, details=dict(details or {}))


def evaluate_command(
    values: CommandValues | None,
    state: AdapterState,
    config: AdapterConfig,
) -> ConversionResult:
    details = {
        "input_age_sec": state.input_age_sec,
        "input_publisher_count": state.input_publisher_count,
        "output_publisher_count": state.output_publisher_count,
        "expected_frame": config.expected_frame,
        "observed_frame": values.frame_id if values is not None else "",
        "computed_radius_m": None,
        "heartbeat_hz": config.heartbeat_hz,
        "input_timeout_sec": config.input_timeout_sec,
        "steady_time": True,
    }

    if state.output_publisher_count != 1:
        return zero_result(AdapterReason.OUTPUT_AUTHORITY_INVALID, details)
    if state.input_publisher_count != 1:
        return zero_result(AdapterReason.INPUT_AUTHORITY_INVALID, details)
    if not state.has_input or values is None:
        return zero_result(AdapterReason.STARTUP_ZERO, details)
    if state.input_age_sec is None or state.input_age_sec > config.input_timeout_sec:
        return zero_result(AdapterReason.INPUT_STALE, details)

    if not values.frame_id or values.frame_id != config.expected_frame:
        return zero_result(AdapterReason.FRAME_INVALID, details)

    fields = (
        values.linear_x,
        values.linear_y,
        values.linear_z,
        values.angular_x,
        values.angular_y,
        values.angular_z,
    )
    if not all(math.isfinite(x) for x in fields):
        return zero_result(AdapterReason.NUMERICAL_INVALID, details)

    if (
        abs(values.linear_y) > config.unsupported_axis_epsilon
        or abs(values.linear_z) > config.unsupported_axis_epsilon
        or abs(values.angular_x) > config.unsupported_axis_epsilon
        or abs(values.angular_y) > config.unsupported_axis_epsilon
    ):
        return zero_result(AdapterReason.UNSUPPORTED_AXES, details)

    v = 0.0 if abs(values.linear_x) <= config.reverse_epsilon and values.linear_x < 0.0 else values.linear_x
    w = values.angular_z

    if v < -config.reverse_epsilon:
        return zero_result(AdapterReason.REVERSE_UNSUPPORTED, details)
    if v > config.max_forward_velocity or abs(w) > config.max_angular_velocity:
        return zero_result(AdapterReason.OVER_LIMIT, details)
    if abs(v) < config.in_place_linear_epsilon and abs(w) > config.in_place_angular_epsilon:
        return zero_result(AdapterReason.IN_PLACE_ROTATION_UNSUPPORTED, details)
    if v == 0.0 and w != 0.0:
        return zero_result(AdapterReason.IN_PLACE_ROTATION_UNSUPPORTED, details)
    if v == 0.0 and w == 0.0:
        return ConversionResult(True, AdapterReason.VALID, STOP_ARRAY, details)
    if v > 0.0 and w == 0.0:
        return ConversionResult(
            True,
            AdapterReason.VALID,
            (config.straight_radius_m * 1000.0, v * 1000.0, 0.0),
            details,
        )

    radius_m = v / w
    details["computed_radius_m"] = radius_m
    abs_radius_m = abs(radius_m)
    if abs_radius_m >= config.straight_radius_m:
        return ConversionResult(
            True,
            AdapterReason.VALID,
            (config.straight_radius_m * 1000.0, v * 1000.0, 0.0),
            details,
        )
    if abs_radius_m >= config.minimum_turn_radius_m:
        return ConversionResult(
            True,
            AdapterReason.VALID,
            (-radius_m * 1000.0, v * 1000.0, 0.0),
            details,
        )
    return zero_result(AdapterReason.TURN_RADIUS_UNSUPPORTED, details)
