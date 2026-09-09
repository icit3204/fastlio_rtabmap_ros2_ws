"""Pure MK-mini Ackermann kinematics.

Inputs are body-frame longitudinal velocity and body-frame yaw rate. This
module deliberately does not choose a stationary CAN gear and does not know
about ROS, CAN, or transport bytes.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


WHEELBASE_M = 0.600
TRACK_WIDTH_M = 0.518
MANUFACTURER_STEERING_LIMIT_DEG = 34.0
NUMERICAL_EPSILON = 1.0e-12


class Direction(str, Enum):
    FORWARD = "FORWARD"
    REVERSE = "REVERSE"
    STATIONARY = "STATIONARY"


class KinematicsReason(str, Enum):
    VALID = "VALID"
    NONFINITE_INPUT = "NONFINITE_INPUT"
    ACKERMANN_SPIN_IN_PLACE_UNSUPPORTED = "ACKERMANN_SPIN_IN_PLACE_UNSUPPORTED"
    GEOMETRIC_DENOMINATOR_INVALID = "GEOMETRIC_DENOMINATOR_INVALID"
    STEERING_LIMIT_EXCEEDED = "STEERING_LIMIT_EXCEEDED"


@dataclass(frozen=True)
class KinematicsConfig:
    wheelbase_m: float = WHEELBASE_M
    track_width_m: float = TRACK_WIDTH_M
    steering_limit_deg: float = MANUFACTURER_STEERING_LIMIT_DEG
    numerical_epsilon: float = NUMERICAL_EPSILON


@dataclass(frozen=True)
class KinematicsResult:
    valid: bool
    reason: KinematicsReason
    requested_v_mps: float
    requested_w_radps: float
    direction: Direction
    speed_magnitude_mps: float
    curvature_1pm: float | None
    signed_rear_radius_m: float | None
    inner_steering_rad: float
    inner_steering_deg: float


def _invalid(v: float, w: float, reason: KinematicsReason) -> KinematicsResult:
    return KinematicsResult(
        valid=False,
        reason=reason,
        requested_v_mps=v,
        requested_w_radps=w,
        direction=Direction.STATIONARY,
        speed_magnitude_mps=0.0,
        curvature_1pm=None,
        signed_rear_radius_m=None,
        inner_steering_rad=0.0,
        inner_steering_deg=0.0,
    )


class MkminiKinematicsCore:
    """Stateless body-twist to physical Ackermann command model."""

    def __init__(self, config: KinematicsConfig | None = None) -> None:
        self.config = config or KinematicsConfig()
        if self.config.wheelbase_m <= 0 or self.config.track_width_m <= 0:
            raise ValueError("wheelbase and track width must be positive")
        if self.config.steering_limit_deg < 0 or self.config.numerical_epsilon <= 0:
            raise ValueError("steering limit must be non-negative and epsilon positive")

    def compute(self, v_mps: float, w_radps: float) -> KinematicsResult:
        if not math.isfinite(v_mps) or not math.isfinite(w_radps):
            return _invalid(v_mps, w_radps, KinematicsReason.NONFINITE_INPUT)

        eps = self.config.numerical_epsilon
        if abs(v_mps) <= eps:
            if abs(w_radps) <= eps:
                return KinematicsResult(
                    valid=True,
                    reason=KinematicsReason.VALID,
                    requested_v_mps=v_mps,
                    requested_w_radps=w_radps,
                    direction=Direction.STATIONARY,
                    speed_magnitude_mps=0.0,
                    curvature_1pm=0.0,
                    signed_rear_radius_m=None,
                    inner_steering_rad=0.0,
                    inner_steering_deg=0.0,
                )
            return _invalid(v_mps, w_radps, KinematicsReason.ACKERMANN_SPIN_IN_PLACE_UNSUPPORTED)

        direction = Direction.FORWARD if v_mps > 0 else Direction.REVERSE
        speed = abs(v_mps)
        if abs(w_radps) <= eps:
            return KinematicsResult(
                valid=True,
                reason=KinematicsReason.VALID,
                requested_v_mps=v_mps,
                requested_w_radps=w_radps,
                direction=direction,
                speed_magnitude_mps=speed,
                curvature_1pm=0.0,
                signed_rear_radius_m=None,
                inner_steering_rad=0.0,
                inner_steering_deg=0.0,
            )

        curvature = w_radps / v_mps
        if abs(curvature) <= eps:
            return KinematicsResult(
                valid=True,
                reason=KinematicsReason.VALID,
                requested_v_mps=v_mps,
                requested_w_radps=w_radps,
                direction=direction,
                speed_magnitude_mps=speed,
                curvature_1pm=curvature,
                signed_rear_radius_m=None,
                inner_steering_rad=0.0,
                inner_steering_deg=0.0,
            )

        abs_radius = abs(1.0 / curvature)
        denominator = abs_radius - self.config.track_width_m / 2.0
        if denominator <= 0.0:
            return KinematicsResult(
                valid=False,
                reason=KinematicsReason.GEOMETRIC_DENOMINATOR_INVALID,
                requested_v_mps=v_mps,
                requested_w_radps=w_radps,
                direction=direction,
                speed_magnitude_mps=speed,
                curvature_1pm=curvature,
                signed_rear_radius_m=1.0 / curvature,
                inner_steering_rad=0.0,
                inner_steering_deg=0.0,
            )

        steering_magnitude = math.atan(self.config.wheelbase_m / denominator)
        steering = math.copysign(steering_magnitude, curvature)
        steering_deg = math.degrees(steering)
        valid = abs(steering_deg) <= self.config.steering_limit_deg
        return KinematicsResult(
            valid=valid,
            reason=KinematicsReason.VALID if valid else KinematicsReason.STEERING_LIMIT_EXCEEDED,
            requested_v_mps=v_mps,
            requested_w_radps=w_radps,
            direction=direction,
            speed_magnitude_mps=speed,
            curvature_1pm=curvature,
            signed_rear_radius_m=1.0 / curvature,
            inner_steering_rad=steering,
            inner_steering_deg=steering_deg,
        )

    @staticmethod
    def equivalent_inner_steering_rad(
        curvature_1pm: float,
        wheelbase_m: float = WHEELBASE_M,
        track_width_m: float = TRACK_WIDTH_M,
    ) -> float:
        """Equivalent signed formula, for independent test comparison."""
        denominator = 1.0 - (track_width_m / 2.0) * abs(curvature_1pm)
        if denominator <= 0.0:
            raise ValueError("curvature is outside Ackermann geometric domain")
        return math.atan(wheelbase_m * curvature_1pm / denominator)
