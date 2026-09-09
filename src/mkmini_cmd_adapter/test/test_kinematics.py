import math

import pytest

from mkmini_cmd_adapter import (
    CtrlCommand,
    Direction,
    Gear,
    KinematicsReason,
    MkminiCanCodec,
    MkminiKinematicsCore,
    MockTransport,
)


CORE = MkminiKinematicsCore()


def test_zero_and_straight_commands():
    zero = CORE.compute(0.0, 0.0)
    assert zero.valid and zero.reason is KinematicsReason.VALID
    assert zero.direction is Direction.STATIONARY
    assert zero.speed_magnitude_mps == 0.0
    assert zero.inner_steering_deg == 0.0
    assert CORE.compute(0.2, 0.0).direction is Direction.FORWARD
    assert CORE.compute(-0.2, 0.0).direction is Direction.REVERSE
    assert CORE.compute(-0.2, 0.0).speed_magnitude_mps == pytest.approx(0.2)


@pytest.mark.parametrize(
    "v,w,expected_direction,sign",
    [
        (0.2, 0.1, Direction.FORWARD, 1),
        (0.2, -0.1, Direction.FORWARD, -1),
        (-0.2, 0.1, Direction.REVERSE, -1),
        (-0.2, -0.1, Direction.REVERSE, 1),
    ],
)
def test_forward_reverse_steering_sign_contract(v, w, expected_direction, sign):
    result = CORE.compute(v, w)
    assert result.valid
    assert result.direction is expected_direction
    assert math.copysign(1.0, result.inner_steering_deg) == sign


def test_spin_in_place_is_invalid_and_not_silently_zeroed():
    result = CORE.compute(0.0, 0.2)
    assert result.valid is False
    assert result.reason is KinematicsReason.ACKERMANN_SPIN_IN_PLACE_UNSUPPORTED
    assert result.direction is Direction.STATIONARY


def test_small_values_use_only_numerical_epsilon():
    assert CORE.compute(0.2, 0.5e-12).inner_steering_deg == 0.0
    assert CORE.compute(0.2, 1.5e-12).inner_steering_deg != 0.0
    assert CORE.compute(0.5e-12, 0.5e-12).valid
    assert CORE.compute(0.5e-12, 2.0e-12).reason is KinematicsReason.ACKERMANN_SPIN_IN_PLACE_UNSUPPORTED
    assert CORE.compute(1.0e-6, 1.0e-3).reason is not KinematicsReason.ACKERMANN_SPIN_IN_PLACE_UNSUPPORTED


@pytest.mark.parametrize("v,w", [(float("nan"), 0.0), (0.0, float("nan")), (float("inf"), 0.0), (0.0, float("-inf"))])
def test_nonfinite_inputs_invalid(v, w):
    result = CORE.compute(v, w)
    assert result.valid is False
    assert result.reason is KinematicsReason.NONFINITE_INPUT


def test_curvature_and_radius_are_body_twist_values():
    result = CORE.compute(0.2, 0.1)
    assert result.curvature_1pm == pytest.approx(0.5)
    assert result.signed_rear_radius_m == pytest.approx(2.0)
    assert result.speed_magnitude_mps == pytest.approx(0.2)


def test_equivalent_inner_angle_formulations_agree():
    for v, w in ((0.2, 0.1), (0.4, 0.2), (0.2, -0.1), (-0.2, 0.1), (1.0, 0.01), (0.1, 0.05)):
        result = CORE.compute(v, w)
        equivalent = CORE.equivalent_inner_steering_rad(result.curvature_1pm)
        assert result.inner_steering_rad == pytest.approx(equivalent, abs=1e-12)


def test_same_curvature_has_same_steering():
    first = CORE.compute(0.2, 0.1)
    second = CORE.compute(0.4, 0.2)
    assert first.curvature_1pm == pytest.approx(second.curvature_1pm)
    assert first.inner_steering_deg == pytest.approx(second.inner_steering_deg)


def test_forward_reverse_symmetry():
    first = CORE.compute(0.2, 0.1)
    same_curvature = CORE.compute(-0.2, -0.1)
    opposite_curvature = CORE.compute(-0.2, 0.1)
    assert same_curvature.curvature_1pm == pytest.approx(first.curvature_1pm)
    assert same_curvature.inner_steering_deg == pytest.approx(first.inner_steering_deg)
    assert same_curvature.direction is Direction.REVERSE
    assert same_curvature.speed_magnitude_mps == pytest.approx(first.speed_magnitude_mps)
    assert opposite_curvature.curvature_1pm == pytest.approx(-first.curvature_1pm)
    assert opposite_curvature.inner_steering_deg == pytest.approx(-first.inner_steering_deg)


def test_manufacturer_steering_boundaries_and_over_limit():
    boundary_radius = 0.600 / math.tan(math.radians(34.0)) + 0.518 / 2.0
    at_positive = CORE.compute(1.0, 1.0 / boundary_radius)
    at_negative = CORE.compute(1.0, -1.0 / boundary_radius)
    assert at_positive.valid and at_positive.inner_steering_deg == pytest.approx(34.0)
    assert at_negative.valid and at_negative.inner_steering_deg == pytest.approx(-34.0)
    beyond = CORE.compute(1.0, 1.0 / (boundary_radius - 0.001))
    assert beyond.valid is False
    assert beyond.reason is KinematicsReason.STEERING_LIMIT_EXCEEDED
    assert beyond.inner_steering_deg > 34.0


def test_geometric_denominator_invalid():
    result = CORE.compute(1.0, 10.0)
    assert result.valid is False
    assert result.reason is KinematicsReason.GEOMETRIC_DENOMINATOR_INVALID


def test_manufacturer_minimum_radius_consistency():
    r_rear = 0.600 / math.tan(math.radians(34.0)) + 0.518 / 2.0
    outside_front = math.hypot(0.600, r_rear + 0.518 / 2.0)
    assert r_rear == pytest.approx(1.149, abs=0.002)
    assert outside_front == pytest.approx(1.53, abs=0.01)


def test_nonzero_kinematics_to_codec_to_mock_transport_composition():
    result = CORE.compute(0.2, 0.1)
    assert result.valid
    gear = Gear.D if result.direction is Direction.FORWARD else Gear.R
    frame = MkminiCanCodec.encode(CtrlCommand(gear, result.speed_magnitude_mps, result.inner_steering_deg, 1))
    transport = MockTransport()
    transport.send(frame, timestamp=123.0)
    assert transport.count == 1
    assert transport.latest().frame.data == bytes.fromhex("84 0C E0 76 00 00 10 0E")


def test_stationary_composition_refuses_undefined_gear_policy():
    result = CORE.compute(0.0, 0.0)
    assert result.direction is Direction.STATIONARY

    def choose_moving_gear(kinematic_result):
        if kinematic_result.direction is Direction.STATIONARY:
            raise ValueError("stationary gear policy is deferred")
        return Gear.D if kinematic_result.direction is Direction.FORWARD else Gear.R

    with pytest.raises(ValueError, match="stationary gear policy"):
        choose_moving_gear(result)
