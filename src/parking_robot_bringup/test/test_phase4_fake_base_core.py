import math
import pytest
from parking_robot_bringup.phase2_fake_base_math import Pose2D, Twist2D, integrate_pose, quaternion_from_yaw
from parking_robot_bringup.phase4_fake_base_core import (
    FakeBaseCondition as C, FakeBaseConfig, TwistValues, evaluate_command,
    validate_initial_pose, validate_input_topic,
)

CFG = FakeBaseConfig()
def evaluate(values=TwistValues(0.1, 0, 0, 0, 0, 0.05), frame="base_footprint", count=1, age=0.1):
    return evaluate_command(values, frame, count, age, CFG)

def test_phase2_math_parity_and_straight_motion():
    expected = integrate_pose(Pose2D(0, 0, 0), Twist2D(0.1, 0), 1.0, 2.0)
    actual = integrate_pose(Pose2D(0, 0, 0), evaluate(TwistValues(linear_x=0.1)).applied, 1.0, 2.0)
    assert actual == expected == Pose2D(0.1, 0.0, 0.0)

@pytest.mark.parametrize("w", [0.05, -0.05])
def test_circular_arcs(w):
    pose = integrate_pose(Pose2D(0, 0, 0), evaluate(TwistValues(linear_x=0.1, angular_z=w)).applied, 2, 3)
    assert pose.x == pytest.approx((0.1 / w) * math.sin(2 * w))
    assert pose.y == pytest.approx((0.1 / w) * (1 - math.cos(2 * w)))
    assert pose.yaw == pytest.approx(2 * w)

def test_zero_and_dt_cap_and_yaw_normalization():
    assert integrate_pose(Pose2D(1, 2, 0), Twist2D(0, 0), 1, .1) == Pose2D(1, 2, 0)
    assert integrate_pose(Pose2D(0, 0, 0), Twist2D(1, 0), 5, .1).x == pytest.approx(.1)
    assert -math.pi <= integrate_pose(Pose2D(0, 0, 3.13), Twist2D(0, 1), .1, .1).yaw < math.pi

def test_startup_and_freshness_boundary():
    assert evaluate(None, age=None).condition == C.STARTUP_ZERO
    assert evaluate(age=.5).condition == C.VALID
    assert evaluate(age=.5000001).condition == C.INPUT_STALE

@pytest.mark.parametrize("frame", ["map", ""])
def test_invalid_frame(frame):
    result = evaluate(frame=frame)
    assert result.condition == C.FRAME_INVALID and result.applied == Twist2D(0, 0)

@pytest.mark.parametrize("field", TwistValues.__dataclass_fields__)
@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_every_field_rejects_every_nonfinite(field, value):
    values = TwistValues(**{field: value})
    result = evaluate(values)
    assert result.condition == C.NUMERICAL_INVALID and result.applied == Twist2D(0, 0)

@pytest.mark.parametrize("field", ["linear_y", "linear_z", "angular_x", "angular_y"])
def test_unsupported_axes(field):
    result = evaluate(TwistValues(**{field: 1.1e-6}))
    assert result.condition == C.UNSUPPORTED_AXES and result.applied == Twist2D(0, 0)

@pytest.mark.parametrize("count,condition", [(0, C.INPUT_AUTHORITY_INVALID), (1, C.VALID), (2, C.INPUT_AUTHORITY_INVALID)])
def test_authority_matrix(count, condition):
    result = evaluate(count=count)
    assert result.condition == condition
    if count != 1:
        assert result.applied == Twist2D(0, 0)

def test_duplicate_to_unique_recovery():
    assert evaluate(count=2).applied == Twist2D(0, 0)
    assert evaluate(count=1).applied == Twist2D(.1, .05)

def test_every_invalid_state_applies_zero():
    invalid = [evaluate(None, age=None), evaluate(age=.6), evaluate(count=0), evaluate(frame="map"),
               evaluate(TwistValues(linear_x=float("nan"))), evaluate(TwistValues(linear_y=.1))]
    assert all(result.applied == Twist2D(0, 0) for result in invalid)

def test_initial_pose_validation_and_yaw():
    q = quaternion_from_yaw(1.2)
    assert validate_initial_pose((1, 2, 0), q, "odom") == pytest.approx((1, 2, 1.2))
    for bad in [(float("nan"), 2, 0), (1, float("inf"), 0)]:
        with pytest.raises(ValueError): validate_initial_pose(bad, q, "odom")
    with pytest.raises(ValueError): validate_initial_pose((1, 2, 0), (0, 0, .1, .1), "odom")
    with pytest.raises(ValueError): validate_initial_pose((1, 2, 0), (.01, 0, 0, math.sqrt(.9999)), "odom")
    with pytest.raises(ValueError): validate_initial_pose((1, 2, 0), q, "map")

def test_frozen_input_topic():
    validate_input_topic("/vehicle_cmd_safe")
    with pytest.raises(ValueError): validate_input_topic("/cmd_vel")
