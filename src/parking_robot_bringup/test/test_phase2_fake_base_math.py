import math

import pytest

from parking_robot_bringup.phase2_fake_base_math import (
    Pose2D,
    Twist2D,
    command_for_integration,
    integrate_pose,
    is_finite_twist,
    normalize_yaw,
    quaternion_from_yaw,
    reset_pose,
    stale_or_missing_command,
    yaw_from_quaternion,
)


def assert_close(a, b, tol=1e-6):
    assert abs(a - b) <= tol


def test_zero_command():
    pose = integrate_pose(Pose2D(1.0, 2.0, 0.5), Twist2D(0.0, 0.0), 1.0, 0.2)
    assert_close(pose.x, 1.0)
    assert_close(pose.y, 2.0)
    assert_close(pose.yaw, 0.5)


def test_straight_forward_integration():
    pose = integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(1.0, 0.0), 1.0, 2.0)
    assert_close(pose.x, 1.0)
    assert_close(pose.y, 0.0)
    assert_close(pose.yaw, 0.0)


def test_positive_yaw_rotation():
    pose = integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(0.0, 1.0), 1.0, 2.0)
    assert_close(pose.yaw, 1.0)


def test_negative_yaw_rotation():
    pose = integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(0.0, -1.0), 1.0, 2.0)
    assert_close(pose.yaw, -1.0)


def test_combined_curve():
    pose = integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(1.0, 1.0), 1.0, 2.0)
    assert_close(pose.x, math.sin(1.0))
    assert_close(pose.y, 1.0 - math.cos(1.0))
    assert_close(pose.yaw, 1.0)


def test_yaw_normalization():
    assert -math.pi <= normalize_yaw(5.0 * math.pi) < math.pi
    pose = integrate_pose(Pose2D(0.0, 0.0, 3.1), Twist2D(0.0, 2.0), 1.0, 2.0)
    assert -math.pi <= pose.yaw < math.pi


def test_invalid_nan_command_rejection():
    assert not is_finite_twist(Twist2D(float("nan"), 0.0))
    with pytest.raises(ValueError):
        integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(float("nan"), 0.0), 1.0, 2.0)


def test_invalid_infinity_rejection():
    assert not is_finite_twist(Twist2D(0.0, float("inf")))
    with pytest.raises(ValueError):
        integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(0.0, float("inf")), 1.0, 2.0)


def test_large_dt_clamping():
    pose = integrate_pose(Pose2D(0.0, 0.0, 0.0), Twist2D(1.0, 0.0), 10.0, 0.25)
    assert_close(pose.x, 0.25)


def test_stale_command_zeroing_logic():
    assert stale_or_missing_command(10.6, 10.0, 0.5)
    assert command_for_integration(Twist2D(1.0, 0.0), 10.6, 10.0, 0.5) == Twist2D(0.0, 0.0)
    assert command_for_integration(Twist2D(1.0, 0.0), 10.4, 10.0, 0.5) == Twist2D(1.0, 0.0)


def test_initial_pose_reset_semantics():
    pose = reset_pose(1.0, 2.0, 5.0)
    assert_close(pose.x, 1.0)
    assert_close(pose.y, 2.0)
    assert -math.pi <= pose.yaw < math.pi


def test_quaternion_normalization():
    q = quaternion_from_yaw(1.25)
    norm = math.sqrt(sum(v * v for v in q))
    assert_close(norm, 1.0)
    assert_close(yaw_from_quaternion(*q), 1.25)

