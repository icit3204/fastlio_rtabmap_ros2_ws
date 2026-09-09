import math

import pytest

from vehicle_cmd_safety.localization_validity_core import (
    LocalizationValidityConfig,
    LocalizationValidityCore,
    Pose3,
)


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return Pose3(x, y, z, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def core(**kwargs):
    defaults = dict(
        odom_freshness_sec=1.0,
        tf_freshness_sec=1.0,
        stability_duration_sec=0.5,
        stability_min_observations=3,
        translation_jump_threshold_m=1.0,
        rotation_jump_threshold_rad=0.5,
    )
    defaults.update(kwargs)
    return LocalizationValidityCore(LocalizationValidityConfig(**defaults))


def observe(c, now, sequence, map_pose=None, odom_pose=None):
    c.set_odometry(odom_pose or pose(), now, now)
    c.set_map_odom(now, now)
    c.set_complete_pose(map_pose or pose(), now, now, sequence)


def make_valid(c, start=0.0, map_pose=None):
    observe(c, start, 1, map_pose)
    assert c.tick(start, start).reason == "STABILITY_WAIT"
    observe(c, start + 0.25, 2, map_pose)
    assert c.tick(start + 0.25, start + 0.25).reason == "STABILITY_WAIT"
    observe(c, start + 0.50, 3, map_pose)
    status = c.tick(start + 0.50, start + 0.50)
    assert status.valid
    return status


def contract_core(**kwargs):
    """Production-shaped monitor core for RTAB's bounded +0.10 s TF contract."""
    defaults = dict(
        odom_freshness_sec=0.50,
        tf_freshness_sec=0.50,
        stability_duration_sec=1.0,
        stability_min_observations=3,
        translation_jump_threshold_m=1.0,
        rotation_jump_threshold_rad=math.pi / 4.0,
        future_stamp_tolerance_sec=0.10,
    )
    defaults.update(kwargs)
    return LocalizationValidityCore(LocalizationValidityConfig(**defaults))


def observe_contract(c, now, sequence, future_offset=0.10):
    c.set_odometry(pose(), now, now)
    c.set_map_odom(now, now + future_offset)
    c.set_complete_pose(pose(), now, now, sequence)


def test_rtab_future_timestamp_exact_authorized_boundary_is_accepted():
    c = contract_core()
    observe_contract(c, 0.0, 1, 0.10)
    assert c.tick(0.0, 0.0).reason == "STABILITY_WAIT"
    observe_contract(c, 0.5, 2, 0.10)
    assert c.tick(0.5, 0.5).reason == "STABILITY_WAIT"
    observe_contract(c, 1.0, 3, 0.10)
    assert c.tick(1.0, 1.0).valid


@pytest.mark.parametrize("future_offset", [0.100001, 0.50])
def test_rtab_future_timestamp_above_authorized_boundary_fails_closed(future_offset):
    c = contract_core()
    observe_contract(c, 0.0, 1, future_offset)
    status = c.tick(0.0, 0.0)
    assert status.reason == "MAP_ODOM_TIMESTAMP_IN_FUTURE"
    assert not status.valid


def test_intermittent_authorized_future_samples_do_not_reset_stability():
    c = contract_core()
    for now, sequence, offset in ((0.0, 1, 0.10), (0.5, 2, 0.099999), (1.0, 3, 0.10)):
        observe_contract(c, now, sequence, offset)
        status = c.tick(now, now)
    assert status.valid and status.reason == "VALID"


def test_intermittent_out_of_bound_future_sample_resets_stability():
    c = contract_core()
    observe_contract(c, 0.0, 1, 0.10)
    assert c.tick(0.0, 0.0).reason == "STABILITY_WAIT"
    observe_contract(c, 0.5, 2, 0.10)
    assert c.tick(0.5, 0.5).reason == "STABILITY_WAIT"
    observe_contract(c, 1.0, 3, 0.100001)
    assert c.tick(1.0, 1.0).reason == "MAP_ODOM_TIMESTAMP_IN_FUTURE"
    observe_contract(c, 1.1, 4, 0.10)
    assert c.tick(1.1, 1.1).reason == "STABILITY_WAIT"


def test_recovery_after_future_fault_requires_full_stability_window():
    c = contract_core()
    observe_contract(c, 0.0, 1, 0.10)
    c.tick(0.0, 0.0)
    observe_contract(c, 0.5, 2, 0.10)
    c.tick(0.5, 0.5)
    observe_contract(c, 1.0, 3, 0.100001)
    assert c.tick(1.0, 1.0).reason == "MAP_ODOM_TIMESTAMP_IN_FUTURE"
    observe_contract(c, 1.1, 4, 0.10)
    assert c.tick(1.1, 1.1).reason == "STABILITY_WAIT"
    observe_contract(c, 1.6, 5, 0.10)
    assert c.tick(1.6, 1.6).reason == "STABILITY_WAIT"
    observe_contract(c, 2.1, 6, 0.10)
    assert c.tick(2.1, 2.1).valid


def test_rtab_like_future_tf_becomes_stale_after_producer_loss():
    c = contract_core()
    for now, sequence in ((0.0, 1), (0.5, 2), (1.0, 3)):
        observe_contract(c, now, sequence, 0.10)
        status = c.tick(now, now)
    assert status.valid
    c.set_odometry(pose(), 1.51, 1.51)
    c.set_complete_pose(pose(), 1.51, 1.51, 4)
    stale = c.tick(1.51, 1.51)
    assert stale.reason == "MAP_ODOM_STALE"
    assert not stale.valid


def test_startup_is_fail_closed_and_reason_priority_is_deterministic():
    c = core()
    assert c.tick(0.0, 0.0).reason == "ODOM_NOT_RECEIVED"
    c.set_odometry(pose(), 0.0, 0.0)
    assert c.tick(0.0, 0.0).reason == "MAP_ODOM_MISSING"
    c.set_map_odom(0.0, 0.0)
    assert c.tick(0.0, 0.0).reason == "MAP_BASE_UNRESOLVABLE"


def test_nominal_requires_duration_and_multiple_observations():
    make_valid(core())


def test_odom_stale_then_fresh_restarts_full_stability_window():
    c = core()
    observe(c, 0.0, 1)
    assert c.tick(1.01, 1.01).reason == "ODOM_STALE"
    observe(c, 1.1, 2)
    assert c.tick(1.1, 1.1).reason == "STABILITY_WAIT"
    observe(c, 1.35, 3)
    assert c.tick(1.35, 1.35).reason == "STABILITY_WAIT"
    observe(c, 1.6, 4)
    assert c.tick(1.6, 1.6).valid


def test_receipt_deadman_rejects_replayed_fresh_ros_stamp():
    c = core()
    observe(c, 0.0, 1)
    c.odometry.ros_stamp_sec = 1.1
    assert c.tick(1.1, 1.1).reason == "ODOM_STALE"


def test_map_odom_missing_stale_and_restored():
    c = core()
    observe(c, 0.0, 1)
    c.set_map_odom_unavailable()
    assert c.tick(0.1, 0.1).reason == "MAP_ODOM_MISSING"
    c.set_map_odom(0.0, 0.0)
    assert c.tick(1.01, 1.01).reason == "ODOM_STALE"
    c.set_odometry(pose(), 1.01, 1.01)
    assert c.tick(1.01, 1.01).reason == "MAP_ODOM_STALE"
    observe(c, 1.1, 2)
    assert c.tick(1.1, 1.1).reason == "STABILITY_WAIT"


def test_complete_transform_missing_fails_closed_and_restores_to_stability_wait():
    c = core()
    observe(c, 0.0, 1)
    c.set_complete_pose_unavailable()
    assert c.tick(0.1, 0.1).reason == "MAP_BASE_UNRESOLVABLE"
    observe(c, 0.2, 2)
    assert c.tick(0.2, 0.2).reason == "STABILITY_WAIT"


@pytest.mark.parametrize(
    "bad_pose,reason",
    [
        (Pose3(math.nan, 0, 0, 0, 0, 0, 1), "NONFINITE_POSITION"),
        (Pose3(math.inf, 0, 0, 0, 0, 0, 1), "NONFINITE_POSITION"),
        (Pose3(0, 0, 0, math.nan, 0, 0, 1), "NONFINITE_ORIENTATION"),
        (Pose3(0, 0, 0, 0, 0, 0, 0), "INVALID_QUATERNION"),
        (Pose3(0, 0, 0, 0, 0, 0, 1.02), "INVALID_QUATERNION"),
    ],
)
def test_numeric_and_quaternion_rejection(bad_pose, reason):
    c = core()
    observe(c, 0.0, 1, map_pose=bad_pose)
    assert c.tick(0.0, 0.0).reason == reason


def test_translation_below_threshold_is_accepted_above_threshold_latches_until_stable():
    c = core()
    make_valid(c)
    observe(c, 0.6, 4, pose(0.9))
    below_threshold = c.tick(0.6, 0.6)
    assert below_threshold.reason != "TRANSLATION_JUMP"
    observe(c, 0.7, 5, pose(2.1))
    jumped = c.tick(0.7, 0.7)
    assert jumped.reason == "TRANSLATION_JUMP" and jumped.pose_jump_detected
    observe(c, 0.95, 6, pose(2.1))
    assert c.tick(0.95, 0.95).pose_jump_detected
    observe(c, 1.20, 7, pose(2.1))
    recovered = c.tick(1.20, 1.20)
    assert recovered.valid and not recovered.pose_jump_detected


def test_rotation_jump_and_wraparound_safe_small_rotation():
    c = core()
    make_valid(c, map_pose=pose(yaw=math.pi - 0.01))
    observe(c, 0.6, 4, pose(yaw=-math.pi + 0.01))
    assert c.tick(0.6, 0.6).reason != "ROTATION_JUMP"
    observe(c, 0.7, 5, pose(yaw=-2.0))
    status = c.tick(0.7, 0.7)
    assert status.reason == "ROTATION_JUMP" and status.pose_jump_detected


def test_age_outputs_are_seconds_and_unavailable_is_minus_one():
    c = core()
    empty = c.tick(2.0, 2.0)
    assert empty.pose_age_sec == -1.0 and empty.tf_age_sec == -1.0
    c.set_odometry(pose(), 1.9, 1.75)
    c.set_map_odom(1.9, 1.8)
    c.set_complete_pose(pose(), 1.9, 1.8, 1)
    status = c.tick(2.0, 2.0)
    assert status.pose_age_sec == pytest.approx(0.25)
    assert status.tf_age_sec == pytest.approx(0.20)


def test_future_timestamps_fail_closed():
    c = core()
    observe(c, 1.0, 1)
    c.odometry.ros_stamp_sec = 1.1
    assert c.tick(1.0, 1.0).reason == "ODOM_TIMESTAMP_IN_FUTURE"
    c.odometry.ros_stamp_sec = 1.0
    c.map_odom.ros_stamp_sec = 1.1
    assert c.tick(1.0, 1.0).reason == "MAP_ODOM_TIMESTAMP_IN_FUTURE"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"odom_freshness_sec": -1.0},
        {"tf_freshness_sec": 0.0},
        {"translation_jump_threshold_m": math.nan},
        {"stability_min_observations": 1},
        {"future_stamp_tolerance_sec": -0.1},
    ],
)
def test_invalid_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        LocalizationValidityCore(LocalizationValidityConfig(**kwargs))
