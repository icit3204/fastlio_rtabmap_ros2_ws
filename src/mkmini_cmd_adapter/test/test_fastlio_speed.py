import math

from mkmini_cmd_adapter.fastlio_speed import FastLioSpeedEstimator


def test_consecutive_finite_poses_derive_planar_speed_and_distance():
    estimator = FastLioSpeedEstimator()
    estimator.observe(received_monotonic_sec=1.0, stamp_sec=10.0, x_m=0.0, y_m=0.0, z_m=0.0)
    sample = estimator.observe(received_monotonic_sec=1.1, stamp_sec=10.1, x_m=.004, y_m=0.0, z_m=0.0)
    assert math.isclose(sample.speed_mps, .04)
    snapshot = estimator.snapshot(1.11)
    assert snapshot["valid"]
    assert math.isclose(snapshot["displacement_from_origin_m"], .004)


def test_stale_nonfinite_and_nonadvancing_odometry_fail_closed():
    estimator = FastLioSpeedEstimator()
    estimator.observe(received_monotonic_sec=1.0, stamp_sec=10.0, x_m=0.0, y_m=0.0, z_m=0.0)
    assert estimator.snapshot(1.31)["reason"] == "FASTLIO_ODOMETRY_STALE"
    nonfinite = estimator.observe(received_monotonic_sec=1.1, stamp_sec=10.1, x_m=float("nan"), y_m=0.0, z_m=0.0)
    assert not nonfinite.valid and nonfinite.reason == "FASTLIO_ODOMETRY_NONFINITE"
    estimator = FastLioSpeedEstimator()
    estimator.observe(received_monotonic_sec=1.0, stamp_sec=10.0, x_m=0.0, y_m=0.0, z_m=0.0)
    repeat = estimator.observe(received_monotonic_sec=1.1, stamp_sec=10.0, x_m=0.0, y_m=0.0, z_m=0.0)
    assert not repeat.valid and repeat.reason == "FASTLIO_ODOMETRY_TIMESTAMP_NOT_ADVANCING"


def test_speed_above_commissioning_ceiling_is_observable():
    estimator = FastLioSpeedEstimator()
    estimator.observe(received_monotonic_sec=1.0, stamp_sec=1.0, x_m=0.0, y_m=0.0, z_m=0.0)
    sample = estimator.observe(received_monotonic_sec=1.1, stamp_sec=1.1, x_m=.0061, y_m=0.0, z_m=0.0)
    assert sample.speed_mps > .060
