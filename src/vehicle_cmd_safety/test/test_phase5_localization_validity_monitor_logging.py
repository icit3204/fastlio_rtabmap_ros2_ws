import pytest
import rclpy

from vehicle_cmd_safety.localization_validity_core import (
    LocalizationValidityConfig,
    LocalizationValidityCore,
    LocalizationValidityStatus,
    Pose3,
)
from vehicle_cmd_safety.localization_validity_monitor import LocalizationValidityMonitor


@pytest.fixture(scope="module")
def monitor():
    rclpy.init(args=[])
    node = LocalizationValidityMonitor()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def status(valid, reason, stable_duration=0.0, stable_observations=0):
    return LocalizationValidityStatus(
        valid=valid,
        reason=reason,
        pose_age_sec=0.01,
        tf_age_sec=0.01,
        pose_jump_detected=False,
        stable_duration_sec=stable_duration,
        stable_observations=stable_observations,
    )


def emit_sequence(monitor, sequence):
    for item in sequence:
        monitor._log_transition(item)


def test_invalid_stability_wait_valid_transition_uses_actual_production_helper(monitor):
    emit_sequence(monitor, [
        status(False, "MAP_ODOM_MISSING"),
        status(False, "STABILITY_WAIT", 0.5, 2),
        status(True, "VALID", 1.0, 3),
    ])


def test_valid_to_invalid_transition_does_not_raise(monitor):
    emit_sequence(monitor, [status(True, "VALID", 1.0, 3), status(False, "MAP_ODOM_STALE")])


def test_reason_and_severity_transitions_do_not_raise(monitor):
    emit_sequence(monitor, [
        status(False, "ODOM_NOT_RECEIVED"),
        status(False, "MAP_ODOM_MISSING"),
        status(False, "STABILITY_WAIT", 0.5, 2),
        status(True, "VALID", 1.0, 3),
    ])


def test_repeated_valid_and_invalid_observations_do_not_raise(monitor):
    emit_sequence(monitor, [status(True, "VALID", 1.0, 3)] * 5)
    emit_sequence(monitor, [status(False, "MAP_ODOM_STALE")] * 5)


def test_stale_recovery_transition_does_not_raise(monitor):
    emit_sequence(monitor, [
        status(True, "VALID", 1.0, 3),
        status(False, "MAP_ODOM_STALE"),
        status(False, "STABILITY_WAIT", 0.5, 2),
        status(True, "VALID", 1.0, 3),
    ])


def test_rtab_future_boundary_and_above_bound_logging_do_not_raise(monitor):
    emit_sequence(monitor, [
        status(False, "STABILITY_WAIT", 0.5, 2),
        status(True, "VALID", 1.0, 3),
        status(False, "MAP_ODOM_TIMESTAMP_IN_FUTURE"),
    ])


def test_timer_publishes_decision_outputs_before_transition_logging(monitor):
    events = []

    class Publisher:
        def __init__(self, name):
            self.name = name

        def publish(self, _message):
            events.append(self.name)

    original_observe = monitor._observe_tf
    original_tick = monitor._core.tick
    original_publishers = (
        monitor._valid_pub, monitor._reason_pub, monitor._pose_age_pub,
        monitor._tf_age_pub, monitor._jump_pub,
    )
    original_log = monitor._log_transition
    try:
        monitor._observe_tf = lambda _now: None
        monitor._core.tick = lambda _steady_now, _ros_now: status(True, "VALID", 1.0, 3)
        monitor._valid_pub = Publisher("valid")
        monitor._reason_pub = Publisher("reason")
        monitor._pose_age_pub = Publisher("pose_age")
        monitor._tf_age_pub = Publisher("tf_age")
        monitor._jump_pub = Publisher("jump")

        def log_after_publication(current_status):
            events.append("log")
            original_log(current_status)

        monitor._log_transition = log_after_publication
        monitor._last_state = None
        monitor._timer_cb()
    finally:
        monitor._observe_tf = original_observe
        monitor._core.tick = original_tick
        (monitor._valid_pub, monitor._reason_pub, monitor._pose_age_pub,
         monitor._tf_age_pub, monitor._jump_pub) = original_publishers
        monitor._log_transition = original_log

    assert events == ["valid", "reason", "pose_age", "tf_age", "jump", "log"]


def test_state_machine_semantics_remain_unchanged_through_transition_matrix():
    def pose():
        return Pose3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    core = LocalizationValidityCore(LocalizationValidityConfig(
        odom_freshness_sec=0.50,
        tf_freshness_sec=0.50,
        stability_duration_sec=1.0,
        stability_min_observations=3,
        future_stamp_tolerance_sec=0.10,
    ))
    for now, sequence in ((0.0, 1), (0.5, 2), (1.0, 3)):
        core.set_odometry(pose(), now, now)
        core.set_map_odom(now, now + 0.10)
        core.set_complete_pose(pose(), now, now, sequence)
        result = core.tick(now, now)
    assert result.valid and result.reason == "VALID"

    core.set_odometry(pose(), 1.1, 1.1)
    core.set_map_odom(1.1, 1.200001)
    core.set_complete_pose(pose(), 1.1, 1.1, 4)
    result = core.tick(1.1, 1.1)
    assert not result.valid and result.reason == "MAP_ODOM_TIMESTAMP_IN_FUTURE"
