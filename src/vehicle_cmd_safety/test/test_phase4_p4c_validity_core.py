from vehicle_cmd_safety.validity_core import (
    CollisionMonitorState,
    CollisionMonitorValidityCore,
    ValidityConfig,
)


def healthy_state() -> CollisionMonitorState:
    return CollisionMonitorState(
        lifecycle_reachable=True,
        lifecycle_active=True,
        config_reachable=True,
        configured_source_present=True,
        configured_source_type_matches=True,
        configured_source_topic_matches=True,
    )


def test_startup_false_and_recovery_stability_window():
    core = CollisionMonitorValidityCore(ValidityConfig())
    assert core.tick(1.0).reason_code == "SOURCE_NEVER_RECEIVED"
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.1)
    assert core.tick(1.1).reason_code == "RECOVERY_STABILITY_WAIT"
    core.set_observation("base_footprint", True, 1.61)
    status = core.tick(1.62)
    assert status.valid
    assert status.reason_code == "VALID"


def test_source_stale_false_and_recovery_requires_new_window():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.0)
    assert core.tick(1.6).reason_code == "SOURCE_STALE"
    core.set_observation("base_footprint", True, 1.7)
    assert not core.tick(1.9).valid
    core.set_observation("base_footprint", True, 2.1)
    assert not core.tick(2.1).valid
    core.set_observation("base_footprint", True, 2.3)
    assert core.tick(2.79).valid
    core.set_observation("base_footprint", True, 2.81)
    assert core.tick(3.10).valid
    core.set_observation("base_footprint", True, 3.20)
    assert core.tick(3.69).valid


def test_frame_structure_lifecycle_and_config_failures():
    cases = [
        ("bad_frame", "SOURCE_FRAME_MISMATCH", lambda c: c.set_observation("laser", True, 1.0), healthy_state()),
        ("bad_structure", "SOURCE_MESSAGE_INVALID", lambda c: c.set_observation("base_footprint", False, 1.0), healthy_state()),
        ("inactive", "COLLISION_MONITOR_NOT_ACTIVE", lambda c: c.set_observation("base_footprint", True, 1.0), CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=False, config_reachable=True)),
        ("missing", "CONFIGURED_SOURCE_MISSING", lambda c: c.set_observation("base_footprint", True, 1.0), CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=True, config_reachable=True)),
        ("type", "CONFIGURED_SOURCE_TYPE_MISMATCH", lambda c: c.set_observation("base_footprint", True, 1.0), CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=True, config_reachable=True, configured_source_present=True)),
        ("topic", "CONFIGURED_SOURCE_TOPIC_MISMATCH", lambda c: c.set_observation("base_footprint", True, 1.0), CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=True, config_reachable=True, configured_source_present=True, configured_source_type_matches=True)),
    ]
    for _, reason, observation, state in cases:
        core = CollisionMonitorValidityCore(ValidityConfig())
        core.set_collision_monitor_state(state)
        observation(core)
        assert core.tick(1.1).reason_code == reason


def test_source_type_topic_mismatch_and_publisher_conflict():
    core = CollisionMonitorValidityCore(ValidityConfig(source_type="pointcloud"))
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.0)
    assert core.tick(1.1).reason_code == "SOURCE_TYPE_MISMATCH"

    core = CollisionMonitorValidityCore(ValidityConfig(source_topic="/other"))
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.0)
    assert core.tick(1.1).reason_code == "SOURCE_TOPIC_MISMATCH"

    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.0)
    core.set_valid_publisher_count(2)
    assert core.tick(1.1).reason_code == "COLLISION_VALID_AUTHORITY_CONFLICT"


def test_pointcloud_profile_validates_independently():
    config = ValidityConfig(
        source_type="pointcloud",
        source_topic="/phase4/synthetic_points",
        expected_observation_source_name="pointcloud",
        expected_observation_source_type="pointcloud",
        expected_observation_source_topic="/phase4/synthetic_points",
    )
    core = CollisionMonitorValidityCore(config)
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.0)
    core.set_observation("base_footprint", True, 1.6)
    assert not core.tick(2.09).valid
    core.set_observation("base_footprint", True, 2.10)
    assert core.tick(2.59).valid


def test_collision_monitor_state_requires_all_mandatory_fields():
    core = CollisionMonitorValidityCore(ValidityConfig())
    state = healthy_state()
    state.lifecycle_reachable = False
    core.set_collision_monitor_state(state)
    core.set_observation("base_footprint", True, 1.0)
    assert core.tick(1.6).reason_code == "COLLISION_MONITOR_UNREACHABLE"


def test_health_window_persists_after_cached_state_becomes_available():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.0)
    assert core.tick(1.1).reason_code == "RECOVERY_STABILITY_WAIT"
    assert not core.tick(1.4).valid
    core.set_observation("base_footprint", True, 1.21)
    assert core.tick(1.6).valid


def simulate_heartbeat(core, start=1.0, count=20, step=0.05):
    return [core.tick(start + i * step) for i in range(count)]


def assert_false_heartbeat(statuses, reason):
    assert len(statuses) == 20
    assert all(not status.valid for status in statuses)
    assert all(status.reason_code == reason for status in statuses)
    assert all(status.diagnostics["state"] == "INVALID" for status in statuses)


def test_services_unavailable_forever_keeps_false_heartbeat_and_diagnostics():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    assert_false_heartbeat(simulate_heartbeat(core), "COLLISION_MONITOR_UNREACHABLE")


def test_services_appear_after_startup_without_active_state_remains_false():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    assert core.tick(1.0).reason_code == "COLLISION_MONITOR_UNREACHABLE"
    core.set_collision_monitor_state(CollisionMonitorState(lifecycle_reachable=True, config_reachable=True))
    assert_false_heartbeat(simulate_heartbeat(core, start=1.05), "COLLISION_MONITOR_NOT_ACTIVE")


def test_services_respond_normally_then_eventual_healthy_state_is_evaluated():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(healthy_state())
    assert core.tick(1.0).reason_code == "RECOVERY_STABILITY_WAIT"
    assert core.tick(1.49).reason_code == "RECOVERY_STABILITY_WAIT"
    status = core.tick(1.50)
    assert status.valid
    assert status.reason_code == "VALID"


def test_slow_lifecycle_response_keeps_heartbeat_false_until_active():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(CollisionMonitorState(lifecycle_reachable=True, config_reachable=True))
    assert_false_heartbeat(simulate_heartbeat(core), "COLLISION_MONITOR_NOT_ACTIVE")
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 2.0)
    assert not core.tick(2.0).valid
    assert core.tick(2.5).valid


def test_slow_parameter_response_keeps_heartbeat_false_until_config_matches():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=True, config_reachable=True))
    assert_false_heartbeat(simulate_heartbeat(core), "CONFIGURED_SOURCE_MISSING")
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 2.0)
    assert not core.tick(2.0).valid
    assert core.tick(2.5).valid


def test_service_future_never_completes_equivalent_cached_state_does_not_starve_heartbeat():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=False, config_reachable=False))
    assert_false_heartbeat(simulate_heartbeat(core), "COLLISION_MONITOR_NOT_ACTIVE")


def test_service_exception_equivalent_cached_unreachable_state_does_not_terminate_decision_loop():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    statuses = simulate_heartbeat(core)
    assert_false_heartbeat(statuses, "COLLISION_MONITOR_UNREACHABLE")
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 2.0)
    assert not core.tick(2.0).valid
    assert core.tick(2.5).valid


def test_lifecycle_state_becomes_active_after_inactive_samples():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=False, config_reachable=True))
    assert core.tick(1.0).reason_code == "COLLISION_MONITOR_NOT_ACTIVE"
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.5)
    assert not core.tick(1.5).valid
    assert core.tick(2.0).valid


def test_parameter_values_become_available_after_missing_config_samples():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(CollisionMonitorState(lifecycle_reachable=True, lifecycle_active=True, config_reachable=True))
    assert core.tick(1.0).reason_code == "CONFIGURED_SOURCE_MISSING"
    core.set_collision_monitor_state(healthy_state())
    core.set_observation("base_footprint", True, 1.5)
    assert not core.tick(1.5).valid
    assert core.tick(2.0).valid


def test_source_configuration_mismatch_keeps_false_heartbeat_and_diagnostics():
    core = CollisionMonitorValidityCore(ValidityConfig())
    core.set_observation("base_footprint", True, 1.0)
    core.set_collision_monitor_state(
        CollisionMonitorState(
            lifecycle_reachable=True,
            lifecycle_active=True,
            config_reachable=True,
            configured_source_present=True,
            configured_source_type_matches=True,
            configured_source_topic_matches=False,
        )
    )
    assert_false_heartbeat(simulate_heartbeat(core), "CONFIGURED_SOURCE_TOPIC_MISMATCH")
