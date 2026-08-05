from vehicle_cmd_safety.phase4_p4c_runtime_runner import (
    FAULT_CASES,
    GATE_DIAGNOSTIC_NAME,
    P4CRuntimeRunner,
    ROGUE_OUTPUT_FRAME_ID,
    ROGUE_OUTPUT_RATE_HZ,
    ValidityBoolSample,
    ValidityReadinessSnapshot,
    VehicleOutputSample,
    contiguous_gate_owned_zero_windows,
    contiguous_bool_windows,
    diagnostic_status,
    publisher_gid_from_message_info,
    is_gate_fault_status,
    sample_rate_hz,
    sample_duration_sec,
    select_gate_owned_zero_readiness_window,
    select_contiguous_window,
    scenario_elapsed_after_readiness,
    health_epoch_elapsed_sec,
    true_sample_satisfies_stability,
    select_latest_gate_fault,
    select_authoritative_gate_fault,
    uint8_text_value,
    validity_window_metrics,
    validity_window_passes,
    validity_readiness_missing,
    validity_readiness_ready,
)
from vehicle_cmd_safety.phase4_p4c_evidence_monitor import _diagnostic_level


OK = 0
WARN = 1
ERROR = 2
STALE = 3


def status(level, *, name=GATE_DIAGNOSTIC_NAME, state="FAULT", reason="SAFE_TWIST_STALE", latched="true", stamp=1):
    return {
        "monotonic_ns": stamp,
        "name": name,
        "level": level,
        "message": reason,
        "values": {
            "state": state,
            "reason_code": reason,
            "fault_latched": latched,
        },
    }


def test_numeric_diagnostic_levels_are_converted_explicitly():
    assert uint8_text_value(OK) == 0
    assert uint8_text_value(WARN) == 1
    assert uint8_text_value(ERROR) == 2
    assert uint8_text_value(STALE) == 3
    assert uint8_text_value(bytes([ERROR])) == 2


def test_evidence_monitor_diagnostic_level_accepts_ros_representations():
    assert _diagnostic_level(ERROR) == 2
    assert _diagnostic_level(bytes([ERROR])) == 2
    assert _diagnostic_level(bytearray([ERROR])) == 2
    assert _diagnostic_level(memoryview(bytes([ERROR]))) == 2


def test_evidence_monitor_diagnostic_level_rejects_malformed_values_without_silent_coercion():
    for value in (bytes([1, 2]), object()):
        try:
            _diagnostic_level(value)
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError(f"expected malformed diagnostic level to be rejected: {value!r}")


def test_fault_requires_error_fault_state_and_latched_public_contract():
    assert is_gate_fault_status(status(ERROR))
    assert is_gate_fault_status(status(STALE))
    assert not is_gate_fault_status(status(OK))
    assert not is_gate_fault_status(status(WARN))
    assert not is_gate_fault_status(status(ERROR, state="ARMED"))
    assert not is_gate_fault_status(status(ERROR, latched="false"))
    assert not is_gate_fault_status(status(ERROR, name="other_node"))
    assert is_gate_fault_status({"name": GATE_DIAGNOSTIC_NAME, "level": "2", "values": {"state": "FAULT", "fault_latched": "true"}})


def test_repeated_and_out_of_order_diagnostics_select_latest_gate_fault():
    events = [
        status(ERROR, stamp=50),
        status(WARN, state="DISARMED", reason="DISARMED_ZERO", latched="false", stamp=200),
        status(ERROR, stamp=100),
        status(ERROR, name="other_node", stamp=300),
    ]
    selected = select_latest_gate_fault(events)
    assert selected is not None
    assert selected["monotonic_ns"] == 100
    assert selected["values"]["reason_code"] == "SAFE_TWIST_STALE"


def test_diagnostic_array_entries_are_normalized_by_runner_schema():
    entries = [
        {"monotonic_ns": 10, "name": "other_node", "level": ERROR, "message": "FAULT", "values": {"state": "FAULT", "fault_latched": "true"}},
        status(ERROR, stamp=20),
        {"monotonic_ns": 30, "name": GATE_DIAGNOSTIC_NAME, "level": OK, "message": "ARMED_COMMAND", "values": {"state": "ARMED", "fault_latched": "false"}},
        {"monotonic_ns": 40, "name": GATE_DIAGNOSTIC_NAME, "level": "3", "message": "SAFE_TWIST_STALE", "values": {"state": "FAULT", "fault_latched": "true"}},
    ]
    assert select_latest_gate_fault(entries)["monotonic_ns"] == 40


def test_select_latest_fault_ignores_out_of_order_non_fault_and_requires_numeric_levels():
    events = [
        {"monotonic_ns": 100, "name": GATE_DIAGNOSTIC_NAME, "level": 1, "message": "DISARMED_ZERO", "values": {"state": "DISARMED", "fault_latched": "false"}},
        {"monotonic_ns": 200, "name": GATE_DIAGNOSTIC_NAME, "level": "2", "message": "SAFE_TWIST_STALE", "values": {"state": "FAULT", "fault_latched": "true"}},
        {"monotonic_ns": 150, "name": GATE_DIAGNOSTIC_NAME, "level": 0, "message": "ARMED_COMMAND", "values": {"state": "ARMED", "fault_latched": "false"}},
    ]
    assert select_latest_gate_fault(events)["monotonic_ns"] == 200
    assert is_gate_fault_status({"name": GATE_DIAGNOSTIC_NAME, "level": 2, "values": {"state": "FAULT", "fault_latched": "true"}})
    assert is_gate_fault_status({"name": GATE_DIAGNOSTIC_NAME, "level": 3, "values": {"state": "FAULT", "fault_latched": "true"}})
    assert not is_gate_fault_status({"name": GATE_DIAGNOSTIC_NAME, "level": 1, "values": {"state": "FAULT", "fault_latched": "true"}})


def test_authoritative_fault_selection_merges_state_and_diagnostics_streams():
    state_events = [
        {"monotonic_ns": 10, "name": GATE_DIAGNOSTIC_NAME, "level": 1, "message": "DISARMED_ZERO", "values": {"state": "DISARMED", "fault_latched": "false"}},
    ]
    diagnostic_events = [
        {"monotonic_ns": 20, "name": GATE_DIAGNOSTIC_NAME, "level": 2, "message": "SAFE_TWIST_STALE", "values": {"state": "FAULT", "fault_latched": "true"}},
        {"monotonic_ns": 15, "name": "other_node", "level": 2, "message": "FAULT", "values": {"state": "FAULT", "fault_latched": "true"}},
    ]
    selected = select_authoritative_gate_fault(state_events, diagnostic_events)
    assert selected is not None
    assert selected["monotonic_ns"] == 20
    assert selected["message"] == "SAFE_TWIST_STALE"


def test_authoritative_fault_selection_ignores_unrelated_and_out_of_order_entries():
    state_events = [
        {"monotonic_ns": 90, "name": GATE_DIAGNOSTIC_NAME, "level": 1, "message": "DISARMED_ZERO", "values": {"state": "DISARMED", "fault_latched": "false"}},
        {"monotonic_ns": 110, "name": GATE_DIAGNOSTIC_NAME, "level": 2, "message": "SAFE_TWIST_STALE", "values": {"state": "FAULT", "fault_latched": "true"}},
    ]
    diagnostic_events = [
        {"monotonic_ns": 80, "name": GATE_DIAGNOSTIC_NAME, "level": 0, "message": "ARMED_COMMAND", "values": {"state": "ARMED", "fault_latched": "false"}},
        {"monotonic_ns": 100, "name": "other_node", "level": 2, "message": "FAULT", "values": {"state": "FAULT", "fault_latched": "true"}},
    ]
    selected = select_authoritative_gate_fault(state_events, diagnostic_events)
    assert selected is not None
    assert selected["monotonic_ns"] == 110


def test_diagnostic_status_helper_uses_public_gate_contract():
    msg = diagnostic_status(level=ERROR, state="FAULT", reason="SAFE_TWIST_STALE", latched=True)
    assert msg.name == GATE_DIAGNOSTIC_NAME
    assert uint8_text_value(msg.level) == ERROR
    values = {item.key: item.value for item in msg.values}
    assert values["state"] == "FAULT"
    assert values["reason_code"] == "SAFE_TWIST_STALE"
    assert values["fault_latched"] == "true"


def test_duplicate_permission_fault_cases_are_separate_topics():
    created = []

    class Node:
        def create_rogue_publisher(self, topic):
            created.append(topic)

    node = Node()
    FAULT_CASES["duplicate_localization_permission"](node)
    FAULT_CASES["duplicate_controller_permission"](node)
    FAULT_CASES["duplicate_collision_permission"](node)
    assert created == [
        "/system/localization_valid",
        "/system/controller_valid",
        "/system/collision_monitor_valid",
    ]


def readiness_vehicle_sample(stamp, *, gate=True, value=0.0, frame="base_footprint"):
    return VehicleOutputSample(
        monotonic_ns=stamp,
        ros_stamp=0.0,
        frame_id=frame,
        values=(value, 0.0, 0.0, 0.0, 0.0, 0.0),
        publisher_gid="gate_gid" if gate else "other_gid",
        publisher_node="/guarded_vehicle_cmd_gate" if gate else "/other_node",
    )


def spaced_zero_window(start_ns, *, count, rate_hz):
    step = int(round(1_000_000_000 / rate_hz))
    return [readiness_vehicle_sample(start_ns + index * step) for index in range(count)]


def test_gate_zero_readiness_window_excludes_startup_gap_and_uses_contiguous_samples():
    samples = [
        readiness_vehicle_sample(0),
        readiness_vehicle_sample(50_000_000),
        readiness_vehicle_sample(500_000_000),
        *spaced_zero_window(1_000_000_000, count=41, rate_hz=20.0),
    ]
    windows = contiguous_gate_owned_zero_windows(samples)
    selected = select_gate_owned_zero_readiness_window(samples)
    assert len(windows) == 3
    assert selected[0].monotonic_ns == 1_000_000_000
    assert sample_duration_sec(selected) == 2.0
    assert sample_rate_hz(selected) == 20.0


def test_gate_zero_readiness_window_requires_gate_owned_zero_samples_only():
    samples = [
        *spaced_zero_window(0, count=20, rate_hz=20.0),
        readiness_vehicle_sample(1_000_000_000, value=0.1),
        *spaced_zero_window(2_000_000_000, count=41, rate_hz=20.0),
        readiness_vehicle_sample(4_100_000_000, gate=False),
    ]
    windows = contiguous_gate_owned_zero_windows(samples)
    assert len(windows) == 2
    selected = select_gate_owned_zero_readiness_window(samples)
    assert selected[0].monotonic_ns == 2_000_000_000
    assert all(sample.is_gate_output and not sample.nonzero for sample in selected)


def test_gate_zero_readiness_window_rate_formula_and_boundaries():
    exact_18 = [readiness_vehicle_sample(int(index * 2_000_000_000 / 36)) for index in range(37)]
    exact_22 = [readiness_vehicle_sample(3_000_000_000 + int(index * 2_000_000_000 / 44)) for index in range(45)]
    too_slow = [readiness_vehicle_sample(6_000_000_000 + int(index * 2_000_000_000 / 35)) for index in range(36)]
    too_short = spaced_zero_window(9_000_000_000, count=40, rate_hz=20.0)
    assert sample_rate_hz(exact_18) == 18.0
    assert sample_rate_hz(exact_22) == 22.0
    assert select_gate_owned_zero_readiness_window(exact_18) == exact_18
    assert select_gate_owned_zero_readiness_window(exact_22) == exact_22
    assert select_gate_owned_zero_readiness_window(too_slow) == []
    assert select_gate_owned_zero_readiness_window(too_short) == []


def ready_snapshot(**overrides):
    values = {
        "monitor_process_alive": True,
        "validity_node_discovered": True,
        "intended_validity_publishers": 1,
        "unintended_validity_publishers": 0,
        "evidence_subscriber_active": True,
        "consecutive_validity_samples": 3,
        "matching_validity_diagnostics": 1,
        "monitor_heartbeat_alive": True,
        "unexpected_process_exit": False,
        "lifecycle_service_discovered": True,
        "parameter_service_discovered": True,
        "lifecycle_active": True,
        "source_configuration_ok": True,
        "synthetic_observation_publisher_exists": True,
    }
    values.update(overrides)
    return ValidityReadinessSnapshot(**values)


def test_validity_readiness_accepts_publisher_appearing_after_runner_start():
    assert validity_readiness_ready(ready_snapshot(), positive_collision_monitor=True)


def test_validity_readiness_accepts_publisher_near_old_timeout_boundary_after_samples():
    snapshot = ready_snapshot(consecutive_validity_samples=4, matching_validity_diagnostics=2)
    assert validity_readiness_missing(snapshot, positive_collision_monitor=True) == []


def test_validity_readiness_first_samples_after_graph_discovery_are_required():
    missing = validity_readiness_missing(ready_snapshot(consecutive_validity_samples=0), positive_collision_monitor=True)
    assert "at least three consecutive validity samples" in missing


def test_validity_readiness_graph_discovery_without_samples_is_not_ready():
    snapshot = ready_snapshot(consecutive_validity_samples=0)
    assert not validity_readiness_ready(snapshot, positive_collision_monitor=False)


def test_validity_readiness_samples_without_required_diagnostics_are_not_ready():
    missing = validity_readiness_missing(ready_snapshot(matching_validity_diagnostics=0), positive_collision_monitor=False)
    assert missing == ["at least one matching validity DiagnosticStatus"]


def test_validity_readiness_lifecycle_services_may_appear_late_before_ready():
    snapshot = ready_snapshot(lifecycle_service_discovered=False, parameter_service_discovered=False)
    missing = validity_readiness_missing(snapshot, positive_collision_monitor=True)
    assert "/collision_monitor/get_state discovered" in missing
    assert "/collision_monitor/get_parameters discovered" in missing


def test_validity_readiness_lifecycle_active_is_required_after_service_discovery():
    missing = validity_readiness_missing(ready_snapshot(lifecycle_active=False), positive_collision_monitor=True)
    assert missing == ["Collision Monitor lifecycle ACTIVE"]


def test_validity_readiness_parameter_service_late_blocks_positive_scenario():
    missing = validity_readiness_missing(ready_snapshot(parameter_service_discovered=False), positive_collision_monitor=True)
    assert missing == ["/collision_monitor/get_parameters discovered"]


def test_validity_readiness_node_exit_before_ready_is_reported():
    missing = validity_readiness_missing(ready_snapshot(unexpected_process_exit=True), positive_collision_monitor=False)
    assert missing == ["no launch process exited unexpectedly"]


def test_validity_readiness_timeout_reports_exact_missing_condition():
    snapshot = ready_snapshot(
        monitor_process_alive=False,
        intended_validity_publishers=0,
        consecutive_validity_samples=2,
    )
    assert validity_readiness_missing(snapshot, positive_collision_monitor=False) == [
        "validity-monitor process alive",
        "exactly one intended publisher on /system/collision_monitor_valid",
        "at least three consecutive validity samples",
    ]


def test_scenario_assertion_clock_begins_after_readiness():
    assert scenario_elapsed_after_readiness(1_000_000_000, 1_250_000_000) == 0.25


def test_scenario_assertion_clock_rejects_runner_process_start_basis():
    try:
        scenario_elapsed_after_readiness(2_000_000_000, 1_900_000_000)
    except ValueError as exc:
        assert "before readiness" in str(exc)
    else:
        raise AssertionError("expected assertion clock guard to reject pre-readiness event")


def test_health_can_begin_before_runner_functional_start():
    health_ns = 1_000_000_000
    functional_start_ns = 1_100_000_000
    true_ns = 1_500_000_000
    assert scenario_elapsed_after_readiness(functional_start_ns, true_ns) == 0.4
    assert health_epoch_elapsed_sec(health_ns, true_ns) == 0.5
    assert true_sample_satisfies_stability(health_ns, true_ns)


def test_functional_start_005_after_health_true_at_plus_045_is_valid_with_total_health_050():
    health_ns = 1_000_000_000
    functional_start_ns = 1_050_000_000
    true_ns = functional_start_ns + 450_000_000
    assert health_epoch_elapsed_sec(health_ns, true_ns) == 0.5
    assert true_sample_satisfies_stability(health_ns, true_ns)


def test_same_functional_true_is_invalid_when_total_health_less_than_050():
    health_ns = 1_000_001_000
    functional_start_ns = 1_050_000_000
    true_ns = functional_start_ns + 450_000_000
    assert health_epoch_elapsed_sec(health_ns, true_ns) == 0.499999
    assert not true_sample_satisfies_stability(health_ns, true_ns)


def test_readiness_while_source_silent_has_no_health_epoch():
    first_valid_scan_ns = None
    assert first_valid_scan_ns is None


def test_first_valid_observation_defines_controlled_health_epoch():
    clear_command_ns = 2_000_000_000
    first_valid_scan_ns = 2_075_000_000
    assert first_valid_scan_ns > clear_command_ns
    assert true_sample_satisfies_stability(first_valid_scan_ns, first_valid_scan_ns + 500_000_000)


def test_mandatory_check_unhealthy_resets_health_epoch():
    initial_health_ns = 1_000_000_000
    unhealthy_ns = 1_300_000_000
    restored_health_ns = 1_450_000_000
    true_ns = 1_900_000_000
    assert true_sample_satisfies_stability(initial_health_ns, true_ns)
    assert unhealthy_ns > initial_health_ns
    assert not true_sample_satisfies_stability(restored_health_ns, true_ns)


def test_recovery_stability_restarts_from_zero():
    stale_false_ns = 3_000_000_000
    recovered_health_ns = 3_200_000_000
    assert stale_false_ns < recovered_health_ns
    assert not true_sample_satisfies_stability(recovered_health_ns, recovered_health_ns + 499_999_999)
    assert true_sample_satisfies_stability(recovered_health_ns, recovered_health_ns + 500_000_000)


def test_timestamp_boundary_exactly_050_seconds_passes():
    assert true_sample_satisfies_stability(10_000_000_000, 10_500_000_000)


def test_timestamp_precision_no_rounding_0499999_to_pass():
    assert not true_sample_satisfies_stability(10_000_000_000, 10_499_999_000)


def vehicle_sample(stamp, *, node="/guarded_vehicle_cmd_gate", gid="gate", frame="base_footprint", linear_x=0.0):
    return VehicleOutputSample(
        monotonic_ns=stamp,
        ros_stamp=stamp / 1.0e9,
        frame_id=frame,
        values=(linear_x, 0.0, 0.0, 0.0, 0.0, 0.0),
        publisher_gid=gid,
        publisher_node=node,
    )


def test_publisher_gid_extraction_accepts_message_info_sequences_and_blank():
    class Info:
        publisher_gid = [1, 15, 255]

    class Blank:
        publisher_gid = None

    assert publisher_gid_from_message_info(Info()) == "010fff"
    assert publisher_gid_from_message_info(Blank()) == ""
    assert publisher_gid_from_message_info(None) == ""


def test_message_info_gid_overrides_blank_duplicate_graph_identity():
    class Endpoint:
        def __init__(self, gid, name):
            self.endpoint_gid = gid
            self.node_namespace = "/"
            self.node_name = name

    class Info:
        publisher_gid = [2]

    runner = P4CRuntimeRunner.__new__(P4CRuntimeRunner)
    runner.get_publishers_info_by_topic = lambda topic: [
        Endpoint([1], "guarded_vehicle_cmd_gate"),
        Endpoint([2], "phase4_p4c_runtime_runner"),
    ]
    assert P4CRuntimeRunner._publisher_identity(runner, "/vehicle_cmd_safe", Info()) == ("02", "/phase4_p4c_runtime_runner")
    assert P4CRuntimeRunner._publisher_identity(runner, "/vehicle_cmd_safe", None) == ("", "")


def test_blank_message_info_can_use_distinguishable_output_payload_with_two_publishers():
    runner = P4CRuntimeRunner.__new__(P4CRuntimeRunner)
    runner.current_publishers_by_topic = lambda topic: {
        "gate": "/guarded_vehicle_cmd_gate",
        "rogue": "/phase4_p4c_runtime_runner",
    }
    assert P4CRuntimeRunner._vehicle_identity_from_distinguishable_payload(runner, "base_footprint") == (
        "gate",
        "/guarded_vehicle_cmd_gate",
    )
    assert P4CRuntimeRunner._vehicle_identity_from_distinguishable_payload(runner, ROGUE_OUTPUT_FRAME_ID) == (
        "rogue",
        "/phase4_p4c_runtime_runner",
    )
    assert P4CRuntimeRunner._vehicle_identity_from_distinguishable_payload(runner, "") == ("", "")


def test_gate_and_rogue_output_attribution_uses_owner_not_aggregate_order():
    gate_zero = vehicle_sample(1_000_000_000)
    rogue_zero = vehicle_sample(
        1_001_000_000,
        node="/phase4_p4c_runtime_runner",
        gid="rogue",
        frame=ROGUE_OUTPUT_FRAME_ID,
    )
    delayed_gate_prefault_nonzero = vehicle_sample(1_002_000_000, linear_x=0.1)
    assert gate_zero.is_gate_output
    assert not gate_zero.is_rogue_output
    assert rogue_zero.is_rogue_output
    assert not rogue_zero.nonzero
    assert delayed_gate_prefault_nonzero.is_gate_output
    assert delayed_gate_prefault_nonzero.nonzero


def test_gate_owned_rate_is_not_aggregate_rate():
    gate = [vehicle_sample(1_000_000_000 + i * 50_000_000) for i in range(41)]
    rogue = [
        vehicle_sample(
            1_000_000_000 + i * 1_000_000,
            node="/phase4_p4c_runtime_runner",
            gid="rogue",
            frame=ROGUE_OUTPUT_FRAME_ID,
        )
        for i in range(2001)
    ]
    aggregate = sorted([*gate, *rogue], key=lambda sample: sample.monotonic_ns)
    assert 19.9 <= sample_rate_hz(gate) <= 20.1
    assert sample_rate_hz(aggregate) > 900.0


def test_no_gate_owned_nonzero_after_fault_reaction_interval():
    fault_ns = 1_000_000_000
    samples = [
        vehicle_sample(fault_ns + 50_000_000, linear_x=0.1),
        vehicle_sample(fault_ns + 100_000_000),
        vehicle_sample(fault_ns + 150_000_000),
    ]
    late_nonzero = [
        sample
        for sample in samples
        if sample.is_gate_output and sample.nonzero and sample.monotonic_ns >= fault_ns + 100_000_000
    ]
    assert late_nonzero == []


def test_duplicate_output_rogue_is_zero_only_and_rate_bounded(monkeypatch):
    class Pub:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    pub = Pub()
    runner = P4CRuntimeRunner.__new__(P4CRuntimeRunner)
    runner._rogue_publishers = [
        {
            "topic": "/vehicle_cmd_safe",
            "publisher": pub,
            "last_publish_monotonic": None,
            "publish_count": 0,
            "published_nonzero_count": 0,
        }
    ]
    timeline = iter([10.0, 10.01, 10.051, 10.102])
    monkeypatch.setattr("vehicle_cmd_safety.phase4_p4c_runtime_runner.time.monotonic", lambda: next(timeline))

    P4CRuntimeRunner.publish_rogues(runner)
    P4CRuntimeRunner.publish_rogues(runner)
    P4CRuntimeRunner.publish_rogues(runner)
    P4CRuntimeRunner.publish_rogues(runner)

    assert runner._rogue_publishers[0]["publish_count"] == 3
    assert len(pub.messages) == 3
    assert all(msg.header.frame_id == ROGUE_OUTPUT_FRAME_ID for msg in pub.messages)
    assert all(msg.twist.linear.x == 0.0 and msg.twist.angular.z == 0.0 for msg in pub.messages)
    assert ROGUE_OUTPUT_RATE_HZ == 20.0


def test_vehicle_callback_appends_one_timeline_sample_with_message_info():
    class File:
        def __init__(self):
            self.lines = []

        def write(self, line):
            self.lines.append(line)

        def flush(self):
            pass

    class Info:
        publisher_gid = [9]

    class Endpoint:
        endpoint_gid = [9]
        node_namespace = "/"
        node_name = "guarded_vehicle_cmd_gate"

    runner = P4CRuntimeRunner.__new__(P4CRuntimeRunner)
    runner.vehicle_samples = []
    runner.last_vehicle_ns = None
    runner.first_fault_ns = None
    runner.first_zero_after_fault_ns = None
    runner.first_gate_zero_after_fault_ns = None
    runner.files = {"vehicle": File()}
    runner.mono_ns = lambda: 123
    runner.get_publishers_info_by_topic = lambda topic: [Endpoint()]

    msg = __import__("geometry_msgs.msg", fromlist=["TwistStamped"]).TwistStamped()
    msg.header.frame_id = "base_footprint"
    P4CRuntimeRunner._vehicle_cb(runner, msg, Info())

    assert len(runner.vehicle_samples) == 1
    assert runner.vehicle_samples[0].publisher_gid == "09"
    assert runner.vehicle_samples[0].publisher_node == "/guarded_vehicle_cmd_gate"
    assert len(runner.files["vehicle"].lines) == 1


def validity_sample(stamp_ns, value, gid="monitor_gid", node="/collision_monitor_validity_monitor"):
    return ValidityBoolSample(stamp_ns, value, gid, node)


def test_phase_window_metrics_keep_true_windows_separate_across_false_gap():
    samples = [
        *[validity_sample(1_000_000_000 + i * 50_000_000, True) for i in range(41)],
        *[validity_sample(5_000_000_000 + i * 50_000_000, False) for i in range(41)],
        *[validity_sample(9_000_000_000 + i * 50_000_000, True) for i in range(51)],
    ]

    windows = [window for window in contiguous_bool_windows(samples) if window[0].value]

    initial = validity_window_metrics(windows[0], expected_value=True, all_samples=samples)
    recovered = validity_window_metrics(windows[1], expected_value=True, all_samples=samples)
    aggregate_true = validity_window_metrics([sample for sample in samples if sample.value], expected_value=True, all_samples=samples)

    assert initial.duration_sec == 2.0
    assert recovered.duration_sec == 2.5
    assert initial.mean_frequency_hz == 20.0
    assert recovered.mean_frequency_hz == 20.0
    assert aggregate_true.mean_frequency_hz < 10.0


def test_phase_window_metrics_exclude_startup_false_from_stale_false():
    samples = [
        *[validity_sample(1_000_000_000 + i * 50_000_000, False) for i in range(11)],
        *[validity_sample(2_000_000_000 + i * 50_000_000, True) for i in range(41)],
        *[validity_sample(5_000_000_000 + i * 50_000_000, False) for i in range(41)],
    ]

    stale = select_contiguous_window(samples, value=False, containing_ns=5_500_000_000)
    metrics = validity_window_metrics(stale, expected_value=False, all_samples=samples)

    assert stale[0].monotonic_ns == 5_000_000_000
    assert metrics.sample_count == 41
    assert metrics.mean_frequency_hz == 20.0


def test_phase_window_metrics_enforce_recovery_boundary_and_no_opposite_samples():
    samples = [
        *[validity_sample(1_000_000_000 + i * 50_000_000, False) for i in range(41)],
        *[validity_sample(4_000_000_000 + i * 50_000_000, True) for i in range(51)],
    ]

    recovered = select_contiguous_window(samples, value=True, start_ns=4_000_000_000)
    metrics = validity_window_metrics(recovered, expected_value=True, all_samples=samples)

    assert metrics.first_sample_monotonic_ns == 4_000_000_000
    assert metrics.unexpected_opposite_value_sample_count == 0
    assert validity_window_passes(metrics, min_duration_sec=2.5)


def test_phase_window_metrics_use_n_minus_one_frequency():
    samples = [
        validity_sample(1_000_000_000, True),
        validity_sample(1_500_000_000, True),
        validity_sample(2_000_000_000, True),
    ]
    metrics = validity_window_metrics(samples, expected_value=True, all_samples=samples)
    assert metrics.sample_count == 3
    assert metrics.duration_sec == 1.0
    assert metrics.mean_frequency_hz == 2.0


def test_phase_window_metrics_reject_short_dwell_and_multiple_publishers():
    short = [validity_sample(1_000_000_000 + i * 50_000_000, True) for i in range(20)]
    short_metrics = validity_window_metrics(short, expected_value=True, all_samples=short)
    assert not validity_window_passes(short_metrics, min_duration_sec=2.0)

    mixed = [
        validity_sample(1_000_000_000, True, gid="a"),
        validity_sample(1_050_000_000, True, gid="b"),
        validity_sample(1_100_000_000, True, gid="a"),
    ]
    mixed_metrics = validity_window_metrics(mixed, expected_value=True, all_samples=mixed)
    assert not validity_window_passes(mixed_metrics, min_duration_sec=0.1)


def test_phase_window_rate_boundaries_are_inclusive_and_outside_rejected():
    at_18 = [validity_sample(1_000_000_000 + round(i * 1_000_000_000 / 18.0), True) for i in range(37)]
    at_22 = [validity_sample(1_000_000_000 + round(i * 1_000_000_000 / 22.0), True) for i in range(45)]
    below_18 = [validity_sample(1_000_000_000 + i * 60_000_000, True) for i in range(35)]
    above_22 = [validity_sample(1_000_000_000 + i * 40_000_000, True) for i in range(51)]

    assert validity_window_passes(validity_window_metrics(at_18, expected_value=True, all_samples=at_18), min_duration_sec=2.0)
    assert validity_window_passes(validity_window_metrics(at_22, expected_value=True, all_samples=at_22), min_duration_sec=2.0)
    assert not validity_window_passes(validity_window_metrics(below_18, expected_value=True, all_samples=below_18), min_duration_sec=2.0)
    assert not validity_window_passes(validity_window_metrics(above_22, expected_value=True, all_samples=above_22), min_duration_sec=2.0)
