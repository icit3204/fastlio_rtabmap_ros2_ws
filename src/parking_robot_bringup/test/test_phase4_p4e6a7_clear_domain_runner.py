from pathlib import Path

from parking_robot_bringup.phase4_p4e6a7_clear_domain_runner import (
    CALLBACK_DRAIN_SEC,
    EvidenceRecordError,
    FORMAL_READINESS_EPOCH_SEC,
    domain,
    epoch_health,
    formal_entry_health,
    interval_rate,
    prospective_epoch,
    authoritative_goal_record,
    adjudicate_command_pairs,
    priming_for_goal,
    summarize,
)

from parking_robot_interfaces.msg import MissionState


ROOT = Path(__file__).parents[1]
RUNNER = ROOT / "parking_robot_bringup" / "phase4_p4e6a7_clear_domain_runner.py"


def row(v, w, frame="base_footprint"):
    return (1, 2, v, 0.0, 0.0, 0.0, 0.0, w, frame, "gid", "/guarded_vehicle_cmd_gate")


def test_exact_domain_boundary_and_rejections():
    assert domain(row(0.2, 0.2), stamped=True)["valid"]
    assert domain(row(0.0, 0.0), stamped=True)["valid"]
    assert not domain(row(0.2, 0.2000001), stamped=True)["valid"]
    assert not domain(row(-0.01, 0.0), stamped=True)["valid"]
    assert not domain(row(0.2, 0.0, "map"), stamped=True)["valid"]


def test_summary_does_not_round_before_adjudication():
    stats = summarize([row(0.1, 0.1), row(0.1, 0.10000000001)], stamped=True)
    assert stats["invalid"] == 1
    assert stats["minimum_margin"] < 0


def test_runner_is_clear_only_passive_mission_client():
    text = RUNNER.read_text()
    assert "ActionClient" not in text
    assert 'node.mode("STOP")' not in text
    assert 'node.mode("SLOWDOWN")' not in text
    assert 'node.trigger("cancel")' not in text
    assert "create_publisher(Twist" not in text


def test_runner_stops_on_first_adapter_invalid_and_checks_loaded_profile():
    text = RUNNER.read_text()
    assert "P4E6A7B_ADAPTER_INVALID_RECURRED_NEEDS_REVIEW" in text
    assert 'bad = next((x for x in fresh' in text
    assert '"FollowPath.AckermannConstraints.min_turning_r"' in text
    assert 'behavior["behavior_plugins"] != ["wait"]' in text
    assert '"default_nav_through_poses_bt_xml"' in text
    assert 'x["level"] != 0' in text


def test_runner_requires_two_successful_distinct_waypoints():
    text = RUNNER.read_text()
    assert "len(uuid0s) != 1" in text
    assert "len(uuid1s) != 1" in text
    assert "GoalStatus.STATUS_SUCCEEDED" in text
    assert 'terminal["completed"] != 2' in text


def stamps(rate, seconds, start=0):
    step = round(1e9 / rate)
    return [start + i * step for i in range(round(rate * seconds) + 1)]


def test_interval_count_rate_boundaries_and_rejections():
    assert interval_rate([0, 1_000_000_000]) == 1.0
    assert abs(interval_rate(stamps(20, 2)) - 20.0) < 1e-6
    assert abs(interval_rate(stamps(50, 2)) - 50.0) < 1e-6
    assert abs(interval_rate(stamps(48, 2)) - 48.0) < 1e-5
    assert abs(interval_rate(stamps(52, 2)) - 52.0) < 1e-5
    assert interval_rate(stamps(47, 2)) < 48.0
    assert interval_rate(stamps(53, 2)) > 52.0
    assert interval_rate([]) is None
    assert interval_rate([1]) is None
    assert interval_rate([1, 1]) is None
    assert interval_rate([2, 1]) is None


def test_backlog_drain_is_excluded_from_prospective_epoch():
    drain = [100_000_000, 105_000_000, 110_000_000, 600_000_000]
    formal = stamps(50, 2, start=1_000_000_000)
    result = prospective_epoch(drain + formal, 1_000_000_000, 3_000_000_000,
                               minimum_duration_sec=2.0, lower_hz=48.0,
                               upper_hz=52.0, large_gap_sec=.05)
    assert result["passes"]
    assert abs(result["rate_hz"] - 50.0) < 1e-6
    assert result["sample_count"] == len(formal)


def test_true_out_of_band_formal_epochs_fail_without_reselection():
    fast = prospective_epoch(stamps(53, 2), 0, 2_000_000_000,
                             minimum_duration_sec=2.0, lower_hz=48.0,
                             upper_hz=52.0, large_gap_sec=.05)
    slow = prospective_epoch(stamps(47, 2), 0, 2_000_000_000,
                             minimum_duration_sec=2.0, lower_hz=48.0,
                             upper_hz=52.0, large_gap_sec=.05)
    assert not fast["passes"] and fast["rate_hz"] > 52.0
    assert not slow["passes"] and slow["rate_hz"] < 48.0
    # A later passing slice is deliberately irrelevant to the fixed failed epoch.
    assert prospective_epoch(stamps(50, 2, 3_000_000_000), 3_000_000_000,
                             5_000_000_000, minimum_duration_sec=2.0,
                             lower_hz=48.0, upper_hz=52.0,
                             large_gap_sec=.05)["passes"]


def test_short_epoch_is_insufficient_even_with_nominal_samples():
    result = prospective_epoch(stamps(50, 1.9), 0, 1_900_000_000,
                               minimum_duration_sec=2.0, lower_hz=48.0,
                               upper_hz=52.0, large_gap_sec=.05)
    assert not result["sufficient"]
    assert not result["passes"]


def diagnostic(ns, level):
    return {"monotonic_ns": ns, "level": level}


def test_adapter_warn_before_epoch_is_retained_but_not_debounced_into_epoch():
    history = [diagnostic(500, 1), diagnostic(1100, 0), diagnostic(1900, 0)]
    health = [{"monotonic_ns": 1100, "valid": True}, {"monotonic_ns": 1900, "valid": True}]
    assert epoch_health(history, health, 1000, 2000)["passes"]
    assert history[0]["level"] == 1


def test_adapter_warn_or_health_failure_in_formal_epoch_fails_once():
    warning = epoch_health([diagnostic(1100, 0), diagnostic(1500, 1)],
                           [{"monotonic_ns": 1200, "valid": True}], 1000, 2000)
    invalid = epoch_health([diagnostic(1100, 0), diagnostic(1900, 0)],
                           [{"monotonic_ns": 1200, "valid": True},
                            {"monotonic_ns": 1600, "valid": False}], 1000, 2000)
    assert not warning["passes"] and warning["adapter_warn_count"] == 1
    assert not invalid["passes"] and invalid["health_invalid_count"] == 1


def test_runner_has_strict_query_drain_formal_epoch_order_and_no_retry_loop():
    text = RUNNER.read_text()
    assert CALLBACK_DRAIN_SEC >= 1.0
    assert FORMAL_READINESS_EPOCH_SEC >= 2.0
    query = text.index("last_synchronous_query_complete_ns =")
    drain_start = text.index("callback_drain_start_ns =", query)
    drain_end = text.index("callback_drain_end_ns =", drain_start)
    formal_start = text.index("formal_readiness_epoch_start_ns =", drain_end)
    formal_end = text.index("formal_readiness_epoch_end_ns =", formal_start)
    adjudication = text.index('"gate": prospective_epoch', formal_end)
    assert query < drain_start < drain_end < formal_start < formal_end < adjudication
    assert "while not all(x[\"passes\"]" not in text


def test_preserved_p4e6a7b_rate_replay_keeps_formula_unchanged():
    def evenly_spaced(first, last, count):
        return [first + (last-first) * i // (count-1) for i in range(count)]

    old_odom = evenly_spaced(459138042297906, 459140236856677, 117)
    old_tf = evenly_spaced(459138043088979, 459140217696550, 116)
    assert abs(interval_rate(old_odom) - 52.8580056879) < 1e-10
    assert abs(interval_rate(old_tf) - 52.8831047650) < 1e-10
    # Final preserved settled-epoch values are a replay oracle, not a searched
    # runtime window. Future collection selects its epoch prospectively.
    assert abs(100 / 50.1124667828 - 1.995511425) < 1e-9
    assert abs(100 / 50.0540502906 - 1.997840323) < 1e-9


def adapter_record(ns=1_900_000_000, level=0, fields=None):
    return {"monotonic_ns": ns, "level": level, "message": "VALID",
            "values": fields or {"input_publisher_count": "1", "output_publisher_count": "1"}}


def test_exact_gate_tuple_executes_failed_formal_entry_path():
    gate = ("DISARMED", {"state": "DISARMED", "fault_latched": "false"}, 1_900_000_000)
    permissions = [(True, 1_900_000_000)] * 3
    assert formal_entry_health(2_000_000_000, gate, permissions, adapter_record())
    assert not formal_entry_health(2_500_000_001, gate, permissions, adapter_record())


def test_actual_callback_storage_shapes_enter_formal_readiness():
    # diagnostics() stores diag_conditions as (message, fields, monotonic_ns),
    # diagnostic_history as a mapping, and permission() as (bool, monotonic_ns).
    gate = ("DISARMED", {"state": "DISARMED", "fault_latched": "false"}, 9_900)
    adapter = adapter_record(9_900)
    permissions = [(True, 9_900), (True, 9_900), (True, 9_900)]
    assert formal_entry_health(10_000, gate, permissions, adapter)


def test_malformed_readiness_records_fail_closed_without_builtin_exceptions():
    valid_gate = ("DISARMED", {"state": "DISARMED", "fault_latched": "false"}, 100)
    valid_permissions = [(True, 100)] * 3
    malformed = [
        (("DISARMED", {}), valid_permissions, adapter_record(100)),
        (("DISARMED", {}, "100"), valid_permissions, adapter_record(100)),
        (("DISARMED", {}, None), valid_permissions, adapter_record(100)),
        (("DISARMED", "fields", 100), valid_permissions, adapter_record(100)),
        ((7, {}, 100), valid_permissions, adapter_record(100)),
        (valid_gate, valid_permissions, {}),
    ]
    for gate, permissions, adapter in malformed:
        try:
            formal_entry_health(200, gate, permissions, adapter)
        except EvidenceRecordError:
            pass
        else:
            raise AssertionError("malformed record did not fail closed")


UUID_A = "1a40296d8f4840bbbeb9b883c9c49c51"
UUID_B = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"


def state(ns, code=MissionState.NAVIGATING, uuid="", waypoint=0, completed=0):
    return {"monotonic_ns": ns, "state": code, "active_goal_uuid": uuid,
            "waypoint_index": waypoint, "completed": completed,
            "mission_id": "mission", "route_id": "route"}


def policy(ns, activation, uuid):
    return {"monotonic_ns": ns, "level": 0, "message": "MISSION_PROGRESS_POLICY",
            "values": {"progress_policy_activation_state": activation,
                       "active_goal_uuid": uuid}}


def acquire(rows, waypoint=0, prior=()):
    return authoritative_goal_record(
        rows, after_ns=0, waypoint_index=waypoint, mission_id="mission",
        route_id="route", prior_uuids=prior)


def test_p4e6a7g_empty_uuid_early_navigating_regression():
    rows = [state(10), state(20, uuid=UUID_A)]
    bound = acquire(rows)
    assert bound["active_goal_uuid"] == UUID_A
    assert priming_for_goal([policy(15, "INITIAL_PRIMING", UUID_A)],
                            after_ns=0, goal_uuid=UUID_A)


def test_cross_topic_order_and_duplicate_empty_states_are_accepted():
    rows = [state(10), state(11), state(20, uuid=UUID_A)]
    assert acquire(rows)["active_goal_uuid"] == UUID_A
    # A diagnostic may be received before the MissionState carrying the UUID.
    assert priming_for_goal([policy(15, "INITIAL_PRIMING", UUID_A)],
                            after_ns=0, goal_uuid=UUID_A)


def test_terminal_before_uuid_fails_fast_and_missing_uuid_stays_unbound():
    try:
        acquire([state(10), state(20, MissionState.FAILED)])
    except EvidenceRecordError as exc:
        assert "terminated before" in str(exc)
    else:
        raise AssertionError("terminal was misclassified as UUID timeout")
    assert acquire([state(10), state(20)]) is None


def test_duplicate_uuid_counts_once_but_conflicting_uuid_fails():
    assert acquire([state(10, uuid=UUID_A), state(20, uuid=UUID_A)])["active_goal_uuid"] == UUID_A
    try:
        acquire([state(10, uuid=UUID_A), state(20, uuid=UUID_B)])
    except EvidenceRecordError as exc:
        assert "multiple authoritative UUIDs" in str(exc)
    else:
        raise AssertionError("conflicting UUID was silently rebound")


def test_priming_requires_exact_bound_nonempty_uuid():
    rows = [policy(10, "INITIAL_PRIMING", ""),
            policy(20, "INITIAL_PRIMING", UUID_B),
            policy(30, "INITIAL_PRIMING", UUID_A)]
    assert priming_for_goal(rows[:2], after_ns=0, goal_uuid=UUID_A) is None
    assert priming_for_goal(rows, after_ns=0, goal_uuid=UUID_A)["monotonic_ns"] == 30
    try:
        priming_for_goal(rows, after_ns=0, goal_uuid="")
    except EvidenceRecordError:
        pass
    else:
        raise AssertionError("empty UUID satisfied priming")


def test_sequential_waypoint_skips_stale_uuid_and_binds_new_generation():
    rows = [state(10, uuid=UUID_A),
            state(20, MissionState.PLANNING, uuid="", waypoint=1, completed=1),
            state(30, uuid="", waypoint=1, completed=1),
            state(31, uuid=UUID_A, waypoint=1, completed=1),
            state(40, uuid=UUID_B, waypoint=1, completed=1),
            state(50, uuid=UUID_B, waypoint=1, completed=1)]
    bound = acquire(rows, waypoint=1, prior={UUID_A})
    assert bound["active_goal_uuid"] == UUID_B
    assert UUID_B != UUID_A
    assert priming_for_goal([policy(45, "ACTIVE", UUID_B)],
                            after_ns=0, goal_uuid=UUID_B) is None


def test_final_empty_uuid_and_duplicate_publications_do_not_add_goal():
    rows = [state(10, uuid=UUID_A), state(11, uuid=UUID_A),
            state(20, uuid=UUID_B, waypoint=1, completed=1),
            state(21, uuid=UUID_B, waypoint=1, completed=1),
            state(30, MissionState.SUCCEEDED, uuid="", waypoint=1, completed=2)]
    assert acquire(rows)["active_goal_uuid"] == UUID_A
    assert acquire(rows, waypoint=1, prior={UUID_A})["active_goal_uuid"] == UUID_B


def test_malformed_uuid_record_fails_closed():
    malformed = state(10)
    malformed["active_goal_uuid"] = None
    try:
        acquire([malformed])
    except EvidenceRecordError:
        pass
    else:
        raise AssertionError("malformed UUID did not fail closed")


def command(ns, v=0.1, w=0.05):
    return (ns, ns, v, 0.0, 0.0, 0.0, 0.0, w, "base_footprint")


def test_p4e6a7i_old_pair_invocation_exception_and_repaired_path():
    from parking_robot_bringup.phase4_p4e2a_slowdown_runner import pair_commands
    raw = [command(100)]
    safe = [command(110)]
    try:
        pair_commands(raw, safe)
    except TypeError as exc:
        assert "start_ns" in str(exc) and "end_ns" in str(exc)
    else:
        raise AssertionError("old two-argument invocation did not reproduce")
    result = adjudicate_command_pairs(raw, safe, 90, 120)
    assert result["pair_count"] == 1
    assert result["invalid_safe_count"] == 0


def test_pair_adjudication_boundaries_and_fail_closed_contract():
    valid = command(100)
    for start, end in ((None, 200), (0, None), (100, 100), (101, 100)):
        try:
            adjudicate_command_pairs([valid], [valid], start, end)
        except EvidenceRecordError:
            pass
        else:
            raise AssertionError("invalid bounds did not fail closed")
    assert adjudicate_command_pairs([], [], 0, 200)["pair_count"] == 0
    assert adjudicate_command_pairs([valid], [], 0, 200)["pair_count"] == 0
    assert adjudicate_command_pairs([], [valid], 0, 200)["pair_count"] == 0
    assert adjudicate_command_pairs([command(300)], [command(300)], 0, 200)["pair_count"] == 0


def test_pair_adjudication_multiple_unpairable_and_invalid_safe():
    raw = [command(100), command(200)]
    safe = [command(110), command(210), command(400_000_000), command(205, v=0.1, w=0.11)]
    result = adjudicate_command_pairs(raw, safe, 0, 500_000_000)
    assert result["pair_count"] == 3
    assert result["invalid_safe_count"] == 1
    assert result["safe_count"] == 4


def test_pair_adjudication_malformed_timestamp_fails_closed():
    malformed = list(command(100)); malformed[0] = "100"
    try:
        adjudicate_command_pairs([tuple(malformed)], [command(100)], 0, 200)
    except EvidenceRecordError:
        pass
    else:
        raise AssertionError("malformed timestamp did not fail closed")


def test_complete_post_terminal_adjudication_path_with_actual_record_shapes():
    states = [state(10), state(20, uuid=UUID_A),
              state(30, MissionState.PLANNING, waypoint=1, completed=1),
              state(40, uuid="", waypoint=1, completed=1),
              state(50, uuid=UUID_B, waypoint=1, completed=1),
              state(100, MissionState.SUCCEEDED, waypoint=2, completed=2)]
    policies = [policy(15, "INITIAL_PRIMING", UUID_A),
                policy(60, "ACTIVE", UUID_B)]
    waypoint0 = acquire(states)
    assert priming_for_goal(policies, after_ns=0,
                            goal_uuid=waypoint0["active_goal_uuid"])
    waypoint1 = acquire(states, waypoint=1, prior={UUID_A})
    assert waypoint1["active_goal_uuid"] == UUID_B
    assert policies[-1]["values"]["progress_policy_activation_state"] == "ACTIVE"
    raw = [command(20), command(50)]
    safe = [command(21), command(51)]
    result = adjudicate_command_pairs(raw, safe, 10, 100)
    assert result["pair_count"] == 2
    assert result["invalid_safe_count"] == 0
    assert summarize(raw)["invalid"] == 0
    assert summarize(safe)["invalid"] == 0
    assert states[-1]["state"] == MissionState.SUCCEEDED
    assert states[-1]["completed"] == 2
