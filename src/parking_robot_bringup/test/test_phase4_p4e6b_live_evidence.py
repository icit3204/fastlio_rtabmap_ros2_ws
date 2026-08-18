import hashlib
import json

import pytest

from parking_robot_bringup.phase4_p4e6b_live_evidence import (
    _command_row, load_genuine_runner_evidence, load_genuine_witness_outcome,
    post_zero_angular_zero, require_feedback_stale_age,
)


def _write_jsonl(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def _witness(tmp_path, *, bad_hash=False, drain=.251, translation=0.001,
             canceled=True):
    out = tmp_path / "witness"
    out.mkdir(parents=True)
    mission = [
        {"monotonic_ns": 110, "mission_id": "m", "route_id": "r", "waypoint_index": 1,
         "active_goal_uuid": "u", "state_name": "CANCELLING", "reason_code": "FEEDBACK_STALE"},
        {"monotonic_ns": 120, "mission_id": "m", "route_id": "r", "waypoint_index": 1,
         "active_goal_uuid": "u", "state_name": "CANCELLING", "reason_code": "HEALTH_CANCEL_ACK_ACCEPTED"},
        {"monotonic_ns": 130, "mission_id": "m", "route_id": "r", "waypoint_index": 1,
         "active_goal_uuid": "u", "state_name": "FAILED", "reason_code": "FEEDBACK_STALE"},
    ]
    statuses = [{"monotonic_ns": 125, "goal_uuid": "u", "status_name": "CANCELING"}]
    if canceled: statuses.append({"monotonic_ns": 140, "goal_uuid": "u", "status_name": "CANCELED"})
    files = {
        "mission_state_events.jsonl": mission,
        "navigate_action_status_events.jsonl": statuses,
        "terminal_witness_events.jsonl": [{"event": "FAILED_RECEIVED", "monotonic_ns": 130}],
        "raw_cmd_timeline.jsonl": [{"monotonic_ns": 90, "linear_x": .1, "linear_y": 0, "linear_z": 0, "angular_x": 0, "angular_y": 0, "angular_z": 0}],
        "safe_cmd_timeline.jsonl": [{"monotonic_ns": 90, "linear_x": .1, "linear_y": 0, "linear_z": 0, "angular_x": 0, "angular_y": 0, "angular_z": 0}],
        "vehicle_cmd_safe_timeline.jsonl": [
            {"monotonic_ns": 90, "linear_x": .1, "linear_y": 0, "linear_z": 0, "angular_x": 0, "angular_y": 0, "angular_z": 0},
            {"monotonic_ns": 200, "linear_x": 0, "linear_y": 0, "linear_z": 0, "angular_x": 0, "angular_y": 0, "angular_z": 0}],
        "adapter_output_timeline.jsonl": [
            {"monotonic_ns": 90, "values": [.1, 0]},
            {"monotonic_ns": 210, "values": [0, 0]}],
        "odometry_timeline.jsonl": [{"monotonic_ns": 220, "x": 0, "y": 0, "yaw": 0},
                                     {"monotonic_ns": 230000000, "x": translation, "y": 0, "yaw": .5}],
        "gate_diagnostics.jsonl": [{"monotonic_ns": 220, "message": "healthy"}],
    }
    hashes = {}
    for name, rows in files.items():
        _write_jsonl(out / name, rows)
        hashes[name] = hashlib.sha256((out / name).read_bytes()).hexdigest()
    start, end = 1000, 1000 + int(drain * 1e9)
    marker = {"terminal_identity": {"mission_id": "m", "route_id": "r", "waypoint": 1, "uuid": "u"},
              "reason": "FEEDBACK_STALE", "FAILED_RECEIVED_NS": 130,
              "DRAIN_START_NS": start, "DRAIN_END_NS": end, "ACTUAL_DRAIN_SEC": drain,
              "file_hashes": hashes, "terminal_evidence_complete": True,
              "physical_evidence_complete": True}
    if bad_hash: marker["file_hashes"]["mission_state_events.jsonl"] = "bad"
    (out / "TERMINAL_WITNESS_OUTCOME_COMMITTED").write_text(json.dumps(marker))
    return out


def test_genuine_witness_loader_and_command_mapping(tmp_path):
    outcome = load_genuine_witness_outcome(_witness(tmp_path))
    assert outcome["actual_drain_sec"] >= .250
    assert outcome["statuses"][1]["status_name"] == "CANCELED"
    assert outcome["applied_rows"][0]["linear_x"] == .1
    assert post_zero_angular_zero(outcome, 200)


def test_witness_hash_mismatch_fails_closed(tmp_path):
    with pytest.raises(RuntimeError, match="INTEGRITY_FAILURE"):
        load_genuine_witness_outcome(_witness(tmp_path, bad_hash=True))


def test_witness_missing_canceled_and_short_drain_fail_closed(tmp_path):
    with pytest.raises(RuntimeError): load_genuine_witness_outcome(_witness(tmp_path, canceled=False))
    with pytest.raises(RuntimeError): load_genuine_witness_outcome(_witness(tmp_path / "short", drain=.2))


def test_runner_loader_requires_genuine_files(tmp_path):
    with pytest.raises(RuntimeError, match="GENUINE_RUNNER_EVIDENCE"):
        load_genuine_runner_evidence(tmp_path)


def _runner_fixture(tmp_path, age=2.015479, event="FEEDBACK_STALE", previous=1.943586,
                    statuses=(2, 3), uuid="u", status_uuid=None):
    status_uuid = uuid if status_uuid is None else status_uuid
    out = tmp_path / "runner"
    out.mkdir(parents=True)
    _write_jsonl(out / "health_campaign.jsonl", [
        {"event": "ATTEMPT_STARTED", "monotonic_ns": 10},
        {"event": "ARMED_FOR_CASE", "goal_uuid": uuid, "monotonic_ns": 20},
        {"event": "INJECTED", "request_ns": 30, "ack_ns": 31,
         "success": True, "tool_state": "INJECTED", "injection_count": 1},
    ])
    values = {"mission_id": "m", "route_id": "r", "active_goal_uuid": "u",
              "feedback_age_sec": f"{age:.6f}", "feedback_receipt_steady_sec": "10.0",
              "feedback_sample_count": "4", "progress_supervisor_event": event}
    _write_jsonl(out / "mission_policy_diagnostics.jsonl", [
        {"monotonic_ns": 100, "ros_ns": 1000, "values": {**values, "feedback_age_sec": f"{previous:.6f}", "progress_supervisor_event": "NO_PROGRESS_PENDING"}},
        {"monotonic_ns": 200, "ros_ns": 2000, "values": values},
    ])
    _write_jsonl(out / "mission_state_events.jsonl", [
        {"monotonic_ns": 150, "mission_id": "m", "route_id": "r", "active_goal_uuid": uuid, "state_name": "CANCELLING", "reason_code": "FEEDBACK_STALE"},
        {"monotonic_ns": 160, "mission_id": "m", "route_id": "r", "active_goal_uuid": uuid, "state_name": "CANCELLING", "reason_code": "HEALTH_CANCEL_ACK_ACCEPTED"},
    ])
    _write_jsonl(out / "navigate_action_status_events.jsonl", [
        {"monotonic_ns": 155 + i, "goal_uuid": status_uuid, "status": status} for i, status in enumerate(statuses)])
    (out / "terminal_metrics.json").write_text(json.dumps({"pass": False, "error": "qualification fixture"}))
    return out


def test_genuine_runner_loader_replays_mission_manager_stale_authority(tmp_path):
    evidence = load_genuine_runner_evidence(_runner_fixture(tmp_path))
    assert evidence["feedback_stale_age_sec"] == 2.015479
    assert evidence["previous_feedback_age_sec"] == 1.943586
    assert evidence["stale_age_authority"] == "MISSION_MANAGER_POLICY_DIAGNOSTIC"
    assert evidence["suppression_request_count"] == evidence["injection_request_count"] == 1
    assert evidence["originating_reason"] == "FEEDBACK_STALE"
    assert evidence["health_cancel_ack"] == "HEALTH_CANCEL_ACK_ACCEPTED"


def test_genuine_runner_loader_rejects_non_strict_stale_age(tmp_path):
    for age in (1.999999, 2.0):
        with pytest.raises(RuntimeError, match="STALE_AGE_THRESHOLD_FAILURE"):
            load_genuine_runner_evidence(_runner_fixture(tmp_path / str(age), age=age))


def test_canceling_enum_uses_status_three_not_executing(tmp_path):
    evidence = load_genuine_runner_evidence(_runner_fixture(tmp_path))
    assert evidence["executing_count"] == 1
    assert evidence["cancel_count"] == 1
    assert evidence["canceling_status"] == 3
    with pytest.raises(RuntimeError, match="CANCELING"):
        load_genuine_runner_evidence(_runner_fixture(tmp_path / "executing", statuses=(2,)))
    with pytest.raises(RuntimeError, match="CANCELING"):
        load_genuine_runner_evidence(_runner_fixture(tmp_path / "duplicate", statuses=(3, 3)))
    with pytest.raises(RuntimeError, match="CANCELING"):
        load_genuine_runner_evidence(_runner_fixture(tmp_path / "wrong", statuses=(3,), status_uuid="wrong"))


def test_genuine_runner_without_feedback_age_fails_closed():
    with pytest.raises(RuntimeError, match="STALE_AGE_EVIDENCE_MISSING"):
        require_feedback_stale_age({"injection_response_reason": "INJECTED"})


def test_command_shape_is_strict():
    assert _command_row({"monotonic_ns": 1, "values": [.2, .3]})["angular_z"] == .3
    with pytest.raises(RuntimeError): _command_row({"monotonic_ns": 1, "values": [1]})
