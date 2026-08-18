"""Publication-path-aware qualification adapter for S02, S03, and C01."""
from __future__ import annotations

import json
from pathlib import Path

from parking_robot_bringup.phase4_p4e6b_live_evidence import (
    _jsonl, _mission_identity,
)

REMAINING_CASES = {
    "S02": {"case_id": "B-S02", "expected_reason": "ODOMETRY_STALE",
            "threshold_sec": .50, "age_field": "odometry_age_sec",
            "receipt_field": "odometry_receipt_steady_sec"},
    "S03": {"case_id": "B-S03", "expected_reason": "TF_STALE",
            "threshold_sec": .50, "age_field": "tf_age_sec",
            "receipt_field": "tf_receipt_steady_sec"},
    "C01": {"case_id": "B-C01", "expected_reason": "COMMAND_PAIR_STALE",
            "threshold_sec": .25, "age_field": None, "receipt_field": None},
}


def _config(case_name: str) -> dict:
    try:
        return dict(REMAINING_CASES[str(case_name).upper()])
    except KeyError as exc:
        raise RuntimeError("REMAINING_HEALTH_CASE_DENIED") from exc


def _values(row: dict) -> dict:
    values = row.get("values", {})
    if not isinstance(values, dict):
        raise RuntimeError("GENUINE_RUNNER_DIAGNOSTIC_INTEGRITY_FAILURE:values")
    result = dict(values)
    for key, value in list(result.items()):
        if key.endswith("_age_sec") or key.endswith("_receipt_steady_sec") or key == "raw_safe_skew_sec":
            if value not in (None, "none"):
                try:
                    result[key] = float(value)
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(f"GENUINE_RUNNER_DIAGNOSTIC_INTEGRITY_FAILURE:{key}") from exc
    if result.get("feedback_sample_count") not in (None, "none"):
        result["feedback_sample_count"] = int(result["feedback_sample_count"])
    return result


def _policy_authority(output_dir: Path, events: list[dict], case_name: str) -> dict:
    config = _config(case_name)
    diagnostics = _jsonl(Path(output_dir) / "mission_policy_diagnostics.jsonl")
    states = _jsonl(Path(output_dir) / "mission_state_events.jsonl")
    statuses = _jsonl(Path(output_dir) / "navigate_action_status_events.jsonl")
    mission_id, route_id, uuid = _mission_identity(states, events)
    relevant = []
    for row in diagnostics:
        values = _values(row)
        if (values.get("mission_id") == mission_id and values.get("route_id") == route_id
                and values.get("active_goal_uuid") == uuid):
            relevant.append((row, values))
    reason = config["expected_reason"]
    transitions = [(row, values) for row, values in relevant
                   if values.get("progress_supervisor_event") == reason]
    if not transitions:
        raise RuntimeError(f"REMAINING_CASE_AGE_EVIDENCE_MISSING:{reason}")
    first_row, first_values = transitions[0]
    previous = [(row, values) for row, values in relevant
                if int(row.get("monotonic_ns", 0)) < int(first_row.get("monotonic_ns", 0))]
    if any(values.get("progress_supervisor_event") == reason for _, values in previous):
        raise RuntimeError(f"REMAINING_CASE_EARLY_{reason}")
    branch = {"event": reason, "threshold_sec": config["threshold_sec"]}
    if config["age_field"]:
        age = first_values.get(config["age_field"])
        if age is None or float(age) <= config["threshold_sec"]:
            raise RuntimeError(f"REMAINING_CASE_THRESHOLD_FAILURE:{reason}")
        passive = [(row, values) for row, values in previous
                   if values.get(config["receipt_field"]) not in (None, "none")]
        if not passive:
            raise RuntimeError(f"REMAINING_CASE_RECEIPT_PROVENANCE_MISSING:{reason}")
        branch.update({"age_field": config["age_field"], "age_sec": float(age),
                       "receipt_field": config["receipt_field"],
                       "last_passive_receipt_steady_sec": passive[-1][1][config["receipt_field"]],
                       "passive_rows_before_transition": len(passive)})
    else:
        raw_age = first_values.get("raw_command_age_sec")
        safe_age = first_values.get("safe_command_age_sec")
        skew = first_values.get("raw_safe_skew_sec")
        pair_age = first_values.get("pair_liveness_age_sec")
        phase = first_values.get("command_stream_phase")
        candidates = [x for x in (raw_age, safe_age, skew) if x is not None]
        branch_proven = any(float(x) > config["threshold_sec"] for x in candidates)
        if not branch_proven and phase in ("ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW", "NONE"):
            branch_proven = pair_age is not None and float(pair_age) >= config["threshold_sec"]
        if not branch_proven:
            raise RuntimeError("REMAINING_CASE_COMMAND_PAIR_BRANCH_UNPROVEN")
        branch.update({"raw_command_age_sec": raw_age, "safe_command_age_sec": safe_age,
                       "raw_safe_skew_sec": skew, "pair_liveness_age_sec": pair_age,
                       "command_stream_phase": phase})
    owned_statuses = [row for row in statuses if row.get("goal_uuid") == uuid]
    canceling = [row for row in owned_statuses if int(row.get("status", -1)) == 3]
    if len(canceling) != 1:
        raise RuntimeError("REMAINING_CASE_CANCELING_CARDINALITY_FAILURE")
    mission_events = [row for row in states if row.get("mission_id") == mission_id and row.get("route_id") == route_id]
    origin = [row for row in mission_events if row.get("active_goal_uuid") == uuid
              and row.get("reason_code") == reason and row.get("state_name") == "CANCELLING"]
    ack = [row for row in mission_events if row.get("active_goal_uuid") == uuid
           and row.get("reason_code") == "HEALTH_CANCEL_ACK_ACCEPTED"]
    if len(origin) != 1 or len(ack) != 1:
        raise RuntimeError("REMAINING_CASE_ORIGIN_OR_ACK_FAILURE")
    return {"case_name": str(case_name).upper(), "case_id": config["case_id"],
            "expected_reason": reason, "mission_id": mission_id, "route_id": route_id,
            "active_goal_uuid": uuid, "first_transition_monotonic_ns": int(first_row["monotonic_ns"]),
            "first_transition_policy_event": reason, "branch": branch,
            "originating_reason": reason, "cancel_count": 1, "duplicate_cancel": 0,
            "canceling_status": 3, "health_cancel_ack": "HEALTH_CANCEL_ACK_ACCEPTED",
            "mission_state_events": mission_events, "action_status_events": owned_statuses}


def load_genuine_remaining_case_evidence(output_dir: Path, case_name: str) -> dict:
    config = _config(case_name)
    out = Path(output_dir)
    events = _jsonl(out / "health_campaign.jsonl")
    metrics = json.loads((out / "terminal_metrics.json").read_text(encoding="utf-8"))
    injected = [row for row in events if row.get("event") == "INJECTED"]
    if len(injected) != 1:
        raise RuntimeError("GENUINE_RUNNER_INJECTED_CARDINALITY_FAILURE")
    row = injected[0]
    result = {"case_name": str(case_name).upper(), "case_id": config["case_id"],
              "attempt_started": any(r.get("event") == "ATTEMPT_STARTED" for r in events),
              "premission_health_ready": any(r.get("event") == "PREMISSION_HEALTH_READY" for r in events),
              "armed_for_case": any(r.get("event") == "ARMED_FOR_CASE" for r in events),
              "injection_request_count": 1, "suppression_request_count": 1,
              "request_ns": row.get("request_ns"), "ack_ns": row.get("ack_ns"),
              "injection_response_successful": row.get("success") is True,
              "injection_response_reason": row.get("tool_state"),
              "relay_injection_count": row.get("injection_count"), "tool_state": row.get("tool_state"),
              "terminal_metrics": metrics, "events": events}
    result.update(_policy_authority(out, events, case_name))
    if not result["attempt_started"] or not result["armed_for_case"]:
        raise RuntimeError("GENUINE_RUNNER_EVIDENCE_INTEGRITY_FAILURE")
    injected_case_ids = {r.get("case_id") for r in injected}
    armed_case_ids = {r.get("case_id") for r in events if r.get("event") == "ARMED_FOR_CASE"}
    if (not result["injection_response_successful"]
            or result["relay_injection_count"] != 1
            or injected_case_ids != {config["case_id"]}
            or armed_case_ids != {config["case_id"]}):
        raise RuntimeError("GENUINE_RUNNER_INJECTION_FAILURE")
    return result
