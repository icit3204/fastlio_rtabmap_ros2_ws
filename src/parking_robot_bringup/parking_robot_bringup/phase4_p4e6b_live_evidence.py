"""Strict adapters for genuine P4-E.6B runner and witness evidence.

This module is qualification-side only.  It parses the files written by the
real runner/witness and never creates synthetic live evidence.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import time
from pathlib import Path


REQUIRED_WITNESS_FILES = (
    "mission_state_events.jsonl", "navigate_action_status_events.jsonl",
    "terminal_witness_events.jsonl", "raw_cmd_timeline.jsonl",
    "safe_cmd_timeline.jsonl", "vehicle_cmd_safe_timeline.jsonl",
    "adapter_output_timeline.jsonl", "odometry_timeline.jsonl",
    "gate_diagnostics.jsonl",
)

ACTION_STATUS_NAMES = {
    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
    4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}


def action_status_name(row: dict) -> str:
    if row.get("status_name"):
        return str(row["status_name"])
    try:
        return ACTION_STATUS_NAMES[int(row["status"])]
    except (KeyError, TypeError, ValueError):
        return "UNKNOWN"


def _jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        raise RuntimeError(f"TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:missing:{path.name}")
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            value = json.loads(line)
            if not isinstance(value, dict):
                raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:row")
            rows.append(value)
    return rows


def _diagnostic_values(row: dict) -> dict:
    values = row.get("values", {})
    if not isinstance(values, dict):
        raise RuntimeError("GENUINE_RUNNER_DIAGNOSTIC_INTEGRITY_FAILURE:values")
    result = dict(values)
    for key in ("feedback_age_sec", "feedback_receipt_steady_sec"):
        if key not in result or result[key] in (None, "none"):
            raise RuntimeError(f"STALE_AGE_EVIDENCE_MISSING:{key}")
        try:
            result[key] = float(result[key])
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"GENUINE_RUNNER_DIAGNOSTIC_INTEGRITY_FAILURE:{key}") from exc
    try:
        result["feedback_sample_count"] = int(result["feedback_sample_count"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("STALE_AGE_EVIDENCE_MISSING:feedback_sample_count") from exc
    return result


def _mission_identity(state_rows: list[dict], campaign_events: list[dict]) -> tuple[str, str, str]:
    armed = next((r for r in campaign_events if r.get("event") == "ARMED_FOR_CASE"), {})
    uuid = armed.get("goal_uuid", "")
    candidates = [r for r in state_rows if r.get("active_goal_uuid") == uuid and uuid]
    if not candidates:
        candidates = [r for r in state_rows if r.get("state_name") == "NAVIGATING" and r.get("active_goal_uuid")]
    if not candidates:
        raise RuntimeError("GENUINE_RUNNER_MISSION_IDENTITY_FAILURE")
    row = candidates[0]
    return str(row.get("mission_id", "")), str(row.get("route_id", "")), str(uuid or row.get("active_goal_uuid", ""))


def _mission_manager_stale_authority(out: Path, events: list[dict]) -> dict:
    diagnostics = _jsonl(out / "mission_policy_diagnostics.jsonl")
    states = _jsonl(out / "mission_state_events.jsonl")
    statuses = _jsonl(out / "navigate_action_status_events.jsonl")
    mission_id, route_id, uuid = _mission_identity(states, events)
    relevant = []
    for row in diagnostics:
        try:
            values = _diagnostic_values(row)
        except RuntimeError:
            # Terminal diagnostic rows may intentionally omit the live
            # feedback sample fields; they cannot establish stale age.
            continue
        if (values.get("mission_id") == mission_id and values.get("route_id") == route_id
                and values.get("active_goal_uuid") == uuid):
            relevant.append((row, values))
    stale = [(row, values) for row, values in relevant
             if values.get("progress_supervisor_event") == "FEEDBACK_STALE"]
    if not stale:
        raise RuntimeError("STALE_AGE_EVIDENCE_MISSING:first_feedback_stale_diagnostic")
    stale_ages = {round(float(values["feedback_age_sec"]), 9) for _, values in stale}
    if len(stale_ages) != 1:
        raise RuntimeError("STALE_AGE_DIAGNOSTIC_INCONSISTENT")
    first_row, first_values = stale[0]
    if float(first_values["feedback_age_sec"]) <= 2.0:
        raise RuntimeError("STALE_AGE_THRESHOLD_FAILURE")
    previous = [(row, values) for row, values in relevant if row.get("monotonic_ns", 0) < first_row.get("monotonic_ns", 0)]
    previous_non_stale = [item for item in previous if item[1].get("progress_supervisor_event") != "FEEDBACK_STALE"]
    if not previous_non_stale or float(previous_non_stale[-1][1]["feedback_age_sec"]) > 2.0:
        raise RuntimeError("STALE_AGE_PREVIOUS_DIAGNOSTIC_FAILURE")
    owned_statuses = [r for r in statuses if r.get("goal_uuid") == uuid]
    executing = [r for r in owned_statuses if action_status_name(r) == "EXECUTING"]
    canceling = [r for r in owned_statuses if action_status_name(r) == "CANCELING"]
    if len(canceling) != 1:
        raise RuntimeError("GENUINE_RUNNER_CANCELING_CARDINALITY_FAILURE")
    mission_events = [r for r in states if r.get("mission_id") == mission_id and r.get("route_id") == route_id]
    origin = [r for r in mission_events if r.get("active_goal_uuid") == uuid
              and r.get("reason_code") == "FEEDBACK_STALE"
              and r.get("state_name") == "CANCELLING"]
    ack = [r for r in mission_events if r.get("active_goal_uuid") == uuid
           and r.get("reason_code") == "HEALTH_CANCEL_ACK_ACCEPTED"]
    if len(origin) != 1 or len(ack) != 1:
        raise RuntimeError("GENUINE_RUNNER_HEALTH_CANCEL_EVIDENCE_FAILURE")
    return {
        "mission_id": mission_id, "route_id": route_id, "active_goal_uuid": uuid,
        "feedback_stale_age_sec": float(first_values["feedback_age_sec"]),
        "feedback_receipt_steady_sec": float(first_values["feedback_receipt_steady_sec"]),
        "feedback_sample_count": int(first_values["feedback_sample_count"]),
        "first_feedback_stale_diagnostic_monotonic_ns": int(first_row["monotonic_ns"]),
        "first_feedback_stale_diagnostic_ros_ns": int(first_row.get("ros_ns", 0)),
        "previous_feedback_age_sec": float(previous_non_stale[-1][1]["feedback_age_sec"]),
        "first_feedback_stale_policy_event": "FEEDBACK_STALE",
        "stale_age_authority": "MISSION_MANAGER_POLICY_DIAGNOSTIC",
        "executing_count": len(executing), "canceling_status": 3,
        "originating_reason": "FEEDBACK_STALE", "cancel_count": 1,
        "duplicate_cancel": 0, "health_cancel_ack": "HEALTH_CANCEL_ACK_ACCEPTED",
        "mission_state_events": mission_events, "action_status_events": statuses,
    }


def load_genuine_runner_evidence(output_dir: Path) -> dict:
    out = Path(output_dir)
    campaign = out / "health_campaign.jsonl"
    metrics = out / "terminal_metrics.json"
    if not campaign.is_file() or not metrics.is_file():
        raise RuntimeError("GENUINE_RUNNER_EVIDENCE_INTEGRITY_FAILURE")
    events = _jsonl(campaign)
    terminal_metrics = json.loads(metrics.read_text(encoding="utf-8"))
    injected = [row for row in events if row.get("event") == "INJECTED"]
    if len(injected) != 1:
        raise RuntimeError("GENUINE_RUNNER_INJECTED_CARDINALITY_FAILURE")
    row = injected[0]
    result = {
        "attempt_started": any(r.get("event") == "ATTEMPT_STARTED" for r in events),
        "premission_health_ready": any(r.get("event") == "PREMISSION_HEALTH_READY" for r in events),
        "armed_for_case": any(r.get("event") == "ARMED_FOR_CASE" for r in events),
        "injection_request_count": 1, "suppression_request_count": 1,
        "request_ns": row.get("request_ns"), "ack_ns": row.get("ack_ns"),
        "injection_response_successful": row.get("success") is True,
        "injection_response_reason": row.get("tool_state"),
        "relay_injection_count": row.get("injection_count"),
        "tool_state": row.get("tool_state"),
        "terminal_metrics": terminal_metrics,
        "events": events,
    }
    result.update(_mission_manager_stale_authority(out, events))
    if not result["attempt_started"] or not result["armed_for_case"]:
        raise RuntimeError("GENUINE_RUNNER_EVIDENCE_INTEGRITY_FAILURE")
    if not result["injection_response_successful"] or result["tool_state"] != "INJECTED" or result["relay_injection_count"] != 1:
        raise RuntimeError("GENUINE_RUNNER_INJECTION_FAILURE")
    return result


def require_feedback_stale_age(runner_evidence: dict) -> float:
    value = runner_evidence.get("feedback_stale_age_sec")
    if value is None or not math.isfinite(float(value)):
        raise RuntimeError("STALE_AGE_EVIDENCE_MISSING")
    if float(value) <= 2.0:
        raise RuntimeError("STALE_AGE_THRESHOLD_FAILURE")
    return float(value)


def _command_row(row: dict) -> dict:
    if "values" in row:
        values = [float(x) for x in row["values"]]
        if len(values) == 2:
            return {"monotonic_ns": int(row["monotonic_ns"]), "linear_x": values[0],
                    "linear_y": 0.0, "linear_z": 0.0, "angular_x": 0.0,
                    "angular_y": 0.0, "angular_z": values[1]}
        if len(values) == 3:
            # wheelchair_cmd_adapter encodes (radius_mm, speed_mm_s, 0).
            # Straight motion uses the configured 10 m radius sentinel;
            # curved motion decodes w = -v/radius.  Zero is unambiguously
            # [0, 0, 0].
            radius_mm, speed_mm_s, _mode = values
            linear_x = speed_mm_s / 1000.0
            angular_z = 0.0 if radius_mm == 0.0 or abs(radius_mm) >= 10000.0 else -speed_mm_s / radius_mm
            return {"monotonic_ns": int(row["monotonic_ns"]), "linear_x": linear_x,
                    "linear_y": 0.0, "linear_z": 0.0, "angular_x": 0.0,
                    "angular_y": 0.0, "angular_z": angular_z,
                    "encoded_radius_mm": radius_mm, "encoded_speed_mm_s": speed_mm_s}
        if len(values) >= 6:
            return {"monotonic_ns": int(row["monotonic_ns"]), **dict(zip(
                ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"), values[:6]))}
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:command_shape")
    required = ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")
    if not all(key in row for key in required):
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:command_fields")
    return {key: float(row[key]) for key in ("monotonic_ns", *required)}


def _hash_check(out: Path, marker: dict) -> None:
    hashes = marker.get("file_hashes")
    if not isinstance(hashes, dict):
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:hashes")
    for name, expected in hashes.items():
        path = out / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:hash:{name}")


def load_genuine_witness_outcome(output_dir: Path) -> dict:
    out = Path(output_dir)
    marker_path = out / "TERMINAL_WITNESS_OUTCOME_COMMITTED"
    if not marker_path.is_file():
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:marker")
    marker = json.loads(marker_path.read_text(encoding="utf-8"))
    _hash_check(out, marker)
    for name in REQUIRED_WITNESS_FILES:
        _jsonl(out / name)
    if not marker.get("terminal_evidence_complete") or not marker.get("physical_evidence_complete"):
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:incomplete")
    start, end = marker.get("DRAIN_START_NS"), marker.get("DRAIN_END_NS")
    actual = marker.get("ACTUAL_DRAIN_SEC")
    if not isinstance(start, int) or not isinstance(end, int) or not math.isfinite(float(actual or float("nan"))):
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:drain")
    if end < start or abs(float(actual) - (end - start) / 1e9) > 1e-12 or float(actual) < .250:
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:drain")
    mission = _jsonl(out / "mission_state_events.jsonl")
    statuses = _jsonl(out / "navigate_action_status_events.jsonl")
    phase = _jsonl(out / "terminal_witness_events.jsonl")
    identity = marker.get("terminal_identity") or {}
    reason = marker.get("reason")
    origin_rows = [row for row in mission if row.get("mission_id") == identity.get("mission_id")
                   and row.get("route_id") == identity.get("route_id")
                   and row.get("active_goal_uuid") == identity.get("uuid")
                   and row.get("state_name") == "CANCELLING"
                   and row.get("reason_code") == reason]
    if len(origin_rows) != 1:
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:origin")
    terminal = {"origin_ns": origin_rows[0].get("monotonic_ns"),
                "failed_received_ns": marker.get("FAILED_RECEIVED_NS"),
                "mission_id": identity.get("mission_id"), "route_id": identity.get("route_id"),
                "waypoint": identity.get("waypoint"), "uuid": identity.get("uuid")}
    if not terminal["origin_ns"] or not terminal["mission_id"] or not terminal["route_id"] or not terminal["uuid"] or not reason:
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:identity")
    action_rows = [{"goal_uuid": row.get("goal_uuid"),
                    "status_name": action_status_name(row),
                    "monotonic_ns": row.get("monotonic_ns")} for row in statuses]
    matching_canceling = [row for row in action_rows if row["goal_uuid"] == terminal["uuid"] and row["status_name"] == "CANCELING"]
    matching_canceled = [row for row in action_rows if row["goal_uuid"] == terminal["uuid"] and row["status_name"] == "CANCELED"]
    if len(matching_canceling) != 1 or len(matching_canceled) != 1:
        raise RuntimeError("TERMINAL_WITNESS_EVIDENCE_INTEGRITY_FAILURE:cancel_cardinality")
    states = [{"mission_id": row.get("mission_id"), "route_id": row.get("route_id"),
               "waypoint_index": row.get("waypoint_index"), "active_goal_uuid": row.get("active_goal_uuid"),
               "monotonic_ns": row.get("monotonic_ns"), "state_name": row.get("state_name"),
               "reason_code": row.get("reason_code")} for row in mission]
    vehicle = [_command_row(row) for row in _jsonl(out / "vehicle_cmd_safe_timeline.jsonl")]
    applied = [_command_row(row) for row in _jsonl(out / "adapter_output_timeline.jsonl")]
    odometry = _jsonl(out / "odometry_timeline.jsonl")
    return {"terminal": terminal, "reason": reason, "states": states, "statuses": action_rows,
            "drain_start_ns": start, "drain_end_ns": end, "actual_drain_sec": float(actual),
            "vehicle_rows": vehicle, "applied_rows": applied, "odometry_rows": odometry,
            "phase_events": phase, "marker": marker}


def post_zero_angular_zero(witness: dict, anchor_ns: int) -> bool:
    def zero(row):
        return float(row.get("angular_z", 0.0)) == 0.0 and float(row.get("angular_x", 0.0)) == 0.0 and float(row.get("angular_y", 0.0)) == 0.0
    commanded = [row for row in witness["vehicle_rows"] if int(row["monotonic_ns"]) >= anchor_ns]
    applied = [row for row in witness["applied_rows"] if int(row["monotonic_ns"]) >= anchor_ns]
    return bool(commanded and applied and all(zero(row) for row in commanded + applied))


TARGET_PROCESS_MARKERS = (
    "/nav2_lifecycle_manager/lifecycle_manager", "planner_server", "controller_server",
    "behavior_server", "bt_navigator", "waypoint_follower", "collision_monitor",
    "collision_monitor_validity_monitor", "parking_robot_mission_manager",
    "guarded_vehicle_cmd_gate", "phase4_vehicle_cmd_fake_base", "wheelchair_cmd_adapter",
    "phase4_p4e6b_health_failure_runner", "phase4_p4e6b_terminal_closure_witness",
    "phase4_p4e6b_feedback_relay", "phase4_p4e6b_s01_attempt4_controller",
)


def global_target_scan() -> list[dict]:
    """Read-only /proc scan using the accepted Phase-4 target markers."""
    mine = {os.getpid(), os.getppid()}
    rows = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(proc.name)
            if pid in mine:
                continue
            command = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            if not any(marker in command for marker in TARGET_PROCESS_MARKERS):
                continue
            stat = (proc / "stat").read_text().split()
            rows.append({"pid": pid, "ppid": int(stat[3]), "pgid": int(stat[4]),
                         "sid": int(stat[5]), "command": command})
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            pass
    return sorted(rows, key=lambda row: row["pid"])


def graph_clean() -> dict:
    """Measure the ROS graph; no hard-coded clean result."""
    try:
        result = subprocess.run(["ros2", "node", "list"], capture_output=True,
                                text=True, timeout=5, check=False)
        nodes = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        target_nodes = [node for node in nodes if any(marker.strip("/") in node for marker in TARGET_PROCESS_MARKERS)]
        return {"returncode": result.returncode, "nodes": nodes,
                "target_nodes": target_nodes, "clean": result.returncode == 0 and not target_nodes}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"returncode": None, "nodes": [], "target_nodes": [], "clean": False,
                "error": repr(exc)}


def authoritative_global_zero(wait_sec: float = 2.0) -> dict:
    first = global_target_scan()
    time.sleep(wait_sec)
    second = global_target_scan()
    graph = graph_clean()
    return {"scan1": first, "scan2": second, "wait_sec": wait_sec,
            "graph": graph, "pass": not first and not second and graph["clean"]}
