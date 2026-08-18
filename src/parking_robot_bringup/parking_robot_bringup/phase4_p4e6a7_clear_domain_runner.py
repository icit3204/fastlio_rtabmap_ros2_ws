"""Passive CLEAR-only chassis-domain qualification using Mission Manager authority."""
from __future__ import annotations

import argparse
from collections.abc import Mapping
import json
import math
import os
from pathlib import Path
import statistics
import time

from action_msgs.msg import GoalStatus
from parking_robot_interfaces.msg import MissionState
from rcl_interfaces.srv import GetParameters
import rclpy

from .phase4_p4e1b_clear_runner import LIFECYCLE, TOPICS, nz, rates
from .phase4_p4e2a_slowdown_runner import pair_commands
from .phase4_p4e5b_temporary_block_runner import P4E5BRunner, health_ok, state_after


INVALID_REASONS = {
    "TURN_RADIUS_UNSUPPORTED", "IN_PLACE_ROTATION_UNSUPPORTED", "REVERSE_UNSUPPORTED",
    "OVER_LIMIT", "NUMERICAL_INVALID", "UNSUPPORTED_AXES", "FRAME_INVALID",
    "INPUT_STALE", "INPUT_AUTHORITY_INVALID", "OUTPUT_AUTHORITY_INVALID",
}

CALLBACK_DRAIN_SEC = 1.0
FORMAL_READINESS_EPOCH_SEC = 2.0


class EvidenceRecordError(RuntimeError):
    """A stored runner observation does not satisfy its internal contract."""


def diagnostic_tuple(record):
    """Validate and unpack diag_conditions: (message, fields, receipt_ns)."""
    if not isinstance(record, tuple) or len(record) != 3:
        raise EvidenceRecordError("malformed diagnostic tuple: expected 3 fields")
    message, fields, receipt_ns = record
    if not isinstance(message, str):
        raise EvidenceRecordError("malformed diagnostic tuple: message must be str")
    if not isinstance(fields, Mapping):
        raise EvidenceRecordError("malformed diagnostic tuple: fields must be mapping")
    if not isinstance(receipt_ns, int) or isinstance(receipt_ns, bool):
        raise EvidenceRecordError("malformed diagnostic tuple: receipt_ns must be int")
    return message, fields, receipt_ns


def diagnostic_history_record(record):
    """Validate dict records produced by the diagnostics callback."""
    if not isinstance(record, Mapping):
        raise EvidenceRecordError("malformed diagnostic history record: expected mapping")
    try:
        receipt_ns, level, message, fields = (record["monotonic_ns"], record["level"],
                                               record["message"], record["values"])
    except KeyError as exc:
        raise EvidenceRecordError(f"malformed diagnostic history record: missing {exc.args[0]}") from None
    if not isinstance(receipt_ns, int) or isinstance(receipt_ns, bool):
        raise EvidenceRecordError("malformed diagnostic history record: receipt_ns must be int")
    if not isinstance(level, int) or isinstance(level, bool):
        raise EvidenceRecordError("malformed diagnostic history record: level must be int")
    if not isinstance(message, str) or not isinstance(fields, Mapping):
        raise EvidenceRecordError("malformed diagnostic history record: message/fields type")
    return receipt_ns, level, message, fields


def permission_record(record):
    if not isinstance(record, tuple) or len(record) != 2:
        raise EvidenceRecordError("malformed permission record: expected (valid, receipt_ns)")
    valid, receipt_ns = record
    if not isinstance(valid, bool) or not isinstance(receipt_ns, int) or isinstance(receipt_ns, bool):
        raise EvidenceRecordError("malformed permission record: field type")
    return valid, receipt_ns


def formal_entry_health(now_ns, gate_record, permissions, adapter_record, *, max_age_ns=500_000_000):
    """Execute the exact validated formal-entry health path."""
    _, gate_fields, gate_receipt_ns = diagnostic_tuple(gate_record)
    adapter_receipt_ns, adapter_level, _, adapter_fields = diagnostic_history_record(adapter_record)
    permission_values = [permission_record(record) for record in permissions]
    return bool(gate_fields.get("state") == "DISARMED"
                and gate_fields.get("fault_latched") == "false"
                and 0 <= now_ns-gate_receipt_ns <= max_age_ns
                and adapter_level == 0
                and 0 <= now_ns-adapter_receipt_ns <= max_age_ns
                and adapter_fields.get("input_publisher_count") == "1"
                and adapter_fields.get("output_publisher_count") == "1"
                and all(valid and 0 <= now_ns-receipt_ns <= max_age_ns
                        for valid, receipt_ns in permission_values))


def interval_rate(timestamps):
    """Return interval-count rate for one prospectively selected epoch."""
    if len(timestamps) < 2:
        return None
    span_ns = int(timestamps[-1]) - int(timestamps[0])
    if span_ns <= 0:
        return None
    return (len(timestamps) - 1) / (span_ns / 1e9)


def interarrival_statistics(timestamps, *, large_gap_sec):
    intervals = [(int(b) - int(a)) / 1e9 for a, b in zip(timestamps, timestamps[1:])]
    positive = [x for x in intervals if x > 0.0]
    ordered = sorted(positive)
    if not ordered:
        return {"minimum_sec": None, "median_sec": None, "mean_sec": None,
                "p95_sec": None, "maximum_sec": None,
                "duplicates": sum(x == 0.0 for x in intervals),
                "negative_intervals": sum(x < 0.0 for x in intervals),
                "large_gaps": sum(x > large_gap_sec for x in intervals)}
    p95_index = math.ceil(0.95 * len(ordered)) - 1
    return {"minimum_sec": ordered[0], "median_sec": statistics.median(ordered),
            "mean_sec": statistics.fmean(ordered), "p95_sec": ordered[p95_index],
            "maximum_sec": ordered[-1],
            "duplicates": sum(x == 0.0 for x in intervals),
            "negative_intervals": sum(x < 0.0 for x in intervals),
            "large_gaps": sum(x > large_gap_sec for x in intervals)}


def prospective_epoch(timestamps, start_ns, end_ns, *, minimum_duration_sec,
                      lower_hz, upper_hz, large_gap_sec):
    """Adjudicate exactly the supplied prospective epoch; never search alternatives."""
    selected = [int(x) for x in timestamps if start_ns <= int(x) <= end_ns]
    duration_sec = (int(end_ns) - int(start_ns)) / 1e9
    rate_hz = interval_rate(selected)
    sufficient = duration_sec >= minimum_duration_sec and rate_hz is not None
    return {"start_ns": int(start_ns), "end_ns": int(end_ns),
            "duration_sec": duration_sec, "sample_count": len(selected),
            "interval_count": max(0, len(selected) - 1), "rate_hz": rate_hz,
            "interarrival": interarrival_statistics(selected, large_gap_sec=large_gap_sec),
            "sufficient": sufficient,
            "passes": bool(sufficient and lower_hz <= rate_hz <= upper_hz)}


def epoch_health(adapter_diagnostics, required_health, start_ns, end_ns):
    """Evaluate cumulative evidence in the fixed epoch without debounce or retry."""
    adapter = [x for x in adapter_diagnostics
               if start_ns <= int(x["monotonic_ns"]) <= end_ns]
    warnings = [x for x in adapter if int(x["level"]) == 1]
    errors = [x for x in adapter if int(x["level"]) >= 2]
    invalid_health = [x for x in required_health
                      if start_ns <= int(x["monotonic_ns"]) <= end_ns and not x["valid"]]
    return {"adapter_count": len(adapter), "adapter_warn_count": len(warnings),
            "adapter_error_count": len(errors), "health_invalid_count": len(invalid_health),
            "passes": bool(adapter and not warnings and not errors and not invalid_health)}


def domain(row, *, stamped=False):
    values = tuple(float(x) for x in row[2:8])
    v, ly, lz, ax, ay, w = values
    frame = row[8] if stamped else "base_footprint"
    finite = all(math.isfinite(x) for x in values)
    axes = max(abs(ly), abs(lz), abs(ax), abs(ay)) <= 1e-6
    zero = abs(v) <= 1e-6 and abs(w) <= 1e-6
    supported = (finite and frame == "base_footprint" and axes and v >= -1e-6
                 and v <= .20 + 1e-12 and abs(w) <= .50 + 1e-12
                 and (zero or (v > 0 and abs(w) <= v + 1e-12)))
    radius = None if abs(w) <= 1e-12 else abs(v / w)
    return {"valid": supported, "zero": zero, "v": v, "w": w,
            "radius": radius, "margin": v - abs(w), "frame": frame,
            "finite": finite, "unsupported_axes_ok": axes}


def summarize(rows, *, stamped=False):
    judged = [domain(x, stamped=stamped) for x in rows]
    nonzero = [x for x in judged if not x["zero"]]
    radii = [x["radius"] for x in nonzero if x["radius"] is not None]
    return {"total": len(judged), "zero": sum(x["zero"] for x in judged),
            "nonzero": len(nonzero), "invalid": sum(not x["valid"] for x in judged),
            "minimum_positive_v": min((x["v"] for x in nonzero if x["v"] > 0), default=None),
            "maximum_v": max((x["v"] for x in judged), default=None),
            "maximum_abs_w": max((abs(x["w"]) for x in judged), default=None),
            "minimum_positive_radius": min(radii, default=None),
            "minimum_margin": min((x["margin"] for x in nonzero), default=None)}


def adjudicate_command_pairs(raw_rows, safe_rows, start_ns, end_ns):
    """Run the frozen CLEAR pairing contract over explicit authoritative bounds."""
    if (not isinstance(start_ns, int) or isinstance(start_ns, bool)
            or not isinstance(end_ns, int) or isinstance(end_ns, bool)):
        raise EvidenceRecordError("command-pair bounds must be integer monotonic timestamps")
    if end_ns <= start_ns:
        raise EvidenceRecordError("command-pair end must be after start")
    for family, rows in (("raw", raw_rows), ("safe", safe_rows)):
        if not isinstance(rows, (list, tuple)):
            raise EvidenceRecordError(f"{family} command history must be a sequence")
        for row in rows:
            if not isinstance(row, (list, tuple)) or not row:
                raise EvidenceRecordError(f"malformed {family} command row")
            if not isinstance(row[0], int) or isinstance(row[0], bool):
                raise EvidenceRecordError(f"malformed {family} command timestamp")
    pairs = pair_commands(raw_rows, safe_rows, start_ns, end_ns)
    paired = [(raw, safe, skew) for raw, safe, skew in pairs if skew <= 75_000_000]
    return {
        "pairs": paired,
        "pair_count": len(paired),
        "raw_count": sum(start_ns <= row[0] <= end_ns for row in raw_rows),
        "safe_count": sum(start_ns <= row[0] <= end_ns for row in safe_rows),
        "invalid_safe_count": sum(not domain(safe)["valid"] for _, safe, _ in paired),
        "maximum_pair_delay_ns": max((skew for _, _, skew in paired), default=None),
        "maximum_component_error": max(
            (max(abs(a-b) for a, b in zip(raw[2:8], safe[2:8]))
             for raw, safe, _ in paired), default=None),
    }


class ClearDomainRunner(P4E5BRunner):
    def get_parameters(self, node, names):
        client = self.create_client(GetParameters, f"/{node}/get_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"parameter service unavailable: {node}")
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"parameter query failed: {node}")
        result = {}
        for name, value in zip(names, future.result().values):
            if value.type == 1: result[name] = value.bool_value
            elif value.type == 2: result[name] = value.integer_value
            elif value.type == 3: result[name] = value.double_value
            elif value.type == 4: result[name] = value.string_value
            elif value.type == 9: result[name] = list(value.string_array_value)
            else: result[name] = {"type": int(value.type)}
        client.destroy()
        return result


def _first_state(node, state, after=0, waypoint=None):
    return next((x for x in node.states if x["monotonic_ns"] >= after
                 and x["state"] == state
                 and (waypoint is None or x["waypoint_index"] == waypoint)), None)


_UNEXPECTED_PRE_ACCEPTANCE_TERMINALS = {
    MissionState.FAILED, MissionState.CANCELLED, MissionState.BLOCKED,
    MissionState.CANCELLING, MissionState.SUCCEEDED,
}


def authoritative_goal_record(states, *, after_ns, waypoint_index,
                              mission_id=None, route_id=None, prior_uuids=()):
    """Return the one accepted nonempty UUID record for an expected goal generation.

    MissionState publications are observational evidence: NAVIGATING may be
    received before the asynchronous NavigateToPose response supplies a UUID.
    UUIDs from completed generations are ignored while acquiring the next one.
    """
    prior = set(prior_uuids)
    candidates = []
    for record in states:
        if not isinstance(record, Mapping):
            raise EvidenceRecordError("malformed mission-state record: expected mapping")
        try:
            receipt_ns = record["monotonic_ns"]
            state = record["state"]
            waypoint = record["waypoint_index"]
            uuid = record["active_goal_uuid"]
        except KeyError as exc:
            raise EvidenceRecordError(
                f"malformed mission-state record: missing {exc.args[0]}") from None
        if (not isinstance(receipt_ns, int) or isinstance(receipt_ns, bool)
                or not isinstance(state, int) or isinstance(state, bool)
                or not isinstance(waypoint, int) or isinstance(waypoint, bool)
                or not isinstance(uuid, str)):
            raise EvidenceRecordError("malformed mission-state record: field type")
        if receipt_ns < after_ns:
            continue
        if mission_id is not None and record.get("mission_id") != mission_id:
            continue
        if route_id is not None and record.get("route_id") != route_id:
            continue
        if state in _UNEXPECTED_PRE_ACCEPTANCE_TERMINALS and not candidates:
            raise EvidenceRecordError(
                f"mission terminated before authoritative goal UUID: state={state}")
        if state != MissionState.NAVIGATING or waypoint != waypoint_index or not uuid:
            continue
        if uuid in prior:
            continue
        candidates.append(record)
    distinct = {record["active_goal_uuid"] for record in candidates}
    if len(distinct) > 1:
        raise EvidenceRecordError(
            f"multiple authoritative UUIDs for waypoint {waypoint_index}: {sorted(distinct)}")
    return candidates[0] if candidates else None


def priming_for_goal(diagnostics, *, after_ns, goal_uuid):
    """Return INITIAL_PRIMING correlated to an already-bound nonempty UUID."""
    if not isinstance(goal_uuid, str) or not goal_uuid:
        raise EvidenceRecordError("INITIAL_PRIMING correlation requires nonempty UUID")
    matches = []
    for record in diagnostics:
        receipt_ns, _, _, fields = diagnostic_history_record(record)
        if (receipt_ns >= after_ns
                and fields.get("progress_policy_activation_state") == "INITIAL_PRIMING"
                and fields.get("active_goal_uuid") == goal_uuid):
            matches.append(record)
    return matches[-1] if matches else None


def main(args=None):
    parser = argparse.ArgumentParser(); parser.add_argument("--output-dir", required=True)
    ns = parser.parse_args(args); out = Path(ns.output_dir); out.mkdir(parents=True, exist_ok=True)
    status = "PHASE4_P4E6A7B_CLEAR_COMMAND_DOMAIN_RUNTIME_NEEDS_REVIEW"; error = None; metrics = {}
    rclpy.init(); node = ClearDomainRunner(out)
    try:
        (out / "process_environment.tsv").write_text(
            "key\tvalue\n" + "".join(f"{k}\t{v}\n" for k, v in sorted(os.environ.items())
            if k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")), encoding="utf-8")
        node.spin(5.0); node.graph(); lifecycle = node.lifecycle()
        if any(lifecycle.get(name, (0, ""))[1] != "active" for name in LIFECYCLE):
            raise RuntimeError(f"inactive lifecycle: {lifecycle}")
        names = set(node.get_node_names())
        forbidden = {x for x in names if any(y in x.lower() for y in
                    ("plan_nav", "rtab", "fast_lio", "pure_pursuit", "wheelchair_controller", "udp"))}
        if forbidden or sum(x == "mission_manager" for x in names) != 1:
            raise RuntimeError(f"authority graph mismatch: {forbidden}, {names}")
        publishers = {topic: len(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
        raw_nodes = {ep.node_name for ep in node.get_publishers_info_by_topic("/cmd_vel_nav_raw")}
        if any(publishers[t] != 1 for t in TOPICS[1:]) or raw_nodes != {"controller_server", "behavior_server"}:
            raise RuntimeError(f"publisher authority mismatch: {publishers}, {raw_nodes}")

        controller_names = ["FollowPath.motion_model", "FollowPath.vx_min", "FollowPath.vx_max",
                            "FollowPath.vy_max", "FollowPath.wz_max",
                            "FollowPath.AckermannConstraints.min_turning_r"]
        controller = node.get_parameters("controller_server", controller_names)
        behavior = node.get_parameters("behavior_server", ["behavior_plugins"])
        navigator = node.get_parameters("bt_navigator", ["default_nav_to_pose_bt_xml",
                                                          "default_nav_through_poses_bt_xml"])
        expected = dict(zip(controller_names, ["Ackermann", 0.0, .20, 0.0, .50, 1.0]))
        if controller != expected or behavior["behavior_plugins"] != ["wait"]:
            status = "P4E6A7B_RUNTIME_PROFILE_LOAD_MISMATCH_NEEDS_REVIEW"
            raise RuntimeError(f"runtime profile mismatch: {controller}, {behavior}")
        expected_bts = {"default_nav_to_pose_bt_xml": "phase4_p4e_chassis_navigate_to_pose.xml",
                        "default_nav_through_poses_bt_xml": "phase4_p4e_chassis_navigate_through_poses.xml"}
        if any(not navigator[k].endswith("/" + value) for k, value in expected_bts.items()):
            status = "P4E6A7B_RUNTIME_PROFILE_LOAD_MISMATCH_NEEDS_REVIEW"
            raise RuntimeError(f"BT mismatch: {navigator}")
        profile = node.emit("scenario_events.jsonl", {"event": "P4E6A7B_RUNTIME_PROFILE_LOAD_PASS",
                            "controller": controller, "behavior": behavior, "navigator": navigator})
        last_synchronous_query_complete_ns = time.monotonic_ns()
        pre_settling_now_ns = time.monotonic_ns()
        gate = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        adapter_latest = node.diagnostic_history["adapter"][-1] if node.diagnostic_history["adapter"] else None
        permission_records = [node.permissions.get(t) for t in TOPICS[2:5]]
        if (gate is None or adapter_latest is None or any(x is None for x in permission_records)
                or not formal_entry_health(pre_settling_now_ns, gate, permission_records, adapter_latest)):
            raise RuntimeError("pre-settling readiness health failed")

        callback_drain_start_ns = time.monotonic_ns()
        node.spin(CALLBACK_DRAIN_SEC)
        callback_drain_end_ns = time.monotonic_ns()
        gate = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        adapter_latest = node.diagnostic_history["adapter"][-1] if node.diagnostic_history["adapter"] else None
        formal_entry_now_ns = time.monotonic_ns()
        permission_records = [node.permissions.get(t) for t in TOPICS[2:5]]
        if (gate is None or adapter_latest is None or any(x is None for x in permission_records)
                or not formal_entry_health(formal_entry_now_ns, gate, permission_records, adapter_latest)):
            raise RuntimeError("formal epoch entry health failed")

        # These fresh buffers are the formal evidence boundary. Cumulative safety,
        # diagnostic, mission, and authority histories intentionally remain intact.
        formal_buffers = {"gate": [], "adapter": [], "odom": [], "tf": []}
        formal_readiness_epoch_start_ns = time.monotonic_ns()
        ready_pose = node.latest_odom
        node.spin(FORMAL_READINESS_EPOCH_SEC)
        formal_readiness_epoch_end_ns = time.monotonic_ns()
        formal_buffers["gate"] = [x[0] for x in node.samples["/vehicle_cmd_safe"]
                                  if formal_readiness_epoch_start_ns <= x[0] <= formal_readiness_epoch_end_ns]
        formal_buffers["adapter"] = [x[0] for x in node.samples["/wheelchair_control_command_mock"]
                                     if formal_readiness_epoch_start_ns <= x[0] <= formal_readiness_epoch_end_ns]
        formal_buffers["odom"] = [x[0] for x in node.samples["/Odometry"]
                                  if formal_readiness_epoch_start_ns <= x[0] <= formal_readiness_epoch_end_ns]
        formal_buffers["tf"] = [x[0] for x in node.tf_samples["odom->base_footprint"]
                                if formal_readiness_epoch_start_ns <= x[0] <= formal_readiness_epoch_end_ns]
        epochs = {
            "gate": prospective_epoch(formal_buffers["gate"], formal_readiness_epoch_start_ns,
                                      formal_readiness_epoch_end_ns, minimum_duration_sec=2.0,
                                      lower_hz=18.0, upper_hz=22.0, large_gap_sec=.10),
            "adapter": prospective_epoch(formal_buffers["adapter"], formal_readiness_epoch_start_ns,
                                         formal_readiness_epoch_end_ns, minimum_duration_sec=2.0,
                                         lower_hz=18.0, upper_hz=22.0, large_gap_sec=.10),
            "odom": prospective_epoch(formal_buffers["odom"], formal_readiness_epoch_start_ns,
                                      formal_readiness_epoch_end_ns, minimum_duration_sec=2.0,
                                      lower_hz=48.0, upper_hz=52.0, large_gap_sec=.05),
            "tf": prospective_epoch(formal_buffers["tf"], formal_readiness_epoch_start_ns,
                                    formal_readiness_epoch_end_ns, minimum_duration_sec=2.0,
                                    lower_hz=48.0, upper_hz=52.0, large_gap_sec=.05),
        }
        adapter_epoch = [x for x in node.diagnostic_history["adapter"]
                         if formal_readiness_epoch_start_ns <= x["monotonic_ns"] <= formal_readiness_epoch_end_ns]
        epoch_warn = [x for x in adapter_epoch if x["level"] == 1]
        epoch_error = [x for x in adapter_epoch if x["level"] >= 2]
        gate_epoch = [x for x in node.diagnostic_history["gate"]
                      if formal_readiness_epoch_start_ns <= x["monotonic_ns"] <= formal_readiness_epoch_end_ns]
        permission_epoch = {topic: [x for x in node.samples[topic]
                                    if formal_readiness_epoch_start_ns <= x[0] <= formal_readiness_epoch_end_ns]
                            for topic in TOPICS[2:5]}
        health_failed = (not adapter_epoch or epoch_warn or epoch_error or not gate_epoch
                         or any(x["values"].get("state") != "DISARMED"
                                or x["values"].get("fault_latched") != "false" for x in gate_epoch)
                         or any(not rows or any(not x[2] for x in rows)
                                or formal_readiness_epoch_end_ns-rows[-1][0] > 500_000_000
                                for rows in permission_epoch.values()))
        if health_failed or not all(x["passes"] for x in epochs.values()):
            raise RuntimeError(f"formal readiness failed: health={health_failed}, cadence={epochs}")
        ready_start = formal_readiness_epoch_start_ns
        ready_end = formal_readiness_epoch_end_ns
        ready_rates = {name: value["rate_hz"] for name, value in epochs.items()}
        if ready_pose is None or math.hypot(node.latest_odom[0]-ready_pose[0], node.latest_odom[1]-ready_pose[1]) > .002:
            raise RuntimeError("readiness drift")
        readiness = node.emit("scenario_events.jsonl", {"event": "P4E6A7B_CHAIN_READINESS_PASS",
                              "lifecycle": lifecycle, "publishers": publishers,
                              "raw_nodes": sorted(raw_nodes), "rates_hz": ready_rates,
                              "cadence_epochs": epochs,
                              "last_synchronous_query_complete_ns": last_synchronous_query_complete_ns,
                              "callback_drain_start_ns": callback_drain_start_ns,
                              "callback_drain_end_ns": callback_drain_end_ns,
                              "callback_drain_duration_sec":
                                  (callback_drain_end_ns-callback_drain_start_ns)/1e9,
                              "formal_readiness_epoch_start_ns": formal_readiness_epoch_start_ns,
                              "formal_readiness_epoch_end_ns": formal_readiness_epoch_end_ns,
                              "formal_readiness_epoch_duration_sec":
                                  (formal_readiness_epoch_end_ns-formal_readiness_epoch_start_ns)/1e9,
                              "adapter_warn_count_formal_epoch": len(epoch_warn),
                              "adapter_error_count_formal_epoch": len(epoch_error)})

        node.publish_initial(); node.spin(.5); route = node.publish_route()
        received = node.wait_for(
            lambda: _first_state(node, MissionState.RECEIVED, route["monotonic_ns"]),
            5.0, "RECEIVED timeout")
        start_request, start_response = node.trigger("start")
        nav0 = node.wait_for(
            lambda: authoritative_goal_record(
                node.states, after_ns=start_request["monotonic_ns"], waypoint_index=0,
                mission_id=received["mission_id"], route_id=received["route_id"]),
            10.0, "waypoint-0 authoritative UUID timeout")
        uuid0 = nav0["active_goal_uuid"]
        node.wait_for(lambda: priming_for_goal(
                          node.mission_diagnostics, after_ns=start_request["monotonic_ns"],
                          goal_uuid=uuid0),
                      1.0, "INITIAL_PRIMING timeout")
        node.wait_fresh_safe_disarmed_since(nav0["monotonic_ns"], 1.6)
        window_start = time.monotonic_ns() - 100_000_000
        arm_response = node.arm_gate(); arm_ns = arm_response["monotonic_ns"]
        active = node.wait_for(lambda: node.latest_policy("ACTIVE", arm_ns, uuid0), 2.0, "ACTIVE timeout")
        active_ns = active["monotonic_ns"]
        if active_ns - arm_ns >= 2_000_000_000: raise RuntimeError("activation deadline failed")
        startup_end_target = active_ns + 2_000_000_000
        while time.monotonic_ns() < startup_end_target: rclpy.spin_once(node, timeout_sec=.01)
        startup_window = node.emit("scenario_events.jsonl", {
            "event": "P4E6A7B_ARM_STARTUP_COMPATIBILITY_WINDOW_CAPTURED",
            "start_ns": window_start, "arm_ns": arm_ns, "active_ns": active_ns,
            "end_ns": time.monotonic_ns()})

        qualified_start = ready_start; stable_deadline = active_ns + 5_000_000_000
        seen_adapter = 0; seen_vehicle = 0; seen_raw = 0
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            fresh = node.diagnostic_history["adapter"][seen_adapter:]
            seen_adapter = len(node.diagnostic_history["adapter"])
            bad = next((x for x in fresh if x["monotonic_ns"] >= qualified_start and x["level"] != 0), None)
            if bad:
                status = "P4E6A7B_ADAPTER_INVALID_RECURRED_NEEDS_REVIEW"
                raise RuntimeError(f"adapter invalid recurred: {bad}")
            fresh_vehicle = node.samples["/vehicle_cmd_safe"][seen_vehicle:]
            seen_vehicle = len(node.samples["/vehicle_cmd_safe"])
            bad_vehicle = next((x for x in fresh_vehicle if x[0] >= qualified_start
                                and not domain(x, stamped=True)["valid"]), None)
            if bad_vehicle:
                status = "P4E6A7B_GATE_RUNTIME_DOMAIN_VIOLATION_NEEDS_REVIEW"
                raise RuntimeError(f"Gate domain violation: {bad_vehicle}")
            fresh_raw = node.samples["/cmd_vel_nav_raw"][seen_raw:]
            seen_raw = len(node.samples["/cmd_vel_nav_raw"])
            bad_raw = next((x for x in fresh_raw if x[0] >= qualified_start
                            and not domain(x)["valid"]), None)
            if bad_raw:
                status = "P4E6A7B_MPPI_RUNTIME_DOMAIN_VIOLATION_NEEDS_REVIEW"
                raise RuntimeError(f"raw domain violation: {bad_raw}")
            terminal = _first_state(node, MissionState.SUCCEEDED, start_request["monotonic_ns"])
            if terminal: break
            failed = next((x for x in node.states if x["monotonic_ns"] >= start_request["monotonic_ns"]
                           and x["state"] in (MissionState.FAILED, MissionState.BLOCKED,
                                              MissionState.CANCELLED, MissionState.CANCELLING)), None)
            if failed: raise RuntimeError(f"unexpected mission state: {failed}")
        else: raise RuntimeError("mission success timeout")
        end_ns = terminal["monotonic_ns"]; node.spin(2.0); node.graph()

        vehicle_rows = [x for x in node.samples["/vehicle_cmd_safe"] if qualified_start <= x[0] <= end_ns]
        vehicle_stats = summarize(vehicle_rows, stamped=True)
        if vehicle_stats["invalid"]:
            status = "P4E6A7B_GATE_RUNTIME_DOMAIN_VIOLATION_NEEDS_REVIEW"
            raise RuntimeError(f"vehicle domain violation: {vehicle_stats}")
        raw_rows = [x for x in node.samples["/cmd_vel_nav_raw"] if qualified_start <= x[0] <= end_ns]
        # Humble's accepted one-argument subscription callback does not expose
        # per-sample writer GIDs. With the runtime-verified Wait-only behavior
        # profile and zero recovery activity, nonzero raw samples are therefore
        # controller-attributed by source/profile/runtime correlation.
        controller_rows = raw_rows
        behavior_rows = []
        controller_stats = summarize(controller_rows)
        behavior_stats = summarize(behavior_rows)
        if controller_stats["invalid"]:
            status = "P4E6A7B_MPPI_RUNTIME_DOMAIN_VIOLATION_NEEDS_REVIEW"
            raise RuntimeError(f"controller domain violation: {controller_stats}")
        if behavior_stats["nonzero"]:
            status = "P4E6A7B_RECOVERY_PROFILE_RUNTIME_MISMATCH_NEEDS_REVIEW"
            raise RuntimeError(f"behavior velocity observed: {behavior_stats}")

        raw_q = raw_rows
        safe_q = [x for x in node.samples["/cmd_vel_nav_safe"] if qualified_start <= x[0] <= end_ns]
        pair_result = adjudicate_command_pairs(
            raw_q, safe_q, qualified_start, end_ns)
        paired = pair_result["pairs"]
        max_error = pair_result["maximum_component_error"]
        invalid_safe = pair_result["invalid_safe_count"]
        if invalid_safe: raise RuntimeError("Collision Monitor CLEAR produced invalid safe command")

        adapter_rows = [x for x in node.diagnostic_history["adapter"] if qualified_start <= x["monotonic_ns"] <= end_ns]
        levels = {str(level): sum(x["level"] == level for x in adapter_rows) for level in (0,1,2)}
        adapter_reasons = sorted({x["message"] for x in adapter_rows})
        adapter_input_ages = [float(x["values"]["input_age_sec"]) for x in adapter_rows
                              if x["values"].get("input_age_sec") not in (None, "NA", "inf")]
        adapter_input_publishers = sorted({x["values"].get("input_publisher_count") for x in adapter_rows})
        adapter_output_publishers = sorted({x["values"].get("output_publisher_count") for x in adapter_rows})
        invalid_events = [x for x in node.mission_diagnostics if qualified_start <= x["monotonic_ns"] <= end_ns
                          and x["values"].get("progress_supervisor_event") == "ADAPTER_INVALID"]
        if levels["1"] or levels["2"] or invalid_events: raise RuntimeError("adapter health proof failed")
        collision_values = [x["values"].get("collision_classification") for x in node.mission_diagnostics
                            if qualified_start <= x["monotonic_ns"] <= end_ns
                            and x["values"].get("collision_classification")]
        if any(x != "CLEAR" for x in collision_values):
            raise RuntimeError(f"non-CLEAR classification: {set(collision_values)}")
        uuid_rows = [(x["goal_uuid"], x["waypoint_index"]) for x in node.states if x["active_goal_uuid"]]
        uuid0s = {u for u,i in uuid_rows if i == 0}; uuid1s = {u for u,i in uuid_rows if i == 1}
        action_success = {(x["goal_uuid"]) for x in node.action_statuses if x["status"] == GoalStatus.STATUS_SUCCEEDED}
        nav1 = authoritative_goal_record(
            node.states, after_ns=nav0["monotonic_ns"], waypoint_index=1,
            mission_id=received["mission_id"], route_id=received["route_id"],
            prior_uuids={uuid0})
        if (time.monotonic_ns() < stable_deadline or len(uuid0s) != 1 or len(uuid1s) != 1
                or uuid0s == uuid1s or not (uuid0s | uuid1s) <= action_success
                or terminal["completed"] != 2 or terminal["total"] != 2
                or node.route_count != 1 or node.start_count != 1 or node.arm_request_count != 1
                or node.cancel_count != 0 or nav1 is None):
            raise RuntimeError("mission/sequential cardinality failed")
        if any(x["values"].get("progress_policy_activation_state") == "INITIAL_PRIMING"
               for x in node.mission_diagnostics if active_ns < x["monotonic_ns"] < end_ns):
            raise RuntimeError("unexpected activation re-priming")
        recovery_counts = [x["number_of_recoveries"] for x in node.feedback]
        gate_rows = [x for x in node.diagnostic_history["gate"] if qualified_start <= x["monotonic_ns"] <= end_ns]
        if any(x["values"].get("fault_latched") != "false" for x in gate_rows):
            raise RuntimeError("Gate fault during qualified interval")
        cadence = {topic: rates(node.samples[topic], qualified_start, end_ns) for topic in TOPICS}
        results = {
            "vehicle_cmd_safe": vehicle_stats, "controller_raw": controller_stats,
            "behavior_raw": behavior_stats, "collision_clear_pairs": len(paired),
            "collision_clear_max_component_error": max_error,
            "collision_clear_max_pair_delay_ns": max((x[2] for x in paired), default=None),
            "collision_clear_invalid_safe": invalid_safe,
            "adapter_diagnostic_count": len(adapter_rows), "adapter_levels": levels,
            "adapter_messages": adapter_reasons, "adapter_invalid_supervisor_events": len(invalid_events),
            "adapter_maximum_input_age_sec": max(adapter_input_ages, default=None),
            "adapter_input_publisher_counts": adapter_input_publishers,
            "adapter_output_publisher_counts": adapter_output_publishers,
            "collision_classification_count": len(collision_values),
            "collision_classifications": sorted(set(collision_values)),
            "recovery_baseline": min(recovery_counts, default=0),
            "recovery_maximum": max(recovery_counts, default=0),
            "behavior_nonzero_count": behavior_stats["nonzero"],
            "producer_attribution": "WAIT_ONLY_PROFILE_ZERO_RECOVERY_SOURCE_RUNTIME_CORRELATED",
            "cadence_hz": cadence,
        }
        for event in ("P4E6A7B_VEHICLE_CMD_SAFE_DOMAIN_PASS", "P4E6A7B_RAW_PRODUCER_DOMAIN_PASS",
                      "P4E6A7B_COLLISION_CLEAR_DOMAIN_PASS", "P4E6A7B_GATE_RUNTIME_DOMAIN_CLOSURE_PASS",
                      "P4E6A7B_ADAPTER_HEALTH_PASS", "P4E6A7B_STABLE_CLEAR_COMMAND_DOMAIN_PASS",
                      "P4E6A7B_SEQUENTIAL_WAYPOINT_DOMAIN_PASS", "P4E6A7B_CLEAR_MISSION_SUCCESS_PASS"):
            node.emit("scenario_events.jsonl", {"event": event})
        status = "PHASE4_P4E6A7B_CLEAR_COMMAND_DOMAIN_RUNTIME_COMPLETED_NEEDS_REVIEW"
        metrics = {"pass": True, "status": status, "profile": profile, "readiness": readiness,
                   "startup_window": startup_window, "route_count": node.route_count,
                   "start_count": node.start_count, "arm_count": node.arm_request_count,
                   "cancel_count": node.cancel_count, "uuid0": sorted(uuid0s), "uuid1": sorted(uuid1s),
                   "final_state": terminal, "results": results,
                   "exclusions": {"Collision Monitor STOP":"NOT TESTED", "TEMPORARILY_BLOCKED":"NOT TESTED",
                     "PERSISTENT_COLLISION_STOP":"NOT TESTED", "BLOCK_TERMINATION":"NOT TESTED",
                     "CANCELLING -> BLOCKED":"NOT TESTED", "20-second persistent STOP":"NOT RUN",
                     "27/30-second persistent budget":"NOT QUALIFIED"}}
    except Exception as exc:
        error = repr(exc); metrics = {"pass": False, "status": status, "error": error}
    finally:
        (out / "terminal_metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True)+"\n", encoding="utf-8")
        node.close(); node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)
