"""Software-only CLEAR -> SLOWDOWN -> CLEAR qualification for Phase 4 P4-E.2A."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import time

from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy

from parking_robot_bringup.phase4_p4e1b_clear_runner import (
    FORBIDDEN, LIFECYCLE, TOPICS, Runner, adjudicate_terminal_stop, nz, rates,
)

MOVEMENT_THRESHOLD = 1.0e-3
PAIR_SKEW_NS = 75_000_000
SLOWDOWN_CLASSIFICATION_RATIO = 0.80
MATERIAL_EXCESS = 0.05


def command_magnitude(row):
    return math.hypot(float(row[2]), float(row[7]))


def pair_commands(raw_rows, safe_rows, start_ns, end_ns, max_skew_ns=PAIR_SKEW_NS):
    """Pair each safe sample to its closest raw sample without synthesizing data."""
    raw = [r for r in raw_rows if start_ns <= r[0] <= end_ns]
    safe = [s for s in safe_rows if start_ns <= s[0] <= end_ns]
    pairs = []
    for safe_row in safe:
        if not raw:
            break
        raw_row = min(raw, key=lambda row: abs(row[0] - safe_row[0]))
        skew = abs(raw_row[0] - safe_row[0])
        if skew <= max_skew_ns:
            pairs.append((raw_row, safe_row, skew))
    return pairs


def classify_slowdown_pair(raw, safe, *, collision_valid, gate_valid=True,
                           permissions_valid=True, cancellation_intent=False,
                           terminal=False):
    raw_mag = command_magnitude(raw)
    safe_mag = command_magnitude(safe)
    same_direction = all(
        abs(float(r)) <= MOVEMENT_THRESHOLD or abs(float(s)) <= MOVEMENT_THRESHOLD
        or math.copysign(1.0, float(r)) == math.copysign(1.0, float(s))
        for r, s in ((raw[2], safe[2]), (raw[7], safe[7]))
    )
    ratio = safe_mag / raw_mag if raw_mag > MOVEMENT_THRESHOLD else None
    reasons = []
    if not collision_valid: reasons.append("collision validity is not fresh true")
    if not gate_valid or not permissions_valid: reasons.append("gate/permission health invalid")
    if cancellation_intent: reasons.append("cancellation intent")
    if terminal: reasons.append("terminal sample")
    if raw_mag <= MOVEMENT_THRESHOLD: reasons.append("raw movement intent below threshold")
    if safe_mag <= MOVEMENT_THRESHOLD: reasons.append("safe command is zero/STOP-like")
    if not same_direction: reasons.append("safe direction differs from raw")
    if ratio is None or ratio > SLOWDOWN_CLASSIFICATION_RATIO: reasons.append("reduction threshold not met")
    if safe_mag > raw_mag + MATERIAL_EXCESS: reasons.append("safe materially exceeds raw")
    return {"slowdown": not reasons, "raw_magnitude": raw_mag,
            "safe_magnitude": safe_mag, "ratio": ratio,
            "same_direction": same_direction, "reasons": reasons}


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered: return None
    return ordered[int(round((len(ordered) - 1) * fraction))]


def summarize_window(node, start_ns, end_ns, expect_slow):
    pairs = pair_commands(node.samples["/cmd_vel_nav_raw"],
                          node.samples["/cmd_vel_nav_safe"], start_ns, end_ns)
    collision = [r for r in node.samples["/system/collision_monitor_valid"]
                 if start_ns <= r[0] <= end_ns and r[2]]
    results = [classify_slowdown_pair(r, s, collision_valid=bool(collision))
               for r, s, _ in pairs]
    moving = [x for x in results if x["raw_magnitude"] > MOVEMENT_THRESHOLD]
    ratios = [x["ratio"] for x in moving if x["ratio"] is not None]
    qualifying = [x for x in moving if x["slowdown"]]
    gate = [r for r in node.samples["/vehicle_cmd_safe"] if start_ns <= r[0] <= end_ns]
    mock = [r for r in node.samples["/wheelchair_control_command_mock"] if start_ns <= r[0] <= end_ns]
    odom = [r for r in node.samples["/Odometry"] if start_ns <= r[0] <= end_ns]
    tf = [r for r in node.tf_samples["odom->base_footprint"] if start_ns <= r[0] <= end_ns]
    if len(pairs) < 30 or not moving: raise RuntimeError("insufficient fresh paired command evidence")
    if expect_slow and len(qualifying) / len(moving) <= 0.5:
        raise RuntimeError(f"SLOWDOWN majority not established: {len(qualifying)}/{len(moving)}")
    if expect_slow and any(x["safe_magnitude"] <= MOVEMENT_THRESHOLD for x in moving):
        raise RuntimeError("STOP-like zero encountered in SLOWDOWN moving pairs")
    if not expect_slow and any(x["slowdown"] for x in moving):
        raise RuntimeError("unexpected SLOWDOWN classification in CLEAR")
    if any(x["safe_magnitude"] > x["raw_magnitude"] + MATERIAL_EXCESS for x in moving):
        raise RuntimeError("safe command materially exceeded raw")
    duration = (end_ns - start_ns) / 1e9
    if duration < 2.0: raise RuntimeError("window shorter than two seconds")
    return {
        "start_ns": start_ns, "end_ns": end_ns, "duration_sec": duration,
        "pair_count": len(pairs), "moving_pair_count": len(moving),
        "qualifying_slowdown_count": len(qualifying),
        "slowdown_fraction": len(qualifying) / len(moving),
        "ratio": {"minimum": min(ratios), "median": statistics.median(ratios),
                  "p95": percentile(ratios, .95), "maximum": max(ratios)},
        "raw_linear_x_median": statistics.median(r[2] for r, _, _ in pairs),
        "raw_angular_z_median": statistics.median(r[7] for r, _, _ in pairs),
        "safe_linear_x_median": statistics.median(s[2] for _, s, _ in pairs),
        "safe_angular_z_median": statistics.median(s[7] for _, s, _ in pairs),
        "gate_linear_x_median": statistics.median(r[2] for r in gate),
        "gate_angular_z_median": statistics.median(r[7] for r in gate),
        "odom_linear_x_median": statistics.median(abs(r[5]) for r in odom),
        "rates_hz": {"raw": rates(node.samples["/cmd_vel_nav_raw"], start_ns, end_ns),
                     "safe": rates(node.samples["/cmd_vel_nav_safe"], start_ns, end_ns),
                     "gate": rates(gate, start_ns, end_ns), "mock": rates(mock, start_ns, end_ns),
                     "odometry": rates(odom, start_ns, end_ns), "tf": rates(tf, start_ns, end_ns)},
    }


class SlowdownRunner(Runner):
    def __init__(self, out):
        super().__init__(out)
        self.set_mode_client = self.create_client(
            SetParameters, "/phase4_p4b_synthetic_obstacles/set_parameters")
        self._open("obstacle_mode_events.jsonl", "")

    def mode(self, value):
        if value not in ("CLEAR", "SLOW"): raise RuntimeError("P4-E.2A permits only CLEAR and SLOW")
        request_ns = time.monotonic_ns()
        request = {"event": "mode_change_request", "mode": value,
                   "monotonic_ns": request_ns}
        self.persist_event(request)
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("synthetic obstacle parameter service unavailable")
        msg = Parameter(name="mode", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=value))
        future = self.set_mode_client.call_async(SetParameters.Request(parameters=[msg]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().results[0].successful:
            raise RuntimeError(f"mode change failed: {value}")
        event = {"event": "mode_change_confirmation", "mode": value,
                 "request_ns": request_ns, "monotonic_ns": time.monotonic_ns()}
        self.files["obstacle_mode_events.jsonl"].write(json.dumps(event, sort_keys=True)+"\n")
        self.files["obstacle_mode_events.jsonl"].flush(); os.fsync(self.files["obstacle_mode_events.jsonl"].fileno())
        self.persist_event(event)
        return event


def main(args=None):
    parser = argparse.ArgumentParser(); parser.add_argument("--output-dir", required=True)
    ns = parser.parse_args(args); out = Path(ns.output_dir); metrics = {}; error = None
    rclpy.init(); node = SlowdownRunner(out)
    try:
        env = {k:v for k,v in os.environ.items() if k in ("ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION")}
        (out/"process_environment.tsv").write_text("key\tvalue\n"+"".join(f"{k}\t{v}\n" for k,v in sorted(env.items())), encoding="utf-8")
        node.spin(5.0); node.graph(); states=node.lifecycle()
        pubs={t:len(node.get_publishers_info_by_topic(t)) for t in TOPICS}
        raw_nodes={ep.node_name for ep in node.get_publishers_info_by_topic("/cmd_vel_nav_raw")}
        if any(states.get(n,(0,""))[1] != "active" for n in LIFECYCLE): raise RuntimeError(f"inactive lifecycle: {states}")
        if any(pubs[t] != 1 for t in TOPICS[1:]): raise RuntimeError(f"publisher authority: {pubs}")
        if not raw_nodes or not raw_nodes <= {"controller_server","behavior_server"}: raise RuntimeError(f"raw authority: {raw_nodes}")
        readiness_start=time.monotonic_ns(); pose=node.latest_odom; node.spin(3.0); readiness_end=time.monotonic_ns()
        readiness_rates={"gate":rates(node.samples["/vehicle_cmd_safe"],readiness_start,readiness_end),"mock":rates(node.samples["/wheelchair_control_command_mock"],readiness_start,readiness_end),"odometry":rates(node.samples["/Odometry"],readiness_start,readiness_end),"tf":rates(node.tf_samples["odom->base_footprint"],readiness_start,readiness_end)}
        if not (18<=readiness_rates["gate"]<=22 and 18<=readiness_rates["mock"]<=22 and 48<=readiness_rates["odometry"]<=52 and 48<=readiness_rates["tf"]<=52): raise RuntimeError(f"readiness rates {readiness_rates}")
        if any(nz(r[2:8]) for r in node.samples["/vehicle_cmd_safe"] if readiness_start<=r[0]<=readiness_end): raise RuntimeError("nonzero disarmed gate")
        if pose is None or math.hypot(node.latest_odom[0]-pose[0],node.latest_odom[1]-pose[1])>.002: raise RuntimeError("readiness drift")
        readiness={"event":"P4E2A_SLOWDOWN_CHAIN_READINESS_PASS","monotonic_ns":time.monotonic_ns(),"rates_hz":readiness_rates}; node.persist_event(readiness)
        node.publish_initial(); node.spin(1.0); goal=node.start_goal(); interlock=node.wait_fresh_safe_disarmed(); fresh_ns=time.monotonic_ns(); arm=node.arm_gate()
        clear_start=time.monotonic_ns(); node.spin(2.2); clear_end=time.monotonic_ns(); clear=summarize_window(node,clear_start,clear_end,False)
        node.persist_event({"event":"P4E2A_CLEAR_REFERENCE_WINDOW_PASS","monotonic_ns":clear_end})
        slow_event=node.mode("SLOW"); slow_start=slow_event["monotonic_ns"]+300_000_000; node.spin(2.7); slow_end=time.monotonic_ns(); slow=summarize_window(node,slow_start,slow_end,True)
        node.persist_event({"event":"P4E2A_SLOWDOWN_FULL_CHAIN_PASS","monotonic_ns":slow_end})
        clear_event=node.mode("CLEAR"); recovery_start=clear_event["monotonic_ns"]+500_000_000; node.spin(.8); recovery_end=time.monotonic_ns(); recovery=summarize_window(node,recovery_start,recovery_end,False) if recovery_end-recovery_start>=2_000_000_000 else None
        # Preserve a full two-second recovery window while the same goal remains active.
        if recovery is None:
            node.spin(1.8); recovery_end=time.monotonic_ns(); recovery=summarize_window(node,recovery_start,recovery_end,False)
        node.persist_event({"event":"P4E2A_SLOWDOWN_CLEAR_RECOVERY_PASS","monotonic_ns":recovery_end})
        node.wait_goal_done(); result_ns=node.events[-1]["monotonic_ns"]; node.spin(2.2); end_ns=time.monotonic_ns(); node.graph(); node.lifecycle()
        post=math.hypot(node.latest_odom[0]-node.success_pose[0],node.latest_odom[1]-node.success_pose[1])
        layers={"raw":[(x[0],nz(x[2:8])) for x in node.samples["/cmd_vel_nav_raw"]],"safe":[(x[0],nz(x[2:8])) for x in node.samples["/cmd_vel_nav_safe"]],"gate":[(x[0],nz(x[2:8])) for x in node.samples["/vehicle_cmd_safe"]],"mock":[(x[0],nz(x[2])) for x in node.samples["/wheelchair_control_command_mock"]],"fake":[(x["monotonic_ns"],nz((x["values"].get("applied_linear_x",0),x["values"].get("applied_angular_z",0)))) for x in node.diagnostic_history["fake_base"]]}
        terminal=adjudicate_terminal_stop(layers,result_ns,True,end_ns,post)
        if not terminal["pass"]: raise RuntimeError(terminal["final_reason"])
        if node.goal_request_count != 1 or node.arm_request_count != 1: raise RuntimeError("goal/arm cardinality")
        node.persist_event({"event":"P4E2A_SLOWDOWN_POST_SUCCESS_STOP_PASS","monotonic_ns":time.monotonic_ns()})
        metrics={"pass":True,"status":"PHASE4_P4E2A_SLOWDOWN_FULL_CHAIN_COMPLETED_NEEDS_REVIEW","goal_status":"SUCCEEDED","goal_request_count":1,"arm_request_count":1,"configured_slowdown_ratio":.30,"classification_threshold":.80,"publisher_counts":pubs,"raw_nodes":sorted(raw_nodes),"lifecycle":states,"readiness":readiness,"interlock":interlock,"safe_freshness_ns":fresh_ns,"arm_response":arm,"goal_response":goal,"clear_reference":clear,"slowdown":slow,"clear_recovery":recovery,"mode_events":[slow_event,clear_event],"terminal_stop":terminal,"post_success_translation_m":post}
    except Exception as exc:
        error=repr(exc); metrics={"pass":False,"status":"P4E2A_SLOWDOWN_RUNTIME_CONTRACT_NEEDS_REVIEW","error":error}
    finally:
        (out/"terminal_metrics.json").write_text(json.dumps(metrics,indent=2,sort_keys=True)+"\n",encoding="utf-8")
        node.close(); node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)

