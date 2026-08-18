"""Bounded CLEAR -> STOP -> CLEAR full-chain qualification for P4-E.2B."""
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
    LIFECYCLE, TOPICS, Runner, adjudicate_terminal_stop, nz, rates,
)
from parking_robot_bringup.phase4_p4e2a_slowdown_runner import pair_commands

LINEAR_INTENT_THRESHOLD = 0.01
ANGULAR_INTENT_THRESHOLD = 0.02
PAIR_SKEW_NS = 75_000_000


def classify_stop_pair(raw, safe, *, paired_fresh=True, collision_valid=True,
                       permissions_valid=True, gate_state="ARMED",
                       gate_fault=False, cancellation_intent=False, terminal=False):
    raw_intent = (abs(float(raw[2])) > LINEAR_INTENT_THRESHOLD or
                  abs(float(raw[7])) > ANGULAR_INTENT_THRESHOLD)
    safe_zero = (abs(float(safe[2])) < LINEAR_INTENT_THRESHOLD and
                 abs(float(safe[7])) < ANGULAR_INTENT_THRESHOLD)
    health_failure = (not collision_valid or not permissions_valid or gate_fault or
                      gate_state == "FAULT")
    reasons = []
    if not paired_fresh: reasons.append("raw/safe pair is stale")
    if health_failure: reasons.append("health failure")
    if gate_state != "ARMED": reasons.append("gate is not ARMED")
    if cancellation_intent: reasons.append("cancellation intent")
    if terminal: reasons.append("terminal result")
    if not raw_intent: reasons.append("raw movement intent absent")
    if not safe_zero: reasons.append("safe command is nonzero/SLOWDOWN")
    return {"stop": not reasons, "raw_intent": raw_intent,
            "safe_zero": safe_zero, "health_failure": health_failure,
            "reasons": reasons}


class StopClearLatch:
    """Require a continuous CLEAR interval before releasing STOP classification."""
    def __init__(self, stable_ns=500_000_000):
        self.stable_ns = stable_ns; self.stop_active = False; self.clear_since = None

    def observe(self, stamp_ns, is_stop, is_clear):
        if is_stop:
            self.stop_active = True; self.clear_since = None
        elif self.stop_active and is_clear:
            if self.clear_since is None: self.clear_since = stamp_ns
            if stamp_ns - self.clear_since >= self.stable_ns: self.stop_active = False
        else:
            self.clear_since = None
        return self.stop_active


def command_mag(row): return math.hypot(float(row[2]), float(row[7]))


class TemporaryStopRunner(Runner):
    def __init__(self, out):
        super().__init__(out)
        self.set_mode_client = self.create_client(
            SetParameters, "/phase4_p4b_synthetic_obstacles/set_parameters")
        self._open("obstacle_mode_events.jsonl", "")

    def mode(self, value):
        if value not in ("CLEAR", "STOP"):
            raise RuntimeError("P4-E.2B permits only CLEAR and STOP")
        request_ns=time.monotonic_ns(); request={"event":"mode_change_request","mode":value,"monotonic_ns":request_ns}; self.persist_event(request)
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0): raise RuntimeError("mode service unavailable")
        param=Parameter(name="mode",value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=value))
        future=self.set_mode_client.call_async(SetParameters.Request(parameters=[param])); rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().results[0].successful: raise RuntimeError(f"mode change failed: {value}")
        event={"event":"mode_change_confirmation","mode":value,"request_ns":request_ns,"monotonic_ns":time.monotonic_ns()}
        self.files["obstacle_mode_events.jsonl"].write(json.dumps(event,sort_keys=True)+"\n"); self.files["obstacle_mode_events.jsonl"].flush(); os.fsync(self.files["obstacle_mode_events.jsonl"].fileno()); self.persist_event(event)
        return event


def clear_summary(node, start, end):
    pairs=pair_commands(node.samples["/cmd_vel_nav_raw"],node.samples["/cmd_vel_nav_safe"],start,end)
    moving=[(r,s,k) for r,s,k in pairs if command_mag(r)>.01 and command_mag(s)>.01]
    if end-start<2_000_000_000 or len(moving)<30: raise RuntimeError("insufficient CLEAR reference")
    if any(abs(command_mag(r)-command_mag(s))>.05 for r,s,_ in moving): raise RuntimeError("CLEAR raw/safe mismatch")
    return {"start_ns":start,"end_ns":end,"duration_sec":(end-start)/1e9,"pair_count":len(pairs),"moving_pair_count":len(moving),"raw_linear_median":statistics.median(r[2] for r,_,_ in moving),"safe_linear_median":statistics.median(s[2] for _,s,_ in moving)}


def stop_summary(node, start, end):
    pairs=pair_commands(node.samples["/cmd_vel_nav_raw"],node.samples["/cmd_vel_nav_safe"],start,end)
    results=[classify_stop_pair(r,s) for r,s,_ in pairs]
    raw_fraction=sum(x["raw_intent"] for x in results)/len(results) if results else 0
    zero_fraction=sum(x["safe_zero"] for x in results)/len(results) if results else 0
    gate=[x for x in node.samples["/vehicle_cmd_safe"] if start<=x[0]<=end]
    mock=[x for x in node.samples["/wheelchair_control_command_mock"] if start<=x[0]<=end]
    odom=[x for x in node.samples["/Odometry"] if start<=x[0]<=end]
    tf=[x for x in node.tf_samples["odom->base_footprint"] if start<=x[0]<=end]
    fake=[x for x in node.diagnostic_history["fake_base"] if start<=x["monotonic_ns"]<=end]
    perms={t:[x for x in node.samples[t] if start<=x[0]<=end] for t in TOPICS[2:5]}
    if end-start<2_000_000_000 or len(pairs)<30: raise RuntimeError("insufficient STOP window")
    if raw_fraction<.90 or zero_fraction!=1.0 or not all(x["stop"] for x in results if x["raw_intent"]): raise RuntimeError(f"STOP classification fractions {raw_fraction},{zero_fraction}")
    if any(nz(x[2:8]) for x in gate): raise RuntimeError("gate nonzero during STOP")
    if any(len(x[2])!=3 or nz(x[2]) for x in mock): raise RuntimeError("mock nonzero during STOP")
    if any(nz((x["values"].get("applied_linear_x",0),x["values"].get("applied_angular_z",0))) for x in fake): raise RuntimeError("fake applied nonzero during STOP")
    if any(not rows or any(not x[2] for x in rows) for rows in perms.values()): raise RuntimeError("permission invalid during STOP")
    gate_diag=node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
    if gate_diag is None or gate_diag[1].get("state")!="ARMED" or gate_diag[1].get("fault_latched")!="false": raise RuntimeError(f"gate unhealthy {gate_diag}")
    if node.result_future.done(): raise RuntimeError("goal became terminal during bounded STOP")
    displacement=math.hypot(float(odom[-1][2])-float(odom[0][2]),float(odom[-1][3])-float(odom[0][3]))
    window_rates={"raw":rates(node.samples["/cmd_vel_nav_raw"],start,end),"safe":rates(node.samples["/cmd_vel_nav_safe"],start,end),"gate":rates(gate,start,end),"mock":rates(mock,start,end),"odometry":rates(odom,start,end),"tf":rates(tf,start,end)}
    if not (18<=window_rates["gate"]<=22 and 18<=window_rates["mock"]<=22 and 48<=window_rates["odometry"]<=52 and 48<=window_rates["tf"]<=52): raise RuntimeError(f"STOP rates {window_rates}")
    if displacement>.02: raise RuntimeError(f"STOP displacement {displacement}")
    return {"start_ns":start,"end_ns":end,"duration_sec":(end-start)/1e9,"pair_count":len(pairs),"raw_intent_fraction":raw_fraction,"safe_zero_fraction":zero_fraction,"translation_m":displacement,"rates_hz":window_rates}


def first_after(rows, stamp, predicate): return next(x for x in rows if x[0]>=stamp and predicate(x))


def healthy_stop_checkpoint(node, confirmation_ns, target_offset_ns, label):
    """Capture the first observation at/after a frozen STOP-time boundary."""
    target_ns = confirmation_ns + target_offset_ns
    while time.monotonic_ns() < target_ns:
        rclpy.spin_once(node, timeout_sec=.01)
    observed_ns = time.monotonic_ns()
    safe = [x for x in node.samples["/cmd_vel_nav_safe"] if x[0] <= observed_ns]
    if not safe:
        raise RuntimeError(f"{label}: no safe sample")
    latest = safe[-1]
    safe_age_ns = observed_ns - latest[0]
    gate_diag = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
    permissions = {
        topic: node.permissions.get(topic) for topic in (
            "/system/collision_monitor_valid",
            "/system/localization_valid",
            "/system/controller_valid",
        )
    }
    permission_ok = all(value and value[0] and observed_ns - value[1] <= 500_000_000
                        for value in permissions.values())
    if nz(latest[2:8]) or safe_age_ns > 250_000_000:
        raise RuntimeError(f"{label}: safe zero heartbeat unhealthy age={safe_age_ns}")
    if gate_diag is None or gate_diag[1].get("state") != "ARMED" or gate_diag[1].get("fault_latched") != "false":
        raise RuntimeError(f"{label}: gate unhealthy {gate_diag}")
    if not permission_ok:
        raise RuntimeError(f"{label}: permission health/freshness failed {permissions}")
    event = {
        "event": label,
        "monotonic_ns": observed_ns,
        "target_ns": target_ns,
        "target_offset_ns": target_offset_ns,
        "actual_offset_ns": observed_ns - confirmation_ns,
        "latest_safe_ns": latest[0],
        "latest_safe_age_ns": safe_age_ns,
        "safe_is_zero": True,
        "safe_publisher_count": len(node.get_publishers_info_by_topic("/cmd_vel_nav_safe")),
        "gate_state": gate_diag[1].get("state"),
        "fault_latched": gate_diag[1].get("fault_latched"),
        "permissions": {topic: value[0] for topic, value in permissions.items()},
    }
    node.persist_event(event)
    return event


def main(args=None):
    parser=argparse.ArgumentParser(); parser.add_argument("--output-dir",required=True); ns=parser.parse_args(args)
    out=Path(ns.output_dir); metrics={}; error=None; rclpy.init(); node=TemporaryStopRunner(out)
    try:
        env={k:v for k,v in os.environ.items() if k in ("ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION")}; (out/"process_environment.tsv").write_text("key\tvalue\n"+"".join(f"{k}\t{v}\n" for k,v in sorted(env.items())),encoding="utf-8")
        node.spin(5); node.graph(); states=node.lifecycle(); pubs={t:len(node.get_publishers_info_by_topic(t)) for t in TOPICS}; raw_nodes={e.node_name for e in node.get_publishers_info_by_topic("/cmd_vel_nav_raw")}
        if any(states.get(n,(0,""))[1]!="active" for n in LIFECYCLE): raise RuntimeError(f"inactive lifecycle {states}")
        if any(pubs[t]!=1 for t in TOPICS[1:]): raise RuntimeError(f"publisher authority {pubs}")
        if not raw_nodes or not raw_nodes<={"controller_server","behavior_server"}: raise RuntimeError(f"raw authority {raw_nodes}")
        ready_start=time.monotonic_ns(); pose=node.latest_odom; node.spin(3); ready_end=time.monotonic_ns(); rr={"gate":rates(node.samples["/vehicle_cmd_safe"],ready_start,ready_end),"mock":rates(node.samples["/wheelchair_control_command_mock"],ready_start,ready_end),"odometry":rates(node.samples["/Odometry"],ready_start,ready_end),"tf":rates(node.tf_samples["odom->base_footprint"],ready_start,ready_end)}
        if not (18<=rr["gate"]<=22 and 18<=rr["mock"]<=22 and 48<=rr["odometry"]<=52 and 48<=rr["tf"]<=52): raise RuntimeError(f"readiness rates {rr}")
        if any(nz(x[2:8]) for x in node.samples["/vehicle_cmd_safe"] if ready_start<=x[0]<=ready_end): raise RuntimeError("disarmed output nonzero")
        if pose is None or math.hypot(node.latest_odom[0]-pose[0],node.latest_odom[1]-pose[1])>.002: raise RuntimeError("readiness drift")
        readiness={"event":"P4E2B_TEMPORARY_STOP_CHAIN_READINESS_PASS","monotonic_ns":time.monotonic_ns(),"rates_hz":rr}; node.persist_event(readiness)
        node.publish_initial(); node.spin(1); goal=node.start_goal(); interlock=node.wait_fresh_safe_disarmed(); fresh=time.monotonic_ns(); arm=node.arm_gate()
        clear_start=time.monotonic_ns(); node.spin(2.2); clear_end=time.monotonic_ns(); clear=clear_summary(node,clear_start,clear_end); node.persist_event({"event":"P4E2B_CLEAR_REFERENCE_WINDOW_PASS","monotonic_ns":clear_end})
        stop_event=node.mode("STOP"); stop_confirmation=stop_event["monotonic_ns"]
        raw_zero_pair=pair_commands(node.samples["/cmd_vel_nav_raw"],node.samples["/cmd_vel_nav_safe"],stop_confirmation,time.monotonic_ns())
        node.spin(.35); stable_start=time.monotonic_ns(); stop_pose=node.latest_odom
        old_boundary=healthy_stop_checkpoint(node,stop_confirmation,2_000_000_000,"P4E2B2_OLD_2S_TIMEOUT_BOUNDARY_SURVIVED_PASS")
        extended_boundary=healthy_stop_checkpoint(node,stop_confirmation,3_000_000_000,"P4E2B2_EXTENDED_SAFE_ZERO_HEARTBEAT_PASS")
        while time.monotonic_ns() < stop_confirmation + 3_500_000_000:
            rclpy.spin_once(node,timeout_sec=.01)
        stable_end=time.monotonic_ns(); stop=stop_summary(node,stable_start,stable_end); stop["total_deliberate_interval_ns_so_far"]=stable_end-stop_confirmation
        stop_safe=[x for x in node.samples["/cmd_vel_nav_safe"] if stop_confirmation<=x[0]<=stable_end]
        stop["safe_zero_publication_duration_ns"]=stop_safe[-1][0]-stop_safe[0][0]
        stop["safe_zero_rate_hz"]=rates(stop_safe,stop_confirmation,stable_end)
        stop["safe_sample_age_at_end_ns"]=stable_end-stop_safe[-1][0]
        node.persist_event({"event":"P4E2B_TEMPORARY_STOP_FULL_CHAIN_PASS","monotonic_ns":stable_end})
        clear_event=node.mode("CLEAR"); clear_confirmation=clear_event["monotonic_ns"]; deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            pairs=pair_commands(node.samples["/cmd_vel_nav_raw"],node.samples["/cmd_vel_nav_safe"],clear_confirmation,time.monotonic_ns())
            moving=[(r,s) for r,s,_ in pairs if command_mag(r)>.01 and command_mag(s)>.01 and abs(command_mag(r)-command_mag(s))<=.05]
            if moving and time.monotonic_ns()-moving[0][1][0]>=500_000_000: break
        else: raise RuntimeError("stable CLEAR recovery timeout")
        recovered_ns=time.monotonic_ns()
        if node.result_future.done(): raise RuntimeError("goal terminal before recovery proof")
        gate_nonzero=first_after(node.samples["/vehicle_cmd_safe"],clear_confirmation,lambda x:nz(x[2:8])); fake_nonzero=next(x for x in node.diagnostic_history["fake_base"] if x["monotonic_ns"]>=clear_confirmation and nz((x["values"].get("applied_linear_x",0),x["values"].get("applied_angular_z",0)))); odom_nonzero=first_after(node.samples["/Odometry"],clear_confirmation,lambda x:abs(x[5])>.01 or abs(x[6])>.02)
        node.persist_event({"event":"P4E2B_TEMPORARY_STOP_CLEAR_RECOVERY_PASS","monotonic_ns":recovered_ns})
        node.wait_goal_done(); result_ns=node.events[-1]["monotonic_ns"]; node.spin(2.2); end=time.monotonic_ns(); node.graph(); node.lifecycle(); post=math.hypot(node.latest_odom[0]-node.success_pose[0],node.latest_odom[1]-node.success_pose[1])
        layers={"raw":[(x[0],nz(x[2:8])) for x in node.samples["/cmd_vel_nav_raw"]],"safe":[(x[0],nz(x[2:8])) for x in node.samples["/cmd_vel_nav_safe"]],"gate":[(x[0],nz(x[2:8])) for x in node.samples["/vehicle_cmd_safe"]],"mock":[(x[0],nz(x[2])) for x in node.samples["/wheelchair_control_command_mock"]],"fake":[(x["monotonic_ns"],nz((x["values"].get("applied_linear_x",0),x["values"].get("applied_angular_z",0)))) for x in node.diagnostic_history["fake_base"]]}
        terminal=adjudicate_terminal_stop(layers,result_ns,True,end,post)
        if not terminal["pass"]: raise RuntimeError(terminal["final_reason"])
        if node.goal_request_count!=1 or node.arm_request_count!=1: raise RuntimeError("goal/arm cardinality")
        node.persist_event({"event":"P4E2B_TEMPORARY_STOP_SAME_GOAL_SUCCESS_PASS","monotonic_ns":result_ns,"goal_uuid":goal["goal_uuid"]})
        node.persist_event({"event":"P4E2B_TEMPORARY_STOP_POST_SUCCESS_PASS","monotonic_ns":time.monotonic_ns()})
        metrics={"pass":True,"status":"PHASE4_P4E2B2_TEMPORARY_STOP_CLEAR_RUNTIME_COMPLETED_NEEDS_REVIEW","goal_status":"SUCCEEDED","goal_request_count":1,"arm_request_count":1,"publisher_counts":pubs,"raw_nodes":sorted(raw_nodes),"lifecycle":states,"readiness":readiness,"interlock":interlock,"safe_freshness_ns":fresh,"arm_response":arm,"goal_response":goal,"clear_reference":clear,"stop_window":stop,"stop_event":stop_event,"old_2s_checkpoint":old_boundary,"extended_3s_checkpoint":extended_boundary,"clear_event":clear_event,"recovery":{"proof_ns":recovered_ns,"gate_nonzero_ns":gate_nonzero[0],"fake_nonzero_ns":fake_nonzero["monotonic_ns"],"odom_nonzero_ns":odom_nonzero[0]},"terminal_stop":terminal,"post_success_translation_m":post}
    except Exception as exc:
        error=repr(exc); metrics={"pass":False,"status":"P4E2B2_TEMPORARY_STOP_RECOVERY_CONTRACT_NEEDS_REVIEW","error":error}
    finally:
        (out/"terminal_metrics.json").write_text(json.dumps(metrics,indent=2,sort_keys=True)+"\n",encoding="utf-8"); node.close(); node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)
