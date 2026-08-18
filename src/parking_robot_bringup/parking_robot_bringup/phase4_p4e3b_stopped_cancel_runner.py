"""P4-E.3B CLEAR -> healthy STOP -> user cancel while pre-stopped."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from parking_robot_interfaces.msg import MissionState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy

from .phase4_p4e1b_clear_runner import TOPICS, LIFECYCLE, nz, rates
from .phase4_p4e3a_mission_cancel_runner import MissionCancelRunner


def adjudicate_pre_stopped_user_cancel(layers, cancel_ns, stop_confirmation_ns,
                                       full_downstream_zero_ns, action_status,
                                       mission_state, observation_end_ns,
                                       post_cancel_translation_m, *, uuid_count=1,
                                       waypoint_one_dispatched=False,
                                       minimum_stop_ns=500_000_000,
                                       maximum_stop_to_cancel_ns=1_000_000_000,
                                       minimum_stationary_ns=2_500_000_000,
                                       freshness_ns=150_000_000):
    """Pure P4-E.3B evidence adjudicator anchored to the original STOP zeros."""
    report={"mode":"PRE_STOPPED_USER_CANCEL","cancel_request_ns":int(cancel_ns),
            "stop_confirmation_ns":int(stop_confirmation_ns),"layers":{},"reasons":[]}
    for name in ("raw","safe","gate","mock","fake"):
        samples=sorted((int(t),bool(v)) for t,v in layers.get(name,()))
        before=[x for x in samples if x[0]<=cancel_ns]
        latest=before[-1] if before else None
        report["layers"][name]={"latest_at_cancel":latest,"age_at_cancel_ns":None if latest is None else cancel_ns-latest[0]}
        if latest is None or cancel_ns-latest[0]>freshness_ns:
            report["reasons"].append(f"{name}: no fresh sample at cancel")
        expected_nonzero=name=="raw"
        if latest is not None and latest[1]!=expected_nonzero:
            report["reasons"].append(f"{name}: wrong pre-stopped value at cancel")
        if name!="raw":
            # The first retained STOP zero is frozen before cancel. Any later
            # nonzero is a violation; a later zero can never conceal it.
            zeros=[t for t,v in samples if stop_confirmation_ns<=t<=cancel_ns and not v]
            first_zero=zeros[0] if zeros else None
            later_nonzero=first_zero is not None and any(t>first_zero and v for t,v in samples)
            report["layers"][name].update({"original_stop_zero_ns":first_zero,"later_nonzero_violation":later_nonzero})
            if first_zero is None: report["reasons"].append(f"{name}: no STOP zero before cancel")
            if later_nonzero: report["reasons"].append(f"{name}: nonzero after original STOP zero")
    stop_to_cancel=cancel_ns-stop_confirmation_ns; zero_to_cancel=cancel_ns-full_downstream_zero_ns
    stationary=observation_end_ns-cancel_ns
    report.update({"chain_already_zero_at_cancel":True,"stop_confirmation_to_cancel_ns":stop_to_cancel,
                   "full_downstream_zero_to_cancel_ns":zero_to_cancel,"stationary_duration_ns":stationary,
                   "action_status":int(action_status),"mission_state":int(mission_state),"uuid_count":int(uuid_count),
                   "waypoint_one_dispatched":bool(waypoint_one_dispatched),"post_cancel_translation_m":float(post_cancel_translation_m)})
    if zero_to_cancel<minimum_stop_ns: report["reasons"].append("stable pre-cancel STOP below 0.50 seconds")
    if stop_to_cancel>=maximum_stop_to_cancel_ns: report["reasons"].append("cancel request reached 1.0-second temporary-block threshold")
    if action_status!=GoalStatus.STATUS_CANCELED: report["reasons"].append("action terminal is not CANCELED")
    if mission_state!=MissionState.CANCELLED: report["reasons"].append("mission terminal is not CANCELLED")
    if uuid_count!=1: report["reasons"].append("second action UUID observed")
    if waypoint_one_dispatched: report["reasons"].append("waypoint 1 dispatched")
    if stationary<minimum_stationary_ns: report["reasons"].append("post-cancel stationary observation below 2.5 seconds")
    if post_cancel_translation_m>0.02: report["reasons"].append("post-cancel translation exceeds 0.02 m")
    report["pass"]=not report["reasons"]
    report["final_reason"]="PASS" if report["pass"] else "; ".join(report["reasons"])
    return report


class StoppedCancelRunner(MissionCancelRunner):
    def __init__(self,out:Path):
        super().__init__(out)
        self.set_mode_client=self.create_client(SetParameters,"/phase4_p4b_synthetic_obstacles/set_parameters")
        self._open("obstacle_mode_events.jsonl","")

    def mode(self,value):
        if value not in ("CLEAR","STOP"): raise RuntimeError("P4-E.3B permits only CLEAR and STOP")
        request=self.emit("obstacle_mode_events.jsonl",{"event":"mode_change_request","mode":value})
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0): raise RuntimeError("mode service unavailable")
        param=Parameter(name="mode",value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=value))
        future=self.set_mode_client.call_async(SetParameters.Request(parameters=[param])); rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().results[0].successful: raise RuntimeError(f"mode change failed: {value}")
        return self.emit("obstacle_mode_events.jsonl",{"event":"mode_change_confirmation","mode":value,"request_ns":request["monotonic_ns"]})

    def wait_all_stop_zero(self,after_ns,timeout=2.0):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.01)
            rows={"safe":[x for x in self.samples["/cmd_vel_nav_safe"] if x[0]>=after_ns],
                  "gate":[x for x in self.samples["/vehicle_cmd_safe"] if x[0]>=after_ns],
                  "mock":[x for x in self.samples["/wheelchair_control_command_mock"] if x[0]>=after_ns]}
            fake=[x for x in self.diagnostic_history["fake_base"] if x["monotonic_ns"]>=after_ns]
            zeros={"safe":next((x[0] for x in rows["safe"] if not nz(x[2:8])),None),
                   "gate":next((x[0] for x in rows["gate"] if not nz(x[2:8])),None),
                   "mock":next((x[0] for x in rows["mock"] if not nz(x[2])),None),
                   "fake":next((x["monotonic_ns"] for x in fake if not nz((x["values"].get("applied_linear_x",0),x["values"].get("applied_angular_z",0)))),None)}
            raw=[x for x in self.samples["/cmd_vel_nav_raw"] if x[0]>=after_ns and nz(x[2:8])]
            if all(zeros.values()) and raw and time.monotonic_ns()-raw[-1][0]<150_000_000: return zeros
        raise RuntimeError("full downstream STOP zero timeout")


def main(args=None):
    ap=argparse.ArgumentParser(); ap.add_argument("--output-dir",required=True); ns=ap.parse_args(args)
    out=Path(ns.output_dir); out.mkdir(parents=True,exist_ok=True); error=None; metrics={}; rclpy.init(); n=StoppedCancelRunner(out)
    try:
        with (out/"process_environment.tsv").open("w",encoding="utf-8") as h:
            h.write("key\tvalue\n"); [h.write(f"{k}\t{v}\n") for k,v in sorted(os.environ.items()) if k in ("ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION")]
        n.spin(5); n.graph(); lifecycle=n.lifecycle(); nodes=set(n.get_node_names())
        if any(lifecycle.get(x,(0,""))[1]!="active" for x in LIFECYCLE): raise RuntimeError(f"inactive lifecycle: {lifecycle}")
        if any(any(y in x.lower() for y in ("plan_nav","rtab","fast_lio","pure_pursuit","wheelchair_controller")) for x in nodes): raise RuntimeError(f"forbidden nodes: {nodes}")
        if sum(1 for x in n.get_node_names() if x=="mission_manager")!=1: raise RuntimeError("mission manager ownership")
        pubs={t:len(n.get_publishers_info_by_topic(t)) for t in TOPICS}
        if any(pubs[t]!=1 for t in TOPICS[1:]): raise RuntimeError(f"publisher authority: {pubs}")
        if n.scan_count<50 or any(not n.permissions.get(t,(False,0))[0] for t in TOPICS[2:5]): raise RuntimeError("validity not fresh true")
        gate=n.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        if gate is None or gate[1].get("state")!="DISARMED" or gate[1].get("fault_latched")!="false": raise RuntimeError(f"gate readiness: {gate}")
        ready=n.emit("scenario_events.jsonl",{"event":"P4E3B_STOPPED_CANCEL_CHAIN_READINESS_PASS","lifecycle":lifecycle,"publisher_counts":pubs})
        n.publish_initial(); n.spin(.5); route=n.publish_route(); n.wait_for(lambda:any(s["state"]==MissionState.RECEIVED for s in n.states),5,"RECEIVED timeout")
        start_req,start_resp=n.trigger("start"); n.wait_for(lambda:any(s["state"]==MissionState.NAVIGATING and s["active_goal_uuid"] for s in n.states),10,"NAVIGATING timeout")
        nav=next(s for s in n.states if s["state"]==MissionState.NAVIGATING and s["active_goal_uuid"]); uuid=nav["active_goal_uuid"]
        n.wait_fresh_safe_disarmed(); arm=n.arm_gate(); motion_start=time.monotonic_ns(); n.wait_for(lambda:any(x[0]>=motion_start and nz(x[2:8]) for x in n.samples["/vehicle_cmd_safe"]),5,"motion timeout")
        clear_start=time.monotonic_ns(); n.spin(2.1)
        if n.states[-1]["state"]!=MissionState.NAVIGATING or n.states[-1]["waypoint_index"]!=0: raise RuntimeError("not waypoint-0 NAVIGATING")
        clear=n.emit("scenario_events.jsonl",{"event":"P4E3B_PRE_STOP_CLEAR_MOTION_PASS","duration_ns":time.monotonic_ns()-clear_start,"goal_uuid":uuid})
        stop=n.mode("STOP"); stop_ns=stop["monotonic_ns"]; stop_pose=n.latest_odom; zeros=n.wait_all_stop_zero(stop_ns); full_zero=max(zeros.values())
        target=full_zero+520_000_000
        while time.monotonic_ns()<target: rclpy.spin_once(n,timeout_sec=.01)
        cancel_pose=n.latest_odom; cancel_req,cancel_resp=n.trigger("cancel"); cancel_ns=cancel_req["monotonic_ns"]
        if cancel_ns-stop_ns>=1_000_000_000: raise RuntimeError("cancel crossed 1.0-second threshold")
        if math.hypot(cancel_pose[0]-stop_pose[0],cancel_pose[1]-stop_pose[1])>.02: raise RuntimeError("pre-cancel STOP drift")
        pre=n.emit("scenario_events.jsonl",{"event":"P4E3B_PRE_CANCEL_HEALTHY_STOP_PASS","stop_confirmation_ns":stop_ns,"full_zero_ns":full_zero,"cancel_ns":cancel_ns,"zeros":zeros,"chain_already_zero_at_cancel":True})
        n.wait_for(lambda:any(s["state"]==MissionState.CANCELLING for s in n.states if s["monotonic_ns"]>=cancel_ns),5,"CANCELLING timeout")
        n.wait_for(lambda:any(x["goal_uuid"]==uuid and x["status"]==GoalStatus.STATUS_CANCELED for x in n.action_statuses),8,"CANCELED timeout")
        n.wait_for(lambda:any(s["state"]==MissionState.CANCELLED for s in n.states),8,"CANCELLED timeout")
        terminal_ns=time.monotonic_ns(); gate=n.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        if gate is None or gate[1].get("state")!="ARMED" or gate[1].get("fault_latched")!="false": raise RuntimeError(f"gate not healthy armed at terminal: {gate}")
        n.emit("scenario_events.jsonl",{"event":"P4E3B_STOPPED_USER_CANCEL_PROTOCOL_PASS","terminal_evidence_ns":terminal_ns})
        n.spin(2.7); end=time.monotonic_ns(); n.graph(); final=n.states[-1]
        post=math.hypot(n.latest_odom[0]-cancel_pose[0],n.latest_odom[1]-cancel_pose[1]); yaw=abs(n.latest_odom[2]-cancel_pose[2])
        fake=[(x["monotonic_ns"],nz((x["values"].get("applied_linear_x",0),x["values"].get("applied_angular_z",0)))) for x in n.diagnostic_history["fake_base"]]
        layers={"raw":[(x[0],nz(x[2:8])) for x in n.samples["/cmd_vel_nav_raw"]],"safe":[(x[0],nz(x[2:8])) for x in n.samples["/cmd_vel_nav_safe"]],"gate":[(x[0],nz(x[2:8])) for x in n.samples["/vehicle_cmd_safe"]],"mock":[(x[0],nz(x[2])) for x in n.samples["/wheelchair_control_command_mock"]],"fake":fake}
        uuids=set(g[1] for g in n.goal_uuids); later=any(g[2]>0 for g in n.goal_uuids)
        adj=adjudicate_pre_stopped_user_cancel(layers,cancel_ns,stop_ns,full_zero,GoalStatus.STATUS_CANCELED,final["state"],end,post,uuid_count=len(uuids),waypoint_one_dispatched=later)
        if not adj["pass"]: raise RuntimeError(adj["final_reason"])
        n.emit("scenario_events.jsonl",{"event":"P4E3B_STOPPED_USER_CANCEL_POST_TERMINAL_PASS"})
        metrics={"pass":True,"status":"PHASE4_P4E3B_STOPPED_MISSION_CANCEL_FULL_CHAIN_COMPLETED_NEEDS_REVIEW","chain_already_zero_at_cancel":True,"route_count":n.route_count,"start_count":n.start_count,"arm_count":n.arm_request_count,"cancel_count":n.cancel_count,"goal_uuid":uuid,"goal_uuid_count":len(uuids),"final_action_status":"CANCELED","final_mission_state":"CANCELLED","final_waypoint_index":final["waypoint_index"],"completed_waypoint_count":final["completed"],"post_cancel_translation_m":post,"post_cancel_yaw_rad":yaw,"events":{"readiness":ready,"route":route,"start_request":start_req,"start_response":start_resp,"navigating":nav,"arm_response":arm,"clear_motion":clear,"stop_confirmation":stop,"pre_cancel_stop":pre,"cancel_request":cancel_req,"cancel_response":cancel_resp},"stop_zeros":zeros,"adjudication":adj,"rates_hz":{t:rates(n.samples[t],clear_start,end) for t in TOPICS},"observation_end_ns":end,"cleanup_disarm_used":False}
    except Exception as exc:
        error=repr(exc); metrics={"pass":False,"status":"P4E3B_STOPPED_CANCEL_RUNTIME_CONTRACT_NEEDS_REVIEW","error":error}
    finally:
        with (out/"terminal_metrics.json").open("w",encoding="utf-8") as h: json.dump(metrics,h,indent=2,sort_keys=True); h.write("\n")
        n.close(); n.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)
