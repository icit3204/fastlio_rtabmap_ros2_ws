"""Fake-only P4-E.3A Mission Manager CLEAR user-cancel campaign."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import MissionState, RouteMission
import rclpy
from std_srvs.srv import SetBool, Trigger

from .phase4_p4e1b_clear_runner import Runner, TOPICS, LIFECYCLE, nz, quat, rates


STATE_NAMES = {value: name for name, value in vars(MissionState).items()
               if name.isupper() and isinstance(value, int)}


def adjudicate_cancel_stop(layers, cancel_ns, action_status, mission_state,
                           observation_end_ns, post_cancel_translation_m,
                           later_waypoint_dispatch=False, minimum_stationary_ns=2_000_000_000,
                           maximum_translation_m=0.02):
    """Pure adjudicator anchored to cancellation, never to a later nonzero."""
    report = {"cancel_request_ns": int(cancel_ns), "action_status": int(action_status),
              "mission_state": int(mission_state), "layers": {}, "reasons": []}
    terminal = []
    for name in ("raw", "safe", "gate", "mock", "fake"):
        samples = sorted((int(t), bool(v)) for t, v in layers.get(name, ()))
        pre = [t for t, v in samples if t <= cancel_ns and v]
        post_zeros = [t for t, v in samples if t >= cancel_ns and not v]
        zero = post_zeros[0] if post_zeros else None
        later = zero is not None and any(t > zero and v for t, v in samples)
        report["layers"][name] = {"last_pre_cancel_nonzero_ns": pre[-1] if pre else None,
                                  "cancellation_terminal_zero_ns": zero,
                                  "later_nonzero_violation": later}
        if not pre: report["reasons"].append(f"{name}: no active-mission nonzero before cancel")
        if zero is None: report["reasons"].append(f"{name}: missing cancellation terminal zero")
        if later: report["reasons"].append(f"{name}: nonzero after cancellation terminal zero")
        if zero is not None: terminal.append(zero)
    final_zero = max(terminal) if len(terminal) == 5 else None
    duration = None if final_zero is None else int(observation_end_ns) - final_zero
    report.update({"final_terminal_zero_ns": final_zero,
                   "observation_end_ns": int(observation_end_ns),
                   "stationary_duration_ns": duration,
                   "post_cancel_translation_m": float(post_cancel_translation_m),
                   "later_waypoint_dispatch": bool(later_waypoint_dispatch)})
    if action_status != GoalStatus.STATUS_CANCELED: report["reasons"].append("action terminal is not CANCELED")
    if mission_state != MissionState.CANCELLED: report["reasons"].append("mission terminal is not CANCELLED")
    if later_waypoint_dispatch: report["reasons"].append("later waypoint dispatched")
    if duration is None or duration < minimum_stationary_ns: report["reasons"].append("stationary observation below 2.0 seconds")
    if post_cancel_translation_m > maximum_translation_m: report["reasons"].append("post-cancel translation exceeds 0.02 m")
    report["pass"] = not report["reasons"]
    report["final_reason"] = "PASS" if report["pass"] else "; ".join(report["reasons"])
    return report


class MissionCancelRunner(Runner):
    def __init__(self, out: Path):
        super().__init__(out, create_action_client=False)
        self.states=[]; self.action_statuses=[]; self.goal_uuids=[]
        self.route_count=0; self.start_count=0; self.cancel_count=0; self.disarm_count=0
        self.route_pub=self.create_publisher(RouteMission,"/mission/route",10)
        self.start_client=self.create_client(Trigger,"/mission/start")
        self.cancel_client=self.create_client(Trigger,"/mission/cancel")
        self.create_subscription(MissionState,"/mission/state",self.mission_state,100)
        self.create_subscription(GoalStatusArray,"/navigate_to_pose/_action/status",self.action_status,100)
        for name in ("route_mission_events.jsonl","mission_service_events.jsonl",
                     "mission_state_events.jsonl","mission_status_events.jsonl",
                     "navigate_action_status_events.jsonl","navigate_goal_observations.jsonl"):
            (out/name).write_text("",encoding="utf-8")
        self._open("fake_applied_timeline.tsv","mono_ns\tros_ns\tlinear_x\tangular_z\n")
        # Required filename; retain accepted P4-E filename too.
        self._open("mock_output_timeline.tsv","mono_ns\tros_ns\tvalues\n")

    def emit(self,name,item):
        item=dict(item); item.setdefault("monotonic_ns",time.monotonic_ns()); item.setdefault("ros_ns",self.get_clock().now().nanoseconds)
        self.events.append(item)
        with (self.out/name).open("a",encoding="utf-8") as h:
            h.write(json.dumps(item,sort_keys=True)+"\n"); h.flush(); os.fsync(h.fileno())
        return item

    def mission_state(self,msg):
        mono,ros=self.now(); uuid=str(msg.active_goal_uuid)
        item={"monotonic_ns":mono,"ros_ns":ros,"message_ros_stamp_ns":msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
              "state":int(msg.state),"state_name":STATE_NAMES.get(int(msg.state),str(msg.state)),"mission_id":msg.mission_id,
              "route_id":msg.route_id,"waypoint_index":int(msg.current_waypoint_index),"completed":int(msg.completed_waypoint_count),
              "total":int(msg.total_waypoint_count),"active_goal_uuid":uuid,"reason_code":msg.reason_code,"detail":msg.detail}
        self.states.append(item); self.emit("mission_state_events.jsonl",item); self.emit("mission_status_events.jsonl",item)
        if uuid:
            self.goal_uuids.append((mono,uuid,int(msg.current_waypoint_index)))
            self.emit("navigate_goal_observations.jsonl",{"monotonic_ns":mono,"goal_uuid":uuid,"waypoint_index":int(msg.current_waypoint_index),"source":"MissionState"})

    def action_status(self,msg):
        mono,ros=self.now()
        for s in msg.status_list:
            uuid=bytes(s.goal_info.goal_id.uuid).hex(); item={"monotonic_ns":mono,"ros_ns":ros,"goal_uuid":uuid,"status":int(s.status)}
            self.action_statuses.append(item); self.emit("navigate_action_status_events.jsonl",item)

    def diagnostics(self,topic,msg):
        super().diagnostics(topic,msg)
        mono,ros=self.now()
        for s in msg.status:
            if "phase4_vehicle_cmd_fake_base" in s.name:
                fields={v.key:v.value for v in s.values}; x=float(fields.get("applied_linear_x",0.0)); z=float(fields.get("applied_angular_z",0.0))
                self.files["fake_applied_timeline.tsv"].write(f"{mono}\t{ros}\t{x}\t{z}\n"); self.files["fake_applied_timeline.tsv"].flush()

    def mock(self,msg):
        super().mock(msg); mono,ros=self.now(); self.files["mock_output_timeline.tsv"].write(f"{mono}\t{ros}\t{json.dumps(list(msg.data))}\n"); self.files["mock_output_timeline.tsv"].flush()

    def fixture(self):
        m=RouteMission(); m.header.frame_id="map"; m.header.stamp=self.get_clock().now().to_msg()
        m.mission_id="p4e3a-20260807-clear-user-cancel"; m.route_id="p4e3a-phase2-straight-two-waypoint"; m.topology_version="v1"
        m.node_ids=["p4e3a-wp0","p4e3a-wp1"]; m.edge_ids=["p4e3a-edge0"]; m.edge_directions=[1]
        for x in (8.425,9.425):
            p=PoseStamped(); p.header.frame_id="map"; p.header.stamp=m.header.stamp; p.pose.position.x=x; p.pose.position.y=-53.725; p.pose.orientation=quat(0.0); m.poses.append(p)
        return m

    def publish_route(self):
        if self.route_count: raise RuntimeError("RouteMission republish forbidden")
        m=self.fixture(); self.route_count=1; item=self.emit("route_mission_events.jsonl",{"event":"route_publish","mission_id":m.mission_id,"route_id":m.route_id,"topology_version":m.topology_version,"poses":[[p.pose.position.x,p.pose.position.y] for p in m.poses],"count":1})
        self.route_pub.publish(m); return item

    def trigger(self,kind):
        client=self.start_client if kind=="start" else self.cancel_client
        attr="start_count" if kind=="start" else "cancel_count"; count=getattr(self,attr)
        if count: raise RuntimeError(f"{kind} retry forbidden")
        setattr(self,attr,1); request=self.emit("mission_service_events.jsonl",{"event":f"{kind}_request","count":1})
        if not client.wait_for_service(timeout_sec=10.0): raise RuntimeError(f"{kind} service unavailable")
        fut=client.call_async(Trigger.Request()); rclpy.spin_until_future_complete(self,fut,timeout_sec=5.0)
        if not fut.done() or fut.result() is None: raise RuntimeError(f"{kind} response timeout")
        response=self.emit("mission_service_events.jsonl",{"event":f"{kind}_response","success":fut.result().success,"message":fut.result().message,"count":1})
        if not fut.result().success: raise RuntimeError(f"{kind} rejected: {fut.result().message}")
        return request,response

    def wait_for(self,predicate,timeout,reason):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=0.02)
            if predicate(): return
        raise RuntimeError(reason)


def main(args=None):
    ap=argparse.ArgumentParser(); ap.add_argument("--output-dir",required=True); ns=ap.parse_args(args)
    out=Path(ns.output_dir); out.mkdir(parents=True,exist_ok=True); error=None; metrics={}
    rclpy.init(); n=MissionCancelRunner(out)
    try:
        with (out/"process_environment.tsv").open("w",encoding="utf-8") as h:
            h.write("key\tvalue\n"); [h.write(f"{k}\t{v}\n") for k,v in sorted(os.environ.items()) if k in ("ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION")]
        n.spin(5.0); n.graph(); lifecycle=n.lifecycle()
        if any(lifecycle.get(x,(0,""))[1]!="active" for x in LIFECYCLE): raise RuntimeError(f"inactive lifecycle: {lifecycle}")
        nodes=set(n.get_node_names()); forbidden={x for x in nodes if any(y in x.lower() for y in ("plan_nav","rtab","fast_lio","pure_pursuit","wheelchair_controller"))}
        if forbidden: raise RuntimeError(f"forbidden nodes: {forbidden}")
        if sum(1 for x in n.get_node_names() if x=="mission_manager")!=1: raise RuntimeError(f"mission manager ownership: {nodes}")
        pubs={t:len(n.get_publishers_info_by_topic(t)) for t in TOPICS}
        if any(pubs[t]!=1 for t in TOPICS[1:]): raise RuntimeError(f"publisher authority: {pubs}")
        if n.scan_count<50 or any(not n.permissions.get(t,(False,0))[0] for t in TOPICS[2:5]): raise RuntimeError("validity not fresh true")
        gate=n.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        if gate is None or gate[1].get("state")!="DISARMED" or gate[1].get("fault_latched")!="false": raise RuntimeError(f"gate readiness: {gate}")
        readiness=n.emit("scenario_events.jsonl",{"event":"P4E3A_CANCEL_CHAIN_READINESS_PASS","lifecycle":lifecycle,"publisher_counts":pubs})
        n.publish_initial(); n.spin(0.5); route=n.publish_route()
        n.wait_for(lambda:any(s["state"]==MissionState.RECEIVED for s in n.states),5.0,"RECEIVED timeout")
        start_req,start_resp=n.trigger("start")
        n.wait_for(lambda:any(s["state"]==MissionState.NAVIGATING and s["active_goal_uuid"] for s in n.states),10.0,"NAVIGATING/UUID timeout")
        nav=next(s for s in n.states if s["state"]==MissionState.NAVIGATING and s["active_goal_uuid"]); uuid=nav["active_goal_uuid"]
        n.wait_fresh_safe_disarmed(); arm=n.arm_gate()
        motion_start=time.monotonic_ns()
        n.wait_for(lambda:any(x[0]>=motion_start and nz(x[2:8]) for x in n.samples["/vehicle_cmd_safe"]),5.0,"gate motion timeout")
        stable_start=time.monotonic_ns(); n.spin(2.1)
        if not n.states or n.states[-1]["state"]!=MissionState.NAVIGATING or n.states[-1]["waypoint_index"]!=0: raise RuntimeError("not stable waypoint-0 NAVIGATING")
        if any(x["goal_uuid"]!=uuid for x in n.action_statuses): raise RuntimeError("second goal UUID observed")
        pre=n.emit("scenario_events.jsonl",{"event":"P4E3A_PRE_CANCEL_CLEAR_MOTION_PASS","duration_ns":time.monotonic_ns()-stable_start,"goal_uuid":uuid})
        cancel_pose=n.latest_odom; cancel_req,cancel_resp=n.trigger("cancel"); cancel_ns=cancel_req["monotonic_ns"]
        n.wait_for(lambda:any(s["state"]==MissionState.CANCELLING for s in n.states if s["monotonic_ns"]>=cancel_ns),5.0,"CANCELLING timeout")
        n.wait_for(lambda:any(x["goal_uuid"]==uuid and x["status"]==GoalStatus.STATUS_CANCELED for x in n.action_statuses),8.0,"action CANCELED timeout")
        n.wait_for(lambda:any(s["state"]==MissionState.CANCELLED for s in n.states),8.0,"Mission CANCELLED timeout")
        n.spin(2.8); end=time.monotonic_ns(); n.graph(); final=n.states[-1]
        post=math.hypot(n.latest_odom[0]-cancel_pose[0],n.latest_odom[1]-cancel_pose[1]); yaw_drift=abs(n.latest_odom[2]-cancel_pose[2])
        fake=[(x["monotonic_ns"],nz((x["values"].get("applied_linear_x",0.0),x["values"].get("applied_angular_z",0.0)))) for x in n.diagnostic_history["fake_base"]]
        layers={"raw":[(x[0],nz(x[2:8])) for x in n.samples["/cmd_vel_nav_raw"]],"safe":[(x[0],nz(x[2:8])) for x in n.samples["/cmd_vel_nav_safe"]],"gate":[(x[0],nz(x[2:8])) for x in n.samples["/vehicle_cmd_safe"]],"mock":[(x[0],nz(x[2])) for x in n.samples["/wheelchair_control_command_mock"]],"fake":fake}
        later_goal=any(g[2]>0 or g[1]!=uuid for g in n.goal_uuids)
        adjudication=adjudicate_cancel_stop(layers,cancel_ns,GoalStatus.STATUS_CANCELED,final["state"],end,post,later_goal)
        if not adjudication["pass"]: raise RuntimeError(adjudication["final_reason"])
        for event in ("P4E3A_FULL_CHAIN_CANCEL_ZERO_PASS","P4E3A_MISSION_CANCEL_PROTOCOL_PASS","P4E3A_CLEAR_USER_CANCEL_POST_TERMINAL_PASS"):
            n.emit("scenario_events.jsonl",{"event":event})
        metrics={"pass":True,"status":"PHASE4_P4E3A_CLEAR_MISSION_CANCEL_FULL_CHAIN_COMPLETED_NEEDS_REVIEW","route_count":n.route_count,"start_count":n.start_count,"arm_count":n.arm_request_count,"cancel_count":n.cancel_count,"goal_uuid":uuid,"goal_uuid_count":len(set(g[1] for g in n.goal_uuids)),"final_action_status":"CANCELED","final_mission_state":"CANCELLED","final_waypoint_index":final["waypoint_index"],"completed_waypoint_count":final["completed"],"post_cancel_translation_m":post,"post_cancel_yaw_rad":yaw_drift,"observation_end_ns":end,"events":{"route":route,"start_request":start_req,"start_response":start_resp,"navigating":nav,"arm_response":arm,"pre_cancel":pre,"cancel_request":cancel_req,"cancel_response":cancel_resp},"rates_hz":{t:rates(n.samples[t],stable_start,end) for t in TOPICS},"adjudication":adjudication,"readiness":readiness}
    except Exception as exc:
        error=repr(exc); metrics={"pass":False,"status":"P4E3A_MISSION_CANCEL_PROTOCOL_NEEDS_REVIEW","error":error}
    finally:
        with (out/"terminal_metrics.json").open("w",encoding="utf-8") as h: json.dump(metrics,h,indent=2,sort_keys=True); h.write("\n")
        n.close(); n.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)
