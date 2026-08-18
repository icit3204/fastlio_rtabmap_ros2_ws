"""B2P exact-context, passive Collision Monitor activation capture.

The observer is fully ready before the matrix is spawned.  It never calls a
lifecycle transition and never starts a route, mission, helper, Gate arm, or
physical command.  Matrix and witness are owned process groups with a fresh
token/UUID per probe.
"""
import argparse, json, os, signal, subprocess, time, uuid
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

MATRIX = ["ros2", "launch", "parking_robot_bringup", "phase4_p4e6b_health_matrix.launch.py", "enable_health_runner:=false"]
WITNESS = ["/usr/bin/python3", str(Path(__file__).resolve()), "--witness-child"]
TARGET_MARKERS = ("planner_server", "controller_server", "behavior_server", "bt_navigator",
                  "lifecycle_manager", "collision_monitor", "mission_manager",
                  "phase4_p4e6b_passive_readiness_witness")


def proc_rows(token):
    result=[]
    for p in Path("/proc").glob("[0-9]*"):
        try:
            env=(p/"environ").read_bytes().split(b"\0")
            if ("P4E6B_EPISODE_TOKEN="+token).encode() not in env: continue
            stat=(p/"stat").read_text().split(); cmd=(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace")
            rss=int((p/"statm").read_text().split()[1])*os.sysconf("SC_PAGE_SIZE")
            result.append({"pid":int(p.name),"ppid":int(stat[3]),"pgid":int(stat[4]),"state":stat[2],"starttime":stat[21],"rss_bytes":rss,"argv":cmd})
        except (FileNotFoundError,PermissionError,IndexError,ValueError): pass
    return sorted(result,key=lambda x:x["pid"])


def all_target_rows():
    rows=[]
    for p in Path("/proc").glob("[0-9]*"):
        try:
            cmd=(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace")
            if any(x in cmd for x in TARGET_MARKERS): rows.append({"pid":int(p.name),"argv":cmd})
        except (FileNotFoundError,PermissionError,ValueError): pass
    return rows


def kill_group(p):
    if p is None or p.poll() is not None: return
    try: os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
    except (ProcessLookupError,subprocess.TimeoutExpired):
        if p.poll() is None:
            try: os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError: pass
            p.wait(timeout=5)


def level(x): return x[0] if isinstance(x,(bytes,bytearray)) else int(x)


def stage_from_events(out, active, valid, bool_true):
    rows=[]
    try:
        rows=[json.loads(x) for x in (out/"observer_events.jsonl").read_text().splitlines()]
    except FileNotFoundError:
        pass
    polls=[x for x in rows if x.get("event")=="LIFECYCLE_POLL"]
    if bool_true: return "BOOL_TRUE", "ACTIVE_VALID"
    if valid: return "VALID", "ACTIVE_BUT_VALIDITY_NEVER_VALID"
    if active: return "ACTIVE", "ACTIVE_BUT_VALIDITY_NEVER_VALID"
    states=[x.get("get_state",{}).get("label") for x in polls if x.get("get_state")]
    if "activating" in states: return "ACTIVATING", "ACTIVATE_TRANSITION_FAILED"
    if "inactive" in states: return "INACTIVE", "CONFIGURED_BUT_NEVER_ACTIVATED"
    if "configuring" in states: return "CONFIGURING", "CONFIGURE_TRANSITION_FAILED"
    if "unconfigured" in states: return "UNCONFIGURED", "STUCK_UNCONFIGURED"
    if any(x.get("get_state_service_available") for x in polls): return "LIFECYCLE_SERVICE_AVAILABLE", "UNRESOLVED_WITH_CAPTURE"
    if any(x.get("collision_node_present") for x in polls): return "NODE_DISCOVERED", "LIFECYCLE_SERVICES_UNAVAILABLE"
    if any(x.get("processes") for x in polls): return "PROCESS_STARTED", "COLLISION_NODE_NOT_DISCOVERED"
    return "PROCESS_NOT_STARTED", "COLLISION_PROCESS_NOT_STARTED"


class CaptureNode:
    def __init__(self, out, token):
        self.node=rclpy.create_node("p4e6b_exact_context_observer")
        self.out=out; self.token=token; self.events=out/"observer_events.jsonl"; self.diags=[]; self.bools=[]; self.scans=[]; self.transitions=[]
        self.state_client=self.node.create_client(GetState,"/collision_monitor/get_state")
        self.node.create_subscription(DiagnosticArray,"/diagnostics",self.on_diag,50)
        self.node.create_subscription(Bool,"/system/collision_monitor_valid",self.on_bool,50)
        self.node.create_subscription(LaserScan,"/phase4/synthetic_scan",self.on_scan,50)
        self.node.create_subscription(TransitionEvent,"/collision_monitor/transition_event",self.on_transition,50)

    def emit(self,event,**fields):
        row={"monotonic_ns":time.monotonic_ns(),"event":event,**fields}
        with self.events.open("a") as f: f.write(json.dumps(row,sort_keys=True)+"\n"); f.flush(); os.fsync(f.fileno())
        return row

    def on_diag(self,msg):
        now=time.monotonic_ns()
        for s in msg.status:
            if s.name!="vehicle_cmd_safety/collision_monitor_validity_monitor": continue
            values={x.key:x.value for x in s.values}; row={"monotonic_ns":now,"level":level(s.level),"message":s.message,"values":values,"reason_code":values.get("reason_code"),"state":values.get("state")}; self.diags.append(row); self.emit("VALIDITY_DIAGNOSTIC",**row)

    def on_bool(self,msg):
        row={"monotonic_ns":time.monotonic_ns(),"value":bool(msg.data)}; self.bools.append(row); self.emit("VALIDITY_BOOL",**row)

    def on_scan(self,msg):
        row={"monotonic_ns":time.monotonic_ns(),"frame_id":msg.header.frame_id,"range_count":len(msg.ranges)}; self.scans.append(row); self.emit("SOURCE_SCAN",**row)

    def on_transition(self,msg):
        row={"monotonic_ns":time.monotonic_ns(),"start_id":int(msg.start_state.id),"start_label":msg.start_state.label,"goal_id":int(msg.goal_state.id),"goal_label":msg.goal_state.label}; self.transitions.append(row); self.emit("LIFECYCLE_TRANSITION",**row)

    def poll(self,phase):
        names=sorted(self.node.get_node_names_and_namespaces()); services=self.node.get_service_names_and_types(); state=None; ready=self.state_client.service_is_ready()
        if ready:
            fut=self.state_client.call_async(GetState.Request()); rclpy.spin_until_future_complete(self.node,fut,timeout_sec=.15)
            if fut.done() and fut.exception() is None: state={"id":int(fut.result().current_state.id),"label":fut.result().current_state.label}
        row={"monotonic_ns":time.monotonic_ns(),"event":"LIFECYCLE_POLL","phase":phase,"node_names":names,"collision_node_present":any(n=="collision_monitor" for n,_ in names),"manager_node_present":any(n=="lifecycle_manager_collision_monitor" for n,_ in names),"get_state_service_listed":any(n=="/collision_monitor/get_state" for n,_ in services),"get_state_service_available":ready,"get_state":state,"processes":proc_rows(self.token),"target_processes":all_target_rows()}; self.emit("LIFECYCLE_POLL",**{k:v for k,v in row.items() if k not in ("event",)}); return row

    def close(self):
        self.node.destroy_node()


def witness_child():
    # Child entrypoint is only used by the parent to provide the exact B2M witness.
    import runpy,sys
    witness=Path(__file__).with_name("phase4_p4e6b_passive_readiness_witness.py")
    sys.argv=[str(witness),"--output-dir",os.environ["B2P_WITNESS_OUTPUT"]]; runpy.run_path(str(witness),run_name="__main__")


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--output-dir"); ap.add_argument("--domain",type=int); ap.add_argument("--observe-sec",type=float,default=15.0); ap.add_argument("--witness-child",action="store_true"); a=ap.parse_args()
    if a.witness_child: return witness_child()
    out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True); token="b2p-"+uuid.uuid4().hex; episode_uuid=str(uuid.uuid4()); env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY="1",P4E6B_EPISODE_TOKEN=token,P4E6B_EPISODE_UUID=episode_uuid)
    (out/"identity.json").write_text(json.dumps({"domain":a.domain,"token":token,"episode_uuid":episode_uuid,"qualification_marker":"ACTIVATION_PROBE","new_b2m_samples":0,"formal_samples":0,"route_mission":0,"mission_start":0,"gate_arm":0,"physical_can":0},indent=2)+"\n")
    os.environ.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY="1",P4E6B_EPISODE_TOKEN=token,P4E6B_EPISODE_UUID=episode_uuid)
    rclpy.init(args=[]); node=CaptureNode(out,token); node.emit("OBSERVER_READY",domain=a.domain,token=token,episode_uuid=episode_uuid); (out/"OBSERVER_READY").write_text(json.dumps({"event":"OBSERVER_READY","monotonic_ns":time.monotonic_ns()})+"\n")
    matrix=None; witness=None; matrix_start=None; active_at=None; forensic=False; deadline=None
    try:
        matrix=subprocess.Popen(MATRIX,env=env,start_new_session=True,stdout=(out/"matrix.stdout").open("w"),stderr=(out/"matrix.stderr").open("w")); matrix_start=time.monotonic_ns(); node.emit("ACTIVATION_PROBE_MATRIX_STARTED",pid=matrix.pid,pgid=matrix.pid); (out/"ACTIVATION_PROBE_MATRIX_STARTED").write_text(json.dumps({"monotonic_ns":matrix_start,"pid":matrix.pid})+"\n")
        wenv=dict(env,B2P_WITNESS_OUTPUT=str(out/"witness")); wenv["PYTHONPATH"]=str(Path(__file__).parent)+":"+wenv.get("PYTHONPATH",""); witness=subprocess.Popen(WITNESS,env=wenv,start_new_session=True,stdout=(out/"witness.stdout").open("w"),stderr=(out/"witness.stderr").open("w")); node.emit("WITNESS_STARTED",pid=witness.pid,pgid=witness.pid)
        deadline=time.monotonic()+a.observe_sec
        while time.monotonic()<deadline:
            rclpy.spin_once(node.node,timeout_sec=.02); poll=node.poll("ACTIVE_GATE")
            if active_at is None and (poll.get("get_state") or {}).get("label")=="active": active_at=poll["monotonic_ns"]; node.emit("ACTIVE_OBSERVED",at_ns=active_at)
            if active_at is None and matrix_start and time.monotonic_ns()-matrix_start>=10_000_000_000: forensic=True; break
            time.sleep(.08)
        if forensic:
            node.emit("FORENSIC_HOLD",reason="ACTIVE_NOT_REACHED_BY_10S"); deadline=time.monotonic()+max(0.0,35.0-(time.monotonic_ns()-matrix_start)/1e9)
            while time.monotonic()<deadline:
                rclpy.spin_once(node.node,timeout_sec=.02); node.poll("FORENSIC_HOLD"); time.sleep(.08)
        else:
            deadline=time.monotonic()+max(0.0,3.0-(time.monotonic_ns()-matrix_start)/1e9)
            while time.monotonic()<deadline: rclpy.spin_once(node.node,timeout_sec=.02); node.poll("POST_VALID_HOLD"); time.sleep(.08)
    finally:
        node.poll("PRE_CLEANUP"); kill_group(witness); kill_group(matrix); node.poll("POST_CLEANUP")
        for _ in range(30):
            if not proc_rows(token): break
            time.sleep(.5)
        node.emit("CLEANUP_VERIFIED",token_processes_after=proc_rows(token),target_processes_after=all_target_rows())
        (out/"cleanup.json").write_text(json.dumps({"matrix_returncode":None if matrix is None else matrix.returncode,"witness_returncode":None if witness is None else witness.returncode,"token_processes_after":proc_rows(token),"target_processes_after":all_target_rows()},indent=2,sort_keys=True)+"\n")
        valid_observed=any(x.get("reason_code")=="VALID" for x in node.diags); bool_true_observed=any(x.get("value") is True for x in node.bools); stage, failure_class=stage_from_events(out,active_at is not None,valid_observed,bool_true_observed)
        success=(active_at is not None and valid_observed and bool_true_observed)
        state_statuses=sorted({x.get("values",{}).get("state_query_status") for x in node.diags if x.get("values",{}).get("state_query_status")})
        params_statuses=sorted({x.get("values",{}).get("params_query_status") for x in node.diags if x.get("values",{}).get("params_query_status")})
        summary={"qualification_marker":"ACTIVATION_PROBE_CLOSED","domain":a.domain,"token":token,"episode_uuid":episode_uuid,"matrix_start_ns":matrix_start,"active_observed":active_at is not None,"forensic_hold":forensic,"active_gate_sec":None if active_at is None else (active_at-matrix_start)/1e9,"valid_observed":valid_observed,"bool_true_observed":bool_true_observed,"furthest_proven_stage":stage,"failure_capture_class":None if success else (failure_class if active_at is None else "ACTIVE_BUT_VALIDITY_NEVER_VALID"),"query_state_statuses":state_statuses,"query_params_statuses":params_statuses,"query_timeout_observed":("TIMED_OUT" in state_statuses or "TIMED_OUT" in params_statuses),"query_valid_response_observed":("VALID_RESPONSE" in state_statuses and "VALID_RESPONSE" in params_statuses),"transition_count":len(node.transitions),"poll_count":sum(1 for x in node.events.read_text().splitlines() if '"event": "LIFECYCLE_POLL"' in x),"new_b2m_samples":0,"formal_samples":0,"route_mission":0,"mission_start":0,"gate_arm":0,"physical_can":0}
        (out/"summary.json").write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n"); (out/"ACTIVATION_PROBE_CLOSED").write_text(json.dumps(summary,sort_keys=True)+"\n"); node.close(); rclpy.shutdown(); subprocess.run(["ros2","daemon","stop"],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)


if __name__=="__main__": main()
