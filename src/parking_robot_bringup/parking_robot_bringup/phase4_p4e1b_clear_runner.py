"""Test-only direct NavigateToPose client and passive evidence monitor for P4-E.1B."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


TOPICS = ("/cmd_vel_nav_raw", "/cmd_vel_nav_safe", "/system/collision_monitor_valid",
          "/system/localization_valid", "/system/controller_valid", "/vehicle_cmd_safe",
          "/wheelchair_control_command_mock", "/Odometry")
LIFECYCLE = ("map_server", "planner_server", "controller_server", "behavior_server",
             "bt_navigator", "collision_monitor")
FORBIDDEN = ("/cmd_vel", "/cmd_vel_phase2_mock", "/cmd_vel_nav",
             "/wheelchair_control_command", "/wheelchair_control_command_raw")


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def quat(value):
    q = Quaternion(); q.z = math.sin(value / 2.0); q.w = math.cos(value / 2.0); return q


def nz(values):
    return any(abs(float(v)) > 1.0e-6 for v in values)


def adjudicate_terminal_stop(layers, result_ns, succeeded, observation_end_ns,
                             post_result_translation_m, downstream_limit_ns=100_000_000,
                             minimum_stationary_ns=2_000_000_000,
                             maximum_translation_m=0.02):
    """Pure, timestamp-preserving terminal-stop adjudication for P4-E.1B.

    ``layers`` maps raw/safe/gate/mock/fake to chronological ``(mono_ns,
    nonzero)`` samples.  A terminal zero can precede the action-result callback;
    it is the first zero after the layer's final nonzero (or the first zero for
    a layer that never became nonzero).  Any subsequent nonzero invalidates it.
    """
    required = ("raw", "safe", "gate", "mock", "fake")
    report = {"result_ns": int(result_ns), "succeeded": bool(succeeded), "layers": {},
              "downstream_limit_ns": int(downstream_limit_ns), "reasons": []}
    terminal = {}
    for name in required:
        samples = sorted((int(stamp), bool(nonzero)) for stamp, nonzero in layers.get(name, ()))
        nonzero_stamps = [stamp for stamp, nonzero in samples if nonzero]
        last_nonzero = nonzero_stamps[-1] if nonzero_stamps else None
        zeros = [stamp for stamp, nonzero in samples if not nonzero and
                 (last_nonzero is None or stamp > last_nonzero)]
        terminal_zero = zeros[0] if zeros else None
        later_nonzero = (terminal_zero is not None and
                         any(nonzero and stamp > terminal_zero for stamp, nonzero in samples))
        classification = (None if terminal_zero is None else
                          "ZERO_BEFORE_RESULT" if terminal_zero < result_ns else
                          "ZERO_AFTER_RESULT" if terminal_zero > result_ns else
                          "ZERO_AT_RESULT")
        report["layers"][name] = {
            "last_nonzero_ns": last_nonzero,
            "terminal_zero_ns": terminal_zero,
            "later_nonzero_violation": later_nonzero,
            "result_ordering": classification,
        }
        terminal[name] = terminal_zero
        if terminal_zero is None:
            report["reasons"].append(f"{name}: missing terminal-zero sample")
        if later_nonzero:
            report["reasons"].append(f"{name}: nonzero after terminal zero")

    # Downstream components stop causally after the action result or their
    # immediate upstream terminal zero, whichever occurs later.
    for name, upstream in (("gate", "safe"), ("mock", "gate"), ("fake", "gate")):
        if terminal[name] is None or terminal[upstream] is None:
            latency = None; signed_delta = None
        else:
            reference = max(int(result_ns), terminal[upstream])
            signed_delta = terminal[name] - reference
            latency = max(0, signed_delta)  # Already stopped at the reference is zero latency.
        report["layers"][name]["stop_reference_ns"] = (
            None if terminal[upstream] is None else max(int(result_ns), terminal[upstream]))
        report["layers"][name]["downstream_stop_latency_ns"] = latency
        report["layers"][name]["signed_stop_delta_ns"] = signed_delta
        if latency is not None and latency > downstream_limit_ns:
            report["reasons"].append(f"{name}: downstream stop latency {latency} ns exceeds bound")

    final_zero = max((stamp for stamp in terminal.values() if stamp is not None), default=None)
    stationary_ns = None if final_zero is None else int(observation_end_ns) - final_zero
    report["final_terminal_zero_ns"] = final_zero
    report["observation_end_ns"] = int(observation_end_ns)
    report["stationary_duration_ns"] = stationary_ns
    report["stationary_window_pass"] = stationary_ns is not None and stationary_ns >= minimum_stationary_ns
    report["post_result_translation_m"] = float(post_result_translation_m)
    report["post_terminal_translation_pass"] = post_result_translation_m <= maximum_translation_m
    if not succeeded:
        report["reasons"].append("NavigateToPose result is not SUCCEEDED")
    if not report["stationary_window_pass"]:
        report["reasons"].append("stationary observation is shorter than 2.0 seconds")
    if not report["post_terminal_translation_pass"]:
        report["reasons"].append("post-result translation exceeds 0.02 m")
    report["pass"] = not report["reasons"]
    report["final_reason"] = "PASS" if report["pass"] else "; ".join(report["reasons"])
    return report


class Runner(Node):
    def __init__(self, out: Path, *, create_action_client: bool = True):
        super().__init__("phase4_p4e1b_clear_runner")
        self.out = out; out.mkdir(parents=True, exist_ok=True)
        self.files = {}; self.samples = {}; self.events = []; self.arm_events = []
        self.goal_status = None; self.goal_handle = None; self.result_future = None
        self.start_pose = (5.425, -53.725, 0.0); self.goal_pose = (8.425, -53.725, 0.0)
        self.initial_odom = None; self.success_pose = None; self.latest_odom = None
        self.permissions = {}; self.diag_conditions = {}; self.scan_count = 0
        self.diagnostic_counts = {"gate": 0, "adapter": 0, "fake_base": 0}
        self.diagnostic_history = {"gate": [], "adapter": [], "fake_base": []}
        self.tf_samples = {"map->odom": [], "odom->base_footprint": []}
        self.goal_request_count = 0; self.arm_request_count = 0
        self._open("cmd_vel_nav_raw_timeline.tsv", "mono_ns\tros_ns\tlinear_x\tlinear_y\tlinear_z\tangular_x\tangular_y\tangular_z\tframe\tpublisher_gid\tattributed_node\n")
        self._open("cmd_vel_nav_safe_timeline.tsv", "mono_ns\tros_ns\tlinear_x\tlinear_y\tlinear_z\tangular_x\tangular_y\tangular_z\tframe\tpublisher_gid\tattributed_node\n")
        self._open("vehicle_cmd_safe_timeline.tsv", "mono_ns\tros_ns\tlinear_x\tlinear_y\tlinear_z\tangular_x\tangular_y\tangular_z\tframe\tpublisher_gid\tattributed_node\n")
        self._open("mock_wheelchair_output_timeline.tsv", "mono_ns\tros_ns\tvalues\tpublisher_gid\tattributed_node\n")
        self._open("odometry_timeline.tsv", "mono_ns\tros_ns\tx\ty\tyaw\tlinear_x\tangular_z\tpublisher_gid\tattributed_node\n")
        self._open("tf_timeline.tsv", "mono_ns\tros_ns\tparent\tchild\tx\ty\tyaw\tpublisher_gid\tattributed_node\n")
        self._open("permission_timeline.tsv", "mono_ns\tros_ns\ttopic\tvalue\tpublisher_gid\tattributed_node\n")
        self._open("collision_validity_timeline.tsv", "mono_ns\tros_ns\tvalue\tpublisher_gid\tattributed_node\n")
        self._open("synthetic_source_timeline.tsv", "mono_ns\tros_ns\tframe\tminimum_range\tpublisher_gid\tattributed_node\n")
        self._open("topic_endpoint_timeline.tsv", "mono_ns\ttopic\tdirection\tcount\tgid\tnode\ttype\n")
        self._open("node_graph_timeline.tsv", "mono_ns\tnode\n")
        self._open("tf_owner_timeline.tsv", "mono_ns\ttransform\tgid\tnode\n")
        self._open("lifecycle_timeline.tsv", "mono_ns\tnode\tstate_id\tstate_label\n")
        self._open("process_lifetime.tsv", "mono_ns\tpid\tcommand\n")
        self._open("gate_state_timeline.tsv", "mono_ns\tstate\tcondition\n")
        for name in ("gate_diagnostics.jsonl", "adapter_diagnostics.jsonl", "fake_base_diagnostics.jsonl"):
            self._open(name, "")
        for name in ("arm_service_events.jsonl", "navigate_to_pose_events.jsonl", "scenario_events.jsonl"):
            (self.out / name).write_text("", encoding="utf-8")
        for topic in TOPICS: self.samples[topic] = []
        self.create_subscription(Twist, "/cmd_vel_nav_raw", lambda m:self.cmd("/cmd_vel_nav_raw",m), 100)
        self.create_subscription(Twist, "/cmd_vel_nav_safe", lambda m:self.cmd("/cmd_vel_nav_safe",m), 100)
        self.create_subscription(TwistStamped, "/vehicle_cmd_safe", self.vehicle, 100)
        self.create_subscription(Float32MultiArray, "/wheelchair_control_command_mock", self.mock, 100)
        self.create_subscription(Odometry, "/Odometry", self.odom, 100)
        self.create_subscription(TFMessage, "/tf", lambda m:self.tf(m,False), 100)
        self.create_subscription(TFMessage, "/tf_static", lambda m:self.tf(m,True), 100)
        for topic in ("/system/localization_valid", "/system/controller_valid", "/system/collision_monitor_valid"):
            self.create_subscription(Bool, topic, lambda m,t=topic:self.permission(t,m), 50)
        self.create_subscription(LaserScan, "/phase4/synthetic_scan", self.scan, 50)
        self.create_subscription(DiagnosticArray, "/diagnostics", lambda m:self.diagnostics("/diagnostics",m), 100)
        self.create_subscription(DiagnosticArray, "/wheelchair_cmd_adapter/diagnostics", lambda m:self.diagnostics("/wheelchair_cmd_adapter/diagnostics",m), 100)
        self.create_subscription(DiagnosticArray, "/phase4_fake_base/diagnostics", lambda m:self.diagnostics("/phase4_fake_base/diagnostics",m), 100)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.arm = self.create_client(SetBool, "/vehicle_cmd_safety/arm")
        # P4-E.3A reuses this passive evidence plumbing without creating a
        # second NavigateToPose client.  The default preserves P4-E.1B.
        self.action = (ActionClient(self, NavigateToPose, "/navigate_to_pose")
                       if create_action_client else None)

    def _open(self,name,header):
        h=(self.out/name).open("w",encoding="utf-8"); h.write(header); h.flush(); self.files[name]=h
    def ident(self, topic):
        """Attribute using the contemporaneous graph, as accepted by P4-C/P4-D."""
        endpoints = self.get_publishers_info_by_topic(topic)
        identities = sorted((bytes(ep.endpoint_gid).hex(),
                             f"{ep.node_namespace.rstrip('/')}/{ep.node_name}".replace("//", "/"))
                            for ep in endpoints)
        if not identities:
            return "", ""
        return ",".join(x[0] for x in identities), ",".join(x[1] for x in identities)
    def now(self): return time.monotonic_ns(),self.get_clock().now().nanoseconds
    def cmd(self,topic,msg):
        mono,ros=self.now(); gid,node=self.ident(topic); v=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        self.files[topic.strip("/")+"_timeline.tsv"].write("\t".join(map(str,(mono,ros,*v,"",gid,node)))+"\n")
        self.samples[topic].append((mono,ros,*v,gid,node))
    def vehicle(self,msg):
        mono,ros=self.now(); gid,node=self.ident("/vehicle_cmd_safe"); t=msg.twist; v=(t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)
        self.files["vehicle_cmd_safe_timeline.tsv"].write("\t".join(map(str,(mono,ros,*v,msg.header.frame_id,gid,node)))+"\n"); self.samples["/vehicle_cmd_safe"].append((mono,ros,*v,msg.header.frame_id,gid,node))
    def mock(self,msg):
        mono,ros=self.now(); gid,node=self.ident("/wheelchair_control_command_mock"); values=list(msg.data)
        self.files["mock_wheelchair_output_timeline.tsv"].write(f"{mono}\t{ros}\t{json.dumps(values)}\t{gid}\t{node}\n"); self.samples["/wheelchair_control_command_mock"].append((mono,ros,values,gid,node))
    def odom(self,msg):
        mono,ros=self.now(); gid,node=self.ident("/Odometry"); p=msg.pose.pose; values=(p.position.x,p.position.y,yaw(p.orientation),msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        self.files["odometry_timeline.tsv"].write("\t".join(map(str,(mono,ros,*values,gid,node)))+"\n"); self.samples["/Odometry"].append((mono,ros,*values,gid,node)); self.latest_odom=values
        if self.initial_odom is None: self.initial_odom=values
    def tf(self,msg,static):
        topic="/tf_static" if static else "/tf"; mono,ros=self.now(); gid,node=self.ident(topic)
        for t in msg.transforms:
            q=t.transform.rotation; tr=t.transform.translation; values=(tr.x,tr.y,yaw(q))
            self.files["tf_timeline.tsv"].write("\t".join(map(str,(mono,ros,t.header.frame_id,t.child_frame_id,*values,gid,node)))+"\n")
            pair=f"{t.header.frame_id}->{t.child_frame_id}"; self.files["tf_owner_timeline.tsv"].write(f"{mono}\t{pair}\t{gid}\t{node}\n")
            if pair in self.tf_samples: self.tf_samples[pair].append((mono,ros,*values,gid,node))
    def permission(self,topic,msg):
        mono,ros=self.now(); gid,node=self.ident(topic); row=f"{mono}\t{ros}\t{topic}\t{msg.data}\t{gid}\t{node}\n"; self.files["permission_timeline.tsv"].write(row)
        if topic=="/system/collision_monitor_valid": self.files["collision_validity_timeline.tsv"].write(f"{mono}\t{ros}\t{msg.data}\t{gid}\t{node}\n")
        self.permissions[topic]=(bool(msg.data),mono); self.samples[topic].append((mono,ros,bool(msg.data),gid,node))
    def scan(self,msg):
        mono,ros=self.now(); gid,node=self.ident("/phase4/synthetic_scan"); finite=[x for x in msg.ranges if math.isfinite(x)]; minimum=min(finite) if finite else float("inf")
        self.files["synthetic_source_timeline.tsv"].write(f"{mono}\t{ros}\t{msg.header.frame_id}\t{minimum}\t{gid}\t{node}\n"); self.scan_count+=1
    def diagnostics(self,topic,msg):
        mono,ros=self.now(); gid,node=self.ident(topic)
        for s in msg.status:
            fields={v.key:v.value for v in s.values}; raw_level=s.level; level=raw_level[0] if isinstance(raw_level,(bytes,bytearray,memoryview)) else int(raw_level); item={"monotonic_ns":mono,"ros_ns":ros,"name":s.name,"level":level,"message":s.message,"values":fields,"publisher_gid":gid,"attributed_node":node,"attribution_basis":"contemporaneous_graph_endpoint"}
            if "guarded_vehicle_cmd_gate" in s.name:
                self.diagnostic_counts["gate"] += 1
                self.diagnostic_history["gate"].append(item)
                self.files["gate_diagnostics.jsonl"].write(json.dumps(item,sort_keys=True)+"\n"); self.files["gate_state_timeline.tsv"].write(f"{mono}\t{fields.get('state','')}\t{s.message}\n")
            elif "mock_wheelchair_cmd_adapter" in s.name: self.diagnostic_counts["adapter"] += 1; self.diagnostic_history["adapter"].append(item); self.files["adapter_diagnostics.jsonl"].write(json.dumps(item,sort_keys=True)+"\n")
            elif "phase4_vehicle_cmd_fake_base" in s.name: self.diagnostic_counts["fake_base"] += 1; self.diagnostic_history["fake_base"].append(item); self.files["fake_base_diagnostics.jsonl"].write(json.dumps(item,sort_keys=True)+"\n")
            self.diag_conditions[s.name]=(s.message,fields,mono)
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: rclpy.spin_once(self,timeout_sec=0.02)
    def graph(self):
        mono=time.monotonic_ns(); types=dict(self.get_topic_names_and_types())
        for name,ns in self.get_node_names_and_namespaces(): self.files["node_graph_timeline.tsv"].write(f"{mono}\t{ns.rstrip('/')}/{name}\n".replace("//","/"))
        for topic in (*TOPICS,*FORBIDDEN,"/tf","/tf_static"):
            for direction,getter in (("publisher",self.get_publishers_info_by_topic),("subscriber",self.get_subscriptions_info_by_topic)):
                eps=getter(topic)
                if not eps: self.files["topic_endpoint_timeline.tsv"].write(f"{mono}\t{topic}\t{direction}\t0\t\t\t{','.join(types.get(topic,()))}\n")
                for ep in eps: self.files["topic_endpoint_timeline.tsv"].write(f"{mono}\t{topic}\t{direction}\t{len(eps)}\t{bytes(ep.endpoint_gid).hex()}\t{ep.node_namespace.rstrip('/')}/{ep.node_name}\t{','.join(types.get(topic,()))}\n")
        ps=subprocess.run(["ps","-eo","pid=,args="],text=True,capture_output=True,check=False)
        for line in ps.stdout.splitlines():
            if any(x in line.lower() for x in ("nav2_","collision_monitor","guarded_vehicle","fake_base","wheelchair_cmd","phase4_p4")):
                bits=line.strip().split(maxsplit=1); self.files["process_lifetime.tsv"].write(f"{mono}\t{bits[0]}\t{bits[1] if len(bits)>1 else ''}\n")
    def lifecycle(self):
        states={}
        for name in LIFECYCLE:
            client=self.create_client(GetState,f"/{name}/get_state")
            if client.wait_for_service(timeout_sec=1.0):
                fut=client.call_async(GetState.Request()); rclpy.spin_until_future_complete(self,fut,timeout_sec=2.0)
                if fut.done() and fut.result(): states[name]=(fut.result().current_state.id,fut.result().current_state.label)
            client.destroy()
        mono=time.monotonic_ns()
        for name,value in states.items(): self.files["lifecycle_timeline.tsv"].write(f"{mono}\t{name}\t{value[0]}\t{value[1]}\n")
        return states
    def publish_initial(self):
        m=PoseWithCovarianceStamped(); m.header.frame_id="odom"; m.pose.pose.position.x=self.start_pose[0]; m.pose.pose.position.y=self.start_pose[1]; m.pose.pose.orientation=quat(0.0)
        for _ in range(10): m.header.stamp=self.get_clock().now().to_msg(); self.initial_pub.publish(m); self.spin(0.1)
    def persist_event(self,event,arm=False):
        event=dict(event); event.setdefault("ros_ns",self.get_clock().now().nanoseconds)
        targets=["scenario_events.jsonl", "arm_service_events.jsonl" if arm else "navigate_to_pose_events.jsonl"]
        for name in targets:
            with (self.out/name).open("a",encoding="utf-8") as h:
                h.write(json.dumps(event,sort_keys=True)+"\n"); h.flush(); os.fsync(h.fileno())
    def arm_gate(self):
        if not self.arm.wait_for_service(timeout_sec=10.0): raise RuntimeError("arm service unavailable")
        if self.arm_request_count != 0: raise RuntimeError("arm retry forbidden")
        request=SetBool.Request(); request.data=True; sent=time.monotonic_ns(); self.arm_request_count += 1
        event={"event":"arm_request","monotonic_ns":sent,"data":True,"request_count":self.arm_request_count}; self.arm_events.append(event); self.persist_event(event,arm=True)
        future=self.arm.call_async(request); rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)
        if not future.done() or future.result() is None: raise RuntimeError("arm call timeout")
        result=future.result(); response={"event":"arm_response","monotonic_ns":time.monotonic_ns(),"success":result.success,"message":result.message,"request_count":self.arm_request_count}; self.arm_events.append(response); self.persist_event(response,arm=True)
        if not result.success: raise RuntimeError(f"arm rejected: {result.message}")
        return response
    def start_goal(self):
        if not self.action.wait_for_server(timeout_sec=30.0): raise RuntimeError("NavigateToPose unavailable")
        goal=NavigateToPose.Goal(); goal.pose=PoseStamped(); goal.pose.header.frame_id="map"; goal.pose.header.stamp=self.get_clock().now().to_msg(); goal.pose.pose.position.x=self.goal_pose[0]; goal.pose.pose.position.y=self.goal_pose[1]; goal.pose.pose.orientation=quat(0.0)
        if self.goal_request_count != 0: raise RuntimeError("goal resend forbidden")
        sent=time.monotonic_ns(); self.goal_request_count += 1; request={"event":"goal_request","monotonic_ns":sent,"goal":self.goal_pose,"request_count":self.goal_request_count}; self.events.append(request); self.persist_event(request)
        fut=self.action.send_goal_async(goal); rclpy.spin_until_future_complete(self,fut,timeout_sec=10.0)
        if not fut.done() or fut.result() is None: raise RuntimeError("goal response timeout")
        self.goal_handle=fut.result(); accepted=time.monotonic_ns(); uuid=bytes(self.goal_handle.goal_id.uuid).hex(); response={"event":"goal_response","monotonic_ns":accepted,"accepted":self.goal_handle.accepted,"goal_uuid":uuid,"request_count":self.goal_request_count}; self.events.append(response); self.persist_event(response)
        if not self.goal_handle.accepted: raise RuntimeError("goal rejected")
        self.result_future=self.goal_handle.get_result_async()
        return response
    def wait_fresh_safe_disarmed(self, timeout_sec=10.0):
        accepted_ns=self.events[-1]["monotonic_ns"]; deadline=time.monotonic()+timeout_sec
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.02)
            raw=[x for x in self.samples["/cmd_vel_nav_raw"] if x[0]>=accepted_ns and nz(x[2:8])]
            safe=[x for x in self.samples["/cmd_vel_nav_safe"] if x[0]>=accepted_ns and nz(x[2:8])]
            if len(raw)<3 or len(safe)<3 or time.monotonic_ns()-safe[-1][0]>250_000_000: continue
            if time.monotonic_ns()-safe[0][0] < 500_000_000: continue
            latest_gate=self.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
            if latest_gate is None: continue
            _,fields,_=latest_gate
            if fields.get("state")!="DISARMED" or fields.get("fault_latched")!="false": raise RuntimeError(f"gate not healthy disarmed: {fields}")
            if any(nz(x[2:8]) for x in self.samples["/vehicle_cmd_safe"] if x[0]>=accepted_ns): raise RuntimeError("gate bypass while disarmed")
            if any(nz(x[2]) for x in self.samples["/wheelchair_control_command_mock"] if x[0]>=accepted_ns): raise RuntimeError("adapter bypass while disarmed")
            if self.latest_odom is None or self.initial_odom is None or math.hypot(self.latest_odom[0]-self.initial_odom[0],self.latest_odom[1]-self.initial_odom[1])>0.002: raise RuntimeError("fake base moved while disarmed")
            if len(self.get_publishers_info_by_topic("/cmd_vel_nav_safe"))!=1: raise RuntimeError("safe publisher authority changed")
            if any(not self.permissions.get(t,(False,0))[0] or time.monotonic_ns()-self.permissions[t][1]>500_000_000 for t in TOPICS[2:5]): raise RuntimeError("permission freshness lost")
            skew=abs(safe[-1][0]-raw[-1][0]); numeric=max(abs(a-b) for a,b in zip(safe[-1][2:8],raw[-1][2:8]))
            if skew>75_000_000 or numeric>0.05: raise RuntimeError(f"raw/safe pairing failed: {skew},{numeric}")
            proof={"event":"P4E1B_GOAL_ACTIVE_DISARMED_INTERLOCK_PASS","monotonic_ns":time.monotonic_ns(),"safe_sample_count":len(safe),"raw_sample_count":len(raw),"newest_safe_age_ns":time.monotonic_ns()-safe[-1][0],"raw_safe_skew_ns":skew,"raw_safe_max_error":numeric,"goal_active_disarmed_duration_ns":time.monotonic_ns()-accepted_ns}; self.events.append(proof); self.persist_event(proof); return proof
        raise RuntimeError("fresh safe-command precondition timeout")
    def wait_goal_done(self):
        deadline=time.monotonic()+180.0
        while time.monotonic()<deadline and not self.result_future.done(): rclpy.spin_once(self,timeout_sec=0.02)
        if not self.result_future.done(): raise RuntimeError("goal result timeout")
        result=self.result_future.result(); self.goal_status=int(result.status); event={"event":"goal_result","monotonic_ns":time.monotonic_ns(),"status":self.goal_status,"status_name":"SUCCEEDED" if self.goal_status==GoalStatus.STATUS_SUCCEEDED else str(self.goal_status)}; self.events.append(event); self.persist_event(event)
        if self.goal_status != GoalStatus.STATUS_SUCCEEDED: raise RuntimeError(f"goal terminal status {self.goal_status}")
        self.success_pose=self.latest_odom
    def close(self):
        for h in self.files.values(): h.flush(); h.close()


def rates(rows,start,end):
    stamps=[r[0] for r in rows if start<=r[0]<=end]
    return 0.0 if len(stamps)<2 else (len(stamps)-1)/((stamps[-1]-stamps[0])/1e9)


def main(args=None):
    parser=argparse.ArgumentParser(); parser.add_argument("--output-dir",required=True); parser.add_argument("--readiness-only",action="store_true"); ns=parser.parse_args(args)
    out=Path(ns.output_dir); error=None; metrics={}; rclpy.init(); node=Runner(out)
    try:
        env={k:v for k,v in os.environ.items() if k in ("ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION")}
        with (out/"process_environment.tsv").open("w",encoding="utf-8") as h:
            h.write("key\tvalue\n"); [h.write(f"{k}\t{v}\n") for k,v in sorted(env.items())]
        node.spin(5.0); node.graph(); states=node.lifecycle(); ready_ns=time.monotonic_ns()
        pubs={t:len(node.get_publishers_info_by_topic(t)) for t in TOPICS}
        raw_nodes={ep.node_name for ep in node.get_publishers_info_by_topic("/cmd_vel_nav_raw")}
        required_single=TOPICS[1:]
        if any(states.get(n,(0,""))[1] != "active" for n in LIFECYCLE): raise RuntimeError(f"inactive lifecycle: {states}")
        if any(pubs[t] != 1 for t in required_single): raise RuntimeError(f"publisher authority: {pubs}")
        if not raw_nodes or not raw_nodes <= {"controller_server","behavior_server"}: raise RuntimeError(f"raw authority: {raw_nodes}")
        if node.scan_count < 50 or any(not node.permissions.get(t,(False,0))[0] for t in TOPICS[2:5]): raise RuntimeError("sources or permissions not ready")
        window_start=time.monotonic_ns(); start_pose=node.latest_odom; node.spin(4.0); window_end=time.monotonic_ns()
        gate_rate=rates(node.samples["/vehicle_cmd_safe"],window_start,window_end); mock_rate=rates(node.samples["/wheelchair_control_command_mock"],window_start,window_end); odom_rate=rates(node.samples["/Odometry"],window_start,window_end); tf_rate=rates(node.tf_samples["odom->base_footprint"],window_start,window_end)
        if not all(node.diagnostic_counts[k] > 0 for k in node.diagnostic_counts): raise RuntimeError(f"missing diagnostic callback: {node.diagnostic_counts}")
        if not (18.0 <= gate_rate <= 22.0 and 18.0 <= mock_rate <= 22.0 and 48.0 <= odom_rate <= 52.0 and 48.0 <= tf_rate <= 52.0): raise RuntimeError(f"readiness rates: {gate_rate},{mock_rate},{odom_rate},{tf_rate}")
        if any(nz(r[2:8]) for r in node.samples["/vehicle_cmd_safe"] if window_start <= r[0] <= window_end): raise RuntimeError("nonzero disarmed gate output")
        if any(len(r[2]) != 3 or nz(r[2]) for r in node.samples["/wheelchair_control_command_mock"] if window_start <= r[0] <= window_end): raise RuntimeError("invalid disarmed adapter output")
        if start_pose is None or node.latest_odom is None or math.hypot(node.latest_odom[0]-start_pose[0],node.latest_odom[1]-start_pose[1]) > 0.002: raise RuntimeError("fake-base drift during readiness")
        fake_diag=node.diag_conditions.get("phase4_vehicle_cmd_fake_base"); adapter_diag=node.diag_conditions.get("mock_wheelchair_cmd_adapter")
        if fake_diag is None or fake_diag[0] != "VALID": raise RuntimeError(f"fake-base diagnostic not VALID: {fake_diag}")
        if adapter_diag is None or adapter_diag[0] != "VALID": raise RuntimeError(f"adapter diagnostic not VALID: {adapter_diag}")
        readiness={"event":"P4E1B_CLEAR_CHAIN_READINESS_PASS","monotonic_ns":time.monotonic_ns(),"diagnostic_counts":node.diagnostic_counts,"gate_rate_hz":gate_rate,"mock_rate_hz":mock_rate,"odom_rate_hz":odom_rate,"tf_rate_hz":tf_rate}; node.events.append(readiness); node.persist_event(readiness)
        if ns.readiness_only:
            node.spin(1.0)
            metrics={"pass":True,"status":"P4E1B_CLEAR_CHAIN_READINESS_PASS","runner_alive_after_metrics":True,"publisher_counts":pubs,"raw_nodes":sorted(raw_nodes),"lifecycle":states,"readiness":readiness}
            return_code=0
            raise SystemExit(0)
        node.publish_initial(); node.spin(1.0); node.start_goal(); interlock=node.wait_fresh_safe_disarmed(); safe_fresh_ns=time.monotonic_ns(); arm_response=node.arm_gate(); node.spin(0.2)
        gate_diag=node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
        if gate_diag is None or gate_diag[1].get("state")!="ARMED" or gate_diag[1].get("fault_latched")!="false": raise RuntimeError(f"gate did not remain healthy armed: {gate_diag}")
        armed={"event":"P4E1B_FRESH_SAFE_THEN_ARM_PASS","monotonic_ns":time.monotonic_ns(),"safe_freshness_completion_ns":safe_fresh_ns,"arm_response_ns":arm_response["monotonic_ns"]}; node.events.append(armed); node.persist_event(armed)
        node.wait_goal_done(); success_ns=node.events[-1]["monotonic_ns"]; node.spin(2.2); end_ns=time.monotonic_ns(); node.graph(); node.lifecycle()
        if node.success_pose is None or node.latest_odom is None: raise RuntimeError("missing terminal odometry")
        post=math.hypot(node.latest_odom[0]-node.success_pose[0],node.latest_odom[1]-node.success_pose[1])
        if post>0.02: raise RuntimeError(f"post-success displacement {post}")
        if node.goal_request_count!=1 or node.arm_request_count!=1: raise RuntimeError("goal/arm cardinality violation")
        terminal_layers={
            "raw": [(x[0],nz(x[2:8])) for x in node.samples["/cmd_vel_nav_raw"]],
            "safe": [(x[0],nz(x[2:8])) for x in node.samples["/cmd_vel_nav_safe"]],
            "gate": [(x[0],nz(x[2:8])) for x in node.samples["/vehicle_cmd_safe"]],
            "mock": [(x[0],nz(x[2])) for x in node.samples["/wheelchair_control_command_mock"]],
            "fake": [(x["monotonic_ns"],nz((x["values"].get("applied_linear_x",0.0),
                                              x["values"].get("applied_angular_z",0.0))))
                     for x in node.diagnostic_history["fake_base"]],
        }
        terminal_stop=adjudicate_terminal_stop(terminal_layers,success_ns,True,end_ns,post)
        if not terminal_stop["pass"]: raise RuntimeError(f"terminal-stop adjudication: {terminal_stop['final_reason']}")
        for status in ("P4E1B_CLEAR_NAVIGATE_TO_POSE_PASS","P4E1B_CLEAR_POST_SUCCESS_STOP_PASS"):
            event={"event":status,"monotonic_ns":time.monotonic_ns()}; node.events.append(event); node.persist_event(event)
        metrics={"pass":True,"statuses":["P4E1B_CLEAR_CHAIN_READINESS_PASS","P4E1B_GOAL_ACTIVE_DISARMED_INTERLOCK_PASS","P4E1B_FRESH_SAFE_THEN_ARM_PASS","P4E1B_CLEAR_NAVIGATE_TO_POSE_PASS","P4E1B_CLEAR_POST_SUCCESS_STOP_PASS"],"goal_status":"SUCCEEDED","goal_request_count":node.goal_request_count,"arm_request_count":node.arm_request_count,"publisher_counts":pubs,"raw_nodes":sorted(raw_nodes),"lifecycle":states,"post_success_translation_m":post,"readiness_ns":ready_ns,"end_ns":end_ns,"rates_hz":{t:rates(node.samples[t],ready_ns,end_ns) for t in TOPICS},"interlock":interlock,"terminal_stop":terminal_stop}
    except Exception as exc:
        error=repr(exc); metrics={"pass":False,"error":error}
    finally:
        with (out/"terminal_metrics.json").open("w",encoding="utf-8") as h: json.dump(metrics,h,indent=2,sort_keys=True); h.write("\n")
        node.close(); node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)
