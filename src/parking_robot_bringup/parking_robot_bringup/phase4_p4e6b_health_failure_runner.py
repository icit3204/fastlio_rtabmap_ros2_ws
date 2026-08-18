"""Safely gated, exactly-one-case P4-E.6B qualification campaign runner."""
from __future__ import annotations
import argparse, json, os, queue, sys, threading, time
from pathlib import Path

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from parking_robot_interfaces.msg import MissionState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

from .phase4_p4e5b_temporary_block_runner import P4E5BRunner, health_ok, state_after
from .phase4_p4e1b_clear_runner import TOPICS
from .phase4_p4e6b_health_core import (
    ACTIVE_CASES, PRIMING_CASES, CampaignState, HealthCampaignModel, SCENARIOS,
    case_plan, adjudicate_health_terminal_history)


_PREINJECTION_TERMINALS=frozenset(("CANCELLING", "FAILED", "BLOCKED", "CANCELLED", "PAUSED"))


def nz(values):
    """Return whether a scalar or numeric command sequence is meaningfully nonzero."""
    if isinstance(values, (int, float)):
        return abs(float(values)) > 1.0e-6
    return any(abs(float(value)) > 1.0e-6 for value in values)


def competing_terminal_for_current_goal(states, *, accepted_ns, mission_id, route_id,
                                        waypoint, goal_uuid):
    """Return only a post-start terminal that belongs to this mission/goal.

    This is runner qualification logic: it observes Mission Manager authority;
    it neither chooses a reason nor sends a cancel request.
    """
    for row in reversed(states):
        if int(row.get("monotonic_ns", 0)) < accepted_ns:
            continue
        if row.get("mission_id") != mission_id or row.get("route_id") != route_id:
            continue
        if int(row.get("waypoint_index", -1)) != waypoint:
            continue
        if row.get("state_name") not in _PREINJECTION_TERMINALS:
            continue
        reason=row.get("reason_code", "")
        if not reason or reason == "HEALTH_CANCEL_ACK_ACCEPTED":
            continue
        active=row.get("active_goal_uuid", "")
        # CANCELLING must still own the active UUID.  FAILED legitimately
        # clears it, so mission/route/waypoint identity is then authoritative.
        if row.get("state_name") == "CANCELLING" and active != goal_uuid:
            continue
        return {"reason":reason, "state":row["state_name"],
                "monotonic_ns":int(row["monotonic_ns"])}
    return None

CAPACITY=4096
PREMISSION_COLLISION_STABLE_SEC=1.0
PREMISSION_CURRENTNESS_MAX_AGE_NS=250_000_000
PREMISSION_SOURCE_FRESHNESS_NS=500_000_000
# Startup includes lifecycle activation, 0.50 s monitor recovery, the required
# 1.0 s qualification epoch, and bounded local DDS scheduling margin.
PREMISSION_HEALTH_TIMEOUT_SEC=8.0
_EXPECTED_COLLISION_VALIDITY_NODE="collision_monitor_validity_monitor"
# Derived from the fake-base 50 Hz timer, its 0.50 s receipt deadman, and a
# bounded scheduling margin.  Direct command zero is expected within 0.15 s;
# the full drain remains long enough to observe and label deadman fallback.
DIRECT_ZERO_HORIZON_SEC=0.15
PHYSICAL_ZERO_HORIZON_SEC=0.75
STATIONARY_WINDOW_SEC=0.25
ANCHOR_TIMEOUT_SEC=0.15


_PHYSICAL_POLICY_BY_CASE={
    "B-H01":"Gate direct zero expected after intentional disarm",
    "B-H02":"Gate fault direct zero expected",
    "B-H03":"startup/preexisting Gate DISARMED zero and stationarity required",
    "B-H05":"startup/preexisting Gate DISARMED zero and stationarity required",
    "B-H06":"adapter invalid immediate zero plus canonical cancellation/Gate stop path",
    "B-S01":"cancellation-driven canonical stop path; feedback shadow does not hide physical evidence",
    "B-S02":"cancellation-driven canonical stop path; odometry shadow does not hide canonical physical odometry",
    "B-S03":"cancellation-driven canonical stop path; TF shadow does not hide canonical physical odometry",
    "B-C01":"cancellation-driven canonical stop path",
}


class PremissionCollisionReadiness:
    """Qualification-side, current-generation collision readiness only.

    `observe` receives runner-owned evidence; it has no ROS side effects and
    deliberately knows nothing about command acquisition or mission state.
    Bool and diagnostic receipts are independently fresh (<100 ms) rather
    than being required to share a timestamp.  The monitor publishes at 20 Hz.
    """
    def __init__(self): self.generation=None

    def observe(self, *, now_ns, now_ros_ns, publisher_count, publisher_node, publisher_gid,
                bool_value, bool_receipt_ns, bool_writer_gid, diagnostic_state, diagnostic_reason,
                diagnostic_healthy_stable_sec, diagnostic_source_age_sec,
                diagnostic_receipt_ns, diagnostic_header_ros_ns, diagnostic_writer_gid, diagnostic_graph_gids,
                bool_post_epoch_count=0, epoch_start_ns=0, epoch_start_ros_ns=0):
        transport_age=None if diagnostic_header_ros_ns is None else now_ros_ns-diagnostic_header_ros_ns
        current=(publisher_count==1 and publisher_node==_EXPECTED_COLLISION_VALIDITY_NODE
                 and bool_value is True and diagnostic_state=="VALID" and diagnostic_reason=="VALID"
                 and bool_post_epoch_count >= 2 and bool_receipt_ns > epoch_start_ns
                 and diagnostic_receipt_ns > epoch_start_ns and diagnostic_header_ros_ns >= epoch_start_ros_ns
                 and diagnostic_writer_gid in diagnostic_graph_gids
                 and diagnostic_healthy_stable_sec is not None and diagnostic_healthy_stable_sec >= PREMISSION_COLLISION_STABLE_SEC
                 and bool_receipt_ns is not None and diagnostic_receipt_ns is not None and transport_age is not None
                 and 0 <= transport_age < PREMISSION_CURRENTNESS_MAX_AGE_NS
                 and now_ns-bool_receipt_ns < PREMISSION_CURRENTNESS_MAX_AGE_NS
                 and now_ns-diagnostic_receipt_ns < PREMISSION_CURRENTNESS_MAX_AGE_NS
                 and diagnostic_source_age_sec is not None
                 and diagnostic_source_age_sec*1e9 + transport_age < PREMISSION_SOURCE_FRESHNESS_NS
                 and abs(bool_receipt_ns-diagnostic_receipt_ns) < PREMISSION_CURRENTNESS_MAX_AGE_NS)
        generation=(publisher_node,publisher_gid)
        generation_changed=self.generation is not None and generation != self.generation
        self.generation=generation
        if generation_changed: current=False
        return {"ready":current, "generation":generation, "semantic_healthy_stable_sec":diagnostic_healthy_stable_sec,
                "bool_age_ns":None if bool_receipt_ns is None else now_ns-bool_receipt_ns,
                "diagnostic_age_ns":None if diagnostic_receipt_ns is None else now_ns-diagnostic_receipt_ns,
                "diagnostic_transport_age_ns":transport_age, "source_age_upper_bound_sec":None if transport_age is None or diagnostic_source_age_sec is None else diagnostic_source_age_sec+transport_age/1e9,
                "diagnostic_state":diagnostic_state, "diagnostic_reason":diagnostic_reason, "bool_value":bool_value,
                "bool_writer_gid":bool_writer_gid,"diagnostic_writer_gid":diagnostic_writer_gid,
                "bool_post_epoch_count":bool_post_epoch_count,"epoch_start_ns":epoch_start_ns,"epoch_start_ros_ns":epoch_start_ros_ns,
                "publisher_count": publisher_count}


def dry_plan(case_id):
    """Serialize the runtime-owned physical contract without changing execution."""
    plan=case_plan(case_id)
    plan.update({
        "expected_health_terminal":f"FAILED / {plan['expected_reason']}",
        "post_health_terminal_phase":"PHYSICAL_EVIDENCE_DRAIN",
        "preferred_zero_source":"DIRECT_COMMAND_ZERO",
        "zero_source_classifications":["DIRECT_COMMAND_ZERO","DEADMAN_ZERO","NO_ZERO_WITHIN_HORIZON"],
        "direct_zero_horizon_sec":DIRECT_ZERO_HORIZON_SEC,
        "zero_classification_deadline_sec":PHYSICAL_ZERO_HORIZON_SEC,
        "post_zero_anchor_required":True,
        "anchor_definition":"first valid zero-twist odometry/fake physical sample at or after confirmed fake-applied zero",
        "anchor_timeout_sec":ANCHOR_TIMEOUT_SEC,
        "stationary_window_sec":STATIONARY_WINDOW_SEC,
        "stationary_window_independent_of_zero_deadline":True,
        "stationary_translation_bound_m":0.02,
        "rotation":"reported separately",
        "writer_finalization":"only after physical adjudication or explicit physical failure",
        "rearm_during_drain":False,
        "additional_health_cancel_during_drain":False,
        "physical_case_policy":_PHYSICAL_POLICY_BY_CASE[case_id],
        "zero_source_semantics":{
            "DIRECT_COMMAND_ZERO":"proves direct canonical command-zero propagation",
            "DEADMAN_ZERO":"proves stale-input backstop zero only; not direct transport PASS",
            "NO_ZERO_WITHIN_HORIZON":"physical failure",
            "stationarity_requirement":"DIRECT_COMMAND_ZERO and DEADMAN_ZERO both require post-zero anchor plus stationary window before physical completion",
        },
    })
    return plan


def _zero(values):
    return all(abs(float(value)) <= 1.0e-12 for value in values)


def classify_physical_evidence(*, terminal_ns, vehicle_samples, fake_diagnostics,
                               odometry, now_ns):
    """Pure classification; receipt times are never treated as source stamps."""
    vehicle_zero = next((row for row in vehicle_samples
                         if row[0] >= terminal_ns and _zero(row[2:8])), None)
    direct_deadline=terminal_ns+int(DIRECT_ZERO_HORIZON_SEC*1e9)
    overall_deadline=terminal_ns+int(PHYSICAL_ZERO_HORIZON_SEC*1e9)
    fake_zero = next((row for row in fake_diagnostics if terminal_ns <= row["monotonic_ns"] <= direct_deadline
                      and row["values"].get("condition") == "VALID"
                      and _zero((row["values"].get("applied_linear_x", 1),
                                 row["values"].get("applied_angular_z", 1)))), None)
    stale_zero = next((row for row in fake_diagnostics if terminal_ns <= row["monotonic_ns"] <= overall_deadline
                       and row["values"].get("condition") == "INPUT_STALE"), None)
    source="DIRECT_COMMAND_ZERO" if fake_zero is not None else "DEADMAN_ZERO" if stale_zero is not None else None
    source_event=fake_zero or stale_zero
    if source_event is None:
        return {"classification":"NO_ZERO_WITHIN_HORIZON",
                "complete":now_ns >= overall_deadline,
                "terminal_ns":terminal_ns}
    zero_ns=source_event["monotonic_ns"]
    anchor=next((row for row in odometry if row[0] >= zero_ns and _zero((row[5],row[6]))), None)
    if anchor is None:
        if now_ns >= zero_ns+int(ANCHOR_TIMEOUT_SEC*1e9):
            return {"classification":source, "complete":True, "physical_pass":False,
                    "failure":"ANCHOR_TIMEOUT", "terminal_ns":terminal_ns,"zero_source_ns":zero_ns}
        return {"classification":source, "complete":False,
                "terminal_ns":terminal_ns,"fake_zero_ns":zero_ns,"vehicle_zero_ns":None if vehicle_zero is None else vehicle_zero[0]}
    end_ns=anchor[0]+int(STATIONARY_WINDOW_SEC*1e9)
    if now_ns < end_ns:
        return {"classification":source, "complete":False,
                "terminal_ns":terminal_ns,"fake_zero_ns":zero_ns,"anchor_ns":anchor[0]}
    window=[row for row in odometry if anchor[0] <= row[0] <= end_ns]
    if any(not _zero((row[5],row[6])) for row in window):
        raise RuntimeError("P4E6B2A4_NONZERO_ODOM_AFTER_ANCHOR")
    dx=float(window[-1][2])-float(anchor[2]); dy=float(window[-1][3])-float(anchor[3])
    if (dx*dx+dy*dy)**.5 > .02:
        raise RuntimeError("P4E6B2A4_STATIONARY_TRANSLATION_BOUND")
    return {"classification":source, "complete":True,"physical_pass":True,
            "terminal_ns":terminal_ns,"vehicle_zero_ns":None if vehicle_zero is None else vehicle_zero[0],
            "fake_zero_ns":zero_ns,"anchor_ns":anchor[0],"stationary_end_ns":end_ns,
            "translation_m":(dx*dx+dy*dy)**.5,"rotation_rad":float(window[-1][4])-float(anchor[4])}


class BoundedWriter:
    def __init__(self,out,capacity=CAPACITY):
        self.out=Path(out); self.capacity=capacity; self.q=queue.Queue(capacity)
        self.next=1; self.persisted=0; self.failure=None; self.handles={}
        self.thread=threading.Thread(target=self._run,name="p4e6b-writer",daemon=False); self.thread.start()
    def put(self,name,item):
        if self.failure: raise RuntimeError(self.failure)
        row=dict(item,evidence_sequence=self.next); self.next+=1
        try:self.q.put_nowait((row["evidence_sequence"],name,json.dumps(row,sort_keys=True)))
        except queue.Full: self.failure="P4E6B2A1_WRITER_OVERFLOW_NEEDS_REVIEW"; raise RuntimeError(self.failure)
        return row
    def _run(self):
        try:
            while True:
                item=self.q.get()
                if item is None:self.q.task_done();break
                seq,name,text=item
                if seq!=self.persisted+1: raise RuntimeError("writer sequence gap")
                h=self.handles.setdefault(name,(self.out/name).open("a",encoding="utf-8")); h.write(text+"\n")
                self.persisted=seq; self.q.task_done()
        except Exception as exc:self.failure=repr(exc)
    def finalize(self):
        self.q.join(); self.q.put(None); self.thread.join(2)
        for h in self.handles.values():h.flush();os.fsync(h.fileno());h.close()
        status={"capacity":self.capacity,"last_enqueued":self.next-1,"last_persisted":self.persisted,
                "queue_depth":self.q.qsize(),"worker_stopped":not self.thread.is_alive(),"failure":self.failure}
        if self.failure or self.persisted!=self.next-1 or self.thread.is_alive():raise RuntimeError("P4E6B2A1_WRITER_NEEDS_REVIEW")
        return status


class HealthRuntimeRunner(P4E5BRunner):
    def __init__(self,out,case_id,*,premission_diagnostic_reliability=None,premission_diagnostic_depth=None):
        super().__init__(out); self.case_id=case_id; self.model=HealthCampaignModel(case_id)
        self.writer=BoundedWriter(out); self._injection_clients={}
        self._premission_bool=None; self._premission_diag=None; self._premission_epoch=None; self._premission_epoch_start_ns=0; self._premission_epoch_start_ros_ns=0; self._premission_bool_count=0
        latest=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        # Readiness needs current target state, not historical shared-topic
        # replay.  The D3C actual-helper campaign selected BEST_EFFORT depth 1;
        # diagnostics still fail closed on publication/receipt/source-age.
        diagnostic_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,
                                  depth=(1 if premission_diagnostic_depth is None else premission_diagnostic_depth),
                                  reliability=(ReliabilityPolicy.BEST_EFFORT if premission_diagnostic_reliability is None
                                               else premission_diagnostic_reliability),
                                  durability=DurabilityPolicy.VOLATILE)
        self._premission_bool_qos=latest; self._premission_diag_qos=diagnostic_qos
        self._premission_bool_sub=None; self._premission_diag_sub=None
        self._recreate_premission_subscriptions()
        self.arm_control=self.create_client(SetBool,"/vehicle_cmd_safety/arm")
        targets={"B-H02":"/p4e6b_gate_localization_relay/set_parameters",
          "B-H03":"/phase4_p4e_permission_fixture/set_parameters","B-H05":"/phase4_p4b_synthetic_obstacles/set_parameters",
          "B-H06":"/mock_wheelchair_cmd_adapter/set_parameters","B-S01":"/p4e6b_feedback_relay/set_parameters",
          "B-S02":"/p4e6b_odom_relay/set_parameters","B-S03":"/p4e6b_tf_relay/set_parameters",
          "B-C01":"/p4e6b_safe_relay/set_parameters"}
        for case,target in targets.items():self._injection_clients[case]=self.create_client(SetParameters,target)

    def _recreate_premission_subscriptions(self):
        for sub in (self._premission_bool_sub,self._premission_diag_sub):
            if sub is not None: self.destroy_subscription(sub)
        self._premission_bool_sub=self.create_subscription(Bool,"/system/collision_monitor_valid",self._premission_bool_cb,self._premission_bool_qos)
        self._premission_diag_sub=self.create_subscription(DiagnosticArray,"/diagnostics",self._premission_diag_cb,self._premission_diag_qos)
        self._premission_bool=None; self._premission_diag=None; self._premission_bool_count=0
        self._premission_epoch_start_ns=time.monotonic_ns(); self._premission_epoch_start_ros_ns=self.get_clock().now().nanoseconds

    def _premission_bool_cb(self,msg):
        self._premission_bool=(bool(msg.data),time.monotonic_ns()); self._premission_bool_count+=1
    def _premission_diag_cb(self,msg):
        receipt=time.monotonic_ns(); header=self.get_clock().now().nanoseconds
        header=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
        for status in msg.status:
            if status.name=="vehicle_cmd_safety/collision_monitor_validity_monitor":
                values={v.key:v.value for v in status.values}
                self._premission_diag=(values,receipt,header)

    def _raise_if_competing_terminal(self, nav):
        terminal=competing_terminal_for_current_goal(
            self.states, accepted_ns=nav["monotonic_ns"],
            mission_id=nav["mission_id"], route_id=nav["route_id"],
            waypoint=int(nav["waypoint_index"]), goal_uuid=nav["active_goal_uuid"])
        if terminal is not None:
            self.evidence("PRE_INJECTION_COMPETING_TERMINAL", **terminal)
            raise RuntimeError(f"PRE_INJECTION_COMPETING_REASON:{terminal['reason']}")

    def wait_active_case_precondition(self, nav, timeout_sec=1.6):
        """H02/H01-style command acquisition, terminal-aware by construction."""
        accepted_ns=nav["monotonic_ns"]; deadline=time.monotonic()+timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self,timeout_sec=.01); self._raise_if_competing_terminal(nav)
            raw=[x for x in self.samples["/cmd_vel_nav_raw"] if x[0]>=accepted_ns and nz(x[2:8])]
            safe=[x for x in self.samples["/cmd_vel_nav_safe"] if x[0]>=accepted_ns and nz(x[2:8])]
            if len(raw)<3 or len(safe)<3: continue
            now_ns=time.monotonic_ns()
            if now_ns-safe[-1][0]>250_000_000 or now_ns-safe[0][0]<500_000_000: continue
            gate=self.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
            if gate is None: continue
            if gate[1].get("state")!="DISARMED" or gate[1].get("fault_latched")!="false":
                raise RuntimeError(f"gate not healthy DISARMED: {gate[1]}")
            if any(nz(x[2:8]) for x in self.samples["/vehicle_cmd_safe"] if x[0]>=accepted_ns):
                raise RuntimeError("gate bypass while DISARMED")
            if any(nz(x[2]) for x in self.samples["/wheelchair_control_command_mock"] if x[0]>=accepted_ns):
                raise RuntimeError("adapter bypass while DISARMED")
            if any(not self.permissions.get(t,(False,0))[0] or now_ns-self.permissions[t][1]>500_000_000 for t in TOPICS[2:5]):
                raise RuntimeError("permission freshness lost")
            skew=abs(safe[-1][0]-raw[-1][0]); numeric=max(abs(a-b) for a,b in zip(safe[-1][2:8],raw[-1][2:8]))
            if skew>75_000_000 or numeric>.05: raise RuntimeError(f"raw/safe pairing failed: {skew},{numeric}")
            return self.emit("scenario_events.jsonl",{"event":"P4E6B_ACTIVE_PRECONDITION_PASS","accepted_ns":accepted_ns,"safe_sample_count":len(safe),"raw_sample_count":len(raw),"newest_safe_age_ns":now_ns-safe[-1][0],"raw_safe_skew_ns":skew,"raw_safe_max_error":numeric})
        self._raise_if_competing_terminal(nav)
        raise RuntimeError("fresh safe-command precondition timeout")

    def _premission_collision_snapshot(self, now_ns):
        endpoints=self.get_publishers_info_by_topic("/system/collision_monitor_valid")
        names=[ep.node_name for ep in endpoints]
        gids=[bytes(ep.endpoint_gid).hex() for ep in endpoints]
        boolean=self._premission_bool; diag=self._premission_diag
        targets=[ep for ep in self.get_publishers_info_by_topic("/diagnostics") if ep.node_name==_EXPECTED_COLLISION_VALIDITY_NODE and ep.node_namespace=="/"]
        token=(names[0] if len(names)==1 else "",gids[0] if len(gids)==1 else "",bytes(targets[0].endpoint_gid).hex() if len(targets)==1 else "")
        if token!=self._premission_epoch:
            self._premission_epoch=token; self._recreate_premission_subscriptions(); boolean=None;diag=None
        values={} if diag is None else diag[0]
        def number(key):
            try:return float(values[key])
            except (KeyError,ValueError,TypeError):return None
        return {"now_ns":now_ns,"now_ros_ns":self.get_clock().now().nanoseconds,"publisher_count":len(endpoints),
                "publisher_node":names[0] if len(names)==1 else "",
                "publisher_gid":gids[0] if len(gids)==1 else "",
                "bool_value":None if boolean is None else boolean[0],"bool_receipt_ns":None if boolean is None else boolean[1],
                "bool_writer_gid":gids[0] if len(gids)==1 else None,"bool_post_epoch_count":self._premission_bool_count,
                "diagnostic_state":values.get("state"),"diagnostic_reason":values.get("reason_code"),
                "diagnostic_healthy_stable_sec":number("healthy_stable_sec"),"diagnostic_source_age_sec":number("source_age_sec"),
                "diagnostic_receipt_ns":None if diag is None else diag[1],"diagnostic_header_ros_ns":None if diag is None else diag[2],
                "diagnostic_writer_gid":token[2] if len(targets)==1 else None,"diagnostic_graph_gids":{token[2]} if len(targets)==1 else set(),
                "epoch_start_ns":self._premission_epoch_start_ns,"epoch_start_ros_ns":self._premission_epoch_start_ros_ns}

    def wait_active_premission_health(self, timeout_sec=PREMISSION_HEALTH_TIMEOUT_SEC):
        """Hold RouteMission for ACTIVE cases until canonical startup health is real.

        No terminal/UUID check belongs here: there is intentionally no mission
        yet.  Raw/safe acquisition remains in wait_active_case_precondition.
        """
        model=PremissionCollisionReadiness(); deadline=time.monotonic()+timeout_sec; last={}
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.01)
            last=model.observe(**self._premission_collision_snapshot(time.monotonic_ns()))
            if last["ready"]:
                return self.evidence("PREMISSION_HEALTH_READY", **last)
        raise RuntimeError("PREMISSION_HEALTH_READINESS_TIMEOUT:"+json.dumps(last,sort_keys=True))

    def wait_preinjection(self, predicate, timeout, reason, nav):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.01); self._raise_if_competing_terminal(nav)
            result=predicate()
            if result:return result
        self._raise_if_competing_terminal(nav)
        raise RuntimeError(reason)
    def evidence(self,event,**fields):return self.writer.put("health_campaign.jsonl",{"monotonic_ns":time.monotonic_ns(),"event":event,"case_id":self.case_id,**fields})
    def _parameter(self,name,value):
        if isinstance(value,bool): pv=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value)
        else: pv=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=value)
        return Parameter(name=name,value=pv)
    def inject_once(self):
        request_ns=time.monotonic_ns(); case=self.case_id
        if case=="B-H01":
            client=self.arm_control; req=SetBool.Request(data=False)
        else:
            client=self._injection_clients[case]
            values={"B-H02":("force_invalid",True),"B-H03":("localization_valid",False),
              "B-H05":("mode","SILENT"),"B-H06":("qualification_force_invalid",True),
              "B-S01":("suppress_feedback",True),"B-S02":("suppress",True),
              "B-S03":("suppress_dynamic",True),"B-C01":("suppress",True)}
            req=SetParameters.Request(parameters=[self._parameter(*values[case])])
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError("injection service unavailable")
        future=client.call_async(req); rclpy.spin_until_future_complete(self,future,timeout_sec=5)
        response=future.result(); success=bool(response and (response.success if case=="B-H01" else response.results[0].successful))
        tool_state="CONFIRMED" if case=="B-H01" else str(response.results[0].reason or "CONFIRMED")
        if case=="B-S01" and tool_state!="INJECTED": success=False
        ack_ns=time.monotonic_ns(); self.model.inject(request_ns,ack_ns,successful=success,tool_state=tool_state)
        return self.evidence("INJECTED",request_ns=request_ns,ack_ns=ack_ns,success=success,
                             tool_state=tool_state,injection_count=1)

    def drain_physical_evidence(self, terminal_ns):
        self.evidence("PHYSICAL_EVIDENCE_DRAIN_STARTED", health_terminal_ns=terminal_ns,
                      direct_horizon_sec=DIRECT_ZERO_HORIZON_SEC,
                      total_horizon_sec=PHYSICAL_ZERO_HORIZON_SEC,
                      stationary_window_sec=STATIONARY_WINDOW_SEC)
        deadline=time.monotonic()+PHYSICAL_ZERO_HORIZON_SEC+ANCHOR_TIMEOUT_SEC+STATIONARY_WINDOW_SEC
        while time.monotonic() < deadline:
            result=classify_physical_evidence(terminal_ns=terminal_ns,
                vehicle_samples=self.samples["/vehicle_cmd_safe"],
                fake_diagnostics=self.diagnostic_history["fake_base"],
                odometry=self.samples["/Odometry"],now_ns=time.monotonic_ns())
            if result["complete"]:
                self.evidence("PHYSICAL_EVIDENCE_DRAIN_COMPLETE", **result)
                return result
            rclpy.spin_once(self,timeout_sec=.01)
        result=classify_physical_evidence(terminal_ns=terminal_ns,
            vehicle_samples=self.samples["/vehicle_cmd_safe"],
            fake_diagnostics=self.diagnostic_history["fake_base"],
            odometry=self.samples["/Odometry"],now_ns=time.monotonic_ns())
        self.evidence("PHYSICAL_EVIDENCE_DRAIN_COMPLETE", **result)
        return result


def execute_campaign(case_id,out,ros_args=None):
    """Full-chain entry point. Never called without the explicit CLI execute flag."""
    rclpy.init(args=ros_args); node=HealthRuntimeRunner(out,case_id); error=None; result={}
    try:
        node.model.begin_runtime(); node.evidence("ATTEMPT_STARTED",attempt_count=1)
        node.spin(5); node.graph(); node.lifecycle()
        node.publish_initial(); node.spin(.5)
        if case_id in ACTIVE_CASES:
            node.wait_active_premission_health()
        node.publish_route(); node.trigger("start")
        node.wait_for(lambda:next((x for x in node.states if x["state"]==MissionState.NAVIGATING and x["active_goal_uuid"]),None),10,"UUID timeout")
        nav=next(x for x in node.states if x["state"]==MissionState.NAVIGATING and x["active_goal_uuid"]); uuid=nav["active_goal_uuid"]
        node.model.mission_start(uuid); node.wait_preinjection(lambda:node.latest_policy("INITIAL_PRIMING",nav["monotonic_ns"],uuid),2,"priming timeout",nav)
        if case_id in ACTIVE_CASES:
            node.wait_active_case_precondition(nav); node.arm_gate(); node.model.arm_active(stream_established=True,healthy=True)
            clear_start=time.monotonic_ns(); node.spin(5.1)
            node._raise_if_competing_terminal(nav)
            if not health_ok(node,time.monotonic_ns()):raise RuntimeError("healthy CLEAR failed")
            node.model.stable_clear((time.monotonic_ns()-clear_start)/1e9)
        node.model.arm_case(target_healthy=True); node.evidence("ARMED_FOR_CASE",goal_uuid=uuid)
        ack=node.inject_once(); reason=SCENARIOS[case_id].reason
        node.wait_for(lambda:state_after(node,MissionState.FAILED,ack["ack_ns"],reason),8,"FAILED timeout")
        history=adjudicate_health_terminal_history(origin_ns=ack["ack_ns"],reason=reason,
            mission_id=nav["mission_id"],route_id=nav["route_id"],waypoint=0,uuid=uuid,
            states=node.states,statuses=[{**x,"status_name":{GoalStatus.STATUS_CANCELING:"CANCELING",GoalStatus.STATUS_CANCELED:"CANCELED"}.get(x["status"],str(x["status"]))} for x in node.action_statuses])
        terminal=state_after(node,MissionState.FAILED,ack["ack_ns"],reason)
        physical=node.drain_physical_evidence(terminal["monotonic_ns"])
        result={"pass":True,"case_id":case_id,"attempt_count":1,"history":history,"physical":physical}
    except Exception as exc:error=repr(exc);result={"pass":False,"case_id":case_id,"error":error}
    finally:
        result["writer"]=node.writer.finalize(); (Path(out)/"terminal_metrics.json").write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
        node.close();node.destroy_node();rclpy.shutdown()
    if error:raise SystemExit(1)


def parse_args(args=None):
    p=argparse.ArgumentParser(); p.add_argument("--case-id",required=True,choices=sorted(SCENARIOS)); p.add_argument("--output-dir",required=True)
    modes=p.add_mutually_exclusive_group(required=True); modes.add_argument("--dry-plan-only",action="store_true"); modes.add_argument("--execute-authorized-case",action="store_true")
    raw=list(sys.argv[1:] if args is None else args)
    return p.parse_args(remove_ros_args(raw))


def main(args=None, *, campaign_executor=execute_campaign):
    raw=list(sys.argv[1:] if args is None else args); ns=parse_args(raw)
    out=Path(ns.output_dir); out.mkdir(parents=True,exist_ok=True)
    plan=dry_plan(ns.case_id); (out/"scenario_spec.json").write_text(json.dumps(plan,indent=2,sort_keys=True)+"\n")
    if ns.dry_plan_only: print(json.dumps(plan,sort_keys=True)); return
    campaign_executor(ns.case_id,out,raw)
