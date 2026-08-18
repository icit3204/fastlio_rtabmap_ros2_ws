"""Pure bounded contracts for P4-E.6B qualification tooling."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Mapping, Sequence


SHADOW_PREFIX = "/phase4_qualification/p4e6b"


class InjectionPhase(Enum):
    DISABLED = auto()
    ARMED_FOR_CASE = auto()
    INJECTED = auto()
    TERMINAL_OBSERVED = auto()
    FINALIZED = auto()


@dataclass(frozen=True)
class ScenarioSpec:
    case_id: str
    reason: str
    activation: str
    threshold_sec: float
    comparison: str
    owner: str
    mechanism: str
    zero_source: str
    competing: tuple[str, ...]
    needs_relay: bool = False


SCENARIOS: Mapping[str, ScenarioSpec] = {
    "B-H01": ScenarioSpec("B-H01", "GATE_DISARMED", "ACTIVE", 0.0, "immediate", "Gate", "arm_false", "Gate", ("GATE_FAULT",)),
    "B-H02": ScenarioSpec("B-H02", "GATE_FAULT", "ACTIVE", 0.0, "immediate", "gate permission relay", "gate_only_false", "Gate", ()),
    "B-H03": ScenarioSpec("B-H03", "LOCALIZATION_INVALID", "INITIAL_PRIMING", 0.0, "immediate", "permission fixture", "localization_false", "Gate DISARMED", ("GATE_FAULT",)),
    "B-H05": ScenarioSpec("B-H05", "COLLISION_MONITOR_INVALID", "INITIAL_PRIMING", .5, ">", "synthetic scan/validity monitor", "scan_silent", "Gate DISARMED", ("GATE_FAULT",)),
    "B-H06": ScenarioSpec("B-H06", "ADAPTER_INVALID", "ACTIVE", 0.0, "immediate", "MM diagnostic relay", "adapter_warn_shadow", "cancel/Gate", ("GATE_FAULT",), True),
    "B-S01": ScenarioSpec("B-S01", "FEEDBACK_STALE", "ACTIVE", 2.0, ">", "feedback-only shadow relay", "suppress_feedback", "cancel/Gate", ("GOAL_ABORTED",), True),
    "B-S02": ScenarioSpec("B-S02", "ODOMETRY_STALE", "ACTIVE", .5, ">", "MM odometry relay", "suppress_odom", "cancel/Gate", ("TF_STALE",), True),
    "B-S03": ScenarioSpec("B-S03", "TF_STALE", "ACTIVE", .5, ">", "MM TF relay", "suppress_dynamic_tf", "cancel/Gate", ("ODOMETRY_STALE",), True),
    "B-C01": ScenarioSpec("B-C01", "COMMAND_PAIR_STALE", "ACTIVE_STREAM_ESTABLISHED", .25, ">=", "MM safe relay", "suppress_safe", "cancel/Gate", ("FEEDBACK_STALE",), True),
}


class InjectionController:
    def __init__(self, case_id: str):
        if case_id not in SCENARIOS:
            raise ValueError("case is not runnable P4-E.6B scope")
        self.spec = SCENARIOS[case_id]
        self.phase = InjectionPhase.DISABLED
        self.inject_count = 0

    def arm(self):
        if self.phase is not InjectionPhase.DISABLED:
            raise RuntimeError("case may be armed once")
        self.phase = InjectionPhase.ARMED_FOR_CASE

    def inject(self):
        if self.phase is not InjectionPhase.ARMED_FOR_CASE:
            raise RuntimeError("injection requires armed case")
        self.inject_count += 1
        self.phase = InjectionPhase.INJECTED

    def terminal(self):
        if self.phase is not InjectionPhase.INJECTED:
            raise RuntimeError("terminal requires asserted fault")
        self.phase = InjectionPhase.TERMINAL_OBSERVED

    def finalize(self):
        if self.phase is not InjectionPhase.TERMINAL_OBSERVED:
            raise RuntimeError("fault must remain asserted through terminal")
        self.phase = InjectionPhase.FINALIZED


def threshold_fired(age: float, spec: ScenarioSpec) -> bool:
    if spec.comparison == ">": return age > spec.threshold_sec
    if spec.comparison == ">=": return age >= spec.threshold_sec
    return True


def validate_authority(*, canonical_publishers: int, shadow_publishers: int) -> None:
    if canonical_publishers != 1 or shadow_publishers != 1:
        raise RuntimeError("publisher authority must be exactly one on canonical and shadow")


def validate_s01_injection(*, ready_before_mission: bool, healthy_at_command: bool,
                           acknowledgement: dict, first_reason: str) -> None:
    """Reject accidental relay loss as an S01 injection."""
    if not ready_before_mission or not healthy_at_command:
        raise RuntimeError("P4E6B_S01_TOOL_FAILURE_NEEDS_REVIEW")
    if acknowledgement.get("state") != "INJECTED":
        raise RuntimeError("P4E6B1A_S01_INJECTION_ACK_NEEDS_REVIEW")
    if acknowledgement.get("injection_count") != 1 or int(
            acknowledgement.get("injection_monotonic_ns", 0)) <= 0:
        raise RuntimeError("P4E6B1A_S01_INJECTION_ACK_NEEDS_REVIEW")
    if first_reason != "FEEDBACK_STALE":
        raise RuntimeError("unexpected first reason")


def adjudicate_health_terminal_history(*, origin_ns: int, reason: str, mission_id: str,
                                       route_id: str, waypoint: int, uuid: str,
                                       states: Sequence[dict], statuses: Sequence[dict]) -> dict:
    owned=lambda r: r.get("mission_id")==mission_id and r.get("route_id")==route_id and int(r.get("waypoint_index",-1))==waypoint
    cancel=next((r for r in states if int(r["monotonic_ns"])>=origin_ns and owned(r) and r.get("state_name")=="CANCELLING" and r.get("reason_code")==reason and r.get("active_goal_uuid")==uuid),None)
    if cancel is None: raise RuntimeError("missing identity-qualified CANCELLING")
    failed=next((r for r in states if int(r["monotonic_ns"])>=int(cancel["monotonic_ns"]) and owned(r) and r.get("state_name")=="FAILED"),None)
    if failed is None or failed.get("reason_code")!=reason: raise RuntimeError("missing FAILED with originating reason")
    if any(owned(r) and r.get("state_name") in ("BLOCKED","CANCELLED","PAUSED") and int(r["monotonic_ns"])>=origin_ns for r in states): raise RuntimeError("wrong terminal family")
    ack=next((r for r in states if owned(r) and r.get("state_name")=="CANCELLING" and r.get("reason_code")=="HEALTH_CANCEL_ACK_ACCEPTED" and int(cancel["monotonic_ns"])<=int(r["monotonic_ns"])<=int(failed["monotonic_ns"])),None)
    canceling=[r for r in statuses if r.get("goal_uuid")==uuid and r.get("status_name")=="CANCELING" and int(cancel["monotonic_ns"])<=int(r["monotonic_ns"])<=int(failed["monotonic_ns"])]
    canceled=[r for r in statuses if r.get("goal_uuid")==uuid and r.get("status_name")=="CANCELED" and int(cancel["monotonic_ns"])<=int(r["monotonic_ns"])<=int(failed["monotonic_ns"])]
    if ack is None or len(canceling)!=1 or len(canceled)!=1: raise RuntimeError("cancel cardinality/history invalid")
    times=[origin_ns,int(cancel["monotonic_ns"]),int(canceling[0]["monotonic_ns"]),int(ack["monotonic_ns"]),int(canceled[0]["monotonic_ns"]),int(failed["monotonic_ns"])]
    if times != sorted(times): raise RuntimeError("terminal chronology invalid")
    return {"pass":True,"reason":reason,"times":times}


def dry_model(case_id: str, age: float, competing: Sequence[str]=()) -> str:
    spec=SCENARIOS[case_id]
    if competing: return competing[0]
    return spec.reason if threshold_fired(age,spec) else "PENDING"


MISSION_GEOMETRY = {"initial": (5.425, -53.725, 0.0),
                    "waypoints": ((8.425, -53.725), (9.425, -53.725))}
ACTIVE_CASES = frozenset(("B-H01", "B-H02", "B-H06", "B-S01", "B-S02",
                          "B-S03", "B-C01"))
PRIMING_CASES = frozenset(("B-H03", "B-H05"))
INJECTION_TARGETS = {
    "B-H01": ("/vehicle_cmd_safety/arm", "data=false"),
    "B-H02": ("/p4e6b_gate_localization_relay/set_parameters", "force_invalid=true"),
    "B-H03": ("/phase4_p4e_permission_fixture/set_parameters", "localization_valid=false"),
    "B-H05": ("/phase4_p4b_synthetic_obstacles/set_parameters", "mode=SILENT"),
    "B-H06": ("/mock_wheelchair_cmd_adapter/set_parameters", "qualification_force_invalid=true"),
    "B-S01": ("/p4e6b_feedback_relay/set_parameters", "suppress_feedback=true"),
    "B-S02": ("/p4e6b_odom_relay/set_parameters", "suppress=true"),
    "B-S03": ("/p4e6b_tf_relay/set_parameters", "suppress_dynamic=true"),
    "B-C01": ("/p4e6b_safe_relay/set_parameters", "suppress=true"),
}


class CampaignState(Enum):
    PREFLIGHT = auto(); READINESS = auto(); MISSION_STARTING = auto()
    INITIAL_PRIMING = auto(); ACTIVE = auto(); HEALTHY_CLEAR = auto()
    ARMED_FOR_CASE = auto(); INJECTED = auto(); CANCELLING = auto()
    TERMINAL = auto(); FINALIZED = auto()


def case_plan(case_id: str) -> dict:
    if case_id not in SCENARIOS:
        raise ValueError("exactly one accepted case ID is required")
    spec = SCENARIOS[case_id]; target, operation = INJECTION_TARGETS[case_id]
    return {"case_id": case_id, "expected_reason": spec.reason,
            "phase": spec.activation, "threshold_sec": spec.threshold_sec,
            "comparison": spec.comparison, "injection_owner": spec.owner,
            "injection_mechanism": spec.mechanism, "injection_target": target,
            "injection_operation": operation, "physical_zero_source": spec.zero_source,
            "competing_reasons": list(spec.competing), "mission_geometry": MISSION_GEOMETRY,
            "startup_requirements": ("UUID before injection; healthy DISARMED; no initial arm"
                                     if case_id in PRIMING_CASES else
                                     "STREAM_ESTABLISHED; one arm; ACTIVE; healthy CLEAR >=5s"),
            "terminal_contract": ("reason -> CANCELLING -> one CANCELING -> "
                                  "HEALTH_CANCEL_ACK_ACCEPTED -> CANCELED -> FAILED/reason; "
                                  "no UUID permits immediate FAILED and zero cancels"),
            "one_attempt": True, "runtime_executed": False,
            "no_runtime_statement": "dry plan creates no ROS autonomy traffic"}


class HealthCampaignModel:
    """Pure fail-closed policy shared by runtime and isolated qualification."""
    def __init__(self, case_id: str):
        self.spec = SCENARIOS[case_id]; self.case_id = case_id
        self.state = CampaignState.PREFLIGHT; self.attempt_count = 0
        self.route_count = self.start_count = self.arm_count = 0
        self.injection_count = 0; self.ack = None; self.goal_uuid = ""

    def begin_runtime(self):
        if self.attempt_count: raise RuntimeError("campaign retry forbidden")
        self.attempt_count = 1; self.state = CampaignState.READINESS

    def mission_start(self, goal_uuid="uuid-0"):
        if self.attempt_count != 1 or self.route_count or self.start_count:
            raise RuntimeError("second mission forbidden")
        self.route_count = self.start_count = 1; self.goal_uuid = goal_uuid
        self.state = CampaignState.INITIAL_PRIMING

    def arm_active(self, *, stream_established: bool, healthy: bool):
        if self.case_id in PRIMING_CASES: raise RuntimeError("priming case must inject before arm")
        if not stream_established or not healthy or self.state is not CampaignState.INITIAL_PRIMING:
            raise RuntimeError("ACTIVE preconditions failed")
        self.arm_count = 1; self.state = CampaignState.ACTIVE

    def stable_clear(self, duration_sec: float, *, healthy=True):
        if self.state is not CampaignState.ACTIVE or not healthy or duration_sec < 5.0:
            raise RuntimeError("stable healthy CLEAR <5s")
        self.state = CampaignState.HEALTHY_CLEAR

    def arm_case(self, *, target_healthy=True):
        expected = CampaignState.INITIAL_PRIMING if self.case_id in PRIMING_CASES else CampaignState.HEALTHY_CLEAR
        if self.state is not expected or not target_healthy: raise RuntimeError("injection preconditions failed")
        self.state = CampaignState.ARMED_FOR_CASE

    def inject(self, request_ns: int, ack_ns: int, *, successful=True, tool_state="INJECTED"):
        if self.state is not CampaignState.ARMED_FOR_CASE or self.injection_count:
            raise RuntimeError("duplicate or unarmed injection")
        if not successful or ack_ns < request_ns or tool_state not in ("INJECTED", "CONFIRMED"):
            raise RuntimeError("P4E6B2A1_INJECTION_DISPATCH_NEEDS_REVIEW")
        self.injection_count = 1; self.ack = {"case_id": self.case_id,
            "request_ns": request_ns, "ack_ns": ack_ns, "tool_state": tool_state,
            "injection_count": 1}; self.state = CampaignState.INJECTED

    def adjudicate(self, *, first_reason, states, statuses):
        if first_reason != self.spec.reason: raise RuntimeError("competing first reason")
        if not self.goal_uuid:
            failed=[x for x in states if x.get("state_name")=="FAILED" and x.get("reason_code")==self.spec.reason]
            if len(failed)!=1 or statuses: raise RuntimeError("no-UUID terminal contract invalid")
            self.state=CampaignState.TERMINAL; return {"pass":True,"cancel_count":0}
        result=adjudicate_health_terminal_history(origin_ns=self.ack["ack_ns"],
            reason=self.spec.reason,mission_id="m",route_id="r",waypoint=0,
            uuid=self.goal_uuid,states=states,statuses=statuses)
        self.state=CampaignState.TERMINAL; return result

    def finalize(self):
        if self.state is not CampaignState.TERMINAL: raise RuntimeError("terminal missing")
        self.state=CampaignState.FINALIZED


def physical_stop_ledger(samples: Mapping[str, Sequence[dict]], *, anchor_ns: int,
                         origin_xy: tuple[float, float], final_xy: tuple[float, float],
                         origin_yaw=0.0, final_yaw=0.0) -> dict:
    required=("vehicle","adapter","fake")
    violations={name:[x for x in samples.get(name,()) if int(x["ns"])>=anchor_ns and x.get("nonzero")]
                for name in required}
    translation=math.hypot(final_xy[0]-origin_xy[0],final_xy[1]-origin_xy[1])
    result={"anchor_ns":anchor_ns,"translation_m":translation,
            "rotation_rad":final_yaw-origin_yaw,"post_anchor_nonzero":violations}
    if any(violations.values()) or translation > .02: raise RuntimeError("physical zero ledger failed")
    result["pass"]=True; return result
