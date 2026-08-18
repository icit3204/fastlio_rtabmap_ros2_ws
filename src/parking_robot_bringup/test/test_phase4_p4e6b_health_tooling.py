import pytest
from pathlib import Path
from parking_robot_bringup.phase4_p4e6b_health_core import *

def state(ns,name,reason,uuid="u",mission="m"):
 return {"monotonic_ns":ns,"state_name":name,"reason_code":reason,"active_goal_uuid":uuid,"mission_id":mission,"route_id":"r","waypoint_index":0}
def valid_history(reason="GATE_DISARMED"):
 return [state(10,"CANCELLING",reason),state(12,"CANCELLING","HEALTH_CANCEL_ACK_ACCEPTED"),state(14,"FAILED",reason,uuid="")],[{"monotonic_ns":11,"goal_uuid":"u","status_name":"CANCELING"},{"monotonic_ns":13,"goal_uuid":"u","status_name":"CANCELED"}]

def test_exact_nine_specs_and_exclusions():
 assert set(SCENARIOS)=={"B-H01","B-H02","B-H03","B-H05","B-H06","B-S01","B-S02","B-S03","B-C01"}
 assert all(x.reason not in {"CONTROLLER_INVALID","INITIAL_COMMAND_ACQUISITION_TIMEOUT"} for x in SCENARIOS.values())

@pytest.mark.parametrize("case",sorted(SCENARIOS))
def test_injection_once_and_fault_hold(case):
 c=InjectionController(case); c.arm(); c.inject()
 with pytest.raises(RuntimeError):c.inject()
 with pytest.raises(RuntimeError):c.finalize()
 c.terminal(); c.finalize(); assert c.inject_count==1

@pytest.mark.parametrize("case",["B-S01","B-S02","B-S03"])
@pytest.mark.parametrize("delta",[-1e-6,0,1e-6])
def test_strict_stale_boundaries(case,delta):
 s=SCENARIOS[case]; assert threshold_fired(s.threshold_sec+delta,s)==(delta>0)

@pytest.mark.parametrize("ms,expected",[(249,False),(250,True),(251,True)])
def test_command_pair_boundary(ms,expected): assert threshold_fired(ms/1000,SCENARIOS["B-C01"]) is expected

def test_health_terminal_history_passes():
 s,a=valid_history(); assert adjudicate_health_terminal_history(origin_ns=9,reason="GATE_DISARMED",mission_id="m",route_id="r",waypoint=0,uuid="u",states=s,statuses=a)["pass"]

@pytest.mark.parametrize("mutation",["wrong_reason","wrong_uuid","wrong_mission","missing_cancel","duplicate_cancel","blocked","cancelled","paused","failed_early","aborted"])
def test_terminal_negative_matrix(mutation):
 s,a=valid_history()
 if mutation=="wrong_reason":s[0]["reason_code"]="OTHER"
 elif mutation=="wrong_uuid":s[0]["active_goal_uuid"]="x"
 elif mutation=="wrong_mission":s[0]["mission_id"]="x"
 elif mutation=="missing_cancel":s=s[1:]
 elif mutation=="duplicate_cancel":a.append(dict(a[0]))
 elif mutation in ("blocked","cancelled","paused"):s[-1]["state_name"]=mutation.upper()
 elif mutation=="failed_early":s[-1]["monotonic_ns"]=10
 elif mutation=="aborted":a[-1]["status_name"]="ABORTED"
 with pytest.raises(RuntimeError):adjudicate_health_terminal_history(origin_ns=9,reason="GATE_DISARMED",mission_id="m",route_id="r",waypoint=0,uuid="u",states=s,statuses=a)

@pytest.mark.parametrize("canonical,shadow",[(0,1),(2,1),(1,0),(1,2)])
def test_authority_rejects_wrong_counts(canonical,shadow):
 with pytest.raises(RuntimeError):validate_authority(canonical_publishers=canonical,shadow_publishers=shadow)
def test_authority_accepts_one_each():validate_authority(canonical_publishers=1,shadow_publishers=1)

@pytest.mark.parametrize("case",sorted(SCENARIOS))
def test_dry_model_each_case(case):
 s=SCENARIOS[case]; age=s.threshold_sec+(.001 if s.comparison==">" else 0); assert dry_model(case,age)==s.reason

def test_launch_and_relay_contract_static():
 root=Path(__file__).parents[1]; launch=(root/"launch/phase4_p4e6b_health_matrix.launch.py").read_text(); relay=(root/"parking_robot_bringup/phase4_p4e6b_observation_relay.py").read_text()
 assert "/phase4_qualification/p4e6b/" in relay and "force_invalid\":False" in launch
 assert '"/vehicle_cmd_safe"' not in relay

def test_at_least_fifty_order_variants_per_timed_case():
 for case in ("B-H05","B-S01","B-S02","B-S03","B-C01"):
  spec=SCENARIOS[case]
  outcomes=[]
  for phase in range(64):
   before=spec.threshold_sec-(phase+1)*1e-9
   at_or_after=spec.threshold_sec+(phase*1e-9 if spec.comparison==">=" else (phase+1)*1e-9)
   outcomes.append((dry_model(case,before),dry_model(case,at_or_after)))
  assert len(outcomes)>=50
  assert all(a=="PENDING" and b==spec.reason for a,b in outcomes)

def test_s01_at_least_one_hundred_callback_phase_variants():
 spec=SCENARIOS["B-S01"]
 for phase in range(128):
  epsilon=(phase+1)*1e-10
  assert dry_model("B-S01",spec.threshold_sec-epsilon)=="PENDING"
  assert dry_model("B-S01",spec.threshold_sec)=="PENDING"
  assert dry_model("B-S01",spec.threshold_sec+epsilon)=="FEEDBACK_STALE"

def test_competing_reason_is_not_relabelled():
 assert dry_model("B-C01",.251,["FEEDBACK_STALE"])=="FEEDBACK_STALE"

def test_default_launch_is_transparent_and_dedicated():
 root=Path(__file__).parents[1]; text=(root/"launch/phase4_p4e6b_health_matrix.launch.py").read_text()
 assert "IncludeLaunchDescription" not in text
 for token in ('"suppress":False','"force_invalid":False','"suppress_dynamic":False','"suppress_feedback":False'):
  assert token in text
 assert '("/navigate_to_pose/_action/feedback","/phase4_qualification/p4e6b/navigate_to_pose_feedback")' in text
 assert '"navigate_to_pose_action":"/navigate_to_pose"' in text
 assert "feedback_proxy" not in text
 assert '("/tf","/phase4_qualification/p4e6b/mm_tf")' in text
 assert '("/Odometry","/phase4_qualification/p4e6b/mm_odometry")' in text
 assert '("/system/localization_valid","/phase4_qualification/p4e6b/gate_localization_valid")' in text

def test_no_canonical_output_authority_created_by_tooling():
 root=Path(__file__).parents[1]/"parking_robot_bringup"
 texts="\n".join((root/name).read_text() for name in ("phase4_p4e6b_observation_relay.py","phase4_p4e6b_tf_observation_relay.py","phase4_p4e6b_feedback_relay.py"))
 assert 'create_publisher(TwistStamped,"/vehicle_cmd_safe"' not in texts
 assert 'create_publisher(Twist,"/cmd_vel_nav_raw"' not in texts
 assert 'create_publisher(Twist,"/cmd_vel_nav_safe"' not in texts
 assert 'create_publisher(Odometry,"/Odometry"' not in texts

def test_s01_requires_explicit_injection_ack_and_health():
 ack={"state":"INJECTED","injection_count":1,"injection_monotonic_ns":123}
 validate_s01_injection(ready_before_mission=True,healthy_at_command=True,
                        acknowledgement=ack,first_reason="FEEDBACK_STALE")
 for mutation in ("not_ready","not_healthy","no_ack","bad_count","no_time","wrong_reason"):
  kwargs=dict(ready_before_mission=True,healthy_at_command=True,
              acknowledgement=dict(ack),first_reason="FEEDBACK_STALE")
  if mutation=="not_ready": kwargs["ready_before_mission"]=False
  elif mutation=="not_healthy": kwargs["healthy_at_command"]=False
  elif mutation=="no_ack": kwargs["acknowledgement"]["state"]="READY_FORWARD"
  elif mutation=="bad_count": kwargs["acknowledgement"]["injection_count"]=2
  elif mutation=="no_time": kwargs["acknowledgement"]["injection_monotonic_ns"]=0
  else: kwargs["first_reason"]="GOAL_ABORTED"
  with pytest.raises(RuntimeError): validate_s01_injection(**kwargs)

def test_s01_relay_disappearance_before_ack_is_tool_failure_not_pass():
 with pytest.raises(RuntimeError,match="^P4E6B_S01_TOOL_FAILURE_NEEDS_REVIEW$"):
  validate_s01_injection(ready_before_mission=True,healthy_at_command=False,
                         acknowledgement={},first_reason="FEEDBACK_STALE")

def test_feedback_relay_has_no_action_authority_static():
 root=Path(__file__).parents[1]/"parking_robot_bringup"
 text=(root/"phase4_p4e6b_feedback_relay.py").read_text()
 assert "ActionServer" not in text and "ActionClient" not in text
 assert "publish(message)" in text
 assert "NavigateToPose.Impl.FeedbackMessage" in text
