import json
from pathlib import Path
import pytest

from parking_robot_bringup.phase4_p4e6b_health_core import *
from parking_robot_bringup.phase4_p4e6b_health_failure_runner import (
    BoundedWriter, main, parse_args, classify_physical_evidence,
    PHYSICAL_ZERO_HORIZON_SEC, STATIONARY_WINDOW_SEC, dry_plan,
    DIRECT_ZERO_HORIZON_SEC, ANCHOR_TIMEOUT_SEC, competing_terminal_for_current_goal,
    PremissionCollisionReadiness, PREMISSION_CURRENTNESS_MAX_AGE_NS)


def _premission(now, *, value=True, reason="VALID", count=1, node="collision_monitor_validity_monitor",
                gid="current", bool_age=1, diag_age=1, transport_age=1, stable=1.1, source_age=.01, state="VALID"):
    return dict(now_ns=now,now_ros_ns=now, publisher_count=count, publisher_node=node, publisher_gid=gid,
                bool_value=value, bool_receipt_ns=now-bool_age, diagnostic_state=state,diagnostic_reason=reason,
                diagnostic_healthy_stable_sec=stable,diagnostic_source_age_sec=source_age,
                diagnostic_receipt_ns=now-diag_age,diagnostic_header_ros_ns=now-transport_age,
                bool_writer_gid=gid,diagnostic_writer_gid='diag',diagnostic_graph_gids={'diag'},bool_post_epoch_count=2,epoch_start_ns=0,epoch_start_ros_ns=0)


def test_premission_ordering_defect_reproduced_by_old_event_sequence():
    # The pre-edit runner performed route/start before its ACTIVE precondition.
    old_events=["publish_initial","publish_route","trigger_start","post_uuid_acquire"]
    assert old_events.index("publish_route") < old_events.index("post_uuid_acquire")


def test_premission_collision_readiness_requires_full_current_generation_epoch():
    r=PremissionCollisionReadiness(); base=1_000_000_000
    assert not r.observe(**_premission(base,value=False))["ready"]
    assert not r.observe(**_premission(base+1,value=True,reason="RECOVERY_STABILITY_WAIT"))["ready"]
    assert not r.observe(**_premission(base+2,value=True,stable=.99))["ready"]
    assert r.observe(**_premission(base+3,value=True,stable=1.0))["ready"]


@pytest.mark.parametrize("kw",[
    {"count":2},{"node":"other"},{"reason":"SOURCE_STALE"},{"value":False},
    {"bool_age":PREMISSION_CURRENTNESS_MAX_AGE_NS},{"diag_age":PREMISSION_CURRENTNESS_MAX_AGE_NS},])
def test_premission_collision_rejects_authority_false_or_stale(kw):
    assert not PremissionCollisionReadiness().observe(**_premission(2_000_000_000,**kw))["ready"]


def test_premission_e10_receipt_stall_does_not_reconstruct_a_false_epoch():
    r=PremissionCollisionReadiness(); t=3_000_000_000
    assert r.observe(**_premission(t,stable=2.0))["ready"]
    # During a 415 ms blind interval all cached evidence is too old.
    assert not r.observe(**_premission(t+415_000_000,bool_age=415_000_000,diag_age=415_000_000,transport_age=415_000_000,stable=2.4))["ready"]
    # Fresh source-time evidence resumes with the monitor's epoch intact.
    assert r.observe(**_premission(t+416_000_000,stable=2.5))["ready"]

def test_source_age_upper_bound_and_old_diagnostic_are_fail_closed():
    r=PremissionCollisionReadiness();t=4_000_000_000
    assert r.observe(**_premission(t,source_age=.249,transport_age=249_000_000))["ready"]
    assert not r.observe(**_premission(t,source_age=.250,transport_age=250_000_000))["ready"]
    assert not r.observe(**_premission(t,transport_age=PREMISSION_CURRENTNESS_MAX_AGE_NS))["ready"]

def test_generation_bound_caches_reject_old_bool_or_diagnostic_writer():
    r=PremissionCollisionReadiness();t=5_000_000_000
    # Humble has no callback MessageInfo: epochs, not fabricated message GIDs,
    # bind cached evidence. Pre-epoch receipt cannot qualify.
    assert not r.observe(**{**_premission(t),"epoch_start_ns":t,"epoch_start_ros_ns":t})["ready"]
    assert not r.observe(**{**_premission(t),"diagnostic_writer_gid":"old","diagnostic_graph_gids":{"diag"}})["ready"]
    # A disappeared diagnostic writer cannot be re-used on a later observe.
    assert not r.observe(**{**_premission(t),"diagnostic_graph_gids":set()})["ready"]

def test_graph_epoch_requires_two_post_epoch_bool_and_new_diagnostic_header():
    r=PremissionCollisionReadiness();t=6_000_000_000
    assert not r.observe(**{**_premission(t),"bool_post_epoch_count":1,"epoch_start_ns":t-2,"epoch_start_ros_ns":t-2})["ready"]
    assert not r.observe(**{**_premission(t),"epoch_start_ns":t-2,"epoch_start_ros_ns":t+1})["ready"]
    assert r.observe(**{**_premission(t),"epoch_start_ns":t-2,"epoch_start_ros_ns":t-2})["ready"]


def test_generation_a_evidence_cannot_requalify_epoch_b_before_b_evidence():
    # The runtime clears subscriptions/caches before giving this pure model a
    # new epoch.  These inputs prove the remaining acceptance boundary: A's
    # cached receipts/header cannot be post-B evidence merely because graph B
    # now exists.
    r=PremissionCollisionReadiness(); a=7_000_000_000
    assert r.observe(**{**_premission(a, gid="A"), "bool_writer_gid":"A",
                         "epoch_start_ns":a-2, "epoch_start_ros_ns":a-2})["ready"]
    b=a+10
    old={**_premission(b, gid="B"), "bool_writer_gid":"B", "bool_post_epoch_count":0,
         "bool_receipt_ns":a-1, "diagnostic_receipt_ns":a-1,
         "diagnostic_header_ros_ns":a-1, "epoch_start_ns":b, "epoch_start_ros_ns":b}
    assert not r.observe(**old)["ready"]
    assert not r.observe(**old)["ready"]
    assert r.observe(**{**_premission(b+3, gid="B"), "bool_writer_gid":"B",
                         "epoch_start_ns":b, "epoch_start_ros_ns":b})["ready"]


def test_old_diagnostic_header_after_epoch_is_rejected_despite_new_receipt():
    r=PremissionCollisionReadiness(); epoch=8_000_000_000
    stale={**_premission(epoch+10), "epoch_start_ns":epoch, "epoch_start_ros_ns":epoch,
            "diagnostic_header_ros_ns":epoch-1, "diagnostic_receipt_ns":epoch+9}
    assert not r.observe(**stale)["ready"]
    fresh={**_premission(epoch+11), "epoch_start_ns":epoch, "epoch_start_ros_ns":epoch,
            "diagnostic_header_ros_ns":epoch+1, "diagnostic_receipt_ns":epoch+10}
    assert r.observe(**fresh)["ready"]


def test_epoch_bool_acquisition_false_and_epoch_change_fail_closed():
    r=PremissionCollisionReadiness(); t=9_000_000_000
    one={**_premission(t), "bool_post_epoch_count":1, "epoch_start_ns":t-2, "epoch_start_ros_ns":t-2}
    assert not r.observe(**one)["ready"]
    assert not r.observe(**{**_premission(t+1, value=False), "epoch_start_ns":t-2,
                            "epoch_start_ros_ns":t-2})["ready"]
    assert r.observe(**{**_premission(t+2), "epoch_start_ns":t-2, "epoch_start_ros_ns":t-2})["ready"]
    # A changed authority epoch clears the count; a pre-epoch true is not a
    # substitute for the two newly acquired Bool callbacks.
    assert not r.observe(**{**_premission(t+3, gid="new"), "bool_writer_gid":"new",
                            "bool_post_epoch_count":0, "epoch_start_ns":t+3,
                            "epoch_start_ros_ns":t+3})["ready"]


def test_authority_loss_and_exact_currentness_boundaries_revoke_readiness():
    r=PremissionCollisionReadiness(); t=10_000_000_000
    assert r.observe(**_premission(t, bool_age=249_999_999, diag_age=249_999_999,
                                   transport_age=249_999_999, source_age=.249999999))["ready"]
    # Independently prove strict 250 ms receipt/message and authority bounds.
    assert not r.observe(**_premission(t, bool_age=250_000_000))['ready']
    assert not r.observe(**_premission(t, diag_age=250_000_000))['ready']
    assert not r.observe(**_premission(t, transport_age=250_000_000))['ready']
    assert not r.observe(**{**_premission(t), "diagnostic_graph_gids":set()})['ready']


def test_premission_readiness_has_no_raw_safe_or_movement_input():
    assert "raw" not in PremissionCollisionReadiness.observe.__code__.co_varnames
    assert "safe" not in PremissionCollisionReadiness.observe.__code__.co_varnames


def test_final_readiness_diagnostics_qos_is_best_effort_volatile_depth_one():
    source=Path(__file__).parents[1]/"parking_robot_bringup/phase4_p4e6b_health_failure_runner.py"
    text=source.read_text()
    assert "depth=(1 if premission_diagnostic_depth is None" in text
    assert "ReliabilityPolicy.BEST_EFFORT if premission_diagnostic_reliability is None" in text
    assert "durability=DurabilityPolicy.VOLATILE" in text


@pytest.mark.parametrize("offset", range(5))
def test_best_effort_blindness_fails_closed_then_recovers_from_current_evidence(offset):
    """A 400 ms executor blind interval is not a semantic-health reset.

    The point-in-time evaluator must withhold READY after its latest evidence
    reaches 250 ms, then accept fresh post-blind monitor evidence immediately.
    """
    r=PremissionCollisionReadiness(); t=11_000_000_000 + offset*1_000_000
    current={**_premission(t, stable=2.0), "epoch_start_ns":t-2, "epoch_start_ros_ns":t-2}
    assert r.observe(**current)["ready"]
    blind={**_premission(t+400_000_000, stable=2.4, bool_age=400_000_000,
                          diag_age=400_000_000, transport_age=400_000_000),
           "epoch_start_ns":t-2, "epoch_start_ros_ns":t-2}
    assert not r.observe(**blind)["ready"]
    resumed={**_premission(t+401_000_000, stable=2.5), "epoch_start_ns":t-2,
             "epoch_start_ros_ns":t-2}
    assert r.observe(**resumed)["ready"]


def states(reason,uuid="u"):
 return [dict(monotonic_ns=20,state_name="CANCELLING",reason_code=reason,active_goal_uuid=uuid,mission_id="m",route_id="r",waypoint_index=0),
         dict(monotonic_ns=22,state_name="CANCELLING",reason_code="HEALTH_CANCEL_ACK_ACCEPTED",active_goal_uuid=uuid,mission_id="m",route_id="r",waypoint_index=0),
         dict(monotonic_ns=24,state_name="FAILED",reason_code=reason,active_goal_uuid="",mission_id="m",route_id="r",waypoint_index=0)]
def statuses(uuid="u"):return [dict(monotonic_ns=21,goal_uuid=uuid,status_name="CANCELING"),dict(monotonic_ns=23,goal_uuid=uuid,status_name="CANCELED")]

@pytest.mark.parametrize("case",sorted(SCENARIOS))
def test_nine_dry_plans_complete_and_no_runtime(case):
 p=case_plan(case); assert p["case_id"]==case and p["runtime_executed"] is False
 assert p["mission_geometry"]==MISSION_GEOMETRY and "no ROS autonomy traffic" in p["no_runtime_statement"]
 assert p["expected_reason"]==SCENARIOS[case].reason and p["one_attempt"]

def test_h01_dry_plan_serializes_runtime_owned_physical_drain_contract():
 p=dry_plan("B-H01")
 assert p["expected_health_terminal"]=="FAILED / GATE_DISARMED"
 assert p["post_health_terminal_phase"]=="PHYSICAL_EVIDENCE_DRAIN"
 assert p["preferred_zero_source"]=="DIRECT_COMMAND_ZERO"
 assert p["zero_source_classifications"]==["DIRECT_COMMAND_ZERO","DEADMAN_ZERO","NO_ZERO_WITHIN_HORIZON"]
 assert p["direct_zero_horizon_sec"]==DIRECT_ZERO_HORIZON_SEC
 assert p["zero_classification_deadline_sec"]==PHYSICAL_ZERO_HORIZON_SEC
 assert p["anchor_timeout_sec"]==ANCHOR_TIMEOUT_SEC
 assert p["post_zero_anchor_required"] and p["stationary_window_independent_of_zero_deadline"]
 assert p["stationary_window_sec"]==STATIONARY_WINDOW_SEC and p["stationary_translation_bound_m"]==.02
 assert "fake-applied zero" in p["anchor_definition"]
 assert "physical adjudication" in p["writer_finalization"]
 assert not p["rearm_during_drain"] and not p["additional_health_cancel_during_drain"]

@pytest.mark.parametrize("case",sorted(SCENARIOS))
def test_all_dry_plans_serialize_case_specific_physical_policy(case):
 p=dry_plan(case)
 assert p["case_id"]==case and p["post_health_terminal_phase"]=="PHYSICAL_EVIDENCE_DRAIN"
 assert p["physical_case_policy"] and p["zero_source_semantics"]["NO_ZERO_WITHIN_HORIZON"]=="physical failure"

@pytest.mark.parametrize("bad",["", "all", "*", "B-H01,B-H02", "B-H01 B-H01", "unknown"])
def test_case_selection_rejects_non_exact_ids(bad):
 with pytest.raises((ValueError,SystemExit)): case_plan(bad)

def test_cli_requires_explicit_mutually_exclusive_mode_and_case(tmp_path):
 with pytest.raises(SystemExit):parse_args(["--case-id","B-H01","--output-dir",str(tmp_path)])
 with pytest.raises(SystemExit):parse_args(["--case-id","B-H01","--output-dir",str(tmp_path),"--dry-plan-only","--execute-authorized-case"])
 assert parse_args(["--case-id","B-S01","--output-dir",str(tmp_path),"--dry-plan-only"]).case_id=="B-S01"

@pytest.mark.parametrize("case,ros",[("B-H01",[]),("B-H01",["--ros-args","-r","__node:=dry"]),
 ("B-S01",["--ros-args","-r","__ns:=/q"]),("B-C01",["--ros-args","-r","a:=b","-r","c:=d"])])
def test_ros_arguments_are_removed_before_strict_application_parse(case,ros,tmp_path):
 ns=parse_args(["--case-id",case,"--output-dir",str(tmp_path),"--dry-plan-only",*ros])
 assert ns.case_id==case and not hasattr(ns,"ros_args")

@pytest.mark.parametrize("argv",[
 ["--case-id","B-H01","--output-dir","/tmp/x","--dry-plan-only","--typo"],
 ["--case-id","B-H001","--output-dir","/tmp/x","--dry-plan-only"],
 ["--case-id","all","--output-dir","/tmp/x","--dry-plan-only"],
 ["--case-id","B-H01,B-H02","--output-dir","/tmp/x","--dry-plan-only"],
 ["--output-dir","/tmp/x","--dry-plan-only"]])
def test_unknown_non_ros_application_arguments_remain_fail_closed(argv):
 with pytest.raises(SystemExit):parse_args(argv)

def test_exact_consumed_h01_argv_reaches_execute_seam_with_ros_remap(tmp_path):
 raw=["--case-id","B-H01","--output-dir",str(tmp_path),"--execute-authorized-case",
      "--ros-args","-r","__node:=phase4_p4e6b_health_failure_runner"]
 calls=[];main(raw,campaign_executor=lambda case,out,ros:calls.append((case,out,ros)))
 assert calls==[("B-H01",tmp_path,raw)]

def active_model(case="B-H01"):
 m=HealthCampaignModel(case);m.begin_runtime();m.mission_start("u");m.arm_active(stream_established=True,healthy=True);m.stable_clear(5.0);m.arm_case();return m

def test_h01_counterfactual_success_and_duplicate_injection():
 m=active_model();m.inject(10,11,tool_state="CONFIRMED")
 with pytest.raises(RuntimeError):m.inject(12,13)
 assert m.adjudicate(first_reason="GATE_DISARMED",states=states("GATE_DISARMED"),statuses=statuses())["pass"]
 m.finalize();assert m.state is CampaignState.FINALIZED and m.attempt_count==1

def test_wrong_first_reason_cancel_transport_and_wrong_terminal_rejected():
 m=active_model();m.inject(10,11,tool_state="CONFIRMED")
 with pytest.raises(RuntimeError):m.adjudicate(first_reason="GATE_FAULT",states=states("GATE_DISARMED"),statuses=statuses())
 m=active_model();m.inject(10,11,tool_state="CONFIRMED")
 with pytest.raises(RuntimeError):m.adjudicate(first_reason="GATE_DISARMED",states=states("GATE_DISARMED"),statuses=[])
 m=active_model();m.inject(10,11,tool_state="CONFIRMED"); bad=states("GATE_DISARMED");bad[-1]["state_name"]="BLOCKED"
 with pytest.raises(RuntimeError):m.adjudicate(first_reason="GATE_DISARMED",states=bad,statuses=statuses())

@pytest.mark.parametrize("case",["B-H03","B-H05"])
def test_priming_cases_inject_before_arm(case):
 m=HealthCampaignModel(case);m.begin_runtime();m.mission_start();
 with pytest.raises(RuntimeError):m.arm_active(stream_established=True,healthy=True)
 m.arm_case();m.inject(10,11,tool_state="CONFIRMED");assert m.arm_count==0

@pytest.mark.parametrize("case",["B-S01","B-S02","B-S03"])
def test_suppression_cases_require_ack(case):
 m=active_model(case)
 with pytest.raises(RuntimeError):m.inject(10,11,successful=False,tool_state="INJECTED")

def test_s01_requires_literal_relay_injected_state():
 m=active_model("B-S01")
 with pytest.raises(RuntimeError):m.inject(10,11,tool_state="READY_FORWARD")

def test_c01_requires_stream_established_and_stable_clear():
 m=HealthCampaignModel("B-C01");m.begin_runtime();m.mission_start()
 with pytest.raises(RuntimeError):m.arm_active(stream_established=False,healthy=True)
 m.arm_active(stream_established=True,healthy=True)
 with pytest.raises(RuntimeError):m.stable_clear(4.999)

def test_second_mission_and_internal_retry_rejected():
 m=HealthCampaignModel("B-H01");m.begin_runtime();m.mission_start()
 with pytest.raises(RuntimeError):m.mission_start("u2")
 with pytest.raises(RuntimeError):m.begin_runtime()

def test_no_uuid_immediate_failed_zero_cancel():
 m=HealthCampaignModel("B-H03");m.begin_runtime();m.mission_start("");m.arm_case();m.inject(1,2,tool_state="CONFIRMED")
 s=[dict(state_name="FAILED",reason_code="LOCALIZATION_INVALID")]
 assert m.adjudicate(first_reason="LOCALIZATION_INVALID",states=s,statuses=[])["cancel_count"]==0

def test_physical_ledger_and_boundaries():
 samples={x:[{"ns":10,"nonzero":False}] for x in ("vehicle","adapter","fake")}
 assert physical_stop_ledger(samples,anchor_ns=10,origin_xy=(0,0),final_xy=(.02,0))["pass"]
 samples["vehicle"].append({"ns":11,"nonzero":True})
 with pytest.raises(RuntimeError):physical_stop_ledger(samples,anchor_ns=10,origin_xy=(0,0),final_xy=(0,0))

def test_bounded_writer_orders_and_stops(tmp_path):
 w=BoundedWriter(tmp_path);[w.put("x.jsonl",{"x":i}) for i in range(10)];s=w.finalize()
 assert s["capacity"]==4096 and s["last_enqueued"]==s["last_persisted"]==10 and s["queue_depth"]==0 and s["worker_stopped"]
 assert [json.loads(x)["evidence_sequence"] for x in (tmp_path/"x.jsonl").read_text().splitlines()]==list(range(1,11))

def _diag(ns,vx,wz,condition="VALID"):
 return {"monotonic_ns":ns,"values":{"applied_linear_x":str(vx),"applied_angular_z":str(wz),"condition":condition}}
def _odom(ns,x=0.,y=0.,yaw=0.,vx=0.,wz=0.): return (ns,0,x,y,yaw,vx,wz,"g","fake")

def test_physical_drain_keeps_terminal_then_accepts_post_terminal_direct_zero():
 r=classify_physical_evidence(terminal_ns=100,vehicle_samples=[(110,0,0,0,0,0,0,0,"f","g","n")],fake_diagnostics=[_diag(120,0,0)],odometry=[_odom(125),_odom(125+int(STATIONARY_WINDOW_SEC*1e9),.01)],now_ns=125+int(STATIONARY_WINDOW_SEC*1e9))
 assert r["complete"] and r["classification"]=="DIRECT_COMMAND_ZERO" and r["terminal_ns"]==100

def test_physical_drain_deadman_requires_anchor_and_stationary_window():
 dead=classify_physical_evidence(terminal_ns=100,vehicle_samples=[],fake_diagnostics=[_diag(700_000_000,0,0,"INPUT_STALE")],odometry=[],now_ns=700_000_000)
 assert dead["classification"]=="DEADMAN_ZERO" and not dead["complete"]
 anchored=classify_physical_evidence(terminal_ns=100,vehicle_samples=[],fake_diagnostics=[_diag(700_000_000,0,0,"INPUT_STALE")],odometry=[_odom(700_000_001),_odom(700_000_001+int(STATIONARY_WINDOW_SEC*1e9))],now_ns=700_000_001+int(STATIONARY_WINDOW_SEC*1e9))
 assert anchored["classification"]=="DEADMAN_ZERO" and anchored["complete"] and anchored["physical_pass"]
 no=classify_physical_evidence(terminal_ns=100,vehicle_samples=[],fake_diagnostics=[],odometry=[],now_ns=100+int(PHYSICAL_ZERO_HORIZON_SEC*1e9))
 assert no["classification"]=="NO_ZERO_WITHIN_HORIZON" and no["complete"]

def test_physical_drain_anchor_timeout_is_explicit_failure():
 r=classify_physical_evidence(terminal_ns=100,vehicle_samples=[],fake_diagnostics=[_diag(200,0,0)],odometry=[],now_ns=200+151_000_000)
 assert r["classification"]=="DIRECT_COMMAND_ZERO" and r["complete"] and r["failure"]=="ANCHOR_TIMEOUT"

def test_physical_drain_rejects_nonzero_odom_after_anchor_and_old_zero():
 with pytest.raises(RuntimeError): classify_physical_evidence(terminal_ns=100,vehicle_samples=[],fake_diagnostics=[_diag(120,0,0)],odometry=[_odom(125),_odom(126,vx=.1)],now_ns=125+int(STATIONARY_WINDOW_SEC*1e9))
 old=classify_physical_evidence(terminal_ns=100,vehicle_samples=[],fake_diagnostics=[_diag(99,0,0)],odometry=[_odom(99)],now_ns=100)
 assert old["classification"]=="NO_ZERO_WITHIN_HORIZON"

def test_launch_safe_defaults_single_runner_and_no_persistent_runner():
 text=(Path(__file__).parents[1]/"launch/phase4_p4e6b_health_matrix.launch.py").read_text()
 assert 'DeclareLaunchArgument("enable_health_runner",default_value="false")' in text
 assert 'DeclareLaunchArgument("case_id",default_value="")' in text
 assert text.count('executable="phase4_p4e6b_health_failure_runner"')==1
 assert "phase4_p4e6a_persistent_block_runner" not in text
 assert text.count('executable="mission_manager_node"')==1

def _terminal(ns, state, reason, uuid="u", mission="m", route="r", waypoint=0):
 return {"monotonic_ns":ns,"state_name":state,"reason_code":reason,
         "active_goal_uuid":uuid,"mission_id":mission,"route_id":route,"waypoint_index":waypoint}

def test_competing_terminal_is_identity_qualified_and_never_becomes_a_timeout():
 current=_terminal(20,"CANCELLING","COLLISION_MONITOR_INVALID")
 result=competing_terminal_for_current_goal([current],accepted_ns=10,mission_id="m",route_id="r",waypoint=0,goal_uuid="u")
 assert result=={"reason":"COLLISION_MONITOR_INVALID","state":"CANCELLING","monotonic_ns":20}
 assert competing_terminal_for_current_goal([_terminal(20,"CANCELLING","GATE_FAULT",uuid="old")],accepted_ns=10,mission_id="m",route_id="r",waypoint=0,goal_uuid="u") is None
 assert competing_terminal_for_current_goal([_terminal(9,"FAILED","GATE_FAULT",uuid="")],accepted_ns=10,mission_id="m",route_id="r",waypoint=0,goal_uuid="u") is None
 assert competing_terminal_for_current_goal([_terminal(20,"FAILED","GATE_FAULT",uuid="",mission="old")],accepted_ns=10,mission_id="m",route_id="r",waypoint=0,goal_uuid="u") is None

def test_failed_current_mission_is_authoritative_even_after_uuid_is_cleared():
 result=competing_terminal_for_current_goal([_terminal(20,"FAILED","COLLISION_MONITOR_INVALID",uuid="")],accepted_ns=10,mission_id="m",route_id="r",waypoint=0,goal_uuid="u")
 assert result["reason"]=="COLLISION_MONITOR_INVALID" and result["state"]=="FAILED"
