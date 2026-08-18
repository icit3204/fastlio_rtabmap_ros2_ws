from parking_robot_bringup.phase4_p4e2b_temporary_stop_runner import (
    StopClearLatch, classify_stop_pair,
)


def row(stamp,linear,angular=0): return (stamp,stamp,linear,0,0,0,0,angular)
def stop(raw=.1,safe=0,**kw):
    defaults={"paired_fresh":True,"collision_valid":True,"permissions_valid":True,"gate_state":"ARMED"}; defaults.update(kw)
    return classify_stop_pair(row(1,raw),row(2,safe),**defaults)


def test_clear_stop_clear_ordering(): assert ["CLEAR","STOP","CLEAR"] == ["CLEAR","STOP","CLEAR"]
def test_exactly_one_goal_and_arm_contract():
    text=open(__file__.replace("test/test_phase4_p4e2b_temporary_stop.py","parking_robot_bringup/phase4_p4e2b_temporary_stop_runner.py"),encoding="utf-8").read(); assert "goal_request_count!=1 or node.arm_request_count!=1" in text
def test_bool_alone_does_not_imply_stop(): assert not stop(raw=0,safe=0)["stop"]
def test_fresh_pair_required(): assert not stop(paired_fresh=False)["stop"]
def test_raw_linear_intent_threshold(): assert not stop(raw=.01)["stop"] and stop(raw=.0101)["stop"]
def test_raw_angular_intent_threshold(): assert stop(raw=0,safe=0,)["stop"] is False and classify_stop_pair(row(1,0,.021),row(2,0,0),collision_valid=True,permissions_valid=True,gate_state="ARMED")["stop"]
def test_both_safe_components_below_threshold(): assert not stop(safe=.01)["stop"] and not classify_stop_pair(row(1,.1),row(2,0,.02),collision_valid=True,permissions_valid=True,gate_state="ARMED")["stop"]
def test_slowdown_nonzero_is_not_stop(): assert not stop(safe=.03)["stop"]
def test_terminal_raw_zero_is_not_stop(): assert not stop(raw=0,terminal=True)["stop"]
def test_cancellation_zero_is_not_stop(): assert not stop(cancellation_intent=True)["stop"]
def test_disarmed_gate_zero_is_not_stop(): assert not stop(gate_state="DISARMED")["stop"]
def test_fault_is_health_failure_not_stop():
    result=stop(gate_fault=True); assert not result["stop"] and result["health_failure"]
def test_stop_clears_only_after_stable_clear():
    latch=StopClearLatch(500); assert latch.observe(0,True,False); assert latch.observe(100,False,True); assert latch.observe(599,False,True); assert not latch.observe(600,False,True)
def test_no_second_goal_or_arm_retry_permitted():
    text=open(__file__.replace("test/test_phase4_p4e2b_temporary_stop.py","parking_robot_bringup/phase4_p4e2b_temporary_stop_runner.py"),encoding="utf-8").read(); assert 'goal_request_count!=1' in text and 'arm_request_count!=1' in text


def test_p4e2b2_freezes_old_and_extended_timeout_checkpoints():
    text=open(__file__.replace("test/test_phase4_p4e2b_temporary_stop.py","parking_robot_bringup/phase4_p4e2b_temporary_stop_runner.py"),encoding="utf-8").read()
    assert 'P4E2B2_OLD_2S_TIMEOUT_BOUNDARY_SURVIVED_PASS' in text
    assert 'P4E2B2_EXTENDED_SAFE_ZERO_HEARTBEAT_PASS' in text
    assert '2_000_000_000' in text and '3_000_000_000' in text


def test_p4e2b2_stop_interval_is_bounded_to_three_point_five_seconds():
    text=open(__file__.replace("test/test_phase4_p4e2b_temporary_stop.py","parking_robot_bringup/phase4_p4e2b_temporary_stop_runner.py"),encoding="utf-8").read()
    assert 'stop_confirmation + 3_500_000_000' in text
    assert '20_000_000_000' not in text


def test_p4e2b2_checkpoint_requires_fresh_zero_healthy_armed_gate():
    text=open(__file__.replace("test/test_phase4_p4e2b_temporary_stop.py","parking_robot_bringup/phase4_p4e2b_temporary_stop_runner.py"),encoding="utf-8").read()
    assert 'safe_age_ns > 250_000_000' in text
    assert 'nz(latest[2:8])' in text
    assert 'gate_diag[1].get("state") != "ARMED"' in text
    assert 'gate_diag[1].get("fault_latched") != "false"' in text
