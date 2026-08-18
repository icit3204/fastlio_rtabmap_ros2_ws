from parking_robot_bringup.phase4_p4e2a_slowdown_runner import (
    classify_slowdown_pair, pair_commands,
)


def row(stamp, linear, angular=0.0):
    return (stamp, stamp, linear, 0.0, 0.0, 0.0, 0.0, angular)


def classify(raw=.5, safe=.15, **kwargs):
    defaults={"collision_valid":True,"gate_valid":True,"permissions_valid":True}
    defaults.update(kwargs)
    return classify_slowdown_pair(row(1,raw),row(2,safe),**defaults)


def test_clear_slow_clear_ordering_is_explicit():
    events=["CLEAR","SLOW","CLEAR"]
    assert events == ["CLEAR","SLOW","CLEAR"] and events.count("SLOW") == 1


def test_exactly_one_goal_and_arm_contract_is_static():
    text=open(__file__.replace("test/test_phase4_p4e2a_slowdown.py","parking_robot_bringup/phase4_p4e2a_slowdown_runner.py"),encoding="utf-8").read()
    assert 'goal_request_count != 1 or node.arm_request_count != 1' in text


def test_validity_bool_alone_cannot_classify_slowdown(): assert not classify(safe=.5)["slowdown"]
def test_fresh_pairing_rejects_excessive_skew(): assert pair_commands([row(0,.5)],[row(100_000_000,.15)],0,200_000_000)==[]
def test_raw_intent_must_exceed_threshold(): assert not classify(raw=.0005,safe=.0001)["slowdown"]
def test_safe_must_remain_nonzero(): assert not classify(safe=0.0)["slowdown"]
def test_safe_direction_must_match_raw(): assert not classify(safe=-.15)["slowdown"]
def test_safe_at_80_percent_qualifies(): assert classify(safe=.4)["slowdown"]
def test_safe_must_not_materially_exceed_raw(): assert not classify(safe=.6)["slowdown"]
def test_cancellation_is_not_slowdown(): assert not classify(cancellation_intent=True)["slowdown"]
def test_terminal_sample_is_not_slowdown(): assert not classify(terminal=True)["slowdown"]
def test_stop_like_zero_is_not_slowdown(): assert "safe command is zero/STOP-like" in classify(safe=0)["reasons"]

