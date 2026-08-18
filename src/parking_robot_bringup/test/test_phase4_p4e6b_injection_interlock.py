from parking_robot_bringup.phase4_p4e6b_injection_interlock import InjectionInterlock

def test_default_denies():
    x=InjectionInterlock(); d=x.request(parameter="force_invalid",value=True)
    assert not d.accepted and not d.forwarded and x.forward_count == 0

def test_shadow_denies_and_counts_request():
    x=InjectionInterlock(mode="SHADOW_DENY"); d=x.request(parameter="force_invalid",value=True)
    assert d.reason == "SHADOW_DENY" and x.request_count == 1 and x.forward_count == 0

def test_wrong_parameter_denies():
    x=InjectionInterlock(mode="LIVE_SINGLE_SHOT",authority_token="t",expected_token="t")
    assert not x.request(parameter="other",value=True).accepted

def test_wrong_value_denies():
    x=InjectionInterlock(mode="LIVE_SINGLE_SHOT",authority_token="t",expected_token="t")
    assert not x.request(parameter="force_invalid",value=False).accepted

def test_missing_token_denies():
    x=InjectionInterlock(mode="LIVE_SINGLE_SHOT",expected_token="t")
    assert not x.request(parameter="force_invalid",value=True).accepted

def test_wrong_token_denies():
    x=InjectionInterlock(mode="LIVE_SINGLE_SHOT",authority_token="bad",expected_token="t")
    assert not x.request(parameter="force_invalid",value=True).accepted

def test_live_single_shot_forwards_once():
    x=InjectionInterlock(mode="LIVE_SINGLE_SHOT",authority_token="t",expected_token="t")
    assert x.request(parameter="force_invalid",value=True).forwarded
    assert not x.request(parameter="force_invalid",value=True).forwarded
    assert x.forward_count == 1

def test_shadow_duplicate_requests_all_denied():
    x=InjectionInterlock(mode="SHADOW_DENY")
    assert not x.request(parameter="force_invalid",value=True).forwarded
    assert not x.request(parameter="force_invalid",value=True).forwarded
    assert x.request_count == 2 and x.forward_count == 0

def test_invalid_mode_defaults_to_deny():
    x=InjectionInterlock(mode="LIVE_MAYBE")
    assert x.mode == "SHADOW_DENY"

def test_missing_mode_defaults_to_deny():
    x=InjectionInterlock(mode=None)
    assert x.mode == "SHADOW_DENY"

def test_forward_marker_precedes_effectful_decision_record():
    x=InjectionInterlock(mode="LIVE_SINGLE_SHOT",authority_token="t",expected_token="t")
    d=x.request(parameter="force_invalid",value=True)
    assert d.forwarded and d.forward_count == 1

def test_new_process_state_starts_denied():
    x=InjectionInterlock()
    assert not x.request(parameter="force_invalid",value=True).forwarded
