from mkmini_cmd_adapter import (
    ContainedContinuityContext,
    ContainedContinuityReason as Reason,
    ContainedSingleFrameLossPolicy,
    classify_stationary_one_count_jitter,
    classify_stationary_encoder_one_count_dither,
    classify_stationary_encoder_boundary_dither,
)


def healthy(**overrides):
    values = dict(
        can_state="ERROR-ACTIVE",
        can_error_count=0,
        socket_overflow_count=0,
        ctrl_current=True,
        diagnostic_current=True,
        opposite_wheel_current=True,
        hard_diagnostic=False,
        command_inside_contained_envelope=True,
        unexpected_overspeed=False,
        contradictory_pulse_speed=False,
    )
    values.update(overrides)
    return ContainedContinuityContext(**values)


def start(policy, alive=8, timestamp=1.0):
    return policy.observe("right", alive, timestamp, checksum_valid=True, context=healthy())


def test_normal_8_to_9_is_accepted():
    policy = ContainedSingleFrameLossPolicy()
    start(policy)
    result = policy.observe("right", 9, 1.010, checksum_valid=True, context=healthy())
    assert result.accepted and result.reason is Reason.NORMAL_CONTINUITY and not result.warning


def test_isolated_8_to_10_within_35ms_is_explicit_warning():
    policy = ContainedSingleFrameLossPolicy()
    start(policy)
    result = policy.observe("right", 10, 1.025438, checksum_valid=True, context=healthy())
    assert result.accepted and result.reason is Reason.TIMING_COHERENT_OMISSION_WARNING
    assert result.warning and result.alive_delta == 2


def test_30ms_delta3_is_timing_coherent_warning():
    policy = ContainedSingleFrameLossPolicy()
    start(policy)
    result = policy.observe("right", 11, 1.030, checksum_valid=True, context=healthy())
    assert result.accepted and result.warning
    assert result.reason is Reason.TIMING_COHERENT_OMISSION_WARNING


def test_repeated_isolated_omissions_inside_one_second_are_warnings():
    policy = ContainedSingleFrameLossPolicy()
    start(policy, alive=3, timestamp=1.0)
    for alive, timestamp in ((5, 1.020), (7, 1.040), (9, 1.060)):
        result = policy.observe("right", alive, timestamp, checksum_valid=True, context=healthy())
        assert result.accepted
        assert result.warning
        assert result.reason is Reason.TIMING_COHERENT_OMISSION_WARNING


def test_more_than_35ms_is_stale_abort():
    policy = ContainedSingleFrameLossPolicy()
    start(policy)
    result = policy.observe("right", 9, 1.036, checksum_valid=True, context=healthy())
    assert not result.accepted and result.reason is Reason.STALE_FEEDBACK


def test_checksum_failure_aborts():
    policy = ContainedSingleFrameLossPolicy()
    start(policy)
    result = policy.observe("right", 9, 1.010, checksum_valid=False, context=healthy())
    assert not result.accepted and result.reason is Reason.CHECKSUM_INVALID


def test_can_degradation_or_overflow_aborts():
    for context in (healthy(can_state="ERROR-PASSIVE"), healthy(can_error_count=1), healthy(socket_overflow_count=1)):
        policy = ContainedSingleFrameLossPolicy()
        start(policy)
        result = policy.observe("right", 9, 1.010, checksum_valid=True, context=context)
        assert not result.accepted and result.reason is Reason.CAN_DEGRADED


def test_other_channel_stale_aborts():
    cases = (
        (healthy(ctrl_current=False), Reason.CTRL_FEEDBACK_STALE),
        (healthy(diagnostic_current=False), Reason.DIAGNOSTIC_FEEDBACK_STALE),
        (healthy(opposite_wheel_current=False), Reason.OPPOSITE_WHEEL_FEEDBACK_STALE),
    )
    for context, reason in cases:
        policy = ContainedSingleFrameLossPolicy()
        start(policy)
        result = policy.observe("right", 9, 1.010, checksum_valid=True, context=context)
        assert not result.accepted and result.reason is reason


def test_out_of_order_duplicate_aborts():
    policy = ContainedSingleFrameLossPolicy()
    start(policy)
    result = policy.observe("right", 8, 1.010, checksum_valid=True, context=healthy())
    assert not result.accepted and result.reason is Reason.OUT_OF_ORDER


def test_10ms_delta8_is_inconsistent_abort():
    policy = ContainedSingleFrameLossPolicy()
    start(policy, alive=3, timestamp=1.0)
    result = policy.observe("right", 11, 1.010, checksum_valid=True, context=healthy())
    assert not result.accepted and result.reason is Reason.ALIVE_SEQUENCE_INCONSISTENT


def test_current_frame_age_over_35ms_aborts_even_with_coherent_delta():
    policy = ContainedSingleFrameLossPolicy()
    start(policy, alive=3, timestamp=1.0)
    result = policy.observe(
        "right", 5, 1.020, checksum_valid=True, context=healthy(current_frame_age_sec=0.036)
    )
    assert not result.accepted and result.reason is Reason.STALE_FEEDBACK


def test_policy_is_explicitly_contained_and_does_not_modify_production_health():
    import mkmini_cmd_adapter.contained_continuity as module
    assert "production" not in ContainedSingleFrameLossPolicy.__dict__
    assert not hasattr(module, "KeyboardCommissioningCore")


def test_one_count_zero_net_stationary_bounce_is_warning():
    result = classify_stationary_one_count_jitter(
        speed_samples_mps=(0.0, 0.005, 0.0),
        pulse_samples=(-15, -16, -15),
        odometer_samples=(0.077, 0.077, 0.077),
        can_state="ERROR-ACTIVE",
        can_error_count=0,
        visible_wheel_rotation=False,
        hard_diagnostic=False,
    )
    assert result.accepted
    assert result.reason == "STATIONARY_ONE_COUNT_JITTER_WARNING"


def test_stationary_jitter_classifier_rejects_persistent_or_hazardous_evidence():
    common = dict(
        speed_samples_mps=(0.0, 0.005, 0.005),
        odometer_samples=(0.077, 0.077, 0.077),
        can_state="ERROR-ACTIVE",
        can_error_count=0,
        visible_wheel_rotation=False,
        hard_diagnostic=False,
    )
    for pulse_samples, overrides in (
        ((-15, -14, -13), {}),
        ((-15, -17, -15), {}),
        ((-15, -16, -15), {"odometer_samples": (0.077, 0.078)}),
        ((-15, -16, -15), {"visible_wheel_rotation": True}),
        ((-15, -16, -15), {"can_state": "ERROR-PASSIVE"}),
        ((-15, -16, -15), {"hard_diagnostic": True}),
    ):
        values = dict(common)
        values["pulse_samples"] = pulse_samples
        values.update(overrides)
        assert not classify_stationary_one_count_jitter(**values).accepted


def dither(**overrides):
    values = dict(
        speed_mps=0.018,
        pulse_before=-657,
        pulse_current=-656,
        pulse_following=-656,
        odometer_before=0.154,
        odometer_current=0.154,
        ctrl_feedback_stationary=True,
        opposite_wheel_coherent_motion=False,
        can_state="ERROR-ACTIVE",
        can_error_count=0,
        socket_overflow_count=0,
        visible_wheel_rotation=False,
        hard_diagnostic=False,
    )
    values.update(overrides)
    return classify_stationary_encoder_one_count_dither(**values)


def test_k4e_one_count_dither_is_contained_warning():
    for speed, before, current, following in (
        (0.018, -657, -656, -656),
        (-0.021, -656, -657, -657),
        (-0.028, -656, -657, -657),
    ):
        result = dither(speed_mps=speed, pulse_before=before, pulse_current=current, pulse_following=following)
        assert result.accepted
        assert result.reason == "STATIONARY_ENCODER_ONE_COUNT_DITHER_WARNING"


def test_k4e_dither_rejects_accumulation_and_hazards():
    for values in (
        {"pulse_before": 10, "pulse_current": 11, "pulse_following": 12},
        {"pulse_before": 10, "pulse_current": 12, "pulse_following": 12},
        {"speed_mps": 0.031},
        {"odometer_current": 0.155},
        {"ctrl_feedback_stationary": False},
        {"opposite_wheel_coherent_motion": True},
        {"visible_wheel_rotation": True},
        {"can_state": "ERROR-PASSIVE"},
        {"hard_diagnostic": True},
    ):
        assert not dither(**values).accepted


def boundary(**overrides):
    values = dict(
        pulse_anchor=-685,
        pulse_samples=(-686, -685, -686, -685),
        speed_samples_mps=(0.0, -0.055, 0.0, 0.054),
        odometer_anchor=0.154,
        odometer_samples=(0.154, 0.154, 0.154, 0.154),
        opposite_wheel_pulse_samples=(-252, -252, -252, -252),
        ctrl_feedback_stationary=True,
        can_state="ERROR-ACTIVE",
        can_error_count=0,
        socket_overflow_count=0,
        visible_wheel_rotation=False,
        hard_diagnostic=False,
    )
    values.update(overrides)
    return classify_stationary_encoder_boundary_dither(**values)


def test_k4f_repeated_adjacent_position_cluster_is_warning():
    result = boundary()
    assert result.accepted
    assert result.reason == "STATIONARY_ENCODER_BOUNDARY_DITHER"
    assert result.pulse_span == 1


def test_k4f_accumulation_or_odometer_change_is_hard_abort():
    assert not boundary(pulse_samples=(-686, -687)).accepted
    assert not boundary(pulse_anchor=-685, pulse_samples=(-684, -683)).accepted
    assert not boundary(odometer_samples=(0.155,)).accepted
    assert not boundary(can_state="ERROR-PASSIVE").accepted
    assert not boundary(opposite_wheel_pulse_samples=(-252, -253)).accepted


def test_k4f_opposite_wheel_one_count_out_and_back_is_not_coherent_motion():
    result = boundary(opposite_wheel_pulse_samples=(-252, -253, -253, -252))
    assert result.accepted
