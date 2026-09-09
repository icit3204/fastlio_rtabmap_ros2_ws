import math

import pytest

from mkmini_cmd_adapter import (
    HealthAssessment,
    HealthReason,
    HealthStatus,
    MkminiControllerValidityCore,
    ValidityEvidence,
    ValidityReason,
    ValidityState,
)


def healthy():
    return HealthAssessment(HealthStatus.VALID, True)


def evidence(*, ctrl=healthy(), diag=healthy(), ctrl_age=0.01, diag_age=0.02):
    return ValidityEvidence(ctrl, diag, ctrl_age, diag_age)


def test_startup_and_missing_feedback_are_false():
    core = MkminiControllerValidityCore(0.5, 0.2)
    result = core.evaluate(ValidityEvidence(None, None, None, None), 10.0)
    assert not result.valid
    assert result.state is ValidityState.INVALID
    assert ValidityReason.NO_CTRL_FEEDBACK.value in result.contributing_reasons
    assert ValidityReason.NO_DIAGNOSTIC_FEEDBACK.value in result.contributing_reasons


def test_unresolved_project_thresholds_are_ineligible():
    for timeout, stability in ((None, 0.2), (0.5, None), (-1.0, 0.2), (0.5, 0.0)):
        result = MkminiControllerValidityCore(timeout, stability).evaluate(evidence(), 0.0)
        assert not result.valid
        assert result.reason == ValidityReason.PARAMETERS_UNRESOLVED.value
        assert not result.configured


def test_healthy_requires_explicit_recovery_stability():
    core = MkminiControllerValidityCore(0.5, 1.0)
    first = core.evaluate(evidence(), 10.0)
    assert first.state is ValidityState.STABILITY_WAIT
    assert not first.valid
    second = core.evaluate(evidence(), 10.99)
    assert second.state is ValidityState.STABILITY_WAIT
    third = core.evaluate(evidence(), 11.0)
    assert third.state is ValidityState.VALID
    assert third.valid


@pytest.mark.parametrize(
    "assessment, expected",
    [
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.MODE_REMOTE,)), "MODE_REMOTE"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.MODE_STOP,)), "MODE_STOP"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.ESTOP_ASSERTED,)), "ESTOP_ASSERTED"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.AUTO_CAN_FAULT,)), "AUTO_CAN_FAULT"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.EPS_FAULT,)), "EPS_FAULT"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.LEFT_DRIVE_FAULT,)), "LEFT_DRIVE_FAULT"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.RIGHT_DRIVE_FAULT,)), "RIGHT_DRIVE_FAULT"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.BMS_CAN_FAULT,)), "BMS_CAN_FAULT"),
        (HealthAssessment(HealthStatus.INVALID, False, (HealthReason.REMOTE_RECEIVER_FAULT,)), "REMOTE_RECEIVER_FAULT"),
        (HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.MODE_UNKNOWN,)), "MODE_UNKNOWN"),
    ],
)
def test_explicit_invalid_or_unknown_evidence_is_false(assessment, expected):
    result = MkminiControllerValidityCore(0.5, 0.1).evaluate(
        evidence(ctrl=assessment), 1.0
    )
    assert not result.valid
    assert expected in result.contributing_reasons


def test_stale_checksum_alive_and_timestamp_evidence_is_false():
    bad = HealthAssessment(
        HealthStatus.INVALID,
        False,
        (HealthReason.FEEDBACK_CHECKSUM_INVALID, HealthReason.FEEDBACK_ALIVE_DISCONTINUITY),
    )
    result = MkminiControllerValidityCore(0.5, 0.1).evaluate(
        evidence(ctrl=bad, ctrl_age=0.51), 1.0
    )
    assert not result.valid
    assert "FEEDBACK_CHECKSUM_INVALID" in result.contributing_reasons
    assert "FEEDBACK_ALIVE_DISCONTINUITY" in result.contributing_reasons
    assert ValidityReason.CTRL_FEEDBACK_STALE.value in result.contributing_reasons


def test_recovery_resets_after_any_required_failure():
    core = MkminiControllerValidityCore(0.5, 1.0)
    assert core.evaluate(evidence(), 0.0).state is ValidityState.STABILITY_WAIT
    assert core.evaluate(evidence(), 0.9).state is ValidityState.STABILITY_WAIT
    failed = core.evaluate(evidence(ctrl_age=0.6), 0.95)
    assert failed.state is ValidityState.INVALID
    restarted = core.evaluate(evidence(), 1.0)
    assert restarted.state is ValidityState.STABILITY_WAIT
    assert restarted.stable_for_sec == 0.0


def test_timestamp_reversal_and_nonfinite_now_fail_closed():
    timestamp = HealthAssessment(
        HealthStatus.UNKNOWN, False,
        unknown_reasons=(HealthReason.FEEDBACK_TIMESTAMP_REVERSAL,),
    )
    result = MkminiControllerValidityCore(0.5, 0.1).evaluate(
        evidence(ctrl=timestamp), 1.0
    )
    assert not result.valid
    assert "FEEDBACK_TIMESTAMP_REVERSAL" in result.contributing_reasons
    invalid_now = MkminiControllerValidityCore(0.5, 0.1).evaluate(evidence(), math.nan)
    assert invalid_now.reason == ValidityReason.INVALID_TIMESTAMP.value
