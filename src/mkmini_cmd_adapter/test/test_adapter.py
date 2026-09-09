import math

import pytest

from mkmini_cmd_adapter import (
    AdapterConfig,
    AdapterReason,
    AdapterState,
    CtrlFeedback,
    Direction,
    GateContext,
    Gear,
    HealthAssessment,
    HealthReason,
    HealthStatus,
    MkminiAdapterCore,
    MkminiCanCodec,
    MockTransport,
    RunningMode,
    StationaryGearPolicy,
    WheelFeedback,
)
from mkmini_cmd_adapter.model import CtrlCommand


HEALTHY = HealthAssessment(HealthStatus.VALID, True)


def ctrl_feedback(speed=0.0, mode=RunningMode.AUTO):
    return CtrlFeedback(
        gear=Gear.D,
        speed_magnitude_mps=speed,
        inner_wheel_steering_deg=0.0,
        running_mode=mode,
        alive_counter=0,
        checksum_valid=True,
        reserved_bits_36_43=0,
        reserved_bits_46_51=0,
    )


def wheel_feedback(speed=0.0):
    return WheelFeedback(speed_mps=speed, pulse_count=0, alive_counter=0, checksum_valid=True)


def gate():
    return GateContext(state="ARMED")


def configured(policy=StationaryGearPolicy.UNRESOLVED):
    return AdapterConfig(
        standstill_speed_threshold_mps=0.01,
        standstill_stability_sec=1.0,
        stationary_gear_policy=policy,
    )


def standstill_step(core, now, *, health=HEALTHY):
    return core.step(
        0.0,
        0.0,
        command_timestamp_sec=now,
        now_sec=now,
        feedback_health=health,
        ctrl_feedback=ctrl_feedback(),
        left_wheel_feedback=wheel_feedback(),
        right_wheel_feedback=wheel_feedback(),
    )


def test_startup_without_feedback_is_zero_and_waiting():
    result = MkminiAdapterCore().step(0.0, 0.0, 0.0, 0.0, None)
    assert result.state is AdapterState.WAITING_FOR_FEEDBACK
    assert result.reason is AdapterReason.WAITING_FOR_FEEDBACK
    assert result.command.is_zero_speed
    assert result.command.gear is None


def test_healthy_stationary_requires_explicit_evidence_and_stability():
    core = MkminiAdapterCore(configured())
    first = standstill_step(core, 0.0)
    assert first.state is AdapterState.STATIONARY_UNCONFIRMED
    assert first.standstill_evidence is True
    assert first.standstill_stable_sec == pytest.approx(0.0)
    pending = standstill_step(core, 0.5)
    assert pending.state is AdapterState.STATIONARY_UNCONFIRMED
    assert pending.reason is AdapterReason.STANDSTILL_STABILITY_PENDING
    held = standstill_step(core, 1.0)
    assert held.state is AdapterState.STATIONARY_HOLD
    assert held.reason is AdapterReason.STATIONARY_POLICY_UNRESOLVED
    assert held.valid is False
    assert held.command.speed_magnitude_mps == 0.0
    assert held.command.gear is None


@pytest.mark.parametrize(
    "health",
    [
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.MODE_REMOTE,)),
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.MODE_STOP,)),
        HealthAssessment(HealthStatus.UNKNOWN, False, unknown_reasons=(HealthReason.MODE_UNKNOWN,)),
    ],
)
def test_non_auto_feedback_never_allows_nonzero(health):
    result = MkminiAdapterCore().step(0.2, 0.0, 0.0, 0.0, health, gate=gate())
    assert result.command.is_zero_speed
    assert result.state is AdapterState.WAITING_FOR_FEEDBACK


def test_forward_motion_requires_armed_gate_and_maps_to_d():
    result = MkminiAdapterCore().step(0.2, 0.1, 0.0, 0.0, HEALTHY, gate=gate())
    assert result.valid
    assert result.state is AdapterState.MOVING_FORWARD
    assert result.command.gear is Gear.D
    assert result.command.speed_magnitude_mps == pytest.approx(0.2)
    assert result.command.inner_wheel_steering_deg > 0.0


def test_reverse_motion_maps_to_r_and_uses_positive_speed_magnitude():
    result = MkminiAdapterCore().step(-0.2, 0.1, 0.0, 0.0, HEALTHY, gate=gate())
    assert result.valid
    assert result.state is AdapterState.MOVING_REVERSE
    assert result.command.gear is Gear.R
    assert result.command.speed_magnitude_mps == pytest.approx(0.2)
    assert result.command.inner_wheel_steering_deg < 0.0


def test_forward_zero_enters_stopping_and_retains_direction_and_steering():
    core = MkminiAdapterCore(configured())
    moving = core.step(0.2, 0.1, 0.0, 0.0, HEALTHY, gate=gate())
    stopping = core.step(
        0.0,
        0.0,
        1.0,
        1.0,
        HEALTHY,
        ctrl_feedback=ctrl_feedback(0.2),
        left_wheel_feedback=wheel_feedback(0.2),
        right_wheel_feedback=wheel_feedback(0.2),
    )
    assert moving.command.inner_wheel_steering_deg != 0.0
    assert stopping.state is AdapterState.STOPPING_FORWARD
    assert stopping.command.gear is Gear.D
    assert stopping.command.speed_magnitude_mps == 0.0
    assert stopping.command.inner_wheel_steering_deg == pytest.approx(moving.command.inner_wheel_steering_deg)
    assert stopping.standstill_evidence is False


def test_reverse_zero_enters_stopping_reverse_and_retains_steering():
    core = MkminiAdapterCore(configured())
    moving = core.step(-0.2, -0.1, 0.0, 0.0, HEALTHY, gate=gate())
    stopping = core.step(
        0.0,
        0.0,
        1.0,
        1.0,
        HEALTHY,
        ctrl_feedback=ctrl_feedback(0.2),
        left_wheel_feedback=wheel_feedback(0.2),
        right_wheel_feedback=wheel_feedback(0.2),
    )
    assert stopping.state is AdapterState.STOPPING_REVERSE
    assert stopping.command.gear is Gear.R
    assert stopping.command.inner_wheel_steering_deg == pytest.approx(moving.command.inner_wheel_steering_deg)


def test_stopping_reaches_hold_only_after_standstill_stability_window():
    core = MkminiAdapterCore(configured())
    core.step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=gate())
    moving_feedback = dict(
        ctrl_feedback=ctrl_feedback(0.2),
        left_wheel_feedback=wheel_feedback(0.2),
        right_wheel_feedback=wheel_feedback(0.2),
    )
    core.step(0.0, 0.0, 1.0, 1.0, HEALTHY, **moving_feedback)
    pending = standstill_step(core, 1.5)
    assert pending.state is AdapterState.STOPPING_FORWARD
    assert pending.standstill_evidence is True
    held = standstill_step(core, 2.5)
    assert held.state is AdapterState.STATIONARY_HOLD
    assert held.command.speed_magnitude_mps == 0.0


@pytest.mark.parametrize(
    "policy,expected",
    [
        (StationaryGearPolicy.P, Gear.P),
        (StationaryGearPolicy.N, Gear.N),
        (StationaryGearPolicy.DISABLE, Gear.DISABLE),
        (StationaryGearPolicy.HOLD_LAST_DIRECTION, Gear.D),
    ],
)
def test_candidate_stationary_policies_are_explicit_and_zero_speed(policy, expected):
    core = MkminiAdapterCore(configured(policy))
    core.step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=gate())
    core.step(0.0, 0.0, 1.0, 1.0, HEALTHY, ctrl_feedback=ctrl_feedback(0.2))
    standstill_step(core, 3.0)
    held = standstill_step(core, 4.0)
    assert held.state is AdapterState.STATIONARY_HOLD
    assert held.valid is True
    assert held.command.gear is expected
    assert held.command.speed_magnitude_mps == 0.0


def test_new_nonzero_before_standstill_resumes_only_with_eligible_command():
    core = MkminiAdapterCore(configured())
    core.step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=gate())
    core.step(0.0, 0.0, 1.0, 1.0, HEALTHY, ctrl_feedback=ctrl_feedback(0.2))
    resumed = core.step(0.1, -0.05, 1.5, 1.5, HEALTHY, gate=gate())
    assert resumed.state is AdapterState.MOVING_FORWARD
    assert resumed.command.gear is Gear.D


@pytest.mark.parametrize(
    "health",
    [
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.FEEDBACK_CHECKSUM_INVALID,)),
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.FEEDBACK_ALIVE_DISCONTINUITY,)),
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.FEEDBACK_STALE,)),
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.ESTOP_ASSERTED,)),
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.EPS_FAULT,)),
    ],
)
def test_feedback_loss_or_fault_while_moving_forces_zero(health):
    core = MkminiAdapterCore()
    core.step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=gate())
    result = core.step(0.2, 0.0, 0.1, 0.1, health, gate=gate())
    assert result.state is AdapterState.FAULTED
    assert result.command.is_zero_speed
    assert result.requires_fresh_command is True


def test_fault_recovery_does_not_auto_resume_and_requires_fresh_eligible_command():
    core = MkminiAdapterCore()
    core.step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=gate())
    core.step(0.2, 0.0, 0.1, 0.1, HealthAssessment(HealthStatus.INVALID, False), gate=gate())
    zero = core.step(0.0, 0.0, 0.2, 0.2, HEALTHY)
    assert zero.state is AdapterState.FAULTED
    assert zero.command.is_zero_speed
    resumed = core.step(0.2, 0.0, 0.3, 0.3, HEALTHY, gate=gate())
    assert resumed.state is AdapterState.MOVING_FORWARD
    assert resumed.requires_fresh_command is False


@pytest.mark.parametrize("context", [None, GateContext(state="DISARMED"), GateContext(state="FAULT"), GateContext(state="ARMED", arm_pending=True), GateContext(state="ARMED", safe_zero_quiescent=True)])
def test_motion_requires_explicit_non_quiescent_armed_gate_context(context):
    result = MkminiAdapterCore().step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=context)
    assert result.command.is_zero_speed
    assert result.state is AdapterState.WAITING_FOR_FEEDBACK
    assert result.reason in (AdapterReason.GATE_CONTEXT_MISSING, AdapterReason.GATE_NOT_ARMED)


def test_input_deadman_expiry_is_fail_closed():
    core = MkminiAdapterCore(AdapterConfig(input_timeout_sec=0.25))
    result = core.step(0.2, 0.0, 0.0, 0.251, HEALTHY, gate=gate())
    assert result.state is AdapterState.FAULTED
    assert result.reason is AdapterReason.INPUT_DEADMAN_EXPIRED
    assert result.command.is_zero_speed


def test_stationary_policy_unresolved_never_fabricates_codec_frame():
    core = MkminiAdapterCore(configured())
    core.step(0.2, 0.0, 0.0, 0.0, HEALTHY, gate=gate())
    core.step(0.0, 0.0, 1.0, 1.0, HEALTHY, ctrl_feedback=ctrl_feedback(0.2))
    standstill_step(core, 3.0)
    held = standstill_step(core, 4.0)
    assert held.command.gear is None
    assert held.command.speed_magnitude_mps == 0.0
    transport = MockTransport()
    if held.command.gear is not None:
        transport.send(MkminiCanCodec.encode(CtrlCommand(held.command.gear, 0.0, 0.0, 0)))
    assert transport.count == 0


def test_nonzero_composition_maps_physical_model_to_codec_and_mock_transport():
    core = MkminiAdapterCore()
    result = core.step(0.2, 0.1, 0.0, 0.0, HEALTHY, gate=gate())
    assert result.valid and result.command.gear is Gear.D
    frame = MkminiCanCodec.encode(
        CtrlCommand(
            result.command.gear,
            result.command.speed_magnitude_mps,
            result.command.inner_wheel_steering_deg,
            1,
        )
    )
    transport = MockTransport()
    transport.send(frame, timestamp=1.0)
    assert transport.count == 1
    assert transport.latest().frame.can_id == 0x18C4D2D0


def test_stationary_configuration_defaults_are_unselected():
    config = AdapterConfig()
    assert config.stationary_gear_policy is StationaryGearPolicy.UNRESOLVED
    assert config.standstill_speed_threshold_mps is None
    assert config.standstill_stability_sec is None
    assert config.input_timeout_sec is None


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_command_timestamps_are_fail_closed(value):
    result = MkminiAdapterCore().step(0.0, 0.0, value, 0.0, None)
    assert result.state is AdapterState.FAULTED
    assert result.command.is_zero_speed


def test_zero_never_produces_nonzero_speed_even_with_unhealthy_feedback():
    result = MkminiAdapterCore().step(
        0.0,
        0.0,
        0.0,
        0.0,
        HealthAssessment(HealthStatus.INVALID, False, reasons=(HealthReason.ESTOP_ASSERTED,)),
    )
    assert result.command.speed_magnitude_mps == 0.0
    assert math.isfinite(result.command.inner_wheel_steering_deg)
