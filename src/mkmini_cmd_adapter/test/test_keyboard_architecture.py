import math

import pytest

from mkmini_cmd_adapter import (
    Gear,
    KeyboardCommissioningCore,
    KeyboardFeedbackEvidence,
    KeyboardFrameScheduler,
    KeyboardState,
    MockTransport,
    RealCanAuthorityError,
    RealTransportAuthority,
    ReadOnlyTransport,
    TransportFactory,
    TransportMode,
)


HEALTHY = KeyboardFeedbackEvidence(
    ctrl_current=True,
    diagnostic_current=True,
    ctrl_checksum_valid=True,
    diagnostic_checksum_valid=True,
    ctrl_alive_contiguous=True,
    diagnostic_alive_contiguous=True,
    mode=0,
)


def armed_core():
    core = KeyboardCommissioningCore()
    assert core.observe_feedback(HEALTHY).state is KeyboardState.ARM_READY
    assert core.arm().state is KeyboardState.ARMED_ZERO
    return core


def test_authority_is_unqualified_and_refuses_before_injected_transport():
    called = []

    with pytest.raises(RealCanAuthorityError, match="REAL_CAN_BLOCKED_BRAKE_PROFILE_UNRESOLVED"):
        TransportFactory.create(
            TransportMode.TX_CAPABLE,
            authority=RealTransportAuthority.current_unqualified(),
            tx_constructor=lambda: called.append("constructed"),
        )
    assert called == []


def test_modes_are_distinct_and_only_mock_is_current_output_surface():
    mock = TransportFactory.create(TransportMode.MOCK)
    readonly = TransportFactory.create(TransportMode.READ_ONLY)
    assert isinstance(mock, MockTransport)
    assert isinstance(readonly, ReadOnlyTransport)
    assert not hasattr(readonly, "send")


def test_feedback_and_explicit_arm_are_required_for_mock_motion():
    core = KeyboardCommissioningCore()
    assert core.set_command(0.2, 0.0, 0.0).accepted is False
    assert core.observe_feedback(HEALTHY).state is KeyboardState.ARM_READY
    assert core.set_command(0.2, 0.0, 0.0).accepted is False
    assert core.arm().state is KeyboardState.ARMED_ZERO
    moving = core.set_command(0.2, 0.1, 1.0)
    assert moving.accepted is True
    assert moving.state is KeyboardState.MOVING
    assert moving.command.gear is Gear.D


def test_deadman_stops_with_zero_speed_and_retains_direction_and_steering():
    core = armed_core()
    moving = core.set_command(0.2, 0.1, 0.0)
    stopping = core.tick(0.251)
    assert moving.command.inner_wheel_steering_deg != 0.0
    assert stopping.state is KeyboardState.STOPPING
    assert stopping.command.gear is Gear.D
    assert stopping.command.speed_magnitude_mps == 0.0
    assert stopping.command.inner_wheel_steering_deg == pytest.approx(moving.command.inner_wheel_steering_deg)


def test_feedback_fault_and_recovery_do_not_resume_automatically():
    core = armed_core()
    core.set_command(0.2, 0.0, 0.0)
    bad = KeyboardFeedbackEvidence(
        ctrl_current=True,
        diagnostic_current=True,
        ctrl_checksum_valid=False,
        diagnostic_checksum_valid=True,
        ctrl_alive_contiguous=True,
        diagnostic_alive_contiguous=True,
        mode=0,
    )
    assert core.observe_feedback(bad).state is KeyboardState.FAULTED
    assert core.observe_feedback(HEALTHY).state is KeyboardState.FAULTED
    assert core.set_command(0.2, 0.0, 1.0).accepted is False
    assert core.acknowledge_fault().state is KeyboardState.ARM_READY
    assert core.set_command(0.2, 0.0, 1.0).accepted is False
    assert core.arm().state is KeyboardState.ARMED_ZERO
    assert core.set_command(0.2, 0.0, 1.0).accepted is True


@pytest.mark.parametrize(
    "evidence",
    [
        KeyboardFeedbackEvidence(mode=1, ctrl_current=True, diagnostic_current=True, ctrl_checksum_valid=True, diagnostic_checksum_valid=True, ctrl_alive_contiguous=True, diagnostic_alive_contiguous=True),
        KeyboardFeedbackEvidence(mode=2, ctrl_current=True, diagnostic_current=True, ctrl_checksum_valid=True, diagnostic_checksum_valid=True, ctrl_alive_contiguous=True, diagnostic_alive_contiguous=True),
        KeyboardFeedbackEvidence(mode=0, ctrl_current=True, diagnostic_current=True, ctrl_checksum_valid=True, diagnostic_checksum_valid=True, ctrl_alive_contiguous=True, diagnostic_alive_contiguous=True, emergency_stop_asserted=True),
    ],
)
def test_remote_stop_and_estop_are_ineligible(evidence):
    assert evidence.eligible is False


def test_invalid_spin_and_steering_limit_never_produce_motion():
    core = armed_core()
    spin = core.set_command(0.0, 0.1, 0.0)
    assert spin.accepted is False
    assert core.state is KeyboardState.FAULTED
    core = armed_core()
    radius = 0.600 / math.tan(math.radians(34.0)) + 0.518 / 2.0
    limited = core.set_command(1.0, 1.0 / (radius - 0.001), 0.0)
    assert limited.accepted is False
    assert limited.command is None


def test_scheduler_is_event_rate_independent_and_alive_is_per_emission():
    transport = MockTransport(max_records=64)
    scheduler = KeyboardFrameScheduler(transport)
    core = armed_core()
    moving = core.set_command(0.2, 0.1, 0.0)
    scheduler.update(moving.command)
    # One command event, ten caller-owned 100 Hz ticks.
    emitted = [scheduler.emit_due(index * 0.01) for index in range(10)]
    emitted = [item for item in emitted if item is not None]
    assert len(emitted) == 10
    assert [item.alive_counter for item in emitted] == list(range(10))
    assert all(item.frame.data[7] == __import__("mkmini_cmd_adapter.codec", fromlist=["checksum"]).checksum(item.frame.data[:7]) for item in emitted)


def test_stopping_frame_is_zero_speed_and_stationary_unresolved_has_no_frame():
    transport = MockTransport()
    scheduler = KeyboardFrameScheduler(transport)
    core = armed_core()
    moving = core.set_command(-0.2, -0.1, 0.0)
    scheduler.update(moving.command)
    assert scheduler.emit(0.0).frame.data[0] & 0x0F == Gear.R
    stopping = core.set_command(0.0, 0.0, 0.1)
    scheduler.update(stopping.command)
    emitted = scheduler.emit(0.1)
    assert emitted.frame.data[0] & 0x0F == Gear.R
    assert ((int.from_bytes(emitted.frame.data[:7], "little") >> 4) & 0xFFFF) == 0
    scheduler.update(None)
    assert scheduler.emit(0.2) is None


def test_shutdown_is_fail_safe_and_does_not_retain_motion():
    core = armed_core()
    core.set_command(0.2, 0.0, 0.0)
    result = core.fail_safe_shutdown(exception=True)
    assert result.state is KeyboardState.FAULTED
    assert result.command.speed_magnitude_mps == 0.0
    assert core.set_command(0.2, 0.0, 1.0).accepted is False
