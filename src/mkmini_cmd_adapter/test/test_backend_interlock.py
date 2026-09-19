from dataclasses import replace

import pytest

from mkmini_cmd_adapter import (
    BackendCommand,
    BackendFeedback,
    BackendInterlockConfig,
    BackendInterlockState,
    Gear,
    MkminiBackendInterlock,
    RunningMode,
)


CONFIG = BackendInterlockConfig(
    feedback_timeout_sec=0.05,
    heartbeat_timeout_sec=0.03,
    priming_duration_sec=0.04,
    stable_feedback_duration_sec=0.06,
)
N_ZERO = BackendCommand.safe_neutral()


def feedback(**changes):
    value = BackendFeedback(
        ctrl_age_sec=0.005,
        diagnostic_age_sec=0.005,
        ctrl_checksum_valid=True,
        diagnostic_checksum_valid=True,
        ctrl_alive_contiguous=True,
        diagnostic_alive_contiguous=True,
        gear=Gear.N,
        speed_mps=0.0,
        steering_deg=0.0,
        mode=RunningMode.AUTO,
        vehicle_fault_level=1,
        auto_can_error=False,
        can_state="ERROR-ACTIVE",
        hard_fault=False,
    )
    return replace(value, **changes)


def qualify(core, sample=None):
    sample = sample or feedback()
    core.arm(0.0)
    results = [core.step(stamp, N_ZERO, sample) for stamp in (0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12)]
    return results[-1]


def test_fresh_n_zero_priming_and_stable_required_state_permits_motion():
    core = MkminiBackendInterlock(CONFIG)
    result = qualify(core)
    assert result.state is BackendInterlockState.MOTION_PERMITTED
    assert result.motion_permitted
    assert result.output == N_ZERO
    drive = BackendCommand(Gear.D, 0.05, 0.0)
    admitted = core.step(0.14, drive, feedback())
    assert admitted.command_admitted and admitted.output == drive


def test_permission_is_false_until_both_priming_and_feedback_stability_complete():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    for stamp in (0.00, 0.02, 0.04, 0.06, 0.08, 0.10):
        result = core.step(stamp, N_ZERO, feedback())
        assert not result.motion_permitted
    assert core.step(0.12, N_ZERO, feedback()).motion_permitted


@pytest.mark.parametrize(
    "bad,reason",
    [
        (feedback(ctrl_age_sec=0.051), "CTRL_FEEDBACK_STALE"),
        (feedback(ctrl_age_sec=None), "CTRL_FEEDBACK_MISSING_OR_MALFORMED"),
        (feedback(vehicle_fault_level=2), "HARD_FAULT_LEVEL_2"),
        (feedback(vehicle_fault_level=3), "HARD_FAULT_LEVEL_3"),
        (feedback(can_state="ERROR-PASSIVE"), "CAN_UNHEALTHY"),
        (feedback(ctrl_checksum_valid=False), "FEEDBACK_CHECKSUM_INVALID"),
    ],
)
def test_invalid_feedback_fails_closed_and_latches(bad, reason):
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    result = core.step(0.0, N_ZERO, bad)
    assert result.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert not result.motion_permitted and not result.command_admitted
    assert result.output == N_ZERO
    assert reason in result.reasons
    assert core.step(0.01, N_ZERO, feedback()).state is BackendInterlockState.MOTION_NOT_PERMITTED


def test_state_change_during_admission_fails_and_requires_requalification():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    for stamp in (0.00, 0.02, 0.04, 0.06):
        core.step(stamp, N_ZERO, feedback())
    result = core.step(0.08, N_ZERO, feedback(mode=RunningMode.STOP, gear=Gear.P, auto_can_error=True))
    assert result.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert {"FEEDBACK_GEAR_NOT_N", "FEEDBACK_MODE_NOT_AUTO", "AUTO_CAN_NOT_FALSE"}.issubset(result.reasons)
    assert not core.step(0.10, N_ZERO, feedback()).motion_permitted
    core.arm(1.0)
    assert qualify_from_armed(core).motion_permitted


def test_waiting_allows_initial_p_stop_to_change_to_n_auto_before_stability_starts():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    parked = feedback(gear=Gear.P, mode=RunningMode.STOP, auto_can_error=True)
    for stamp in (0.00, 0.02, 0.04, 0.06):
        result = core.step(stamp, N_ZERO, parked)
    assert result.state is BackendInterlockState.WAITING_FOR_VALID_FEEDBACK
    assert not result.motion_permitted
    for stamp in (0.08, 0.10, 0.12, 0.14):
        result = core.step(stamp, N_ZERO, feedback())
    assert result.motion_permitted


def qualify_from_armed(core):
    result = None
    for stamp in (1.00, 1.02, 1.04, 1.06, 1.08, 1.10, 1.12):
        result = core.step(stamp, N_ZERO, feedback())
    return result


def test_nonzero_before_permission_fails_closed():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    result = core.step(0.0, BackendCommand(Gear.D, 0.01, 0.0), feedback())
    assert result.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert result.reasons == ("NONZERO_OR_DRIVE_BEFORE_PERMISSION",)
    assert result.output == N_ZERO


def test_permission_loss_with_motion_command_immediately_substitutes_zero():
    core = MkminiBackendInterlock(CONFIG)
    assert qualify(core).motion_permitted
    drive = BackendCommand(Gear.D, 0.05, 0.0)
    assert core.step(0.14, drive, feedback()).output == drive
    lost = core.step(0.16, drive, feedback(diagnostic_age_sec=0.10))
    assert lost.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert not lost.motion_permitted and not lost.command_admitted
    assert lost.output == N_ZERO


def test_expected_d_feedback_and_speed_remain_permitted_during_motion():
    core = MkminiBackendInterlock(CONFIG)
    qualify(core)
    drive = BackendCommand(Gear.D, 0.05, 0.0)
    assert core.step(0.14, drive, feedback()).motion_permitted
    moving = feedback(gear=Gear.D, speed_mps=0.05)
    assert core.step(0.16, drive, moving).motion_permitted


def test_commanded_steering_transition_may_lag_briefly_but_must_settle():
    core = MkminiBackendInterlock(CONFIG)
    assert qualify(core).motion_permitted
    priming = BackendCommand(Gear.D, 0.0, 5.0)
    assert core.step(0.14, priming, feedback(gear=Gear.N, steering_deg=0.0)).motion_permitted
    # R16 measured roughly 0.45 s settling for a 5 degree physical command.
    # A newly commanded moving transition is bounded rather than compared to
    # the new target as though the actuator response were instantaneous.
    drive = BackendCommand(Gear.D, .04, 5.0)
    result = core.step(0.16, drive, feedback(gear=Gear.D, speed_mps=.01, steering_deg=0.0))
    assert result.motion_permitted
    for stamp in (0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38):
        assert core.step(stamp, drive, feedback(gear=Gear.D, speed_mps=.01)).motion_permitted
    assert core.step(0.40, drive, feedback(
        gear=Gear.D, speed_mps=.01, steering_deg=5.0)).motion_permitted


def test_steering_transition_still_fails_for_wrong_sign_excess_or_timeout():
    for stamp, observed, reason in (
        (0.16, -5.0, "FEEDBACK_STEERING_SIGN_INVALID"),
        (0.16, 20.1, "FEEDBACK_STEERING_TRANSITION_ERROR"),
        (1.645, 0.0, "FEEDBACK_STEERING_TRANSITION_TIMEOUT"),
    ):
        core = MkminiBackendInterlock(CONFIG)
        qualify(core)
        drive = BackendCommand(Gear.D, .04, 5.0)
        assert core.step(.14, drive, feedback(gear=Gear.D, speed_mps=.01)).motion_permitted
        if stamp > .16:
            for tick in [0.16 + .02 * index for index in range(74)]:
                assert core.step(tick, drive, feedback(
                    gear=Gear.D, speed_mps=.01)).motion_permitted
        result = core.step(stamp, drive, feedback(gear=Gear.D, speed_mps=.01, steering_deg=observed))
        assert result.state is BackendInterlockState.MOTION_NOT_PERMITTED
        assert reason in result.reasons


def test_n_to_d_to_zero_is_a_legitimate_in_motion_sequence():
    core = MkminiBackendInterlock(CONFIG)
    assert qualify(core).motion_permitted
    drive = BackendCommand(Gear.D, 0.05, 0.0)
    assert core.step(0.14, drive, feedback(gear=Gear.N)).motion_permitted
    assert core.step(0.16, drive, feedback(gear=Gear.D, speed_mps=0.03)).motion_permitted
    stopped = core.step(0.18, BackendCommand(Gear.D, 0.0, 0.0), feedback(gear=Gear.D))
    assert stopped.motion_permitted and stopped.output.speed_mps == 0.0


@pytest.mark.parametrize(
    "bad,reason",
    [
        (feedback(gear=Gear.D, ctrl_age_sec=0.051), "CTRL_FEEDBACK_STALE"),
        (feedback(gear=Gear.D, vehicle_fault_level=2), "HARD_FAULT_LEVEL_2"),
    ],
)
def test_d_motion_feedback_still_fails_closed_for_stale_or_hard_fault(bad, reason):
    core = MkminiBackendInterlock(CONFIG)
    assert qualify(core).motion_permitted
    result = core.step(0.14, BackendCommand(Gear.D, 0.05, 0.0), bad)
    assert result.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert reason in result.reasons


@pytest.mark.parametrize(
    "changed,reason",
    [
        (feedback(gear=Gear.D, speed_mps=0.056), "FEEDBACK_MOTION_OVERSPEED"),
        (feedback(gear=Gear.D, speed_mps=0.05, steering_deg=0.11), "UNEXPECTED_FEEDBACK_STEERING"),
        (feedback(gear=Gear.R, speed_mps=0.01), "FEEDBACK_GEAR_NOT_FORWARD_SAFE"),
    ],
)
def test_motion_feedback_violation_removes_permission(changed, reason):
    core = MkminiBackendInterlock(CONFIG)
    qualify(core)
    result = core.step(0.14, BackendCommand(Gear.D, 0.05, 0.0), changed)
    assert result.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert reason in result.reasons
    assert result.output == N_ZERO


def test_missing_heartbeat_watchdog_fails_closed():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    core.step(0.0, N_ZERO, feedback())
    lost = core.watchdog(0.031, feedback())
    assert lost.state is BackendInterlockState.MOTION_NOT_PERMITTED
    assert lost.reasons == ("N_ZERO_HEARTBEAT_STALE",)


def test_dedicated_sender_timestamp_is_the_heartbeat_authority():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    core.step(0.0, N_ZERO, feedback(), heartbeat_emitted=False)
    core.note_heartbeat_emitted(0.0)
    # A later supervisor evaluation does not itself refresh the watchdog.
    core.step(0.020, N_ZERO, feedback(), heartbeat_emitted=False)
    assert core.watchdog(0.029, feedback()).state is not BackendInterlockState.MOTION_NOT_PERMITTED
    assert core.watchdog(0.031, feedback()).reasons == ("N_ZERO_HEARTBEAT_STALE",)


def test_native_sender_health_witness_advances_interlock_watchdog():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    core.note_native_sender_healthy(0.01)
    assert core._last_heartbeat == pytest.approx(0.01)


def test_native_sender_health_witness_is_refreshed_before_delayed_supervisor_step():
    core = MkminiBackendInterlock(CONFIG)
    core.arm(0.0)
    result = None
    for stamp in (0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12):
        core.note_native_sender_healthy(stamp)
        result = core.step(stamp, N_ZERO, feedback(), heartbeat_emitted=False)
    assert result is not None and result.motion_permitted

    # A 46 ms Python supervisor scheduling pause exceeds the interlock's
    # generic 30 ms sender watchdog, but must not revoke permission when the
    # physical backend has independently verified healthy native CAN TX.
    delayed_tick = 0.166
    core.note_native_sender_healthy(delayed_tick)
    resumed = core.step(delayed_tick, N_ZERO, feedback(), heartbeat_emitted=False)
    assert resumed.state is BackendInterlockState.MOTION_PERMITTED
    assert resumed.motion_permitted


def test_level_one_auto_io_and_remote_off_do_not_enter_contract():
    # The backend contract deliberately consumes only hard fault level 2/3,
    # Auto-CAN, and explicitly consolidated hard diagnostics.
    assert qualify(MkminiBackendInterlock(CONFIG), feedback(vehicle_fault_level=1)).motion_permitted


def test_speed_limit_and_reverse_are_never_admitted():
    for command, reason in (
        (BackendCommand(Gear.D, 0.051, 0.0), "MOTION_SPEED_LIMIT_EXCEEDED"),
        (BackendCommand(Gear.R, 0.01, 0.0), "GEAR_NOT_ADMITTED"),
    ):
        core = MkminiBackendInterlock(CONFIG)
        qualify(core)
        result = core.step(0.14, command, feedback())
        assert not result.motion_permitted
        assert reason in result.reasons
