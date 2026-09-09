import inspect
from dataclasses import replace

import pytest

from mkmini_cmd_adapter import (
    BenchFeedbackSnapshot,
    ContainedBenchEligibilityPolicy,
    Gear,
    MicroMotionPhase,
    MicroMotionRunner,
    MkminiCanCodec,
    PhysicalContainment,
    RunningMode,
)


def containment():
    return PhysicalContainment(True, True, True, True)


def feedback(**overrides):
    values = dict(
        ctrl_current=True,
        diagnostic_current=True,
        ctrl_checksum_valid=True,
        diagnostic_checksum_valid=True,
        ctrl_alive_contiguous=True,
        diagnostic_alive_contiguous=True,
        can_state="ERROR-ACTIVE",
        mode=RunningMode.AUTO,
        auto_can_error=False,
        auto_io_error=True,
        remote_off_warning=True,
        remote_receiver_loss=False,
        vehicle_fault_level=1,
        emergency_stop=False,
        eps_fault=False,
        left_drive_fault=0,
        right_drive_fault=0,
        bms_can_loss=False,
    )
    values.update(overrides)
    return BenchFeedbackSnapshot(**values)


def test_k3e_warning_baseline_is_contained_only_and_production_false():
    result = ContainedBenchEligibilityPolicy.evaluate(containment(), feedback())
    assert result.eligible is True
    assert result.production_motion_eligible is False
    assert result.baseline_warnings == (
        "KNOWN_BASELINE_FAULT_LEVEL_1",
        "KNOWN_BASELINE_AUTO_IO_ERROR",
        "KNOWN_BASELINE_REMOTE_OFF_WARNING",
    )


@pytest.mark.parametrize("field", ["all_four_wheels_clear", "area_clear", "hands_clear", "estop_or_poweroff_accessible"])
def test_containment_is_mandatory(field):
    values = dict(all_four_wheels_clear=True, area_clear=True, hands_clear=True, estop_or_poweroff_accessible=True)
    values[field] = False
    result = ContainedBenchEligibilityPolicy.evaluate(PhysicalContainment(**values), feedback())
    assert result.eligible is False
    assert result.hard_abort_reasons


@pytest.mark.parametrize(
    "override,reason",
    [
        ({"vehicle_fault_level": 2}, "VEHICLE_FAULT_LEVEL_2_OR_HIGHER"),
        ({"emergency_stop": True}, "ESTOP_ASSERTED"),
        ({"eps_fault": True}, "EPS_FAULT"),
        ({"left_drive_fault": 1}, "LEFT_DRIVE_FAULT"),
        ({"right_drive_fault": 1}, "RIGHT_DRIVE_FAULT"),
        ({"bms_can_loss": True}, "BMS_CAN_LOSS"),
        ({"remote_receiver_loss": True}, "REMOTE_RECEIVER_LOSS"),
        ({"auto_can_error": True}, "AUTO_CAN_ERROR"),
        ({"mode": RunningMode.STOP}, "MODE_NOT_AUTO"),
        ({"can_state": "ERROR-PASSIVE"}, "CAN_NOT_ERROR_ACTIVE"),
        ({"unexpected_diagnostic": True}, "UNEXPECTED_DIAGNOSTIC"),
    ],
)
def test_hard_abort_and_unknown_conditions_fail_closed(override, reason):
    result = ContainedBenchEligibilityPolicy.evaluate(containment(), feedback(**override))
    assert result.eligible is False
    assert reason in result.hard_abort_reasons or reason in result.unexpected_warnings


def test_level_one_warning_can_clear_and_does_not_create_new_warning():
    result = ContainedBenchEligibilityPolicy.evaluate(
        containment(), feedback(vehicle_fault_level=0, auto_io_error=False, remote_off_warning=False)
    )
    assert result.eligible is True
    assert result.baseline_warnings == ()


def test_eligibility_requires_heartbeat_establishment():
    result = ContainedBenchEligibilityPolicy.evaluate(containment(), feedback(), heartbeat_established=False)
    assert result.eligible is False
    assert "COMMAND_HEARTBEAT_NOT_ESTABLISHED" in result.hard_abort_reasons


def test_fixed_sequence_contains_stops_before_neutral_and_no_other_phases():
    sequence = MicroMotionRunner.fixed_sequence()
    assert sequence.index(MicroMotionPhase.FORWARD_STOPPING) < sequence.index(MicroMotionPhase.FORWARD_NEUTRAL)
    assert sequence.index(MicroMotionPhase.REVERSE_STOPPING) < sequence.index(MicroMotionPhase.REVERSE_NEUTRAL)
    assert MicroMotionPhase.FORWARD_PULSE in sequence
    assert MicroMotionPhase.REVERSE_PULSE in sequence
    assert len(sequence) == 12


@pytest.mark.parametrize(
    "phase,gear,speed,steering",
    [
        (MicroMotionPhase.ZERO_HANDSHAKE, Gear.N, 0.0, 0.0),
        (MicroMotionPhase.FORWARD_PULSE, Gear.D, 0.05, 0.0),
        (MicroMotionPhase.FORWARD_STOPPING, Gear.D, 0.0, 0.0),
        (MicroMotionPhase.REVERSE_PULSE, Gear.R, 0.05, 0.0),
        (MicroMotionPhase.REVERSE_STOPPING, Gear.R, 0.0, 0.0),
        (MicroMotionPhase.STEER_POSITIVE, Gear.D, 0.0, 2.0),
        (MicroMotionPhase.STEER_NEGATIVE, Gear.D, 0.0, -2.0),
        (MicroMotionPhase.FINAL_NEUTRAL, Gear.N, 0.0, 0.0),
    ],
)
def test_fixed_commands_have_exact_safety_envelope(phase, gear, speed, steering):
    command = MicroMotionRunner.command_for(phase)
    assert (command.gear, command.speed_magnitude_mps, command.inner_wheel_steering_deg) == (gear, speed, steering)


def test_reverse_uses_unsigned_speed_magnitude():
    assert MicroMotionRunner.command_for(MicroMotionPhase.REVERSE_PULSE).speed_magnitude_mps == 0.05


def test_nonzero_pulses_are_bounded_and_100hz():
    emissions = MicroMotionRunner.synthesize_phase(MicroMotionPhase.FORWARD_PULSE, 0.10)
    assert len(emissions) == 10
    assert emissions[-1].timestamp_sec == pytest.approx(0.09)
    with pytest.raises(ValueError, match="fixed commissioning limit"):
        MicroMotionRunner.synthesize_phase(MicroMotionPhase.FORWARD_PULSE, 0.21)
    with pytest.raises(ValueError, match="fixed commissioning limit"):
        MicroMotionRunner.synthesize_phase(MicroMotionPhase.STEER_POSITIVE, 0.21)


def test_codec_generates_reserved_zero_and_valid_alive_checksum():
    for phase in (MicroMotionPhase.FORWARD_PULSE, MicroMotionPhase.REVERSE_PULSE, MicroMotionPhase.STEER_POSITIVE):
        frame = MicroMotionRunner.synthesize_phase(phase, 0.01, alive_initial=7)[0].frame
        assert frame.data[7] == (frame.data[0] ^ frame.data[1] ^ frame.data[2] ^ frame.data[3] ^ frame.data[4] ^ frame.data[5] ^ frame.data[6])
        assert ((int.from_bytes(frame.data[:7], "little") >> 36) & 0xFFFF) == 0


def test_alive_progresses_per_emitted_frame_and_uses_codec():
    emissions = MicroMotionRunner.synthesize_phase(MicroMotionPhase.FORWARD_PULSE, 0.05, alive_initial=14)
    assert [item.alive_counter for item in emissions] == [14, 15, 0, 1, 2]
    for item in emissions:
        expected = MkminiCanCodec.encode(
            replace(MicroMotionRunner.command_for(item.phase), alive_counter=item.alive_counter)
        )
        assert expected.data == item.frame.data


def test_runner_has_no_arbitrary_command_or_cli_speed_steering_surface():
    signature = inspect.signature(MicroMotionRunner.__init__)
    assert set(signature.parameters) == {"self", "transport"}
    assert not hasattr(MicroMotionRunner, "set_command")
    assert not hasattr(MicroMotionRunner, "set_speed")
    assert not hasattr(MicroMotionRunner, "set_steering")


def test_no_nonzero_speed_during_steering_only_phases():
    for phase in (MicroMotionPhase.STEER_POSITIVE, MicroMotionPhase.STEER_NEGATIVE):
        command = MicroMotionRunner.command_for(phase)
        assert command.speed_magnitude_mps == 0.0
        assert abs(command.inner_wheel_steering_deg) == 2.0


def test_encoder_scale_is_not_part_of_runner():
    assert not hasattr(MicroMotionRunner, "pulses_per_revolution")


class FakeBenchTransport:
    def __init__(self):
        self.frames = []

    def send_frame(self, frame, timestamp=None):
        self.frames.append((frame, timestamp))


def test_runner_emits_only_through_injected_fake_transport():
    transport = FakeBenchTransport()
    runner = MicroMotionRunner(transport)
    emissions = runner.emit_phase(MicroMotionPhase.FORWARD_PULSE, 0.02)
    assert len(transport.frames) == 2
    assert [item[0] for item in transport.frames] == [item.frame for item in emissions]
    assert all(item[0].is_extended for item in transport.frames)
