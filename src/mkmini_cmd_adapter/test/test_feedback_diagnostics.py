import pytest

from mkmini_cmd_adapter import (
    AliveTracker,
    BmsFlagFeedback,
    ChargingState,
    CanFrame,
    DiagnosticState,
    FeedbackFreshnessState,
    Gear,
    HealthAssessment,
    HealthReason,
    HealthStatus,
    MkminiFeedbackCodec,
    RunningMode,
    VehicleDiagnosticFeedback,
    assess_feedback_health,
    assess_mode,
)
from mkmini_cmd_adapter.codec import (
    BMS_FLAG_FB_ID,
    BMS_INFO_FB_ID,
    CTRL_FB_ID,
    ODO_FB_ID,
    ULTRASONIC_1_FB_ID,
    ULTRASONIC_2_FB_ID,
    VEH_DIAG_FB_ID,
    checksum,
)


def frame(can_id, word=0, trailing=b"", alive=0):
    word |= alive << 52
    data = bytearray((word >> (8 * index)) & 0xFF for index in range(7))
    data.extend(trailing)
    assert len(data) == 7
    data.append(checksum(data))
    return CanFrame(can_id, bytes(data), True)


def test_vehicle_diagnostic_decodes_all_defined_fields():
    word = (
        3
        | (1 << 4)
        | (1 << 5)
        | (0x00C << 8)
        | (0x13 << 32)
        | (0x0E << 38)
        | (1 << 44)
        | (1 << 45)
        | (1 << 46)
        | (1 << 47)
        | (0xA << 48)
    )
    decoded = MkminiFeedbackCodec.decode_vehicle_diagnostic(frame(VEH_DIAG_FB_ID, word, alive=15))
    assert decoded.vehicle_fault_level == 3
    assert decoded.auto_can_communication_error is True
    assert decoded.auto_io_can_communication_error is True
    assert decoded.eps_fault_code == 0xC
    assert decoded.left_drive_fault == 0x13
    assert decoded.right_drive_fault == 0x0E
    assert decoded.bms_can_communication_loss is True
    assert decoded.emergency_stop_asserted is True
    assert decoded.remote_off_warning is True
    assert decoded.remote_receiver_loss is True
    assert decoded.reserved_bits_48_51 == 0xA
    assert decoded.alive_counter == 15
    assert decoded.checksum_valid is True


def test_bms_info_and_flags_decode_units_and_enums():
    bms_word = 4800 | (-(1234) & 0xFFFF) << 16 | (5678 << 32)
    bms = MkminiFeedbackCodec.decode_bms_info(frame(BMS_INFO_FB_ID, bms_word, alive=4))
    assert bms.pack_voltage_v == pytest.approx(48.0)
    assert bms.pack_current_a == pytest.approx(-12.34)
    assert bms.remaining_capacity_ah == pytest.approx(56.78)
    assert bms.alive_counter == 4
    assert bms.checksum_valid is True

    flags_word = 87 | (0x1FFF << 8) | (2 << 21) | (1 << 23) | (1 << 24)
    flags_word |= ((-123 & 0xFFF) << 28) | ((456 & 0xFFF) << 40)
    flags = MkminiFeedbackCodec.decode_bms_flags(frame(BMS_FLAG_FB_ID, flags_word, alive=2))
    assert flags.soc_percent == 87
    assert all(
        getattr(flags, field)
        for field in (
            "cell_over_voltage", "cell_under_voltage", "pack_over_voltage", "pack_under_voltage",
            "charge_over_temperature", "charge_low_temperature", "discharge_over_temperature",
            "discharge_low_temperature", "charge_over_current", "discharge_over_current",
            "short_circuit", "front_end_ic_error", "software_mos_lock", "soc_low_alarm", "battery_low_alarm",
        )
    )
    assert flags.charging_state is ChargingState.FRONT_CHARGING_PILE
    assert flags.highest_temperature_c == pytest.approx(-12.3)
    assert flags.lowest_temperature_c == pytest.approx(45.6)


def test_ultrasonic_frames_decode_all_four_12_bit_distances():
    word = 100 | (200 << 12) | (300 << 24) | (400 << 36)
    for decoder, can_id in (
        (MkminiFeedbackCodec.decode_ultrasonic_1, ULTRASONIC_1_FB_ID),
        (MkminiFeedbackCodec.decode_ultrasonic_2, ULTRASONIC_2_FB_ID),
    ):
        decoded = decoder(frame(can_id, word, alive=9))
        assert decoded.probe_distances_mm == (100, 200, 300, 400)
        assert decoded.probe_distances_m == pytest.approx((0.1, 0.2, 0.3, 0.4))
        assert decoded.alive_counter == 9
        assert decoded.checksum_valid is True


def test_odometer_decodes_only_documented_cumulative_distance():
    raw = (123456).to_bytes(4, "little", signed=True) + bytes(3)
    data = bytearray(raw)
    data.append(checksum(data))
    decoded = MkminiFeedbackCodec.decode_odometer(CanFrame(ODO_FB_ID, bytes(data), True))
    assert decoded.cumulative_distance_m == pytest.approx(123.456)
    assert decoded.raw_payload == bytes(data)
    assert decoded.trailing_bytes_defined is False


@pytest.mark.parametrize("decoder,can_id", [
    (MkminiFeedbackCodec.decode_vehicle_diagnostic, VEH_DIAG_FB_ID),
    (MkminiFeedbackCodec.decode_bms_info, BMS_INFO_FB_ID),
    (MkminiFeedbackCodec.decode_bms_flags, BMS_FLAG_FB_ID),
    (MkminiFeedbackCodec.decode_ultrasonic_1, ULTRASONIC_1_FB_ID),
    (MkminiFeedbackCodec.decode_ultrasonic_2, ULTRASONIC_2_FB_ID),
    (MkminiFeedbackCodec.decode_odometer, ODO_FB_ID),
])
def test_new_decoders_reject_wrong_id_and_dlc_and_flag_checksum(decoder, can_id):
    good = frame(can_id)
    with pytest.raises(ValueError):
        decoder(CanFrame(can_id ^ 1, good.data, True))
    with pytest.raises(ValueError):
        decoder(CanFrame(can_id, good.data[:-1], True))
    corrupted = bytearray(good.data)
    corrupted[0] ^= 0x01
    decoded = decoder(CanFrame(can_id, bytes(corrupted), True))
    assert decoded.checksum_valid is False if hasattr(decoded, "checksum_valid") else True


def test_mode_is_explicit_and_unknown_fails_closed():
    assert assess_mode(RunningMode.AUTO).status is HealthStatus.VALID
    assert assess_mode(RunningMode.AUTO).eligible is True
    assert assess_mode(RunningMode.REMOTE).reasons == (HealthReason.MODE_REMOTE,)
    assert assess_mode(RunningMode.STOP).reasons == (HealthReason.MODE_STOP,)
    unknown = assess_mode(7)
    assert unknown.status is HealthStatus.UNKNOWN
    assert unknown.eligible is False
    assert unknown.unknown_reasons == (HealthReason.MODE_UNKNOWN,)


def diagnostic(**overrides):
    values = dict(
        vehicle_fault_level=0,
        auto_can_communication_error=False,
        auto_io_can_communication_error=False,
        eps_fault_code=0,
        left_drive_fault=0,
        right_drive_fault=0,
        bms_can_communication_loss=False,
        emergency_stop_asserted=False,
        remote_off_warning=False,
        remote_receiver_loss=False,
        alive_counter=0,
        checksum_valid=True,
        reserved_bits_48_51=0,
    )
    values.update(overrides)
    return VehicleDiagnosticFeedback(**values)


def test_diagnostic_state_preserves_explicit_fault_provenance():
    assessment = DiagnosticState.assess(
        diagnostic(
            vehicle_fault_level=2,
            auto_can_communication_error=True,
            eps_fault_code=2,
            left_drive_fault=1,
            right_drive_fault=2,
            bms_can_communication_loss=True,
            emergency_stop_asserted=True,
            remote_receiver_loss=True,
        )
    )
    assert assessment.status is HealthStatus.INVALID
    assert assessment.eligible is False
    assert assessment.reasons == (
        HealthReason.VEHICLE_FAULT_LEVEL,
        HealthReason.AUTO_CAN_FAULT,
        HealthReason.EPS_FAULT,
        HealthReason.LEFT_DRIVE_FAULT,
        HealthReason.RIGHT_DRIVE_FAULT,
        HealthReason.BMS_CAN_FAULT,
        HealthReason.ESTOP_ASSERTED,
        HealthReason.REMOTE_RECEIVER_FAULT,
    )
    unknown = DiagnosticState.assess(diagnostic(vehicle_fault_level=9))
    assert unknown.status is HealthStatus.UNKNOWN
    assert unknown.eligible is False


def test_freshness_is_timestamp_injected_and_recovery_is_explicit():
    freshness = FeedbackFreshnessState()
    assert freshness.assess(0.0, 0.5).status is HealthStatus.UNKNOWN
    freshness.observe(10.0)
    assert freshness.assess(10.4, 0.5).status is HealthStatus.VALID
    assert freshness.assess(10.6, 0.5).reasons == (HealthReason.FEEDBACK_STALE,)
    freshness.observe(10.7)
    assert freshness.assess(10.7, 0.5).eligible is True
    freshness.observe(10.6)
    assert freshness.assess(10.7, 0.5).status is HealthStatus.UNKNOWN


def test_combined_health_invalid_unknown_and_recovery_cases():
    fresh = HealthAssessment(HealthStatus.VALID, True)
    auto = assess_mode(RunningMode.AUTO)
    good_diag = DiagnosticState.assess(diagnostic())
    tracker = AliveTracker()
    assert assess_feedback_health(
        checksum_valid=True,
        alive_contiguous=tracker.observe(4).contiguous,
        freshness=fresh,
        mode=auto,
        diagnostics=good_diag,
    ).eligible is True
    bad = assess_feedback_health(
        checksum_valid=False,
        alive_contiguous=False,
        freshness=fresh,
        mode=assess_mode(RunningMode.REMOTE),
        diagnostics=good_diag,
    )
    assert bad.status is HealthStatus.INVALID
    assert HealthReason.FEEDBACK_CHECKSUM_INVALID in bad.reasons
    assert HealthReason.FEEDBACK_ALIVE_DISCONTINUITY in bad.reasons
    assert HealthReason.MODE_REMOTE in bad.reasons
