import math

import pytest

from mkmini_cmd_adapter import (
    AliveCounter,
    AliveTracker,
    CanFrame,
    CtrlCommand,
    Gear,
    ManufacturerCommandValidator,
    MkminiCanCodec,
    MkminiFeedbackCodec,
    MockTransport,
    RunningMode,
)
from mkmini_cmd_adapter.codec import CTRL_CMD_ID, CTRL_FB_ID, checksum


def command(gear=Gear.D, speed=0.0, steering=0.0, alive=0):
    return CtrlCommand(gear, speed, steering, alive)


def payload(frame):
    assert frame.can_id == CTRL_CMD_ID
    assert frame.is_extended is True
    assert frame.dlc == 8
    return frame.data.hex(" ").upper()


def test_manufacturer_golden_d_vectors():
    assert payload(MkminiCanCodec.encode(command(alive=1))) == "04 00 00 00 00 00 10 14"
    assert payload(MkminiCanCodec.encode(command(alive=2))) == "04 00 00 00 00 00 20 24"
    assert payload(MkminiCanCodec.encode(command(alive=3))) == "04 00 00 00 00 00 30 34"


def test_manufacturer_golden_speed_vectors():
    assert payload(MkminiCanCodec.encode(command(speed=1.0, alive=0))) == "84 3E 00 00 00 00 00 BA"
    assert payload(MkminiCanCodec.encode(command(speed=1.0, alive=1))) == "84 3E 00 00 00 00 10 AA"
    assert payload(MkminiCanCodec.encode(command(speed=1.0, alive=2))) == "84 3E 00 00 00 00 20 9A"


def test_manufacturer_golden_steering_vectors():
    assert payload(MkminiCanCodec.encode(command(gear=Gear.DISABLE, steering=-25.0, alive=0))) == "00 00 C0 63 0F 00 00 AC"
    assert payload(MkminiCanCodec.encode(command(gear=Gear.DISABLE, steering=-25.0, alive=1))) == "00 00 C0 63 0F 00 10 BC"
    assert payload(MkminiCanCodec.encode(command(gear=Gear.DISABLE, steering=-25.0, alive=2))) == "00 00 C0 63 0F 00 20 8C"


@pytest.mark.parametrize("gear", list(Gear))
def test_all_gear_values_and_reserved_bits(gear):
    frame = MkminiCanCodec.encode(command(gear=gear, speed=0.001, steering=0.01, alive=15))
    assert frame.data[4] & 0xF0 == 0
    assert frame.data[5] == 0
    assert frame.data[6] & 0x0F == 0
    assert frame.data[7] == checksum(frame.data[:7])


def test_wire_boundaries_and_signed_steering():
    assert MkminiCanCodec.encode(command(speed=0.001)).data[:2] == bytes((0x14, 0x00))
    assert MkminiCanCodec.encode(command(speed=65.535)).data[:2] == bytes((0xF4, 0xFF))
    assert MkminiCanCodec.encode(command(steering=-327.68)).data[2:5] == bytes((0x00, 0x00, 0x08))
    assert MkminiCanCodec.encode(command(steering=327.67)).data[2:5] == bytes((0xF0, 0xFF, 0x07))
    assert MkminiCanCodec.encode(command(steering=34.0)).data[2:5] == bytes((0x80, 0xD4, 0x00))
    assert MkminiCanCodec.encode(command(steering=-34.0)).data[2:5] == bytes((0x80, 0x2B, 0x0F))


def test_quantization_is_half_away_from_zero():
    assert MkminiCanCodec.encode(command(speed=0.0005)).data[0] == 0x14
    assert MkminiCanCodec.encode(command(steering=0.005)).data[2] == 0x10
    assert MkminiCanCodec.encode(command(steering=-0.005)).data[2] == 0xF0


@pytest.mark.parametrize(
    "bad",
    [
        command(gear=99),
        command(alive=16),
        command(speed=-0.001),
        command(speed=float("nan")),
        command(speed=float("inf")),
        command(steering=float("nan")),
        command(speed=65.536),
        command(steering=327.68),
        command(steering=-327.69),
    ],
)
def test_invalid_inputs_fail_closed(bad):
    with pytest.raises(ValueError):
        MkminiCanCodec.encode(bad)


def test_checksum_and_every_payload_bit_corruption():
    frame = MkminiCanCodec.encode(command(speed=1.0, steering=-25.0, alive=7))
    assert checksum(frame.data[:7]) == frame.data[7]
    for byte_index in range(7):
        for bit in range(8):
            corrupted = bytearray(frame.data)
            corrupted[byte_index] ^= 1 << bit
            assert corrupted[7] != checksum(corrupted[:7])


def test_alive_counter_and_tracker():
    counter = AliveCounter(14)
    assert [counter.next(), counter.next(), counter.next(), counter.next()] == [14, 15, 0, 1]
    tracker = AliveTracker()
    assert tracker.observe(9).initialized is True
    assert tracker.observe(10).contiguous is True
    assert tracker.observe(12).contiguous is False
    assert tracker.observe(13).contiguous is True


def test_manufacturer_limits_are_separate_from_wire_packing():
    assert ManufacturerCommandValidator.validate(command(speed=65.535)) == ("manufacturer_speed_limit",)
    assert ManufacturerCommandValidator.validate(command(steering=40.0)) == ("manufacturer_steering_limit",)
    assert MkminiCanCodec.encode(command(speed=65.535))


def test_mock_transport_is_bounded_and_memory_only():
    transport = MockTransport(max_records=2)
    transport.send(MkminiCanCodec.encode(command(alive=0)), timestamp=1.0)
    transport.send(MkminiCanCodec.encode(command(alive=1)), timestamp=2.0)
    transport.send(MkminiCanCodec.encode(command(alive=2)), timestamp=3.0)
    assert transport.count == 2
    assert transport.latest().timestamp == 3.0
    assert transport.records[0].timestamp == 2.0
    transport.clear()
    assert transport.count == 0


def feedback_frame(can_id, word, alive=0):
    word |= alive << 52
    data = bytearray((word >> (8 * index)) & 0xFF for index in range(7))
    data.append(checksum(data))
    return CanFrame(can_id, bytes(data), True)


def test_ctrl_feedback_decodes_fields_and_flags_checksum():
    word = 4 | (1234 << 4) | ((-2500 & 0xFFFF) << 20) | (2 << 44)
    decoded = MkminiFeedbackCodec.decode_ctrl_fb(feedback_frame(CTRL_FB_ID, word, alive=6))
    assert decoded.gear == 4
    assert decoded.speed_magnitude_mps == pytest.approx(1.234)
    assert decoded.inner_wheel_steering_deg == pytest.approx(-25.0)
    assert decoded.running_mode is RunningMode.STOP
    assert decoded.alive_counter == 6
    assert decoded.checksum_valid is True
    bad = bytearray(feedback_frame(CTRL_FB_ID, word).data)
    bad[0] ^= 1
    assert MkminiFeedbackCodec.decode_ctrl_fb(CanFrame(CTRL_FB_ID, bytes(bad), True)).checksum_valid is False


@pytest.mark.parametrize("decoder,can_id", [
    (MkminiFeedbackCodec.decode_lr_wheel_fb, 0x18C4D7EF),
    (MkminiFeedbackCodec.decode_rr_wheel_fb, 0x18C4D8EF),
])
def test_wheel_feedback_decoders(decoder, can_id):
    speed = (-1234) & 0xFFFF
    pulse = (-123456) & 0xFFFFFFFF
    frame = CanFrame(can_id, speed.to_bytes(2, "little") + pulse.to_bytes(4, "little") + bytes((0x00, 0x00)), True)
    data = bytearray(frame.data)
    data[6] = 0xB0
    data[7] = checksum(data[:7])
    decoded = decoder(CanFrame(can_id, bytes(data), True))
    assert decoded.speed_mps == pytest.approx(-1.234)
    assert decoded.pulse_count == -123456
    assert decoded.alive_counter == 11
    assert decoded.checksum_valid is True


def test_feedback_identity_and_dlc_rejected():
    frame = feedback_frame(CTRL_FB_ID, 0)
    with pytest.raises(ValueError):
        MkminiFeedbackCodec.decode_ctrl_fb(CanFrame(0x123, frame.data, True))
    with pytest.raises(ValueError):
        MkminiFeedbackCodec.decode_ctrl_fb(CanFrame(CTRL_FB_ID, frame.data[:-1], True))
    with pytest.raises(ValueError):
        MkminiFeedbackCodec.decode_ctrl_fb(CanFrame(CTRL_FB_ID, frame.data, False))
