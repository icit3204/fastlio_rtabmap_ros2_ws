"""Pure MK-mini command/feedback codecs and continuity helpers."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from enum import IntEnum
import math
from typing import Iterable

from .model import CanFrame, CtrlCommand, Gear


CTRL_CMD_ID = 0x18C4D2D0
CTRL_FB_ID = 0x18C4D2EF
LR_WHEEL_FB_ID = 0x18C4D7EF
RR_WHEEL_FB_ID = 0x18C4D8EF
ODO_FB_ID = 0x18C4DEEF
ULTRASONIC_1_FB_ID = 0x18C4E8EF
ULTRASONIC_2_FB_ID = 0x18C4E9EF
VEH_DIAG_FB_ID = 0x18C4EAEF
BMS_INFO_FB_ID = 0x18C4E1EF
BMS_FLAG_FB_ID = 0x18C4E2EF
CTRL_CMD_DLC = 8
CTRL_CMD_PERIOD_SEC = 0.010
WIRE_MAX_SPEED_MPS = 65.535
WIRE_MIN_STEERING_DEG = -327.68
WIRE_MAX_STEERING_DEG = 327.67
MANUFACTURER_MAX_SPEED_MPS = 9.7 / 3.6
MANUFACTURER_MIN_STEERING_DEG = -34.0
MANUFACTURER_MAX_STEERING_DEG = 34.0


class FrameValidationError(ValueError):
    """Frame identity, length, or field validation failure."""


class ChecksumError(FrameValidationError):
    """Reserved for callers that choose strict checksum rejection."""


class RunningMode(IntEnum):
    AUTO = 0
    REMOTE = 1
    STOP = 2


class ChargingState(IntEnum):
    NOT_CHARGING = 0
    MANUAL_CHARGING = 1
    FRONT_CHARGING_PILE = 2
    REAR_CHARGING_PILE = 3


@dataclass(frozen=True)
class CtrlFeedback:
    gear: int
    speed_magnitude_mps: float
    inner_wheel_steering_deg: float
    running_mode: RunningMode | int
    alive_counter: int
    checksum_valid: bool
    reserved_bits_36_43: int
    reserved_bits_46_51: int


@dataclass(frozen=True)
class WheelFeedback:
    speed_mps: float
    pulse_count: int
    alive_counter: int
    checksum_valid: bool


@dataclass(frozen=True)
class VehicleDiagnosticFeedback:
    vehicle_fault_level: int
    auto_can_communication_error: bool
    auto_io_can_communication_error: bool
    eps_fault_code: int
    left_drive_fault: int
    right_drive_fault: int
    bms_can_communication_loss: bool
    emergency_stop_asserted: bool
    remote_off_warning: bool
    remote_receiver_loss: bool
    alive_counter: int
    checksum_valid: bool
    reserved_bits_48_51: int


@dataclass(frozen=True)
class BmsInfoFeedback:
    pack_voltage_v: float
    pack_current_a: float
    remaining_capacity_ah: float
    alive_counter: int
    checksum_valid: bool


@dataclass(frozen=True)
class BmsFlagFeedback:
    soc_percent: int
    cell_over_voltage: bool
    cell_under_voltage: bool
    pack_over_voltage: bool
    pack_under_voltage: bool
    charge_over_temperature: bool
    charge_low_temperature: bool
    discharge_over_temperature: bool
    discharge_low_temperature: bool
    charge_over_current: bool
    discharge_over_current: bool
    short_circuit: bool
    front_end_ic_error: bool
    software_mos_lock: bool
    charging_state: ChargingState | int
    soc_low_alarm: bool
    battery_low_alarm: bool
    highest_temperature_c: float
    lowest_temperature_c: float
    alive_counter: int
    checksum_valid: bool


@dataclass(frozen=True)
class UltrasonicFeedback:
    probe_distances_mm: tuple[int, int, int, int]
    alive_counter: int
    checksum_valid: bool

    @property
    def probe_distances_m(self) -> tuple[float, float, float, float]:
        return tuple(distance / 1000.0 for distance in self.probe_distances_mm)


@dataclass(frozen=True)
class OdometerFeedback:
    cumulative_distance_m: float
    raw_payload: bytes
    trailing_bytes_defined: bool = False


@dataclass(frozen=True)
class AliveObservation:
    value: int
    initialized: bool
    contiguous: bool
    expected_previous_successor: int | None


class AliveCounter:
    """Explicit caller-owned 4-bit transmit counter."""

    def __init__(self, initial: int = 0) -> None:
        self._validate(initial)
        self._value = initial

    @staticmethod
    def _validate(value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 15:
            raise ValueError("alive counter must be an integer in [0, 15]")

    @property
    def value(self) -> int:
        return self._value

    def next(self) -> int:
        value = self._value
        self._value = (self._value + 1) & 0x0F
        return value

    def increment(self) -> int:
        self._value = (self._value + 1) & 0x0F
        return self._value


class AliveTracker:
    """Feedback continuity tracker with first sample accepted at any value."""

    def __init__(self) -> None:
        self._last: int | None = None

    @property
    def last(self) -> int | None:
        return self._last

    def observe(self, value: int) -> AliveObservation:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 15:
            raise ValueError("alive counter must be an integer in [0, 15]")
        expected = None if self._last is None else (self._last + 1) & 0x0F
        contiguous = expected is None or value == expected
        observation = AliveObservation(
            value=value,
            initialized=self._last is None,
            contiguous=contiguous,
            expected_previous_successor=expected,
        )
        self._last = value
        return observation


def checksum(data: Iterable[int]) -> int:
    """Manufacturer BCC over exactly the first seven payload bytes."""

    values = tuple(data)
    if len(values) != 7:
        raise ValueError("checksum requires exactly seven bytes")
    if any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255 for value in values):
        raise ValueError("checksum input must contain byte integers")
    result = 0
    for value in values:
        result ^= value
    return result


def verify_checksum(frame: CanFrame | bytes | bytearray) -> bool:
    data = frame.data if isinstance(frame, CanFrame) else bytes(frame)
    return len(data) == 8 and data[7] == checksum(data[:7])


def _set_bits(word: int, start: int, length: int, value: int) -> int:
    mask = (1 << length) - 1
    return word | ((value & mask) << start)


def _get_bits(word: int, start: int, length: int) -> int:
    return (word >> start) & ((1 << length) - 1)


def _signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def _signed(value: int, width: int) -> int:
    sign_bit = 1 << (width - 1)
    return value - (1 << width) if value & sign_bit else value


def _quantize(value: float, scale: str, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    try:
        decimal_value = Decimal(str(value)) / Decimal(scale)
        return int(decimal_value.to_integral_value(rounding=ROUND_HALF_UP))
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{name} cannot be quantized") from exc


class MkminiCanCodec:
    """Stateless, transport-neutral manufacturer ctrl_cmd codec."""

    @staticmethod
    def encode(command: CtrlCommand) -> CanFrame:
        if not isinstance(command.gear, Gear):
            raise ValueError("unknown outgoing gear is rejected")
        if isinstance(command.speed_magnitude_mps, bool) or command.speed_magnitude_mps < 0:
            raise ValueError("speed magnitude must be non-negative")
        speed_raw = _quantize(command.speed_magnitude_mps, "0.001", "speed")
        steering_raw = _quantize(command.inner_wheel_steering_deg, "0.01", "steering")
        if not 0 <= speed_raw <= 0xFFFF:
            raise ValueError("speed is outside wire representation")
        if not -0x8000 <= steering_raw <= 0x7FFF:
            raise ValueError("steering is outside wire representation")
        if isinstance(command.alive_counter, bool) or not isinstance(command.alive_counter, int) or not 0 <= command.alive_counter <= 15:
            raise ValueError("alive counter must be an integer in [0, 15]")

        word = 0
        word = _set_bits(word, 0, 4, int(command.gear))
        word = _set_bits(word, 4, 16, speed_raw)
        word = _set_bits(word, 20, 16, steering_raw & 0xFFFF)
        word = _set_bits(word, 52, 4, command.alive_counter)
        payload = bytearray((word >> (8 * index)) & 0xFF for index in range(7))
        payload.append(checksum(payload))
        return CanFrame(can_id=CTRL_CMD_ID, is_extended=True, data=bytes(payload))


class ManufacturerCommandValidator:
    """Optional manufacturer-envelope checks kept separate from wire packing."""

    @staticmethod
    def validate(command: CtrlCommand) -> tuple[str, ...]:
        errors: list[str] = []
        if not isinstance(command.gear, Gear):
            errors.append("invalid_gear")
        if not isinstance(command.speed_magnitude_mps, (int, float)) or not math.isfinite(float(command.speed_magnitude_mps)):
            errors.append("nonfinite_speed")
        elif command.speed_magnitude_mps > MANUFACTURER_MAX_SPEED_MPS:
            errors.append("manufacturer_speed_limit")
        if not isinstance(command.inner_wheel_steering_deg, (int, float)) or not math.isfinite(float(command.inner_wheel_steering_deg)):
            errors.append("nonfinite_steering")
        elif not MANUFACTURER_MIN_STEERING_DEG <= command.inner_wheel_steering_deg <= MANUFACTURER_MAX_STEERING_DEG:
            errors.append("manufacturer_steering_limit")
        return tuple(errors)


class MkminiFeedbackCodec:
    """Pure decoders for the initial manufacturer feedback surfaces."""

    @staticmethod
    def _validate(frame: CanFrame, expected_id: int) -> None:
        if frame.can_id != expected_id:
            raise FrameValidationError(f"unexpected CAN ID 0x{frame.can_id:X}")
        if not frame.is_extended:
            raise FrameValidationError("feedback frame must be extended")
        if frame.dlc != 8:
            raise FrameValidationError("feedback frame must have DLC 8")

    @staticmethod
    def decode_ctrl_fb(frame: CanFrame) -> CtrlFeedback:
        MkminiFeedbackCodec._validate(frame, CTRL_FB_ID)
        word = int.from_bytes(frame.data[:7], "little")
        gear = _get_bits(word, 0, 4)
        mode_raw = _get_bits(word, 44, 2)
        mode: RunningMode | int = RunningMode(mode_raw) if mode_raw in (0, 1, 2) else mode_raw
        return CtrlFeedback(
            gear=gear,
            speed_magnitude_mps=_get_bits(word, 4, 16) * 0.001,
            inner_wheel_steering_deg=_signed16(_get_bits(word, 20, 16)) * 0.01,
            running_mode=mode,
            alive_counter=_get_bits(word, 52, 4),
            checksum_valid=verify_checksum(frame),
            reserved_bits_36_43=_get_bits(word, 36, 8),
            reserved_bits_46_51=_get_bits(word, 46, 6),
        )

    @staticmethod
    def _decode_wheel(frame: CanFrame, expected_id: int) -> WheelFeedback:
        MkminiFeedbackCodec._validate(frame, expected_id)
        speed_raw = int.from_bytes(frame.data[0:2], "little", signed=True)
        pulse_count = int.from_bytes(frame.data[2:6], "little", signed=True)
        return WheelFeedback(
            speed_mps=speed_raw * 0.001,
            pulse_count=pulse_count,
            alive_counter=(frame.data[6] >> 4) & 0x0F,
            checksum_valid=verify_checksum(frame),
        )

    @staticmethod
    def decode_lr_wheel_fb(frame: CanFrame) -> WheelFeedback:
        return MkminiFeedbackCodec._decode_wheel(frame, LR_WHEEL_FB_ID)

    @staticmethod
    def decode_rr_wheel_fb(frame: CanFrame) -> WheelFeedback:
        return MkminiFeedbackCodec._decode_wheel(frame, RR_WHEEL_FB_ID)

    @staticmethod
    def decode_vehicle_diagnostic(frame: CanFrame) -> VehicleDiagnosticFeedback:
        MkminiFeedbackCodec._validate(frame, VEH_DIAG_FB_ID)
        word = int.from_bytes(frame.data[:7], "little")
        return VehicleDiagnosticFeedback(
            vehicle_fault_level=_get_bits(word, 0, 4),
            auto_can_communication_error=bool(_get_bits(word, 4, 1)),
            auto_io_can_communication_error=bool(_get_bits(word, 5, 1)),
            eps_fault_code=_get_bits(word, 8, 12),
            left_drive_fault=_get_bits(word, 32, 6),
            right_drive_fault=_get_bits(word, 38, 6),
            bms_can_communication_loss=bool(_get_bits(word, 44, 1)),
            emergency_stop_asserted=bool(_get_bits(word, 45, 1)),
            remote_off_warning=bool(_get_bits(word, 46, 1)),
            remote_receiver_loss=bool(_get_bits(word, 47, 1)),
            alive_counter=_get_bits(word, 52, 4),
            checksum_valid=verify_checksum(frame),
            reserved_bits_48_51=_get_bits(word, 48, 4),
        )

    @staticmethod
    def decode_bms_info(frame: CanFrame) -> BmsInfoFeedback:
        MkminiFeedbackCodec._validate(frame, BMS_INFO_FB_ID)
        word = int.from_bytes(frame.data[:7], "little")
        return BmsInfoFeedback(
            pack_voltage_v=_get_bits(word, 0, 16) * 0.01,
            pack_current_a=_signed(_get_bits(word, 16, 16), 16) * 0.01,
            remaining_capacity_ah=_get_bits(word, 32, 16) * 0.01,
            alive_counter=_get_bits(word, 52, 4),
            checksum_valid=verify_checksum(frame),
        )

    @staticmethod
    def decode_bms_flags(frame: CanFrame) -> BmsFlagFeedback:
        MkminiFeedbackCodec._validate(frame, BMS_FLAG_FB_ID)
        word = int.from_bytes(frame.data[:7], "little")
        charging_raw = _get_bits(word, 21, 2)
        charging_state: ChargingState | int = (
            ChargingState(charging_raw) if charging_raw in range(4) else charging_raw
        )
        return BmsFlagFeedback(
            soc_percent=_get_bits(word, 0, 8),
            cell_over_voltage=bool(_get_bits(word, 8, 1)),
            cell_under_voltage=bool(_get_bits(word, 9, 1)),
            pack_over_voltage=bool(_get_bits(word, 10, 1)),
            pack_under_voltage=bool(_get_bits(word, 11, 1)),
            charge_over_temperature=bool(_get_bits(word, 12, 1)),
            charge_low_temperature=bool(_get_bits(word, 13, 1)),
            discharge_over_temperature=bool(_get_bits(word, 14, 1)),
            discharge_low_temperature=bool(_get_bits(word, 15, 1)),
            charge_over_current=bool(_get_bits(word, 16, 1)),
            discharge_over_current=bool(_get_bits(word, 17, 1)),
            short_circuit=bool(_get_bits(word, 18, 1)),
            front_end_ic_error=bool(_get_bits(word, 19, 1)),
            software_mos_lock=bool(_get_bits(word, 20, 1)),
            charging_state=charging_state,
            soc_low_alarm=bool(_get_bits(word, 23, 1)),
            battery_low_alarm=bool(_get_bits(word, 24, 1)),
            highest_temperature_c=_signed(_get_bits(word, 28, 12), 12) * 0.1,
            lowest_temperature_c=_signed(_get_bits(word, 40, 12), 12) * 0.1,
            alive_counter=_get_bits(word, 52, 4),
            checksum_valid=verify_checksum(frame),
        )

    @staticmethod
    def _decode_ultrasonic(frame: CanFrame, expected_id: int) -> UltrasonicFeedback:
        MkminiFeedbackCodec._validate(frame, expected_id)
        word = int.from_bytes(frame.data[:7], "little")
        return UltrasonicFeedback(
            probe_distances_mm=tuple(_get_bits(word, start, 12) for start in (0, 12, 24, 36)),
            alive_counter=_get_bits(word, 52, 4),
            checksum_valid=verify_checksum(frame),
        )

    @staticmethod
    def decode_ultrasonic_1(frame: CanFrame) -> UltrasonicFeedback:
        return MkminiFeedbackCodec._decode_ultrasonic(frame, ULTRASONIC_1_FB_ID)

    @staticmethod
    def decode_ultrasonic_2(frame: CanFrame) -> UltrasonicFeedback:
        return MkminiFeedbackCodec._decode_ultrasonic(frame, ULTRASONIC_2_FB_ID)

    @staticmethod
    def decode_odometer(frame: CanFrame) -> OdometerFeedback:
        MkminiFeedbackCodec._validate(frame, ODO_FB_ID)
        # The manual specifies only signed cumulative mileage in bits 0..31.
        # Remaining bytes have no documented field definitions and stay raw.
        distance_raw = int.from_bytes(frame.data[:4], "little", signed=True)
        return OdometerFeedback(
            cumulative_distance_m=distance_raw * 0.001,
            raw_payload=frame.data,
        )
