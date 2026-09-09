"""Explicit CLI for one bounded MK2E2A zero-speed candidate trial.

Without ``--execute`` this prints the requested plan and never opens CAN.  An
executing run first requires a strict, passive healthy baseline; STOP mode,
any reported diagnostic fault, and Auto-CAN/Auto-IO/remote warnings all block
TX.  It is intentionally independent of ROS, Nav2, keyboard control, and the
production adapter.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

from .codec import AliveTracker, CtrlFeedback, MkminiFeedbackCodec, RunningMode, VehicleDiagnosticFeedback
from .model import Gear
from .receive_only import SocketCanReadOnlyTransport
from .tx import SocketCanTxTransport
from .zero_speed_adjudication import Mk2E2aZeroSpeedAuthority, ZeroSpeedGearSession


_ACK = "I_CONFIRM_WHEELS_OFF_FLOOR_ZERO_SPEED_ONLY"
_GEARS = {"disable": Gear.DISABLE, "n": Gear.N, "d": Gear.D, "p": Gear.P}


def _json(value: Any) -> Any:
    if hasattr(value, "value"):
        return value.value
    if hasattr(value, "__dataclass_fields__"):
        return {key: _json(item) for key, item in vars(value).items()}
    if isinstance(value, (tuple, list)):
        return [_json(item) for item in value]
    return value


def _capture_baseline(interface: str, duration_sec: float) -> tuple[dict[str, Any], tuple[str, ...]]:
    ctrl: CtrlFeedback | None = None
    diagnostic: VehicleDiagnosticFeedback | None = None
    counts = {"ctrl": 0, "diagnostic": 0, "wheel": 0}
    valid = {"ctrl": True, "diagnostic": True}
    trackers = {"ctrl": AliveTracker(), "diagnostic": AliveTracker()}
    contiguous = {"ctrl": True, "diagnostic": True}
    last_wheels: list[Any] = []
    receiver = SocketCanReadOnlyTransport(interface, receive_timeout_sec=0.05)
    receiver.open()
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration_sec:
            try:
                record = receiver.receive()
            except TimeoutError:
                continue
            decoded = record.decoded
            if isinstance(decoded, CtrlFeedback):
                ctrl = decoded
                counts["ctrl"] += 1
                valid["ctrl"] = valid["ctrl"] and decoded.checksum_valid
                contiguous["ctrl"] = contiguous["ctrl"] and trackers["ctrl"].observe(decoded.alive_counter).contiguous
            elif isinstance(decoded, VehicleDiagnosticFeedback):
                diagnostic = decoded
                counts["diagnostic"] += 1
                valid["diagnostic"] = valid["diagnostic"] and decoded.checksum_valid
                contiguous["diagnostic"] = contiguous["diagnostic"] and trackers["diagnostic"].observe(decoded.alive_counter).contiguous
            elif decoded is not None and decoded.__class__.__name__ == "WheelFeedback":
                counts["wheel"] += 1
                last_wheels.append(decoded)
                last_wheels = last_wheels[-2:]
    finally:
        receiver.close()

    reasons: list[str] = []
    if counts["ctrl"] < 10 or ctrl is None:
        reasons.append("CTRL_FB_MISSING_OR_INSUFFICIENT")
    if counts["diagnostic"] < 10 or diagnostic is None:
        reasons.append("VEH_DIAG_MISSING_OR_INSUFFICIENT")
    if not valid["ctrl"] or not valid["diagnostic"]:
        reasons.append("FEEDBACK_CHECKSUM_INVALID")
    if not contiguous["ctrl"] or not contiguous["diagnostic"]:
        reasons.append("FEEDBACK_ALIVE_NOT_CONTIGUOUS")
    if ctrl is not None and ctrl.running_mode is not RunningMode.AUTO:
        reasons.append("VCU_NOT_AUTO")
    if diagnostic is not None:
        if diagnostic.vehicle_fault_level != 0:
            reasons.append("VEHICLE_FAULT_LEVEL_NONZERO")
        if diagnostic.auto_can_communication_error:
            reasons.append("AUTO_CAN_COMMUNICATION_ERROR")
        if diagnostic.auto_io_can_communication_error:
            reasons.append("AUTO_IO_CAN_COMMUNICATION_ERROR")
        if diagnostic.remote_off_warning or diagnostic.remote_receiver_loss:
            reasons.append("REMOTE_NOT_HEALTHY")
        if diagnostic.emergency_stop_asserted or diagnostic.eps_fault_code != 0:
            reasons.append("ESTOP_OR_EPS_FAULT")
        if diagnostic.left_drive_fault != 0 or diagnostic.right_drive_fault != 0 or diagnostic.bms_can_communication_loss:
            reasons.append("DRIVE_OR_BMS_FAULT")
    summary = {
        "duration_sec": duration_sec,
        "counts": counts,
        "ctrl": None if ctrl is None else _json(ctrl),
        "diagnostic": None if diagnostic is None else _json(diagnostic),
        "wheels": [_json(item) for item in last_wheels],
        "checksums_all_valid": valid,
        "alive_contiguous": contiguous,
    }
    return summary, tuple(dict.fromkeys(reasons))


def _frame_record(emission) -> dict[str, Any]:
    word = int.from_bytes(emission.frame.data[:7], "little")
    return {
        "monotonic_sec": emission.timestamp_sec,
        "can_id": f"0x{emission.frame.can_id:08X}",
        "extended": emission.frame.is_extended,
        "dlc": emission.frame.dlc,
        "payload_hex": emission.frame.data.hex(" "),
        "gear": emission.gear.name,
        "speed_mps": 0.0,
        "steering_deg": 0.0,
        "alive": emission.alive_counter,
        "reserved_bits_36_51": (word >> 36) & 0xFFFF,
        "checksum_valid": emission.frame.data[7] == (emission.frame.data[0] ^ emission.frame.data[1] ^ emission.frame.data[2] ^ emission.frame.data[3] ^ emission.frame.data[4] ^ emission.frame.data[5] ^ emission.frame.data[6]),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gear", required=True, choices=tuple(_GEARS))
    parser.add_argument("--duration-sec", type=float, default=0.50)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--operator-ack", default="")
    arguments = parser.parse_args(argv)
    if not math.isfinite(arguments.duration_sec) or not 0.10 <= arguments.duration_sec <= 2.0:
        parser.error("--duration-sec must be finite and in [0.10, 2.0]")
    if arguments.execute and arguments.operator_ack != _ACK:
        parser.error("--execute requires the exact wheels-off-floor acknowledgement")

    report: dict[str, Any] = {
        "task": "P5A-MK2E2A",
        "requested_gear": _GEARS[arguments.gear].name,
        "speed_mps": 0.0,
        "steering_deg": 0.0,
        "duration_sec": arguments.duration_sec,
        "interface": arguments.interface,
        "execute": arguments.execute,
    }
    if not arguments.execute:
        report["status"] = "PLAN_ONLY_NO_CAN_SOCKET_OPENED"
        arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 0

    baseline, reasons = _capture_baseline(arguments.interface, 5.0)
    report["passive_baseline"] = baseline
    if reasons:
        report["status"] = "BLOCKED_UNHEALTHY_VCU_BASELINE"
        report["block_reasons"] = list(reasons)
        arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 2

    transport = SocketCanTxTransport(arguments.interface, authority=None)
    # SocketCanTxTransport's broader historical authority intentionally does
    # not apply here.  This runner is task-scoped and opens only after the
    # explicit acknowledgement plus strict live baseline above.
    transport.authority = Mk2E2aZeroSpeedAuthority(acknowledged=True)
    session = ZeroSpeedGearSession(
        transport,
        gear=_GEARS[arguments.gear],
        authority=Mk2E2aZeroSpeedAuthority(acknowledged=True),
        output_hz=100.0,
    )
    emissions = []
    session.open()
    try:
        start = time.monotonic()
        count = int(math.ceil(arguments.duration_sec * 100.0))
        for index in range(count):
            due = start + index * 0.01
            remaining = due - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            emissions.append(_frame_record(session.emit(time.monotonic())))
    finally:
        session.close()
    report["status"] = "ZERO_SPEED_BURST_COMPLETE"
    report["frames"] = emissions
    arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
