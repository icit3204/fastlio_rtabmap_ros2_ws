"""Bounded receive-only validation for the MK-mini backend interlock.

This executable imports no TX transport and has no command-line switch that
can transmit.  It captures installed-unit feedback, verifies SocketCAN state,
and asks the production interlock core to classify the final live snapshot.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import time

from .backend_interlock import BackendFeedback, MkminiBackendInterlock
from .codec import AliveTracker, CtrlFeedback, VehicleDiagnosticFeedback
from .model import Gear
from .receive_only import SocketCanReadOnlyTransport


def _can_link(interface: str) -> dict:
    completed = subprocess.run(
        ["ip", "-details", "-statistics", "-json", "link", "show", interface],
        check=True,
        capture_output=True,
        text=True,
        timeout=2.0,
    )
    values = json.loads(completed.stdout)
    if len(values) != 1:
        raise RuntimeError("CAN interface lookup did not return exactly one link")
    return values[0]


def _jsonable(value):
    if isinstance(value, bytes):
        return value.hex()
    if hasattr(value, "value"):
        return value.value
    if hasattr(value, "__dataclass_fields__"):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def capture_live_snapshot(interface: str, duration_sec: float) -> tuple[BackendFeedback, dict]:
    link_before = _can_link(interface)
    info_before = link_before.get("linkinfo", {}).get("info_data", {})
    stats_before = link_before.get("stats64", {})
    ctrl = None
    diagnostic = None
    ctrl_stamp = None
    diagnostic_stamp = None
    ctrl_tracker = AliveTracker()
    diagnostic_tracker = AliveTracker()
    continuity = {"ctrl": True, "diagnostic": True}
    counts = {"ctrl": 0, "diagnostic": 0, "other": 0, "decode_error": 0}
    receiver = SocketCanReadOnlyTransport(interface, receive_timeout_sec=0.05)
    receiver.open()
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration_sec:
            try:
                record = receiver.receive()
            except TimeoutError:
                continue
            if record.decode_error is not None:
                counts["decode_error"] += 1
            elif isinstance(record.decoded, CtrlFeedback):
                ctrl = record.decoded
                ctrl_stamp = record.received_monotonic_sec
                counts["ctrl"] += 1
                continuity["ctrl"] &= ctrl_tracker.observe(ctrl.alive_counter).contiguous
            elif isinstance(record.decoded, VehicleDiagnosticFeedback):
                diagnostic = record.decoded
                diagnostic_stamp = record.received_monotonic_sec
                counts["diagnostic"] += 1
                continuity["diagnostic"] &= diagnostic_tracker.observe(diagnostic.alive_counter).contiguous
            else:
                counts["other"] += 1
    finally:
        receiver.close()
    ended = time.monotonic()
    link_after = _can_link(interface)
    info_after = link_after.get("linkinfo", {}).get("info_data", {})
    stats_after = link_after.get("stats64", {})

    hard_fault = True
    if diagnostic is not None:
        hard_fault = any((
            diagnostic.emergency_stop_asserted,
            diagnostic.eps_fault_code != 0,
            diagnostic.left_drive_fault != 0,
            diagnostic.right_drive_fault != 0,
            diagnostic.bms_can_communication_loss,
        ))
    gear = None
    if ctrl is not None and ctrl.gear in tuple(int(item) for item in Gear):
        gear = Gear(ctrl.gear)
    feedback = BackendFeedback(
        ctrl_age_sec=None if ctrl_stamp is None else ended - ctrl_stamp,
        diagnostic_age_sec=None if diagnostic_stamp is None else ended - diagnostic_stamp,
        ctrl_checksum_valid=ctrl is not None and ctrl.checksum_valid,
        diagnostic_checksum_valid=diagnostic is not None and diagnostic.checksum_valid,
        ctrl_alive_contiguous=counts["ctrl"] >= 2 and continuity["ctrl"],
        diagnostic_alive_contiguous=counts["diagnostic"] >= 2 and continuity["diagnostic"],
        gear=gear,
        speed_mps=None if ctrl is None else ctrl.speed_magnitude_mps,
        steering_deg=None if ctrl is None else ctrl.inner_wheel_steering_deg,
        mode=None if ctrl is None else ctrl.running_mode,
        vehicle_fault_level=None if diagnostic is None else diagnostic.vehicle_fault_level,
        auto_can_error=None if diagnostic is None else diagnostic.auto_can_communication_error,
        can_state=str(info_after.get("state", "UNKNOWN")),
        hard_fault=hard_fault,
    )
    report = {
        "mode": "RECEIVE_ONLY_NO_APPLICATION_CAN_TX",
        "interface": interface,
        "duration_sec": ended - started,
        "counts": counts,
        "continuity": continuity,
        "ctrl_feedback": _jsonable(ctrl),
        "diagnostic_feedback": _jsonable(diagnostic),
        "can_before": {"info": info_before, "stats": stats_before},
        "can_after": {"info": info_after, "stats": stats_after},
        "feedback_snapshot": _jsonable(feedback),
    }
    return feedback, report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if not 1.0 <= arguments.duration_sec <= 30.0:
        parser.error("--duration-sec must be in [1, 30]")
    feedback, report = capture_live_snapshot(arguments.interface, arguments.duration_sec)
    interlock = MkminiBackendInterlock()
    report["base_feedback_reasons"] = list(interlock.assess_feedback(feedback, require_n_auto=False))
    report["motion_qualification_reasons"] = list(interlock.assess_feedback(feedback, require_n_auto=True))
    report["motion_permitted"] = False
    report["interlock_state"] = "DISARMED"
    report["application_can_tx_opened"] = False
    arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if not report["base_feedback_reasons"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
