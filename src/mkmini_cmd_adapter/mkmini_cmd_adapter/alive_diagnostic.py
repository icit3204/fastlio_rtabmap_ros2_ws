"""Bounded receive-only raw ctrl-feedback alive sequence recorder."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from .codec import CTRL_FB_ID, CtrlFeedback
from .receive_only import ReceiveFilter, SocketCanReadOnlyTransport


def capture(interface: str, duration_sec: float) -> dict:
    receiver = SocketCanReadOnlyTransport(
        interface,
        filters=(ReceiveFilter(CTRL_FB_ID),),
        receive_timeout_sec=0.05,
    )
    rows = []
    previous = None
    previous_stamp = None
    receiver.open()
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration_sec:
            try:
                record = receiver.receive()
            except TimeoutError:
                continue
            if not isinstance(record.decoded, CtrlFeedback):
                continue
            alive = record.decoded.alive_counter
            rows.append({
                "timestamp_monotonic_sec": record.received_monotonic_sec,
                "can_id": f"0x{record.frame.can_id:08X}",
                "alive": alive,
                "previous_alive": previous,
                "delta_mod16": None if previous is None else (alive - previous) & 0x0F,
                "checksum_valid": record.decoded.checksum_valid,
                "inter_frame_interval_sec": (
                    None if previous_stamp is None else record.received_monotonic_sec - previous_stamp
                ),
                "payload_hex": record.frame.data.hex(),
            })
            previous = alive
            previous_stamp = record.received_monotonic_sec
    finally:
        receiver.close()
    deltas = [row["delta_mod16"] for row in rows if row["delta_mod16"] is not None]
    return {
        "mode": "RECEIVE_ONLY_NO_APPLICATION_CAN_TX",
        "interface": interface,
        "duration_sec": time.monotonic() - started,
        "ctrl_can_id": f"0x{CTRL_FB_ID:08X}",
        "counter_width_bits": 4,
        "rows": rows,
        "summary": {
            "frames": len(rows),
            "normal_delta_1": sum(delta == 1 for delta in deltas),
            "wrap_15_to_0": sum(
                current["previous_alive"] == 15 and current["alive"] == 0 for current in rows
            ),
            "repeated_delta_0": sum(delta == 0 for delta in deltas),
            "forward_skip_delta_2_to_5": sum(delta is not None and 2 <= delta <= 5 for delta in deltas),
            "implausible_or_backward_delta_6_to_15": sum(delta is not None and 6 <= delta <= 15 for delta in deltas),
            "checksum_invalid": sum(not row["checksum_valid"] for row in rows),
            "maximum_interval_sec": max(
                (row["inter_frame_interval_sec"] for row in rows if row["inter_frame_interval_sec"] is not None),
                default=None,
            ),
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    if not 1.0 <= args.duration_sec <= 30.0:
        parser.error("--duration-sec must be in [1, 30]")
    args.report.write_text(json.dumps(capture(args.interface, args.duration_sec), indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
