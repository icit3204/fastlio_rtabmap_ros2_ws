import ast
from pathlib import Path
import time

import pytest

from mkmini_cmd_adapter.backend_zero_heartbeat import (
    ACKNOWLEDGEMENT,
    R3lR2ZeroHeartbeatAuthority,
    POST_QUALIFICATION_STABILITY_SEC,
    _send_neutral,
)
from mkmini_cmd_adapter.codec import AliveCounter, CTRL_FB_ID, VEH_DIAG_FB_ID
from mkmini_cmd_adapter import CtrlFeedback, Gear, RunningMode, VehicleDiagnosticFeedback
from mkmini_cmd_adapter.backend_zero_heartbeat import LiveFeedbackMonitor
from mkmini_cmd_adapter.model import CanFrame
from mkmini_cmd_adapter.receive_only import CapturedFrame, RedundantSocketCanReadOnlyTransport


class FakeTx:
    def __init__(self):
        self.frames = []

    def send_frame(self, frame, timestamp=None):
        self.frames.append((frame, timestamp))


def decode(frame):
    word = int.from_bytes(frame.data[:7], "little")
    return word & 0xF, (word >> 4) & 0xFFFF, (word >> 20) & 0xFFFF, (word >> 36) & 0xFFFF


def test_exact_operator_ack_is_required_before_tx():
    with pytest.raises(RuntimeError, match="AUTHORIZATION_REQUIRED"):
        R3lR2ZeroHeartbeatAuthority("").require_tx()
    with pytest.raises(RuntimeError, match="AUTHORIZATION_REQUIRED"):
        R3lR2ZeroHeartbeatAuthority(ACKNOWLEDGEMENT.upper()).require_tx()
    R3lR2ZeroHeartbeatAuthority(ACKNOWLEDGEMENT).require_tx()
    assert POST_QUALIFICATION_STABILITY_SEC == 10.0


def test_only_emission_helper_is_codec_generated_n_zero_zero():
    tx = FakeTx()
    alive = AliveCounter(15)
    _send_neutral(tx, alive, 1.0)
    _send_neutral(tx, alive, 1.01)
    assert [decode(item[0]) for item in tx.frames] == [(3, 0, 0, 0), (3, 0, 0, 0)]
    assert [((int.from_bytes(item[0].data[:7], "little") >> 52) & 0xF) for item in tx.frames] == [15, 0]


def test_cli_has_no_motion_arguments_or_motion_command_literals():
    path = Path(__file__).parents[1] / "mkmini_cmd_adapter" / "backend_zero_heartbeat.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    option_strings = {
        arg.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "add_argument"
        for arg in node.args if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
    }
    assert "--speed" not in option_strings
    assert "--steering" not in option_strings
    assert "--gear" not in option_strings
    assert "Gear.D" not in path.read_text(encoding="utf-8")
    assert "Gear.R" not in path.read_text(encoding="utf-8")


def test_concurrent_receipt_after_caller_timestamp_never_creates_negative_age():
    monitor = LiveFeedbackMonitor("unused")
    monitor._ctrl = CtrlFeedback(3, 0.0, 0.0, RunningMode.AUTO, 1, True, 0, 0)
    monitor._diagnostic = VehicleDiagnosticFeedback(1, False, True, 0, 0, 0, False, False, True, False, 1, True, 0)
    caller_now = time.monotonic()
    monitor._ctrl_stamp = caller_now + 0.001
    monitor._diagnostic_stamp = caller_now + 0.001
    monitor._counts.update(ctrl=2, diagnostic=2)
    while time.monotonic() < monitor._ctrl_stamp:
        pass
    feedback, _ = monitor.snapshot(caller_now, {
        "linkinfo": {"info_data": {"state": "ERROR-ACTIVE", "berr_counter": {"tx": 0, "rx": 0}},
                     "info_xstats": {"restarts": 0, "bus_error": 0, "arbitration_lost": 0,
                                     "error_warning": 0, "error_passive": 0, "bus_off": 0}}
    })
    assert feedback.ctrl_age_sec >= 0.0
    assert feedback.diagnostic_age_sec >= 0.0


def test_alive_reset_creates_a_new_pre_tx_qualification_epoch_only():
    monitor = LiveFeedbackMonitor("unused")
    monitor._counts.update(ctrl=99, diagnostic=99)
    monitor._ctrl_contiguous = False
    monitor._diagnostic_contiguous = False
    monitor._ctrl_alive_epoch_count = 17
    monitor._diagnostic_alive_epoch_count = 17

    monitor.reset_alive_epoch()

    assert monitor._counts["ctrl"] == 99  # raw evidence remains available
    assert monitor._counts["diagnostic"] == 99
    assert monitor._ctrl_contiguous is True
    assert monitor._diagnostic_contiguous is True
    assert monitor._ctrl_alive_epoch_count == 0
    assert monitor._diagnostic_alive_epoch_count == 0


def test_live_monitor_uses_redundant_buffered_kernel_timestamp_receiver():
    monitor = LiveFeedbackMonitor("unused")
    assert isinstance(monitor._receiver, RedundantSocketCanReadOnlyTransport)
    record = CapturedFrame(
        CanFrame(0x123, bytes(8)),
        received_monotonic_sec=50.0,
        kernel_timestamp_ns=1_234_567_890,
    )
    assert monitor._sequence_timestamp(record) == pytest.approx(1.234567890)


def test_physical_safety_monitor_filters_to_interlock_frames():
    monitor = LiveFeedbackMonitor("unused", safety_critical_only=True)
    assert tuple(item.can_id for item in monitor._receiver._filters) == (
        CTRL_FB_ID, VEH_DIAG_FB_ID)
