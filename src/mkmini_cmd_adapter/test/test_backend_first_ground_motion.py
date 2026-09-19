import ast
from pathlib import Path

import pytest

from mkmini_cmd_adapter import AliveCounter, BackendCommand, Gear
from mkmini_cmd_adapter.backend_interlock import BackendFeedback, BackendInterlockResult, BackendInterlockState, MkminiBackendInterlock
from mkmini_cmd_adapter.backend_first_ground_motion import (
    ACKNOWLEDGEMENT,
    COMMAND_SPEED_MPS,
    COMMISSIONING_FEEDBACK_CEILING_MPS,
    MAX_COMMAND_DURATION_SEC,
    COMMISSIONING_SPEED_CEILING_MPS,
    R3lR9GroundAuthority,
    R9_INTERLOCK_CONFIG,
    TARGET_DISTANCE_M,
    _send,
    commissioning_drive_command,
    permission_drop_trace,
)
from mkmini_cmd_adapter.backend_receive_only import _jsonable
from mkmini_cmd_adapter import OdometerFeedback


class FakeTx:
    def __init__(self):
        self.frames = []
    def send_frame(self, frame, timestamp=None):
        self.frames.append(frame)


def decode(frame):
    word = int.from_bytes(frame.data[:7], "little")
    return word & 0xF, ((word >> 4) & 0xFFFF) * 0.001, ((word >> 20) & 0xFFFF)


def test_exact_ground_ack_required():
    with pytest.raises(RuntimeError, match="AUTHORIZATION_REQUIRED"):
        R3lR9GroundAuthority("").require_tx()
    R3lR9GroundAuthority(ACKNOWLEDGEMENT).require_tx()


def test_fixed_envelope_is_forward_only_and_bounded_distance():
    assert COMMAND_SPEED_MPS == 0.04
    assert COMMISSIONING_SPEED_CEILING_MPS == 0.04
    assert COMMISSIONING_FEEDBACK_CEILING_MPS == 0.060
    assert TARGET_DISTANCE_M == 0.22
    assert 0.20 <= TARGET_DISTANCE_M <= 0.30
    assert COMMAND_SPEED_MPS * MAX_COMMAND_DURATION_SEC <= 0.30


def test_commissioning_speed_is_bounded_below_feedback_safety_ceiling():
    assert commissioning_drive_command(0.04) == BackendCommand(Gear.D, 0.04, 0.0)
    with pytest.raises(ValueError, match="exceeds"):
        commissioning_drive_command(0.040001)


def test_r9_feedback_envelope_is_runner_local_and_still_fail_closed():
    # The production default stays at 0.050; R9's empirical 0.060 applies
    # only to this independently cross-checked commissioning runner.
    assert MkminiBackendInterlock().config.maximum_motion_speed_mps == 0.05
    core = MkminiBackendInterlock(R9_INTERLOCK_CONFIG)
    assert core.config.maximum_motion_speed_mps == 0.060
    good = BackendFeedback(
        ctrl_age_sec=.001, diagnostic_age_sec=.001, ctrl_checksum_valid=True,
        diagnostic_checksum_valid=True, ctrl_alive_contiguous=True,
        diagnostic_alive_contiguous=True, gear=Gear.N, speed_mps=0.0,
        steering_deg=0.0, mode=0, vehicle_fault_level=1,
        auto_can_error=False, can_state="ERROR-ACTIVE", hard_fault=False,
    )
    core.arm(0.0)
    for tick in range(81):
        core.step(tick * .01, BackendCommand.safe_neutral(), good)
    permitted = core.step(.81, BackendCommand(Gear.D, .04, .0),
                          BackendFeedback(**{**good.__dict__, "gear": Gear.D, "speed_mps": .060}))
    assert permitted.motion_permitted
    tripped = core.step(.82, BackendCommand(Gear.D, .04, .0),
                         BackendFeedback(**{**good.__dict__, "gear": Gear.D, "speed_mps": .061}))
    assert not tripped.motion_permitted
    assert "FEEDBACK_MOTION_OVERSPEED" in tripped.reasons


def test_r9_pass_envelope_uses_feedback_ceiling_not_command_value():
    source = (Path(__file__).parents[1] / "mkmini_cmd_adapter" / "backend_first_ground_motion.py").read_text()
    assert "max_actual_speed <= COMMISSIONING_FEEDBACK_CEILING_MPS" in source


def test_sender_uses_only_supplied_interlocked_fixed_commands():
    tx = FakeTx()
    alive = AliveCounter()
    _send(tx, alive, BackendCommand(Gear.D, 0.05, 0.0), 1.0)
    _send(tx, alive, BackendCommand.safe_neutral(), 1.01)
    assert decode(tx.frames[0]) == (4, 0.05, 0)
    assert decode(tx.frames[1]) == (3, 0.0, 0)


def test_cli_exposes_no_speed_distance_steering_or_gear_controls():
    path = Path(__file__).parents[1] / "mkmini_cmd_adapter" / "backend_first_ground_motion.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    options = {arg.value for node in ast.walk(tree) if isinstance(node, ast.Call)
               and isinstance(node.func, ast.Attribute) and node.func.attr == "add_argument"
               for arg in node.args if isinstance(arg, ast.Constant) and isinstance(arg.value, str)}
    assert options == {"--interface", "--operator-ack", "--report"}
    assert "Gear.R" not in path.read_text(encoding="utf-8")


def test_odometer_raw_bytes_are_evidence_serializable():
    assert _jsonable(OdometerFeedback(1.25, b"\x01\x02"))["raw_payload"] == "0102"


def test_permission_drop_trace_preserves_exact_chassis_contract_reason():
    feedback = BackendFeedback(
        ctrl_age_sec=0.004, diagnostic_age_sec=0.005,
        ctrl_checksum_valid=True, diagnostic_checksum_valid=True,
        ctrl_alive_contiguous=True, diagnostic_alive_contiguous=True,
        gear=Gear.D, speed_mps=0.051, steering_deg=0.0, mode=2,
        vehicle_fault_level=1, auto_can_error=False, can_state="ERROR-ACTIVE",
        hard_fault=False,
    )
    result = BackendInterlockResult(
        BackendInterlockState.MOTION_NOT_PERMITTED, False, False,
        BackendCommand.safe_neutral(), ("FEEDBACK_MOTION_OVERSPEED",), 0.0,
    )
    trace = permission_drop_trace(
        timestamp_monotonic_sec=123.0,
        previous_state=BackendInterlockState.MOTION_PERMITTED,
        result=result, command=BackendCommand(Gear.D, 0.05, 0.0), feedback=feedback,
    )
    assert trace["transition_reason"] == ["FEEDBACK_MOTION_OVERSPEED"]
    assert trace["requested_gear"] == Gear.D.value
    assert trace["feedback_gear"] == Gear.D.value
    assert trace["actual_speed_mps"] == 0.051
    assert trace["can_health"] == "ERROR-ACTIVE"
