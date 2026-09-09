import struct
from dataclasses import replace

import pytest

from mkmini_cmd_adapter import (
    AdapterConfig,
    AdapterState,
    CtrlFeedback,
    Gear,
    HealthAssessment,
    HealthStatus,
    MkminiAdapterCore,
    MkminiCanCodec,
    MkminiKinematicsCore,
    PhysicalCommandModel,
    RealCanAuthorityError,
    RealTransportAuthority,
    RunningMode,
    SocketCanTxTransport,
    StationaryGearPolicy,
    ZeroOnlyFirstTxSession,
    ZeroOnlyFirstTxViolation,
    WheelFeedback,
)
from mkmini_cmd_adapter.codec import checksum
from mkmini_cmd_adapter.model import CanFrame, CtrlCommand


def qualified_test_authority() -> RealTransportAuthority:
    """Synthetic all-qualified authority; never used by a production default."""

    return RealTransportAuthority(
        protocol_profile_qualified=True,
        brake_profile_qualified=True,  # resolved zero-reserved authority
        stationary_policy_qualified=True,
        feedback_path_qualified=True,
        real_transport_qualified=True,
        feedback_transport_qualified=True,
        runtime_feedback_healthy=True,
    )


class FakeTxSocket:
    def __init__(self):
        self.bound = None
        self.sent = []
        self.closed = False

    def bind(self, address):
        self.bound = address

    def send(self, payload):
        self.sent.append(payload)
        return len(payload)

    def close(self):
        self.closed = True


class FakeSocketModule:
    AF_CAN = 29
    SOCK_RAW = 3
    CAN_RAW = 1


def fake_transport():
    sock = FakeTxSocket()
    transport = SocketCanTxTransport(
        "fake-can",
        socket_module=FakeSocketModule,
        socket_factory=lambda: sock,
        clock=lambda: 12.0,
        authority=qualified_test_authority(),
    )
    return transport, sock


def test_codec_golden_vectors_are_generated_by_codec_and_reserved_bits_zero():
    frame0 = MkminiCanCodec.encode(CtrlCommand(Gear.N, 0.0, 0.0, 0))
    frame1 = MkminiCanCodec.encode(CtrlCommand(Gear.N, 0.0, 0.0, 1))
    assert frame0.data.hex(" ") == "03 00 00 00 00 00 00 03"
    assert frame1.data.hex(" ") == "03 00 00 00 00 00 10 13"
    for frame in (frame0, frame1):
        word = int.from_bytes(frame.data[:7], "little")
        assert ((word >> 36) & 0xFFFF) == 0
        assert frame.data[7] == checksum(frame.data[:7])


def test_zero_only_requires_authority_before_transport_open():
    transport, sock = fake_transport()
    session = ZeroOnlyFirstTxSession(transport)
    with pytest.raises(RealCanAuthorityError, match="BRAKE_PROFILE_UNRESOLVED"):
        session.open()
    assert sock.bound is None
    assert sock.sent == []


def test_zero_only_accepts_only_centered_neutral_and_synthesizes_at_100hz():
    transport, sock = fake_transport()
    session = ZeroOnlyFirstTxSession(transport, authority=qualified_test_authority())
    session.open()
    session.request_neutral()
    emissions = [session.emit_due(index * 0.01) for index in range(18)]
    emissions = [item for item in emissions if item is not None]
    assert len(emissions) == 18
    assert [item.alive_counter for item in emissions] == [index & 0x0F for index in range(18)]
    assert sock.bound == (("fake-can",),)[0]
    assert len(sock.sent) == 18
    for emission in emissions:
        assert emission.frame.can_id == 0x18C4D2D0
        assert emission.frame.data[0] & 0x0F == Gear.N
        assert int.from_bytes(emission.frame.data[:7], "little") >> 36 & 0xFFFF == 0
        assert emission.frame.data[7] == checksum(emission.frame.data[:7])
    session.close()
    assert sock.closed


@pytest.mark.parametrize(
    "command,reason",
    [
        (PhysicalCommandModel(Gear.D, 0.0, 0.0), "N_GEAR"),
        (PhysicalCommandModel(Gear.R, 0.0, 0.0), "N_GEAR"),
        (PhysicalCommandModel(Gear.P, 0.0, 0.0), "N_GEAR"),
        (PhysicalCommandModel(Gear.N, 0.001, 0.0), "ZERO_SPEED"),
        (PhysicalCommandModel(Gear.N, 0.0, 0.01), "ZERO_STEERING"),
    ],
)
def test_zero_only_rejects_non_neutral_before_send(command, reason):
    transport, sock = fake_transport()
    session = ZeroOnlyFirstTxSession(transport, authority=qualified_test_authority())
    session.open()
    with pytest.raises(ZeroOnlyFirstTxViolation, match=reason):
        session.request(command)
    assert sock.sent == []


def test_tx_transport_uses_fake_socket_and_extended_frame_encoding():
    transport, sock = fake_transport()
    transport.open()
    frame = CanFrame(0x18C4D2D0, bytes.fromhex("03 00 00 00 00 00 00 03"), True)
    transport.send_frame(frame, timestamp=1.5)
    can_id, dlc, data = struct.unpack("=IB3x8s", sock.sent[0])
    assert can_id == 0x80000000 | 0x18C4D2D0
    assert dlc == 8
    assert data == frame.data
    transport.close()


def test_static_feedback_transport_capability_does_not_bypass_runtime_health():
    authority = replace(
        RealTransportAuthority.current_unqualified(),
        feedback_path_qualified=True,
        feedback_transport_qualified=True,
        runtime_feedback_healthy=False,
    )
    assert authority.feedback_transport_qualified is True
    assert authority.runtime_feedback_healthy is False
    assert authority.tx_qualified is False
    assert "REAL_CAN_BLOCKED_RUNTIME_FEEDBACK_UNHEALTHY" in authority.block_reasons()


def test_general_keyboard_tx_path_remains_blocked_even_with_synthetic_authority():
    from mkmini_cmd_adapter import KeyboardCommissioningCore, KeyboardFeedbackEvidence, KeyboardState, TransportMode

    evidence = KeyboardFeedbackEvidence(
        ctrl_current=True,
        diagnostic_current=True,
        ctrl_checksum_valid=True,
        diagnostic_checksum_valid=True,
        ctrl_alive_contiguous=True,
        diagnostic_alive_contiguous=True,
        mode=RunningMode.AUTO,
    )
    core = KeyboardCommissioningCore(transport_mode=TransportMode.TX_CAPABLE, authority=qualified_test_authority())
    core.observe_feedback(evidence)
    core.arm()
    result = core.set_command(0.1, 0.0, 0.0)
    assert result.accepted is False
    assert result.state is KeyboardState.FAULTED
    assert "GENERAL_KEYBOARD_TX_BLOCKED" in result.reason


def test_n_stationary_candidate_requires_feedback_proven_standstill():
    health = HealthAssessment(HealthStatus.VALID, True)
    ctrl = CtrlFeedback(0, 0.2, 0.0, RunningMode.AUTO, 0, True, 0, 0)
    wheel = WheelFeedback(0.2, 0, 0, True)
    core = MkminiAdapterCore(
        AdapterConfig(
            standstill_speed_threshold_mps=0.01,
            standstill_stability_sec=0.5,
            stationary_gear_policy=StationaryGearPolicy.N,
        )
    )
    moving = core.step(0.2, 0.1, 0.0, 0.0, health, gate=__import__("mkmini_cmd_adapter", fromlist=["GateContext"]).GateContext("ARMED"))
    stopping = core.step(0.0, 0.0, 1.0, 1.0, health, ctrl_feedback=ctrl, left_wheel_feedback=wheel, right_wheel_feedback=wheel)
    assert moving.state is AdapterState.MOVING_FORWARD
    assert stopping.state is AdapterState.STOPPING_FORWARD
    assert stopping.command.gear is Gear.D
    stopped = CtrlFeedback(0, 0.0, 0.0, RunningMode.AUTO, 0, True, 0, 0)
    zero_wheel = WheelFeedback(0.0, 0, 0, True)
    core.step(0.0, 0.0, 1.5, 1.5, health, ctrl_feedback=stopped, left_wheel_feedback=zero_wheel, right_wheel_feedback=zero_wheel)
    held = core.step(0.0, 0.0, 2.0, 2.0, health, ctrl_feedback=stopped, left_wheel_feedback=zero_wheel, right_wheel_feedback=zero_wheel)
    assert held.state is AdapterState.STATIONARY_HOLD
    assert held.command.gear is Gear.N
    assert held.command.speed_magnitude_mps == 0.0
    assert held.command.inner_wheel_steering_deg == 0.0


def test_alpha_kinematics_remains_project_authority_and_beta_is_not_imported():
    result = MkminiKinematicsCore().compute(0.2, 0.1)
    assert result.valid
    assert result.inner_steering_deg > 0.0
    source = __import__("pathlib").Path(__file__).parents[1].joinpath("mkmini_cmd_adapter", "kinematics.py").read_text()
    assert "beta" not in source.lower()


def test_k3b_encoder_scale_remains_unqualified():
    # The transport/session has no metric pulse conversion surface.
    assert not hasattr(ZeroOnlyFirstTxSession, "pulses_per_revolution")
