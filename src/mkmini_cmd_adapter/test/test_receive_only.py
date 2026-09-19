import io
import json
import socket
import struct
import time

import pytest

from mkmini_cmd_adapter import (
    CAN_EFF_FLAG,
    CaptureAccumulator,
    JsonlCaptureArchive,
    RedundantSocketCanReadOnlyTransport,
    SocketCanReadOnlyTransport,
    TransportMode,
    MkminiCanCodec,
    CtrlCommand,
    Gear,
    feedback_filters,
    linux_filter_bytes,
    capture_frames,
)
from mkmini_cmd_adapter.codec import CTRL_CMD_ID, CTRL_FB_ID
from mkmini_cmd_adapter.receive_only import FEEDBACK_IDS
from mkmini_cmd_adapter.receive_only import ReceiveFilter
from mkmini_cmd_adapter.model import CanFrame
from mkmini_cmd_adapter.receive_only import _captured_from_wire


class FakeSocket:
    def __init__(self, raw_frames):
        self.raw_frames = list(raw_frames)
        self.bound = None
        self.sockopt = None
        self.closed = False

    def bind(self, address):
        self.bound = address

    def setsockopt(self, level, option, value):
        self.sockopt = (level, option, value)

    def recv(self, size):
        assert size == 16
        return self.raw_frames.pop(0)

    def close(self):
        self.closed = True


class FakeSocketApi:
    AF_CAN = 29
    SOCK_RAW = 3
    CAN_RAW = 1
    SOL_CAN_RAW = 101
    CAN_RAW_FILTER = 2


class FakeRecvmsgSocket(FakeSocket):
    def __init__(self, raw_frames, kernel_timestamps):
        super().__init__(raw_frames)
        self.kernel_timestamps = list(kernel_timestamps)
        self.timeout = None
        self.options = []

    def settimeout(self, value):
        self.timeout = value

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))

    def recvmsg(self, size, ancillary_size):
        if not self.raw_frames:
            raise socket.timeout()
        raw = self.raw_frames.pop(0)
        kernel_ns = self.kernel_timestamps.pop(0)
        seconds, nanoseconds = divmod(kernel_ns, 1_000_000_000)
        ancillary = [(socket.SOL_SOCKET, 35, struct.pack("=qq", seconds, nanoseconds))]
        return raw, ancillary, 0, ("test0",)


class FakeRedundantSocketApi(FakeSocketApi):
    SOL_SOCKET = socket.SOL_SOCKET
    SO_RCVBUF = socket.SO_RCVBUF


def raw_frame(can_id, payload, extended=True):
    wire_id = can_id | CAN_EFF_FLAG if extended else can_id
    return struct.pack("=IB3x8s", wire_id, len(payload), payload.ljust(8, b"\0"))


def valid_ctrl(alive=0):
    command = CtrlCommand(Gear.D, 0.0, 0.0, alive)
    return MkminiCanCodec.encode(command).data


def test_filters_are_extended_feedback_only_and_exclude_command_id():
    assert tuple(item.can_id for item in feedback_filters()) == FEEDBACK_IDS
    assert 0x18C4D2D0 not in FEEDBACK_IDS
    encoded = linux_filter_bytes()
    values = [struct.unpack("=II", encoded[index:index + 8]) for index in range(0, len(encoded), 8)]
    assert [can_id & 0x1FFFFFFF for can_id, _ in values] == list(FEEDBACK_IDS)
    assert all(can_id & CAN_EFF_FLAG for can_id, _ in values)
    assert all(mask & CAN_EFF_FLAG for _, mask in values)
    assert linux_filter_bytes(()) == b""


def test_fake_socket_open_receive_timestamp_and_close():
    fake = FakeSocket([raw_frame(CTRL_FB_ID, valid_ctrl(1))])
    factory_calls = []

    def factory(*args):
        factory_calls.append(args)
        return fake

    transport = SocketCanReadOnlyTransport(
        "test0",
        socket_module=FakeSocketApi,
        socket_factory=factory,
        clock=lambda: 12.5,
    )
    assert not transport.is_open
    transport.open()
    assert transport.is_open
    assert fake.bound == ("test0",)
    assert factory_calls == [(FakeSocketApi.AF_CAN, FakeSocketApi.SOCK_RAW, FakeSocketApi.CAN_RAW)]
    captured = transport.receive()
    assert captured.frame.can_id == CTRL_FB_ID
    assert captured.frame.is_extended is True
    assert captured.received_monotonic_sec == 12.5
    assert captured.decoded.checksum_valid is True
    assert not hasattr(transport, "send")
    transport.close()
    assert fake.closed is True
    assert not transport.is_open


def test_command_id_can_only_be_selected_as_an_explicit_passive_receive_filter():
    encoded = linux_filter_bytes((ReceiveFilter(CTRL_CMD_ID),))
    can_id, mask = struct.unpack("=II", encoded)
    assert can_id == CTRL_CMD_ID | CAN_EFF_FLAG
    assert mask == CAN_EFF_FLAG | 0x1FFFFFFF


def test_unknown_and_bad_checksum_frames_are_preserved():
    unknown = _captured_from_wire(raw_frame(0x123, b"unknown!", extended=True), 1.0)
    assert unknown.frame.can_id == 0x123
    assert unknown.decoded is None
    assert unknown.decode_error is None
    bad = bytearray(valid_ctrl(2))
    bad[7] ^= 0xFF
    captured = _captured_from_wire(raw_frame(CTRL_FB_ID, bytes(bad)), 2.0)
    assert captured.decoded is not None
    assert captured.decoded.checksum_valid is False
    assert captured.decode_error is None


def test_capture_summary_archive_and_alive_evidence():
    first = _captured_from_wire(raw_frame(CTRL_FB_ID, valid_ctrl(0)), 1.0)
    second = _captured_from_wire(raw_frame(CTRL_FB_ID, valid_ctrl(1)), 1.1)
    accumulator = CaptureAccumulator()
    accumulator.observe(first)
    accumulator.observe(second)
    summary = accumulator.by_id[CTRL_FB_ID]
    assert summary.count == 2
    assert summary.rate_hz == pytest.approx(10.0)
    assert summary.alive_values == [0, 1]
    assert summary.checksum_valid_count == 2
    assert summary.checksum_invalid_count == 0
    assert summary.payload_variants == 2

    stream = io.StringIO()
    archive = JsonlCaptureArchive(stream)
    archive.append(first)
    record = json.loads(stream.getvalue())
    assert record["can_id"] == CTRL_FB_ID
    assert record["payload_hex"] == first.frame.data.hex()
    assert record["received_monotonic_sec"] == 1.0


def test_archive_normalizes_bytes_from_decoded_feedback():
    odometer = _captured_from_wire(raw_frame(0x18C4DEEF, b"\x01\x00\x00\x00\x02\x03\x04\x05"), 3.0)
    stream = io.StringIO()
    JsonlCaptureArchive(stream).append(odometer)
    record = json.loads(stream.getvalue())
    assert record["decoded"]["raw_payload"] == "0100000002030405"


def test_archive_preserves_redundant_receiver_provenance():
    original = _captured_from_wire(raw_frame(CTRL_FB_ID, valid_ctrl(3)), 3.5)
    captured = type(original)(
        original.frame,
        original.received_monotonic_sec,
        original.decoded,
        original.decode_error,
        kernel_timestamp_ns=123456789,
        receive_source="rx1",
        rxq_overflow_total=0,
    )
    stream = io.StringIO()
    JsonlCaptureArchive(stream).append(captured)
    record = json.loads(stream.getvalue())
    assert record["kernel_timestamp_ns"] == 123456789
    assert record["receive_source"] == "rx1"
    assert record["rxq_overflow_total"] == 0


def test_bounded_capture_utility_uses_only_injected_receiver():
    fake = FakeSocket([raw_frame(CTRL_FB_ID, valid_ctrl(0))])
    transport = SocketCanReadOnlyTransport(
        "test0",
        socket_module=FakeSocketApi,
        socket_factory=lambda *args: fake,
        clock=lambda: 4.0,
    )
    transport.open()
    accumulator = capture_frames(transport, 1)
    assert accumulator.by_id[CTRL_FB_ID].count == 1
    transport.close()


def test_redundant_receiver_deduplicates_kernel_identical_frames_and_has_no_send():
    wire0 = raw_frame(CTRL_FB_ID, valid_ctrl(0))
    wire1 = raw_frame(CTRL_FB_ID, valid_ctrl(1))
    sockets = [
        FakeRecvmsgSocket([wire0, wire1], [1_000_000_000, 1_010_000_000]),
        FakeRecvmsgSocket([wire0, wire1], [1_000_000_000, 1_010_000_000]),
    ]
    transport = RedundantSocketCanReadOnlyTransport(
        "test0",
        filters=(ReceiveFilter(CTRL_FB_ID),),
        socket_module=FakeRedundantSocketApi,
        socket_factory=lambda *_args: sockets.pop(0),
        clock=time.monotonic,
        receive_timeout_sec=0.01,
    )
    transport.open()
    first = transport.receive()
    second = transport.receive()
    deadline = time.monotonic() + 0.2
    while transport.health.duplicate_frame_count < 2 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert [first.decoded.alive_counter, second.decoded.alive_counter] == [0, 1]
    assert first.kernel_timestamp_ns == 1_000_000_000
    assert transport.health.source_frame_counts == (2, 2)
    assert transport.health.duplicate_frame_count == 2
    assert transport.health.maximum_rxq_overflow == 0
    assert transport.health.healthy is True
    assert not hasattr(transport, "send")
    assert not hasattr(transport, "write")
    assert not hasattr(transport, "transmit")
    transport.close()


def test_redundant_receiver_projects_kernel_receive_time_to_monotonic_for_freshness():
    wire = raw_frame(CTRL_FB_ID, valid_ctrl(0))
    sockets = [
        FakeRecvmsgSocket([wire], [100_000_000_000]),
        FakeRecvmsgSocket([wire], [100_000_000_000]),
    ]
    transport = RedundantSocketCanReadOnlyTransport(
        "test0",
        filters=(ReceiveFilter(CTRL_FB_ID),),
        socket_module=FakeRedundantSocketApi,
        socket_factory=lambda *_args: sockets.pop(0),
        clock=lambda: 500.020,
        wall_clock=lambda: 100.020,
        receive_timeout_sec=0.01,
    )
    transport.open()
    record = transport.receive()
    # The userspace dequeue is 20 ms after the kernel timestamp. Freshness
    # must reflect the frame arrival, not the receiver thread's scheduling.
    assert record.received_monotonic_sec == pytest.approx(500.0)
    assert record.kernel_timestamp_ns == 100_000_000_000
    transport.close()


def test_redundant_receiver_does_not_refresh_an_old_kernel_frame_at_dequeue():
    wire = raw_frame(CTRL_FB_ID, valid_ctrl(0))
    sockets = [
        FakeRecvmsgSocket([wire], [99_940_000_000]),
        FakeRecvmsgSocket([wire], [99_940_000_000]),
    ]
    transport = RedundantSocketCanReadOnlyTransport(
        "test0",
        filters=(ReceiveFilter(CTRL_FB_ID),),
        socket_module=FakeRedundantSocketApi,
        socket_factory=lambda *_args: sockets.pop(0),
        clock=lambda: 500.020,
        wall_clock=lambda: 100.020,
        receive_timeout_sec=0.01,
    )
    transport.open()
    record = transport.receive()
    assert record.received_monotonic_sec == pytest.approx(499.940)
    assert 500.020 - record.received_monotonic_sec > 0.050
    transport.close()


def test_redundant_receiver_preserves_frame_missed_by_one_source():
    wire0 = raw_frame(CTRL_FB_ID, valid_ctrl(0))
    wire1 = raw_frame(CTRL_FB_ID, valid_ctrl(1))
    sockets = [
        FakeRecvmsgSocket([wire0], [2_000_000_000]),
        FakeRecvmsgSocket([wire0, wire1], [2_000_000_000, 2_010_000_000]),
    ]
    transport = RedundantSocketCanReadOnlyTransport(
        "test0",
        filters=(ReceiveFilter(CTRL_FB_ID),),
        socket_module=FakeRedundantSocketApi,
        socket_factory=lambda *_args: sockets.pop(0),
        receive_timeout_sec=0.01,
    )
    transport.open()
    records = [transport.receive(), transport.receive()]
    assert [record.decoded.alive_counter for record in records] == [0, 1]
    assert transport.health.healthy is True
    transport.close()


def test_receive_only_module_has_no_mock_motion_mode_dependency():
    import mkmini_cmd_adapter.receive_only as module

    assert not hasattr(module, "MkminiCanCodec")
    assert TransportMode.READ_ONLY.value == "READ_ONLY"
