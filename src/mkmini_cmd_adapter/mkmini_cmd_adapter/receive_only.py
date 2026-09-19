"""Receive-only MK-mini frame capture for a later authorized hardware trial.

The physical socket is dependency-injected and is never constructed by this
package's tests or current commissioning paths.  This module intentionally
does not import command codecs, keyboard code, or output transport interfaces.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
import queue
import socket as _socket
import struct
import threading
import time
from typing import Any, Callable, TextIO

from .codec import (
    BMS_FLAG_FB_ID,
    BMS_INFO_FB_ID,
    CTRL_FB_ID,
    LR_WHEEL_FB_ID,
    ODO_FB_ID,
    RR_WHEEL_FB_ID,
    ULTRASONIC_1_FB_ID,
    ULTRASONIC_2_FB_ID,
    VEH_DIAG_FB_ID,
    MkminiFeedbackCodec,
)
from .model import CanFrame


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
FEEDBACK_IDS = (
    CTRL_FB_ID,
    LR_WHEEL_FB_ID,
    RR_WHEEL_FB_ID,
    VEH_DIAG_FB_ID,
    BMS_INFO_FB_ID,
    BMS_FLAG_FB_ID,
    ODO_FB_ID,
    ULTRASONIC_1_FB_ID,
    ULTRASONIC_2_FB_ID,
)


@dataclass(frozen=True)
class ReceiveFilter:
    """An extended-ID filter in the host-independent logical form."""

    can_id: int
    mask: int = CAN_EFF_MASK | CAN_EFF_FLAG

    def __post_init__(self) -> None:
        if not 0 <= self.can_id <= CAN_EFF_MASK:
            raise ValueError("filter ID is outside the extended-ID range")

    @property
    def encoded_id(self) -> int:
        return self.can_id | CAN_EFF_FLAG


def feedback_filters() -> tuple[ReceiveFilter, ...]:
    """Return only the known feedback filters; no command ID is included."""

    return tuple(ReceiveFilter(can_id) for can_id in FEEDBACK_IDS)


def linux_filter_bytes(filters: tuple[ReceiveFilter, ...] | None = None) -> bytes:
    """Encode filters as the native two-uint32 receive-filter layout."""

    selected = feedback_filters() if filters is None else filters
    return b"".join(struct.pack("=II", item.encoded_id, item.mask) for item in selected)


@dataclass(frozen=True)
class CapturedFrame:
    frame: CanFrame
    received_monotonic_sec: float
    decoded: Any | None = None
    decode_error: str | None = None
    kernel_timestamp_ns: int | None = None
    receive_source: str | None = None
    rxq_overflow_total: int | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.received_monotonic_sec):
            raise ValueError("receive timestamp must be finite")


@dataclass(frozen=True)
class RedundantReceiveHealth:
    """Observability for the fail-closed redundant receive path."""

    source_frame_counts: tuple[int, int]
    duplicate_frame_count: int
    source_errors: tuple[str | None, str | None]
    maximum_rxq_overflow: int

    @property
    def healthy(self) -> bool:
        return not any(self.source_errors) and self.maximum_rxq_overflow == 0


def _decode_known(frame: CanFrame) -> Any | None:
    decoders = {
        CTRL_FB_ID: MkminiFeedbackCodec.decode_ctrl_fb,
        LR_WHEEL_FB_ID: MkminiFeedbackCodec.decode_lr_wheel_fb,
        RR_WHEEL_FB_ID: MkminiFeedbackCodec.decode_rr_wheel_fb,
        VEH_DIAG_FB_ID: MkminiFeedbackCodec.decode_vehicle_diagnostic,
        BMS_INFO_FB_ID: MkminiFeedbackCodec.decode_bms_info,
        BMS_FLAG_FB_ID: MkminiFeedbackCodec.decode_bms_flags,
        ODO_FB_ID: MkminiFeedbackCodec.decode_odometer,
        ULTRASONIC_1_FB_ID: MkminiFeedbackCodec.decode_ultrasonic_1,
        ULTRASONIC_2_FB_ID: MkminiFeedbackCodec.decode_ultrasonic_2,
    }
    decoder = decoders.get(frame.can_id)
    return None if decoder is None else decoder(frame)


def _captured_from_wire(raw: bytes, timestamp: float) -> CapturedFrame:
    if len(raw) < 16:
        raise ValueError("received frame is shorter than the native frame layout")
    raw_id, dlc, payload = struct.unpack("=IB3x8s", raw[:16])
    if dlc > 8:
        raise ValueError("received frame has invalid DLC")
    is_extended = bool(raw_id & CAN_EFF_FLAG)
    frame = CanFrame(can_id=raw_id & CAN_EFF_MASK, data=payload[:dlc], is_extended=is_extended)
    try:
        decoded = _decode_known(frame)
        return CapturedFrame(frame, timestamp, decoded=decoded)
    except Exception as exc:  # preserve the raw frame and decoding failure
        return CapturedFrame(frame, timestamp, decode_error=f"{type(exc).__name__}: {exc}")


class SocketCanReadOnlyTransport:
    """Explicitly receive-only application transport.

    ``open`` is the only method that can construct the injected socket.  The
    default socket factory is intentionally not exercised by current tests;
    future hardware work must invoke this class explicitly after its own
    preflight.  This class has no send/write/transmit operation.
    """

    def __init__(
        self,
        interface_name: str,
        *,
        filters: tuple[ReceiveFilter, ...] | None = None,
        receive_timeout_sec: float | None = None,
        socket_module: Any = _socket,
        socket_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(interface_name, str) or not interface_name.strip():
            raise ValueError("interface_name must be non-empty")
        self.interface_name = interface_name
        # ``None`` selects the known-feedback qualification set.  An explicit
        # empty tuple disables kernel filtering and is the full-archive mode,
        # preserving unknown IDs for offline discovery.
        self._filters = feedback_filters() if filters is None else tuple(filters)
        if receive_timeout_sec is not None and (not math.isfinite(receive_timeout_sec) or receive_timeout_sec <= 0.0):
            raise ValueError("receive_timeout_sec must be finite and positive when configured")
        self._socket_module = socket_module
        self._socket_factory = socket_factory or socket_module.socket
        self._clock = clock
        self._receive_timeout_sec = receive_timeout_sec
        self._socket: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    def open(self) -> None:
        if self._socket is not None:
            raise RuntimeError("receive-only transport is already open")
        sock = self._socket_factory(
            self._socket_module.AF_CAN,
            self._socket_module.SOCK_RAW,
            self._socket_module.CAN_RAW,
        )
        try:
            if self._receive_timeout_sec is not None and hasattr(sock, "settimeout"):
                sock.settimeout(self._receive_timeout_sec)
            sock.setsockopt(
                self._socket_module.SOL_CAN_RAW,
                self._socket_module.CAN_RAW_FILTER,
                linux_filter_bytes(self._filters),
            )
            # Install filters before bind so a frame cannot enter the socket
            # queue during the unfiltered bind-to-filter interval.
            sock.bind((self.interface_name,))
        except Exception:
            sock.close()
            raise
        self._socket = sock

    def receive(self) -> CapturedFrame:
        if self._socket is None:
            raise RuntimeError("receive-only transport is not open")
        return _captured_from_wire(self._socket.recv(16), self._clock())

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None


class RedundantSocketCanReadOnlyTransport:
    """Merged receive-only transport using two independently queued sockets.

    Linux supplies the same kernel packet timestamp to both sockets.  That
    timestamp, CAN ID, and payload form the deduplication identity.  A frame
    missed by one userspace socket therefore remains available from the other.
    This class intentionally has no send/write/transmit method.
    """

    _SO_TIMESTAMPNS = 35
    _SO_RXQ_OVFL = 40

    def __init__(
        self,
        interface_name: str,
        *,
        filters: tuple[ReceiveFilter, ...] | None = None,
        receive_timeout_sec: float = 0.10,
        socket_module: Any = _socket,
        socket_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
        receive_buffer_bytes: int = 4 << 20,
        worker_cpus: tuple[int, int] | None = None,
    ) -> None:
        if not interface_name.strip():
            raise ValueError("interface_name must be non-empty")
        if receive_timeout_sec <= 0.0 or not math.isfinite(receive_timeout_sec):
            raise ValueError("receive_timeout_sec must be finite and positive")
        if receive_buffer_bytes <= 0:
            raise ValueError("receive_buffer_bytes must be positive")
        self.interface_name = interface_name
        self._filters = feedback_filters() if filters is None else tuple(filters)
        self._timeout = receive_timeout_sec
        self._socket_module = socket_module
        self._socket_factory = socket_factory or socket_module.socket
        self._clock = clock
        self._wall_clock = wall_clock
        self._receive_buffer_bytes = receive_buffer_bytes
        self._worker_cpus = worker_cpus
        self._sockets: list[Any] = []
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()
        self._output: queue.Queue[CapturedFrame] = queue.Queue()
        self._lock = threading.Lock()
        self._seen: dict[tuple[int, int, bytes], float] = {}
        self._counts = [0, 0]
        self._duplicates = 0
        self._errors: list[str | None] = [None, None]
        self._maximum_overflow = 0

    @property
    def is_open(self) -> bool:
        return bool(self._sockets)

    @property
    def health(self) -> RedundantReceiveHealth:
        with self._lock:
            return RedundantReceiveHealth(
                tuple(self._counts), self._duplicates, tuple(self._errors), self._maximum_overflow
            )

    def _new_socket(self) -> Any:
        sock = self._socket_factory(
            self._socket_module.AF_CAN,
            self._socket_module.SOCK_RAW,
            self._socket_module.CAN_RAW,
        )
        sock.setsockopt(self._socket_module.SOL_SOCKET, self._socket_module.SO_RCVBUF, self._receive_buffer_bytes)
        sock.setsockopt(self._socket_module.SOL_SOCKET, self._SO_TIMESTAMPNS, 1)
        sock.setsockopt(self._socket_module.SOL_SOCKET, self._SO_RXQ_OVFL, 1)
        sock.setsockopt(
            self._socket_module.SOL_CAN_RAW,
            self._socket_module.CAN_RAW_FILTER,
            linux_filter_bytes(self._filters),
        )
        sock.settimeout(self._timeout)
        sock.bind((self.interface_name,))
        return sock

    def open(self) -> None:
        if self.is_open:
            raise RuntimeError("redundant receive-only transport is already open")
        self._stop.clear()
        try:
            self._sockets = []
            self._sockets.append(self._new_socket())
            self._sockets.append(self._new_socket())
        except Exception:
            for sock in self._sockets:
                sock.close()
            self._sockets = []
            raise
        self._threads = [
            threading.Thread(target=self._receive_loop, args=(index, sock), daemon=True)
            for index, sock in enumerate(self._sockets)
        ]
        for thread in self._threads:
            thread.start()

    @staticmethod
    def _ancillary(ancdata: list[tuple[int, int, bytes]]) -> tuple[int | None, int | None]:
        kernel_ns = None
        overflow = None
        for level, kind, data in ancdata:
            if level == _socket.SOL_SOCKET and kind == RedundantSocketCanReadOnlyTransport._SO_TIMESTAMPNS and len(data) >= 16:
                seconds, nanoseconds = struct.unpack("=qq", data[:16])
                kernel_ns = seconds * 1_000_000_000 + nanoseconds
            elif level == _socket.SOL_SOCKET and kind == RedundantSocketCanReadOnlyTransport._SO_RXQ_OVFL and len(data) >= 4:
                overflow = struct.unpack("=I", data[:4])[0]
        return kernel_ns, overflow

    def _receive_loop(self, index: int, sock: Any) -> None:
        if self._worker_cpus is not None:
            try:
                os.sched_setaffinity(0, {self._worker_cpus[index]})
            except (AttributeError, OSError):
                # Failure remains observable through actual feedback freshness
                # and the unchanged interlock.  Other platforms may not expose
                # Linux thread affinity controls.
                pass
        while not self._stop.is_set():
            try:
                raw, ancdata, _flags, _address = sock.recvmsg(16, 128)
            except (TimeoutError, _socket.timeout):
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._errors[index] = f"{type(exc).__name__}: {exc}"
                return
            monotonic_before = self._clock()
            try:
                kernel_ns, overflow = self._ancillary(ancdata)
                if kernel_ns is None:
                    received = monotonic_before
                else:
                    # SO_TIMESTAMPNS is a kernel CLOCK_REALTIME timestamp.
                    # Project it into the monotonic domain using a bracketed
                    # local offset, so userspace dequeue scheduling cannot
                    # make a fresh queued CAN frame appear stale. A genuinely
                    # old kernel timestamp remains old and fails the existing
                    # feedback freshness bound.
                    realtime_now = self._wall_clock()
                    monotonic_after = self._clock()
                    offset = ((monotonic_before + monotonic_after) * 0.5) - realtime_now
                    received = kernel_ns * 1.0e-9 + offset
                record = _captured_from_wire(raw, received)
            except Exception as exc:
                with self._lock:
                    self._errors[index] = f"{type(exc).__name__}: {exc}"
                return
            record = CapturedFrame(
                record.frame,
                record.received_monotonic_sec,
                record.decoded,
                record.decode_error,
                kernel_timestamp_ns=kernel_ns,
                receive_source=f"rx{index}",
                rxq_overflow_total=overflow,
            )
            # Kernel timestamps are required on the physical path.  A fallback
            # identity exists only for dependency-injected test sockets.
            identity_ns = kernel_ns if kernel_ns is not None else int(received * 1_000_000)
            identity = (identity_ns, record.frame.can_id, record.frame.data)
            with self._lock:
                self._counts[index] += 1
                if overflow is not None:
                    self._maximum_overflow = max(self._maximum_overflow, overflow)
                if identity in self._seen:
                    self._duplicates += 1
                    continue
                self._seen[identity] = received
                # Keep the identity cache bounded without losing the duplicate
                # arriving from the other socket a few scheduling ticks later.
                cutoff = received - 2.0
                if len(self._seen) > 4096:
                    self._seen = {key: stamp for key, stamp in self._seen.items() if stamp >= cutoff}
            self._output.put(record)

    def receive(self) -> CapturedFrame:
        if not self.is_open:
            raise RuntimeError("redundant receive-only transport is not open")
        try:
            return self._output.get(timeout=self._timeout)
        except queue.Empty as exc:
            raise _socket.timeout() from exc

    def close(self) -> None:
        self._stop.set()
        sockets, self._sockets = self._sockets, []
        for sock in sockets:
            sock.close()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads = []


@dataclass
class CaptureSummary:
    """Mutable offline summary; raw records remain separately archivable."""

    count: int = 0
    first_timestamp_sec: float | None = None
    last_timestamp_sec: float | None = None
    payloads: set[bytes] = field(default_factory=set)
    alive_values: list[int] = field(default_factory=list)
    checksum_valid_count: int = 0
    checksum_invalid_count: int = 0

    @property
    def rate_hz(self) -> float | None:
        if self.first_timestamp_sec is None or self.last_timestamp_sec is None:
            return None
        span = self.last_timestamp_sec - self.first_timestamp_sec
        return None if span <= 0.0 or self.count < 2 else (self.count - 1) / span

    @property
    def payload_variants(self) -> int:
        return len(self.payloads)


def _alive_and_checksum(decoded: Any) -> tuple[int | None, bool | None]:
    alive = getattr(decoded, "alive_counter", None)
    checksum_valid = getattr(decoded, "checksum_valid", None)
    return alive, checksum_valid


def _json_safe(value: Any) -> Any:
    """Convert decoder values, including raw bytes and enums, to JSON data."""

    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "value"):
        return _json_safe(value.value)
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "__dataclass_fields__"):
        return {key: _json_safe(item) for key, item in vars(value).items()}
    return str(value)


class CaptureAccumulator:
    """Summarize frames without discarding unknown IDs or bad checksums."""

    def __init__(self) -> None:
        self.by_id: dict[int, CaptureSummary] = {}
        self.records: list[CapturedFrame] = []

    def observe(self, record: CapturedFrame) -> None:
        self.records.append(record)
        key = record.frame.can_id
        summary = self.by_id.setdefault(key, CaptureSummary())
        summary.count += 1
        summary.first_timestamp_sec = record.received_monotonic_sec if summary.first_timestamp_sec is None else summary.first_timestamp_sec
        summary.last_timestamp_sec = record.received_monotonic_sec
        summary.payloads.add(record.frame.data)
        if record.decoded is not None:
            alive, checksum_valid = _alive_and_checksum(record.decoded)
            if alive is not None:
                summary.alive_values.append(alive)
            if checksum_valid is True:
                summary.checksum_valid_count += 1
            elif checksum_valid is False:
                summary.checksum_invalid_count += 1


def capture_frames(
    transport: SocketCanReadOnlyTransport,
    frame_count: int,
    *,
    accumulator: CaptureAccumulator | None = None,
    archive: "JsonlCaptureArchive | None" = None,
) -> CaptureAccumulator:
    """Capture a bounded number of frames using an already-open receiver."""

    if isinstance(frame_count, bool) or not isinstance(frame_count, int) or frame_count < 0:
        raise ValueError("frame_count must be a non-negative integer")
    result = accumulator or CaptureAccumulator()
    for _ in range(frame_count):
        record = transport.receive()
        result.observe(record)
        if archive is not None:
            archive.append(record)
    return result


class JsonlCaptureArchive:
    """Raw archival writer; callers choose the destination outside the repo."""

    def __init__(self, stream: TextIO) -> None:
        self.stream = stream

    def append(self, record: CapturedFrame) -> None:
        decoded = None if record.decoded is None else _json_safe(record.decoded)
        payload = {
            "received_monotonic_sec": record.received_monotonic_sec,
            "kernel_timestamp_ns": record.kernel_timestamp_ns,
            "receive_source": record.receive_source,
            "rxq_overflow_total": record.rxq_overflow_total,
            "can_id": record.frame.can_id,
            "is_extended": record.frame.is_extended,
            "dlc": record.frame.dlc,
            "payload_hex": record.frame.data.hex(),
            "decoded": decoded,
            "decode_error": record.decode_error,
        }
        self.stream.write(json.dumps(payload, sort_keys=True) + "\n")
