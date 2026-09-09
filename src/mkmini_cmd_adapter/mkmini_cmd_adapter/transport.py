"""Transport-neutral memory surfaces for MK-mini qualification.

The package deliberately contains no physical transport implementation.  The
interfaces below make the future read-only and transmit-capable paths
structurally distinct while keeping the current default memory-only.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from .model import CanFrame


@dataclass(frozen=True)
class MockTransportRecord:
    frame: CanFrame
    timestamp: float | None


class MockTransport:
    """Bounded in-memory sink; it never opens or configures a CAN interface."""

    def __init__(self, max_records: int = 128) -> None:
        if not isinstance(max_records, int) or isinstance(max_records, bool) or max_records <= 0:
            raise ValueError("max_records must be a positive integer")
        self._records: deque[MockTransportRecord] = deque(maxlen=max_records)

    def send(self, frame: CanFrame, timestamp: float | None = None) -> None:
        if not isinstance(frame, CanFrame):
            raise TypeError("MockTransport accepts only CanFrame")
        self._records.append(MockTransportRecord(frame=frame, timestamp=timestamp))

    @property
    def count(self) -> int:
        return len(self._records)

    @property
    def records(self) -> tuple[MockTransportRecord, ...]:
        return tuple(self._records)

    def latest(self) -> MockTransportRecord | None:
        return self._records[-1] if self._records else None

    def clear(self) -> None:
        self._records.clear()


class FrameSink(Protocol):
    """Minimal output contract shared by memory and future injected sinks."""

    def send(self, frame: CanFrame, timestamp: float | None = None) -> None:
        ...


class TransportMode(str, Enum):
    """Capability declaration, not a request to access hardware."""

    MOCK = "MOCK"
    READ_ONLY = "READ_ONLY"
    TX_CAPABLE = "TX_CAPABLE"


class ReadOnlyTransport:
    """Memory-only receive surface for a later explicitly authorized trial.

    ``receive`` accepts already captured frame fixtures.  It has no output
    method and therefore cannot transmit a command.
    """

    def __init__(self, max_records: int = 128) -> None:
        if not isinstance(max_records, int) or isinstance(max_records, bool) or max_records <= 0:
            raise ValueError("max_records must be a positive integer")
        self._records: deque[CanFrame] = deque(maxlen=max_records)

    def receive(self, frame: CanFrame) -> None:
        if not isinstance(frame, CanFrame):
            raise TypeError("ReadOnlyTransport accepts only CanFrame")
        self._records.append(frame)

    @property
    def records(self) -> tuple[CanFrame, ...]:
        return tuple(self._records)

    def clear(self) -> None:
        self._records.clear()
