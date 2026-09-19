"""Future SocketCAN transmit transport, guarded by commissioning authority.

This module is intentionally not used by the current default path.  Tests
inject a fake socket; the default socket factory is reserved for a later,
explicitly authorized hardware task.
"""

from __future__ import annotations

import socket as _socket
import struct
import time
from typing import Any, Callable

from .commissioning import RealTransportAuthority
from .model import CanFrame


CAN_EFF_FLAG = 0x80000000


class SocketCanTxTransport:
    """Minimal TX-capable transport with no codec or command policy."""

    def __init__(
        self,
        interface_name: str = "can0",
        *,
        socket_module: Any = _socket,
        socket_factory: Callable[[], Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
        authority: RealTransportAuthority | None = None,
    ) -> None:
        if not interface_name:
            raise ValueError("interface_name must be non-empty")
        self.interface_name = interface_name
        self._socket_module = socket_module
        self._socket_factory = socket_factory
        self._clock = clock
        self.authority = authority or RealTransportAuthority.current_unqualified()
        self._sock: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def open(self) -> None:
        if self._sock is not None:
            return
        self.authority.require_tx()
        if self._socket_factory is None:
            sock = self._socket_module.socket(
                self._socket_module.AF_CAN,
                self._socket_module.SOCK_RAW,
                self._socket_module.CAN_RAW,
            )
        else:
            sock = self._socket_factory()
        try:
            sock.bind((self.interface_name,))
            # A full SocketCAN TX queue is a safety fault, not permission to
            # block the heartbeat scheduler past its watchdog deadline.
            setblocking = getattr(sock, "setblocking", None)
            if setblocking is not None:
                setblocking(False)
        except Exception:
            close = getattr(sock, "close", None)
            if close is not None:
                close()
            raise
        self._sock = sock

    def send_frame(self, frame: CanFrame, timestamp: float | None = None) -> float:
        """Send one already-encoded frame; policy belongs to the caller."""

        if self._sock is None:
            raise RuntimeError("TX transport is not open")
        if not isinstance(frame, CanFrame):
            raise TypeError("send_frame accepts only CanFrame")
        if frame.dlc > 8:
            raise ValueError("CAN frame DLC must be <= 8")
        can_id = frame.can_id | CAN_EFF_FLAG if frame.is_extended else frame.can_id
        payload = struct.pack("=IB3x8s", can_id, frame.dlc, frame.data.ljust(8, b"\x00"))
        self._sock.send(payload)
        return self._clock() if timestamp is None else timestamp

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None


__all__ = ["SocketCanTxTransport"]
