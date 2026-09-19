"""Fail-safe controller for the sole native MK-mini SocketCAN writer."""

from __future__ import annotations

from collections import deque
import mmap
import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_prefix

from .backend_interlock import BackendCommand, BackendFeedback
from .codec import MkminiCanCodec, RunningMode
from .model import CtrlCommand, Gear


FEEDBACK_ABI = struct.Struct("<IIQddQQ14idd")
FEEDBACK_MAGIC = 0x4D4B5258
FEEDBACK_VERSION = 1


class NativeSender:
    def __init__(self, interface: str, *, cpu: int | None = None) -> None:
        self.interface = interface
        if cpu is not None and cpu < 0:
            raise ValueError("native sender CPU must be non-negative")
        self._cpu = cpu
        # A process can be killed before ``close()`` unlinks its AF_UNIX
        # endpoints.  PID/object-address names can then collide after a
        # restart, so add a per-instance nonce and defensively remove only
        # these newly selected paths before binding.
        suffix = f"{os.getpid()}_{uuid.uuid4().hex}"
        self._server_path = Path(f"/tmp/mkmini_native_{suffix}.sock")
        self._client_path = Path(f"/tmp/mkmini_backend_{suffix}.sock")
        self._feedback_path = Path(f"/tmp/mkmini_feedback_{suffix}.mmap")
        self._socket: socket.socket | None = None
        self._process: subprocess.Popen | None = None
        self._feedback_file = None
        self._feedback_mmap: mmap.mmap | None = None
        self._stop = threading.Event()
        self._first_status = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        # Retain a bounded recent window for operator-facing percentiles.
        # The 100 Hz safety supervisor uses watchdog_timing(), which is O(1).
        self._timestamps: deque[float] = deque(maxlen=1200)
        self._native_count = 0
        self._native_mean = None
        self._native_max = None
        self._native_command = None
        self._exit_reason: str | None = None

    @property
    def is_open(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def open(self) -> None:
        if self.is_open:
            return
        binary = Path(get_package_prefix("mkmini_native_sender")) / "lib/mkmini_native_sender/mkmini_native_sender"
        self._client_path.unlink(missing_ok=True)
        self._server_path.unlink(missing_ok=True)
        self._feedback_path.unlink(missing_ok=True)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        process: subprocess.Popen | None = None
        try:
            sock.bind(str(self._client_path))
            sock.settimeout(.05)
            native_core = self._cpu if self._cpu is not None else max(0, (os.cpu_count() or 2) - 2)
            process = subprocess.Popen(
                ["taskset", "-c", str(native_core), str(binary),
                 "--interface", self.interface, str(self._server_path), str(self._feedback_path)],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 1.0
            while (time.monotonic() < deadline
                   and (not self._server_path.exists() or not self._feedback_path.exists())
                   and process.poll() is None):
                time.sleep(.005)
            if (not self._server_path.exists() or not self._feedback_path.exists()
                    or process.poll() is not None):
                error = "" if process.stderr is None else process.stderr.read()
                raise RuntimeError(f"native sender failed to start: {error.strip()}")
            feedback_file = self._feedback_path.open("rb")
            feedback_mmap = mmap.mmap(feedback_file.fileno(), FEEDBACK_ABI.size, access=mmap.ACCESS_READ)
        except Exception:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=.5)
            sock.close()
            self._client_path.unlink(missing_ok=True)
            self._server_path.unlink(missing_ok=True)
            self._feedback_path.unlink(missing_ok=True)
            raise
        self._socket, self._process = sock, process
        self._feedback_file, self._feedback_mmap = feedback_file, feedback_mmap
        self._exit_reason = None
        self._stop.clear()
        self._first_status.clear()
        self._thread = threading.Thread(target=self._receive, name="mkmini-native-status", daemon=True)
        self._thread.start()
        self.update(BackendCommand.safe_neutral(), allowed=True)
        # Do not report the native sender as ready until we have observed an
        # actual native TX timestamp. This prevents a slow process start or a
        # delayed first status datagram from looking like a heartbeat failure
        # immediately after physical enable.
        if not self._first_status.wait(timeout=1.0):
            self.close()
            raise RuntimeError("native sender produced no initial heartbeat status")

    def _receive(self) -> None:
        while not self._stop.is_set():
            try:
                payload = self._socket.recv(256).decode("ascii") if self._socket is not None else ""
            except socket.timeout:
                continue
            except OSError:
                return
            fields = payload.split()
            if len(fields) != 10 or fields[0] != "STAT":
                continue
            try:
                stamp, interval, count, mean, maximum = float(fields[1]), float(fields[2]), int(fields[3]), float(fields[4]), float(fields[5])
                gear, speed_raw, steering_raw, allowed = map(int, fields[6:10])
            except ValueError:
                continue
            with self._lock:
                self._timestamps.append(stamp)
                self._native_count, self._native_mean, self._native_max = count, mean, maximum
                self._native_command = {
                    "timestamp_monotonic_sec": stamp,
                    "gear": gear,
                    "speed_raw_mmps": speed_raw,
                    "steering_raw_cdeg": steering_raw,
                    "steering_deg": steering_raw * .01,
                    "allowed": bool(allowed),
                }
            self._first_status.set()

    def update(self, command: BackendCommand, *, allowed: bool) -> None:
        if not self.is_open or self._socket is None:
            raise RuntimeError("native sender is not open")
        # Reuse the qualified Python codec as the quantization/validation
        # authority; the native writer receives only exact wire fields.
        frame = MkminiCanCodec.encode(CtrlCommand(command.gear, command.speed_mps, command.steering_deg, 0))
        word = int.from_bytes(frame.data[:7], "little")
        speed_raw = (word >> 4) & 0xFFFF
        steering_raw = (word >> 20) & 0xFFFF
        if steering_raw & 0x8000:
            steering_raw -= 0x10000
        payload = f"CMD {int(command.gear)} {speed_raw} {steering_raw} {int(allowed)}".encode("ascii")
        self._socket.sendto(payload, str(self._server_path))

    def latest_send(self) -> float | None:
        with self._lock:
            return None if not self._timestamps else self._timestamps[-1]

    def timing(self) -> dict:
        with self._lock:
            stamps = list(self._timestamps)
            count, mean, maximum = self._native_count, self._native_mean, self._native_max
        intervals = [b-a for a,b in zip(stamps, stamps[1:])]
        ordered = sorted(intervals)
        percentile = lambda q: None if not ordered else ordered[min(len(ordered)-1, int((len(ordered)-1)*q + .999999))]
        return {"count": count, "mean_interval_sec": mean, "p95_interval_sec": percentile(.95),
                "p99_interval_sec": percentile(.99), "max_interval_sec": maximum}

    def watchdog_timing(self) -> dict:
        """Return constant-time native watchdog data for the 100 Hz loop."""
        with self._lock:
            return {"count": self._native_count, "max_interval_sec": self._native_max}

    def transmitted_command(self) -> dict | None:
        """Return the last command the native writer actually put on CAN."""
        with self._lock:
            return None if self._native_command is None else dict(self._native_command)

    def feedback_snapshot(self, now: float, can_state: str) -> tuple[BackendFeedback, dict]:
        """Read the native RX/decoder seqlock without a ROS/Python RX queue."""
        raw = None
        with self._lock:
            mapped = self._feedback_mmap
            if mapped is not None:
                for _attempt in range(5):
                    first = mapped[:FEEDBACK_ABI.size]
                    values = FEEDBACK_ABI.unpack(first)
                    sequence = values[2]
                    if sequence & 1:
                        continue
                    second_sequence = struct.unpack_from("<Q", mapped, 8)[0]
                    if sequence == second_sequence:
                        raw = values
                        break
        if raw is None or raw[0] != FEEDBACK_MAGIC or raw[1] != FEEDBACK_VERSION:
            return BackendFeedback(can_state=can_state, hard_fault=True), {
                "source": "native_mmap", "valid": False, "reason": "SNAPSHOT_INVALID"}
        ctrl_stamp, diag_stamp = raw[3], raw[4]
        ctrl_count, diag_count = raw[5], raw[6]
        gear_raw, speed_raw, steering_raw, mode_raw = raw[7:11]
        fault_level, auto_can, native_hard_fault = raw[12], raw[13], raw[14]
        ctrl_checksum, diag_checksum = bool(raw[15]), bool(raw[16])
        ctrl_alive_valid, diag_alive_valid = bool(raw[17]), bool(raw[18])
        gear = Gear(gear_raw) if gear_raw in tuple(int(item) for item in Gear) else None
        mode = RunningMode(mode_raw) if mode_raw in (0, 1, 2) else mode_raw
        feedback = BackendFeedback(
            ctrl_age_sec=None if ctrl_stamp <= 0. else max(0.0, now - ctrl_stamp),
            diagnostic_age_sec=None if diag_stamp <= 0. else max(0.0, now - diag_stamp),
            ctrl_checksum_valid=ctrl_checksum,
            diagnostic_checksum_valid=diag_checksum,
            ctrl_alive_contiguous=ctrl_count >= 2 and ctrl_alive_valid,
            diagnostic_alive_contiguous=diag_count >= 2 and diag_alive_valid,
            gear=gear,
            speed_mps=None if speed_raw < 0 else speed_raw * .001,
            steering_deg=steering_raw * .01,
            mode=mode,
            vehicle_fault_level=None if fault_level < 0 else fault_level,
            auto_can_error=None if auto_can < 0 else bool(auto_can),
            can_state=can_state,
            hard_fault=bool(native_hard_fault),
        )
        detail = {
            "source": "native_mmap", "valid": True, "sequence": raw[2],
            "ctrl_count": ctrl_count, "diagnostic_count": diag_count,
            "ctrl_alive": raw[11], "ctrl_delta": raw[19], "diagnostic_delta": raw[20],
            "ctrl_interval_sec": raw[21], "diagnostic_interval_sec": raw[22],
        }
        return feedback, detail

    def exit_reason(self) -> str:
        """Return bounded native stderr only after the child has exited."""
        process = self._process
        if process is None or process.poll() is None:
            return ""
        with self._lock:
            if self._exit_reason is None:
                try:
                    text = "" if process.stderr is None else process.stderr.read()
                except OSError:
                    text = ""
                self._exit_reason = text.strip().replace("\n", " ")[:240]
            return self._exit_reason

    def close(self) -> None:
        if self._socket is not None and self.is_open:
            try:
                self._socket.sendto(b"STOP", str(self._server_path))
            except OSError:
                pass
        if self._process is not None:
            try:
                self._process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                self._process.wait(timeout=.5)
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=.2)
        if self._socket is not None:
            self._socket.close()
        if self._feedback_mmap is not None:
            self._feedback_mmap.close()
        if self._feedback_file is not None:
            self._feedback_file.close()
        self._socket = self._process = self._thread = None
        self._feedback_mmap = self._feedback_file = None
        self._client_path.unlink(missing_ok=True)
        self._server_path.unlink(missing_ok=True)
        self._feedback_path.unlink(missing_ok=True)


__all__ = ["NativeSender"]
