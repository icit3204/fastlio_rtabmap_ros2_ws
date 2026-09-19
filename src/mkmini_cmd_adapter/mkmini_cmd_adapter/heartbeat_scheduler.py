"""Dedicated, lease-supervised monotonic CAN heartbeat scheduler.

The scheduler owns timing only.  It has no command policy and cannot continue
emitting when its supervising application stops refreshing a short lease.
Timing records remain in memory during TX and are written only after TX stops.
"""

from __future__ import annotations

from dataclasses import dataclass
import statistics
import threading
import time
from typing import Callable


@dataclass(frozen=True)
class HeartbeatTiming:
    scheduled_time: float
    actual_send_time: float
    inter_send_interval_sec: float | None
    scheduler_lateness_sec: float
    send_duration_sec: float
    state: str


class MonotonicDeadline:
    """Drift-free period arithmetic, independently testable with virtual time."""

    def __init__(self, start_sec: float, period_sec: float) -> None:
        self.period_sec = period_sec
        self.next_time = start_sec

    def consume(self) -> float:
        scheduled = self.next_time
        self.next_time += self.period_sec
        return scheduled


def timing_summary(records: list[HeartbeatTiming]) -> dict:
    intervals = [item.inter_send_interval_sec for item in records if item.inter_send_interval_sec is not None]
    if not intervals:
        return {"count": len(records), "mean_interval_sec": None, "p95_interval_sec": None,
                "p99_interval_sec": None, "max_interval_sec": None}
    ordered = sorted(intervals)
    def percentile(percent: float) -> float:
        index = max(0, min(len(ordered) - 1, int((len(ordered) - 1) * percent + 0.999999)))
        return ordered[index]
    return {"count": len(records), "mean_interval_sec": statistics.fmean(intervals),
            "p95_interval_sec": percentile(.95), "p99_interval_sec": percentile(.99),
            "max_interval_sec": max(intervals)}


class HeartbeatScheduler:
    """100 Hz sender with a supervisor-renewed command lease.

    A publish call is the sole way to refresh authority.  If the supervisor is
    late, the scheduler stops sending instead of sustaining any command.
    """

    def __init__(self, send: Callable[[], None], *, period_sec: float = .010,
                 lease_sec: float | None = .025, clock: Callable[[], float] = time.monotonic) -> None:
        if (lease_sec is not None and not 0.0 < lease_sec < .030) or period_sec <= 0.0:
            raise ValueError("heartbeat lease must be positive and below watchdog, or disabled")
        self._send = send
        self._period = period_sec
        self._lease_sec = lease_sec
        self._clock = clock
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lease_deadline: float | None = None
        self._state = "DISARMED"
        self._fault: str | None = None
        self._records: list[HeartbeatTiming] = []

    def start(self, *, state: str, now: float | None = None) -> None:
        if self._thread is not None:
            raise RuntimeError("heartbeat scheduler already started")
        current = self._clock() if now is None else now
        with self._lock:
            self._state = state
            self._lease_deadline = None if self._lease_sec is None else current + self._lease_sec
        self._thread = threading.Thread(target=self._run, name="mkmini-can-heartbeat", daemon=True)
        self._thread.start()

    def publish(self, *, state: str, now: float | None = None) -> None:
        current = self._clock() if now is None else now
        with self._lock:
            self._state = state
            self._lease_deadline = None if self._lease_sec is None else current + self._lease_sec

    @property
    def fault(self) -> str | None:
        with self._lock:
            return self._fault

    def records(self) -> list[HeartbeatTiming]:
        with self._lock:
            return list(self._records)

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=.25)

    def _run(self) -> None:
        schedule = MonotonicDeadline(self._clock(), self._period)
        previous_send = None
        while not self._stop.is_set():
            scheduled = schedule.consume()
            delay = scheduled - self._clock()
            if delay > 0.0:
                self._wake.wait(delay)
                self._wake.clear()
                # ``stop`` wakes the wait; publishing deliberately does not.
                # Never turn an early wake into an early physical heartbeat.
                if self._clock() < scheduled and not self._stop.is_set():
                    continue
            if self._stop.is_set():
                return
            now = self._clock()
            with self._lock:
                lease_deadline = self._lease_deadline
                state = self._state
            if lease_deadline is not None and now > lease_deadline:
                with self._lock:
                    self._fault = "SUPERVISOR_LEASE_EXPIRED"
                return
            send_started = self._clock()
            try:
                self._send()
            except Exception as exc:
                with self._lock:
                    self._fault = f"TX_SEND_{type(exc).__name__}"
                return
            sent = self._clock()
            record = HeartbeatTiming(scheduled, sent,
                                    None if previous_send is None else sent - previous_send,
                                    max(0.0, sent - scheduled), sent - send_started, state)
            with self._lock:
                self._records.append(record)
            previous_send = sent


__all__ = ["HeartbeatScheduler", "HeartbeatTiming", "MonotonicDeadline", "timing_summary"]
