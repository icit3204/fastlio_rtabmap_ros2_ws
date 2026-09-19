"""Authorized, bounded MK-mini N+0+0 heartbeat qualification.

This executable has no D, reverse, nonzero-speed, or steering command surface.
It opens application TX only after the exact R3L-R2 operator acknowledgement,
continuously feeds installed-unit feedback through ``MkminiBackendInterlock``,
and closes TX immediately after qualification or any fail-closed transition.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import threading
import time

from .backend_interlock import BackendCommand, BackendFeedback, BackendInterlockState, MkminiBackendInterlock
from .backend_receive_only import _can_link, _jsonable
from .codec import (
    AliveCounter,
    CTRL_FB_ID,
    FeedbackAliveObserver,
    CtrlFeedback,
    LR_WHEEL_FB_ID,
    MkminiCanCodec,
    OdometerFeedback,
    RR_WHEEL_FB_ID,
    VEH_DIAG_FB_ID,
    VehicleDiagnosticFeedback,
    WheelFeedback,
)
from .model import CtrlCommand, Gear
from .receive_only import CapturedFrame, ReceiveFilter, RedundantSocketCanReadOnlyTransport, SocketCanReadOnlyTransport
from .heartbeat_scheduler import HeartbeatScheduler, timing_summary
from .tx import SocketCanTxTransport


ACKNOWLEDGEMENT = "r3l r2 zero heartbeat safe"
POST_QUALIFICATION_STABILITY_SEC = 10.0


class R3lR2ZeroHeartbeatAuthority:
    def __init__(self, acknowledgement: str) -> None:
        self.acknowledged = acknowledgement == ACKNOWLEDGEMENT

    def require_tx(self) -> None:
        if not self.acknowledged:
            raise RuntimeError("R3L_R2_ZERO_HEARTBEAT_AUTHORIZATION_REQUIRED")


class LiveFeedbackMonitor:
    def __init__(self, interface: str, *, safety_critical_only: bool = False,
                 receive_worker_cpus: tuple[int, int] | None = None,
                 receive_processing_cpu: int | None = None) -> None:
        # The physical ROS composition can briefly starve Python threads under
        # RTAB/FAST-LIO load.  Two large kernel queues preserve the stream and
        # SO_TIMESTAMPNS supplies the actual kernel receipt interval, so alive
        # continuity is not measured from delayed userspace scheduling.
        filters = None
        if safety_critical_only:
            # The physical backend's interlock consumes only these two frame
            # types.  Excluding wheel/odometer/ultrasonic traffic from its
            # redundant queues removes avoidable decoding contention; those
            # frames remain available to receive-only evidence tools.
            filters = (ReceiveFilter(CTRL_FB_ID), ReceiveFilter(VEH_DIAG_FB_ID))
        self._receiver = RedundantSocketCanReadOnlyTransport(
            interface, filters=filters, receive_timeout_sec=0.02,
            receive_buffer_bytes=4 << 20, worker_cpus=receive_worker_cpus)
        # ``RedundantSocketCanReadOnlyTransport`` timestamps frames in its
        # kernel-receive workers, but this consumer is responsible for moving
        # those records into the safety snapshot.  Under a full ROS stack it
        # must not contend with the general executor: otherwise a fresh frame
        # can sit in the Python queue long enough to fail the unchanged
        # freshness limit.  This is an execution-placement control only; it
        # neither changes timestamp semantics nor weakens the interlock.
        self._receive_processing_cpu = receive_processing_cpu
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ctrl = None
        self._diagnostic = None
        self._odometer = None
        self._wheels = {"left": None, "right": None}
        self._ctrl_stamp = None
        self._diagnostic_stamp = None
        self._ctrl_tracker = FeedbackAliveObserver()
        self._diagnostic_tracker = FeedbackAliveObserver()
        self._ctrl_contiguous = True
        self._diagnostic_contiguous = True
        # These are deliberately qualification-epoch counters, rather than
        # lifetime receive counts.  A physical backend can observe CAN while
        # disabled for an arbitrary time; that historical observation must
        # not become the alive baseline for a later TX enable.
        self._ctrl_alive_epoch_count = 0
        self._diagnostic_alive_epoch_count = 0
        self._counts = {"ctrl": 0, "diagnostic": 0, "odometer": 0, "left_wheel": 0,
                        "right_wheel": 0, "other": 0, "decode_error": 0}
        self._error: str | None = None
        self._alive_anomalies: list[dict] = []

    def open(self) -> None:
        self._receiver.open()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def reset_alive_epoch(self) -> None:
        """Begin a new, pre-TX alive qualification epoch.

        This method is only for an explicit disabled-to-enabled backend
        boundary.  It never runs while the interlock is armed, so runtime
        replay, checksum, and stale failures remain latched fail-closed.
        Raw receive counters and anomaly evidence are intentionally retained.
        """
        with self._lock:
            self._ctrl_tracker = FeedbackAliveObserver()
            self._diagnostic_tracker = FeedbackAliveObserver()
            self._ctrl_contiguous = True
            self._diagnostic_contiguous = True
            self._ctrl_alive_epoch_count = 0
            self._diagnostic_alive_epoch_count = 0

    @staticmethod
    def _sequence_timestamp(record: CapturedFrame) -> float:
        """Use kernel wire-receipt time for sequence intervals when present."""
        if record.kernel_timestamp_ns is not None:
            return record.kernel_timestamp_ns * 1.0e-9
        return record.received_monotonic_sec

    def _run(self) -> None:
        if self._receive_processing_cpu is not None:
            try:
                os.sched_setaffinity(0, {self._receive_processing_cpu})
            except (AttributeError, OSError):
                # The real freshness check remains fail-closed if a platform
                # cannot apply the requested Linux thread affinity.
                pass
        while not self._stop.is_set():
            try:
                record = self._receiver.receive()
            except TimeoutError:
                continue
            except Exception as exc:
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
                return
            with self._lock:
                if record.decode_error is not None:
                    self._counts["decode_error"] += 1
                elif isinstance(record.decoded, CtrlFeedback):
                    self._ctrl = record.decoded
                    self._ctrl_stamp = record.received_monotonic_sec
                    self._counts["ctrl"] += 1
                    observation = self._ctrl_tracker.observe(
                        record.decoded.alive_counter,
                        self._sequence_timestamp(record),
                        checksum_valid=record.decoded.checksum_valid,
                    )
                    self._ctrl_contiguous &= observation.valid
                    self._ctrl_alive_epoch_count += 1
                    if observation.warning or not observation.valid:
                        self._alive_anomalies.append({
                            "channel": "ctrl",
                            "timestamp_monotonic_sec": record.received_monotonic_sec,
                            "previous_alive": observation.previous,
                            "alive": observation.value,
                            "delta_mod16": observation.delta_mod16,
                            "checksum_valid": record.decoded.checksum_valid,
                            "valid": observation.valid,
                            "warning": observation.warning,
                            "reason": observation.reason,
                            "inter_frame_interval_sec": observation.interval_sec,
                        })
                        self._alive_anomalies = self._alive_anomalies[-100:]
                elif isinstance(record.decoded, VehicleDiagnosticFeedback):
                    self._diagnostic = record.decoded
                    self._diagnostic_stamp = record.received_monotonic_sec
                    self._counts["diagnostic"] += 1
                    observation = self._diagnostic_tracker.observe(
                        record.decoded.alive_counter,
                        self._sequence_timestamp(record),
                        checksum_valid=record.decoded.checksum_valid,
                    )
                    self._diagnostic_contiguous &= observation.valid
                    self._diagnostic_alive_epoch_count += 1
                    if observation.warning or not observation.valid:
                        self._alive_anomalies.append({
                            "channel": "diagnostic",
                            "timestamp_monotonic_sec": record.received_monotonic_sec,
                            "previous_alive": observation.previous,
                            "alive": observation.value,
                            "delta_mod16": observation.delta_mod16,
                            "checksum_valid": record.decoded.checksum_valid,
                            "valid": observation.valid,
                            "warning": observation.warning,
                            "reason": observation.reason,
                            "inter_frame_interval_sec": observation.interval_sec,
                        })
                    self._alive_anomalies = self._alive_anomalies[-100:]
                elif isinstance(record.decoded, OdometerFeedback):
                    self._odometer = record.decoded
                    self._counts["odometer"] += 1
                elif isinstance(record.decoded, WheelFeedback):
                    side = "left" if record.frame.can_id == LR_WHEEL_FB_ID else "right"
                    if record.frame.can_id in (LR_WHEEL_FB_ID, RR_WHEEL_FB_ID):
                        self._wheels[side] = record.decoded
                        self._counts[side + "_wheel"] += 1
                else:
                    self._counts["other"] += 1

    @staticmethod
    def _can_state(link: dict) -> str:
        info = link.get("linkinfo", {}).get("info_data", {})
        counters = info.get("berr_counter", {})
        xstats = link.get("linkinfo", {}).get("info_xstats", {})
        healthy = (
            info.get("state") == "ERROR-ACTIVE"
            and counters.get("tx") == 0
            and counters.get("rx") == 0
            and all(value == 0 for value in xstats.values())
        )
        return "ERROR-ACTIVE" if healthy else str(info.get("state", "UNKNOWN")) + "_WITH_ERRORS"

    def snapshot(self, now: float, link: dict, *, include_detail: bool = True) -> tuple[BackendFeedback, dict]:
        with self._lock:
            ctrl = self._ctrl
            diagnostic = self._diagnostic
            odometer = self._odometer
            wheels = dict(self._wheels)
            ctrl_stamp = self._ctrl_stamp
            diagnostic_stamp = self._diagnostic_stamp
            counts = dict(self._counts)
            error = self._error
            ctrl_contiguous = self._ctrl_contiguous
            diagnostic_contiguous = self._diagnostic_contiguous
            ctrl_alive_epoch_count = self._ctrl_alive_epoch_count
            diagnostic_alive_epoch_count = self._diagnostic_alive_epoch_count
            alive_anomalies = list(self._alive_anomalies)
            receiver_health = self._receiver.health
        # Receiver callbacks can advance between the caller taking ``now`` and
        # this locked snapshot.  Assess age only against a clock read made
        # after copying those receipt timestamps.
        assessment_now = max(now, time.monotonic())
        hard_fault = True
        if diagnostic is not None:
            hard_fault = any((
                diagnostic.emergency_stop_asserted,
                diagnostic.eps_fault_code != 0,
                diagnostic.left_drive_fault != 0,
                diagnostic.right_drive_fault != 0,
                diagnostic.bms_can_communication_loss,
            ))
        gear = None
        if ctrl is not None and ctrl.gear in tuple(int(item) for item in Gear):
            gear = Gear(ctrl.gear)
        feedback = BackendFeedback(
            ctrl_age_sec=None if ctrl_stamp is None else assessment_now - ctrl_stamp,
            diagnostic_age_sec=None if diagnostic_stamp is None else assessment_now - diagnostic_stamp,
            ctrl_checksum_valid=ctrl is not None and ctrl.checksum_valid,
            diagnostic_checksum_valid=diagnostic is not None and diagnostic.checksum_valid,
            # Two fresh frames establish a baseline plus one observed forward
            # transition in this explicit qualification epoch.
            ctrl_alive_contiguous=ctrl_alive_epoch_count >= 2 and ctrl_contiguous,
            diagnostic_alive_contiguous=diagnostic_alive_epoch_count >= 2 and diagnostic_contiguous,
            gear=gear,
            speed_mps=None if ctrl is None else ctrl.speed_magnitude_mps,
            steering_deg=None if ctrl is None else ctrl.inner_wheel_steering_deg,
            mode=None if ctrl is None else ctrl.running_mode,
            vehicle_fault_level=None if diagnostic is None else diagnostic.vehicle_fault_level,
            auto_can_error=None if diagnostic is None else diagnostic.auto_can_communication_error,
            can_state=self._can_state(link),
            hard_fault=hard_fault or error is not None or not receiver_health.healthy,
        )
        # Physical 100 Hz supervision needs only the immutable safety sample.
        # Evidence conversion is intentionally optional and remains off the
        # heartbeat-critical path.
        if not include_detail:
            return feedback, {}
        detail = {
            "feedback": _jsonable(feedback),
            "ctrl": _jsonable(ctrl),
            "diagnostic": _jsonable(diagnostic),
            "odometer": _jsonable(odometer),
            "wheels": _jsonable(wheels),
            "counts": counts,
            "receiver_error": error,
            "receiver_health": _jsonable(receiver_health),
            "alive_anomalies": alive_anomalies,
        }
        return feedback, detail

    def close(self) -> None:
        self._stop.set()
        self._receiver.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class LiveCanMonitor:
    """Read-only link-state polling isolated from the command scheduler."""

    def __init__(self, interface: str) -> None:
        self._interface = interface
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._link = _can_link(interface)
        self._error: str | None = None
        self._thread: threading.Thread | None = None

    def open(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(0.05):
            try:
                link = _can_link(self._interface)
            except Exception as exc:
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
                continue
            with self._lock:
                self._link = link
                self._error = None

    def snapshot(self) -> dict:
        with self._lock:
            if self._error is not None:
                link = dict(self._link)
                info = dict(link.get("linkinfo", {}).get("info_data", {}))
                info["state"] = "CAN_MONITOR_ERROR"
                link["linkinfo"] = {"info_data": info, "info_xstats": {"monitor_error": 1}}
                return link
            return self._link

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def _send_neutral(transport, alive: AliveCounter, now: float | None = None) -> None:
    command = CtrlCommand(Gear.N, 0.0, 0.0, alive.next())
    transport.send_frame(MkminiCanCodec.encode(command), timestamp=now)


def run_zero_heartbeat(
    *,
    interface: str,
    maximum_duration_sec: float,
    acknowledgement: str,
    report_path: Path,
) -> int:
    authority = R3lR2ZeroHeartbeatAuthority(acknowledgement)
    authority.require_tx()  # must occur before either physical transport opens
    monitor = LiveFeedbackMonitor(interface)
    can_monitor = LiveCanMonitor(interface)
    tx = SocketCanTxTransport(interface, authority=authority)
    core = MkminiBackendInterlock()
    alive = AliveCounter()
    records = []
    emitted = 0
    first_emit = None
    last_emit = None
    started = time.monotonic()
    qualified_at = None
    stable_observed_sec = 0.0
    status = "BLOCKED_INTERLOCK_NOT_SATISFIED"
    final_detail = None
    scheduler = None
    final_zero_emitted = False
    monitor.open()
    can_monitor.open()
    try:
        # Establish fresh receive evidence before application TX starts.
        baseline_deadline = time.monotonic() + 1.0
        while time.monotonic() < baseline_deadline:
            now = time.monotonic()
            feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
            if not core.assess_feedback(feedback, require_n_auto=False):
                break
            time.sleep(0.01)
        else:
            status = "BLOCKED_BASE_FEEDBACK"
            return_code = 2
            return _finish_report(report_path, locals(), return_code)

        tx.open()
        core.arm(time.monotonic())
        scheduler = HeartbeatScheduler(lambda: _send_neutral(tx, alive), period_sec=.010, lease_sec=.025)
        scheduler.start(state=core.state.value)
        next_supervision = time.monotonic()
        deadline = next_supervision + maximum_duration_sec
        last_state = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
            result = core.step(now, BackendCommand.safe_neutral(), feedback)
            scheduler.publish(state=result.state.value, now=now)
            first_permission = result.motion_permitted and qualified_at is None
            if result.state != last_state or first_permission:
                records.append({
                    "elapsed_sec": now - started,
                    "state": result.state.value,
                    "motion_permitted": result.motion_permitted,
                    "reasons": list(result.reasons),
                    "feedback": _jsonable(feedback),
                })
                last_state = result.state
            if first_permission:
                qualified_at = now
                status = "MOTION_PERMISSION_STABILITY_OBSERVATION"
            if qualified_at is not None and result.motion_permitted:
                stable_observed_sec = now - qualified_at
                if stable_observed_sec >= POST_QUALIFICATION_STABILITY_SEC:
                    status = "MOTION_PERMITTED_STABLE"
                    break
            if result.state is BackendInterlockState.MOTION_NOT_PERMITTED:
                status = "BLOCKED_INTERLOCK_NOT_SATISFIED"
                break
            if scheduler.fault is not None:
                status = "BLOCKED_HEARTBEAT_SCHEDULING"
                break
            next_supervision += .005
            delay = next_supervision - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_supervision = time.monotonic()
        return_code = 0 if status == "MOTION_PERMITTED_STABLE" else 2
    finally:
        # A final N+0+0 is best-effort while the authorized socket is open,
        # followed immediately by closing application TX.
        if tx.is_open:
            try:
                if scheduler is not None:
                    scheduler.stop()
                _send_neutral(tx, alive, time.monotonic())
                final_zero_emitted = True
            finally:
                tx.close()
        monitor.close()
        can_monitor.close()
    return _finish_report(report_path, locals(), return_code)


def _finish_report(path: Path, scope: dict, return_code: int) -> int:
    started = scope["started"]
    qualified_at = scope.get("qualified_at")
    final_link = _can_link(scope["interface"])
    timing = [] if scope.get("scheduler") is None else scope["scheduler"].records()
    timing_json = [{"scheduled_time": item.scheduled_time, "actual_send_time": item.actual_send_time,
                    "inter_send_interval_sec": item.inter_send_interval_sec,
                    "scheduler_lateness_sec": item.scheduler_lateness_sec,
                    "send_duration_sec": item.send_duration_sec, "state": item.state} for item in timing]
    report = {
        "status": scope.get("status"),
        "interface": scope["interface"],
        "heartbeat_target_hz": 100.0,
        "heartbeat_frames": len(timing_json),
        "heartbeat_timing": timing_json,
        "heartbeat_timing_summary": timing_summary(timing),
        "heartbeat_scheduler_fault": None if scope.get("scheduler") is None else scope["scheduler"].fault,
        "elapsed_sec": time.monotonic() - started,
        "time_to_motion_permission_sec": None if qualified_at is None else qualified_at - started,
        "motion_permission_stable_sec": scope.get("stable_observed_sec", 0.0),
        "state_transitions": scope.get("records", []),
        "final_feedback": scope.get("final_detail"),
        "final_can": final_link,
        "final_zero_emitted": scope.get("final_zero_emitted", False),
        "application_can_tx_stopped": not scope["tx"].is_open,
        "command_contract": {"gear": "N", "speed_mps": 0.0, "steering_deg": 0.0},
    }
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return return_code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--maximum-duration-sec", type=float, default=5.0)
    parser.add_argument("--operator-ack", default="")
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if not 1.0 <= arguments.maximum_duration_sec <= 15.0:
        parser.error("--maximum-duration-sec must be in [1, 15]")
    return run_zero_heartbeat(
        interface=arguments.interface,
        maximum_duration_sec=arguments.maximum_duration_sec,
        acknowledgement=arguments.operator_ack,
        report_path=arguments.report,
    )


if __name__ == "__main__":
    raise SystemExit(main())
