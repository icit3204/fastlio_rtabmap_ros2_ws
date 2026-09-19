"""Single-use, bounded first forward ground-motion proof.

The fixed profile requalifies N+0+0, commands only D/0.04/0, stops at a
bounded odometer target or command-time ceiling, proves standstill, returns to
N+0+0, and closes application CAN TX.  There is no reverse or arbitrary
command surface.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

from .backend_interlock import BackendCommand, BackendInterlockConfig, BackendInterlockState, MkminiBackendInterlock
from .backend_receive_only import _can_link, _jsonable
from .backend_zero_heartbeat import LiveCanMonitor, LiveFeedbackMonitor
from .codec import AliveCounter, MkminiCanCodec
from .fastlio_speed import FastLioSpeedMonitor
from .heartbeat_scheduler import HeartbeatScheduler, timing_summary
from .model import CtrlCommand, Gear
from .tx import SocketCanTxTransport


ACKNOWLEDGEMENT = "r3l r9 ground safe"
COMMISSIONING_SPEED_CEILING_MPS = 0.04
COMMAND_SPEED_MPS = 0.04
# R9 only: the repeated installed-unit 0.052--0.054 m/s plateau is bounded by
# an independent FAST-LIO speed observer.  This is never a production limit.
COMMISSIONING_FEEDBACK_CEILING_MPS = 0.060
FASTLIO_SPEED_CEILING_MPS = 0.060
R9_INTERLOCK_CONFIG = BackendInterlockConfig(
    maximum_motion_speed_mps=COMMISSIONING_FEEDBACK_CEILING_MPS,
)
TARGET_DISTANCE_M = 0.22
MAX_COMMAND_DURATION_SEC = 7.0
OUTPUT_PERIOD_SEC = 0.01


class R3lR9GroundAuthority:
    def __init__(self, acknowledgement: str) -> None:
        self.acknowledged = acknowledgement == ACKNOWLEDGEMENT

    def require_tx(self) -> None:
        if not self.acknowledged:
            raise RuntimeError("R3L_R9_GROUND_MOTION_AUTHORIZATION_REQUIRED")


class GroundMotionBlocked(RuntimeError):
    pass


def commissioning_drive_command(speed_mps: float) -> BackendCommand:
    """Construct the sole R3L commissioning drive command or reject it."""

    if speed_mps < 0.0 or speed_mps > COMMISSIONING_SPEED_CEILING_MPS:
        raise ValueError("commissioning command exceeds 0.040 m/s ceiling")
    return BackendCommand(Gear.D, speed_mps, 0.0)


def _send(transport, alive: AliveCounter, command: BackendCommand, now: float) -> None:
    frame = MkminiCanCodec.encode(CtrlCommand(command.gear, command.speed_mps, command.steering_deg, alive.next()))
    transport.send_frame(frame, timestamp=now)


def _odo(detail: dict | None) -> float | None:
    value = None if detail is None else detail.get("odometer")
    return None if value is None else float(value["cumulative_distance_m"])


def _wheel(detail: dict | None, side: str, field: str) -> float | int | None:
    value = None if detail is None else detail.get("wheels", {}).get(side)
    return None if value is None else value.get(field)


def _value(value):
    """Return an enum-safe value suitable for the persisted safety trace."""

    return getattr(value, "value", value)


def permission_drop_trace(
    *,
    timestamp_monotonic_sec: float,
    previous_state: BackendInterlockState,
    result,
    command: BackendCommand,
    feedback,
) -> dict | None:
    """Record the exact contract failure when permission is revoked.

    This deliberately lives beside the physical runner, rather than in the
    Generic Gate: it is chassis feedback evidence for a transition that the
    backend interlock has already decided to fail closed.
    """

    if (previous_state is not BackendInterlockState.MOTION_PERMITTED
            or result.state is BackendInterlockState.MOTION_PERMITTED):
        return None
    return {
        "timestamp_monotonic_sec": timestamp_monotonic_sec,
        "previous_state": previous_state.value,
        "current_state": result.state.value,
        "requested_gear": _value(command.gear),
        "feedback_gear": _value(feedback.gear),
        "requested_speed_mps": command.speed_mps,
        "actual_speed_mps": feedback.speed_mps,
        "requested_steering_deg": command.steering_deg,
        "actual_steering_deg": feedback.steering_deg,
        "feedback_mode": _value(feedback.mode),
        "auto_can_error": feedback.auto_can_error,
        "fault_level": feedback.vehicle_fault_level,
        "hard_fault": feedback.hard_fault,
        "ctrl_feedback_age_sec": feedback.ctrl_age_sec,
        "diagnostic_feedback_age_sec": feedback.diagnostic_age_sec,
        "ctrl_alive_valid": feedback.ctrl_alive_contiguous,
        "diagnostic_alive_valid": feedback.diagnostic_alive_contiguous,
        "ctrl_checksum_valid": feedback.ctrl_checksum_valid,
        "diagnostic_checksum_valid": feedback.diagnostic_checksum_valid,
        "can_health": feedback.can_state,
        "transition_reason": list(result.reasons),
    }


def run_ground_motion(interface: str, acknowledgement: str, report_path: Path) -> int:
    authority = R3lR9GroundAuthority(acknowledgement)
    authority.require_tx()
    monitor = LiveFeedbackMonitor(interface)
    can_monitor = LiveCanMonitor(interface)
    tx = SocketCanTxTransport(interface, authority=authority)
    core = MkminiBackendInterlock(R9_INTERLOCK_CONFIG)
    fastlio = FastLioSpeedMonitor()
    alive = AliveCounter()
    started = time.monotonic()
    samples = []
    transitions = []
    permission_transition_trace = []
    emitted = 0
    max_actual_speed = 0.0
    max_fastlio_speed = 0.0
    fastlio_baseline = None
    fastlio_final = None
    max_abs_steering = 0.0
    max_left_speed = 0.0
    max_right_speed = 0.0
    min_left_signed_speed = 0.0
    min_right_signed_speed = 0.0
    max_left_signed_speed = 0.0
    max_right_signed_speed = 0.0
    odometer_start = None
    odometer_end = None
    left_pulse_start = None
    right_pulse_start = None
    left_pulse_end = None
    right_pulse_end = None
    motion_started = None
    motion_ended = None
    permission_time = None
    permission_lost = False
    final_zero_confirmed = False
    status = "BLOCKED_FIRST_GROUND_MOTION"
    final_detail = None
    scheduler = None
    command_lock = threading.Lock()
    scheduled_command = BackendCommand.safe_neutral()
    monitor.open()
    can_monitor.open()
    fastlio.open()
    try:
        # Receive-only source must be alive and stationary before CAN TX can
        # start.  Its own timestamp must advance; a missing source is never
        # interpreted as a safe zero measurement.
        fastlio_deadline = time.monotonic() + 6.0
        while time.monotonic() < fastlio_deadline:
            fastlio_baseline = fastlio.snapshot()
            if (fastlio_baseline["valid"] and fastlio_baseline["sample_count"] >= 10
                    and fastlio_baseline["speed_mps"] is not None):
                break
            time.sleep(.05)
        else:
            return_code = 2
            raise GroundMotionBlocked("FASTLIO_ODOMETRY_NOT_READY")
        if fastlio_baseline["max_speed_mps"] > FASTLIO_SPEED_CEILING_MPS:
            return_code = 2
            raise GroundMotionBlocked("FASTLIO_STATIONARY_SPEED_OVERSPEED")
        baseline_deadline = time.monotonic() + 1.0
        while time.monotonic() < baseline_deadline:
            now = time.monotonic()
            feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
            if not core.assess_feedback(feedback, require_n_auto=False):
                break
            time.sleep(OUTPUT_PERIOD_SEC)
        else:
            return_code = 2
            raise GroundMotionBlocked("BASE_FEEDBACK")

        tx.open()
        core.arm(time.monotonic())
        def heartbeat_send():
            with command_lock:
                command = scheduled_command
            _send(tx, alive, command, None)
        scheduler = HeartbeatScheduler(heartbeat_send, period_sec=.010, lease_sec=.025)
        scheduler.start(state=core.state.value)
        previous_interlock_state = core.state

        def interlock_step(now, command, feedback):
            nonlocal previous_interlock_state, scheduled_command
            result = core.step(now, command, feedback)
            trace = permission_drop_trace(
                timestamp_monotonic_sec=now,
                previous_state=previous_interlock_state,
                result=result,
                command=command,
                feedback=feedback,
            )
            if trace is not None:
                permission_transition_trace.append(trace)
            previous_interlock_state = result.state
            if result.command_admitted:
                with command_lock:
                    scheduled_command = result.output
                scheduler.publish(state=result.state.value, now=now)
            return result

        next_emit = time.monotonic()
        qualification_deadline = next_emit + 5.0
        last_state = None
        while time.monotonic() < qualification_deadline:
            now = time.monotonic()
            feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
            result = interlock_step(now, BackendCommand.safe_neutral(), feedback)
            if result.state != last_state:
                transitions.append({"elapsed_sec": now - started, "state": result.state.value,
                                    "reasons": list(result.reasons)})
                last_state = result.state
            if result.motion_permitted:
                permission_time = now
                break
            if result.state is BackendInterlockState.MOTION_NOT_PERMITTED:
                return_code = 2
                raise GroundMotionBlocked("INTERLOCK_QUALIFICATION")
            if scheduler.fault is not None:
                return_code = 2
                raise GroundMotionBlocked("HEARTBEAT_SCHEDULER_" + scheduler.fault)
            next_emit += OUTPUT_PERIOD_SEC
            delay = next_emit - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_emit = time.monotonic()
        if permission_time is None:
            return_code = 2
            raise GroundMotionBlocked("INTERLOCK_TIMEOUT")

        odometer_start = _odo(final_detail)
        fastlio.reset_distance_origin()
        left_pulse_start = _wheel(final_detail, "left", "pulse_count")
        right_pulse_start = _wheel(final_detail, "right", "pulse_count")
        drive = commissioning_drive_command(COMMAND_SPEED_MPS)
        motion_started = time.monotonic()
        next_emit = motion_started
        while time.monotonic() - motion_started < MAX_COMMAND_DURATION_SEC:
            now = time.monotonic()
            feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
            result = interlock_step(now, drive, feedback)
            fastlio_current = fastlio.snapshot(now)
            if not fastlio_current["valid"]:
                permission_lost = True
                status = fastlio_current["reason"] or "FASTLIO_ODOMETRY_INVALID"
            elif fastlio_current["speed_mps"] is not None:
                max_fastlio_speed = max(max_fastlio_speed, fastlio_current["speed_mps"])
                if fastlio_current["speed_mps"] > FASTLIO_SPEED_CEILING_MPS:
                    permission_lost = True
                    status = "FASTLIO_GROUND_SPEED_OVERSPEED"
            # Keep the exact triggering feedback too.  R3 recorded only
            # already-permitted samples, which made its drop non-replayable.
            max_actual_speed = max(max_actual_speed, float(feedback.speed_mps or 0.0))
            max_abs_steering = max(max_abs_steering, abs(float(feedback.steering_deg or 0.0)))
            if len(samples) == 0 or now - samples[-1]["monotonic_sec"] >= 0.05 or not result.motion_permitted:
                samples.append({"monotonic_sec": now, "elapsed_motion_sec": now - motion_started,
                                "feedback": _jsonable(feedback), "odometer_m": _odo(final_detail),
                                "left_speed_mps": _wheel(final_detail, "left", "speed_mps"),
                                "right_speed_mps": _wheel(final_detail, "right", "speed_mps"),
                                "interlock_state": result.state.value,
                                "interlock_reasons": list(result.reasons)})
            if not result.motion_permitted or permission_lost:
                permission_lost = True
                status = "PERMISSION_LOST_DURING_MOTION"
                with command_lock:
                    scheduled_command = BackendCommand.safe_neutral()
                scheduler.stop()
                _send(tx, alive, BackendCommand.safe_neutral(), now)
                emitted += 1
                break
            if scheduler.fault is not None:
                permission_lost = True
                status = "HEARTBEAT_SCHEDULER_" + scheduler.fault
                with command_lock:
                    scheduled_command = BackendCommand.safe_neutral()
                scheduler.stop()
                _send(tx, alive, BackendCommand.safe_neutral(), now)
                emitted += 1
                break
            left_speed = float(_wheel(final_detail, "left", "speed_mps") or 0.0)
            right_speed = float(_wheel(final_detail, "right", "speed_mps") or 0.0)
            max_left_speed = max(max_left_speed, abs(left_speed))
            max_right_speed = max(max_right_speed, abs(right_speed))
            min_left_signed_speed = min(min_left_signed_speed, left_speed)
            min_right_signed_speed = min(min_right_signed_speed, right_speed)
            max_left_signed_speed = max(max_left_signed_speed, left_speed)
            max_right_signed_speed = max(max_right_signed_speed, right_speed)
            odometer_end = _odo(final_detail)
            if odometer_start is not None and odometer_end is not None and abs(odometer_end - odometer_start) >= TARGET_DISTANCE_M:
                break
            next_emit += OUTPUT_PERIOD_SEC
            delay = next_emit - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_emit = time.monotonic()
        motion_ended = time.monotonic()

        if not permission_lost:
            stopping = BackendCommand(Gear.D, 0.0, 0.0)
            stopped_since = None
            stop_deadline = time.monotonic() + 2.0
            while time.monotonic() < stop_deadline:
                now = time.monotonic()
                feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
                result = interlock_step(now, stopping, feedback)
                if not result.motion_permitted:
                    permission_lost = True
                    status = "PERMISSION_LOST_DURING_STOP"
                    with command_lock:
                        scheduled_command = BackendCommand.safe_neutral()
                    scheduler.stop()
                    _send(tx, alive, BackendCommand.safe_neutral(), now)
                    emitted += 1
                    break
                if scheduler.fault is not None:
                    permission_lost = True
                    status = "HEARTBEAT_SCHEDULER_" + scheduler.fault
                    with command_lock:
                        scheduled_command = BackendCommand.safe_neutral()
                    scheduler.stop()
                    _send(tx, alive, BackendCommand.safe_neutral(), now)
                    emitted += 1
                    break
                wheel_zero = all(abs(float(_wheel(final_detail, side, "speed_mps") or 0.0)) <= 0.005
                                 for side in ("left", "right"))
                ctrl_zero = feedback.speed_mps is not None and abs(feedback.speed_mps) <= 0.005
                if ctrl_zero and wheel_zero:
                    stopped_since = now if stopped_since is None else stopped_since
                    if now - stopped_since >= 0.30:
                        final_zero_confirmed = True
                        break
                else:
                    stopped_since = None
                time.sleep(OUTPUT_PERIOD_SEC)

        # Return to neutral only after the measured stop, or immediately on a
        # fail-closed permission loss. Keep this bounded, then close TX.
        for _ in range(30):
            now = time.monotonic()
            feedback, final_detail = monitor.snapshot(now, can_monitor.snapshot())
            interlock_step(now, BackendCommand.safe_neutral(), feedback)
            if scheduler.fault is not None:
                break
            time.sleep(OUTPUT_PERIOD_SEC)
        odometer_end = _odo(final_detail)
        fastlio_final = fastlio.snapshot()
        left_pulse_end = _wheel(final_detail, "left", "pulse_count")
        right_pulse_end = _wheel(final_detail, "right", "pulse_count")
        distance = None if odometer_start is None or odometer_end is None else abs(odometer_end - odometer_start)
        forward_wheels = (
            max_left_signed_speed > 0.0 and max_right_signed_speed > 0.0
            and min_left_signed_speed >= -0.005 and min_right_signed_speed >= -0.005
            and (left_pulse_start is None or left_pulse_end is None or left_pulse_end != left_pulse_start)
            and (right_pulse_start is None or right_pulse_end is None or right_pulse_end != right_pulse_start)
        )
        pass_result = (
            not permission_lost and final_zero_confirmed and distance is not None
            and odometer_end > odometer_start
            and 0.20 <= distance <= 0.30
            and max_actual_speed <= COMMISSIONING_FEEDBACK_CEILING_MPS + 1e-9
            and fastlio_final is not None and fastlio_final["valid"]
            and max_fastlio_speed <= FASTLIO_SPEED_CEILING_MPS + 1e-9
            and max_abs_steering <= 0.10 and forward_wheels
        )
        status = "PASS" if pass_result else "BLOCKED_FIRST_GROUND_MOTION"
        return_code = 0 if pass_result else 2
    except GroundMotionBlocked:
        return_code = 2
    finally:
        if tx.is_open:
            try:
                if scheduler is not None:
                    scheduler.stop()
                _send(tx, alive, BackendCommand.safe_neutral(), time.monotonic())
                emitted += 1
            finally:
                tx.close()
        monitor.close()
        can_monitor.close()
        fastlio.close()
    return _write_report(report_path, locals(), return_code)


def _write_report(path: Path, scope: dict, return_code: int) -> int:
    def delta(end, start):
        return None if end is None or start is None else end - start
    odo_delta = delta(scope.get("odometer_end"), scope.get("odometer_start"))
    report = {
        "status": scope.get("status"),
        "command": {"gear": "D", "speed_mps": COMMAND_SPEED_MPS, "steering_deg": 0.0},
        "commissioning_feedback_ceiling_mps": COMMISSIONING_FEEDBACK_CEILING_MPS,
        "fastlio_speed_ceiling_mps": FASTLIO_SPEED_CEILING_MPS,
        "target_distance_m": TARGET_DISTANCE_M,
        "maximum_command_duration_sec": MAX_COMMAND_DURATION_SEC,
        "time_to_motion_permission_sec": None if scope.get("permission_time") is None else scope["permission_time"] - scope["started"],
        "motion_command_duration_sec": None if scope.get("motion_started") is None or scope.get("motion_ended") is None else scope["motion_ended"] - scope["motion_started"],
        "odometer_start_m": scope.get("odometer_start"),
        "odometer_end_m": scope.get("odometer_end"),
        "odometer_delta_m": odo_delta,
        "distance_abs_m": None if odo_delta is None else abs(odo_delta),
        "left_pulse_delta": delta(scope.get("left_pulse_end"), scope.get("left_pulse_start")),
        "right_pulse_delta": delta(scope.get("right_pulse_end"), scope.get("right_pulse_start")),
        "max_commanded_speed_mps": COMMAND_SPEED_MPS,
        "max_actual_ctrl_speed_mps": scope.get("max_actual_speed"),
        "max_fastlio_ground_speed_mps": scope.get("max_fastlio_speed"),
        "fastlio_baseline": scope.get("fastlio_baseline"),
        "fastlio_final": scope.get("fastlio_final"),
        "max_left_wheel_speed_mps": scope.get("max_left_speed"),
        "max_right_wheel_speed_mps": scope.get("max_right_speed"),
        "min_left_signed_speed_mps": scope.get("min_left_signed_speed"),
        "min_right_signed_speed_mps": scope.get("min_right_signed_speed"),
        "max_left_signed_speed_mps": scope.get("max_left_signed_speed"),
        "max_right_signed_speed_mps": scope.get("max_right_signed_speed"),
        "max_abs_steering_deg": scope.get("max_abs_steering"),
        "permission_lost": scope.get("permission_lost"),
        "permission_transition_trace": scope.get("permission_transition_trace"),
        "final_zero_confirmed": scope.get("final_zero_confirmed"),
        "application_can_tx_stopped": not scope["tx"].is_open,
        "heartbeat_frames_total": scope.get("emitted"),
        "heartbeat_timing": [] if scope.get("scheduler") is None else [
            {"scheduled_time": item.scheduled_time, "actual_send_time": item.actual_send_time,
             "inter_send_interval_sec": item.inter_send_interval_sec,
             "scheduler_lateness_sec": item.scheduler_lateness_sec,
             "send_duration_sec": item.send_duration_sec, "state": item.state}
            for item in scope["scheduler"].records()],
        "heartbeat_timing_summary": timing_summary([] if scope.get("scheduler") is None else scope["scheduler"].records()),
        "heartbeat_scheduler_fault": None if scope.get("scheduler") is None else scope["scheduler"].fault,
        "transitions": scope.get("transitions"),
        "motion_samples": scope.get("samples"),
        "final_feedback": scope.get("final_detail"),
        "final_can": _can_link(scope["interface"]),
        "ros_localization_available": True,
        "odometry_authority": "FASTLIO_CONSECUTIVE_POSE_DISPLACEMENT_AND_MKMINI_CAN_ODOMETER",
    }
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return return_code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--operator-ack", default="")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    return run_ground_motion(args.interface, args.operator_ack, args.report)


if __name__ == "__main__":
    raise SystemExit(main())
