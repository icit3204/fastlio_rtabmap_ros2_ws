"""Deterministic qualification-side contracts for P4-E.6C progress cases."""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable


NO_PROGRESS_THRESHOLD_SEC = 20.0
RECOVERY_DELTA_THRESHOLD = 6
MAX_INJECTED_RECOVERIES = 6
POSE_CLAMP_RATE_HZ = 10.0

CASES = {
    "C-P01": {"reason": "CONTROLLER_NO_PROGRESS", "marker": "CP01_ATTEMPT1_RUNTIME_STARTED"},
    "C-P02": {"reason": "RECOVERY_EXHAUSTED_NO_PROGRESS", "marker": "CP02_ATTEMPT1_RUNTIME_STARTED"},
}


@dataclass(frozen=True)
class ProgressAdmission:
    mission_active: bool
    policy_active: bool
    gate_armed: bool
    collision_clear: bool
    health_current: bool
    odom_current: bool
    tf_current: bool
    feedback_current: bool
    command_pair_current: bool
    movement_intent: bool
    action_terminal: bool = False
    recovery_delta: int = 0


def case_config(case: str) -> dict:
    try:
        return dict(CASES[str(case).upper()])
    except KeyError as exc:
        raise RuntimeError("P4E6C_CASE_DENIED") from exc


def validate_pose_clamp_gate(evidence: ProgressAdmission) -> None:
    required = (evidence.mission_active, evidence.policy_active, evidence.gate_armed,
                evidence.collision_clear, evidence.health_current, evidence.odom_current,
                evidence.tf_current, evidence.feedback_current,
                evidence.command_pair_current, evidence.movement_intent)
    if not all(required) or evidence.action_terminal or evidence.recovery_delta >= RECOVERY_DELTA_THRESHOLD:
        raise RuntimeError("P4E6C_POSE_CLAMP_GATE_DENIED")


def validate_no_progress_contract(*, no_progress_age_sec: float,
                                  measurable_progress: bool,
                                  recovery_delta: int,
                                  collision_clear: bool,
                                  competing_reason: str | None = None) -> None:
    if measurable_progress or no_progress_age_sec < NO_PROGRESS_THRESHOLD_SEC:
        raise RuntimeError("P4E6C_NO_PROGRESS_THRESHOLD_FAILURE")
    if recovery_delta >= RECOVERY_DELTA_THRESHOLD:
        raise RuntimeError("P4E6C_RECOVERY_PREEMPTS_CONTROLLER_NO_PROGRESS")
    if not collision_clear or competing_reason:
        raise RuntimeError("P4E6C_COMPETING_REASON_FAILURE")


def validate_recovery_exhaustion(*, recovery_delta: int, measurable_progress: bool,
                                 collision_clear: bool, competing_reason: str | None = None) -> None:
    if recovery_delta < RECOVERY_DELTA_THRESHOLD or measurable_progress:
        raise RuntimeError("P4E6C_RECOVERY_EXHAUSTION_NOT_QUALIFIED")
    if not collision_clear or competing_reason:
        raise RuntimeError("P4E6C_COMPETING_REASON_FAILURE")


def validate_live_observation(row: dict, *, case: str) -> None:
    required = ("odom_current", "tf_current", "feedback_current", "health_current",
                "command_pair_current", "collision_clear", "movement_intent")
    if any(not bool(row.get(key)) for key in required):
        raise RuntimeError("P4E6C_LIVE_OBSERVABILITY_FAILURE")
    if bool(row.get("action_terminal")) or row.get("competing_reason"):
        raise RuntimeError("P4E6C_COMPETING_REASON_FAILURE")
    recovery_delta = int(row.get("recovery_delta", 0))
    if str(case).upper() == "C-P01" and recovery_delta >= RECOVERY_DELTA_THRESHOLD:
        raise RuntimeError("P4E6C_RECOVERY_PREEMPTS_CONTROLLER_NO_PROGRESS")


def validate_feedback_cadence(receipt_times_sec: list[float], *, remaining_window_sec: float,
                               required_steps: int = 6) -> dict:
    if len(receipt_times_sec) < required_steps:
        raise RuntimeError("P4E6C_INSUFFICIENT_FEEDBACK_CADENCE")
    ordered = [float(value) for value in receipt_times_sec]
    if any(right <= left for left, right in zip(ordered, ordered[1:])):
        raise RuntimeError("P4E6C_FEEDBACK_CADENCE_NOT_MONOTONIC")
    elapsed = ordered[required_steps - 1] - ordered[0]
    if elapsed >= float(remaining_window_sec):
        raise RuntimeError("P4E6C_RECOVERY_SEQUENCE_WINDOW_FAILURE")
    return {"required_steps": required_steps, "observed_steps": len(ordered),
            "elapsed_sec": elapsed, "remaining_window_sec": float(remaining_window_sec)}


def clamp_tick_times(start_sec: float, end_sec: float, rate_hz: float = POSE_CLAMP_RATE_HZ) -> list[float]:
    if rate_hz <= 0 or end_sec < start_sec:
        raise ValueError("invalid pose clamp interval")
    period = 1.0 / rate_hz
    count = int((end_sec - start_sec) / period) + 1
    return [start_sec + i * period for i in range(count)]


class LiveProgressRunner:
    """Post-marker stimulation and observability contract.

    Callbacks are supplied by the supervisor-owned ROS launch.  This class
    never creates a RouteMission or Gate authority by itself.
    """

    def __init__(self, case: str, *, monotonic_ns=None):
        self.case = str(case).upper()
        case_config(self.case)
        self.monotonic_ns = monotonic_ns or time.monotonic_ns
        self.clamp_events = []
        self.recovery_events = []

    def validate_pre_marker(self, evidence: ProgressAdmission, *, marker_present: bool) -> None:
        if marker_present:
            raise RuntimeError("P4E6C_PRE_MARKER_STATE_INVALID")
        validate_pose_clamp_gate(evidence)

    @staticmethod
    def _real_wait_until_ns(deadline_ns: int) -> None:
        remaining = (deadline_ns - time.monotonic_ns()) / 1e9
        if remaining > 0:
            time.sleep(remaining)

    def run_pose_clamp(self, publish_initialpose, *, duration_sec: float,
                       rate_hz: float = POSE_CLAMP_RATE_HZ, marker_present: bool = True,
                       wait_until_ns: Callable[[int], None] | None = None,
                       on_tick: Callable[[dict], bool | None] | None = None):
        if not marker_present:
            raise RuntimeError("P4E6C_CLAMP_BEFORE_MARKER")
        if rate_hz != POSE_CLAMP_RATE_HZ:
            raise RuntimeError("P4E6C_CLAMP_RATE_DENIED")
        wait_until_ns = wait_until_ns or self._real_wait_until_ns
        start_ns = self.monotonic_ns()
        period_ns = int(round(1e9 / rate_hz))
        end_ns = start_ns + int(round(float(duration_sec) * 1e9))
        next_deadline_ns = start_ns
        previous_actual_ns = None
        while next_deadline_ns <= end_ns:
            wait_until_ns(next_deadline_ns)
            actual_ns = self.monotonic_ns()
            publish_initialpose()
            event = {"scheduled_monotonic_ns": next_deadline_ns,
                     "actual_publish_monotonic_ns": actual_ns,
                     "actual_interval_sec": (None if previous_actual_ns is None
                                               else (actual_ns - previous_actual_ns) / 1e9),
                     "cumulative_elapsed_sec": (actual_ns - start_ns) / 1e9,
                     "rate_hz": rate_hz}
            self.clamp_events.append(event)
            previous_actual_ns = actual_ns
            if on_tick is not None and on_tick(event):
                break
            next_deadline_ns += period_ns
        return {"start_monotonic_ns": start_ns, "count": len(self.clamp_events),
                "rate_hz": rate_hz, "events": list(self.clamp_events),
                "stopped_by_callback": bool(on_tick and self.clamp_events and
                                              self.clamp_events[-1].get("stop"))}

    def run_cp01(self, *, evidence: ProgressAdmission, publish_initialpose,
                 observe, duration_sec: float, marker_present: bool = True,
                 wait_until_ns: Callable[[int], None] | None = None,
                 record_clamp: Callable[[dict], None] | None = None,
                 record_progress: Callable[[dict], None] | None = None) -> dict:
        self.validate_pre_marker(evidence, marker_present=False)
        if not marker_present:
            raise RuntimeError("P4E6C_CLAMP_BEFORE_MARKER")
        observations = []
        terminal_seen = False
        previous_sequence = -1
        previous_monotonic_ns = -1

        def tick(event):
            nonlocal terminal_seen
            try:
                row = observe()
            except RuntimeError as exc:
                if "STALE_EVIDENCE_ROW" not in str(exc):
                    raise
                row = None
            nonlocal previous_sequence, previous_monotonic_ns
            if row is None:
                if record_clamp is not None:
                    record_clamp(event)
                return False
            sequence = int(row.get("sequence", len(observations)))
            monotonic_ns = int(row.get("monotonic_ns", len(observations)))
            if sequence <= previous_sequence and monotonic_ns <= previous_monotonic_ns:
                raise RuntimeError("P4E6C_STALE_PROGRESS_OBSERVATION")
            previous_sequence, previous_monotonic_ns = sequence, monotonic_ns
            observations.append(row)
            if record_clamp is not None:
                record_clamp(event)
            if row.get("progress_event") == CASES["C-P01"]["reason"]:
                if float(row.get("no_progress_age_sec", 0.0)) < NO_PROGRESS_THRESHOLD_SEC:
                    raise RuntimeError("P4E6C_NO_PROGRESS_THRESHOLD_FAILURE")
                if int(row.get("recovery_delta", 0)) >= RECOVERY_DELTA_THRESHOLD:
                    raise RuntimeError("P4E6C_RECOVERY_PREEMPTS_CONTROLLER_NO_PROGRESS")
                terminal_seen = True
                event["stop"] = True
                return True
            validate_live_observation(row, case="C-P01")
            return False

        clamp = self.run_pose_clamp(publish_initialpose, duration_sec=duration_sec,
                                    marker_present=True, wait_until_ns=wait_until_ns,
                                    on_tick=tick)
        if not terminal_seen:
            raise RuntimeError("P4E6C_CONTROLLER_NO_PROGRESS_NOT_OBSERVED")
        terminal_row = observations[-1]
        return {"case": "C-P01", "expected_reason": CASES["C-P01"]["reason"],
                "actual_reason": terminal_row.get("progress_event"),
                "identity": {key: terminal_row.get(key) for key in
                              ("mission_id", "route_id", "active_goal_uuid")},
                "stimulation": {"pose_clamp": True}, "clamp": clamp,
                "observations": observations}

    def run_cp02(self, *, evidence: ProgressAdmission, publish_initialpose,
                 arm_recovery, observe_feedback, duration_sec: float,
                 marker_present: bool = True, canonical_receipt_times_sec: list[float] | None = None,
                 remaining_window_sec: float = NO_PROGRESS_THRESHOLD_SEC,
                 canonical_receipt_probe: Callable[[], list[float]] | None = None,
                 wait_until_ns: Callable[[int], None] | None = None,
                 record_clamp: Callable[[dict], None] | None = None,
                 record_recovery: Callable[[dict], None] | None = None,
                 step_recovery: Callable[[int], None] | None = None,
                 acknowledge_recovery: Callable[[dict], None] | None = None,
                 observe_source_ack: Callable[[int], dict] | None = None) -> dict:
        self.validate_pre_marker(evidence, marker_present=False)
        if not marker_present:
            raise RuntimeError("P4E6C_RECOVERY_BEFORE_MARKER")
        cadence_times = (canonical_receipt_probe() if canonical_receipt_probe is not None
                         else canonical_receipt_times_sec or [])
        cadence = validate_feedback_cadence(cadence_times,
                                            remaining_window_sec=remaining_window_sec)
        feedback = []
        armed = False
        target_uuid = None
        terminal_seen = False
        requested_step = 0
        acknowledged_step = 0
        previous_sequence = -1
        previous_monotonic_ns = -1

        def tick(event):
            nonlocal armed, target_uuid, terminal_seen
            if not armed:
                arm_recovery()
                armed = True
            try:
                row = observe_feedback()
            except RuntimeError as exc:
                if "STALE_EVIDENCE_ROW" not in str(exc):
                    raise
                row = None
            nonlocal previous_sequence, previous_monotonic_ns
            if row is None:
                if record_clamp is not None:
                    record_clamp(event)
                return False
            sequence = int(row.get("sequence", len(feedback)))
            monotonic_ns = int(row.get("monotonic_ns", len(feedback)))
            if sequence <= previous_sequence and monotonic_ns <= previous_monotonic_ns:
                raise RuntimeError("P4E6C_STALE_RECOVERY_OBSERVATION")
            previous_sequence, previous_monotonic_ns = sequence, monotonic_ns
            feedback.append(row)
            if record_recovery is not None:
                record_recovery(dict(row))
            if record_clamp is not None:
                record_clamp(event)
            uuid = row.get("goal_uuid")
            if target_uuid is None:
                target_uuid = uuid
            if uuid != target_uuid or int(row.get("canonical_recovery_count", 0)) != 0:
                raise RuntimeError("P4E6C_RECOVERY_UUID_OR_CANONICAL_MUTATION")
            count = int(row.get("shadow_recovery_count", -1))
            if count < 0 or count > MAX_INJECTED_RECOVERIES:
                raise RuntimeError("P4E6C_RECOVERY_SEQUENCE_FAILURE")
            if step_recovery is not None:
                nonlocal requested_step, acknowledged_step
                if count != acknowledged_step:
                    if count != acknowledged_step + 1 or count > requested_step:
                        raise RuntimeError("P4E6C_RECOVERY_SEQUENCE_FAILURE")
                    if observe_source_ack is None:
                        raise RuntimeError("P4E6C_SOURCE_ACK_OBSERVER_REQUIRED")
                    source_row = observe_source_ack(count)
                    acknowledged_step = count
                    if acknowledge_recovery is not None:
                        acknowledge_recovery(source_row)
                if requested_step < MAX_INJECTED_RECOVERIES and acknowledged_step == requested_step:
                    requested_step += 1
                    step_recovery(requested_step)
                if count == 0:
                    return False
            else:
                expected = min(len(feedback), MAX_INJECTED_RECOVERIES)
                if count != expected or count < 1:
                    raise RuntimeError("P4E6C_RECOVERY_SEQUENCE_FAILURE")
            if count < MAX_INJECTED_RECOVERIES and row.get("progress_event"):
                raise RuntimeError("P4E6C_RECOVERY_EARLY_TERMINAL")
            if count == MAX_INJECTED_RECOVERIES:
                if row.get("measurable_progress", False):
                    raise RuntimeError("P4E6C_RECOVERY_PROGRESS_PRESENT")
                if row.get("actual_reason") == CASES["C-P02"]["reason"]:
                    terminal_seen = True
                    event["stop"] = True
                    return True
                if row.get("actual_reason") and row.get("actual_reason") != "NO_PROGRESS_PENDING":
                    raise RuntimeError("P4E6C_RECOVERY_COMPETING_REASON")
                return False
            return False

        clamp = self.run_pose_clamp(publish_initialpose, duration_sec=duration_sec,
                                    marker_present=True, wait_until_ns=wait_until_ns,
                                    on_tick=tick)
        if not armed or not terminal_seen:
            raise RuntimeError("P4E6C_RECOVERY_TERMINAL_NOT_OBSERVED")
        self.recovery_events = feedback
        terminal_row = feedback[-1]
        return {"case": "C-P02", "expected_reason": CASES["C-P02"]["reason"],
                "actual_reason": terminal_row.get("actual_reason"),
                "identity": {key: terminal_row.get(key) for key in
                              ("mission_id", "route_id", "active_goal_uuid")},
                "stimulation": {"pose_clamp": True, "recovery_injection": True},
                "clamp": clamp, "cadence": cadence, "recovery_feedback": feedback}
