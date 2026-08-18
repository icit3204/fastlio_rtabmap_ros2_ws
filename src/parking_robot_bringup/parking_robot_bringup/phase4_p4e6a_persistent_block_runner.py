"""P4-E.6A fake-only persistent healthy Collision Monitor STOP runner."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import queue
import threading
import time

from action_msgs.msg import GoalStatus
from parking_robot_interfaces.msg import MissionState
import rclpy

from .phase4_p4e1b_clear_runner import LIFECYCLE, TOPICS, nz, rates
from .phase4_p4e2a_slowdown_runner import pair_commands
from .phase4_p4e5b_temporary_block_runner import (
    P4E5BRunner, command_mag, first_pair, health_ok, state_after,
)
from .phase4_p4e6a7_clear_domain_runner import prospective_epoch


CHECKPOINT_SECONDS = (2, 5, 10, 15, 18, 19, 19.5)
CALLBACK_DRAIN_SEC = 1.0
FORMAL_READINESS_EPOCH_SEC = 2.0
STABLE_CLEAR_SEC = 5.0
PAIR_LIVENESS_NS = 250_000_000
TEMPORARY_STOP_NS = 1_000_000_000
PERSISTENT_STOP_NS = 20_000_000_000
EVIDENCE_QUEUE_CAPACITY = 4096
EVIDENCE_DRAIN_TIMEOUT_SEC = 10.0
MAXIMUM_STATIONARY_TRANSLATION_M = 0.02
FROZEN_SUPERVISOR_SHA256 = "b4c3315796fb52c8b84a76c88e822bf51b49d2e3cae4fac138abd067ae749fe3"


class EvidenceAdjudicationError(RuntimeError):
    """A bounded, authoritative A.8 evidence failure."""

    def __init__(self, reason, detail):
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


class EvidenceWriterError(RuntimeError):
    """Fail-closed bounded evidence persistence error."""

    def __init__(self, reason, detail):
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


class BoundedEvidenceWriter:
    """Persist immutable JSON records off the ROS callback/spin thread."""

    def __init__(self, output_dir, *, capacity=EVIDENCE_QUEUE_CAPACITY,
                 write_delay_sec=0.0, initial_stall_sec=0.0,
                 drain_timeout_sec=EVIDENCE_DRAIN_TIMEOUT_SEC, fail_sequence=None):
        if capacity <= 0:
            raise ValueError("evidence queue capacity must be positive")
        self.output_dir = Path(output_dir)
        self.capacity = int(capacity)
        self.write_delay_sec = float(write_delay_sec)
        self.initial_stall_sec = float(initial_stall_sec)
        self.drain_timeout_sec = float(drain_timeout_sec)
        self.fail_sequence = fail_sequence
        self._queue = queue.Queue(maxsize=self.capacity)
        self._lock = threading.Lock()
        self._next_sequence = 1
        self._expected_sequence = 1
        self._persisted_sequence = 0
        self._failure = None
        self._handles = {}
        self._stopping = False
        self._finalized = False
        self._stop_event = threading.Event()
        self._initial_stall_complete = False
        self._thread = threading.Thread(
            target=self._run, name="p4e6a8b-evidence-writer", daemon=False)
        self._thread.start()

    def _raise_if_failed(self):
        if self._failure is not None:
            raise EvidenceWriterError(self._failure[0], self._failure[1])

    def enqueue(self, name, item):
        """Assign one sequence and enqueue without waiting for disk durability."""
        self._raise_if_failed()
        with self._lock:
            if self._stopping:
                raise EvidenceWriterError(
                    "P4E6A8B2_EVIDENCE_WRITER_INTEGRITY_NEEDS_REVIEW",
                    "enqueue after writer finalization began")
            sequence = self._next_sequence
            self._next_sequence += 1
        record = dict(item)
        record["evidence_sequence"] = sequence
        immutable = json.dumps(record, sort_keys=True, separators=(",", ":"))
        try:
            self._queue.put_nowait((sequence, str(name), immutable))
        except queue.Full as exc:
            self._failure = (
                "P4E6A8B2_EVIDENCE_QUEUE_OVERFLOW_NEEDS_REVIEW",
                f"capacity={self.capacity}, sequence={sequence}")
            raise EvidenceWriterError(*self._failure) from exc
        return record

    def inject_record_for_test(self, sequence, name, item):
        """Test-only boundary for deterministic gap/out-of-order qualification."""
        immutable = json.dumps(dict(item), sort_keys=True, separators=(",", ":"))
        self._queue.put_nowait((int(sequence), str(name), immutable))

    def _fail(self, reason, detail):
        if self._failure is None:
            self._failure = (reason, detail)

    def _run(self):
        try:
            while True:
                try:
                    record = self._queue.get(timeout=0.05)
                except queue.Empty:
                    if self._stop_event.is_set():
                        break
                    continue
                if record is None:
                    self._queue.task_done()
                    break
                sequence, name, immutable = record
                try:
                    if sequence != self._expected_sequence:
                        reason = ("P4E6A8B2_EVIDENCE_SEQUENCE_MISSING_NEEDS_REVIEW"
                                  if sequence > self._expected_sequence else
                                  "P4E6A8B2_EVIDENCE_SEQUENCE_OUT_OF_ORDER_NEEDS_REVIEW")
                        self._fail(
                            reason,
                            f"expected={self._expected_sequence}, observed={sequence}")
                        continue
                    if self.fail_sequence == sequence:
                        raise OSError(f"injected writer failure at sequence {sequence}")
                    if not self._initial_stall_complete and self.initial_stall_sec:
                        self._stop_event.wait(self.initial_stall_sec)
                        self._initial_stall_complete = True
                    if self.write_delay_sec:
                        self._stop_event.wait(self.write_delay_sec)
                    handle = self._handles.get(name)
                    if handle is None:
                        handle = (self.output_dir / name).open("a", encoding="utf-8")
                        self._handles[name] = handle
                    handle.write(immutable + "\n")
                    self._persisted_sequence = sequence
                    self._expected_sequence += 1
                except Exception as exc:  # worker boundary must become explicit state
                    self._fail(
                        "P4E6A8B2_EVIDENCE_WRITER_EXCEPTION_NEEDS_REVIEW", repr(exc))
                finally:
                    self._queue.task_done()
        except Exception as exc:
            self._fail("P4E6A8B2_EVIDENCE_WRITER_EXCEPTION_NEEDS_REVIEW", repr(exc))

    def status(self):
        return {
            "capacity": self.capacity,
            "queue_depth": self._queue.qsize(),
            "next_sequence": self._next_sequence,
            "persisted_sequence": self._persisted_sequence,
            "failure_reason": None if self._failure is None else self._failure[0],
            "failure_detail": None if self._failure is None else self._failure[1],
            "worker_alive": self._thread.is_alive(),
        }

    def finalize(self, *, timeout_sec=None):
        """Drain in order, fsync once off callback cadence, and stop the worker."""
        if self._finalized:
            self._raise_if_failed()
            return self.status()
        timeout = self.drain_timeout_sec if timeout_sec is None else float(timeout_sec)
        with self._lock:
            self._stopping = True
            final_sequence = self._next_sequence - 1
        deadline = time.monotonic() + timeout
        while self._queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.001)
        if self._queue.unfinished_tasks:
            self._fail("P4E6A8B2_EVIDENCE_DRAIN_TIMEOUT_NEEDS_REVIEW",
                       f"unfinished={self._queue.unfinished_tasks}")
            self._stop_event.set()
            recovery_deadline = time.monotonic() + 1.0
            while self._queue.unfinished_tasks and time.monotonic() < recovery_deadline:
                time.sleep(0.001)
        if self._persisted_sequence != final_sequence:
            self._fail("P4E6A8B2_EVIDENCE_SEQUENCE_MISSING_NEEDS_REVIEW",
                       f"final={final_sequence}, persisted={self._persisted_sequence}")
        try:
            self._queue.put(None, timeout=1.0)
        except queue.Full:
            self._fail("P4E6A8B2_EVIDENCE_DRAIN_TIMEOUT_NEEDS_REVIEW",
                       "unable to enqueue writer sentinel")
        self._stop_event.set()
        self._thread.join(1.0)
        if self._thread.is_alive():
            self._fail("P4E6A8B2_EVIDENCE_DRAIN_TIMEOUT_NEEDS_REVIEW",
                       "writer thread did not terminate")
        for handle in self._handles.values():
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
        self._handles.clear()
        self._finalized = True
        self._raise_if_failed()
        return self.status()


class PersistentEvidenceRunner(P4E5BRunner):
    """P4-E.6A runner with callback-isolated evidence persistence only."""

    def __init__(self, out, *, writer_capacity=EVIDENCE_QUEUE_CAPACITY,
                 writer_delay_sec=0.0):
        super().__init__(out)
        self.evidence_writer = BoundedEvidenceWriter(
            out, capacity=writer_capacity, write_delay_sec=writer_delay_sec)

    def emit(self, name, item):
        record = dict(item)
        record.setdefault("monotonic_ns", time.monotonic_ns())
        record.setdefault("ros_ns", self.get_clock().now().nanoseconds)
        persisted = self.evidence_writer.enqueue(name, record)
        self.events.append(persisted)
        return persisted

    def wait_for_state_history(self, state, after_ns, reason, timeout, timeout_reason,
                               *, mission_id=None, route_id=None, waypoint_index=None,
                               goal_uuid=None, before_ns=None):
        """Find a serialized transition even when a later live state is current."""
        deadline = time.monotonic() + float(timeout)
        while True:
            match = find_transition_since(
                self.states, state=state, after_ns=after_ns, reason=reason,
                mission_id=mission_id, route_id=route_id,
                waypoint_index=waypoint_index, goal_uuid=goal_uuid,
                before_ns=before_ns)
            if match is not None:
                return match
            contradiction = first_terminal_contradiction(
                self.states, after_ns=after_ns, mission_id=mission_id,
                route_id=route_id, waypoint_index=waypoint_index,
                allowed_state=state, allowed_reason=reason)
            if contradiction is not None:
                raise RuntimeError(
                    f"{timeout_reason}: incompatible terminal transition: {contradiction}")
            if time.monotonic() >= deadline:
                raise RuntimeError(timeout_reason)
            rclpy.spin_once(self, timeout_sec=0.01)


TERMINAL_MISSION_STATES = frozenset((
    MissionState.SUCCEEDED, MissionState.FAILED, MissionState.CANCELLED,
    MissionState.BLOCKED, MissionState.HELP_REQUIRED,
))


def find_transition_since(rows, *, state, after_ns, reason=None, mission_id=None,
                          route_id=None, waypoint_index=None, goal_uuid=None,
                          before_ns=None):
    """Return the first identity-matched serialized state in a bounded interval."""
    for row in rows:
        receipt_ns = int(row["monotonic_ns"])
        if receipt_ns < int(after_ns) or (before_ns is not None
                                          and receipt_ns > int(before_ns)):
            continue
        if int(row["state"]) != int(state):
            continue
        if reason is not None and row.get("reason_code") != reason:
            continue
        if mission_id is not None and row.get("mission_id") != mission_id:
            continue
        if route_id is not None and row.get("route_id") != route_id:
            continue
        if waypoint_index is not None and int(row.get("waypoint_index", -1)) != int(waypoint_index):
            continue
        if goal_uuid is not None and row.get("active_goal_uuid") != goal_uuid:
            continue
        return row
    return None


def first_terminal_contradiction(rows, *, after_ns, mission_id=None, route_id=None,
                                 waypoint_index=None, allowed_state=None,
                                 allowed_reason=None):
    """Fail early only when the owned mission has made the requested path impossible."""
    for row in rows:
        if int(row["monotonic_ns"]) < int(after_ns):
            continue
        if mission_id is not None and row.get("mission_id") != mission_id:
            continue
        if route_id is not None and row.get("route_id") != route_id:
            continue
        if waypoint_index is not None and int(row.get("waypoint_index", -1)) != int(waypoint_index):
            continue
        state = int(row["state"])
        if state not in TERMINAL_MISSION_STATES:
            continue
        if state == int(allowed_state) and row.get("reason_code") == allowed_reason:
            continue
        return row
    return None


def adjudicate_persistent_terminal_history(*, states, action_statuses, persistent_ns,
                                           mission_id, route_id, waypoint_index,
                                           goal_uuid, history_start_ns=None):
    """Strict historical proof of persistent event -> cancel -> BLOCKED ordering."""
    # The state and diagnostic are separate subscriptions.  Their receipt order
    # is not the production source-event order, so search from the qualified STOP
    # anchor when the diagnostic receipt follows the causal CANCELLING state.
    search_start_ns = int(persistent_ns if history_start_ns is None else history_start_ns)
    cancelling = find_transition_since(
        states, state=MissionState.CANCELLING, after_ns=search_start_ns,
        reason="PERSISTENT_COLLISION_STOP", mission_id=mission_id,
        route_id=route_id, waypoint_index=waypoint_index, goal_uuid=goal_uuid)
    if cancelling is None:
        raise EvidenceAdjudicationError(
            "P4E6A8F1_CANCELLING_HISTORY_MISSING_NEEDS_REVIEW",
            "identity-matched persistent CANCELLING transition is absent")
    blocked = find_transition_since(
        states, state=MissionState.BLOCKED,
        after_ns=cancelling["monotonic_ns"], reason="PERSISTENT_COLLISION_STOP",
        mission_id=mission_id, route_id=route_id, waypoint_index=waypoint_index)
    if blocked is None:
        raise EvidenceAdjudicationError(
            "P4E6A8F1_BLOCKED_HISTORY_MISSING_NEEDS_REVIEW",
            "identity-matched persistent BLOCKED transition is absent")
    if int(cancelling["monotonic_ns"]) > int(blocked["monotonic_ns"]):
        raise EvidenceAdjudicationError(
            "P4E6A8F1_TERMINAL_ORDERING_NEEDS_REVIEW", "CANCELLING follows BLOCKED")
    acknowledgement = find_transition_since(
        states, state=MissionState.CANCELLING,
        after_ns=cancelling["monotonic_ns"], reason="BLOCK_CANCEL_ACK_ACCEPTED",
        mission_id=mission_id, route_id=route_id, waypoint_index=waypoint_index,
        goal_uuid=goal_uuid, before_ns=blocked["monotonic_ns"])
    canceling = [row for row in action_statuses
                 if row.get("goal_uuid") == goal_uuid
                 and row.get("status") == GoalStatus.STATUS_CANCELING
                 and int(cancelling["monotonic_ns"]) <= int(row["monotonic_ns"])
                 <= int(blocked["monotonic_ns"])]
    canceled = [row for row in action_statuses
                if row.get("goal_uuid") == goal_uuid
                and row.get("status") == GoalStatus.STATUS_CANCELED
                and int(cancelling["monotonic_ns"]) <= int(row["monotonic_ns"])
                <= int(blocked["monotonic_ns"])]
    if acknowledgement is None or len(canceling) != 1 or len(canceled) != 1:
        raise EvidenceAdjudicationError(
            "P4E6A8F1_CANCEL_EVIDENCE_NEEDS_REVIEW",
            {"acknowledgement": acknowledgement is not None,
             "canceling_count": len(canceling), "canceled_count": len(canceled)})
    if not (int(cancelling["monotonic_ns"]) <= int(canceling[0]["monotonic_ns"])
            <= int(acknowledgement["monotonic_ns"])
            <= int(canceled[0]["monotonic_ns"])
            <= int(blocked["monotonic_ns"])):
        raise EvidenceAdjudicationError(
            "P4E6A8F1_TERMINAL_ORDERING_NEEDS_REVIEW",
            "cancel submission/status/ack/result/BLOCKED chronology is invalid")
    return {"persistent_diagnostic_receipt_ns": int(persistent_ns),
            "history_search_start_ns": search_start_ns, "cancelling": cancelling,
            "canceling": canceling[0], "acknowledgement": acknowledgement,
            "canceled": canceled[0], "blocked": blocked}


def _xy_from_odom(row):
    """Return x/y from a frozen runner odometry tuple or mapping."""
    if isinstance(row, dict):
        return float(row["x"]), float(row["y"])
    return float(row[2]), float(row[3])


def _odom_receipt(row):
    return int(row["mono_ns"] if isinstance(row, dict) else row[0])


def _odom_zero_twist(row):
    if isinstance(row, dict):
        return abs(float(row["linear_x"])) <= 1e-6 and abs(float(row["angular_z"])) <= 1e-6
    return abs(float(row[5])) <= 1e-6 and abs(float(row[6])) <= 1e-6


def build_stationary_window(odom_rows, *, fake_applied_zero_ns, propagation_pose,
                            fixture_stop_ns=None, fixture_stop_request_ns=None,
                            qualified_stop_ns=None, first_safe_zero_ns=None,
                            first_vehicle_zero_ns=None):
    """Select the first zero-twist odometry at/after confirmed fake zero."""
    if fake_applied_zero_ns is None:
        raise EvidenceAdjudicationError(
            "P4E6A8B2_STATIONARY_FAKE_ZERO_MISSING_NEEDS_REVIEW",
            "confirmed fake-applied zero is required")
    stationary = next((row for row in odom_rows
                       if _odom_receipt(row) >= int(fake_applied_zero_ns)
                       and _odom_zero_twist(row)), None)
    if stationary is None:
        raise EvidenceAdjudicationError(
            "P4E6A8B2_STATIONARY_ODOM_MISSING_NEEDS_REVIEW",
            "zero-twist odometry at/after fake-applied zero is required")
    start_xy = _xy_from_odom(stationary)
    propagation_xy = _xy_from_odom(propagation_pose)
    return {
        "fixture_stop_ns": fixture_stop_ns,
        "fixture_stop_request_ns": fixture_stop_request_ns,
        "qualified_stop_ns": qualified_stop_ns,
        "first_safe_zero_ns": first_safe_zero_ns,
        "first_vehicle_zero_ns": first_vehicle_zero_ns,
        "first_fake_applied_zero_ns": int(fake_applied_zero_ns),
        "first_stationary_odom_ns": _odom_receipt(stationary),
        "stationary_start_pose": start_xy,
        "propagation_translation_m": math.hypot(
            start_xy[0] - propagation_xy[0], start_xy[1] - propagation_xy[1]),
    }


def adjudicate_stationary_checkpoint(window, checkpoint_odom,
                                     maximum_translation_m=MAXIMUM_STATIONARY_TRANSLATION_M):
    """Apply 0.02 m only after confirmed downstream zero."""
    end_xy = _xy_from_odom(checkpoint_odom)
    start_xy = window["stationary_start_pose"]
    translation = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    return {
        **window,
        "checkpoint_odom_ns": _odom_receipt(checkpoint_odom),
        "checkpoint_pose": end_xy,
        "post_downstream_zero_translation_m": translation,
        "maximum_stationary_translation_m": float(maximum_translation_m),
        "passes": translation <= float(maximum_translation_m),
    }


def checkpoint_freshness(evaluation_ns, *, safe_receipt_ns, odom_receipt_ns,
                         policy_receipt_ns, liveness_ns=PAIR_LIVENESS_NS):
    ages = {"safe": int(evaluation_ns) - int(safe_receipt_ns),
            "odom": int(evaluation_ns) - int(odom_receipt_ns),
            "policy": int(evaluation_ns) - int(policy_receipt_ns)}
    stale = [name for name, age in ages.items() if age >= int(liveness_ns)]
    return {"ages_ns": ages, "stale_sources": stale,
            "callback_servicing_unhealthy": len(stale) >= 2,
            "safe_fresh": ages["safe"] < int(liveness_ns)}


def require_checkpoint_freshness(result):
    if result["callback_servicing_unhealthy"]:
        raise EvidenceAdjudicationError(
            "P4E6A8B2_CALLBACK_SERVICING_UNHEALTHY_NEEDS_REVIEW", result)
    if not result["safe_fresh"]:
        raise EvidenceAdjudicationError(
            "P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW", result)
    return result


def verify_supervisor_source_authority(path=None, *, expected_sha256=FROZEN_SUPERVISOR_SHA256):
    """Fail closed unless source-event semantics belong to the accepted core."""
    if path is None:
        from parking_robot_mission_manager import mission_progress_supervisor_core as core
        path = inspect.getfile(core)
    source_path = Path(path)
    observed = hashlib.sha256(source_path.read_bytes()).hexdigest()
    if observed != expected_sha256:
        raise EvidenceAdjudicationError(
            "P4E6A8C2_SUPERVISOR_SOURCE_AUTHORITY_MISMATCH",
            {"path": str(source_path), "expected": expected_sha256, "observed": observed})
    return {"path": str(source_path), "expected_sha256": expected_sha256,
            "observed_sha256": observed, "matches": True}


def reconstruct_policy_timing(*, diagnostic=None, state=None, first_evaluation_ns=None):
    """Return observational estimates; never labels an estimate `_stop_started`."""
    result = {
        "authority": "OBSERVATIONAL_TIMING_METRIC",
        "first_stop_core_evaluation_estimate_ns": first_evaluation_ns,
        "diagnostic_receipt_latency_ns": None,
        "state_publication_estimate_ns": None,
        "state_receipt_latency_ns": None,
    }
    if diagnostic is not None:
        values = diagnostic.get("values", {})
        safe_receipt = values.get("safe_command_receipt_steady_sec")
        safe_age = values.get("safe_command_age_sec")
        if safe_receipt not in (None, "none") and safe_age not in (None, "none"):
            estimate = int(round((float(safe_receipt) + float(safe_age)) * 1e9))
            result["first_stop_core_evaluation_estimate_ns"] = estimate
            result["diagnostic_receipt_latency_ns"] = diagnostic["monotonic_ns"] - estimate
    if state is not None:
        required = ("message_ros_stamp_ns", "ros_ns", "monotonic_ns")
        if all(state.get(name) is not None for name in required):
            offset = int(state["ros_ns"]) - int(state["monotonic_ns"])
            publication = int(state["message_ros_stamp_ns"]) - offset
            result["state_publication_estimate_ns"] = publication
            result["state_receipt_latency_ns"] = int(state["monotonic_ns"]) - publication
    return result


def reproduce_receipt_anchor_defect(*, first_evaluation_ns, first_diagnostic_receipt_ns,
                                    event_publication_ns, event_receipt_ns,
                                    threshold_ns=TEMPORARY_STOP_NS):
    """Behavioral model of the rejected receipt-anchor rule."""
    receipt_delta = int(event_receipt_ns) - int(first_diagnostic_receipt_ns)
    source_interval = int(event_publication_ns) - int(first_evaluation_ns)
    return {"status": "P4E6A8C2_RECEIPT_ANCHOR_DEFECT_REPRODUCED",
            "receipt_delta_ns": receipt_delta, "source_interval_ns": source_interval,
            "old_logic_passes": receipt_delta >= int(threshold_ns),
            "source_consistent": source_interval >= int(threshold_ns),
            "differential_latency_ns": source_interval - receipt_delta}


@dataclass
class StopEpisodeLedger:
    """Explicit same-epoch causal STOP continuity and source-event authority."""

    episode_id: str
    waypoint_uuid: str
    command_epoch: str
    fixture_transition_id: str
    first_qualified_stop_ns: int
    latest_qualified_stop_ns: int | None = None
    pending_intervals: list[tuple[int, int | None]] = field(default_factory=list)
    breakers: list[dict] = field(default_factory=list)
    temporary_event_ns: int | None = None
    persistent_event_ns: int | None = None

    @property
    def valid(self):
        return not self.breakers

    def pending(self, start_ns, end_ns=None):
        if end_ns is not None and int(end_ns) - int(start_ns) >= PAIR_LIVENESS_NS:
            self.break_episode(end_ns, "PAIR_STALE")
        self.pending_intervals.append((int(start_ns), None if end_ns is None else int(end_ns)))

    def qualified_stop(self, now_ns, *, waypoint_uuid=None, command_epoch=None,
                       fixture_transition_id=None):
        mismatches = []
        for name, observed, expected in (
                ("waypoint_uuid", waypoint_uuid, self.waypoint_uuid),
                ("command_epoch", command_epoch, self.command_epoch),
                ("fixture_transition_id", fixture_transition_id, self.fixture_transition_id)):
            if observed is not None and observed != expected:
                mismatches.append((name, observed, expected))
        if mismatches:
            self.break_episode(now_ns, "GOAL_EPOCH_OR_FIXTURE_TRANSITION", mismatches)
        self.latest_qualified_stop_ns = int(now_ns)

    def break_episode(self, now_ns, reason, detail=None):
        self.breakers.append({"ns": int(now_ns), "reason": str(reason), "detail": detail})

    def snapshot(self):
        return {"episode_id": self.episode_id, "waypoint_uuid": self.waypoint_uuid,
                "command_epoch": self.command_epoch,
                "fixture_transition_id": self.fixture_transition_id,
                "first_qualified_stop_ns": self.first_qualified_stop_ns,
                "latest_qualified_stop_ns": self.latest_qualified_stop_ns,
                "pending_intervals": list(self.pending_intervals),
                "breakers": list(self.breakers), "valid": self.valid,
                "temporary_event_ns": self.temporary_event_ns,
                "persistent_event_ns": self.persistent_event_ns}


def adjudicate_source_event(*, supervisor_sha256, ledger, event, event_receipt_ns,
                            mission_state=None, first_evaluation_estimate_ns=None,
                            publication_estimate_ns=None, uncertainty_ns=1_000_000,
                            competing_failure=None):
    """Adjudicate private thresholds from frozen-source events and continuity."""
    if supervisor_sha256 != FROZEN_SUPERVISOR_SHA256:
        raise EvidenceAdjudicationError(
            "P4E6A8C2_SUPERVISOR_SOURCE_AUTHORITY_MISMATCH", supervisor_sha256)
    if competing_failure is not None:
        raise EvidenceAdjudicationError(
            "P4E6A8C2_STOP_EPISODE_LEDGER_NEEDS_REVIEW", competing_failure)
    if not ledger.valid:
        raise EvidenceAdjudicationError(
            "P4E6A8C2_STOP_EPISODE_LEDGER_NEEDS_REVIEW", ledger.snapshot())
    contracts = {
        "COLLISION_STOP_TEMPORARY": (TEMPORARY_STOP_NS, "TEMPORARILY_BLOCKED"),
        "PERSISTENT_COLLISION_STOP": (PERSISTENT_STOP_NS, "CANCELLING"),
    }
    if event not in contracts:
        raise EvidenceAdjudicationError(
            "P4E6A8C2_STOP_EPISODE_LEDGER_NEEDS_REVIEW", f"non-authoritative event {event}")
    threshold_ns, required_state = contracts[event]
    if mission_state is not None and mission_state != required_state:
        raise EvidenceAdjudicationError(
            "P4E6A8C2_STOP_EPISODE_LEDGER_NEEDS_REVIEW",
            {"event": event, "state": mission_state, "required": required_state})
    if first_evaluation_estimate_ns is not None and publication_estimate_ns is not None:
        latest_possible_publication = int(publication_estimate_ns) + int(uncertainty_ns)
        earliest_possible_start = int(first_evaluation_estimate_ns) - int(uncertainty_ns)
        if latest_possible_publication < earliest_possible_start + threshold_ns:
            raise EvidenceAdjudicationError(
                "P4E6A8C2_SOURCE_EVENT_RECONSTRUCTION_CONFLICT_NEEDS_REVIEW",
                {"event": event, "first_evaluation_estimate_ns": first_evaluation_estimate_ns,
                 "publication_estimate_ns": publication_estimate_ns,
                 "uncertainty_ns": uncertainty_ns, "threshold_ns": threshold_ns})
    if event == "COLLISION_STOP_TEMPORARY":
        ledger.temporary_event_ns = int(event_receipt_ns)
    else:
        if ledger.temporary_event_ns is None:
            raise EvidenceAdjudicationError(
                "P4E6A8C2_STOP_EPISODE_LEDGER_NEEDS_REVIEW", "persistent before temporary")
        ledger.persistent_event_ns = int(event_receipt_ns)
    return {"pass": True, "authority": "FROZEN_SOURCE_EVENT_PLUS_CONTINUOUS_CAUSAL_EPISODE",
            "event": event, "event_receipt_ns": int(event_receipt_ns),
            "private_threshold_sec_proven": threshold_ns / 1e9,
            "receipt_delta_threshold_applied": False,
            "first_evaluation_estimate_ns": first_evaluation_estimate_ns,
            "publication_estimate_ns": publication_estimate_ns,
            "ledger": ledger.snapshot()}


@dataclass
class PersistentStopAdjudicator:
    """ROS-free observer for the authorized A.8 evidence state machine."""

    phase: str = "PRE_READINESS"
    stable_clear_start_ns: int | None = None
    stable_clear_end_ns: int | None = None
    fixture_stop_request_ns: int | None = None
    first_movement_raw_under_stop_ns: int | None = None
    first_safe_zero_ns: int | None = None
    first_qualified_stop_ns: int | None = None
    stop_started_ns: int | None = None
    temporary_blocked_ns: int | None = None
    persistent_stop_ns: int | None = None
    cancel_request_ns: int | None = None
    cancel_ack_or_action_terminal_ns: int | None = None
    blocked_ns: int | None = None
    pending_started_ns: int | None = None
    fixture_requests: list[str] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    cancel_count: int = 0
    persistent_count: int = 0
    waypoint1_count: int = 0
    supervisor_sha256: str = FROZEN_SUPERVISOR_SHA256
    stop_episode: StopEpisodeLedger | None = None
    authority_results: list[dict] = field(default_factory=list)

    def _fail(self, reason, detail):
        raise EvidenceAdjudicationError(reason, detail)

    def readiness(self, *, query_ns, drain_start_ns, drain_end_ns, epoch_start_ns,
                  epoch_end_ns, epochs, health_ok):
        if not (query_ns <= drain_start_ns < drain_end_ns <= epoch_start_ns < epoch_end_ns):
            self._fail("P4E6A8A_READINESS_IMPLEMENTATION_NEEDS_REVIEW", "timestamp order")
        if drain_end_ns-drain_start_ns < int(CALLBACK_DRAIN_SEC*1e9):
            self._fail("P4E6A8A_READINESS_IMPLEMENTATION_NEEDS_REVIEW", "drain <1s")
        if epoch_end_ns-epoch_start_ns < int(FORMAL_READINESS_EPOCH_SEC*1e9):
            self._fail("P4E6A8A_READINESS_IMPLEMENTATION_NEEDS_REVIEW", "epoch <2s")
        if not health_ok or not epochs or not all(row.get("passes") for row in epochs.values()):
            self._fail("P4E6A8_SETTLED_READINESS_NEEDS_REVIEW", "formal epoch failed")
        self.phase = "MISSION_START"

    def activate(self):
        if self.phase != "MISSION_START":
            self._fail("P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "activation order")
        self.phase = "ARMED_ACTIVE"

    def observe_clear(self, now_ns, *, pair_state="VALID", semantic="CLEAR",
                      movement=True, health_ok=True, odom_progress=True):
        if self.phase not in ("ARMED_ACTIVE", "STABLE_CLEAR"):
            self._fail("P4E6A8A_OBSERVER_SEMANTICS_NEEDS_REVIEW", "CLEAR outside prerequisite")
        if not health_ok:
            self._fail("HEALTH_FAILURE_TERMINATION", "health invalid during CLEAR")
        if pair_state == "PENDING":
            if self.pending_started_ns is None:
                self.pending_started_ns = now_ns
                if self.stop_episode is not None:
                    self.stop_episode.pending(now_ns)
            if now_ns-self.pending_started_ns >= PAIR_LIVENESS_NS:
                self._fail("P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW", "CLEAR pending >=250ms")
            return
        if self.pending_started_ns is not None and self.stop_episode is not None:
            self.stop_episode.pending_intervals[-1] = (self.pending_started_ns, int(now_ns))
        self.pending_started_ns = None
        if pair_state in ("AMBIGUOUS", "STALE") or semantic != "CLEAR" or not movement or not odom_progress:
            self._fail("P4E6A8A_OBSERVER_SEMANTICS_NEEDS_REVIEW", "stable CLEAR contradicted")
        if self.stable_clear_start_ns is None:
            self.stable_clear_start_ns = now_ns
        self.stable_clear_end_ns = now_ns
        self.phase = "STABLE_CLEAR"

    def inject_stop(self, now_ns):
        if (self.phase != "STABLE_CLEAR" or self.stable_clear_start_ns is None
                or self.stable_clear_end_ns-self.stable_clear_start_ns < int(STABLE_CLEAR_SEC*1e9)):
            self._fail("P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "stable CLEAR <5s")
        if self.fixture_requests:
            self._fail("P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "duplicate fixture request")
        self.fixture_requests.append("STOP")
        self.fixture_stop_request_ns = now_ns
        self.phase = "STOP_INJECTED"

    def fixture_command(self, mode):
        if mode != "STOP" or self.fixture_requests:
            self._fail("P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "one-way fixture violation")
        self.fixture_requests.append(mode)

    def observe_stop(self, now_ns, *, pair_state="VALID", semantic="STOP", event="COLLISION_STOP_PENDING",
                     stop_age_sec=None, no_progress_age_sec=None, movement_raw=True, safe_zero=True,
                     health_ok=True, waypoint_uuid="waypoint-0", command_epoch="epoch-0",
                     fixture_transition_id="stop-1", first_evaluation_estimate_ns=None,
                     publication_estimate_ns=None, reconstruction_uncertainty_ns=1_000_000,
                     competing_failure=None):
        del no_progress_age_sec
        if self.phase not in ("STOP_INJECTED", "STOP_PENDING", "TEMPORARILY_BLOCKED", "PERSISTENCE_WAIT"):
            self._fail("P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "STOP observation order")
        if not health_ok:
            if self.stop_episode is not None:
                self.stop_episode.break_episode(now_ns, "HEALTH_FAILURE")
            self._fail("HEALTH_FAILURE_TERMINATION", "health invalid during STOP")
        if pair_state == "PENDING":
            if self.pending_started_ns is None:
                self.pending_started_ns = now_ns
            if now_ns-self.pending_started_ns >= PAIR_LIVENESS_NS:
                if self.stop_episode is not None:
                    self.stop_episode.break_episode(now_ns, "PAIR_STALE")
                self._fail("P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW", "STOP pending >=250ms")
            return
        self.pending_started_ns = None
        if pair_state == "AMBIGUOUS":
            if self.stop_episode is not None:
                self.stop_episode.break_episode(now_ns, "PAIR_AMBIGUOUS")
            self._fail("P4E6A8_OBSERVER_AMBIGUITY_NEEDS_REVIEW", "STOP continuity broken")
        if pair_state == "STALE" or event == "COMMAND_PAIR_STALE":
            if self.stop_episode is not None:
                self.stop_episode.break_episode(now_ns, "COMMAND_PAIR_STALE")
            self._fail("P4E6A8_COMMAND_PAIR_STALE_NEEDS_REVIEW", "stale command pair")
        if semantic != "STOP" or not movement_raw or not safe_zero:
            if self.stop_episode is not None:
                self.stop_episode.break_episode(now_ns, semantic)
            self._fail("P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW", f"contradictory class {semantic}")
        if self.first_movement_raw_under_stop_ns is None:
            self.first_movement_raw_under_stop_ns = now_ns
        if self.first_safe_zero_ns is None:
            self.first_safe_zero_ns = now_ns
        if self.first_qualified_stop_ns is None:
            self.first_qualified_stop_ns = now_ns
            self.stop_episode = StopEpisodeLedger(
                episode_id="collision-stop-1", waypoint_uuid=waypoint_uuid,
                command_epoch=command_epoch, fixture_transition_id=fixture_transition_id,
                first_qualified_stop_ns=now_ns, latest_qualified_stop_ns=now_ns)
        else:
            self.stop_episode.qualified_stop(
                now_ns, waypoint_uuid=waypoint_uuid, command_epoch=command_epoch,
                fixture_transition_id=fixture_transition_id)
        derived = None if stop_age_sec is None else now_ns-int(float(stop_age_sec)*1e9)
        if derived is not None:
            if self.stop_started_ns is None:
                self.stop_started_ns = derived
            elif abs(derived-self.stop_started_ns) > 2_000_000:
                self.stop_episode.break_episode(now_ns, "INCONSISTENT_EXPLICIT_STOP_AGE")
                self._fail("P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW", "inconsistent stop-specific anchor")
        age = None if self.stop_started_ns is None else now_ns-self.stop_started_ns
        if event == "COLLISION_STOP_TEMPORARY":
            authority = adjudicate_source_event(
                supervisor_sha256=self.supervisor_sha256, ledger=self.stop_episode,
                event=event, event_receipt_ns=now_ns, mission_state="TEMPORARILY_BLOCKED",
                first_evaluation_estimate_ns=(self.stop_started_ns
                    if first_evaluation_estimate_ns is None and stop_age_sec is not None
                    else first_evaluation_estimate_ns),
                publication_estimate_ns=(now_ns
                    if publication_estimate_ns is None and stop_age_sec is not None
                    else publication_estimate_ns),
                uncertainty_ns=(0 if stop_age_sec is not None
                    and publication_estimate_ns is None else reconstruction_uncertainty_ns),
                competing_failure=competing_failure)
            self.authority_results.append(authority)
            self.temporary_blocked_ns = now_ns
            self.phase = "TEMPORARILY_BLOCKED"
        elif event == "PERSISTENT_COLLISION_STOP":
            authority = adjudicate_source_event(
                supervisor_sha256=self.supervisor_sha256, ledger=self.stop_episode,
                event=event, event_receipt_ns=now_ns, mission_state="CANCELLING",
                first_evaluation_estimate_ns=(self.stop_started_ns
                    if first_evaluation_estimate_ns is None and stop_age_sec is not None
                    else first_evaluation_estimate_ns),
                publication_estimate_ns=(now_ns
                    if publication_estimate_ns is None and stop_age_sec is not None
                    else publication_estimate_ns),
                uncertainty_ns=(0 if stop_age_sec is not None
                    and publication_estimate_ns is None else reconstruction_uncertainty_ns),
                competing_failure=competing_failure)
            self.authority_results.append(authority)
            self.persistent_stop_ns = now_ns
            self.persistent_count += 1
            self.phase = "CANCELLING"
        else:
            self.phase = "STOP_PENDING" if self.temporary_blocked_ns is None else "PERSISTENCE_WAIT"
        self.events.append({"ns": now_ns, "state": pair_state, "semantic": semantic,
                            "event": event, "stop_age_ns": age,
                            "timing_authority": "FROZEN_SOURCE_EVENT" if event in (
                                "COLLISION_STOP_TEMPORARY", "PERSISTENT_COLLISION_STOP")
                                else "OBSERVATIONAL_TIMING_METRIC"})

    def nav2_terminal(self, now_ns, *, status, caused_by_persistent=False):
        if status in ("ABORTED", "FAILED") and not caused_by_persistent:
            self._fail("P4E6A8_NAV2_PROGRESS_CHECKER_RACE_NEEDS_REVIEW", status)
        if self.phase != "CANCELLING":
            self._fail("P4E6A8_WRONG_TERMINATION_REASON_NEEDS_REVIEW", "terminal before persistent")
        self.cancel_count += 1
        if self.cancel_count != 1:
            self._fail("P4E6A8_CANCEL_CARDINALITY_NEEDS_REVIEW", "cancel count")
        self.cancel_request_ns = now_ns
        self.cancel_ack_or_action_terminal_ns = now_ns

    def block(self, now_ns, *, reason="PERSISTENT_COLLISION_STOP", completed=0, waypoint1_count=0):
        if reason != "PERSISTENT_COLLISION_STOP":
            self._fail("P4E6A8_WRONG_TERMINATION_REASON_NEEDS_REVIEW", reason)
        if self.cancel_count != 1 or self.persistent_count != 1:
            self._fail("P4E6A8_CANCEL_CARDINALITY_NEEDS_REVIEW", "persistent/cancel cardinality")
        if completed != 0 or waypoint1_count != 0:
            self._fail("P4E6A8A_CANCEL_CARDINALITY_ADJUDICATION_NEEDS_REVIEW", "mission cardinality")
        self.blocked_ns = now_ns
        self.phase = "BLOCKED"

    def summarize(self):
        if self.phase != "BLOCKED":
            self._fail("P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "terminal not BLOCKED")
        self.phase = "COMPLETE"
        return {"pass": True, "phase": self.phase, "fixture_requests": self.fixture_requests,
                "cancel_count": self.cancel_count, "persistent_count": self.persistent_count,
                "stop_episode": None if self.stop_episode is None else self.stop_episode.snapshot(),
                "authority_results": self.authority_results,
                "timing_ns": {name: getattr(self, name) for name in (
                    "fixture_stop_request_ns", "first_movement_raw_under_stop_ns",
                    "first_safe_zero_ns", "first_qualified_stop_ns", "stop_started_ns",
                    "temporary_blocked_ns", "persistent_stop_ns", "cancel_request_ns",
                    "cancel_ack_or_action_terminal_ns", "blocked_ns")}}


def synthetic_complete_path():
    """Execute the entire A.8 adjudication without sleeping or ROS."""
    a = PersistentStopAdjudicator()
    good = {name: {"passes": True} for name in ("gate", "adapter", "odom", "tf")}
    a.readiness(query_ns=0, drain_start_ns=1, drain_end_ns=1_000_000_001,
                epoch_start_ns=1_000_000_002, epoch_end_ns=3_000_000_002,
                epochs=good, health_ok=True)
    a.activate()
    for ns in range(4_000_000_000, 9_100_000_000, 50_000_000):
        if ns % 200_000_000 == 50_000_000:
            a.observe_clear(ns, pair_state="PENDING")
        else:
            a.observe_clear(ns)
    a.inject_stop(9_100_000_000)
    a.observe_stop(9_150_000_000, stop_age_sec=0.0)
    a.observe_stop(10_100_000_000, pair_state="PENDING")
    a.observe_stop(10_170_000_000, event="COLLISION_STOP_TEMPORARY", stop_age_sec=1.02)
    a.observe_stop(29_100_000_000, pair_state="PENDING")
    a.observe_stop(29_180_000_000, event="PERSISTENT_COLLISION_STOP", stop_age_sec=20.03)
    a.nav2_terminal(29_200_000_000, status="CANCELED", caused_by_persistent=True)
    a.block(29_250_000_000)
    return a.summarize()


def synthetic_repaired_complete_path(output_dir, *, writer_initial_stall_sec=0.0,
                                     writer_capacity=EVIDENCE_QUEUE_CAPACITY):
    """Deterministic repaired checkpoint + persistent path with injected time."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    writer = BoundedEvidenceWriter(
        out, capacity=writer_capacity, initial_stall_sec=writer_initial_stall_sec)
    callback_receipts = []
    try:
        # Representative callbacks stay current in injected monotonic time even
        # while the independent durability worker is deliberately stalled.
        sequence = 0
        for now_ns in range(9_150_000_000, 29_200_000_001, 20_000_000):
            callback_receipts.append(now_ns)
            sequence += 1
            writer.enqueue("synthetic_callbacks.jsonl", {
                "kind": "safe" if sequence % 2 else "odom", "receipt_ns": now_ns})
        odom = [
            (9_150_000_000, 0, 5.733459885822728, -53.72723606259897, 0, .07, 0),
            (9_700_000_000, 0, 5.766191219122134, -53.72764660712554, 0, 0, 0),
            (10_000_000_000, 0, 5.766191219122134, -53.72764660712554, 0, 0, 0),
            (11_150_000_000, 0, 5.766191219122134, -53.72764660712554, 0, 0, 0),
        ]
        window = build_stationary_window(
            odom, fake_applied_zero_ns=9_700_000_000,
            propagation_pose=odom[0], fixture_stop_ns=9_100_000_000,
            first_safe_zero_ns=9_150_000_000, first_vehicle_zero_ns=9_160_000_000)
        checkpoint_result = adjudicate_stationary_checkpoint(window, odom[-1])
        if not checkpoint_result["passes"]:
            raise EvidenceAdjudicationError(
                "P4E6A8B2_STATIONARY_WINDOW_NEEDS_REVIEW", checkpoint_result)
        a = PersistentStopAdjudicator()
        good = {name: {"passes": True} for name in ("gate", "adapter", "odom", "tf")}
        a.readiness(query_ns=0, drain_start_ns=1, drain_end_ns=1_000_000_001,
                    epoch_start_ns=1_000_000_002, epoch_end_ns=3_000_000_002,
                    epochs=good, health_ok=True)
        a.activate()
        a.observe_clear(4_000_000_000)
        a.observe_clear(9_050_000_000)
        a.inject_stop(9_100_000_000)
        first_evaluation_ns = 9_150_000_000
        first_diagnostic_receipt_ns = 9_258_929_471
        a.observe_stop(first_diagnostic_receipt_ns)
        a.observe_stop(10_100_000_000, pair_state="PENDING")
        a.observe_stop(10_170_000_000, event="COLLISION_STOP_TEMPORARY",
                       first_evaluation_estimate_ns=first_evaluation_ns,
                       publication_estimate_ns=10_168_520_842)
        # Corrected SAFE_ZERO+2 checkpoint uses the current serviced sample.
        checkpoint_safe_age_ns = 11_150_000_000 - 11_140_000_000
        if checkpoint_safe_age_ns >= PAIR_LIVENESS_NS:
            raise EvidenceAdjudicationError(
                "P4E6A8B2_CALLBACK_SERVICING_UNHEALTHY_NEEDS_REVIEW",
                checkpoint_safe_age_ns)
        a.observe_stop(29_100_000_000, pair_state="PENDING")
        a.observe_stop(29_180_000_000, event="PERSISTENT_COLLISION_STOP",
                       first_evaluation_estimate_ns=first_evaluation_ns,
                       publication_estimate_ns=29_170_000_000)
        a.nav2_terminal(29_200_000_000, status="CANCELED", caused_by_persistent=True)
        a.block(29_250_000_000)
        summary = a.summarize()
        writer_final = writer.finalize(timeout_sec=10.0 + writer_initial_stall_sec)
        persisted = [json.loads(line) for line in
                     (out / "synthetic_callbacks.jsonl").read_text(encoding="utf-8").splitlines()]
        persisted_sequences = [row["evidence_sequence"] for row in persisted]
        if persisted_sequences != list(range(1, len(persisted_sequences) + 1)):
            raise EvidenceWriterError(
                "P4E6A8B2_EVIDENCE_SEQUENCE_MISSING_NEEDS_REVIEW",
                "synthetic sequence mismatch")
        return {**summary, "checkpoint": checkpoint_result,
                "checkpoint_safe_age_ns": checkpoint_safe_age_ns,
                "distorted_receipt_timing": {
                    "first_evaluation_ns": first_evaluation_ns,
                    "first_diagnostic_receipt_ns": first_diagnostic_receipt_ns,
                    "temporary_publication_ns": 10_168_520_842,
                    "temporary_receipt_ns": 10_170_000_000,
                    "temporary_receipt_delta_ns": 911_070_529,
                    "persistent_publication_ns": 29_170_000_000,
                    "persistent_receipt_ns": 29_180_000_000,
                    "persistent_receipt_delta_ns": 19_921_070_529,
                    "classification": "OBSERVATIONAL_TIMING_METRIC"},
                "callback_count": len(callback_receipts),
                "persisted_count": len(persisted), "writer": writer_final}
    finally:
        if writer.status()["worker_alive"]:
            try:
                writer.finalize(timeout_sec=10.0 + writer_initial_stall_sec)
            except EvidenceWriterError:
                pass


def write_deterministic_summary(path, result):
    """Exercise the same fail-closed JSON evidence boundary used after runtime."""
    if not isinstance(result, dict) or not result.get("pass") or result.get("phase") != "COMPLETE":
        raise EvidenceAdjudicationError(
            "P4E6A8A_COMPLETE_PATH_TEST_NEEDS_REVIEW", "refusing incomplete package")
    target = Path(path)
    target.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return json.loads(target.read_text(encoding="utf-8"))


def first_policy_event(node, event, after_ns):
    return next((x for x in node.mission_diagnostics
                 if x["monotonic_ns"] >= after_ns
                 and x["values"].get("progress_supervisor_event") == event), None)


def first_action_status(node, uuid, status, after_ns):
    return next((x for x in node.action_statuses
                 if x["monotonic_ns"] >= after_ns and x["goal_uuid"] == uuid
                 and x["status"] == status), None)


def gate_fields(node):
    row = node.diag_conditions.get("vehicle_cmd_safety/guarded_vehicle_cmd_gate")
    return {} if row is None else dict(row[1])


def _latest_at(rows, receipt_ns):
    return next((row for row in reversed(rows) if row[0] <= receipt_ns), None)


def record_episode_observations(node, start_index, origin_pose, uuid):
    """Persist A.7M.3 observations without treating bounded PENDING as failure."""
    next_index = start_index
    while next_index < len(node.mission_diagnostics):
        diagnostic = node.mission_diagnostics[next_index]
        next_index += 1
        values = diagnostic["values"]
        event = values.get("progress_supervisor_event")
        collision = values.get("collision_classification")
        pair_state = values.get("command_pair_state", "VALID")
        if event is None or collision is None:
            continue
        receipt_ns = diagnostic["monotonic_ns"]
        raw = _latest_at(node.samples["/cmd_vel_nav_raw"], receipt_ns)
        safe = _latest_at(node.samples["/cmd_vel_nav_safe"], receipt_ns)
        gate = gate_fields(node)
        state = next((row for row in reversed(node.states)
                      if row["monotonic_ns"] <= receipt_ns), node.states[-1])
        feedback = next((row for row in reversed(node.feedback)
                         if row["monotonic_ns"] <= receipt_ns
                         and row["goal_uuid"] == uuid), None)
        translation = math.hypot(node.latest_odom[0] - origin_pose[0],
                                 node.latest_odom[1] - origin_pose[1])
        item = node.emit("supervisor_stop_episode.jsonl", {
            "event": "P4E6A8_SUPERVISOR_STOP_EPISODE_OBSERVATION",
            "monotonic_ns": receipt_ns,
            "supervisor_event": event,
            "collision_classification": collision,
            "command_pair_state": pair_state,
            "stop_age_sec": values.get("stop_age_sec"),
            "no_progress_age_sec": values.get("no_progress_age_sec"),
            "stop_age_external_availability": "AVAILABLE" if "stop_age_sec" in values else "NOT_SERIALIZED",
            "raw_linear": None if raw is None else raw[2],
            "raw_angular": None if raw is None else raw[7],
            "safe_linear": None if safe is None else safe[2],
            "safe_angular": None if safe is None else safe[7],
            "raw_age_ns": None if raw is None else receipt_ns - raw[0],
            "safe_age_ns": None if safe is None else receipt_ns - safe[0],
            "raw_safe_skew_ns": None if raw is None or safe is None else abs(raw[0] - safe[0]),
            "collision_valid": node.permissions.get(TOPICS[2], (False, 0))[0],
            "localization_valid": node.permissions.get(TOPICS[3], (False, 0))[0],
            "controller_valid": node.permissions.get(TOPICS[4], (False, 0))[0],
            "gate_state": gate.get("state"),
            "gate_fault_latched": gate.get("fault_latched"),
            "number_of_recoveries": None if feedback is None else feedback["number_of_recoveries"],
            "distance_remaining": None if feedback is None else feedback["distance_remaining"],
            "mission_state": state["state_name"],
            "goal_uuid": state["active_goal_uuid"],
            "user_cancel_service_count": node.cancel_count,
            "odometry_translation_m": translation,
        })
        if pair_state == "PENDING":
            continue
        if pair_state == "AMBIGUOUS":
            raise RuntimeError(f"continuous STOP ambiguity: {item}")
        if pair_state == "STALE" or event == "COMMAND_PAIR_STALE":
            raise RuntimeError(f"continuous STOP stale: {item}")
        if collision != "STOP" or event not in (
                "COLLISION_STOP_PENDING", "COLLISION_STOP_TEMPORARY",
                "PERSISTENT_COLLISION_STOP"):
            raise RuntimeError(f"continuous STOP evidence broken: {item}")
        if (raw is None or not nz(raw[2:8]) or safe is None or nz(safe[2:8])
                or item["safe_age_ns"] > 250_000_000
                or not all((item["collision_valid"], item["localization_valid"],
                            item["controller_valid"]))
                or item["gate_state"] != "ARMED"
                or item["gate_fault_latched"] != "false"
                or item["goal_uuid"] != uuid):
            raise RuntimeError(f"continuous STOP qualifying evidence broken: {item}")
    return next_index


def checkpoint(node, label, reference_ns, stationary_window, uuid, *, target_ns=None):
    evaluation_ns = time.monotonic_ns()
    policy = node.mission_diagnostics[-1] if node.mission_diagnostics else None
    values = {} if policy is None else policy["values"]
    safe = node.samples["/cmd_vel_nav_safe"][-1]
    raw = node.samples["/cmd_vel_nav_raw"][-1]
    odom = node.samples["/Odometry"][-1]
    gate = gate_fields(node)
    stationary = adjudicate_stationary_checkpoint(stationary_window, odom)
    writer = node.evidence_writer.status()
    target = reference_ns if target_ns is None else int(target_ns)
    last_policy_ns = None if policy is None else policy["monotonic_ns"]
    freshness = checkpoint_freshness(
        evaluation_ns, safe_receipt_ns=safe[0], odom_receipt_ns=odom[0],
        policy_receipt_ns=last_policy_ns)
    item = node.emit("persistent_checkpoints.jsonl", {
        "event": "P4E6A_PERSISTENT_CHECKPOINT", "checkpoint": label,
        "reference_ns": reference_ns, "age_ns": evaluation_ns - reference_ns,
        "checkpoint_target_ns": target,
        "checkpoint_evaluation_ns": evaluation_ns,
        "target_overshoot_ns": max(0, evaluation_ns - target),
        "mission_state": node.states[-1]["state_name"],
        "reason_code": node.states[-1]["reason_code"],
        "supervisor_event": values.get("progress_supervisor_event"),
        "collision_classification": values.get("collision_classification"),
        "raw_nonzero": nz(raw[2:8]), "safe_zero": not nz(safe[2:8]),
        "last_safe_receipt_ns": safe[0],
        "latest_safe_age_ns": evaluation_ns - safe[0],
        "last_odom_receipt_ns": odom[0],
        "last_policy_receipt_ns": last_policy_ns,
        "callback_sources_stale": len(freshness["stale_sources"]),
        "callback_stale_source_names": freshness["stale_sources"],
        "writer_queue_depth": writer["queue_depth"],
        "writer_failure_reason": writer["failure_reason"],
        "gate_state": gate.get("state"), "gate_fault_latched": gate.get("fault_latched"),
        "collision_valid": node.permissions.get(TOPICS[2], (False, 0))[0],
        "localization_valid": node.permissions.get(TOPICS[3], (False, 0))[0],
        "controller_valid": node.permissions.get(TOPICS[4], (False, 0))[0],
        "activation_state": values.get("progress_policy_activation_state"),
        "recovery_delta": int(values.get("recovery_delta", "0")),
        "number_of_recoveries": values.get("number_of_recoveries"),
        "distance_remaining": values.get("distance_remaining"),
        "action_status": next((x["status"] for x in reversed(node.action_statuses)
                               if x["goal_uuid"] == uuid), None),
        "goal_uuid": node.states[-1]["active_goal_uuid"],
        "user_cancel_service_count": node.cancel_count,
        **stationary,
    })
    if writer["failure_reason"] is not None:
        raise EvidenceWriterError(writer["failure_reason"], writer["failure_detail"])
    require_checkpoint_freshness(freshness)
    if (item["mission_state"] != "TEMPORARILY_BLOCKED" or not item["raw_nonzero"]
            or not item["safe_zero"]
            or item["gate_state"] != "ARMED" or item["gate_fault_latched"] != "false"
            or not all((item["collision_valid"], item["localization_valid"],
                        item["controller_valid"])) or item["activation_state"] != "ACTIVE"
            or item["goal_uuid"] != uuid or item["user_cancel_service_count"] != 0
            or not item["passes"]):
        raise RuntimeError(f"persistent checkpoint failed: {item}")
    return item


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    ns = parser.parse_args(args)
    out = Path(ns.output_dir); out.mkdir(parents=True, exist_ok=True)
    metrics = {}; error = None; status = "PHASE4_P4E6A8_PERSISTENT_COLLISION_STOP_RUNTIME_NEEDS_REVIEW"
    source_authority = verify_supervisor_source_authority()
    rclpy.init(); node = PersistentEvidenceRunner(out)
    (out / "persistent_checkpoints.jsonl").write_text("", encoding="utf-8")
    (out / "supervisor_stop_episode.jsonl").write_text("", encoding="utf-8")
    try:
        (out / "process_environment.tsv").write_text(
            "key\tvalue\n" + "".join(f"{k}\t{v}\n" for k, v in sorted(os.environ.items())
                                      if k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")),
            encoding="utf-8")
        node.spin(5.0); node.graph(); lifecycle = node.lifecycle()
        if any(lifecycle.get(name, (0, ""))[1] != "active" for name in LIFECYCLE):
            raise RuntimeError(f"inactive lifecycle: {lifecycle}")
        names = set(node.get_node_names())
        forbidden = {name for name in names if any(token in name.lower() for token in
                    ("plan_nav", "rtab", "fast_lio", "pure_pursuit", "wheelchair_controller", "udp"))}
        if forbidden or sum(name == "mission_manager" for name in names) != 1:
            raise RuntimeError(f"runtime authority nodes={names}, forbidden={forbidden}")
        publishers = {topic: len(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
        if any(publishers[topic] != 1 for topic in TOPICS[1:]):
            raise RuntimeError(f"publisher authority: {publishers}")
        raw_nodes = {ep.node_name for ep in node.get_publishers_info_by_topic("/cmd_vel_nav_raw")}
        if not raw_nodes or not raw_nodes <= {"controller_server", "behavior_server"}:
            raise RuntimeError(f"raw authority: {raw_nodes}")
        if node.collision_stop_pub_timeout() != 30.0:
            raise RuntimeError("frozen stop_pub_timeout is not 30.0")
        gate = gate_fields(node)
        if gate.get("state") != "DISARMED" or gate.get("fault_latched") != "false":
            raise RuntimeError(f"initial gate state: {gate}")
        last_synchronous_query_complete_ns = time.monotonic_ns()
        callback_drain_start_ns = time.monotonic_ns()
        node.spin(CALLBACK_DRAIN_SEC)
        callback_drain_end_ns = time.monotonic_ns()
        if callback_drain_end_ns-callback_drain_start_ns < 1_000_000_000:
            raise RuntimeError("callback drain shorter than 1.0 s")
        formal_readiness_epoch_start_ns = time.monotonic_ns()
        ready_pose = node.latest_odom
        node.spin(FORMAL_READINESS_EPOCH_SEC)
        formal_readiness_epoch_end_ns = time.monotonic_ns()
        formal_rows = {
            "gate": [x[0] for x in node.samples["/vehicle_cmd_safe"]],
            "adapter": [x[0] for x in node.samples["/wheelchair_control_command_mock"]],
            "odom": [x[0] for x in node.samples["/Odometry"]],
            "tf": [x[0] for x in node.tf_samples["odom->base_footprint"]],
        }
        epochs = {
            name: prospective_epoch(rows, formal_readiness_epoch_start_ns,
                                    formal_readiness_epoch_end_ns, minimum_duration_sec=2.0,
                                    lower_hz=18.0 if name in ("gate", "adapter") else 48.0,
                                    upper_hz=22.0 if name in ("gate", "adapter") else 52.0,
                                    large_gap_sec=.10 if name in ("gate", "adapter") else .05)
            for name, rows in formal_rows.items()
        }
        ready_rates = {name: row["rate_hz"] for name, row in epochs.items()}
        adapter_epoch = [x for x in node.diagnostic_history["adapter"]
                         if formal_readiness_epoch_start_ns <= x["monotonic_ns"] <= formal_readiness_epoch_end_ns]
        gate_epoch = [x for x in node.diagnostic_history["gate"]
                      if formal_readiness_epoch_start_ns <= x["monotonic_ns"] <= formal_readiness_epoch_end_ns]
        permission_epoch = {topic: [x for x in node.samples[topic]
                                    if formal_readiness_epoch_start_ns <= x[0] <= formal_readiness_epoch_end_ns]
                            for topic in TOPICS[2:5]}
        formal_health = bool(adapter_epoch and gate_epoch
            and not any(x["level"] >= 1 for x in adapter_epoch)
            and all(x["values"].get("input_publisher_count") == "1"
                    and x["values"].get("output_publisher_count") == "1" for x in adapter_epoch)
            and all(x["values"].get("state") == "DISARMED"
                    and x["values"].get("fault_latched") == "false" for x in gate_epoch)
            and all(rows and all(x[2] for x in rows)
                    and formal_readiness_epoch_end_ns-rows[-1][0] <= 500_000_000
                    for rows in permission_epoch.values()))
        if not formal_health or not all(row["passes"] for row in epochs.values()):
            raise RuntimeError(f"formal readiness failed: health={formal_health}, epochs={epochs}")
        ready_start = formal_readiness_epoch_start_ns
        ready_end = formal_readiness_epoch_end_ns
        readiness = node.emit("scenario_events.jsonl", {
            "event": "P4E6A_CHAIN_READINESS_PASS", "rates_hz": ready_rates,
            "lifecycle": lifecycle, "publisher_counts": publishers,
            "raw_nodes": sorted(raw_nodes), "stop_pub_timeout_sec": 30.0,
            "last_synchronous_query_complete_ns": last_synchronous_query_complete_ns,
            "callback_drain_start_ns": callback_drain_start_ns,
            "callback_drain_end_ns": callback_drain_end_ns,
            "callback_drain_duration_sec": (callback_drain_end_ns-callback_drain_start_ns)/1e9,
            "formal_readiness_epoch_start_ns": formal_readiness_epoch_start_ns,
            "formal_readiness_epoch_end_ns": formal_readiness_epoch_end_ns,
            "formal_readiness_epoch_duration_sec":
                (formal_readiness_epoch_end_ns-formal_readiness_epoch_start_ns)/1e9,
            "cadence_epochs": epochs})

        node.publish_initial(); node.spin(.5); route = node.publish_route()
        node.wait_for(lambda: state_after(node, MissionState.RECEIVED, route["monotonic_ns"]),
                      5.0, "RECEIVED timeout")
        start_request, start_response = node.trigger("start")
        nav0 = node.wait_for(lambda: next((x for x in node.states
            if x["monotonic_ns"] >= start_request["monotonic_ns"]
            and x["state"] == MissionState.NAVIGATING and x["active_goal_uuid"]), None),
            10.0, "waypoint-0 NAVIGATING UUID timeout")
        uuid0 = nav0["active_goal_uuid"]; accepted_ns = nav0["monotonic_ns"]
        priming = node.wait_for(lambda: node.latest_policy("INITIAL_PRIMING", accepted_ns, uuid0),
                                1.0, "INITIAL_PRIMING timeout")
        interlock = node.wait_fresh_safe_disarmed_since(accepted_ns, timeout_sec=1.6)
        arm_request_ns = time.monotonic_ns(); arm_response = node.arm_gate()
        active = node.wait_for(lambda: node.latest_policy("ACTIVE", arm_request_ns, uuid0),
                               1.0, "ACTIVE timeout")
        active_ns = active["monotonic_ns"]
        activation_start_ns = int(float(active["values"]["progress_policy_activation_start_sec"]) * 1e9)
        if active_ns - activation_start_ns >= 2_000_000_000:
            raise RuntimeError("activation missed 2.0-second deadline")
        activation = node.emit("scenario_events.jsonl", {
            "event": "P4E6A_INITIAL_MISSION_ACTIVATION_PASS", "goal_uuid": uuid0,
            "activation_start_ns": activation_start_ns, "active_observation_ns": active_ns,
            "activation_delta_ns": active_ns - activation_start_ns})

        clear_start = time.monotonic_ns(); clear_pose = node.latest_odom; node.spin(STABLE_CLEAR_SEC)
        clear_end = time.monotonic_ns()
        clear_pairs = pair_commands(node.samples["/cmd_vel_nav_raw"],
                                    node.samples["/cmd_vel_nav_safe"], clear_start, clear_end)
        clear_policy = [x for x in node.mission_diagnostics
                        if clear_start <= x["monotonic_ns"] <= clear_end]
        clear_bad = [x for x in clear_policy
                     if x["values"].get("command_pair_state") in ("AMBIGUOUS", "STALE")
                     or x["values"].get("progress_supervisor_event") in
                        ("COMMAND_PAIR_STALE", "COLLISION_STOP_PENDING",
                         "COLLISION_STOP_TEMPORARY", "PERSISTENT_COLLISION_STOP")
                     or x["values"].get("collision_classification") in ("STOP", "SLOWDOWN")]
        if (node.states[-1]["state"] != MissionState.NAVIGATING or not health_ok(node, clear_end)
                or clear_end-clear_start < 5_000_000_000 or clear_bad
                or len([1 for raw, safe, _ in clear_pairs
                        if command_mag(raw) > .01 and command_mag(safe) > .01]) < 30):
            raise RuntimeError("stable CLEAR motion failed")
        stable_clear = node.emit("scenario_events.jsonl", {
            "event": "P4E6A_STABLE_CLEAR_MOTION_PASS", "start_ns": clear_start,
            "end_ns": clear_end, "duration_ns": clear_end - clear_start})

        stop = node.mode("STOP"); stop_confirmation_ns = stop["monotonic_ns"]
        qualifying_diag = node.wait_for(
            lambda: first_policy_event(node, "COLLISION_STOP_PENDING", stop_confirmation_ns),
            1.0, "supervisor qualifying STOP reference timeout")
        qualifying_ns = qualifying_diag["monotonic_ns"]
        episode_index = node.mission_diagnostics.index(qualifying_diag)
        runtime_episode = StopEpisodeLedger(
            episode_id="runtime-collision-stop-1", waypoint_uuid=uuid0,
            command_epoch=uuid0, fixture_transition_id=str(stop["request_ns"]),
            first_qualified_stop_ns=qualifying_ns,
            latest_qualified_stop_ns=qualifying_ns)
        stop_pose = node.latest_odom
        safe_zero = node.wait_for(lambda: next((x for x in node.samples["/cmd_vel_nav_safe"]
            if x[0] >= stop_confirmation_ns and not nz(x[2:8])), None), 1.0, "safe zero timeout")
        raw_intent = node.wait_for(lambda: next((x for x in node.samples["/cmd_vel_nav_raw"]
            if x[0] >= stop_confirmation_ns and nz(x[2:8])), None), 1.0, "raw intent timeout")
        vehicle_zero = node.wait_for(lambda: next((x for x in node.samples["/vehicle_cmd_safe"]
            if x[0] >= stop_confirmation_ns and not nz(x[2:8])), None), 1.0,
            "vehicle zero timeout")
        fake_zero = node.wait_for(lambda: next((x for x in node.diagnostic_history["fake_base"]
            if x["monotonic_ns"] >= safe_zero[0]
            and abs(float(x["values"].get("applied_linear_x", "nan"))) <= 1e-6
            and abs(float(x["values"].get("applied_angular_z", "nan"))) <= 1e-6), None),
            1.0, "confirmed fake-applied zero timeout")
        stationary_odom = node.wait_for(lambda: next((x for x in node.samples["/Odometry"]
            if x[0] >= fake_zero["monotonic_ns"] and _odom_zero_twist(x)), None), 1.0,
            "stationary odometry after fake-applied zero timeout")
        stationary_window = build_stationary_window(
            node.samples["/Odometry"], fake_applied_zero_ns=fake_zero["monotonic_ns"],
            propagation_pose=(0, 0, *stop_pose), fixture_stop_ns=stop_confirmation_ns,
            fixture_stop_request_ns=stop["request_ns"], qualified_stop_ns=qualifying_ns,
            first_safe_zero_ns=safe_zero[0], first_vehicle_zero_ns=vehicle_zero[0])
        if _odom_receipt(stationary_odom) != stationary_window["first_stationary_odom_ns"]:
            raise RuntimeError("stationary odometry anchor selection mismatch")
        first_stop_timing = reconstruct_policy_timing(diagnostic=qualifying_diag)
        qualifying = node.emit("scenario_events.jsonl", {
            "event": "P4E6A8_FIRST_DIRECT_STOP_PENDING_RECEIPT",
            "receipt_ns": qualifying_ns, "stop_started_ns": None,
            "stop_anchor_basis": "PRIVATE_NOT_SERIALIZED_SOURCE_EVENT_AUTHORITY",
            "timing_classification": "OBSERVATIONAL_TIMING_METRIC",
            "source_authority": source_authority,
            "timing_reconstruction": first_stop_timing,
            "diagnostic": qualifying_diag,
            "first_raw_intent_ns": raw_intent[0], "first_safe_zero_ns": safe_zero[0],
            "first_vehicle_zero_ns": vehicle_zero[0],
            "first_fake_applied_zero_ns": fake_zero["monotonic_ns"],
            "first_stationary_odom_ns": stationary_window["first_stationary_odom_ns"],
            "propagation_translation_m": stationary_window["propagation_translation_m"],
            "stop_confirmation_ns": stop_confirmation_ns})

        temp = node.wait_for(lambda: state_after(node, MissionState.TEMPORARILY_BLOCKED,
            qualifying_ns, "COLLISION_STOP_TEMPORARY"), 2.0, "TEMPORARILY_BLOCKED timeout")
        temp_ns = temp["monotonic_ns"]; temp_delta = temp_ns - qualifying_ns
        temporary_timing = reconstruct_policy_timing(
            state=temp,
            first_evaluation_ns=first_stop_timing["first_stop_core_evaluation_estimate_ns"])
        temporary_authority = adjudicate_source_event(
            supervisor_sha256=source_authority["observed_sha256"], ledger=runtime_episode,
            event="COLLISION_STOP_TEMPORARY", event_receipt_ns=temp_ns,
            mission_state="TEMPORARILY_BLOCKED",
            first_evaluation_estimate_ns=temporary_timing["first_stop_core_evaluation_estimate_ns"],
            publication_estimate_ns=temporary_timing["state_publication_estimate_ns"])
        if any(x["state"] in (MissionState.CANCELLING, MissionState.BLOCKED,
                              MissionState.FAILED, MissionState.CANCELLED)
               for x in node.states if x["monotonic_ns"] < temp_ns):
            raise RuntimeError("premature terminal before persistent horizon")
        temporary = node.emit("scenario_events.jsonl", {
            "event": "P4E6A_TEMPORARILY_BLOCKED_ENTRY_PASS", "reference_ns": qualifying_ns,
            "temporarily_blocked_ns": temp_ns, "entry_delta_ns": temp_delta,
            "receipt_delta_classification": "OBSERVATIONAL_TIMING_METRIC",
            "source_event_authority": temporary_authority,
            "timing_reconstruction": temporary_timing,
            "goal_uuid": uuid0, "cancel_count": node.cancel_count})

        checkpoints = []
        for seconds in CHECKPOINT_SECONDS:
            target = safe_zero[0] + int(seconds * 1_000_000_000)
            while time.monotonic_ns() < target:
                rclpy.spin_once(node, timeout_sec=.01)
                try:
                    episode_index = record_episode_observations(
                        node, episode_index, stop_pose, uuid0)
                except RuntimeError:
                    status = "P4E6A8_STOP_TIMER_RESET_NEEDS_REVIEW"
                    raise
                wrong = next((event for event in ("CONTROLLER_NO_PROGRESS",
                    "RECOVERY_EXHAUSTED_NO_PROGRESS", "HEALTH_FAILURE_TERMINATION")
                    if first_policy_event(node, event, qualifying_ns)), None)
                if wrong:
                    status = "P4E6A8_WRONG_TERMINATION_REASON_NEEDS_REVIEW"
                    raise RuntimeError(f"wrong terminal trigger: {wrong}")
                aborted = first_action_status(node, uuid0, GoalStatus.STATUS_ABORTED, qualifying_ns)
                if aborted:
                    status = "P4E6A8_NAV2_PROGRESS_CHECKER_RACE_NEEDS_REVIEW"
                    raise RuntimeError("NavigateToPose ABORTED before persistent cancellation")
            checkpoints.append(checkpoint(
                node, f"SAFE_ZERO+{seconds}s", safe_zero[0], stationary_window, uuid0,
                target_ns=target))

        persistent = node.wait_for(lambda: first_policy_event(
            node, "PERSISTENT_COLLISION_STOP", qualifying_ns), 1.0,
            "persistent event not observed after conservative 20-second lower bound")
        episode_index = record_episode_observations(node, episode_index, stop_pose, uuid0)
        persistent_ns = persistent["monotonic_ns"]; persistent_delta = persistent_ns - qualifying_ns
        safe_zero_delta = persistent_ns - safe_zero[0]
        stop_specific_delta = persistent_ns - qualifying_ns
        cancelling = node.wait_for_state_history(
            MissionState.CANCELLING, qualifying_ns, "PERSISTENT_COLLISION_STOP", .5,
            "CANCELLING timeout", mission_id=route["mission_id"],
            route_id=route["route_id"], waypoint_index=0, goal_uuid=uuid0)
        cancelling_ns = cancelling["monotonic_ns"]
        persistent_authority = adjudicate_source_event(
            supervisor_sha256=source_authority["observed_sha256"], ledger=runtime_episode,
            event="PERSISTENT_COLLISION_STOP", event_receipt_ns=persistent_ns,
            mission_state="CANCELLING")
        trigger = node.emit("scenario_events.jsonl", {
            "event": "P4E6A8_STOP_SPECIFIC_PERSISTENCE_TIMER_PASS",
            "first_safe_zero_ns": safe_zero[0], "first_stop_pending_receipt_ns": qualifying_ns,
            "stop_started_ns": None,
            "persistent_event_ns": persistent_ns,
            "stop_specific_age_ns": None,
            "source_event_authority": persistent_authority,
            "receipt_delta_classification": "OBSERVATIONAL_TIMING_METRIC",
            "first_safe_zero_to_persistent_receipt_ns": safe_zero_delta,
            "first_stop_pending_receipt_to_persistent_receipt_ns": persistent_delta,
            "supervisor_diagnostic": persistent})

        canceling_status = node.wait_for(lambda: first_action_status(
            node, uuid0, GoalStatus.STATUS_CANCELING, cancelling_ns), 2.0,
            "cancel response not accepted within 2.0 seconds")
        canceled_status = node.wait_for(lambda: first_action_status(
            node, uuid0, GoalStatus.STATUS_CANCELED, cancelling_ns), 5.0,
            "action CANCELED result timeout")
        blocked = node.wait_for(lambda: state_after(node, MissionState.BLOCKED,
            canceled_status["monotonic_ns"], "PERSISTENT_COLLISION_STOP"), .5,
            "BLOCKED terminal timeout")
        blocked_ns = blocked["monotonic_ns"]
        terminal_history = adjudicate_persistent_terminal_history(
            states=node.states, action_statuses=node.action_statuses,
            persistent_ns=persistent_ns, mission_id=route["mission_id"],
            route_id=route["route_id"], waypoint_index=0, goal_uuid=uuid0,
            history_start_ns=qualifying_ns)
        if blocked_ns - qualifying_ns > 27_000_000_000:
            status = "P4E6A8_PERSISTENT_STOP_TIMING_NEEDS_REVIEW"
            raise RuntimeError("qualifying STOP to BLOCKED exceeded 27 seconds")
        uuids = {x[1] for x in node.goal_uuids}
        if (blocked["waypoint_index"] != 0 or blocked["completed"] != 0 or uuids != {uuid0}
                or node.cancel_count != 0 or any(x["state"] in (MissionState.CANCELLED,
                    MissionState.PAUSED, MissionState.SUCCEEDED, MissionState.FAILED,
                    MissionState.HELP_REQUIRED) for x in node.states)):
            status = "P4E6A8_CANCEL_CARDINALITY_NEEDS_REVIEW"
            raise RuntimeError("final BLOCKED/one-goal/no-user-cancel contract failed")
        protocol = node.emit("scenario_events.jsonl", {
            "event": "P4E6A8_BLOCK_TERMINATION_PROTOCOL_PASS", "pending_intent": "BLOCK_TERMINATION",
            "block_reason": "PERSISTENT_COLLISION_STOP", "goal_uuid": uuid0,
            "cancelling_ns": cancelling_ns, "transport_cancel_submission_ns": cancelling_ns,
            "transport_submission_evidence": "source invariant correlated to sole CANCELLING episode",
            "cancel_response_ns": canceling_status["monotonic_ns"],
            "action_canceled_ns": canceled_status["monotonic_ns"], "blocked_ns": blocked_ns,
            "historical_transition_adjudication": "PASS",
            "history_cancelling_ns": terminal_history["cancelling"]["monotonic_ns"]})

        retained_start = time.monotonic_ns(); node.spin(.55); retained_end = time.monotonic_ns()
        safe_rows = [x for x in node.samples["/cmd_vel_nav_safe"] if retained_start <= x[0] <= retained_end]
        if (retained_end - retained_start < 500_000_000 or not safe_rows
                or any(nz(x[2:8]) for x in safe_rows) or not health_ok(node, retained_end)
                or node.states[-1]["state"] != MissionState.BLOCKED):
            raise RuntimeError("retained pre-cleanup STOP zero failed")
        first_safe_zero_ns = safe_zero[0]
        budget = node.emit("scenario_events.jsonl", {
            "event": "P4E6A8_30S_STOP_PUBLICATION_BUDGET_PASS",
            "qualifying_stop_to_blocked_ns": blocked_ns - qualifying_ns,
            "first_safe_zero_to_blocked_ns": blocked_ns - first_safe_zero_ns,
            "safe_zero_publication_duration_at_blocked_ns": blocked_ns - first_safe_zero_ns,
            "remaining_margin_to_30s_ns": 30_000_000_000 - (blocked_ns - first_safe_zero_ns),
            "retained_zero_start_ns": retained_start, "retained_zero_end_ns": retained_end})

        cleanup_disarm_request_ns = time.monotonic_ns(); cleanup_disarm = node.disarm_gate()
        node.spin(1.5); observation_end_ns = time.monotonic_ns()
        terminal_stationary = adjudicate_stationary_checkpoint(
            stationary_window, node.samples["/Odometry"][-1])
        translation = terminal_stationary["post_downstream_zero_translation_m"]
        if (node.states[-1]["state"] != MissionState.BLOCKED
                or not terminal_stationary["passes"] or len(uuids) != 1):
            raise RuntimeError("post-BLOCKED stationary observation failed")
        feedback = [x for x in node.feedback if x["goal_uuid"] == uuid0]
        recovery_values = [x["number_of_recoveries"] for x in feedback]
        baseline_recovery = recovery_values[0] if recovery_values else 0
        maximum_recovery = max(recovery_values, default=0)
        timing = {
            "mission_publish_ns": route["monotonic_ns"], "mission_start_request_ns": start_request["monotonic_ns"],
            "waypoint0_uuid_receipt_ns": accepted_ns, "activation_start_ns": activation_start_ns,
            "active_ns": active_ns, "stable_clear_start_ns": clear_start,
            "stop_request_ns": stop["request_ns"], "stop_confirmation_ns": stop_confirmation_ns,
            "qualifying_stop_ns": qualifying_ns, "temporarily_blocked_ns": temp_ns,
            "persistent_event_ns": persistent_ns, "cancelling_ns": cancelling_ns,
            "transport_cancel_submission_ns": cancelling_ns,
            "cancel_response_ns": canceling_status["monotonic_ns"],
            "action_canceled_ns": canceled_status["monotonic_ns"], "blocked_ns": blocked_ns,
            "first_post_blocked_retained_zero_ns": safe_rows[0][0],
            "cleanup_disarm_request_ns": cleanup_disarm_request_ns,
            "cleanup_disarm_response_ns": cleanup_disarm["monotonic_ns"],
            "stationary_observation_end_ns": observation_end_ns,
        }
        deltas = {
            "qualifying_stop_to_temporarily_blocked_ns": temp_ns - qualifying_ns,
            "qualifying_stop_to_persistent_event_ns": persistent_ns - qualifying_ns,
            "persistent_event_to_cancelling_ns": cancelling_ns - persistent_ns,
            "cancelling_to_cancel_submission_ns": 0,
            "cancel_submission_to_response_ns": canceling_status["monotonic_ns"] - cancelling_ns,
            "response_to_action_canceled_ns": canceled_status["monotonic_ns"] - canceling_status["monotonic_ns"],
            "action_canceled_to_blocked_ns": blocked_ns - canceled_status["monotonic_ns"],
            "qualifying_stop_to_blocked_ns": blocked_ns - qualifying_ns,
            "first_safe_zero_to_blocked_ns": blocked_ns - first_safe_zero_ns,
        }
        writer_final = node.evidence_writer.finalize()
        episode_rows = [json.loads(line) for line in
                        (out / "supervisor_stop_episode.jsonl").read_text(encoding="utf-8").splitlines()]
        status = "PHASE4_P4E6A8_PERSISTENT_COLLISION_STOP_RUNTIME_COMPLETED_NEEDS_REVIEW"
        metrics = {
            "pass": True, "status": status, "route_count": node.route_count,
            "start_count": node.start_count, "user_cancel_count": node.cancel_count,
            "arm_count": node.arm_request_count, "goal_uuid": uuid0,
            "goal_uuid_count": len(uuids), "final_action_status": "CANCELED",
            "final_mission_state": "BLOCKED", "final_block_reason": blocked["reason_code"],
            "final_waypoint_index": blocked["waypoint_index"],
            "completed_waypoint_count": blocked["completed"],
            "waypoint1_dispatched": False,
            "translation_m": translation,
            "post_downstream_zero_translation_m": translation,
            "propagation_translation_m": stationary_window["propagation_translation_m"],
            "terminal_stationary_window": terminal_stationary,
            "readiness": readiness, "route": route, "start_request": start_request,
            "start_response": start_response, "priming": priming, "interlock": interlock,
            "arm_response": arm_response, "activation": activation,
            "stable_clear": stable_clear, "qualifying_stop": qualifying,
            "temporary_block": temporary, "checkpoints": checkpoints,
            "persistent_trigger": trigger, "block_protocol": protocol, "budget": budget,
            "continuous_stop_episode": {
                "observation_count": len(episode_rows), "episode_break_count": 0,
                "qualified_stop_count": sum(row["command_pair_state"] == "VALID"
                    and row["collision_classification"] == "STOP" for row in episode_rows),
                "pending_count": sum(row["command_pair_state"] == "PENDING"
                    for row in episode_rows),
                "ambiguous_count": sum(row["command_pair_state"] == "AMBIGUOUS"
                    for row in episode_rows),
                "stale_count": sum(row["command_pair_state"] == "STALE"
                    for row in episode_rows),
                "stop_age_external_availability": "NOT_SERIALIZED",
            },
            "timing_ns": timing, "deltas_ns": deltas,
            "recovery": {"baseline": baseline_recovery, "maximum": maximum_recovery,
                         "maximum_delta": maximum_recovery - baseline_recovery,
                         "nav2_entered_recovery": maximum_recovery > baseline_recovery,
                         "feedback_count": len(feedback)},
            "activation_reset": "TERMINAL_ACTIVATION_RESET_SOURCE_AND_TEST_CORRELATED",
            "rates_hz": {topic: rates(node.samples[topic], accepted_ns, observation_end_ns)
                         for topic in TOPICS},
            "cleanup": {"post_terminal": True, "disarm": cleanup_disarm,
                        "fixture_mode_remained_stop": True,
                        "evidence_writer": writer_final},
            "exclusions": {
                "HEALTH_FAILURE_TERMINATION": "NOT TESTED",
                "LOCALIZATION_INVALID termination": "NOT TESTED",
                "CONTROLLER_INVALID termination": "NOT TESTED",
                "COLLISION_MONITOR_INVALID termination": "NOT TESTED",
                "GATE_FAULT termination": "NOT TESTED",
                "CONTROLLER_NO_PROGRESS as intended terminal": "NOT TESTED",
                "RECOVERY_EXHAUSTED_NO_PROGRESS as intended terminal": "NOT TESTED",
                "USER_CANCEL / USER_PAUSE": "NOT TESTED",
            },
        }
    except Exception as exc:
        error = repr(exc); metrics = {"pass": False, "status": status, "error": error}
    finally:
        try:
            writer_final = node.evidence_writer.finalize()
            metrics["evidence_writer_final"] = writer_final
        except Exception as writer_exc:
            error = error or repr(writer_exc)
            metrics = {"pass": False,
                       "status": "P4E6A8B2_EVIDENCE_WRITER_INTEGRITY_NEEDS_REVIEW",
                       "error": error, "writer_error": repr(writer_exc),
                       "evidence_writer": node.evidence_writer.status()}
        (out / "terminal_metrics.json").write_text(
            json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        with (out / "terminal_metrics.json").open("r+", encoding="utf-8") as handle:
            os.fsync(handle.fileno())
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(0 if error is None else 1)


if __name__ == "__main__":
    main()
