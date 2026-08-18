"""Qualification-only terminal closure witness/adjudication helpers.

This module has no ROS publishers, service clients, action clients, or health
control.  It deliberately supplements the historical health core; it does not
replace or rewrite historical adjudications.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping, Sequence


POST_FAILED_DRAIN_SEC = 0.250


def _owned(row: Mapping[str, Any], mission_id: str, route_id: str, waypoint: int,
          uuid: str) -> bool:
    return (row.get("mission_id") == mission_id and row.get("route_id") == route_id
            and int(row.get("waypoint_index", -1)) == waypoint
            and row.get("active_goal_uuid", uuid) in (uuid, ""))


def _statuses(rows: Iterable[Mapping[str, Any]], uuid: str, name: str) -> list[Mapping[str, Any]]:
    return [r for r in rows if r.get("goal_uuid") == uuid and r.get("status_name") == name]


def adjudicate_terminal_v2(*, origin_ns: int, reason: str, mission_id: str,
                           route_id: str, waypoint: int, uuid: str,
                           states: Sequence[Mapping[str, Any]],
                           statuses: Sequence[Mapping[str, Any]],
                           drain_start_ns: int | None = None,
                           drain_end_ns: int | None = None,
                           source_result_canceled: bool = False) -> dict[str, Any]:
    """Adjudicate identity/cardinality while allowing cross-topic receipt reorder.

    ``source_result_canceled`` is only a source-semantic corroborator.  It never
    manufactures a missing action-status row: exactly one matching CANCELED row
    is still required for PASS.
    """
    if not uuid:
        return {"pass": False, "classification": "MISSING_ACTIVE_UUID"}
    states = list(states)
    statuses = list(statuses)
    owned_state = [r for r in states if _owned(r, mission_id, route_id, waypoint, uuid)]
    cancels = [r for r in owned_state if int(r.get("monotonic_ns", -1)) >= origin_ns
               and r.get("state_name") == "CANCELLING"
               and r.get("reason_code") == reason
               and r.get("active_goal_uuid") == uuid]
    failed = [r for r in owned_state if r.get("state_name") == "FAILED"
              and r.get("reason_code") == reason]
    ack = [r for r in owned_state if r.get("state_name") == "CANCELLING"
           and r.get("reason_code") == "HEALTH_CANCEL_ACK_ACCEPTED"]
    canceling = _statuses(statuses, uuid, "CANCELING")
    canceled = _statuses(statuses, uuid, "CANCELED")
    duplicate_cancel = len(canceling) != 1
    result: dict[str, Any] = {
        "pass": False,
        "classification": "INSUFFICIENT_TERMINAL_EVIDENCE",
        "reason": reason,
        "uuid": uuid,
        "cancel_count": len(canceling),
        "canceled_count": len(canceled),
        "health_cancel_ack_count": len(ack),
        "duplicate_cancel": duplicate_cancel,
        "receipt_order_rule": "cross_topic_receipt_inversion_allowed",
        "source_result_canceled": bool(source_result_canceled),
        "post_failed_drain_sec": (None if drain_start_ns is None or drain_end_ns is None
                                   else (drain_end_ns - drain_start_ns) / 1e9),
    }
    if len(cancels) != 1 or len(failed) != 1 or len(ack) != 1:
        result["classification"] = "MISSING_IDENTITY_QUALIFIED_STATE"
        return result
    if len(canceling) != 1 or len(canceled) != 1:
        result["classification"] = "CANCEL_CARDINALITY_OR_CALLBACK_MISSING"
        return result
    c, f, a, cg, cd = cancels[0], failed[0], ack[0], canceling[0], canceled[0]
    result["events"] = {k: int(v.get("monotonic_ns", -1)) for k, v in {
        "origin": {"monotonic_ns": origin_ns}, "cancelling": c,
        "ack": a, "canceling": cg, "canceled": cd, "failed": f}.items()}
    if not (int(a["monotonic_ns"]) >= int(c["monotonic_ns"]) and
            int(f["monotonic_ns"]) >= int(c["monotonic_ns"])):
        result["classification"] = "INVALID_SOURCE_EVENT_ORDER"
        return result
    if drain_start_ns is not None and drain_end_ns is not None and drain_end_ns < drain_start_ns:
        result["classification"] = "INVALID_DRAIN_WINDOW"
        return result
    result.update({"pass": True, "classification": "PASS",
                   "causal_compatibility": "identity_and_source_semantics",
                   "receipt_order_observed": int(cd["monotonic_ns"]) > int(f["monotonic_ns"])})
    return result


def physical_closure(*, translation_m: float, rotation_rad: float,
                     translation_limit_m: float = 0.02,
                     rotation_limit_rad: float = 0.2) -> dict[str, Any]:
    """Pure Phase-4 physical decision; rotation is measured/report-only."""
    if translation_m is None or rotation_rad is None:
        return {"pass": False, "classification": "NOT_ADJUDICATED"}
    translation = float(translation_m)
    rotation = float(rotation_rad)
    if not math.isfinite(translation) or not math.isfinite(rotation):
        return {"pass": False, "classification": "NONFINITE_PHYSICAL_EVIDENCE",
                "translation_m": translation, "rotation_rad": rotation}
    ok = translation <= translation_limit_m
    return {"pass": ok, "classification": "PASS" if ok else "PHYSICAL_MOTION_EXCEEDED",
            "translation_m": translation, "rotation_rad": rotation,
            "rotation_acceptance": "REPORT_ONLY_NO_NUMERIC_PHASE4_BOUND"}


def physical_closure_from_rows(*, vehicle_rows: Sequence[Mapping[str, Any]],
                               applied_rows: Sequence[Mapping[str, Any]],
                               odometry_rows: Sequence[Mapping[str, Any]],
                               terminal_ns: int, drain_end_ns: int) -> dict[str, Any]:
    """Adjudicate the witness's collected canonical/applied/pose rows.

    Missing any required stream is explicitly not adjudicated.  This helper is
    qualification-only and does not create or infer command authority.
    """
    if not vehicle_rows or not applied_rows or not odometry_rows:
        return {"pass": False, "classification": "NOT_ADJUDICATED",
                "missing": [name for name, rows in (("vehicle", vehicle_rows),
                    ("applied", applied_rows), ("odometry", odometry_rows)) if not rows]}
    def zero(row):
        return all(abs(float(row.get(k, 0.0))) == 0.0 for k in ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"))
    first_vehicle_zero = next((r for r in vehicle_rows if int(r["monotonic_ns"]) >= terminal_ns and zero(r)), None)
    first_applied_zero = next((r for r in applied_rows if int(r["monotonic_ns"]) >= terminal_ns and zero(r)), None)
    if first_vehicle_zero is None or first_applied_zero is None:
        return {"pass": False, "classification": "MISSING_CANONICAL_OR_APPLIED_ZERO",
                "first_vehicle_zero_ns": None if first_vehicle_zero is None else first_vehicle_zero["monotonic_ns"],
                "first_applied_zero_ns": None if first_applied_zero is None else first_applied_zero["monotonic_ns"]}
    anchor = max(int(first_vehicle_zero["monotonic_ns"]), int(first_applied_zero["monotonic_ns"]))
    window = [r for r in odometry_rows if anchor <= int(r["monotonic_ns"]) <= drain_end_ns]
    if not window:
        return {"pass": False, "classification": "NOT_ADJUDICATED", "missing": ["stationary_pose_window"]}
    origin, final = window[0], window[-1]
    translation = math.hypot(float(final["x"]) - float(origin["x"]), float(final["y"]) - float(origin["y"]))
    rotation = float(final.get("yaw", 0.0)) - float(origin.get("yaw", 0.0))
    result = physical_closure(translation_m=translation, rotation_rad=rotation)
    result.update({"anchor_ns": anchor, "window_start_ns": int(origin["monotonic_ns"]),
                   "window_end_ns": int(final["monotonic_ns"]),
                   "first_vehicle_zero_ns": int(first_vehicle_zero["monotonic_ns"]),
                   "first_applied_zero_ns": int(first_applied_zero["monotonic_ns"])})
    return result


@dataclass(frozen=True)
class WitnessContract:
    publishers: tuple[str, ...] = ()
    services: tuple[str, ...] = ()
    action_clients: tuple[str, ...] = ()
    application_control_authority: bool = False
    post_failed_drain_sec: float = POST_FAILED_DRAIN_SEC


def controller_lifetime_decision(*, runner_exit_code: int | None,
                                 failed_seen: bool, drain_complete: bool) -> str:
    """The runner may exit nonzero; witness closure remains authoritative."""
    if not failed_seen:
        return "RUNNER_FAILURE_BEFORE_TERMINAL"
    if not drain_complete:
        return "WITNESS_DRAIN_INCOMPLETE"
    return "CONTINUE_WITNESS_THEN_TEARDOWN"
