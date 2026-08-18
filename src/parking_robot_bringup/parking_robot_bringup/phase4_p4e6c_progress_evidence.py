"""Publication-path-aware P4-E.6C evidence helpers.

Textual ``none`` is unavailable data, never numeric zero.  Branch selection is
performed before numeric conversion so C01-style liveness rows remain valid.
"""
from __future__ import annotations

import json
from pathlib import Path


def optional_float(value):
    if value is None or (isinstance(value, str) and value.strip().lower() in {"", "none", "n/a", "not_available"}):
        return None
    return float(value)


def command_pair_branch(values: dict) -> dict:
    state = str(values.get("command_pair_state", "")).upper()
    raw = optional_float(values.get("raw_command_age_sec"))
    safe = optional_float(values.get("safe_command_age_sec"))
    skew = optional_float(values.get("raw_safe_skew_sec"))
    liveness = optional_float(values.get("pair_liveness_age_sec"))
    if state in {"PENDING", "STALE"} and liveness is not None:
        return {"branch": "NON_SLIDING_PAIR_LIVENESS", "pair_liveness_age_sec": liveness,
                "raw_command_age_sec": raw, "safe_command_age_sec": safe, "raw_safe_skew_sec": skew}
    return {"branch": "ESTABLISHED_STREAM_AGE_OR_SKEW", "raw_command_age_sec": raw,
            "safe_command_age_sec": safe, "raw_safe_skew_sec": skew,
            "pair_liveness_age_sec": liveness}


def command_pair_ready(values: dict) -> dict:
    """Adjudicate source command-pair readiness without collapsing PENDING."""
    phase = str(values.get("command_stream_phase", "")).upper()
    state = str(values.get("command_pair_state", "")).upper()
    reason = str(values.get("reason_code", values.get("progress_supervisor_event", ""))).upper()
    raw = optional_float(values.get("raw_command_age_sec"))
    safe = optional_float(values.get("safe_command_age_sec"))
    liveness = optional_float(values.get("pair_liveness_age_sec"))
    skew = optional_float(values.get("raw_safe_skew_sec"))
    result = {"command_pair_branch": None, "command_pair_current": False,
              "rejection_reason": None, "pair_liveness_age_sec": liveness,
              "raw_command_age_sec": raw, "safe_command_age_sec": safe,
              "raw_safe_skew_sec": skew}
    if phase != "STREAM_ESTABLISHED":
        result["rejection_reason"] = "COMMAND_STREAM_NOT_ESTABLISHED"
        return result
    if state == "VALID":
        result["command_pair_branch"] = "VALID"
        if raw is None or raw >= .250:
            result["rejection_reason"] = "RAW_COMMAND_NOT_CURRENT"
        elif safe is None or safe >= .250:
            result["rejection_reason"] = "SAFE_COMMAND_NOT_CURRENT"
        elif skew is not None and skew > .075:
            result["rejection_reason"] = "COMMAND_PAIR_VALID_NOT_CURRENT"
        else:
            result["command_pair_current"] = True
        return result
    if state == "PENDING":
        result["command_pair_branch"] = "ESTABLISHED_PENDING"
        if not values.get("pre_arm_causal_proof", False):
            result["rejection_reason"] = "COMMAND_PAIR_PENDING_LIVENESS_UNAVAILABLE"
        elif liveness is None:
            result["rejection_reason"] = "COMMAND_PAIR_PENDING_LIVENESS_UNAVAILABLE"
        elif liveness >= .250:
            result["rejection_reason"] = "COMMAND_PAIR_PENDING_LIVENESS_EXPIRED"
        elif raw is None or raw >= .250:
            result["rejection_reason"] = "RAW_COMMAND_NOT_CURRENT"
        elif safe is None or safe >= .250:
            result["rejection_reason"] = "SAFE_COMMAND_NOT_CURRENT"
        elif reason == "COMMAND_PAIR_STALE":
            result["rejection_reason"] = "COMMAND_PAIR_STALE"
        else:
            result["command_pair_current"] = True
        return result
    result["rejection_reason"] = "COMMAND_PAIR_UNKNOWN_STATE"
    return result


def validate_progress_event(row: dict, expected_reason: str) -> dict:
    if row.get("progress_supervisor_event") != expected_reason:
        raise RuntimeError("P4E6C_PROGRESS_REASON_FAILURE")
    result = dict(row)
    if "no_progress_age_sec" in result:
        result["no_progress_age_sec"] = optional_float(result["no_progress_age_sec"])
    if "recovery_delta" in result:
        result["recovery_delta"] = int(result["recovery_delta"])
    if "command_pair_state" in result:
        result["command_pair"] = command_pair_branch(result)
    return result


def load_progress_jsonl(path: Path, expected_reason: str) -> dict:
    rows = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]
    matching = [validate_progress_event(row.get("values", row), expected_reason) for row in rows
                if row.get("progress_supervisor_event", row.get("values", {}).get("progress_supervisor_event")) == expected_reason]
    if not matching:
        raise RuntimeError("P4E6C_PROGRESS_EVENT_MISSING")
    return matching[0]


def validate_block_terminal(rows: dict, expected_reason: str) -> dict:
    """Validate BLOCK-family cardinality and identity without health ACKs."""
    if rows.get("originating_reason") != expected_reason:
        raise RuntimeError("P4E6C_BLOCK_REASON_FAILURE")
    if rows.get("block_cancel_ack") != "BLOCK_CANCEL_ACK_ACCEPTED":
        raise RuntimeError("P4E6C_BLOCK_ACK_FAILURE")
    if int(rows.get("cancel_request_count", 0)) != 1:
        raise RuntimeError("P4E6C_BLOCK_CANCEL_REQUEST_FAILURE")
    if int(rows.get("canceling_count", 0)) != 1 or int(rows.get("canceled_count", 0)) != 1:
        raise RuntimeError("P4E6C_BLOCK_ACTION_CARDINALITY_FAILURE")
    if int(rows.get("duplicate_cancel", 1)) != 0:
        raise RuntimeError("P4E6C_BLOCK_DUPLICATE_CANCEL")
    if rows.get("terminal_state") != "BLOCKED":
        raise RuntimeError("P4E6C_BLOCK_TERMINAL_STATE_FAILURE")
    for key in ("mission_id", "route_id", "active_goal_uuid"):
        if not rows.get(key):
            raise RuntimeError("P4E6C_BLOCK_IDENTITY_FAILURE")
    return {"pass": True, "reason": expected_reason, "ack": "BLOCK_CANCEL_ACK_ACCEPTED"}
