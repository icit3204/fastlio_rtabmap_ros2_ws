"""Pure, passive eligibility reconstruction; never emits a runner READY event."""
import json
from pathlib import Path
from dataclasses import dataclass

def _rows(path):
    rows=[]; lines=Path(path).read_text().splitlines()
    for n,line in enumerate(lines,1):
        try: rows.append(json.loads(line))
        except json.JSONDecodeError:
            if n==len(lines): break
            raise ValueError(f'corrupt JSONL line {n}')
    return rows

def witness_samples(directory):
    """Direct consumption of witness JSONL files; no hand-written translation."""
    d=Path(directory); graphs=_rows(d/'graph_observations.jsonl'); bools=_rows(d/'bool_observations.jsonl'); diags=_rows(d/'diagnostic_observations.jsonl'); samples=[]
    for diag in diags:
        now=diag['receipt_monotonic_ns']; g=next((x for x in reversed(graphs) if x['observation_monotonic_ns']<=now),{}); b=next((x for x in reversed(bools) if x['receipt_monotonic_ns']<=now),{})
        samples.append({'monotonic_ns':now,'publisher_count':g.get('canonical_bool_publisher_count'),'publisher_node':g.get('canonical_bool_node_identity'),'publisher_gid':g.get('canonical_bool_gid'),'diagnostic_gid':diag.get('diagnostic_graph_gid'),'post_epoch_bool_count':b.get('post_epoch_bool_count',0),'bool_value':b.get('value'),'bool_age_ns':now-b.get('receipt_monotonic_ns',0),'diagnostic_state':diag.get('state'),'diagnostic_reason':diag.get('reason_code'),'healthy_stable_sec':diag.get('healthy_stable_sec'),'diagnostic_age_ns':0,'diagnostic_message_age_ns':diag['receipt_ros_ns']-diag['header_ros_ns'],'diagnostic_header_ros_ns':diag['header_ros_ns'],'epoch_ros_ns':diag.get('epoch_start_ros_ns') or 0,'source_age_upper_bound_sec':(diag.get('source_age_sec') or 99)+(diag['receipt_ros_ns']-diag['header_ros_ns'])/1e9})
    return samples

@dataclass(frozen=True)
class PassiveObservationWindow:
    """The sole temporal authority for passive transaction analysis."""
    start_ns: int
    end_ns: int

    @classmethod
    def from_helper_enter(cls, helper_enter_ns, bound_sec=20.0):
        if isinstance(helper_enter_ns, bool) or not isinstance(helper_enter_ns, int) or helper_enter_ns < 0:
            raise ValueError("helper_enter_ns must be a non-negative monotonic integer")
        if isinstance(bound_sec, bool) or not isinstance(bound_sec, (int, float)) or bound_sec < 0:
            raise ValueError("bound_sec must be non-negative")
        return cls(helper_enter_ns, helper_enter_ns + int(bound_sec * 1e9))

def _validate_samples(samples, window):
    previous = None
    for row in samples:
        if not isinstance(row, dict) or isinstance(row.get("monotonic_ns"), bool) or not isinstance(row.get("monotonic_ns"), int):
            raise ValueError("witness observation has invalid monotonic timestamp")
        if previous is not None and row["monotonic_ns"] < previous:
            raise ValueError("witness observations are not monotonic")
        previous = row["monotonic_ns"]
    if not samples:
        raise ValueError("no witness observations")

def _eligible(row):
    return (row.get("publisher_count") == 1 and row.get("publisher_node") == "collision_monitor_validity_monitor"
            and bool(row.get("publisher_gid")) and bool(row.get("diagnostic_gid"))
            and row.get("post_epoch_bool_count", 0) >= 2 and row.get("bool_value") is True
            and row.get("bool_age_ns", 10**99) < 250_000_000
            and row.get("diagnostic_state") == "VALID" and row.get("diagnostic_reason") == "VALID"
            and row.get("healthy_stable_sec", 0) >= 1.0
            and row.get("diagnostic_age_ns", 10**99) < 250_000_000
            and row.get("diagnostic_message_age_ns", 10**99) < 250_000_000
            and row.get("diagnostic_header_ros_ns", 0) >= row.get("epoch_ros_ns", 1)
            and row.get("source_age_upper_bound_sec", 10**99) < .5)

def analyze_passive_eligibility(samples, helper_enter_ns, helper_outcome=None, bound_sec=20.0):
    """Analyze only durable observations in the inclusive helper-enter window."""
    window = PassiveObservationWindow.from_helper_enter(helper_enter_ns, bound_sec)
    _validate_samples(samples, window)
    selected = next((row for row in samples if window.start_ns <= row["monotonic_ns"] <= window.end_ns and _eligible(row)), None)
    result = {
        "analysis_window_start_ns": window.start_ns,
        "analysis_window_end_ns": window.end_ns,
        "first_eligible_at_or_after_helper_enter_ns": selected["monotonic_ns"] if selected else None,
        "helper_enter_to_first_eligible_sec": ((selected["monotonic_ns"] - window.start_ns) / 1e9) if selected else None,
    }
    if selected is None:
        result.update({"event": "NONE_BY_BOUND", "monotonic_ns": None})
        result["eligibility_class"] = "NONE_BY_20S"
    else:
        result.update({"event": "PASSIVE_READY_ELIGIBLE", "monotonic_ns": selected["monotonic_ns"]})
        elapsed = selected["monotonic_ns"] - window.start_ns
        if helper_outcome == "TIMEOUT":
            result["eligibility_class"] = "ELIGIBLE_BEFORE_8S" if elapsed < 8_000_000_000 else "ELIGIBLE_AT_OR_AFTER_8S"
        else:
            result["eligibility_class"] = "READY_WITHIN_8S" if elapsed < 8_000_000_000 else "READY_AFTER_8S"
    return result

def first_passive_ready_eligible(samples, helper_enter_ns, bound_sec=20.0):
    """Backward-compatible facade returning the corrected result schema."""
    return analyze_passive_eligibility(samples, helper_enter_ns, bound_sec=bound_sec)
