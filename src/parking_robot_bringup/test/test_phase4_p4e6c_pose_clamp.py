import json
import math
import statistics
from pathlib import Path

import pytest

from parking_robot_bringup.phase4_p4e6c_pose_clamp import validate_clamp_snapshot


def _snapshot(frame="odom", gap=0.1, count=31):
    start = 1_000_000_000
    events = []
    for i in range(count):
        events.append({"frame_id": frame,
                       "actual_publish_monotonic_ns": start + int(i * gap * 1e9),
                       "actual_interval_sec": None if i == 0 else gap})
    intervals = [gap] * (count - 1)
    elapsed = gap * (count - 1)
    return {"frame_id": frame, "events": events,
            "anchor": {"frame_id": frame},
            "effective_frequency_hz": (count - 1) / elapsed,
            "median_interval_sec": statistics.median(intervals)}


def test_persistent_stimulator_contract_accepts_actual_ten_hz():
    result = validate_clamp_snapshot(_snapshot())
    assert result["pass"] is True
    assert 9.0 <= result["effective_frequency_hz"] <= 11.0


def test_stimulator_rejects_map_frame():
    with pytest.raises(RuntimeError, match="FRAME_CONTRACT"):
        validate_clamp_snapshot(_snapshot(frame="map"))


def test_stimulator_rejects_attempt1_cadence_shape():
    root = Path("/home/dog/phase4_reports/P4E6C_CP01_C3M_CP01_ATTEMPT1_011")
    rows = [json.loads(line) for line in (root / "pose_clamp_events.jsonl").read_text().splitlines() if line]
    timestamps = [int(row["actual_publish_monotonic_ns"]) for row in rows]
    intervals = [(b - a) / 1e9 for a, b in zip(timestamps, timestamps[1:])]
    assert len(rows) == 95
    assert statistics.mean(intervals) > 3.0
    assert max(intervals) > 0.2


def test_stimulator_rejects_slow_publication_gap():
    with pytest.raises(RuntimeError, match="CADENCE|GAP"):
        validate_clamp_snapshot(_snapshot(gap=0.25))
