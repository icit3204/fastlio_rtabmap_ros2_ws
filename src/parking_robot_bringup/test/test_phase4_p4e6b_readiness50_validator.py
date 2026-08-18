"""F3A schema tests for the qualification-only READY-event validator."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest


HARNESS = Path(__file__).with_name("phase4_p4e6b_readiness50_harness.py")
SPEC = importlib.util.spec_from_file_location("f3_readiness_harness", HARNESS)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

F3_READY = Path(
    "/home/dog/phase4_runtime/p4e6b2c1f3_readiness50_20260814T004251+0800/"
    "campaign_P4E6B2C1F3_READINESS50_20260814T004725+0800/episode_01/premission_seam.json"
)


def _record():
    return json.loads(F3_READY.read_text())


def test_f3_episode_01_replays_against_frozen_emitted_schema():
    record = _record()
    assert MODULE._valid_ready(record)
    assert "publisher_node" not in record["event"]
    assert record["event"]["generation"][0] == "collision_monitor_validity_monitor"


@pytest.mark.parametrize("mutate", [
    lambda row: row["event"].update(publisher_count=2),
    lambda row: row["event"].update(generation=["wrong", "gid"]),
    lambda row: row["event"].update(bool_post_epoch_count=1),
    lambda row: row["event"].update(bool_value=False),
    lambda row: row["event"].update(diagnostic_state="INVALID"),
    lambda row: row["event"].update(diagnostic_reason="SOURCE_STALE"),
    lambda row: row["event"].update(semantic_healthy_stable_sec=.999),
    lambda row: row["event"].update(bool_age_ns=250_000_000),
    lambda row: row["event"].update(diagnostic_age_ns=250_000_000),
    lambda row: row["event"].update(diagnostic_transport_age_ns=250_000_000),
    lambda row: row["event"].update(source_age_upper_bound_sec=.5),
    lambda row: row["event"].update(epoch_start_ns=0),
    lambda row: row["event"].update(epoch_start_ros_ns=0),
])
def test_validator_rejects_each_invalid_emitted_ready_field(mutate):
    row = copy.deepcopy(_record())
    mutate(row)
    assert not MODULE._valid_ready(row)


@pytest.mark.parametrize("field", [
    "ready", "publisher_count", "generation", "bool_post_epoch_count", "bool_value",
    "diagnostic_state", "diagnostic_reason", "semantic_healthy_stable_sec", "bool_age_ns",
    "diagnostic_age_ns", "diagnostic_transport_age_ns", "source_age_upper_bound_sec",
    "epoch_start_ns", "epoch_start_ros_ns", "diagnostic_writer_gid",
])
def test_missing_required_emitted_field_fails_closed(field):
    row = copy.deepcopy(_record())
    row["event"].pop(field, None)
    assert not MODULE._valid_ready(row)


def test_non_emitted_publisher_node_is_not_a_validator_requirement():
    row = _record()
    row["event"].pop("publisher_node", None)
    assert MODULE._valid_ready(row)
