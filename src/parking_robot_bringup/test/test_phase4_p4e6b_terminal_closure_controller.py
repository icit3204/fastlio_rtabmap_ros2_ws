import json

import pytest

from phase4_p4e6b_terminal_closure_controller import classify_runner_exit, wait_for_witness_commit


def test_runner_failure_before_terminal_is_not_rescued(tmp_path):
    result = classify_runner_exit(returncode=1, terminal_origin_seen=False, witness_dir=tmp_path)
    assert result["classification"] == "RUNNER_FAILURE_BEFORE_TERMINAL"


def test_runner_failure_waits_for_witness_commit(tmp_path):
    marker = tmp_path / "TERMINAL_WITNESS_OUTCOME_COMMITTED"
    marker.write_text(json.dumps({"ACTUAL_DRAIN_SEC": .251}))
    result = classify_runner_exit(returncode=1, terminal_origin_seen=True, witness_dir=tmp_path)
    assert result["classification"] == "WITNESS_CLOSURE_COMMITTED"
    assert result["witness"]["ACTUAL_DRAIN_SEC"] == .251


def test_missing_commit_is_hard_failure(tmp_path):
    with pytest.raises(TimeoutError):
        wait_for_witness_commit(tmp_path, timeout_sec=.01)
