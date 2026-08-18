import json
from pathlib import Path

import pytest

import phase4_p4e6b_s01_attempt4_controller as c


def controller(tmp_path, **kwargs):
    return c.S01Attempt4Controller(tmp_path / "episode", **kwargs)


def test_full_dry_attempt_state_machine_and_order(tmp_path):
    ctl = controller(tmp_path)
    factory = c.RecordingFactory()
    result = ctl.execute_dry(factory)
    assert result["pass"] is True
    assert result["strict_order"] is True
    assert factory.calls == ["witness", "matrix", "runner"]
    assert result["S01_ATTEMPT4_RUNTIME_STARTED"] is False
    assert result["attempt_consumed"] is False
    assert [x["event"] for x in result["events"]].index("TERMINAL_WITNESS_READY") < [x["event"] for x in result["events"]].index("MATRIX_PROCESS_CREATED")


@pytest.mark.parametrize("key", ["runner", "matrix", "live_controller", "freeze_controller", "witness", "relay"])
def test_identity_mismatch_denies_before_any_process(tmp_path, key):
    ctl = controller(tmp_path, identity_overrides={key: "mutated"})
    factory = c.RecordingFactory()
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_IDENTITY_MISMATCH"):
        ctl.execute_dry(factory)
    assert factory.calls == []
    assert not (tmp_path / "episode" / "FINAL_FREEZE_COMMITTED").exists()


def test_terminal_v2_and_physical_identity_mismatches_are_pinned(tmp_path):
    for key in ("terminal_v2", "matrix_installed"):
        ctl = controller(tmp_path / key, identity_overrides={key: "mutated"})
        factory = c.RecordingFactory()
        with pytest.raises(RuntimeError, match="FINAL_FREEZE_IDENTITY_MISMATCH"):
            ctl.execute_dry(factory)
        assert factory.calls == []


@pytest.mark.parametrize("kwargs", [{"fail": "matrix"}, {"matrix_alive": False}])
def test_matrix_failure_preserves_unconsumed_boundary(tmp_path, kwargs):
    ctl = controller(tmp_path)
    factory = c.RecordingFactory(**kwargs)
    result = ctl.execute_dry(factory)
    assert result["classification"] == "UNCONSUMED_PRE_RUNTIME_FAILURE"
    assert factory.calls == ["witness", "matrix"]
    assert "runner" not in factory.calls
    assert not (tmp_path / "episode" / "S01_ATTEMPT4_RUNTIME_STARTED").exists()


@pytest.mark.parametrize("kwargs", [{"fail": "witness"}, {"ready": False}])
def test_witness_failure_preserves_no_matrix_and_unconsumed(tmp_path, kwargs):
    ctl = controller(tmp_path)
    factory = c.RecordingFactory(**kwargs)
    result = ctl.execute_dry(factory)
    assert result["classification"] == "UNCONSUMED_PRE_RUNTIME_FAILURE"
    assert factory.calls == ["witness"]
    assert not (tmp_path / "episode" / "S01_ATTEMPT4_RUNTIME_STARTED").exists()


def test_early_runner_failure_is_not_rescued(tmp_path):
    ctl = controller(tmp_path)
    result = ctl.execute_dry(c.RecordingFactory(runner_exit=1), runner_before_terminal=True)
    assert result["pass"] is False
    assert any(x["event"] == "RUNNER_FAILURE_BEFORE_TERMINAL" for x in result["events"])
    assert result["attempt_consumed"] is False


def test_runner_exit_after_terminal_keeps_witness_alive_for_drain(tmp_path):
    ctl = controller(tmp_path)
    result = ctl.execute_dry(c.RecordingFactory(runner_exit=1), runner_exit_after_terminal=True)
    assert result["pass"] is True
    names = [x["event"] for x in result["events"]]
    assert names.index("WITNESS_POST_FAILED_DRAIN") < names.index("CLEANUP_GLOBAL_ZERO")
    assert "RUNNER_EXIT_AFTER_TERMINAL_WITNESS_DRAIN_CONTINUES" in names


def test_physical_negative_paths_fail_closed(tmp_path):
    for physical in (False,):
        with pytest.raises(RuntimeError, match="PHYSICAL_CLOSURE_FAILURE"):
            controller(tmp_path / "negative").execute_dry(c.RecordingFactory(), physical=physical)


def test_authority_is_prospective_only_and_has_no_attempt_marker(tmp_path):
    authority = json.loads(Path(c.AUTHORITY_FILE).read_text())
    assert authority["authority_status"] == "PROSPECTIVE_ONLY_PENDING_SUPERVISOR_AUTHORIZATION"
    result = controller(tmp_path).execute_dry(c.RecordingFactory())
    assert result["S01_ATTEMPT4_RUNTIME_STARTED"] is False
