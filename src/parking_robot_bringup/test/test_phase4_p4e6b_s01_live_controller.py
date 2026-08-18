import hashlib
from pathlib import Path

import pytest

import phase4_p4e6b_s01_attempt4_controller as c


def test_live_factory_builds_exact_commands_and_environment(tmp_path):
    f = c.LiveRosFactory(tmp_path, domain=231, token="token-231", episode_uuid="uuid-231")
    witness = f.command("witness")
    matrix = f.command("matrix")
    runner = f.command("runner")
    assert witness[-2:] == ["--reason", "FEEDBACK_STALE"]
    assert "phase4_p4e6b_terminal_closure_witness.py" in witness[1]
    assert matrix[-2:] == ["enable_health_runner:=false", "case_id:=B-S01"]
    assert "phase4_p4e6b_health_failure_runner" in " ".join(runner)
    assert "execute_campaign" in runner[-1]
    assert "from pathlib import Path" in runner[-1]
    assert "Path(" in runner[-1]
    assert "execute_campaign('B-S01', Path(" in runner[-1]
    assert "execute_campaign('B-S01', '" not in runner[-1]
    # Exercise the same output-value contract without starting a health case.
    namespace = {}
    exec(f"from pathlib import Path\noutput = Path({str(tmp_path / 'qualification-output')!r})\noutput.mkdir(parents=True, exist_ok=True)", namespace)
    assert namespace["output"].is_dir()
    assert f.env["ROS_DOMAIN_ID"] == "231"
    assert f.env["P4E6B_EPISODE_TOKEN"] == "token-231"
    assert f.env["P4E6B_EPISODE_UUID"] == "uuid-231"


def test_live_mode_without_future_authority_denies_before_processes(tmp_path):
    f = c.RecordingFactory()
    with pytest.raises(RuntimeError, match="LIVE_MODE_AUTHORITY_DENIED"):
        c.S01Attempt4Controller(tmp_path).execute(f, mode="LIVE")
    assert f.calls == []
    assert not (tmp_path / "S01_ATTEMPT4_RUNTIME_STARTED").exists()


def test_shared_live_nonattempt_recording_path_is_nonconsuming(tmp_path):
    f = c.RecordingFactory()
    result = c.S01Attempt4Controller(tmp_path).execute(
        f, mode="LIVE", supervisor_authority="B2AN_NONATTEMPT_SMOKE",
        attempt_identity="B2AS_NONATTEMPT_SMOKE", stop_before_marker=True,
        preflight_fn=lambda: {"pass": True, "scan1": [], "scan2": [],
                              "graph": {"clean": True}})
    assert result["pass"] is True
    assert f.calls == ["witness", "matrix"]
    assert result["S01_ATTEMPT4_RUNTIME_STARTED"] is False
    assert result["attempt_consumed"] is False


def test_real_witness_ready_integration(tmp_path):
    f = c.LiveRosFactory(tmp_path, domain=231, token="b2an-witness-token", episode_uuid="b2an-witness-uuid")
    try:
        process = f.spawn("witness")
        assert f.witness_ready(timeout=15)
        marker = tmp_path / "terminal_witness" / "TERMINAL_WITNESS_READY"
        assert marker.is_file()
        assert process.poll() is None
    finally:
        f.terminate_owned()


def test_real_nonattempt_matrix_smoke(tmp_path):
    f = c.LiveRosFactory(tmp_path, domain=232, token="b2an-matrix-token", episode_uuid="b2an-matrix-uuid")
    try:
        result = c.S01Attempt4Controller(
            tmp_path, preflight_fn=lambda: {"pass": True, "scan1": [], "scan2": [],
                                            "graph": {"clean": True}}).execute_live_nonattempt(f)
        assert result["pass"] is True
        assert result["S01_ATTEMPT4_RUNTIME_STARTED"] is False
        assert not (tmp_path / "S01_ATTEMPT4_RUNTIME_STARTED").exists()
        assert (tmp_path / "terminal_witness" / "TERMINAL_WITNESS_READY").is_file()
    finally:
        f.terminate_owned()
