"""Qualification-only environment and pre-import worker boundary tests."""
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path


TEST_DIR = Path(__file__).parent
HARNESS_PATH = TEST_DIR / "phase4_p4e6b_readiness50_harness.py"
SEAM = TEST_DIR / "phase4_p4e6b_premission_seam.py"
sys.path.insert(0, str(TEST_DIR))
SPEC = importlib.util.spec_from_file_location("env_harness", HARNESS_PATH)
HARNESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HARNESS)


def _phases(directory):
    return [json.loads(line)["phase"] for line in (directory / "worker_phase.jsonl").read_text().splitlines()]


def test_child_environment_preserves_parent_and_overrides_only_episode_values():
    parent = {"PATH": "p", "PYTHONPATH": "py", "AMENT_PREFIX_PATH": "ament",
              "COLCON_PREFIX_PATH": "colcon", "LD_LIBRARY_PATH": "ld", "UNRELATED": "kept",
              "ROS_DOMAIN_ID": "old", "ROS_LOCALHOST_ONLY": "0"}
    child = HARNESS.child_environment(parent, 77, "token", "episode")
    assert parent["ROS_DOMAIN_ID"] == "old" and parent["ROS_LOCALHOST_ONLY"] == "0"
    for key in ("PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH", "UNRELATED"):
        assert child[key] == parent[key]
    assert child["ROS_DOMAIN_ID"] == "77"
    assert child["ROS_LOCALHOST_ONLY"] == "1"
    assert child["P4E6B_EPISODE_TOKEN"] == "token"
    assert child["P4E6B_EPISODE_UUID"] == "episode"


def test_two_child_environments_do_not_mutate_parent_or_each_other():
    parent = {"PATH": "p", "UNRELATED": "kept"}
    first = HARNESS.child_environment(parent, 78, "first", "one")
    second = HARNESS.child_environment(parent, 79, "second", "two")
    assert "P4E6B_EPISODE_TOKEN" not in parent
    assert first["P4E6B_EPISODE_TOKEN"] == "first"
    assert second["P4E6B_EPISODE_TOKEN"] == "second"
    assert first["UNRELATED"] == second["UNRELATED"] == "kept"


def test_bad_environment_records_preimport_failure(tmp_path):
    # This is deliberately not a child_environment() result: it removes every
    # ROS prefix/search-path variable while retaining only enough OS context for
    # /usr/bin/python3 to run the stdlib bootstrap.
    env = {"PATH": "/usr/bin:/bin", "HOME": "/tmp", "P4E6B_EPISODE_TOKEN": "bad-token",
           "P4E6B_EPISODE_UUID": "bad-uuid"}
    result = subprocess.run(["/usr/bin/python3", str(SEAM), "--output-dir", str(tmp_path), "--bootstrap-only"],
                            env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert _phases(tmp_path)[:3] == ["PROCESS_START", "ROS_IMPORT_BEGIN", "ROS_IMPORT_EXCEPTION"]
    assert "HELPER_ENTER" not in _phases(tmp_path)
    artifact = json.loads((tmp_path / "ros_import_exception.json").read_text())
    assert artifact["exception_class"] == "ModuleNotFoundError"
    assert artifact["episode_token"] == "bad-token"
    assert artifact["sys_executable"] == "/usr/bin/python3"
    assert artifact["environment_fingerprint_sha256"]


def test_good_environment_reaches_ros_import_end(tmp_path):
    result = subprocess.run(["/usr/bin/python3", str(SEAM), "--output-dir", str(tmp_path), "--bootstrap-only"],
                            env=os.environ.copy(), capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert _phases(tmp_path)[:3] == ["PROCESS_START", "ROS_IMPORT_BEGIN", "ROS_IMPORT_END"]
    assert "HELPER_ENTER" not in _phases(tmp_path)
