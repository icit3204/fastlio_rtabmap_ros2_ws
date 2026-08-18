"""Pure accounting tests for worker-authored F3B2 lifecycle phases."""
import importlib.util
from pathlib import Path

import pytest


HARNESS = Path(__file__).with_name("phase4_p4e6b_readiness50_harness.py")
SPEC = importlib.util.spec_from_file_location("f3b2_harness", HARNESS)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def phases(*names):
    return [{"phase": name, "monotonic_ns": index} for index, name in enumerate(names, 1)]


def test_popen_only_is_not_committed():
    assert MODULE.lifecycle_accounting([])["state"] == "PRE_HELPER"


@pytest.mark.parametrize("names", [
    ("PROCESS_START",), ("PROCESS_START", "RCLPY_INIT_END"),
    ("PROCESS_START", "RUNNER_CONSTRUCT_END"),
    ("PROCESS_START", "INITIAL_SPIN_END"),
])
def test_pre_helper_phases_are_not_committed(names):
    assert not MODULE.lifecycle_accounting(phases(*names))["committed"]


def test_fsynced_helper_enter_is_the_only_commit_boundary():
    row = MODULE.lifecycle_accounting(phases("PROCESS_START", "INITIAL_SPIN_END", "HELPER_ENTER"))
    assert row == {"state": "COMMITTED", "committed": True, "outcome": None, "process_exit": False}


def test_ready_is_preserved_even_if_teardown_has_not_exited():
    row = MODULE.lifecycle_accounting(phases("HELPER_ENTER", "HELPER_RETURN_READY"))
    assert row["committed"] and row["outcome"] == "READY" and not row["process_exit"]


def test_timeout_exception_is_preserved_even_if_teardown_has_not_exited():
    row = MODULE.lifecycle_accounting(phases("HELPER_ENTER", "HELPER_EXCEPTION"))
    assert row["committed"] and row["outcome"] == "EXCEPTION" and not row["process_exit"]


def test_worker_killed_before_helper_entry_is_pre_helper_failure():
    assert MODULE.lifecycle_accounting(phases("PROCESS_START", "PROCESS_EXIT"))["state"] == "PRE_HELPER"


def test_missing_summary_after_helper_entry_does_not_uncommit():
    assert MODULE.lifecycle_accounting(phases("HELPER_ENTER"))["committed"]


def test_duplicate_helper_enter_fails_closed():
    assert MODULE.lifecycle_accounting(phases("HELPER_ENTER", "HELPER_ENTER"))["state"] == "DUPLICATE_HELPER_ENTER"


@pytest.mark.parametrize("names,expected", [
    (("HELPER_ENTER", "HELPER_RETURN_READY", "PROCESS_EXIT"), "READY"),
    (("HELPER_ENTER", "HELPER_EXCEPTION", "PROCESS_EXIT"), "EXCEPTION"),
])
def test_crash_reconstruction_retains_exact_helper_outcome(names, expected):
    row = MODULE.lifecycle_accounting(phases(*names))
    assert row["committed"] and row["outcome"] == expected and row["process_exit"]
