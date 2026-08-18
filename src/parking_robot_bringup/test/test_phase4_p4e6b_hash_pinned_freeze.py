import json
from pathlib import Path

import pytest

import phase4_p4e6b_hash_pinned_freeze as f


EXPECTED = {
    "runner": "458d2a6e184079a1236d28664c4554292a9b4a2cef72e3c576f5c4b89d545b71",
    "matrix": "74c6f02f13c986574886a14afa98995919ec9e988a83846e895af6228109d069",
    "live_controller": "c365bba0142cf21bb1b0840d202fea3c01a635085f23b7f90a9e705eee1f2bf8",
    "freeze_controller": "ce2bb254cb22a4f649ee2c86ce053eae7e2a849b8645238e0fc2b431f73156cc",
    "witness": "4bda155dffc615da3b78c34b026874e1955b72f93bb2768276eef75c97528881",
    "relay": "0c2ca99568ad5acd08fef2d31bfa70c2b50aa273a02947576931397a799ce252",
}


def actual():
    return dict(EXPECTED)


def test_valid_pinned_admission_commits_then_allows_one_matrix(tmp_path):
    f.RecordingPopen.calls = []
    result = f.prospective_admission(tmp_path, EXPECTED, actual())
    assert result["freeze"]["identity_match"] is True
    assert result["popen_call_count"] == 1
    assert (tmp_path / "FINAL_FREEZE_COMMITTED").is_file()
    record = json.loads((tmp_path / "FINAL_FREEZE.json").read_text())
    assert record["expected_identities"] == EXPECTED
    assert record["actual_identities"] == EXPECTED


@pytest.mark.parametrize("key", ["runner", "matrix", "live_controller", "freeze_controller"])
def test_required_identity_mismatch_denies_before_popen(tmp_path, key):
    mutated = actual(); mutated[key] = "mutated"
    f.RecordingPopen.calls = []
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_IDENTITY_MISMATCH"):
        f.prospective_admission(tmp_path, EXPECTED, mutated)
    assert len(f.RecordingPopen.calls) == 0
    assert not (tmp_path / "FINAL_FREEZE_COMMITTED").exists()


def test_missing_head_denies_before_popen(tmp_path):
    f.RecordingPopen.calls = []
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_IDENTITY_MISMATCH"):
        f.prospective_admission(tmp_path, {"runner": EXPECTED["runner"]}, actual())
    assert len(f.RecordingPopen.calls) == 0


@pytest.mark.parametrize("fault", ["write", "fsync", "rename", "parent_fsync", "readback"])
def test_durability_fault_denies_before_popen(tmp_path, monkeypatch, fault):
    original = f._durable_json

    def fail(path, value):
        raise OSError("injected " + fault)

    monkeypatch.setattr(f, "_durable_json", fail)
    f.RecordingPopen.calls = []
    with pytest.raises(OSError):
        f.prospective_admission(tmp_path, EXPECTED, actual())
    assert len(f.RecordingPopen.calls) == 0
    assert not (tmp_path / "FINAL_FREEZE_COMMITTED").exists()
    assert original is not None


def test_readback_mismatch_denies_before_popen(tmp_path, monkeypatch):
    def mismatch(path, value):
        if path.name == "FINAL_FREEZE.json":
            raise RuntimeError("FINAL_FREEZE_READBACK_MISMATCH")
        return "x"
    monkeypatch.setattr(f, "_durable_json", mismatch)
    f.RecordingPopen.calls = []
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_READBACK_MISMATCH"):
        f.prospective_admission(tmp_path, EXPECTED, actual())
    assert len(f.RecordingPopen.calls) == 0


@pytest.mark.parametrize("order", [(20, 10, 30), (10, 20, 20), (10, 20, 15)])
def test_invalid_process_order_rejected(order):
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_PROCESS_ORDER_INVALID"):
        f.assert_process_order(*order)


def test_runtime_marker_before_matrix_rejected():
    with pytest.raises(RuntimeError):
        f.assert_process_order(20, 30, 10)
