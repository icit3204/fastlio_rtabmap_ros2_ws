import json
from pathlib import Path

import pytest

import phase4_p4e6b_hash_pinned_freeze as freeze
import phase4_p4e6b_live_characterization_controller as controller


class FakeProcess:
    def __init__(self, pid):
        self.pid = pid


class RecordingMatrix:
    def __init__(self):
        self.calls = []

    def __call__(self):
        self.calls.append(True)
        return FakeProcess(990001)


def authority(tmp_path):
    path = tmp_path / "authority.json"
    path.write_text(Path(controller.AUTHORITY_FILE).read_text())
    return path


def run(tmp_path, **kwargs):
    recorder = RecordingMatrix()
    result = controller.admit_live_matrix_before_popen(
        tmp_path / "episode", recorder, authority_path=authority(tmp_path), **kwargs)
    return result, recorder


def test_actual_controller_valid_admission_freezes_before_matrix(tmp_path):
    result, recorder = run(tmp_path)
    assert result["freeze"]["identity_match"] is True
    assert len(recorder.calls) == 1
    assert result["freeze"]["committed_ns"] < result["matrix_created_ns"] < result["runtime_started_marker_ns"]
    assert (tmp_path / "episode" / "FINAL_FREEZE_COMMITTED").is_file()
    record = json.loads((tmp_path / "episode" / "FINAL_FREEZE.json").read_text())
    assert record["expected_identities"] == record["actual_identities"]


@pytest.mark.parametrize("key", ["runner", "matrix", "live_controller", "freeze_controller", "witness", "relay"])
def test_actual_controller_identity_mismatch_denies_before_popen(tmp_path, key):
    path = authority(tmp_path)
    doc = json.loads(path.read_text())
    doc["required_pinned_identity"][key] = "mutated"
    path.write_text(json.dumps(doc))
    recorder = RecordingMatrix()
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_IDENTITY_MISMATCH"):
        controller.admit_live_matrix_before_popen(tmp_path / "episode", recorder, authority_path=path)
    assert recorder.calls == []


@pytest.mark.parametrize("kw", ["runner_path", "matrix_path", "controller_path", "freeze_controller_path", "witness_path", "relay_path"])
def test_actual_controller_mutated_artifact_denies_before_popen(tmp_path, kw):
    mutated = tmp_path / (kw + ".artifact")
    mutated.write_text("mutated artifact")
    recorder = RecordingMatrix()
    with pytest.raises(RuntimeError, match="FINAL_FREEZE_IDENTITY_MISMATCH"):
        controller.admit_live_matrix_before_popen(tmp_path / "episode", recorder,
                                                  authority_path=authority(tmp_path), **{kw: mutated})
    assert recorder.calls == []


@pytest.mark.parametrize("fault", ["write", "fsync", "rename", "parent_fsync", "readback"])
def test_actual_controller_durability_fault_denies_before_popen(tmp_path, monkeypatch, fault):
    def fail(path, value):
        raise OSError("injected " + fault)
    monkeypatch.setattr(freeze, "_durable_json", fail)
    recorder = RecordingMatrix()
    with pytest.raises(OSError):
        controller.admit_live_matrix_before_popen(tmp_path / "episode", recorder,
                                                  authority_path=authority(tmp_path))
    assert recorder.calls == []

