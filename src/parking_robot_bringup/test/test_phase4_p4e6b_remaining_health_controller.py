import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import phase4_p4e6b_remaining_health_controller as remaining_controller_module
from parking_robot_bringup.phase4_p4e6b_remaining_evidence import load_genuine_remaining_case_evidence
from phase4_p4e6b_remaining_health_controller import (
    AUTHORITY_TOKENS, CASE_CONFIG, RemainingHealthController,
    RemainingLiveRosFactory,
)

AUTHORITY_DIR = Path(__file__).resolve().parent


def _write_jsonl(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def _remaining_runner(tmp_path, case, *, age=None, event=None, wrong_uuid=False, tool_state="INJECTED"):
    config = CASE_CONFIG[case]
    out = tmp_path / case
    out.mkdir(parents=True)
    uuid = "wrong" if wrong_uuid else "u"
    _write_jsonl(out / "health_campaign.jsonl", [
        {"event": "ATTEMPT_STARTED", "case_id": config["case_id"], "monotonic_ns": 10},
        {"event": "ARMED_FOR_CASE", "case_id": config["case_id"], "goal_uuid": uuid, "monotonic_ns": 20},
        {"event": "INJECTED", "case_id": config["case_id"], "request_ns": 30, "ack_ns": 31,
         "success": True, "tool_state": tool_state, "injection_count": 1},
    ])
    values = {"mission_id": "m", "route_id": "r", "active_goal_uuid": uuid,
              "progress_supervisor_event": "NO_PROGRESS_PENDING",
              "odometry_receipt_steady_sec": "10.0",
              "tf_receipt_steady_sec": "10.0",
              "raw_command_receipt_steady_sec": "10.0",
              "safe_command_receipt_steady_sec": "10.0"}
    previous = {"monotonic_ns": 100, "values": values}
    transition_values = {**values, "progress_supervisor_event": config["expected_reason"]}
    if case == "S02":
        transition_values["odometry_age_sec"] = 0.501
    elif case == "S03":
        transition_values["tf_age_sec"] = 0.501
    else:
        transition_values.update({"raw_command_age_sec": 0.10, "safe_command_age_sec": 0.30,
                           "raw_safe_skew_sec": 0.01, "pair_liveness_age_sec": 0.30,
                           "command_stream_phase": "PAIRING"})
    if age is not None:
        field = {"S02": "odometry_age_sec", "S03": "tf_age_sec"}.get(case)
        if field:
            transition_values[field] = age
    if event is not None:
        transition_values["progress_supervisor_event"] = event
    transition = {"monotonic_ns": 200, "values": transition_values}
    _write_jsonl(out / "mission_policy_diagnostics.jsonl", [previous, transition])
    _write_jsonl(out / "mission_state_events.jsonl", [
        {"monotonic_ns": 150, "mission_id": "m", "route_id": "r",
         "active_goal_uuid": uuid, "state_name": "CANCELLING",
         "reason_code": config["expected_reason"]},
        {"monotonic_ns": 160, "mission_id": "m", "route_id": "r",
         "active_goal_uuid": uuid, "state_name": "CANCELLING",
         "reason_code": "HEALTH_CANCEL_ACK_ACCEPTED"},
    ])
    _write_jsonl(out / "navigate_action_status_events.jsonl", [
        {"monotonic_ns": 155, "goal_uuid": uuid, "status": 3},
    ])
    (out / "terminal_metrics.json").write_text(json.dumps({"pass": False}))
    return out


@pytest.mark.parametrize("case,reason,case_id", [
    ("S02", "ODOMETRY_STALE", "B-S02"),
    ("S03", "TF_STALE", "B-S03"),
    ("C01", "COMMAND_PAIR_STALE", "B-C01"),
])
def test_fixed_case_mapping_and_genuine_policy_adapter(tmp_path, case, reason, case_id):
    evidence = load_genuine_remaining_case_evidence(_remaining_runner(tmp_path, case), case)
    assert evidence["expected_reason"] == reason
    assert evidence["case_id"] == case_id
    assert evidence["cancel_count"] == 1
    assert evidence["canceling_status"] == 3
    assert evidence["health_cancel_ack"] == "HEALTH_CANCEL_ACK_ACCEPTED"
    assert evidence["originating_reason"] == reason


def test_remaining_adapter_accepts_transition_rows_without_passive_fields(tmp_path):
    evidence = load_genuine_remaining_case_evidence(_remaining_runner(tmp_path, "S02"), "S02")
    assert evidence["branch"]["age_sec"] > .5
    assert evidence["branch"]["passive_rows_before_transition"] == 1


@pytest.mark.parametrize("case", ["S02", "S03", "C01"])
def test_remaining_adapter_accepts_successful_confirmed_response(tmp_path, case):
    evidence = load_genuine_remaining_case_evidence(
        _remaining_runner(tmp_path, case, tool_state="CONFIRMED"), case)
    assert evidence["injection_response_successful"] is True
    assert evidence["relay_injection_count"] == 1
    assert evidence["tool_state"] == "CONFIRMED"


def test_untouched_b2bf_runner_replay_passes_remaining_injection_gate():
    evidence = load_genuine_remaining_case_evidence(
        Path("/home/dog/phase4_reports/B2BF_S02_ATTEMPT1_001/runner"), "S02")
    assert evidence["injection_response_successful"] is True
    assert evidence["relay_injection_count"] == 1
    assert evidence["tool_state"] == "CONFIRMED"


def test_remaining_adapter_fail_closed_on_threshold_and_reason(tmp_path):
    with pytest.raises(RuntimeError, match="THRESHOLD"):
        load_genuine_remaining_case_evidence(_remaining_runner(tmp_path, "S02", age=.5), "S02")
    with pytest.raises(RuntimeError, match="REMAINING_CASE_AGE_EVIDENCE_MISSING"):
        load_genuine_remaining_case_evidence(
            _remaining_runner(tmp_path / "wrong", "S03", event="ODOMETRY_STALE"), "S03")


def test_unknown_case_and_authority_cross_pair_are_denied(tmp_path):
    with pytest.raises(RuntimeError, match="DENIED"):
        RemainingHealthController(tmp_path / "x", case_name="S01", authority_path=tmp_path / "none")
    controller = RemainingHealthController(tmp_path / "s02", case_name="S02", authority_path=tmp_path / "none")
    with pytest.raises(RuntimeError, match="AUTHORITY_DENIED"):
        controller.admission(mode="LIVE", supervisor_authority=AUTHORITY_TOKENS["S03"])


@pytest.mark.parametrize("case", ["S02", "S03", "C01"])
def test_case_specific_commands_are_path_valued_and_namespaced(tmp_path, case):
    factory = RemainingLiveRosFactory(tmp_path / case, case_name=case, domain=220,
                                      token="token", episode_uuid="uuid")
    factory.ros2 = "/usr/bin/ros2"
    controller = RemainingHealthController(tmp_path / (case + "-controller"),
                                            case_name=case, authority_path=tmp_path / "none")
    contract = controller.command_contract(factory)
    assert contract["case_id"] == CASE_CONFIG[case]["case_id"]
    assert CASE_CONFIG[case]["expected_reason"] in contract["commands"]["witness"]
    assert CASE_CONFIG[case]["case_id"] in " ".join(contract["commands"]["matrix"])
    assert "Path(" in " ".join(contract["commands"]["runner"])
    assert contract["environment"]["ROS_DOMAIN_ID"] == "220"


@pytest.mark.parametrize("case", ["S02", "S03", "C01"])
def test_authority_pins_all_direct_live_dependencies(tmp_path, case):
    authority = AUTHORITY_DIR / f"phase4_p4e6b_remaining_{case}_authority.json"
    controller = RemainingHealthController(tmp_path / case, case_name=case,
                                            authority_path=authority)
    result = controller.admission(mode="DRY", supervisor_authority=None)
    assert result["identity_match"] is True
    assert set(result["expected"]) == {
        "remaining_controller", "s01_base_controller", "remaining_evidence",
        "base_live_evidence", "terminal_v2", "runner", "matrix",
        "freeze_controller", "witness", "feedback_relay",
        "observation_relay", "tf_observation_relay", "feedback_relay_installed",
        "observation_relay_installed", "tf_observation_relay_installed",
    }


@pytest.mark.parametrize("field", ["terminal_v2", "s01_base_controller", "base_live_evidence"])
def test_missing_or_mutated_direct_dependency_denied(tmp_path, field):
    source = json.loads((AUTHORITY_DIR / "phase4_p4e6b_remaining_S02_authority.json").read_text())
    source["required_pinned_identity"][field] = "mutated"
    authority = tmp_path / "authority.json"
    authority.write_text(json.dumps(source))
    controller = RemainingHealthController(tmp_path / "out", case_name="S02",
                                            authority_path=authority)
    with pytest.raises(RuntimeError, match="IDENTITY_MISMATCH"):
        controller.admission(mode="DRY", supervisor_authority=None)


def test_missing_expected_identity_denied(tmp_path):
    source = json.loads((AUTHORITY_DIR / "phase4_p4e6b_remaining_S02_authority.json").read_text())
    del source["required_pinned_identity"]["terminal_v2"]
    authority = tmp_path / "authority-missing.json"
    authority.write_text(json.dumps(source))
    controller = RemainingHealthController(tmp_path / "out", case_name="S02",
                                            authority_path=authority)
    with pytest.raises(RuntimeError, match="IDENTITY_MISMATCH"):
        controller.admission(mode="DRY", supervisor_authority=None)


class _FailureProcess:
    pid = 92001
    pgid = 92001
    sid = 92001
    ros_domain = 220
    def poll(self):
        return None


class _FailureFactory:
    def __init__(self, fail_role, cleanup_fail_after=None):
        self.fail_role = fail_role
        self.cleanup_fail_after = cleanup_fail_after
        self.processes = {}
        self.cleanup_calls = 0

    def cleanup_global_zero(self):
        self.cleanup_calls += 1
        if self.cleanup_fail_after is not None and self.cleanup_calls > self.cleanup_fail_after:
            return {"scan1": [], "scan2": [], "wait_sec": 2.0,
                    "graph_clean": False, "pass": False,
                    "graph": {"clean": False, "returncode": 1,
                              "stdout": "", "stderr": "probe failure",
                              "nodes": [], "target_nodes": []}}
        return {"scan1": [], "scan2": [], "wait_sec": 2.0,
                "graph_clean": True, "pass": True,
                "graph": {"clean": True, "returncode": 0,
                          "stdout": "", "stderr": "",
                          "nodes": [], "target_nodes": []}}

    def spawn(self, role):
        if role == self.fail_role:
            raise RuntimeError(role.upper() + "_FAILURE")
        process = _FailureProcess()
        self.processes[role] = process
        return process

    def witness_ready(self):
        return True

    def matrix_alive(self, process):
        return True

    def wait_runner(self, timeout=45):
        raise RuntimeError("RUNNER_FAILURE")

    def runner_evidence(self):
        return {}

    def wait_witness_outcome(self, timeout=5):
        raise RuntimeError("WITNESS_FAILURE")


def _controller_with_authority(tmp_path, case="S02"):
    return RemainingHealthController(
        tmp_path / "out", case_name=case,
        authority_path=AUTHORITY_DIR / f"phase4_p4e6b_remaining_{case}_authority.json")


def test_pre_marker_witness_failure_is_unconsumed_and_cleaned(tmp_path):
    controller = _controller_with_authority(tmp_path)
    result = controller.execute_live(_FailureFactory("witness"),
                                     supervisor_authority=AUTHORITY_TOKENS["S02"],
                                     attempt_identity="B2BB_S02_ATTEMPT1_001")
    assert result["classification"] == "UNCONSUMED_PRE_RUNTIME_FAILURE"
    assert result["attempt_consumed"] is False
    assert not (controller.output / controller._marker_name()).exists()
    assert (controller.output / "S02_ATTEMPT1_PRE_RUNTIME_FAILURE.json").is_file()
    assert not (controller.output / "S02_ATTEMPT1_FINAL_RESULT.json").exists()


def test_pre_marker_matrix_failure_is_unconsumed_and_cleaned(tmp_path):
    controller = _controller_with_authority(tmp_path)
    factory = _FailureFactory("matrix")
    result = controller.execute_live(factory,
                                     supervisor_authority=AUTHORITY_TOKENS["S02"],
                                     attempt_identity="B2BB_S02_ATTEMPT1_001")
    assert result["classification"] == "UNCONSUMED_PRE_RUNTIME_FAILURE"
    assert result["attempt_consumed"] is False
    assert factory.cleanup_calls >= 2


def test_post_marker_failure_is_consumed_and_cleanup_is_recorded(tmp_path):
    controller = _controller_with_authority(tmp_path)
    result = controller.execute_live(_FailureFactory("runner", cleanup_fail_after=1),
                                     supervisor_authority=AUTHORITY_TOKENS["S02"],
                                     attempt_identity="B2BB_S02_ATTEMPT1_001")
    assert result["final_result"] == "CONSUMED_FAILED"
    assert result["attempt_consumed"] is True
    assert result[controller._marker_name()] is True
    assert result["cleanup"]["pass"] is False
    assert result["cleanup"]["graph"]["returncode"] == 1
    assert (controller.output / "S02_ATTEMPT1_FINAL_RESULT.json").is_file()


class _PreflightPredicateFactory(_FailureFactory):
    def __init__(self, preflight):
        super().__init__(fail_role="never")
        self.preflight = preflight
        self.spawn_calls = []

    def cleanup_global_zero(self):
        self.cleanup_calls += 1
        return dict(self.preflight)

    def spawn(self, role):
        self.spawn_calls.append(role)
        raise AssertionError("process creation must not occur")


def test_preflight_requires_authoritative_pass_even_when_graph_is_clean(tmp_path):
    controller = _controller_with_authority(tmp_path)
    factory = _PreflightPredicateFactory({
        "scan1": ["old target"], "scan2": ["old target"],
        "graph_clean": True, "pass": False,
    })
    result = controller.execute_live(
        factory, supervisor_authority=AUTHORITY_TOKENS["S02"],
        attempt_identity="B2BH_S02_PREFLIGHT_NEGATIVE_001")
    assert result["classification"] == "UNCONSUMED_PRE_RUNTIME_FAILURE"
    assert result["attempt_consumed"] is False
    assert factory.spawn_calls == []
    assert not (controller.output / controller._marker_name()).exists()
    assert not (controller.output / "S02_ATTEMPT1_FINAL_RESULT.json").exists()


def test_clean_authoritative_preflight_reaches_process_admission(tmp_path):
    controller = _controller_with_authority(tmp_path)
    factory = _FailureFactory("witness")
    result = controller.execute_live(
        factory, supervisor_authority=AUTHORITY_TOKENS["S02"],
        attempt_identity="B2BH_S02_PREFLIGHT_POSITIVE_001")
    assert result["classification"] == "UNCONSUMED_PRE_RUNTIME_FAILURE"
    assert any(event["event"] == "GLOBAL_ZERO_PREFLIGHT_PASS"
               for event in result["events"])
    assert "witness" not in factory.processes


def _cleanup_factory(tmp_path, *, owned=True):
    factory = RemainingLiveRosFactory(tmp_path / "cleanup", case_name="S03",
                                      domain=222, token="token", episode_uuid="uuid")
    factory.processes = {"witness": SimpleNamespace(pgid=222)} if owned else {}
    factory.terminate_owned = lambda: None
    return factory


def test_post_runtime_owned_settling_allows_one_full_second_observation(tmp_path, monkeypatch):
    observations = iter([
        {"scan1": [{"pgid": 222}], "scan2": [],
         "graph": {"clean": False}, "pass": False, "wait_sec": 2.0},
        {"scan1": [], "scan2": [],
         "graph": {"clean": True}, "pass": True, "wait_sec": 2.0},
    ])
    monkeypatch.setattr(remaining_controller_module, "authoritative_global_zero",
                        lambda: next(observations))
    result = _cleanup_factory(tmp_path).cleanup_global_zero()
    assert result["pass"] is True
    assert result["graph_clean"] is True
    assert result["settle_observation_used"] is True
    assert result["initial"]["pass"] is False


def test_post_runtime_foreign_residue_cannot_use_settling_allowance(tmp_path, monkeypatch):
    observations = iter([{"scan1": [{"pgid": 999}], "scan2": [],
                          "graph": {"clean": False}, "pass": False}])
    monkeypatch.setattr(remaining_controller_module, "authoritative_global_zero",
                        lambda: next(observations))
    result = _cleanup_factory(tmp_path).cleanup_global_zero()
    assert result["pass"] is False
    assert result["settle_observation_used"] is False


def test_post_runtime_persistent_owned_residue_cannot_settle(tmp_path, monkeypatch):
    observations = iter([{"scan1": [{"pgid": 222}], "scan2": [{"pgid": 222}],
                          "graph": {"clean": False}, "pass": False}])
    monkeypatch.setattr(remaining_controller_module, "authoritative_global_zero",
                        lambda: next(observations))
    result = _cleanup_factory(tmp_path).cleanup_global_zero()
    assert result["pass"] is False
    assert result["settle_observation_used"] is False


def test_post_runtime_second_observation_failure_stays_failed(tmp_path, monkeypatch):
    observations = iter([
        {"scan1": [{"pgid": 222}], "scan2": [],
         "graph": {"clean": False}, "pass": False},
        {"scan1": [], "scan2": [],
         "graph": {"clean": False}, "pass": False},
    ])
    monkeypatch.setattr(remaining_controller_module, "authoritative_global_zero",
                        lambda: next(observations))
    result = _cleanup_factory(tmp_path).cleanup_global_zero()
    assert result["pass"] is False
    assert result["settle_observation_used"] is True


def test_preflight_without_owned_processes_has_no_settling_allowance(tmp_path, monkeypatch):
    observations = iter([{"scan1": ["old target"], "scan2": [],
                          "graph": {"clean": True}, "pass": False}])
    monkeypatch.setattr(remaining_controller_module, "authoritative_global_zero",
                        lambda: next(observations))
    result = _cleanup_factory(tmp_path, owned=False).cleanup_global_zero()
    assert result["pass"] is False
    assert result["settle_observation_used"] is False


def test_clean_cleanup_does_not_take_second_observation(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(remaining_controller_module, "authoritative_global_zero",
                        lambda: calls.append(True) or {"scan1": [], "scan2": [],
                        "graph": {"clean": True}, "pass": True})
    result = _cleanup_factory(tmp_path).cleanup_global_zero()
    assert result["pass"] is True
    assert result["settle_observation_used"] is False
    assert len(calls) == 1
