import copy
import json
import os
import signal
import time
from pathlib import Path

import pytest

from parking_robot_bringup.phase4_p4e6c_controller import ProgressQualificationController
from parking_robot_bringup.phase4_p4e6c_progress_evidence import command_pair_branch, command_pair_ready, optional_float, validate_block_terminal
from parking_robot_bringup.phase4_p4e6c_progress_runner import (
    CASES, NO_PROGRESS_THRESHOLD_SEC, POSE_CLAMP_RATE_HZ, ProgressAdmission,
    clamp_tick_times, validate_feedback_cadence, validate_live_observation,
    validate_no_progress_contract, validate_pose_clamp_gate, validate_recovery_exhaustion,
)
from parking_robot_bringup.phase4_p4e6c_progress_witness import ProgressWitness
from parking_robot_bringup.phase4_p4e6c_live_driver import SealedLiveDriver, parse_trigger_success
from parking_robot_bringup.phase4_p4e6c_live_driver import parse_trigger_success
from parking_robot_bringup.phase4_p4e6c_progress_recorder import (
    FreshEvidenceReader, ProgressEvidenceRecorder, readiness_from_mapping,
)
from parking_robot_bringup.phase4_p4e6c_progress_observer import normalize_progress_diagnostic
from parking_robot_bringup.phase4_p4e6c_progress_observer import normalize_uint8


def test_exact_two_cases_and_source_thresholds():
    assert set(CASES) == {"C-P01", "C-P02"}
    assert NO_PROGRESS_THRESHOLD_SEC == 20.0
    assert POSE_CLAMP_RATE_HZ == 10.0


def test_humble_response_block_parser_matches_retained_and_generated_forms():
    retained = """waiting for service to become available...\nresponse:\nstd_srvs.srv.Trigger_Response(success=True, message='VALID')\n"""
    assert parse_trigger_success(retained, returncode=0) is True
    assert parse_trigger_success(
        "response:\nstd_srvs.srv.Trigger_Response(\n    success=False,\n    message='NO_RECEIVED_MISSION'\n)\n",
        returncode=0) is False
    assert parse_trigger_success(
        "response:\nstd_srvs.srv.SetBool_Response(success=True, message='ARMED')\n",
        returncode=0) is True
    assert parse_trigger_success(
        "response:\nstd_srvs.srv.SetBool_Response(success=False, message='DENIED')\n",
        returncode=0) is False
    with pytest.raises(RuntimeError, match="UNRECOGNIZED"):
        parse_trigger_success(
            "response:\nTrigger_Response(success=True, message='x')\n"
            "Trigger_Response(success=False, message='y')\n", returncode=0)
    assert parse_trigger_success(retained, returncode=1) is False


def test_humble_trigger_response_parser_is_strict_and_semantic():
    assert parse_trigger_success("response:\nsuccess: true\nmessage: accepted") is True
    assert parse_trigger_success("response:\nsuccess=True\nmessage: accepted") is True
    assert parse_trigger_success("response:\nsuccess: FALSE\nmessage: rejected") is False
    with pytest.raises(RuntimeError, match="UNRECOGNIZED"):
        parse_trigger_success("returncode: 0\n")


def test_pose_clamp_gate_requires_healthy_stationary_setup():
    good = ProgressAdmission(True, True, True, True, True, True, True, True, True, True)
    validate_pose_clamp_gate(good)
    with pytest.raises(RuntimeError):
        validate_pose_clamp_gate(ProgressAdmission(True, True, True, False, True, True, True, True, True, True))
    with pytest.raises(RuntimeError):
        validate_pose_clamp_gate(ProgressAdmission(True, True, True, True, True, True, True, True, True, True, recovery_delta=6))


def test_c_p01_threshold_progress_reset_and_competing_reason():
    with pytest.raises(RuntimeError):
        validate_no_progress_contract(no_progress_age_sec=19.999, measurable_progress=False, recovery_delta=0, collision_clear=True)
    validate_no_progress_contract(no_progress_age_sec=20.0, measurable_progress=False, recovery_delta=0, collision_clear=True)
    with pytest.raises(RuntimeError):
        validate_no_progress_contract(no_progress_age_sec=30.0, measurable_progress=True, recovery_delta=0, collision_clear=True)
    with pytest.raises(RuntimeError):
        validate_no_progress_contract(no_progress_age_sec=30.0, measurable_progress=False, recovery_delta=0, collision_clear=True, competing_reason="TF_STALE")
    with pytest.raises(RuntimeError):
        validate_no_progress_contract(no_progress_age_sec=30.0, measurable_progress=False, recovery_delta=6, collision_clear=True)


def test_c_p02_recovery_precedence_and_no_progress_requirement():
    with pytest.raises(RuntimeError):
        validate_recovery_exhaustion(recovery_delta=5, measurable_progress=False, collision_clear=True)
    with pytest.raises(RuntimeError):
        validate_recovery_exhaustion(recovery_delta=6, measurable_progress=True, collision_clear=True)
    validate_recovery_exhaustion(recovery_delta=6, measurable_progress=False, collision_clear=True)
    with pytest.raises(RuntimeError):
        validate_recovery_exhaustion(recovery_delta=6, measurable_progress=False, collision_clear=True, competing_reason="PERSISTENT_COLLISION_STOP")


def test_pose_clamp_rate_is_existing_bounded_rate():
    ticks = clamp_tick_times(0.0, 0.3)
    assert len(ticks) == 3
    assert ticks[1] == pytest.approx(0.1)


def test_command_pair_none_is_unavailable_not_zero():
    assert optional_float("none") is None
    branch = command_pair_branch({
        "command_pair_state": "STALE",
        "raw_command_age_sec": "none",
        "safe_command_age_sec": "none",
        "raw_safe_skew_sec": "none",
        "pair_liveness_age_sec": "0.272595",
    })
    assert branch["branch"] == "NON_SLIDING_PAIR_LIVENESS"
    assert branch["pair_liveness_age_sec"] == pytest.approx(0.272595)
    assert branch["raw_command_age_sec"] is None


def _pair(state="PENDING", phase="STREAM_ESTABLISHED", liveness=0.010,
          raw=0.016, safe=0.0, reason="", proof=True):
    return {"command_pair_state": state, "command_stream_phase": phase,
            "pair_liveness_age_sec": liveness, "raw_command_age_sec": raw,
            "safe_command_age_sec": safe, "raw_safe_skew_sec": None,
            "reason_code": reason, "pre_arm_causal_proof": proof}


def test_established_pending_command_pair_is_ready_with_liveness_proof():
    for age in (0.001, 0.010, 0.100, 0.249999):
        result = command_pair_ready(_pair(liveness=age))
        assert result["command_pair_current"] is True
        assert result["command_pair_branch"] == "ESTABLISHED_PENDING"


def test_valid_command_pair_uses_valid_branch():
    result = command_pair_ready(_pair(state="VALID", liveness=None, safe=0.010))
    assert result["command_pair_current"] is True
    assert result["command_pair_branch"] == "VALID"


@pytest.mark.parametrize("values,reason", [
    (_pair(phase="RAW_PENDING"), "COMMAND_STREAM_NOT_ESTABLISHED"),
    (_pair(liveness=None), "COMMAND_PAIR_PENDING_LIVENESS_UNAVAILABLE"),
    (_pair(liveness=0.250), "COMMAND_PAIR_PENDING_LIVENESS_EXPIRED"),
    (_pair(liveness=0.300), "COMMAND_PAIR_PENDING_LIVENESS_EXPIRED"),
    (_pair(raw=0.250), "RAW_COMMAND_NOT_CURRENT"),
    (_pair(safe=0.250), "SAFE_COMMAND_NOT_CURRENT"),
    (_pair(reason="COMMAND_PAIR_STALE"), "COMMAND_PAIR_STALE"),
    (_pair(state="UNKNOWN"), "COMMAND_PAIR_UNKNOWN_STATE"),
    (_pair(proof=False), "COMMAND_PAIR_PENDING_LIVENESS_UNAVAILABLE"),
])
def test_pending_command_pair_fail_closed(values, reason):
    result = command_pair_ready(values)
    assert result["command_pair_current"] is False
    assert result["rejection_reason"] == reason


def test_transient_valid_sample_is_not_required_when_pending_is_healthy():
    samples = [_pair(liveness=0.008), _pair(state="VALID", liveness=None),
               _pair(liveness=0.100), _pair(liveness=0.200)]
    assert all(command_pair_ready(sample)["command_pair_current"] for sample in samples)


def test_pending_liveness_boundary_is_not_sliding():
    assert command_pair_ready(_pair(liveness=0.249))["command_pair_current"] is True
    expired = command_pair_ready(_pair(liveness=0.250))
    assert expired["command_pair_current"] is False
    assert expired["rejection_reason"] == "COMMAND_PAIR_PENDING_LIVENESS_EXPIRED"


def test_production_diagnostic_strings_are_strictly_normalized():
    values = {
        "progress_supervisor_mode": "ACTIVE_POLICY",
        "progress_supervisor_event": "NO_PROGRESS_PENDING",
        "progress_classification": "NOT_MEASURABLE",
        "collision_classification": "CLEAR",
        "command_pair_state": "VALID",
        "recovery_delta": "0",
        "movement_intent": "true",
        "state": "NAVIGATING",
        "no_progress_age_sec": "none",
        "odometry_age_sec": "0.1", "tf_age_sec": "0.1", "feedback_age_sec": "0.1",
        "gate_age_sec": "0.1", "collision_monitor_age_sec": "0.1",
        "localization_age_sec": "0.1", "controller_age_sec": "0.1",
        "adapter_age_sec": "none", "raw_command_age_sec": "0.1",
        "safe_command_age_sec": "0.1",
    }
    row = normalize_progress_diagnostic(values, state={
        "mission_id": "m", "route_id": "r", "active_goal_uuid": "u",
        "movement_intent": True}, now_ns=10)
    assert row["recovery_delta"] == 0
    assert row["movement_intent"] is True
    assert row["no_progress_age_sec"] is None
    assert row["collision_clear"] is True
    values["movement_intent"] = "false"
    assert normalize_progress_diagnostic(values, state={"movement_intent": False}, now_ns=11)["movement_intent"] is False
    values["movement_intent"] = "invalid"
    with pytest.raises(RuntimeError):
        normalize_progress_diagnostic(values, state={}, now_ns=12)


def test_c_p02_count_six_waits_for_fresh_policy_event_without_duplicate_source_rows():
    admission = ProgressAdmission(True, True, True, True, True, True, True, True, True, True)
    rows = []
    for count in range(1, 7):
        rows.append({"sequence": count, "monotonic_ns": count * 100,
                     "goal_uuid": "u", "canonical_recovery_count": 0,
                     "shadow_recovery_count": count, "measurable_progress": False,
                     "actual_reason": ""})
    rows.append({"sequence": 7, "monotonic_ns": 700, "goal_uuid": "u",
                 "canonical_recovery_count": 0, "shadow_recovery_count": 6,
                 "measurable_progress": False,
                 "actual_reason": "RECOVERY_EXHAUSTED_NO_PROGRESS"})
    index = {"i": 0, "clock": 0}
    arms = []
    def now(): return index["clock"]
    def wait(deadline): index["clock"] = deadline
    def observe():
        value = rows[index["i"]]; index["i"] += 1; return value
    runner = __import__("parking_robot_bringup.phase4_p4e6c_progress_runner", fromlist=["LiveProgressRunner"]).LiveProgressRunner("C-P02", monotonic_ns=now)
    result = runner.run_cp02(evidence=admission, publish_initialpose=lambda: None,
                             arm_recovery=lambda: arms.append(1), observe_feedback=observe,
                             canonical_receipt_times_sec=[0, .1, .2, .3, .4, .5],
                             duration_sec=1.0, wait_until_ns=wait, marker_present=True)
    assert len(arms) == 1
    assert result["actual_reason"] == "RECOVERY_EXHAUSTED_NO_PROGRESS"
    assert [row["shadow_recovery_count"] for row in result["recovery_feedback"]] == [1, 2, 3, 4, 5, 6, 6]


def test_controller_dry_admission_and_marker_contract(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    ids = {"dependency": "approved"}
    assert controller.execute_dry(expected_identities=ids, actual_identities=ids)["attempt_consumed"] is False
    with pytest.raises(RuntimeError):
        controller.execute_dry(expected_identities={"missing": "approved"}, actual_identities={})
    assert controller.write_pre_runtime_failure("probe") == tmp_path / "CP01_ATTEMPT1_PRE_RUNTIME_FAILURE.json"
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").exists()


def test_progress_case_identity_and_no_live_activity():
    result = ProgressQualificationController("C-P02", Path("/tmp/p4e6c-dry"), supervisor_authority="DRY_ONLY").execute_dry(
        expected_identities={"x": "y"}, actual_identities={"x": "y"})
    assert result["marker"] == "CP02_ATTEMPT1_RUNTIME_STARTED"
    assert result["progress_injections"] == result["recovery_injections"] == 0
    assert result["route_mission_started"] == result["gate_armed"] == 0


def test_campaign_boot_ready_polls_until_observer_and_start_are_visible(tmp_path, monkeypatch):
    from parking_robot_bringup.phase4_p4e6c_live_driver import _FixedRuntimeBindings
    binding = object.__new__(_FixedRuntimeBindings)
    binding.driver = type("D", (), {"case": "C-P01", "attempt_identity": "BOOT_TEST"})()
    binding.output = tmp_path
    binding.process = None
    binding.boot_observations = []
    binding._durable_write = lambda path, value: path.write_text(json.dumps(value))
    class Process:
        def poll(self): return None
    process = Process()
    binding.process = process
    binding._owned_process_inventory = lambda: [
        {"pid": 10, "pgid": 20, "command": "mission_manager_node", "executable": "mission_manager_node"},
        {"pid": 11, "pgid": 20, "command": "phase4_p4e6c_progress_observer", "executable": "phase4_p4e6c_progress_observer"},
    ]
    heartbeat = {"n": 0}
    def evidence(filename):
        if filename == "physical_evidence.jsonl":
            heartbeat["n"] += 1
            return [{"transaction_id": "BOOT_TEST", "sequence": heartbeat["n"]}]
        if filename == "mission_policy_diagnostics.jsonl":
            return [{"transaction_id": "BOOT_TEST", "sequence": 1}]
        return []
    binding._current_transaction_rows = evidence
    calls = {"count": 0}
    def command(argv, timeout=5.0):
        if "node list" in " ".join(argv):
            calls["count"] += 1
            nodes = "/mission_manager\n/phase4_p4e6c_progress_observer\n" if calls["count"] >= 3 else "/mission_manager\n"
            return type("R", (), {"returncode": 0, "stdout": nodes, "stderr": ""})()
        return type("R", (), {"returncode": 0, "stdout": "/mission/start\n", "stderr": ""})()
    binding._command = command
    result = binding.campaign_boot_ready(process)
    assert result["mission_manager_owned"] is True
    assert result["observer_owned"] is True
    assert calls["count"] == 2
    assert (tmp_path / "boot_readiness.json").is_file()


def test_observer_uses_humble_feedback_transport_and_retains_identity():
    from nav2_msgs.action import NavigateToPose
    from parking_robot_bringup.phase4_p4e6c_progress_observer import ProgressEvidenceObserver
    observer = object.__new__(ProgressEvidenceObserver)
    observer.latest = {}
    retained = []
    observer.recorder = type("Recorder", (), {
        "record": lambda self, filename, row: retained.append((filename, row))
    })()
    message = NavigateToPose.Impl.FeedbackMessage()
    message.goal_id.uuid = [11] * 16
    message.feedback.navigation_time.sec = 4
    message.feedback.number_of_recoveries = 3
    message.feedback.distance_remaining = 1.25
    message.feedback.current_pose.pose.position.x = 2.5
    observer._feedback(message)
    assert retained[0][0] == "feedback_receipts.jsonl"
    assert retained[0][1]["goal_uuid"] == [11] * 16
    assert retained[0][1]["navigation_time_sec"] == 4.0
    assert retained[0][1]["number_of_recoveries"] == 3
    assert retained[0][1]["distance_remaining"] == 1.25
    assert retained[0][1]["current_pose"]["x"] == 2.5
    assert observer.latest["feedback_current"] is True


def test_observer_health_normalizes_ros_uint8_levels_without_crashing():
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from parking_robot_bringup.phase4_p4e6c_progress_observer import ProgressEvidenceObserver
    observer = object.__new__(ProgressEvidenceObserver)
    observer.latest = {}
    healthy = DiagnosticArray(); status = DiagnosticStatus(); status.level = b"\x00"
    healthy.status = [status]
    observer._health(healthy)
    assert observer.latest["health_current"] is True
    status.level = b"\x02"
    observer._health(healthy)
    assert observer.latest["health_current"] is False
    assert normalize_uint8(b"\x00", field="level") == 0
    assert normalize_uint8(bytearray([2]), field="level") == 2
    malformed = type("Malformed", (), {"status": [type("S", (), {"level": b"\x00\x01"})()]})()
    observer._health(malformed)
    assert observer.latest["health_current"] is False


def test_cleanup_allows_one_owned_graph_convergence_observation(monkeypatch, tmp_path):
    from parking_robot_bringup.phase4_p4e6c_live_driver import _FixedRuntimeBindings
    binding = object.__new__(_FixedRuntimeBindings)
    binding.process = None; binding.owned_pgid = 12345; binding.owned_launch_pid = 12346
    binding.signal_chronology = []; binding.output = tmp_path
    binding.recorder = type("Recorder", (), {"write_cleanup": lambda self, value: None})()
    class Process:
        def poll(self): return 0
        def wait(self, timeout=1.0): return 0
    process = Process(); binding.process = process
    monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_live_driver.os.getpgrp", lambda: 99999)
    monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_live_driver.os.killpg", lambda pgid, sig: (_ for _ in ()).throw(ProcessLookupError()) if sig == 0 else None)
    results = iter([
        {"scan1": [], "scan2": [], "graph": {"returncode": 0, "target_nodes": ["/planner_server"], "clean": False}, "pass": False},
        {"scan1": [], "scan2": [], "graph": {"returncode": 0, "target_nodes": [], "clean": True}, "pass": True},
        {"scan1": [], "scan2": [], "graph": {"returncode": 0, "target_nodes": [], "clean": True}, "pass": True},
    ])
    binding.authoritative_global_zero = lambda wait_sec=2.0: next(results)
    result = binding.cleanup(process)
    assert result["pass"] is True
    assert result["settle_observation_used"] is True


def test_cleanup_rejects_foreign_graph_or_process_residue(monkeypatch, tmp_path):
    from parking_robot_bringup.phase4_p4e6c_live_driver import _FixedRuntimeBindings
    def make(initial, second=None):
        binding = object.__new__(_FixedRuntimeBindings)
        binding.process = None; binding.owned_pgid = 12345; binding.owned_launch_pid = 12346
        binding.signal_chronology = []; binding.output = tmp_path
        binding.recorder = type("Recorder", (), {"write_cleanup": lambda self, value: None})()
        class Process:
            def poll(self): return 0
            def wait(self, timeout=1.0): return 0
        process = Process(); binding.process = process
        monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_live_driver.os.getpgrp", lambda: 99999)
        monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_live_driver.os.killpg", lambda pgid, sig: (_ for _ in ()).throw(ProcessLookupError()) if sig == 0 else None)
        values = iter([initial] + ([] if second is None else [second]))
        binding.authoritative_global_zero = lambda wait_sec=2.0: next(values)
        return binding, process
    foreign_graph = {"scan1": [], "scan2": [], "graph": {"returncode": 0, "target_nodes": ["/foreign_runner"], "clean": False}, "pass": False}
    binding, process = make(foreign_graph)
    assert binding.cleanup(process)["settle_observation_used"] is False
    owned_process = {"scan1": [{"pgid": 999}], "scan2": [], "graph": {"returncode": 0, "target_nodes": ["/planner_server"], "clean": False}, "pass": False}
    binding, process = make(owned_process)
    assert binding.cleanup(process)["settle_observation_used"] is False


def test_cleanup_signals_exact_owned_process_group_only(monkeypatch, tmp_path):
    from parking_robot_bringup.phase4_p4e6c_live_driver import _FixedRuntimeBindings
    binding = object.__new__(_FixedRuntimeBindings)
    binding.process = None
    binding.owned_pgid = 12345
    binding.owned_launch_pid = 12346
    binding.signal_chronology = []
    binding.output = tmp_path
    binding.recorder = type("Recorder", (), {"write_cleanup": lambda self, value: setattr(self, "result", value)})()
    class Process:
        pid = 12346
        def poll(self): return 0
        def wait(self, timeout=1.0): return 0
    process = Process()
    binding.process = process
    state = {"alive": True}
    signalled = []
    monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_live_driver.os.getpgrp", lambda: 99999)
    def killpg(pgid, sig):
        assert pgid == 12345
        if sig == 0:
            if not state["alive"]:
                raise ProcessLookupError
            return
        signalled.append((pgid, sig))
        state["alive"] = False
    monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_live_driver.os.killpg", killpg)
    binding.authoritative_global_zero = lambda: {
        "scan1": [], "scan2": [], "graph": {"returncode": 0, "clean": True}, "pass": True}
    result = binding.cleanup(process)
    assert result["pass"] is True
    assert signalled == [(12345, signal.SIGTERM)]
    assert result["owned_group_disappeared"] is True


def test_pre_marker_launch_exception_recovers_registered_ownership(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    owned = object()
    calls = []
    class Binding:
        process = None
        def launch(self):
            self.process = owned
            raise RuntimeError("OWNERSHIP_EVIDENCE_WRITE_FAILED")
        def cleanup(self, process):
            calls.append(process)
            return {"pass": True, "graph_clean": True}
    binding = Binding()
    hooks, _ = _live_hooks()
    hooks["launch"] = binding.launch
    hooks["cleanup"] = binding.cleanup
    with pytest.raises(RuntimeError, match="UNCONSUMED"):
        controller.execute_live(
            attempt_identity="C3B_OWNERSHIP_TEST",
            supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
            expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert calls == [owned]
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").exists()


def _live_hooks(*, preflight=None, run=None, cleanup=None):
    calls = []
    owned = object()
    hooks = {
        "authoritative_global_zero": lambda: preflight or {"scan1": [], "scan2": [],
            "graph": {"returncode": 0, "clean": True}, "pass": True},
        "environment_valid": lambda: calls.append("environment") is None,
        "launch": lambda: calls.append("launch") or owned,
        "processes_alive": lambda value: value is owned,
        "campaign_boot_ready": lambda value: calls.append("campaign"),
        "route_mission_start": lambda value: calls.append("route"),
        "active_identity_ready": lambda value: calls.append("identity"),
        "gate_ready": lambda value: calls.append("gate_ready"),
        "pre_arm_command_ready": lambda value: calls.append("pre_arm_command_ready"),
        "arm_gate": lambda value: calls.append("arm_gate"),
        "active_case_ready": lambda value: calls.append("active_case_ready"),
        "witness_ready": lambda value: calls.append("witness"),
        "runner_ready": lambda value: calls.append("runner"),
        "injection_ready": lambda value: calls.append("injection"),
        "run_after_marker": lambda value: (run() if run else {"actual_reason": "CONTROLLER_NO_PROGRESS"}),
        "terminal_adjudication": lambda value: {"pass": True},
        "physical_adjudication": lambda value: {"pass": True},
        "witness_seal": lambda runtime, terminal, physical, cleanup: {"pass": True, "missing_files": []},
        "cleanup": lambda value: cleanup if cleanup is not None else {"pass": True},
    }
    return hooks, calls


def test_live_controller_wrong_token_and_missing_identity_do_not_start_process(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    hooks, calls = _live_hooks()
    with pytest.raises(RuntimeError, match="AUTHORITY"):
        controller.execute_live(attempt_identity="C1", supervisor_authority="WRONG",
                                expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    with pytest.raises(RuntimeError, match="IDENTITY"):
        controller.execute_live(attempt_identity="C1", supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
                                expected_identities={"x": "y"}, actual_identities={}, hooks=hooks)
    assert calls == []
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").exists()


def test_live_controller_pre_marker_failure_is_unconsumed(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    failed = {"scan1": ["owned"], "scan2": [], "graph": {"returncode": 1, "clean": False}, "pass": False}
    hooks, calls = _live_hooks(preflight=failed)
    with pytest.raises(RuntimeError, match="UNCONSUMED"):
        controller.execute_live(attempt_identity="C1", supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
                                expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").exists()
    assert (tmp_path / "CP01_ATTEMPT1_PRE_RUNTIME_FAILURE.json").is_file()
    assert calls == []


def test_live_controller_marker_order_and_success_ledger(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    hooks, calls = _live_hooks()
    result = controller.execute_live(attempt_identity="CP01_TEST_001",
        supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
        expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert result["final_classification"] == "CONSUMED_PASS"
    assert (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").is_file()
    assert (tmp_path / "CP01_ATTEMPT1_FINAL_RESULT.json").is_file()
    assert calls.index("route") < calls.index("gate_ready")
    assert calls.index("arm_gate") < calls.index("witness")


def test_live_controller_post_marker_failure_is_consumed(tmp_path):
    controller = ProgressQualificationController("C-P02", tmp_path)
    hooks, _ = _live_hooks(run=lambda: (_ for _ in ()).throw(RuntimeError("runner failure")))
    result = controller.execute_live(attempt_identity="CP02_TEST_001",
        supervisor_authority="SUPERVISOR_AUTHORIZED_CP02_ATTEMPT1",
        expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert result["final_classification"] == "CONSUMED_FAILED"
    assert (tmp_path / "CP02_ATTEMPT1_RUNTIME_STARTED").is_file()
    assert (tmp_path / "CP02_ATTEMPT1_FINAL_RESULT.json").is_file()


def test_live_runner_rejects_clamp_before_marker_and_runs_cp02_sequence():
    admission = ProgressAdmission(True, True, True, True, True, True, True, True, True, True)
    runner = __import__("parking_robot_bringup.phase4_p4e6c_progress_runner", fromlist=["LiveProgressRunner"]).LiveProgressRunner("C-P02")
    with pytest.raises(RuntimeError):
        runner.run_pose_clamp(lambda: None, duration_sec=0.1, marker_present=False)
    seen = []
    feedback_index = {"value": 0}
    def feedback():
        feedback_index["value"] += 1
        count = feedback_index["value"]
        return {"shadow_recovery_count": count, "canonical_recovery_count": 0,
                "goal_uuid": "u", "measurable_progress": False,
                "actual_reason": ("RECOVERY_EXHAUSTED_NO_PROGRESS" if count == 6 else None)}
    result = runner.run_cp02(evidence=admission, publish_initialpose=lambda: seen.append("pose"),
        arm_recovery=lambda: seen.append("arm"),
        observe_feedback=feedback,
        duration_sec=0.6, marker_present=True,
        canonical_receipt_times_sec=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert result["expected_reason"] == "RECOVERY_EXHAUSTED_NO_PROGRESS"
    assert seen[:2] == ["pose", "arm"]


def test_live_observability_cadence_and_block_terminal_contract():
    good = {"odom_current": True, "tf_current": True, "feedback_current": True,
            "health_current": True, "command_pair_current": True,
            "collision_clear": True, "movement_intent": True,
            "action_terminal": False, "recovery_delta": 0}
    validate_live_observation(good, case="C-P01")
    with pytest.raises(RuntimeError):
        validate_live_observation({**good, "collision_clear": False}, case="C-P01")
    assert validate_feedback_cadence([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], remaining_window_sec=20.0)["required_steps"] == 6
    with pytest.raises(RuntimeError):
        validate_feedback_cadence([0.1, 0.2], remaining_window_sec=20.0)
    result = validate_block_terminal({
        "originating_reason": "RECOVERY_EXHAUSTED_NO_PROGRESS",
        "block_cancel_ack": "BLOCK_CANCEL_ACK_ACCEPTED", "cancel_request_count": 1,
        "canceling_count": 1, "canceled_count": 1, "duplicate_cancel": 0,
        "terminal_state": "BLOCKED", "mission_id": "m", "route_id": "r",
        "active_goal_uuid": "u",
    }, "RECOVERY_EXHAUSTED_NO_PROGRESS")
    assert result["pass"] is True


def test_progress_witness_ready_and_seal(tmp_path):
    witness = ProgressWitness(tmp_path, case="C-P01")
    ready = witness.mark_ready(identity={"uuid": "u"}, mechanism={"ready": True})
    assert ready["event"] == "PROGRESS_WITNESS_READY"
    for name in ("mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
                 "progress_events.jsonl", "navigate_action_status_events.jsonl",
                 "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
                 "physical_evidence.jsonl", "cleanup_result.json"):
        (tmp_path / name).write_text("{}\n")
    sealed = witness.seal(identity={"uuid": "u"}, outcome={"reason": "CONTROLLER_NO_PROGRESS",
        "terminal_evidence_complete": True, "physical_evidence_complete": True})
    assert sealed["terminal_evidence_complete"] is True
    assert (tmp_path / "PROGRESS_WITNESS_COMMITTED").is_file()


def test_progress_witness_missing_mandatory_file_fails(tmp_path):
    witness = ProgressWitness(tmp_path, case="C-P02")
    witness.mark_ready(identity={"uuid": "u"}, mechanism={"ready": True})
    for name in ("mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
                 "progress_events.jsonl", "navigate_action_status_events.jsonl",
                 "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
                 "physical_evidence.jsonl", "cleanup_result.json"):
        (tmp_path / name).write_text("{}\n")
    with pytest.raises(RuntimeError, match="WITNESS_SEAL_FAIL"):
        witness.seal(identity={"uuid": "u"}, outcome={
            "terminal_evidence_complete": True, "physical_evidence_complete": True})


@pytest.mark.parametrize("missing", [
    "mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
    "progress_events.jsonl", "navigate_action_status_events.jsonl",
    "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
    "physical_evidence.jsonl", "cleanup_result.json",
])
def test_progress_witness_each_common_file_is_mandatory(tmp_path, missing):
    witness = ProgressWitness(tmp_path, case="C-P01")
    witness.mark_ready(identity={"uuid": "u"}, mechanism={"ready": True})
    for name in witness.required_files if hasattr(witness, "required_files") else (
        "mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
        "progress_events.jsonl", "navigate_action_status_events.jsonl",
        "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
        "physical_evidence.jsonl", "cleanup_result.json"):
        if name != missing:
            (tmp_path / name).write_text("{}\n")
    with pytest.raises(RuntimeError, match="WITNESS_SEAL_FAIL"):
        witness.seal(identity={"uuid": "u"}, outcome={
            "terminal_evidence_complete": True, "physical_evidence_complete": True})


def test_sealed_live_driver_computes_fixed_identities_before_effects(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "231")
    for case in ("C-P02",):
        driver = SealedLiveDriver(case)
        report = driver.binding_report()
        assert report["identity_match"] is True
        assert report["caller_supplied_hooks"] is False
        assert "progress_live_driver" in report["actual"]
        driver.expected["progress_runner"] = "0" * 64
        with pytest.raises(RuntimeError, match="IDENTITY_MISMATCH"):
            driver.admit()


def test_sealed_live_driver_enters_controller_only_through_fixed_hooks(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "231")
    driver = SealedLiveDriver("C-P02")
    observed = {}
    original = driver._build_fixed_hooks

    def fixed_hooks():
        hooks = original()
        observed["hooks"] = set(hooks)
        return hooks

    def controller_entry(self, **kwargs):
        observed["controller"] = True
        observed["kwargs"] = kwargs
        return {"final_classification": "TEST_ONLY_NO_RUNTIME"}

    monkeypatch.setattr(driver, "_build_fixed_hooks", fixed_hooks)
    monkeypatch.setattr(ProgressQualificationController, "execute_live", controller_entry)
    monkeypatch.setenv("P4E6C_LIVE_AUTHORIZATION", "SUPERVISOR_C3_EXPLICIT")
    result = driver.execute_live(
        attempt_identity="C2B_TEST_ONLY",
        supervisor_authority=driver.authority["required_authority"])
    assert result["final_classification"] == "TEST_ONLY_NO_RUNTIME"
    assert observed["controller"] is True
    assert observed["hooks"] == {
        "authoritative_global_zero", "environment_valid", "launch", "processes_alive",
        "campaign_boot_ready", "route_mission_start", "active_identity_ready", "gate_ready",
        "pre_arm_command_ready", "arm_gate", "active_case_ready", "witness_ready", "runner_ready", "injection_ready", "run_after_marker",
        "terminal_adjudication", "physical_adjudication", "cleanup", "witness_seal",
    }
    assert "actual_identities" in observed["kwargs"]


def test_sealed_live_driver_effectful_admission_denies_latch_and_cross_token(monkeypatch):
    driver = SealedLiveDriver("C-P01")
    authority = driver.authority["required_authority"]
    with pytest.raises(RuntimeError, match="UNTIL_C3"):
        driver.execute_live(attempt_identity="x",
                            supervisor_authority=authority)
    monkeypatch.setenv("P4E6C_LIVE_AUTHORIZATION", "SUPERVISOR_C3_EXPLICIT")
    with pytest.raises(RuntimeError, match="AUTHORITY"):
        driver.execute_live(attempt_identity="x",
                            supervisor_authority="SUPERVISOR_AUTHORIZED_CP02_ATTEMPT1")


def test_durable_marker_commit_and_stimulation_order(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    hooks, calls = _live_hooks()
    result = controller.execute_live(
        attempt_identity="C2B_MARKER_TEST",
        supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
        expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert result["final_classification"] == "CONSUMED_PASS"
    assert (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").is_file()
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED.tmp").exists()
    assert calls.index("route") < calls.index("witness")


@pytest.mark.parametrize("failure", ["fsync", "replace"])
def test_marker_durability_failure_is_unconsumed(tmp_path, monkeypatch, failure):
    controller = ProgressQualificationController("C-P01", tmp_path)
    if failure == "fsync":
        monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_controller.os.fsync",
                            lambda fd: (_ for _ in ()).throw(OSError("fsync failure")))
    else:
        monkeypatch.setattr("parking_robot_bringup.phase4_p4e6c_controller.os.replace",
                            lambda src, dst: (_ for _ in ()).throw(OSError("rename failure")))
    hooks, _ = _live_hooks()
    with pytest.raises(RuntimeError, match="UNCONSUMED"):
        controller.execute_live(
            attempt_identity="C2B_DURABILITY_FAILURE",
            supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
            expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").exists()
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED.tmp").exists()


def test_full_case_compositions_are_distinct_and_c02_is_shadow_remapped():
    root = Path(__file__).parents[1] / "launch"
    cp01 = (root / "phase4_p4e6c_c_p01_full_campaign.launch.py").read_text()
    cp02 = (root / "phase4_p4e6c_c_p02_full_campaign.launch.py").read_text()
    assert "enable_health_runner" in cp01
    assert "phase4_p4e6c_recovery_feedback_relay" not in cp01
    assert "/phase4_qualification/p4e6c/navigate_to_pose_feedback" in cp02
    assert "phase4_p4e6c_recovery_feedback_relay" in cp02


def _readiness(case):
    value = {
        "mission_active": True, "policy_active": True, "gate_armed": True,
        "collision_clear": True, "health_current": True, "odom_current": True,
        "tf_current": True, "feedback_current": True,
        "command_pair_current": True, "movement_intent": True,
        "action_terminal": False, "recovery_delta": 0,
        "mission_id": "m-c2c", "route_id": "r-c2c", "active_goal_uuid": "u-c2c",
    }
    if case == "C-P02":
        value["recovery_relay_ready"] = True
    return value


@pytest.mark.parametrize("case", ["C-P02"])
def test_c2c_typed_readiness_and_fresh_reader(tmp_path, case):
    readiness = readiness_from_mapping(_readiness(case), case=case)
    assert readiness.admission.mission_active is True
    path = tmp_path / "progress_events.jsonl"
    path.write_text('{"sequence": 0, "monotonic_ns": 10}\n', encoding="utf-8")
    reader = FreshEvidenceReader(path)
    assert reader.next()["sequence"] == 0
    with pytest.raises(RuntimeError, match="STALE"):
        reader.next()
    path.write_text('{"sequence": 0, "monotonic_ns": 10}\n{"sequence": 1, "monotonic_ns": 20}\n', encoding="utf-8")
    assert reader.next()["sequence"] == 1


def test_c2c_readiness_missing_and_gate_fail_closed():
    value = _readiness("C-P01")
    value.pop("active_goal_uuid")
    with pytest.raises(RuntimeError, match="FIELD_MISSING"):
        readiness_from_mapping(value, case="C-P01")
    value = _readiness("C-P01")
    value["gate_armed"] = False
    with pytest.raises(RuntimeError, match="GATE_NOT_ARMED"):
        readiness_from_mapping(value, case="C-P01")


@pytest.mark.parametrize("case", ["C-P02"])
def test_c2c_end_to_end_driver_rehearsal_same_dataflow(monkeypatch, tmp_path, case):
    import parking_robot_bringup.phase4_p4e6c_live_driver as live_driver

    driver = SealedLiveDriver(case)
    monkeypatch.setattr(live_driver, "REPORTS", tmp_path)
    attempt_identity = f"C2C_{case}_REHEARSAL"
    output = tmp_path / f"P4E6C_{case.replace('-', '')}_{attempt_identity}"
    controller = ProgressQualificationController(case, output,
                                                 supervisor_authority=driver.authority["required_authority"],
                                                 attempt_number=int(driver.authority.get("attempt", 1)))
    recorder = ProgressEvidenceRecorder(output, case, controller._write_json)
    readiness = recorder.write_readiness(_readiness(case))
    witness = ProgressWitness(output, case=case)
    for name in ProgressWitness.COMMON_REQUIRED_FILES if hasattr(ProgressWitness, "COMMON_REQUIRED_FILES") else (
        "mission_policy_diagnostics.jsonl", "mission_state_events.jsonl",
        "progress_events.jsonl", "navigate_action_status_events.jsonl",
        "block_cancel_ack_events.jsonl", "pose_clamp_events.jsonl",
        "physical_evidence.jsonl", "cleanup_result.json"):
        recorder.record(name, {"event": "REHEARSAL", "mission_id": "m-c2c"}) if name.endswith(".jsonl") else None
    if case == "C-P02":
        recorder.record("recovery_feedback_events.jsonl", {"sequence": 1, "goal_uuid": "u-c2c", "shadow_recovery_count": 1})
    calls = []
    expected = "CONTROLLER_NO_PROGRESS" if case == "C-P01" else "RECOVERY_EXHAUSTED_NO_PROGRESS"
    runtime = {"case": case, "expected_reason": expected, "actual_reason": expected,
               "identity": {"mission_id": "m-c2c", "route_id": "r-c2c", "active_goal_uuid": "u-c2c"},
               "observations": [], "stimulation": {"rehearsal": True}}
    terminal = {"pass": True, "reason": expected}
    physical = {"pass": True, "translation_m": 0.0, "rotation_rad": 0.0}

    def seal(runtime_value, terminal_value, physical_value, cleanup_value):
        return witness.seal(identity=runtime_value["identity"], outcome={
            "terminal_evidence_complete": terminal_value["pass"],
            "physical_evidence_complete": physical_value["pass"],
            "cleanup": cleanup_value})

    def cleanup(value):
        result = {"pass": True, "graph_clean": True, "scan1": [], "scan2": []}
        recorder.write_cleanup(result)
        return result

    hooks = {
        "authoritative_global_zero": lambda: {"scan1": [], "scan2": [], "graph": {"returncode": 0, "clean": True}, "pass": True},
        "environment_valid": lambda: True,
        "launch": lambda: object(), "processes_alive": lambda value: True,
        "campaign_boot_ready": lambda value: calls.append("campaign"),
        "route_mission_start": lambda value: calls.append("route"),
            "active_identity_ready": lambda value: readiness,
            "gate_ready": lambda value: readiness,
            "pre_arm_command_ready": lambda value: calls.append("pre_arm_command_ready"),
            "arm_gate": lambda value: calls.append("gate"),
        "active_case_ready": lambda value: readiness,
        "witness_ready": lambda value: witness.mark_ready(
            identity={"mission_id": "m-c2c", "route_id": "r-c2c", "active_goal_uuid": "u-c2c"},
            mechanism={"ready": True}),
        "runner_ready": lambda value: {"ready": True},
        "injection_ready": lambda value: {"ready": True},
        "run_after_marker": lambda value: runtime,
        "terminal_adjudication": lambda value: terminal,
        "physical_adjudication": lambda value: physical,
        "cleanup": cleanup,
        "witness_seal": seal,
    }
    monkeypatch.setattr(driver, "_build_fixed_hooks", lambda: hooks)
    monkeypatch.setenv("P4E6C_LIVE_AUTHORIZATION", "SUPERVISOR_C3_EXPLICIT")
    result = driver.execute_live(
        attempt_identity=attempt_identity,
        supervisor_authority=driver.authority["required_authority"])
    assert result["final_classification"] == "CONSUMED_PASS"
    assert (output / controller.marker_name).is_file()
    assert result["witness"]["pass"] is True


@pytest.mark.parametrize("case", ["C-P01", "C-P02"])
def test_c2d_real_fixed_binding_boot_active_gate_order(monkeypatch, tmp_path, case):
    import parking_robot_bringup.phase4_p4e6c_live_driver as live_driver
    from parking_robot_bringup.phase4_p4e6c_live_driver import _FixedRuntimeBindings

    monkeypatch.setattr(live_driver, "REPORTS", tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "231")
    driver = SealedLiveDriver(case)
    driver.attempt_identity = f"C2D_{case}_ORDER"
    binding = _FixedRuntimeBindings(driver)
    binding.environment = {"ROS_LOCALHOST_ONLY": "1", "ROS_DOMAIN_ID": "231"}
    events = []

    class Process:
        def poll(self):
            return None

    process = Process()

    def command(argv, timeout=5.0):
        text = " ".join(argv)
        events.append(text)
        if "node list" in text:
            return type("R", (), {"returncode": 0, "stdout": "/mission_manager\n/phase4_p4e6c_progress_observer\n", "stderr": ""})()
        if "service list" in text:
            return type("R", (), {"returncode": 0, "stdout": "/mission/start\n/phase4_p4e6c_recovery_feedback_relay/arm_recovery_injection\n", "stderr": ""})()
        if "topic list" in text:
            return type("R", (), {"returncode": 0, "stdout": "/vehicle_cmd_safety/state\n", "stderr": ""})()
        return type("R", (), {"returncode": 0, "stdout": "response:\nsuccess: True\nmessage: OK\n", "stderr": ""})()

    binding.process = process
    binding.output = tmp_path / f"boot_{case.replace('-', '')}"
    binding.recorder.output = binding.output
    binding._owned_process_inventory = lambda: [
        {"pid": 101, "pgid": 101, "command": "mission_manager_node", "executable": "mission_manager_node"},
        {"pid": 102, "pgid": 101, "command": "phase4_p4e6c_progress_observer", "executable": "phase4_p4e6c_progress_observer"},
    ]
    binding.recorder.record("physical_evidence.jsonl", {
        "transaction_id": driver.attempt_identity, "monotonic_ns": 10,
    })
    binding.recorder.record("physical_evidence.jsonl", {
        "transaction_id": driver.attempt_identity, "monotonic_ns": 11,
    })
    if case == "C-P02":
        binding.recorder.record("recovery_feedback_events.jsonl", {
            "transaction_id": driver.attempt_identity, "relay_state": "READY_FORWARD",
            "injection_count": 0, "canonical_recovery_count": 0,
            "shadow_recovery_count": 0, "sequence": 0, "monotonic_ns": 12,
        })
    heartbeat_reads = [0]
    def current_rows(filename):
        if filename == "physical_evidence.jsonl":
            seq = min(heartbeat_reads[0], 1)
            heartbeat_reads[0] += 1
            stamp = time.monotonic_ns()
            return [
                {"transaction_id": driver.attempt_identity, "sequence": seq,
                 "monotonic_ns": stamp, "source": "raw_command",
                 "movement_intent": True},
                {"transaction_id": driver.attempt_identity, "sequence": seq,
                 "monotonic_ns": stamp, "source": "canonical_command",
                 "movement_intent": True},
            ]
        return binding._original_current_rows(filename)
    binding._original_current_rows = binding._current_transaction_rows
    binding._current_transaction_rows = current_rows
    monkeypatch.setattr(binding, "_command", command)
    monkeypatch.setattr(binding, "prepare_service_clients", lambda: None)
    binding.service_endpoint_ready = True
    binding.service_clients = {
        "/mission/start": object(), "/vehicle_cmd_safety/arm": object(),
    }
    monkeypatch.setattr(binding, "_call_fixed_service",
                        lambda name, request: type("Response", (), {"success": True, "message": "OK"})())
    # This ordering test supplies a deterministic lifecycle seam for the
    # external ROS RouteMission exchange; the production binding uses the
    # real publisher/RECEIVED observer path.
    monkeypatch.setattr(binding, "publish_route_and_wait_received", lambda: None)
    assert binding.processes_alive(process)
    binding.campaign_boot_ready(process)
    assert binding.readiness is None
    binding.route_mission_start(process)
    binding.recorder.record("mission_state_events.jsonl", {
        "transaction_id": driver.attempt_identity, "observation_monotonic_ns": binding.route_start_ns + 1,
        "mission_id": "m-c2d", "route_id": "r-c2d", "active_goal_uuid": "u-c2d",
        "state": "NAVIGATING", "reason_code": ""})
    values = _readiness(case)
    values.update({"transaction_id": driver.attempt_identity,
                   "observation_monotonic_ns": binding.route_start_ns + 1})
    values["gate_armed"] = False
    binding.recorder.write_readiness(values, require_active_case=False)
    binding.active_identity_ready(process)
    binding.gate_ready(process)
    assert binding.readiness is None
    assert binding.active_identity == {"mission_id": "m-c2d", "route_id": "r-c2d", "active_goal_uuid": "u-c2d"}
    binding.pre_arm_readiness = {"pass": True}
    binding.arm_gate(process)
    binding.recorder.record("gate_state_events.jsonl", {
        "transaction_id": driver.attempt_identity,
        "observation_monotonic_ns": binding.gate_arm_ns + 1,
        "state": "ARMED", "active_goal_uuid": "u-c2d",
        "mission_id": "m-c2d", "route_id": "r-c2d",
        "gate_armed": True, "fault_latched": False,
        "safe_twist_age_sec": 0.05,
        "controller_valid": True, "localization_valid": True,
        "collision_monitor_valid": True,
        "controller_publisher_count": 1, "localization_publisher_count": 1,
        "collision_valid_publisher_count": 1, "safe_input_publisher_count": 1,
        "last_arm_request": True,
    })
    binding.recorder.record("mission_policy_diagnostics.jsonl", {
            "transaction_id": driver.attempt_identity,
            "observation_monotonic_ns": binding.gate_arm_ns + 2,
            "values": {"state": "NAVIGATING", "mission_id": "m-c2d",
                       "route_id": "r-c2d", "active_goal_uuid": "u-c2d",
                       "progress_policy_activation_state": "ACTIVE",
                       "progress_policy_activation_latched": True,
                       "command_stream_phase": "STREAM_ESTABLISHED",
                       "command_pair_state": "VALID",
                       "raw_command_age_sec": 0.05, "safe_command_age_sec": 0.05,
                       "raw_safe_skew_sec": 0.01,
                       "collision_classification": "CLEAR",
                       "localization_age_sec": 0.05, "controller_age_sec": 0.05,
                       "odometry_age_sec": 0.05, "tf_age_sec": 0.05,
                       "feedback_age_sec": 0.05,
                       "recovery_delta": 0,
                       "progress_supervisor_event": ""},
    })
    values["gate_armed"] = True
    values["observation_monotonic_ns"] = binding.gate_arm_ns + 1
    binding.recorder.write_readiness(values)
    if case == "C-P02":
        binding.recorder.record("recovery_feedback_events.jsonl", {
            "transaction_id": driver.attempt_identity, "relay_state": "READY_FORWARD",
            "injection_count": 0, "canonical_recovery_count": 0,
            "shadow_recovery_count": 0, "sequence": 1,
            "monotonic_ns": binding.gate_arm_ns + 2})
    binding.active_case_ready(process)
    if case == "C-P02":
        binding.recorder.record("recovery_feedback_events.jsonl", {
            "transaction_id": driver.attempt_identity, "relay_state": "READY_FORWARD",
            "injection_count": 0, "canonical_recovery_count": 0,
            "shadow_recovery_count": 0, "sequence": 1, "monotonic_ns": binding.gate_arm_ns + 2})
    binding.witness_ready(process)
    binding.runner_ready(process)
    monkeypatch.setattr(binding, "_pose_clamp_ready", lambda: {
        "readiness": "POSE_CLAMP_STIMULATOR_READY", "pass": True})
    binding.injection_ready(process)
    assert not any("ros2 service call" in event for event in events)
    assert not (tmp_path / f"P4E6C_{case.replace('-', '')}_ATTEMPT1_001" /
                "CP01_ATTEMPT1_RUNTIME_STARTED").exists()


def test_realtime_deadline_scheduler_records_actual_pacing():
    clock = {"ns": 0}
    waits = []
    def now():
        return clock["ns"]
    def wait(deadline):
        waits.append(deadline)
        clock["ns"] = max(clock["ns"], deadline)
    runner = __import__("parking_robot_bringup.phase4_p4e6c_progress_runner", fromlist=["LiveProgressRunner"]).LiveProgressRunner("C-P01", monotonic_ns=now)
    result = runner.run_pose_clamp(lambda: None, duration_sec=0.2, marker_present=True, wait_until_ns=wait)
    events = result["events"]
    assert len(events) == 3
    assert [event["scheduled_monotonic_ns"] for event in events] == [0, 100000000, 200000000]
    assert [event["actual_publish_monotonic_ns"] for event in events] == [0, 100000000, 200000000]
    assert events[1]["actual_interval_sec"] == pytest.approx(0.1)
    assert waits == [0, 100000000, 200000000]


def test_realtime_scheduler_smoke_is_not_a_burst():
    runner = __import__("parking_robot_bringup.phase4_p4e6c_progress_runner", fromlist=["LiveProgressRunner"]).LiveProgressRunner("C-P01")
    result = runner.run_pose_clamp(lambda: None, duration_sec=0.3, marker_present=True)
    assert result["count"] == 4
    assert result["events"][-1]["cumulative_elapsed_sec"] >= 0.25


def test_cp01_observes_during_clamp_until_real_event():
    clock = {"ns": 0}
    def wait(deadline):
        clock["ns"] = deadline
    runner = __import__("parking_robot_bringup.phase4_p4e6c_progress_runner", fromlist=["LiveProgressRunner"]).LiveProgressRunner("C-P01", monotonic_ns=lambda: clock["ns"])
    admission = ProgressAdmission(True, True, True, True, True, True, True, True, True, True)
    rows = [{"odom_current": True, "tf_current": True, "feedback_current": True,
             "health_current": True, "command_pair_current": True, "collision_clear": True,
             "movement_intent": True, "action_terminal": False, "recovery_delta": 0},
            {"odom_current": True, "tf_current": True, "feedback_current": True,
             "health_current": True, "command_pair_current": True, "collision_clear": True,
             "movement_intent": True, "action_terminal": False, "recovery_delta": 0,
             "progress_event": "CONTROLLER_NO_PROGRESS", "no_progress_age_sec": 20.0}]
    result = runner.run_cp01(evidence=admission, publish_initialpose=lambda: None,
        observe=lambda: rows.pop(0), duration_sec=2.0, marker_present=True, wait_until_ns=wait)
    assert len(result["observations"]) == 2
    assert result["clamp"]["events"][-1]["actual_publish_monotonic_ns"] == 100000000


def test_cp02_arm_and_recovery_happen_while_clamp_is_active():
    clock = {"ns": 0}
    active = {"value": False}
    arms = []
    def wait(deadline):
        clock["ns"] = deadline
    runner = __import__("parking_robot_bringup.phase4_p4e6c_progress_runner", fromlist=["LiveProgressRunner"]).LiveProgressRunner("C-P02", monotonic_ns=lambda: clock["ns"])
    admission = ProgressAdmission(True, True, True, True, True, True, True, True, True, True)
    index = {"value": 0}
    def publish():
        active["value"] = True
    def arm():
        assert active["value"] is True
        arms.append(clock["ns"])
    def feedback():
        index["value"] += 1
        n = index["value"]
        assert active["value"] is True
        return {"shadow_recovery_count": n, "canonical_recovery_count": 0, "goal_uuid": "u",
                "measurable_progress": False,
                "actual_reason": "RECOVERY_EXHAUSTED_NO_PROGRESS" if n == 6 else None}
    result = runner.run_cp02(evidence=admission, publish_initialpose=publish, arm_recovery=arm,
        observe_feedback=feedback, duration_sec=2.0, marker_present=True,
        canonical_receipt_times_sec=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], wait_until_ns=wait)
    assert len(arms) == 1
    assert len(result["recovery_feedback"]) == 6
    assert result["clamp"]["events"][-1]["actual_publish_monotonic_ns"] == 500000000


def test_cp02_topology_launch_is_shadow_only():
    launch = Path(__file__).parents[1] / "launch" / "phase4_p4e6c_c_p02_topology.launch.py"
    text = launch.read_text(encoding="utf-8")
    assert "/navigate_to_pose/_action/feedback" in text
    assert "/phase4_qualification/p4e6c/navigate_to_pose_feedback" in text
    assert "action_client" not in text
    assert "action_server" not in text


def test_live_controller_injection_not_ready_and_cleanup_failure_are_fail_closed(tmp_path):
    controller = ProgressQualificationController("C-P01", tmp_path)
    hooks, _ = _live_hooks()
    hooks["injection_ready"] = lambda value: (_ for _ in ()).throw(RuntimeError("mechanism not ready"))
    with pytest.raises(RuntimeError, match="UNCONSUMED"):
        controller.execute_live(attempt_identity="CP01_TEST_002",
            supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
            expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert not (tmp_path / "CP01_ATTEMPT1_RUNTIME_STARTED").exists()

    second = ProgressQualificationController("C-P01", tmp_path / "second")
    hooks, _ = _live_hooks(cleanup={"pass": False, "graph_clean": False})
    result = second.execute_live(attempt_identity="CP01_TEST_003",
        supervisor_authority="SUPERVISOR_AUTHORIZED_CP01_ATTEMPT1",
        expected_identities={"x": "y"}, actual_identities={"x": "y"}, hooks=hooks)
    assert result["final_classification"] == "CONSUMED_FAILED"


def test_recovery_relay_real_ros_microqualification():
    rclpy = pytest.importorskip("rclpy")
    from nav2_msgs.action import NavigateToPose
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import serialize_message
    from parking_robot_bringup.phase4_p4e6c_recovery_feedback_relay import (
        CANONICAL_FEEDBACK, SHADOW_FEEDBACK, RecoveryFeedbackRelay,
    )

    os.environ.setdefault("ROS_DOMAIN_ID", "231")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    relay = RecoveryFeedbackRelay(feedback_timeout_sec=2.0)
    publisher_node = rclpy.create_node("p4e6c_micro_publisher")
    collector_node = rclpy.create_node("p4e6c_micro_collector")
    publisher = publisher_node.create_publisher(NavigateToPose.Impl.FeedbackMessage, CANONICAL_FEEDBACK, 10)
    received = []
    subscription = collector_node.create_subscription(
        NavigateToPose.Impl.FeedbackMessage, SHADOW_FEEDBACK, received.append, 10)
    executor = SingleThreadedExecutor()
    for node in (relay, publisher_node, collector_node):
        executor.add_node(node)

    def spin_until(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            executor.spin_once(timeout_sec=0.05)
        return predicate()

    def message(recoveries=0):
        value = NavigateToPose.Impl.FeedbackMessage()
        value.goal_id.uuid = [7] * 16
        value.feedback.navigation_time.sec = 12
        value.feedback.estimated_time_remaining.sec = 34
        value.feedback.distance_remaining = 1.25
        value.feedback.current_pose.pose.position.x = 2.5
        value.feedback.number_of_recoveries = recoveries
        return value

    try:
        baseline = message(0)
        publisher.publish(baseline)
        assert spin_until(lambda: len(received) >= 1)
        assert serialize_message(received[-1]) == serialize_message(baseline)
        relay.arm_recovery_injection()
        for expected_step in range(1, 7):
            relay.advance_recovery_step()
            publisher.publish(message(0))
            assert spin_until(lambda: len(received) >= 1 + expected_step)
            relay.acknowledge_source_step()
        publisher.publish(message(0))
        assert spin_until(lambda: len(received) >= 8)
        counts = [int(item.feedback.number_of_recoveries) for item in received[1:]]
        assert counts == [1, 2, 3, 4, 5, 6, 6]
        for item in received[1:]:
            canonical = copy.deepcopy(item)
            canonical.feedback.number_of_recoveries = 0
            assert serialize_message(canonical) == serialize_message(message(0))
        assert relay.injection_count == 1
        assert relay.sequence_complete is True
        assert relay.state == "INJECTED_HOLDING"
        assert not hasattr(relay, "action_client")
        assert not hasattr(relay, "action_server")
    finally:
        executor.shutdown()
        for node in (relay, publisher_node, collector_node):
            node.destroy_node()
        rclpy.shutdown()


def _fake_progress_zero_binding(tmp_path, monkeypatch, scans, graph_result):
    from parking_robot_bringup.phase4_p4e6c_live_driver import _FixedRuntimeBindings
    binding = object.__new__(_FixedRuntimeBindings)
    binding.driver = type("Driver", (), {"case": "C-P01", "attempt_identity": "C3F_TEST"})()
    binding.environment = {"ROS_DOMAIN_ID": "237", "ROS_LOCALHOST_ONLY": "1"}
    binding.output = tmp_path
    binding._durable_write = lambda path, value: path.write_text(json.dumps(value), encoding="utf-8")
    monkeypatch.setattr(
        "parking_robot_bringup.phase4_p4e6b_live_evidence.global_target_scan",
        lambda: scans.pop(0),
    )
    monkeypatch.setattr(
        "parking_robot_bringup.phase4_p4e6c_live_driver.subprocess.run",
        lambda *args, **kwargs: graph_result,
    )
    return binding


def test_progress_global_zero_uses_daemonless_graph_and_retains_pass(tmp_path, monkeypatch):
    result = type("Result", (), {"returncode": 0, "stdout": "/rviz2\n", "stderr": ""})()
    binding = _fake_progress_zero_binding(tmp_path, monkeypatch, [[], []], result)
    value = binding.progress_global_zero_preflight(wait_sec=0.0)
    assert value["pass"] is True
    assert value["graph"]["command"] == ["ros2", "node", "list", "--no-daemon"]
    assert (tmp_path / "global_zero_preflight.json").is_file()


@pytest.mark.parametrize("failure", ["scan1", "scan2", "graph", "query"])
def test_progress_global_zero_retains_each_failure_branch(tmp_path, monkeypatch, failure):
    scans = [[], []]
    graph = type("Result", (), {"returncode": 0, "stdout": "/rviz2\n", "stderr": ""})()
    if failure == "scan1":
        scans[0] = [{"pid": 11, "ppid": 1, "pgid": 11, "sid": 11, "command": "planner_server"}]
    elif failure == "scan2":
        scans[1] = [{"pid": 12, "ppid": 1, "pgid": 12, "sid": 12, "command": "controller_server"}]
    elif failure == "graph":
        graph.stdout = "/planner_server\n"
    else:
        graph.returncode = 1
        graph.stderr = "query failed"
    binding = _fake_progress_zero_binding(tmp_path, monkeypatch, scans, graph)
    value = binding.progress_global_zero_preflight(wait_sec=0.0)
    assert value["pass"] is False
    assert (tmp_path / "global_zero_preflight.json").is_file()
    retained = json.loads((tmp_path / "global_zero_preflight.json").read_text())
    assert retained["scan1"] == value["scan1"]
    assert retained["scan2"] == value["scan2"]
