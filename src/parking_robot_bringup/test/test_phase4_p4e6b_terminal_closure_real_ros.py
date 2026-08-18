import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from parking_robot_bringup.phase4_p4e6b_terminal_closure import adjudicate_terminal_v2, physical_closure_from_rows
from parking_robot_bringup.phase4_p4e6b_live_evidence import load_genuine_witness_outcome, post_zero_angular_zero


ROOT = Path(__file__).resolve().parent
WITNESS = ROOT / "phase4_p4e6b_terminal_closure_witness.py"
DRIVER = ROOT / "phase4_p4e6b_terminal_closure_fake_driver.py"


def run_case(tmp_path, inverted):
    env = dict(os.environ, ROS_DOMAIN_ID="229", ROS_LOCALHOST_ONLY="1")
    out = tmp_path / ("inverted" if inverted else "normal"); out.mkdir()
    witness = subprocess.Popen([sys.executable, str(WITNESS), "--output-dir", str(out), "--reason", "FEEDBACK_STALE"], env=env)
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not (out / "TERMINAL_WITNESS_READY").exists(): time.sleep(.02)
        assert (out / "TERMINAL_WITNESS_READY").exists()
        driver = subprocess.run([sys.executable, str(DRIVER)] + (["--inverted"] if inverted else []), env=env, timeout=8)
        assert driver.returncode == 0
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not (out / "TERMINAL_WITNESS_OUTCOME_COMMITTED").exists(): time.sleep(.02)
        assert (out / "TERMINAL_WITNESS_OUTCOME_COMMITTED").exists()
        outcome = load_genuine_witness_outcome(out)
        committed_hashes = dict(outcome["marker"]["file_hashes"])
        time.sleep(.5)
        for name, digest in committed_hashes.items():
            assert __import__("hashlib").sha256((out / name).read_bytes()).hexdigest() == digest
        assert witness.poll() is not None
        result = adjudicate_terminal_v2(origin_ns=outcome["terminal"]["origin_ns"], reason="FEEDBACK_STALE", mission_id="fake-mission", route_id="fake-route", waypoint=0, uuid="00112233445566778899aabbccddeeff", states=outcome["states"], statuses=outcome["statuses"], drain_start_ns=outcome["drain_start_ns"], drain_end_ns=outcome["drain_end_ns"], source_result_canceled=True)
        assert result["pass"]
        assert outcome["actual_drain_sec"] >= .250
        assert outcome["marker"]["terminal_evidence_complete"]
        assert outcome["marker"]["physical_evidence_complete"]
        physical = physical_closure_from_rows(vehicle_rows=outcome["vehicle_rows"], applied_rows=outcome["applied_rows"],
                                              odometry_rows=outcome["odometry_rows"],
                                              terminal_ns=outcome["terminal"]["origin_ns"],
                                              drain_end_ns=outcome["drain_end_ns"])
        assert physical["pass"]
        assert post_zero_angular_zero(outcome, physical["anchor_ns"])
        return result
    finally:
        if witness.poll() is None:
            witness.send_signal(signal.SIGTERM); witness.wait(timeout=3)


def test_real_ros_normal_order(tmp_path):
    assert run_case(tmp_path, False)["pass"]


def test_real_ros_inverted_receipt_order(tmp_path):
    assert run_case(tmp_path, True)["pass"]


def test_interrupted_drain_does_not_commit(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID="230", ROS_LOCALHOST_ONLY="1")
    out = tmp_path / "interrupted"; out.mkdir()
    witness = subprocess.Popen([sys.executable, str(WITNESS), "--output-dir", str(out), "--reason", "FEEDBACK_STALE"], env=env)
    driver = None
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not (out / "TERMINAL_WITNESS_READY").exists(): time.sleep(.02)
        assert (out / "TERMINAL_WITNESS_READY").exists()
        driver = subprocess.Popen([sys.executable, str(DRIVER), "--inverted"], env=env)
        deadline = time.monotonic() + 8
        failed_seen = False
        while time.monotonic() < deadline:
            phase = out / "terminal_witness_events.jsonl"
            if phase.exists() and "FAILED_RECEIVED" in phase.read_text():
                failed_seen = True
                break
            time.sleep(.005)
        assert failed_seen
        witness.send_signal(signal.SIGTERM)
        witness.wait(timeout=3)
        assert not (out / "TERMINAL_WITNESS_OUTCOME_COMMITTED").exists()
    finally:
        if driver is not None and driver.poll() is None:
            driver.send_signal(signal.SIGTERM); driver.wait(timeout=3)
        if witness.poll() is None:
            witness.send_signal(signal.SIGTERM); witness.wait(timeout=3)
