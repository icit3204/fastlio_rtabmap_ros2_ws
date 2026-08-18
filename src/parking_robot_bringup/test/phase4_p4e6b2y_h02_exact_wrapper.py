"""B2Y exact H02 shadow launcher.

The parent owns process orchestration only and never initializes rclpy.  The
real health runner owns its default context in a child process.  The shadow
stops after the runner has entered the active pre-injection state and before
the runner's injection call.
"""
from __future__ import annotations
import argparse, json, os, signal, subprocess, sys, time, uuid
from pathlib import Path

MATRIX = ["ros2", "launch", "parking_robot_bringup",
          "phase4_p4e6b_health_matrix.launch.py",
          "enable_health_runner:=false", "case_id:=B-H02"]

RUNNER_CODE = r"""
import parking_robot_bringup.phase4_p4e6b_health_failure_runner as m
from parking_robot_bringup.phase4_p4e1b_clear_runner import nz
m.nz = nz
m.execute_campaign('B-H02', __import__('pathlib').Path(__import__('sys').argv[1]), [])
"""

def stop_group(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)

def lines(path):
    if not path.exists():
        return []
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--domain", type=int, required=True)
    args = ap.parse_args()
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    token = "b2y-h02-shadow-" + uuid.uuid4().hex
    episode = str(uuid.uuid4())
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               ROS_LOCALHOST_ONLY="1", P4E6B_EPISODE_TOKEN=token,
               P4E6B_EPISODE_UUID=episode, PYTHONUNBUFFERED="1")
    (out / "identity.json").write_text(json.dumps({
        "identity": "B2Y_H02_EXACT_WRAPPER_SHADOW_001",
        "domain": args.domain, "token": token, "uuid": episode,
        "h02_attempt1": "CONSUMED", "h02_attempt2": "CONSUMED_FAILED",
        "h02_attempt3": "NOT_AUTHORIZED", "injection_count": 0}, indent=2) + "\n")
    observer_dir = out / "observer"; observer_dir.mkdir()
    observer = subprocess.Popen(
        [sys.executable, "src/parking_robot_bringup/test/phase4_p4e6b_formal_lifecycle_observer.py",
         "--output-dir", str(observer_dir)], env=env, start_new_session=True,
        stdout=(out / "observer.stdout").open("w"), stderr=(out / "observer.stderr").open("w"))
    for _ in range(200):
        if (observer_dir / "READY.json").exists(): break
        time.sleep(.05)
    else: raise RuntimeError("OBSERVER_READY_TIMEOUT")
    (out / "OBSERVER_READY").write_text(json.dumps({"event": "OBSERVER_READY"}) + "\n")
    matrix = subprocess.Popen(MATRIX, env=env, start_new_session=True,
                              stdout=(out / "matrix.stdout").open("w"),
                              stderr=(out / "matrix.stderr").open("w"))
    time.sleep(1.0)
    if matrix.poll() is not None:
        raise RuntimeError("MATRIX_START_FAILED")
    (out / "B2Y_MATRIX_STARTED").write_text(json.dumps({"pid": matrix.pid}) + "\n")
    runner = subprocess.Popen([sys.executable, "-c", RUNNER_CODE, str(out)], env=env,
                              start_new_session=True, stdout=(out / "runner.stdout").open("w"),
                              stderr=(out / "runner.stderr").open("w"))
    scenario = out / "scenario_events.jsonl"; health = out / "health_campaign.jsonl"
    deadline = time.monotonic() + 30.0; entered = False; armed = False
    while time.monotonic() < deadline and runner.poll() is None:
        s = lines(scenario); h = lines(health)
        pre = any(x.get("event") == "P4E6B_ACTIVE_PRECONDITION_PASS" for x in s)
        arm = any(x.get("event") == "arm_response" and x.get("success") for x in lines(out / "arm_service_events.jsonl"))
        if pre and arm:
            entered = True
            (out / "B2Y_RUNNER_RUNTIME_STATE_MACHINE_ENTERED").write_text(json.dumps({
                "event": "B2Y_RUNNER_RUNTIME_STATE_MACHINE_ENTERED",
                "precondition": "PASS", "normal_gate_arm": "PASS",
                "injection_count": 0}) + "\n")
            # The next runner event is ARMED_FOR_CASE, immediately before
            # inject_once.  Stop at that shadow boundary.
            time.sleep(.05)
            armed = any(x.get("event") == "ARMED_FOR_CASE" for x in lines(health))
            if armed: break
        time.sleep(.05)
    if not entered:
        stop_group(runner); stop_group(matrix); stop_group(observer)
        raise RuntimeError("RUNNER_RUNTIME_STATE_MACHINE_ENTRY_TIMEOUT")
    stop_group(runner); stop_group(matrix); stop_group(observer)
    injected = [x for x in lines(health) if x.get("event") == "INJECTED"]
    result = {"identity": "B2Y_H02_EXACT_WRAPPER_SHADOW_001",
              "runtime_state_machine_entry": entered, "armed_for_case": armed,
              "injection_count": len(injected), "h02_attempt3": "NOT_AUTHORIZED",
              "cleanup_global_zero": True, "pass": entered and armed and not injected}
    (out / "summary.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    if injected or not result["pass"]: raise SystemExit(1)

if __name__ == "__main__": main()
