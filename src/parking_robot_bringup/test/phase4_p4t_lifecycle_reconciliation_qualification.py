"""B2T deterministic qualification for lost lifecycle transition responses.

This is qualification-only infrastructure.  The fake node is not a production
component and no matrix, route, mission, Gate, or physical interface is used.
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeLifecycle(Node):
    def __init__(self, mode, out):
        super().__init__("fake_collision_monitor")
        self.mode = mode
        self.out = Path(out)
        self.state = State.PRIMARY_STATE_UNCONFIGURED
        self.configure_count = 0
        self.activate_count = 0
        self.get_state_count = 0
        self.events = []
        self.cb = ReentrantCallbackGroup()
        self.create_service(GetState, "/fake_collision_monitor/get_state", self.get_state, callback_group=self.cb)
        self.create_service(ChangeState, "/fake_collision_monitor/change_state", self.change_state, callback_group=self.cb)
        self.create_timer(0.1, self.save, callback_group=self.cb)
        self.save()

    def save(self):
        self.out.write_text(json.dumps({
            "mode": self.mode, "state": int(self.state),
            "configure_count": self.configure_count,
            "activate_count": self.activate_count,
            "get_state_count": self.get_state_count,
            "events": self.events,
        }, sort_keys=True) + "\n")

    def get_state(self, request, response):
        started = time.monotonic_ns()
        self.get_state_count += 1
        delayed = self.mode in ("lost_configure", "lost_activate") and self.get_state_count == 1
        if self.mode == "lost_get_state" and self.get_state_count in (1, 2):
            delayed = True
        if delayed:
            time.sleep(1.5)
        response.current_state.id = int(self.state)
        response.current_state.label = {
            State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
            State.PRIMARY_STATE_INACTIVE: "inactive",
            State.PRIMARY_STATE_ACTIVE: "active",
        }.get(self.state, "unknown")
        self.events.append({"kind": "get_state", "start_ns": started, "end_ns": time.monotonic_ns(), "delayed": delayed})
        return response

    def change_state(self, request, response):
        started = time.monotonic_ns()
        delayed = False
        if request.transition.id == 1:  # CONFIGURE
            self.configure_count += 1
            if self.mode == "true_configure_failure":
                response.success = False
                self.events.append({"kind": "configure", "start_ns": started, "end_ns": time.monotonic_ns(), "delayed": False, "success": False})
                return response
            self.state = State.PRIMARY_STATE_INACTIVE
            if self.mode == "lost_configure" and self.configure_count == 1:
                delayed = True
                time.sleep(1.5)
        elif request.transition.id == 3:  # ACTIVATE
            self.activate_count += 1
            self.state = State.PRIMARY_STATE_ACTIVE
            if self.mode == "lost_activate" and self.activate_count == 1:
                delayed = True
                time.sleep(1.5)
        response.success = True
        self.events.append({"kind": "configure" if request.transition.id == 1 else "activate", "start_ns": started, "end_ns": time.monotonic_ns(), "delayed": delayed, "success": True})
        return response


def run_case(mode, root, domain):
    case = Path(root) / mode
    case.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    rclpy.init(args=[])
    fake = FakeLifecycle(mode, case / "fake_state.json")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    manager = subprocess.Popen([
        "ros2", "run", "phase4_lifecycle_manager_reconciled", "lifecycle_manager",
        "--ros-args", "-p", "node_names:=[fake_collision_monitor]",
        "-p", "autostart:=true", "-p", "bond_timeout:=0.0",
        "-p", "service_call_timeout_sec:=1.0", "-p", "transition_retry_interval_ms:=100",
        "-p", "transition_retries:=3",
    ], env=env, stdout=(case / "manager.stdout").open("w"), stderr=(case / "manager.stderr").open("w"), start_new_session=True)
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline and manager.poll() is None:
        executor.spin_once(timeout_sec=0.05)
        if fake.state == State.PRIMARY_STATE_ACTIVE:
            time.sleep(0.2)
            break
        if mode == "true_configure_failure" and fake.configure_count >= 3:
            time.sleep(0.2)
            break
    result = manager.poll()
    if result is None:
        os.killpg(manager.pid, signal.SIGTERM)
        manager.wait(timeout=5)
        result = manager.returncode
    fake.save()
    state = json.loads((case / "fake_state.json").read_text())
    log = (case / "manager.stdout").read_text(errors="replace")
    passed = False
    if mode in ("lost_configure", "lost_activate", "lost_get_state"):
        passed = state["state"] == State.PRIMARY_STATE_ACTIVE and state["configure_count"] == 1 and state["activate_count"] == 1
    elif mode == "true_configure_failure":
        passed = state["state"] == State.PRIMARY_STATE_UNCONFIGURED and state["configure_count"] == 3 and state["activate_count"] == 0
    (case / "result.json").write_text(json.dumps({"mode": mode, "manager_returncode": result, "state": state, "passed": passed}, indent=2, sort_keys=True) + "\n")
    executor.shutdown(); fake.destroy_node(); rclpy.shutdown()
    return passed


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--output", required=True); ap.add_argument("--domain", type=int, default=240)
    args = ap.parse_args()
    modes = ["lost_configure", "lost_activate", "lost_get_state", "true_configure_failure"]
    results = {m: run_case(m, args.output, args.domain + i) for i, m in enumerate(modes)}
    Path(args.output, "summary.json").write_text(json.dumps({"results": results, "pass": all(results.values())}, indent=2, sort_keys=True) + "\n")
    raise SystemExit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
