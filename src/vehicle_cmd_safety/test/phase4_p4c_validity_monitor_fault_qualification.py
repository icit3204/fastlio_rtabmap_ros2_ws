"""Qualification-only real-ROS fault/recovery harness for the production monitor."""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import State, TransitionEvent
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


def valid_scan():
    msg = LaserScan()
    msg.header.frame_id = "base_footprint"
    msg.angle_min = -1.0
    msg.angle_max = 1.0
    msg.angle_increment = 0.1
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = [1.0] * 21
    return msg


class FakeCollisionMonitor:
    def __init__(self, mode, output):
        self.node = rclpy.create_node("b2r_fake_collision_monitor")
        self.mode = mode
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.state_calls = 0
        self.params_calls = 0
        group = ReentrantCallbackGroup()
        self.state_service = None if mode == "permanent_unavailable" else self.node.create_service(GetState, "/fake_collision_monitor/get_state", self.state_cb, callback_group=group)
        self.params_service = None if mode == "permanent_unavailable" else self.node.create_service(GetParameters, "/fake_collision_monitor/get_parameters", self.params_cb, callback_group=group)
        self.pub = self.node.create_publisher(LaserScan, "/phase4/synthetic_scan", 10)
        self.timer = self.node.create_timer(0.05, self.publish)

    def publish(self):
        self.pub.publish(valid_scan())

    def state_cb(self, request, response):
        self.state_calls += 1
        if self.mode == "delayed_first" and self.state_calls == 1:
            time.sleep(0.50)
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = "active"
        return response

    def params_cb(self, request, response):
        self.params_calls += 1
        if self.mode == "delayed_first" and self.params_calls == 1:
            time.sleep(0.50)
        values = []
        values.append(ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=["scan"]))
        values.append(ParameterValue(type=ParameterType.PARAMETER_STRING, string_value="scan"))
        values.append(ParameterValue(type=ParameterType.PARAMETER_STRING, string_value="/phase4/synthetic_scan"))
        response.values = values
        return response

    def close(self):
        self.output.write_text(json.dumps({"mode": self.mode, "state_calls": self.state_calls, "params_calls": self.params_calls}, indent=2) + "\n")
        self.node.destroy_node()


def fake_main(args):
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1")
    rclpy.init(args=[])
    fake = FakeCollisionMonitor(args.mode, args.output)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake.node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        fake.close()
        executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            rclpy.shutdown()


class Observer:
    def __init__(self):
        self.node = rclpy.create_node("b2r_fault_observer")
        self.bools = []
        self.diags = []
        self.node.create_subscription(Bool, "/system/collision_monitor_valid", self.bool_cb, 20)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.diag_cb, 20)

    def bool_cb(self, msg):
        self.bools.append((time.monotonic_ns(), bool(msg.data)))

    def diag_cb(self, msg):
        for status in msg.status:
            if status.name != "vehicle_cmd_safety/collision_monitor_validity_monitor":
                continue
            values = {x.key: x.value for x in status.values}
            self.diags.append({"monotonic_ns": time.monotonic_ns(), "message": status.message, "values": values})

    def close(self):
        self.node.destroy_node()


def kill_group(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=6)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=4)


def qualification_main(args):
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1")
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1")
    fake = None
    monitor = None
    rclpy.init(args=[])
    observer = Observer()
    try:
        fake_cmd = [sys.executable, str(Path(__file__).resolve()), "--fake", "--mode", args.mode, "--domain", str(args.domain), "--output", str(out / "fake_summary.json")]
        fake = subprocess.Popen(fake_cmd, env=env, start_new_session=True, stdout=(out / "fake.stdout").open("w"), stderr=(out / "fake.stderr").open("w"))
        time.sleep(0.4)
        monitor = subprocess.Popen(["ros2", "run", "vehicle_cmd_safety", "collision_monitor_validity_monitor", "--ros-args", "-r", "/collision_monitor/get_state:=/fake_collision_monitor/get_state", "-r", "/collision_monitor/get_parameters:=/fake_collision_monitor/get_parameters"], env=env, start_new_session=True, stdout=(out / "monitor.stdout").open("w"), stderr=(out / "monitor.stderr").open("w"))
        deadline = time.monotonic() + (5.0 if args.mode == "delayed_first" else 2.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(observer.node, timeout_sec=0.02)
            time.sleep(0.01)
        statuses = [x["values"].get("state_query_status") for x in observer.diags]
        param_statuses = [x["values"].get("params_query_status") for x in observer.diags]
        result = {
            "mode": args.mode,
            "domain": args.domain,
            "bool_true_observed": any(x[1] for x in observer.bools),
            "bool_false_observed": any(not x[1] for x in observer.bools),
            "state_statuses": sorted(set(x for x in statuses if x)),
            "params_statuses": sorted(set(x for x in param_statuses if x)),
            "diagnostic_count": len(observer.diags),
            "bool_count": len(observer.bools),
            "diagnostics_tail": observer.diags[-10:],
            "recovery_required": args.mode == "delayed_first",
            "pass": (any(x[1] for x in observer.bools) and "TIMED_OUT" in statuses and "VALID_RESPONSE" in statuses and "TIMED_OUT" in param_statuses and "VALID_RESPONSE" in param_statuses) if args.mode == "delayed_first" else (not any(x[1] for x in observer.bools)),
        }
        (out / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        return 0 if result["pass"] else 3
    finally:
        kill_group(monitor)
        kill_group(fake)
        observer.close()
        rclpy.shutdown()
        subprocess.run(["ros2", "daemon", "stop"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--mode", choices=["delayed_first", "permanent_unavailable"], default="delayed_first")
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.fake:
        return fake_main(args)
    return qualification_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
