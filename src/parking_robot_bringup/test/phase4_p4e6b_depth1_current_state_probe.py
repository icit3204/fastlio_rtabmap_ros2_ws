"""One owned actual-matrix depth-1 diagnostics current-state observation.

This qualification probe is deliberately not a runner and never publishes a
route, starts a mission, arms a Gate, or changes health parameters.
"""
import argparse, json, os, signal, subprocess, time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool


TARGET = "vehicle_cmd_safety/collision_monitor_validity_monitor"


def stop_group(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--hold-sec", type=float, default=5.0)
    parser.add_argument("--best-effort", action="store_true")
    args = parser.parse_args()
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1")
    # rclpy reads these when its context is initialized; the launch child also
    # receives the same explicit environment below.
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1")
    rclpy.init(); node = rclpy.create_node("p4e6b_depth1_current_state_probe")
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    rows = {"all": [], "target": [], "bool": []}
    def diagnostics(msg):
        receipt = time.monotonic_ns()
        header = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        rows["all"].append([receipt, header, [status.name for status in msg.status]])
        for status in msg.status:
            if status.name == TARGET:
                values = {value.key: value.value for value in status.values}
                rows["target"].append([receipt, header, values])
    def collision(msg):
        rows["bool"].append([time.monotonic_ns(), bool(msg.data)])
    node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, qos)
    node.create_subscription(Bool, "/system/collision_monitor_valid", collision, qos)
    matrix = subprocess.Popen(["ros2", "launch", "parking_robot_bringup",
                               "phase4_p4e6b_health_matrix.launch.py",
                               "enable_health_runner:=false"], env=env,
                              start_new_session=True,
                              stdout=(out / "matrix.stdout").open("w"),
                              stderr=(out / "matrix.stderr").open("w"))
    valid_start = None
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if rows["target"] and rows["bool"]:
                values = rows["target"][-1][2]
                if (values.get("state") == "VALID" and values.get("reason_code") == "VALID"
                        and rows["bool"][-1][1]):
                    valid_start = time.monotonic_ns(); break
        if valid_start is None:
            raise RuntimeError("no canonical VALID/true")
        until = time.monotonic() + args.hold_sec
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic_ns()
        target = [row for row in rows["target"] if row[0] >= valid_start]
        intervals = [b[0]-a[0] for a,b in zip(target, target[1:])]
        header_intervals = [b[1]-a[1] for a,b in zip(target, target[1:])]
        now_ros = node.get_clock().now().nanoseconds
        latest_age = now_ros - target[-1][1] if target else None
        info = node.get_publishers_info_by_topic("/diagnostics")
        writers = [{"node": ep.node_namespace + ep.node_name,
                    "gid": bytes(ep.endpoint_gid).hex(),
                    "reliability": str(ep.qos_profile.reliability),
                    "durability": str(ep.qos_profile.durability)} for ep in info]
        summary = {"pass": bool(target), "domain": args.domain, "best_effort":args.best_effort, "valid_start_ns": valid_start,
                   "hold_sec": args.hold_sec, "target_count": len(target),
                   "all_array_count": len(rows["all"]), "bool_count": len(rows["bool"]),
                   "max_target_receipt_gap_ns": max(intervals, default=0),
                   "max_target_header_gap_ns": max(header_intervals, default=0),
                   "latest_target_message_age_ns": latest_age,
                   "latest_target_receipt_age_ns": now-target[-1][0] if target else None,
                   "target_receipt_gap_exceeds_250ms_count": sum(gap >= 250_000_000 for gap in intervals),
                   "diagnostics_publishers": writers,
                   "route_mission": 0, "mission_start": 0, "uuid": 0,
                   "gate_arm": 0, "health_injection": 0}
        (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    finally:
        if not (out / "summary.json").exists():
            (out / "partial.json").write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n")
        stop_group(matrix); node.destroy_node(); rclpy.shutdown()
        subprocess.run(["ros2", "daemon", "stop"], env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)


if __name__ == "__main__":
    main()
