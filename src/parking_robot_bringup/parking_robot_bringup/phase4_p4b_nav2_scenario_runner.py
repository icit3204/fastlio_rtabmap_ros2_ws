"""Phase 4 P4-B fresh-source Nav2 scenario runner and evidence monitor."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Any

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


FORBIDDEN_TOPICS = (
    "/vehicle_cmd_safe",
    "/system/collision_monitor_valid",
    "/wheelchair_control_command_mock",
    "/wheelchair_control_command",
    "/wheelchair_control_command_raw",
)

PROCESS_PATTERNS = (
    "controller_server",
    "collision_monitor",
    "phase2_fake_base",
    "bt_navigator",
    "planner_server",
    "map_server",
    "lifecycle_manager",
    "phase4_p4b_synthetic_obstacles",
    "ros2 bag",
    "nav2_velocity_smoother",
    "pure_pursuit",
    "laser_command_safety_filter",
    "wheelchair_controller_node",
    "guarded_vehicle_cmd_gate",
    "wheelchair_cmd_adapter_node",
    "UdpSender",
    "SocketCAN",
)


def normalize_yaw(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * normalize_yaw(yaw))
    q.w = math.cos(0.5 * normalize_yaw(yaw))
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    return normalize_yaw(
        math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
    )


def finite_twist(msg: Twist) -> bool:
    return all(
        math.isfinite(float(value))
        for value in (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
    )


def nonzero_twist(msg: Twist) -> bool:
    return any(
        abs(float(value)) > 1.0e-6
        for value in (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
    )


def twist_values(msg: Twist) -> tuple[float, float, float, float, float, float]:
    return (
        float(msg.linear.x),
        float(msg.linear.y),
        float(msg.linear.z),
        float(msg.angular.x),
        float(msg.angular.y),
        float(msg.angular.z),
    )


def pose_from_odom(msg: Odometry) -> tuple[float, float, float]:
    pose = msg.pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def yaw_error(a: float, b: float) -> float:
    return abs(normalize_yaw(a - b))


class Phase4P4BNav2ScenarioRunner(Node):
    """Run one P4-B scenario and persist command, odometry, graph and action evidence."""

    def __init__(self, out_dir: Path, scenario: str) -> None:
        super().__init__("phase4_p4b_nav2_scenario_runner")
        self.out_dir = out_dir
        self.scenario = scenario
        self.start_pose = (5.425, -53.725, 0.0)
        self.goal_pose = (8.425, -53.725, 0.0)
        self.goal_timeout_sec = 180.0
        self.mode_events: list[dict[str, Any]] = []
        self.action_events: list[dict[str, Any]] = []
        self.raw_count = 0
        self.safe_count = 0
        self.odom_count = 0
        self.tf_count = 0
        self.path_count = 0
        self.last_raw: Twist | None = None
        self.last_safe: Twist | None = None
        self.last_odom: Odometry | None = None
        self.path_pose_count = 0
        self.path_length_m = 0.0
        self.first_nonzero_raw_ns: int | None = None
        self.stop_start_ns: int | None = None
        self.stop_end_ns: int | None = None
        self.stop_start_pose: tuple[float, float, float] | None = None
        self.stop_end_pose: tuple[float, float, float] | None = None
        self._goal_done = False
        self._goal_status: int | None = None
        self._goal_status_name: str | None = None
        self._goal_accepted = False
        self._goal_handle = None
        self._result_future = None
        self._last_heartbeat = 0.0

        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.raw_file = (self.out_dir / "raw_cmd_timeline.tsv").open("w", encoding="utf-8")
        self.safe_file = (self.out_dir / "safe_cmd_timeline.tsv").open("w", encoding="utf-8")
        self.odom_file = (self.out_dir / "odom_timeline.tsv").open("w", encoding="utf-8")
        self.tf_file = (self.out_dir / "tf_timeline.tsv").open("w", encoding="utf-8")
        self.heartbeat_file = (self.out_dir / "monitor_heartbeat.tsv").open("w", encoding="utf-8")
        self.provenance_file = (self.out_dir / "publisher_provenance.tsv").open("w", encoding="utf-8")
        self.process_file = (self.out_dir / "process_authority.tsv").open("w", encoding="utf-8")
        self.resource_file = (self.out_dir / "resource_timeline.tsv").open("w", encoding="utf-8")
        self._write_headers()

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Twist, "/cmd_vel_nav_raw", self._raw_cb, 100)
        self.create_subscription(Twist, "/cmd_vel_nav_safe", self._safe_cb, 100)
        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 100)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, 100)
        self.create_subscription(NavPath, "/plan", self._path_cb, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._status_cb, 10)
        self.param_client = self.create_client(
            SetParameters, "/phase4_p4b_synthetic_obstacles/set_parameters"
        )
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.timer = self.create_timer(0.05, self._timer_cb)

    def _write_headers(self) -> None:
        cmd_header = (
            "monotonic_ns\tros_ns\ttopic\tlinear_x\tlinear_y\tlinear_z\tangular_x\tangular_y\tangular_z"
            "\tfinite\tnonzero\tpublisher_gid\tpublisher_node\n"
        )
        self.raw_file.write(cmd_header)
        self.safe_file.write(cmd_header)
        self.odom_file.write(
            "monotonic_ns\tros_ns\tx\ty\tyaw\tlinear_x\tangular_z\tfinite\n"
        )
        self.tf_file.write("monotonic_ns\tros_ns\tframe_id\tchild_frame_id\n")
        self.heartbeat_file.write(
            "monotonic_ns\tros_ns\texecutor_alive\traw_count\tsafe_count\todom_count\ttf_count"
            "\tcollision_monitor_alive\tfake_base_alive\trosbag_alive\tcurrent_obstacle_mode\n"
        )
        self.provenance_file.write("monotonic_ns\ttopic\tpublisher_count\tgid\tnode_name\tnode_namespace\ttopic_type\n")
        self.process_file.write("monotonic_ns\tpid\tcommand\n")
        self.resource_file.write("monotonic_ns\tloadavg_1\tloadavg_5\tloadavg_15\n")

    def close_files(self) -> None:
        for handle in (
            self.raw_file,
            self.safe_file,
            self.odom_file,
            self.tf_file,
            self.heartbeat_file,
            self.provenance_file,
            self.process_file,
            self.resource_file,
        ):
            handle.close()

    def _publisher_identity(self, topic: str) -> tuple[str, str]:
        infos = self.get_publishers_info_by_topic(topic)
        if len(infos) != 1:
            return "", ""
        info = infos[0]
        gid = getattr(info, "endpoint_gid", None)
        gid_text = "".join(f"{byte:02x}" for byte in gid) if gid is not None else ""
        return gid_text, f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")

    def _cmd_cb(self, topic: str, msg: Twist, handle: Any) -> None:
        gid, node = self._publisher_identity(topic)
        values = twist_values(msg)
        row = [
            time.monotonic_ns(),
            self.get_clock().now().nanoseconds,
            topic,
            *values,
            finite_twist(msg),
            nonzero_twist(msg),
            gid,
            node,
        ]
        handle.write("\t".join(str(value) for value in row) + "\n")
        handle.flush()

    def _raw_cb(self, msg: Twist) -> None:
        self.raw_count += 1
        self.last_raw = msg
        if nonzero_twist(msg) and self.first_nonzero_raw_ns is None:
            self.first_nonzero_raw_ns = time.monotonic_ns()
        self._cmd_cb("/cmd_vel_nav_raw", msg, self.raw_file)

    def _safe_cb(self, msg: Twist) -> None:
        self.safe_count += 1
        self.last_safe = msg
        self._cmd_cb("/cmd_vel_nav_safe", msg, self.safe_file)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_count += 1
        self.last_odom = msg
        pose = pose_from_odom(msg)
        finite = all(math.isfinite(value) for value in pose)
        self.odom_file.write(
            f"{time.monotonic_ns()}\t{self.get_clock().now().nanoseconds}\t{pose[0]}\t{pose[1]}\t"
            f"{pose[2]}\t{float(msg.twist.twist.linear.x)}\t{float(msg.twist.twist.angular.z)}\t{finite}\n"
        )

    def _tf_cb(self, msg: TFMessage) -> None:
        self.tf_count += len(msg.transforms)
        now = time.monotonic_ns()
        ros_now = self.get_clock().now().nanoseconds
        for transform in msg.transforms:
            self.tf_file.write(f"{now}\t{ros_now}\t{transform.header.frame_id}\t{transform.child_frame_id}\n")

    def _path_cb(self, msg: NavPath) -> None:
        self.path_count += 1
        poses = [
            (float(pose.pose.position.x), float(pose.pose.position.y), yaw_from_quaternion(pose.pose.orientation))
            for pose in msg.poses
        ]
        self.path_pose_count = len(poses)
        self.path_length_m = sum(distance(a, b) for a, b in zip(poses, poses[1:]))

    def _status_cb(self, msg: GoalStatusArray) -> None:
        self.action_events.append(
            {
                "monotonic_ns": time.monotonic_ns(),
                "ros_ns": self.get_clock().now().nanoseconds,
                "status_count": len(msg.status_list),
                "statuses": [int(status.status) for status in msg.status_list],
            }
        )

    def _timer_cb(self) -> None:
        now = time.monotonic()
        if now - self._last_heartbeat < 0.1:
            return
        self._last_heartbeat = now
        processes = self._matching_processes()
        collision_alive = any("collision_monitor" in cmd for _, cmd in processes)
        fake_alive = any("phase2_fake_base" in cmd for _, cmd in processes)
        bag_alive = any("ros2 bag" in cmd or "record" in cmd and "rosbag" in cmd for _, cmd in processes)
        current_mode = self.mode_events[-1]["mode"] if self.mode_events else "UNKNOWN"
        mono_ns = time.monotonic_ns()
        ros_ns = self.get_clock().now().nanoseconds
        self.heartbeat_file.write(
            f"{mono_ns}\t{ros_ns}\ttrue\t{self.raw_count}\t{self.safe_count}\t{self.odom_count}\t{self.tf_count}"
            f"\t{collision_alive}\t{fake_alive}\t{bag_alive}\t{current_mode}\n"
        )
        for topic in ("/cmd_vel_nav_raw", "/cmd_vel_nav_safe", *FORBIDDEN_TOPICS):
            infos = self.get_publishers_info_by_topic(topic)
            topic_types = dict(self.get_topic_names_and_types())
            if not infos:
                self.provenance_file.write(f"{mono_ns}\t{topic}\t0\t\t\t\t\n")
            for info in infos:
                gid = getattr(info, "endpoint_gid", None)
                gid_text = "".join(f"{byte:02x}" for byte in gid) if gid is not None else ""
                types = ",".join(sorted(topic_types.get(topic, [])))
                self.provenance_file.write(
                    f"{mono_ns}\t{topic}\t{len(infos)}\t{gid_text}\t{info.node_name}\t{info.node_namespace}\t{types}\n"
                )
        for pid, cmd in processes:
            self.process_file.write(f"{mono_ns}\t{pid}\t{cmd}\n")
        try:
            load1, load5, load15 = os.getloadavg()
        except OSError:
            load1 = load5 = load15 = 0.0
        self.resource_file.write(f"{mono_ns}\t{load1}\t{load5}\t{load15}\n")
        for handle in (self.heartbeat_file, self.provenance_file, self.process_file, self.resource_file):
            handle.flush()

    def _matching_processes(self) -> list[tuple[int, str]]:
        result = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            check=False,
            text=True,
            capture_output=True,
        )
        rows = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            pid_text, _, cmd = stripped.partition(" ")
            if any(pattern in cmd for pattern in PROCESS_PATTERNS):
                rows.append((int(pid_text), cmd))
        return rows

    def set_mode(self, mode: str) -> None:
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("synthetic obstacle parameter service unavailable")
        request = SetParameters.Request()
        request.parameters = [Parameter("mode", Parameter.Type.STRING, mode).to_parameter_msg()]
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"timeout setting obstacle mode {mode}")
        result = future.result().results[0]
        if not result.successful:
            raise RuntimeError(f"failed setting obstacle mode {mode}: {result.reason}")
        event = {
            "monotonic_ns": time.monotonic_ns(),
            "ros_ns": self.get_clock().now().nanoseconds,
            "mode": mode,
        }
        self.mode_events.append(event)

    def publish_initialpose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.start_pose[0]
        msg.pose.pose.position.y = self.start_pose[1]
        msg.pose.pose.orientation = quaternion_from_yaw(self.start_pose[2])
        for _ in range(10):
            self.initialpose_pub.publish(msg)
            self.spin_for(0.1)
        self.action_events.append({"monotonic_ns": time.monotonic_ns(), "event": "initialpose_published"})

    def send_goal(self) -> None:
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("NavigateToPose action server unavailable")
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_pose[0]
        goal.pose.pose.position.y = self.goal_pose[1]
        goal.pose.pose.orientation = quaternion_from_yaw(self.goal_pose[2])
        self.action_events.append({"monotonic_ns": time.monotonic_ns(), "event": "goal_sent"})
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("NavigateToPose goal response timeout")
        self._goal_handle = future.result()
        self._goal_accepted = bool(self._goal_handle.accepted)
        self.action_events.append(
            {"monotonic_ns": time.monotonic_ns(), "event": "goal_response", "accepted": self._goal_accepted}
        )
        if not self._goal_accepted:
            raise RuntimeError("NavigateToPose goal rejected")
        self._result_future = self._goal_handle.get_result_async()

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._process_result()
            rclpy.spin_once(self, timeout_sec=0.02)

    def _process_result(self) -> None:
        if self._result_future is None or self._goal_done or not self._result_future.done():
            return
        result = self._result_future.result()
        self._goal_done = True
        self._goal_status = int(result.status)
        names = {
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
        }
        self._goal_status_name = names.get(self._goal_status, str(self._goal_status))
        self.action_events.append(
            {
                "monotonic_ns": time.monotonic_ns(),
                "event": "goal_result",
                "status": self._goal_status,
                "status_name": self._goal_status_name,
            }
        )

    def wait_for_motion(self, min_seconds: float = 2.0, timeout_sec: float = 45.0) -> None:
        start = time.monotonic()
        first = None
        while time.monotonic() - start < timeout_sec:
            moving = self.last_raw is not None and nonzero_twist(self.last_raw)
            if moving and first is None:
                first = time.monotonic()
            if moving and first is not None and time.monotonic() - first >= min_seconds:
                return
            self.spin_for(0.05)
        raise RuntimeError("timed out waiting for sustained nonzero raw command")

    def wait_goal_done(self) -> None:
        start = time.monotonic()
        while time.monotonic() - start < self.goal_timeout_sec and not self._goal_done:
            self.spin_for(0.05)
        self._process_result()
        if not self._goal_done:
            raise RuntimeError("NavigateToPose result timeout")

    def write_jsonl(self) -> None:
        with (self.out_dir / "obstacle_mode_events.jsonl").open("w", encoding="utf-8") as handle:
            for event in self.mode_events:
                handle.write(json.dumps(event, sort_keys=True) + "\n")
        with (self.out_dir / "action_events.jsonl").open("w", encoding="utf-8") as handle:
            for event in self.action_events:
                handle.write(json.dumps(event, sort_keys=True) + "\n")

    def terminal_metrics(self, error: str | None = None) -> dict[str, Any]:
        final_pose = pose_from_odom(self.last_odom) if self.last_odom is not None else None
        final_xy_error = None if final_pose is None else distance(final_pose, self.goal_pose)
        final_yaw_error = None if final_pose is None else yaw_error(final_pose[2], self.goal_pose[2])
        stop_translation = None
        stop_yaw = None
        if self.stop_start_pose is not None and self.stop_end_pose is not None:
            stop_translation = distance(self.stop_start_pose, self.stop_end_pose)
            stop_yaw = yaw_error(self.stop_start_pose[2], self.stop_end_pose[2])
        metrics = {
            "scenario": self.scenario,
            "pass": error is None and self._goal_status_name == "SUCCEEDED",
            "error": error,
            "goal_accepted": self._goal_accepted,
            "goal_status": self._goal_status,
            "goal_status_name": self._goal_status_name,
            "raw_count": self.raw_count,
            "safe_count": self.safe_count,
            "odom_count": self.odom_count,
            "tf_count": self.tf_count,
            "path_count": self.path_count,
            "path_pose_count": self.path_pose_count,
            "path_length_m": self.path_length_m,
            "final_pose": None
            if final_pose is None
            else {"x": final_pose[0], "y": final_pose[1], "yaw": final_pose[2]},
            "final_xy_error_m": final_xy_error,
            "final_yaw_error_rad": final_yaw_error,
            "stop_window_translation_m": stop_translation,
            "stop_window_yaw_rad": stop_yaw,
        }
        with (self.out_dir / "terminal_metrics.json").open("w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
        return metrics


def run(out_dir: Path, scenario: str) -> dict[str, Any]:
    rclpy.init()
    node = Phase4P4BNav2ScenarioRunner(out_dir, scenario)
    error = None
    try:
        node.spin_for(5.0)
        node.set_mode("CLEAR")
        node.publish_initialpose()
        node.spin_for(2.0)
        node.send_goal()
        if scenario == "clear":
            node.wait_goal_done()
        elif scenario == "slowdown":
            node.wait_for_motion(2.0)
            node.set_mode("SLOW")
            node.spin_for(2.6)
            node.set_mode("CLEAR")
            node.wait_goal_done()
        elif scenario == "stop":
            node.wait_for_motion(2.0)
            node.set_mode("STOP")
            node.stop_start_ns = time.monotonic_ns()
            if node.last_odom is not None:
                node.stop_start_pose = pose_from_odom(node.last_odom)
            node.spin_for(2.6)
            node.stop_end_ns = time.monotonic_ns()
            if node.last_odom is not None:
                node.stop_end_pose = pose_from_odom(node.last_odom)
            node.set_mode("CLEAR")
            node.wait_goal_done()
        else:
            raise RuntimeError(f"unsupported scenario {scenario}")
        node.spin_for(2.6)
    except Exception as exc:  # noqa: BLE001 - evidence must persist on runtime failure.
        error = repr(exc)
    finally:
        node.write_jsonl()
        metrics = node.terminal_metrics(error)
        node.close_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if error is not None:
        raise RuntimeError(error)
    return metrics


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--scenario", required=True, choices=["clear", "slowdown", "stop"])
    args = parser.parse_args(argv)
    print(json.dumps(run(args.out_dir, args.scenario), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
