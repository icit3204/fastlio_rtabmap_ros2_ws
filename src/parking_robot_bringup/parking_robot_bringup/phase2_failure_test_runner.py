"""Phase 2 P2-F controlled failure runner.

The runner validates bounded failure behavior in the isolated fake-base Nav2
stack. It deliberately keeps this code separate from the accepted P2-D and
P2-E runners.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import json
import math
import os
from pathlib import Path
import signal
import time
import traceback
from typing import Any

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml

from parking_robot_bringup.phase2_goal_test_runner import (
    CommandLimits,
    analyze_path_against_map,
    build_goal,
    build_initialpose,
    finite_pose,
    goal_status_name,
    load_command_limits,
    max_path_gap,
    path_length,
    pose_from_odom,
    position_error,
    Pose2D,
    twist_is_finite,
    twist_nonzero,
    twist_within_limits,
    unsupported_twist_fields_nonzero,
    yaw_error,
    yaw_from_quaternion,
)


class P2FState(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    SETTING_START = "SETTING_START"
    VERIFYING_START = "VERIFYING_START"
    PRE_FAULT = "PRE_FAULT"
    GOAL_REQUEST_SENT = "GOAL_REQUEST_SENT"
    GOAL_ACTIVE = "GOAL_ACTIVE"
    CANCEL_REQUEST_SENT = "CANCEL_REQUEST_SENT"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    REJECTED = "REJECTED"
    TIMEOUT = "TIMEOUT"
    FAILED = "FAILED"
    CLEANUP = "CLEANUP"


TERMINAL_STATES = {
    P2FState.SUCCEEDED,
    P2FState.ABORTED,
    P2FState.REJECTED,
    P2FState.TIMEOUT,
    P2FState.FAILED,
}


VALID_TRANSITIONS = {
    P2FState.IDLE: {P2FState.WAITING_FOR_NAV2, P2FState.FAILED},
    P2FState.WAITING_FOR_NAV2: {P2FState.SETTING_START, P2FState.PRE_FAULT, P2FState.TIMEOUT, P2FState.FAILED},
    P2FState.SETTING_START: {P2FState.VERIFYING_START, P2FState.FAILED},
    P2FState.VERIFYING_START: {P2FState.PRE_FAULT, P2FState.GOAL_REQUEST_SENT, P2FState.TIMEOUT, P2FState.FAILED},
    P2FState.PRE_FAULT: {P2FState.GOAL_REQUEST_SENT, P2FState.TIMEOUT, P2FState.FAILED},
    P2FState.GOAL_REQUEST_SENT: {P2FState.GOAL_ACTIVE, P2FState.REJECTED, P2FState.TIMEOUT, P2FState.FAILED},
    P2FState.GOAL_ACTIVE: {
        P2FState.CANCEL_REQUEST_SENT,
        P2FState.SUCCEEDED,
        P2FState.ABORTED,
        P2FState.TIMEOUT,
        P2FState.FAILED,
    },
    P2FState.CANCEL_REQUEST_SENT: {P2FState.TIMEOUT, P2FState.ABORTED, P2FState.FAILED},
    P2FState.SUCCEEDED: {P2FState.CLEANUP},
    P2FState.ABORTED: {P2FState.CLEANUP},
    P2FState.REJECTED: {P2FState.CLEANUP},
    P2FState.TIMEOUT: {P2FState.CLEANUP},
    P2FState.FAILED: {P2FState.CLEANUP},
    P2FState.CLEANUP: set(),
}


def validate_transition(old: P2FState, new: P2FState) -> None:
    if new not in VALID_TRANSITIONS[old]:
        raise ValueError(f"invalid P2-F transition {old.value} -> {new.value}")


def pose_from_mapping(data: dict[str, Any]) -> Pose2D:
    return Pose2D(float(data["x"]), float(data["y"]), float(data["yaw"]))


@dataclass
class PathSnapshot:
    timestamp_elapsed_sec: float
    pose_count: int
    length_m: float
    frame: str
    endpoint_error_m: float | None
    max_gap_m: float | None
    occupied_hits: int
    unknown_hits: int
    out_of_bounds_hits: int


@dataclass
class StopEvidence:
    failure_elapsed_sec: float | None = None
    first_zero_command_after_failure_elapsed_sec: float | None = None
    command_stop_latency_sec: float | None = None
    last_nonzero_command_elapsed_sec: float | None = None
    first_zero_twist_after_failure_elapsed_sec: float | None = None
    odom_velocity_stop_latency_sec: float | None = None
    last_nonzero_odom_twist_elapsed_sec: float | None = None
    translation_after_failure_m: float | None = None
    yaw_change_after_failure_rad: float | None = None
    post_stop_motion_m: float | None = None


class Phase2FailureTestRunner(Node):
    def __init__(self, executor: SingleThreadedExecutor | None = None) -> None:
        super().__init__("phase2_failure_test_runner")
        self._executor = executor
        self._declare_parameters()
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in {
            "planner_failure",
            "controller_no_progress",
            "tf_loss",
            "cold_start_missing_tf",
            "action_response_probe",
        }:
            raise ValueError(f"unsupported mode: {self.mode}")
        self.scenario_name = str(self.get_parameter("scenario_name").value) or self._default_scenario()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.output_result_path = Path(str(self.get_parameter("output_result_path").value))
        self.server_wait_timeout_sec = float(self.get_parameter("server_wait_timeout_sec").value)
        self.initialpose_settle_timeout_sec = float(self.get_parameter("initialpose_settle_timeout_sec").value)
        self.goal_response_timeout_sec = float(self.get_parameter("goal_response_timeout_sec").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.command_stop_timeout_sec = float(self.get_parameter("command_stop_timeout_sec").value)
        self.post_stop_observation_sec = float(self.get_parameter("post_stop_observation_sec").value)
        self.scenario_path = self._resolve_path("scenario_path", "config/phase2_test_scenarios.yaml")
        self.nav2_params_path = self._resolve_path("nav2_params_path", "config/phase2_nav2_params.yaml")
        self.map_yaml_path = self._resolve_path("map_yaml_path", "maps/phase2_clean_map.yaml")
        self.command_limits: CommandLimits = load_command_limits(self.nav2_params_path)
        if self.mode == "action_response_probe":
            self.start_pose = Pose2D(5.425, -53.725, 0.0)
            self.goal_pose = Pose2D(5.425, -53.725, 0.0)
            self.pose_clamp_rate_hz = None
            self.transform_stale_wait_sec = None
        elif self.mode == "cold_start_missing_tf":
            self.start_pose = Pose2D(0.0, 0.0, 0.0)
            self.goal_pose = Pose2D(8.425, -53.725, 0.0)
            self.pose_clamp_rate_hz = None
            self.transform_stale_wait_sec = None
        else:
            self.start_pose, self.goal_pose, self.pose_clamp_rate_hz, self.transform_stale_wait_sec = self._load_scenario()

        self._start_monotonic = time.monotonic()
        self.state = P2FState.IDLE
        self.state_transitions = [{"state": self.state.value, "elapsed_sec": 0.0}]
        self.failure_reasons: list[str] = []
        self._finished = False

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_phase2_mock", self._cmd_cb, 50)
        self.create_subscription(NavPath, str(self.get_parameter("global_path_topic").value), self._path_cb, 20)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._last_odom: Odometry | None = None
        self._last_odom_elapsed_sec: float | None = None
        self._last_cmd: Twist | None = None
        self._initialpose_publish_count = 0
        self._goal_send_count = 0
        self._cancel_count = 0
        self._start_verified = False
        self._start_error_m: float | None = None
        self._start_yaw_error_rad: float | None = None

        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._response_processed = False
        self._result_processed = False
        self._cancel_processed = False
        self._goal_response_exception: str | None = None
        self._result_exception: str | None = None
        self._cancel_exception: str | None = None
        self._goal_response_received = False
        self._result_received = False
        self._cancel_response_received = False
        self._goal_id: str | None = None
        self._accepted = False
        self._goal_request_elapsed_sec: float | None = None
        self._goal_response_elapsed_sec: float | None = None
        self._result_request_elapsed_sec: float | None = None
        self._result_elapsed_sec: float | None = None
        self._cancel_request_elapsed_sec: float | None = None
        self._cancel_response_elapsed_sec: float | None = None
        self._action_status: int | None = None
        self._action_status_name: str | None = None
        self._goal_active_monotonic: float | None = None
        self._goal_active_elapsed_sec: float | None = None
        self._natural_result_deadline_monotonic: float | None = None
        self._cleanup_cancellation_deadline_monotonic: float | None = None

        self._cmd_count = 0
        self._nonzero_cmd_count = 0
        self._invalid_cmd_count = 0
        self._unsupported_cmd_count = 0
        self._cmd_limit_violations = 0
        self._first_nonzero_cmd_elapsed_sec: float | None = None
        self._last_nonzero_cmd_elapsed_sec: float | None = None
        self._zero_cmd_after_failure_elapsed_sec: float | None = None
        self._max_abs_linear_x = 0.0
        self._max_abs_angular_z = 0.0
        self._odom_count = 0
        self._last_nonzero_twist_elapsed_sec: float | None = None
        self._zero_twist_after_failure_elapsed_sec: float | None = None
        self._failure_pose: Pose2D | None = None
        self._post_stop_start_pose: Pose2D | None = None
        self._post_stop_end_pose: Pose2D | None = None
        self.stop_evidence = StopEvidence()
        self._path_message_count = 0
        self._paths: list[PathSnapshot] = []
        self._pose_clamp_started = False
        self._pose_clamp_start_elapsed_sec: float | None = None
        self._pose_clamp_publish_count = 0
        self._pose_clamp_publish_elapsed_sec: list[float] = []
        self._pose_clamp_measured_frequency_hz: float | None = None
        self._fake_base_pid: int | None = None
        self._fake_base_exit_observed = False
        self._odom_stale_before_goal = False
        self._tf_stale_before_goal = False
        self._natural_result_observation_deadline_sec = 360.0 if self.mode == "controller_no_progress" else self.goal_timeout_sec
        self._cleanup_cancellation_deadline_sec = 390.0 if self.mode == "controller_no_progress" else self.goal_timeout_sec
        self._cleanup_cancellation_allowed_elapsed_sec: float | None = None
        self._natural_terminal_result_received_before_cleanup = False
        self._lifecycle_startup_attempted = False
        self._lifecycle_startup_success = False
        self._managed_node_states: dict[str, dict[str, Any]] = {}
        self._navigate_to_pose_server_available = False
        self._tf_checks: dict[str, dict[str, Any]] = {}
        self._cold_start_final_classification: str | None = None
        self._shutdown_result: str | None = None
        self._runner_exception_type: str | None = None
        self._runner_exception_message: str | None = None
        self._runner_exception_state: str | None = None
        self._runner_exception_traceback: str | None = None

    def _declare_parameters(self) -> None:
        self.declare_parameter("mode", "planner_failure")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("nav2_params_path", "")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("server_wait_timeout_sec", 30.0)
        self.declare_parameter("initialpose_settle_timeout_sec", 10.0)
        self.declare_parameter("goal_response_timeout_sec", 10.0)
        self.declare_parameter("goal_timeout_sec", 45.0)
        self.declare_parameter("command_stop_timeout_sec", 0.5)
        self.declare_parameter("post_stop_observation_sec", 1.0)
        self.declare_parameter("output_result_path", "/tmp/phase2_p2f_result.json")
        self.declare_parameter("global_path_topic", "/plan")
        self.declare_parameter("source_revision", "")

    def _default_scenario(self) -> str:
        return {
            "planner_failure": "p2f_planner_occupied",
            "controller_no_progress": "p2f_controller_no_progress",
            "tf_loss": "p2f_tf_loss",
            "cold_start_missing_tf": "p2f_cold_start_missing_tf_internal",
            "action_response_probe": "p2f_action_response_probe_internal",
        }[self.mode]

    def _resolve_path(self, parameter_name: str, package_relative: str) -> Path:
        value = str(self.get_parameter(parameter_name).value)
        if value:
            return Path(value)
        return Path(get_package_share_directory("parking_robot_bringup")) / package_relative

    def _load_scenario(self) -> tuple[Pose2D, Pose2D, float | None, float | None]:
        data = yaml.safe_load(self.scenario_path.read_text())[self.scenario_name]
        return (
            pose_from_mapping(data["initial_pose"]),
            pose_from_mapping(data["goal"]),
            float(data.get("pose_clamp_rate_hz", 0.0)) or None,
            float(data.get("transform_stale_wait_sec", 0.0)) or None,
        )

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_monotonic

    def _transition(self, new: P2FState) -> None:
        validate_transition(self.state, new)
        self.state = new
        self.state_transitions.append({"state": new.value, "elapsed_sec": self._elapsed()})
        self.get_logger().info(f"P2-F state -> {new.value}")

    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        if self._executor is None:
            raise RuntimeError("Phase2FailureTestRunner requires one explicit executor")
        self._executor.spin_once(timeout_sec=timeout_sec)

    def _spin_until(self, predicate, timeout_sec: float, after_spin=None) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.05)
            if after_spin is not None:
                after_spin()
            if predicate():
                return True
        if after_spin is not None:
            after_spin()
        return predicate()

    def _spin_for(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.05)

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg
        elapsed = self._elapsed()
        self._last_odom_elapsed_sec = elapsed
        self._odom_count += 1
        lin = abs(float(msg.twist.twist.linear.x))
        ang = abs(float(msg.twist.twist.angular.z))
        if lin > 1.0e-6 or ang > 1.0e-6:
            self._last_nonzero_twist_elapsed_sec = elapsed
        elif self.stop_evidence.failure_elapsed_sec is not None and self._zero_twist_after_failure_elapsed_sec is None:
            self._zero_twist_after_failure_elapsed_sec = elapsed

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd = msg
        elapsed = self._elapsed()
        self._cmd_count += 1
        if not twist_is_finite(msg):
            self._invalid_cmd_count += 1
            return
        if unsupported_twist_fields_nonzero(msg):
            self._unsupported_cmd_count += 1
        if not twist_within_limits(msg, self.command_limits):
            self._cmd_limit_violations += 1
        self._max_abs_linear_x = max(self._max_abs_linear_x, abs(float(msg.linear.x)))
        self._max_abs_angular_z = max(self._max_abs_angular_z, abs(float(msg.angular.z)))
        if twist_nonzero(msg):
            self._nonzero_cmd_count += 1
            if self._first_nonzero_cmd_elapsed_sec is None:
                self._first_nonzero_cmd_elapsed_sec = elapsed
            self._last_nonzero_cmd_elapsed_sec = elapsed
        elif self.stop_evidence.failure_elapsed_sec is not None and self._zero_cmd_after_failure_elapsed_sec is None:
            self._zero_cmd_after_failure_elapsed_sec = elapsed

    def _path_cb(self, msg: NavPath) -> None:
        self._path_message_count += 1
        if len(msg.poses) == 0:
            return
        poses = [Pose2D(float(p.pose.position.x), float(p.pose.position.y), yaw_from_quaternion(p.pose.orientation)) for p in msg.poses]
        analysis = analyze_path_against_map(poses, self.map_yaml_path)
        self._paths.append(
            PathSnapshot(
                timestamp_elapsed_sec=self._elapsed(),
                pose_count=len(poses),
                length_m=path_length(poses),
                frame=msg.header.frame_id,
                endpoint_error_m=position_error(poses[-1], self.goal_pose),
                max_gap_m=max_path_gap(poses),
                occupied_hits=int(analysis.get("occupied_hits", 0)),
                unknown_hits=int(analysis.get("unknown_hits", 0)),
                out_of_bounds_hits=int(analysis.get("out_of_bounds_hits", 0)),
            )
        )

    def _goal_handle_id(self, goal_handle: Any) -> str | None:
        gid = getattr(goal_handle, "goal_id", None)
        uuid = getattr(gid, "uuid", None)
        if uuid is None:
            return None
        try:
            return "".join(f"{int(b):02x}" for b in uuid)
        except TypeError:
            return str(uuid)

    def _goal_response_cb(self, future: Any) -> None:
        self._process_goal_response_if_ready()

    def _process_goal_response_if_ready(self) -> None:
        if self._send_goal_future is None or not self._send_goal_future.done() or self._response_processed:
            return
        try:
            self._goal_handle = self._send_goal_future.result()
            self._response_processed = True
            self._goal_response_received = True
            self._goal_response_elapsed_sec = self._elapsed()
            if self._goal_handle is not None:
                self._accepted = bool(self._goal_handle.accepted)
                self._goal_id = self._goal_handle_id(self._goal_handle)
        except Exception as exc:
            self._response_processed = True
            self._goal_response_exception = repr(exc)
            self.failure_reasons.append(f"goal response exception: {exc!r}")

    def _result_response_cb(self, future: Any) -> None:
        self._process_result_response_if_ready()

    def _process_result_response_if_ready(self) -> None:
        if self._result_future is None or not self._result_future.done() or self._result_processed:
            return
        try:
            result = self._result_future.result()
            self._result_processed = True
            self._result_received = True
            self._result_elapsed_sec = self._elapsed()
            self._action_status = int(result.status)
            self._action_status_name = goal_status_name(result.status)
        except Exception as exc:
            self._result_processed = True
            self._result_exception = repr(exc)
            self.failure_reasons.append(f"result exception: {exc!r}")

    def _process_cancel_response_if_ready(self) -> None:
        if self._cancel_future is None or not self._cancel_future.done() or self._cancel_processed:
            return
        try:
            self._cancel_future.result()
            self._cancel_response_received = True
        except Exception as exc:
            self._cancel_exception = repr(exc)
            self.failure_reasons.append(f"cancel exception: {exc!r}")
        finally:
            self._cancel_processed = True

    def run(self) -> dict[str, Any]:
        if self.mode == "cold_start_missing_tf":
            return self._run_cold_start_missing_tf()
        self._transition(P2FState.WAITING_FOR_NAV2)
        if self.mode != "tf_loss" and not self._spin_until(lambda: self._last_odom is not None, self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for /Odometry")
            self._transition(P2FState.TIMEOUT)
            return self._finish()
        if self.mode == "tf_loss" and not self._spin_until(lambda: self._last_odom is not None, self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for pre-loss /Odometry")
            self._transition(P2FState.TIMEOUT)
            return self._finish()
        if not self._action_client.wait_for_server(timeout_sec=self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for /navigate_to_pose")
            self._transition(P2FState.TIMEOUT)
            return self._finish()
        if not self._set_initial_pose_once():
            return self._finish()
        if self.mode == "tf_loss":
            if not self._perform_tf_loss_fault():
                return self._finish()
        return self._send_goal_and_observe()

    def _set_initial_pose_once(self) -> bool:
        self._transition(P2FState.SETTING_START)
        self._publish_initialpose()
        self._transition(P2FState.VERIFYING_START)
        def verified() -> bool:
            if self._last_odom is None:
                return False
            pose = pose_from_odom(self._last_odom)
            self._start_error_m = position_error(pose, self.start_pose)
            self._start_yaw_error_rad = yaw_error(pose.yaw, self.start_pose.yaw)
            stopped = abs(float(self._last_odom.twist.twist.linear.x)) <= 1e-6 and abs(float(self._last_odom.twist.twist.angular.z)) <= 1e-6
            return self._start_error_m <= 0.10 and self._start_yaw_error_rad <= 0.05 and stopped
        if not self._spin_until(verified, self.initialpose_settle_timeout_sec):
            self.failure_reasons.append("Start pose reset was not verified")
            self._transition(P2FState.FAILED)
            return False
        self._start_verified = True
        return True

    def _publish_initialpose(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._initialpose_pub.publish(build_initialpose(self.start_pose, self.frame_id, stamp))
        self._initialpose_publish_count += 1

    def _perform_tf_loss_fault(self) -> bool:
        self._transition(P2FState.PRE_FAULT)
        self._fake_base_pid = self._find_fake_base_pid()
        if self._fake_base_pid is None:
            self.failure_reasons.append("phase2_fake_base process not found")
            self._transition(P2FState.FAILED)
            return False
        os.kill(self._fake_base_pid, signal.SIGINT)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            proc_path = Path(f"/proc/{self._fake_base_pid}")
            if not proc_path.exists() or self._proc_state(self._fake_base_pid) == "Z":
                self._fake_base_exit_observed = True
                break
            self._spin_once(0.05)
        stale_wait = float(self.transform_stale_wait_sec or 2.0)
        last = self._last_odom_elapsed_sec
        deadline = time.monotonic() + stale_wait
        while time.monotonic() < deadline:
            self._spin_once(0.05)
        self._odom_stale_before_goal = last is not None and self._last_odom_elapsed_sec == last
        self._tf_stale_before_goal = self._odom_stale_before_goal
        if not self._odom_stale_before_goal:
            self.failure_reasons.append("Odometry did not become stale after fake-base termination")
            self._transition(P2FState.FAILED)
            return False
        return True

    def _find_fake_base_pid(self) -> int | None:
        mine = os.getpid()
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                cmd = (proc / "cmdline").read_bytes().replace(b"\\x00", b" ").decode(errors="ignore")
            except OSError:
                continue
            if "phase2_fake_base" in cmd and int(proc.name) != mine:
                return int(proc.name)
        return None

    def _run_cold_start_missing_tf(self) -> dict[str, Any]:
        self._transition(P2FState.WAITING_FOR_NAV2)
        self._lifecycle_startup_attempted = True
        self._query_managed_node_states()
        self._lifecycle_startup_success = self._all_managed_nodes_active()
        self._navigate_to_pose_server_available = self._action_client.wait_for_server(
            timeout_sec=self.server_wait_timeout_sec
        )
        self._spin_for(1.0)
        # Cold-start missing TF deliberately has no initial-pose stage; PRE_FAULT
        # performs the independent zero-odometry and TF-absence checks.
        self._transition(P2FState.PRE_FAULT)
        self._verify_cold_start_tf_absence()
        if self._odom_count != 0:
            self.failure_reasons.append(f"cold-start expected zero /Odometry messages, got {self._odom_count}")
        missing_dynamic_tf_proven = (
            self._tf_available("map_to_odom") is True
            and self._tf_available("odom_to_base") is False
            and self._tf_available("map_to_base") is False
        )
        if not missing_dynamic_tf_proven:
            self.failure_reasons.append("cold-start missing dynamic TF was not proven independently")
        if not self._lifecycle_startup_success:
            self._transition(P2FState.TIMEOUT)
            self._mark_failure_time()
            self._observe_stop_and_post_motion()
            self._evaluate()
            return self._finish()
        if not self._navigate_to_pose_server_available:
            self._transition(P2FState.TIMEOUT)
            self._mark_failure_time()
            self._observe_stop_and_post_motion()
            self._evaluate()
            return self._finish()
        if self.failure_reasons:
            self._transition(P2FState.FAILED)
            self._mark_failure_time()
            self._observe_stop_and_post_motion()
            self._evaluate()
            return self._finish()
        return self._send_goal_and_observe()

    def _query_managed_node_states(self) -> None:
        for name in ("map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"):
            service_name = f"/{name}/get_state"
            client = self.create_client(GetState, service_name)
            state: dict[str, Any] = {"service": service_name, "available": False, "id": None, "label": None}
            if client.wait_for_service(timeout_sec=0.2):
                state["available"] = True
                future = client.call_async(GetState.Request())
                if self._spin_until(lambda: future.done(), 1.0):
                    try:
                        current = future.result().current_state
                        state["id"] = int(current.id)
                        state["label"] = str(current.label)
                    except Exception as exc:
                        state["error"] = repr(exc)
                else:
                    state["error"] = "get_state timeout"
            self._managed_node_states[name] = state
            self.destroy_client(client)

    def _all_managed_nodes_active(self) -> bool:
        return bool(self._managed_node_states) and all(
            state.get("id") == State.PRIMARY_STATE_ACTIVE for state in self._managed_node_states.values()
        )

    def _tf_available(self, key: str) -> bool | None:
        check = self._tf_checks.get(key, {})
        available = check.get("available")
        return available if isinstance(available, bool) else None

    def _verify_cold_start_tf_absence(self) -> None:
        for key, target, source in (
            ("map_to_odom", "map", "odom"),
            ("odom_to_base", "odom", "base_footprint"),
            ("map_to_base", "map", "base_footprint"),
        ):
            self._tf_checks[key] = self._check_transform(target, source)

    def _check_transform(self, target: str, source: str) -> dict[str, Any]:
        result: dict[str, Any] = {"target": target, "source": source, "available": False, "error": None}
        try:
            result["available"] = bool(self._tf_buffer.can_transform(target, source, Time(), timeout=Duration(seconds=0.5)))
            if result["available"]:
                transform = self._tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.5))
                result["stamp_sec"] = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) / 1.0e9
                result["parent"] = transform.header.frame_id
                result["child"] = transform.child_frame_id
        except Exception as exc:
            result["available"] = False
            result["error"] = repr(exc)
        return result

    def _proc_state(self, pid: int) -> str | None:
        try:
            for line in Path(f"/proc/{pid}/status").read_text().splitlines():
                if line.startswith("State:"):
                    return line.split()[1]
        except OSError:
            return None
        return None

    def _send_goal_and_observe(self) -> dict[str, Any]:
        self._transition(P2FState.GOAL_REQUEST_SENT)
        self._goal_send_count += 1
        self._goal_request_elapsed_sec = self._elapsed()
        try:
            self._send_goal_future = self._action_client.send_goal_async(build_goal(self.goal_pose, self.frame_id, self.get_clock().now().to_msg()))
            self._send_goal_future.add_done_callback(self._goal_response_cb)
        except Exception as exc:
            self.failure_reasons.append(f"send_goal_async failed: {exc!r}")
            self._transition(P2FState.FAILED)
            return self._finish()
        if not self._spin_until(lambda: self._response_processed or self._goal_response_exception is not None, self.goal_response_timeout_sec, self._process_goal_response_if_ready):
            self.failure_reasons.append("goal response timeout")
            self._transition(P2FState.TIMEOUT)
            return self._finish()
        if self._goal_response_exception:
            self._transition(P2FState.FAILED)
            return self._finish()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self._transition(P2FState.REJECTED)
            self._mark_failure_time()
            self._observe_stop_and_post_motion()
            self._evaluate()
            return self._finish()
        self._transition(P2FState.GOAL_ACTIVE)
        self._mark_goal_active_time()
        self._result_request_elapsed_sec = self._elapsed()
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_response_cb)
        deadline = self._natural_result_deadline_monotonic or (time.monotonic() + self._natural_result_observation_deadline_sec)
        next_clamp = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline and not self._result_processed:
            self._spin_once(0.02)
            self._process_result_response_if_ready()
            if self.mode == "controller_no_progress" and self._first_nonzero_cmd_elapsed_sec is not None:
                if not self._pose_clamp_started:
                    self._pose_clamp_started = True
                    self._pose_clamp_start_elapsed_sec = self._elapsed()
                if time.monotonic() >= next_clamp:
                    self._publish_initialpose()
                    self._pose_clamp_publish_count += 1
                    self._pose_clamp_publish_elapsed_sec.append(self._elapsed())
                    self._update_pose_clamp_frequency()
                    next_clamp = time.monotonic() + 1.0 / float(self.pose_clamp_rate_hz or 10.0)
        self._process_result_response_if_ready()
        if not self._result_processed:
            self._mark_failure_time()
            if self.mode == "controller_no_progress":
                cleanup_deadline = self._cleanup_cancellation_deadline_monotonic or (
                    time.monotonic() + (self._cleanup_cancellation_deadline_sec - self._natural_result_observation_deadline_sec)
                )
                while rclpy.ok() and time.monotonic() < cleanup_deadline and not self._result_processed:
                    self._spin_once(0.02)
                    self._process_result_response_if_ready()
                    if self.mode == "controller_no_progress" and self._first_nonzero_cmd_elapsed_sec is not None:
                        if time.monotonic() >= next_clamp:
                            self._publish_initialpose()
                            self._pose_clamp_publish_count += 1
                            self._pose_clamp_publish_elapsed_sec.append(self._elapsed())
                            self._update_pose_clamp_frequency()
                            next_clamp = time.monotonic() + 1.0 / float(self.pose_clamp_rate_hz or 10.0)
                self._process_result_response_if_ready()
            if self._result_processed:
                self._mark_failure_time()
                if self._action_status == GoalStatus.STATUS_ABORTED:
                    self._natural_terminal_result_received_before_cleanup = self._cancel_count == 0
                    self._transition(P2FState.ABORTED)
                else:
                    self._transition(P2FState.FAILED)
                self._observe_stop_and_post_motion()
                self._evaluate()
                return self._finish()
            self._cleanup_cancellation_allowed_elapsed_sec = self._elapsed()
            self._cancel_count += 1
            try:
                self._transition(P2FState.CANCEL_REQUEST_SENT)
                self._cancel_request_elapsed_sec = self._elapsed()
                self._cancel_future = self._goal_handle.cancel_goal_async()
                self._spin_until(lambda: self._cancel_processed, 5.0, self._process_cancel_response_if_ready)
            except Exception as exc:
                self.failure_reasons.append(f"cancel after timeout failed: {exc!r}")
            self._transition(P2FState.TIMEOUT)
        else:
            self._mark_failure_time()
            if self._action_status == GoalStatus.STATUS_ABORTED:
                self._natural_terminal_result_received_before_cleanup = self._cancel_count == 0
                self._transition(P2FState.ABORTED)
            elif self.mode == "action_response_probe" and self._action_status == GoalStatus.STATUS_SUCCEEDED:
                self._transition(P2FState.SUCCEEDED)
            else:
                self._transition(P2FState.FAILED)
        self._observe_stop_and_post_motion()
        self._evaluate()
        return self._finish()

    def _mark_goal_active_time(self) -> None:
        self._goal_active_monotonic = time.monotonic()
        self._goal_active_elapsed_sec = self._elapsed()
        self._natural_result_deadline_monotonic = self._goal_active_monotonic + self._natural_result_observation_deadline_sec
        self._cleanup_cancellation_deadline_monotonic = (
            self._goal_active_monotonic + self._cleanup_cancellation_deadline_sec
        )

    def _update_pose_clamp_frequency(self) -> None:
        if len(self._pose_clamp_publish_elapsed_sec) < 2:
            self._pose_clamp_measured_frequency_hz = None
            return
        elapsed = self._pose_clamp_publish_elapsed_sec[-1] - self._pose_clamp_publish_elapsed_sec[0]
        if elapsed > 0.0:
            self._pose_clamp_measured_frequency_hz = (len(self._pose_clamp_publish_elapsed_sec) - 1) / elapsed

    def _mark_failure_time(self) -> None:
        self.stop_evidence.failure_elapsed_sec = self._result_elapsed_sec or self._elapsed()
        self._failure_pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None

    def _observe_stop_and_post_motion(self) -> None:
        deadline = time.monotonic() + self.command_stop_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.02)
        self.stop_evidence.first_zero_command_after_failure_elapsed_sec = self._zero_cmd_after_failure_elapsed_sec
        self.stop_evidence.last_nonzero_command_elapsed_sec = self._last_nonzero_cmd_elapsed_sec
        if self.stop_evidence.failure_elapsed_sec is not None:
            if self._last_nonzero_cmd_elapsed_sec is None or self._last_nonzero_cmd_elapsed_sec <= self.stop_evidence.failure_elapsed_sec:
                self.stop_evidence.command_stop_latency_sec = 0.0
            elif self._zero_cmd_after_failure_elapsed_sec is not None:
                self.stop_evidence.command_stop_latency_sec = self._zero_cmd_after_failure_elapsed_sec - self.stop_evidence.failure_elapsed_sec
            self.stop_evidence.first_zero_twist_after_failure_elapsed_sec = self._zero_twist_after_failure_elapsed_sec
            self.stop_evidence.last_nonzero_odom_twist_elapsed_sec = self._last_nonzero_twist_elapsed_sec
            if self._last_nonzero_twist_elapsed_sec is None or self._last_nonzero_twist_elapsed_sec <= self.stop_evidence.failure_elapsed_sec:
                self.stop_evidence.odom_velocity_stop_latency_sec = 0.0
            elif self._zero_twist_after_failure_elapsed_sec is not None:
                self.stop_evidence.odom_velocity_stop_latency_sec = self._zero_twist_after_failure_elapsed_sec - self.stop_evidence.failure_elapsed_sec
        start_pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None
        deadline = time.monotonic() + self.post_stop_observation_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.05)
        end_pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None
        if self._failure_pose and end_pose:
            self.stop_evidence.translation_after_failure_m = position_error(self._failure_pose, end_pose)
            self.stop_evidence.yaw_change_after_failure_rad = yaw_error(self._failure_pose.yaw, end_pose.yaw)
        if start_pose and end_pose:
            self.stop_evidence.post_stop_motion_m = position_error(start_pose, end_pose)

    def _evaluate(self) -> None:
        if self.mode != "cold_start_missing_tf" and self._goal_send_count != 1:
            self.failure_reasons.append(f"expected one goal, sent {self._goal_send_count}")
        if self.mode == "planner_failure":
            if self._action_status != GoalStatus.STATUS_ABORTED:
                self.failure_reasons.append(f"planner failure expected ABORTED status 6, got {self._action_status}")
            if self._nonzero_cmd_count != 0:
                self.failure_reasons.append("planner failure produced nonzero commands")
            if self.stop_evidence.post_stop_motion_m is not None and self.stop_evidence.post_stop_motion_m > 0.01:
                self.failure_reasons.append("planner failure post-stop motion exceeded 0.01 m")
        elif self.mode == "controller_no_progress":
            if not self._accepted:
                self.failure_reasons.append("no-progress goal was not accepted")
            if self._first_nonzero_cmd_elapsed_sec is None:
                self.failure_reasons.append("no-progress test saw no nonzero command")
            if not self._pose_clamp_started or self._pose_clamp_publish_count <= 0:
                self.failure_reasons.append("pose clamp did not start")
            if self._pose_clamp_publish_count != len(self._pose_clamp_publish_elapsed_sec):
                self.failure_reasons.append("pose clamp timestamp count did not match publication count")
            if self._pose_clamp_publish_count >= 2 and self._pose_clamp_measured_frequency_hz is None:
                self.failure_reasons.append("pose clamp measured frequency was not computed")
            if (
                self._pose_clamp_measured_frequency_hz is not None
                and not 7.0 <= self._pose_clamp_measured_frequency_hz <= 13.0
            ):
                self.failure_reasons.append(
                    f"pose clamp measured frequency outside expected range: {self._pose_clamp_measured_frequency_hz}"
                )
            if self._action_status != GoalStatus.STATUS_ABORTED:
                self.failure_reasons.append(f"no-progress expected ABORTED status 6, got {self._action_status}")
            if self._cancel_count and self._action_status == GoalStatus.STATUS_CANCELED:
                self.failure_reasons.append("cleanup-generated CANCELED status 5 cannot pass no-progress")
            if self._cancel_count and self._goal_active_elapsed_sec is None:
                self.failure_reasons.append("cleanup cancel timing cannot be proven without GOAL_ACTIVE timestamp")
            if (
                self._cancel_count
                and self._cancel_request_elapsed_sec is not None
                and self._goal_active_elapsed_sec is not None
                and self._cancel_request_elapsed_sec - self._goal_active_elapsed_sec < self._cleanup_cancellation_deadline_sec
            ):
                self.failure_reasons.append("cleanup cancellation occurred before 390 sec after GOAL_ACTIVE")
            if self.stop_evidence.command_stop_latency_sec is None or self.stop_evidence.command_stop_latency_sec > 0.5:
                self.failure_reasons.append("command stop latency exceeded 0.5 sec")
            if self.stop_evidence.odom_velocity_stop_latency_sec is None or self.stop_evidence.odom_velocity_stop_latency_sec > 0.5:
                self.failure_reasons.append("odom velocity stop latency exceeded 0.5 sec")
            if self.stop_evidence.translation_after_failure_m is not None and self.stop_evidence.translation_after_failure_m > 0.01:
                self.failure_reasons.append("post-failure translation exceeded 0.01 m")
        elif self.mode == "tf_loss":
            if not self._fake_base_exit_observed or not self._odom_stale_before_goal:
                self.failure_reasons.append("tf-loss fake-base/odom staleness was not proven")
            acceptable = (not self._accepted) or self._action_status == GoalStatus.STATUS_ABORTED or self.state == P2FState.TIMEOUT
            if not acceptable:
                self.failure_reasons.append(f"tf-loss action disposition not accepted: accepted={self._accepted} status={self._action_status} state={self.state.value}")
            if self._nonzero_cmd_count != 0:
                self.failure_reasons.append("tf-loss produced nonzero commands")
        elif self.mode == "cold_start_missing_tf":
            if self._initialpose_publish_count != 0:
                self.failure_reasons.append("cold-start missing TF must not publish /initialpose")
            if self._odom_count != 0:
                self.failure_reasons.append(f"cold-start missing TF expected zero /Odometry messages, got {self._odom_count}")
            if self._tf_available("map_to_odom") is not True:
                self.failure_reasons.append("cold-start missing TF did not prove static map->odom availability")
            if self._tf_available("odom_to_base") is not False:
                self.failure_reasons.append("cold-start missing TF unexpectedly resolved odom->base_footprint")
            if self._tf_available("map_to_base") is not False:
                self.failure_reasons.append("cold-start missing TF unexpectedly resolved map->base_footprint")
            if self._goal_send_count > 1:
                self.failure_reasons.append(f"cold-start missing TF sent more than one goal: {self._goal_send_count}")
            if self._cancel_count > 1:
                self.failure_reasons.append(f"cold-start missing TF sent more than one cleanup cancel: {self._cancel_count}")
            if self._nonzero_cmd_count != 0:
                self.failure_reasons.append("cold-start missing TF produced nonzero commands")
            if self._accepted and self._action_status not in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED, None):
                self.failure_reasons.append(f"cold-start missing TF action disposition not accepted: {self._action_status}")
            if self._action_status == GoalStatus.STATUS_CANCELED and self._cancel_count != 1:
                self.failure_reasons.append("cold-start missing TF CANCELED status requires exactly one cleanup cancel")
            if self._action_status == GoalStatus.STATUS_ABORTED and self._cancel_count != 0:
                self.failure_reasons.append("cold-start missing TF natural ABORTED must occur before cleanup cancel")
            if not self.failure_reasons:
                self._cold_start_final_classification = "COLD_START_MISSING_TF_SAFE_FAILURE_CONFIRMED"
        else:
            if not self._goal_response_received:
                self.failure_reasons.append("action-response probe did not receive a goal response")
            if not self._accepted:
                self.failure_reasons.append("action-response probe goal was not accepted")
            if not self._result_received:
                self.failure_reasons.append("action-response probe did not receive a result")
            if self._action_status != GoalStatus.STATUS_SUCCEEDED:
                self.failure_reasons.append(f"action-response probe expected SUCCEEDED status 4, got {self._action_status}")
            pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None
            if pose is None:
                self.failure_reasons.append("action-response probe has no final odometry pose")
            else:
                if position_error(self.start_pose, pose) > 0.05:
                    self.failure_reasons.append("action-response probe translation exceeded 0.05 m")
                if abs(yaw_error(self.start_pose.yaw, pose.yaw)) > 0.05:
                    self.failure_reasons.append("action-response probe yaw change exceeded 0.05 rad")
        if self._invalid_cmd_count or self._unsupported_cmd_count or self._cmd_limit_violations:
            self.failure_reasons.append("command stream invalid/unsupported/outside limits")

    def _decision(self) -> str:
        if self.failure_reasons:
            return {
                "planner_failure": "P2F_PLANNER_FAILURE_NEEDS_REVIEW",
                "controller_no_progress": "P2F_CONTROLLER_NO_PROGRESS_NEEDS_REVIEW",
                "tf_loss": "P2F_TF_LOSS_NEEDS_REVIEW",
                "cold_start_missing_tf": "P2F_COLD_START_MISSING_TF_NEEDS_REVIEW",
                "action_response_probe": "P2F_ACTION_RESPONSE_STABILITY_NEEDS_REVIEW",
            }[self.mode]
        return {
            "planner_failure": "P2F_PLANNER_FAILURE_PASS",
            "controller_no_progress": "P2F_CONTROLLER_NO_PROGRESS_PASS",
            "tf_loss": "P2F_TF_LOSS_PASS",
            "cold_start_missing_tf": "P2F_COLD_START_MISSING_TF_PASS",
            "action_response_probe": "P2F_ACTION_RESPONSE_STABILITY_PASS",
        }[self.mode]

    def _finish(self) -> dict[str, Any]:
        if self._finished:
            return self._result_dict()
        self._finished = True
        if self.state in TERMINAL_STATES:
            self._transition(P2FState.CLEANUP)
        self._shutdown_result = "terminal_json_written"
        result = self._result_dict()
        self.output_result_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_result_path.write_text(json.dumps(result, indent=2, sort_keys=True))
        return result

    def _path_dict(self, which: str) -> dict[str, Any] | None:
        if not self._paths:
            return None
        snap = {"first": self._paths[0], "longest": max(self._paths, key=lambda p: p.length_m), "final": self._paths[-1]}[which]
        return asdict(snap)

    def _result_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "scenario_name": self.scenario_name,
            "decision": self._decision(),
            "pass": not self.failure_reasons,
            "source_revision": str(self.get_parameter("source_revision").value),
            "initial_pose": asdict(self.start_pose),
            "requested_goal": asdict(self.goal_pose),
            "initialpose_publication_count": self._initialpose_publish_count,
            "initialpose_verified": self._start_verified,
            "initialpose_position_error_m": self._start_error_m,
            "initialpose_yaw_error_rad": self._start_yaw_error_rad,
            "navigate_to_pose_request_count": self._goal_send_count,
            "request_count": self._goal_send_count,
            "goal_response_received": self._goal_response_received,
            "goal_response_latency_sec": None if self._goal_request_elapsed_sec is None or self._goal_response_elapsed_sec is None else self._goal_response_elapsed_sec - self._goal_request_elapsed_sec,
            "goal_id": self._goal_id,
            "goal_accepted": self._accepted,
            "goal_active_elapsed_sec": self._goal_active_elapsed_sec,
            "natural_result_deadline_elapsed_sec": None
            if self._goal_active_elapsed_sec is None
            else self._goal_active_elapsed_sec + self._natural_result_observation_deadline_sec,
            "cleanup_cancellation_deadline_elapsed_sec": None
            if self._goal_active_elapsed_sec is None
            else self._goal_active_elapsed_sec + self._cleanup_cancellation_deadline_sec,
            "result_received": self._result_received,
            "action_result_status": self._action_status,
            "action_result_status_name": self._action_status_name,
            "result_status_code": self._action_status,
            "result_status_name": self._action_status_name,
            "action_duration_sec": None if self._goal_request_elapsed_sec is None or self._result_elapsed_sec is None else self._result_elapsed_sec - self._goal_request_elapsed_sec,
            "cancellation_count": self._cancel_count,
            "cleanup_cancel_count": self._cancel_count,
            "cleanup_cancel_sent": self._cancel_count > 0,
            "cancel_request_elapsed_sec": self._cancel_request_elapsed_sec,
            "cancel_delay_after_goal_active_sec": None
            if self._goal_active_elapsed_sec is None or self._cancel_request_elapsed_sec is None
            else self._cancel_request_elapsed_sec - self._goal_active_elapsed_sec,
            "cancel_response_received": self._cancel_response_received,
            "natural_terminal_result_received_before_cleanup": self._natural_terminal_result_received_before_cleanup,
            "state_transitions": self.state_transitions,
            "first_path": self._path_dict("first"),
            "longest_path": self._path_dict("longest"),
            "final_path": self._path_dict("final"),
            "path_message_count": self._path_message_count,
            "nonempty_path_count": len(self._paths),
            "path_history": [asdict(p) for p in self._paths],
            "command_message_count": self._cmd_count,
            "nonzero_command_count": self._nonzero_cmd_count,
            "first_nonzero_command_elapsed_sec": self._first_nonzero_cmd_elapsed_sec,
            "last_nonzero_command_elapsed_sec": self._last_nonzero_cmd_elapsed_sec,
            "first_zero_command_after_failure_elapsed_sec": self.stop_evidence.first_zero_command_after_failure_elapsed_sec,
            "command_stop_latency_sec": self.stop_evidence.command_stop_latency_sec,
            "odometry_sample_count": self._odom_count,
            "last_nonzero_odom_twist_elapsed_sec": self.stop_evidence.last_nonzero_odom_twist_elapsed_sec,
            "first_zero_odom_twist_after_failure_elapsed_sec": self.stop_evidence.first_zero_twist_after_failure_elapsed_sec,
            "odometry_velocity_stop_latency_sec": self.stop_evidence.odom_velocity_stop_latency_sec,
            "translation_after_failure_m": self.stop_evidence.translation_after_failure_m,
            "yaw_change_after_failure_rad": self.stop_evidence.yaw_change_after_failure_rad,
            "post_stop_motion_m": self.stop_evidence.post_stop_motion_m,
            "invalid_command_count": self._invalid_cmd_count,
            "unsupported_command_count": self._unsupported_cmd_count,
            "command_limit_violation_count": self._cmd_limit_violations,
            "max_abs_linear_x": self._max_abs_linear_x,
            "max_abs_angular_z": self._max_abs_angular_z,
            "controller_warnings": [],
            "progress_checker_failures": [],
            "planner_failures": [],
            "transform_failures": [],
            "bt_recovery_events": [],
            "pose_clamp_started": self._pose_clamp_started,
            "pose_clamp_start_elapsed_sec": self._pose_clamp_start_elapsed_sec,
            "pose_clamp_publish_count": self._pose_clamp_publish_count,
            "pose_clamp_publish_elapsed_sec": self._pose_clamp_publish_elapsed_sec,
            "pose_clamp_measured_frequency_hz": self._pose_clamp_measured_frequency_hz,
            "natural_result_observation_deadline_sec": self._natural_result_observation_deadline_sec,
            "cleanup_cancellation_deadline_sec": self._cleanup_cancellation_deadline_sec,
            "cleanup_cancellation_allowed_elapsed_sec": self._cleanup_cancellation_allowed_elapsed_sec,
            "fake_base_pid": self._fake_base_pid,
            "fake_base_exit_observed": self._fake_base_exit_observed,
            "odometry_stale_before_goal": self._odom_stale_before_goal,
            "tf_stale_before_goal": self._tf_stale_before_goal,
            "lifecycle_startup_attempted": self._lifecycle_startup_attempted,
            "lifecycle_startup_success": self._lifecycle_startup_success,
            "managed_node_states": self._managed_node_states,
            "navigate_to_pose_server_available": self._navigate_to_pose_server_available,
            "odometry_message_count": self._odom_count,
            "map_to_odom_available": self._tf_available("map_to_odom"),
            "odom_to_base_available": self._tf_available("odom_to_base"),
            "map_to_base_available": self._tf_available("map_to_base"),
            "tf_lookup_results": self._tf_checks,
            "tf_lookup_errors": {key: value.get("error") for key, value in self._tf_checks.items()},
            "command_publishers_by_gid": {},
            "unknown_publisher_count": 0,
            "final_classification": self._cold_start_final_classification,
            "shutdown_result": self._shutdown_result,
            "runner_exception_type": self._runner_exception_type,
            "runner_exception_message": self._runner_exception_message,
            "runner_exception_state": self._runner_exception_state,
            "runner_exception_traceback": self._runner_exception_traceback,
            "failure_reasons": self.failure_reasons,
        }

    def _record_internal_exception(self, exc: Exception) -> None:
        self._runner_exception_type = type(exc).__name__
        self._runner_exception_message = str(exc)
        self._runner_exception_state = self.state.value
        self._runner_exception_traceback = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))[-8000:]
        self.failure_reasons.append(f"RUNNER_INTERNAL_EXCEPTION: {type(exc).__name__}: {exc}")
        if self.mode == "cold_start_missing_tf":
            self._cold_start_final_classification = "RUNNER_INTERNAL_EXCEPTION"
        if self.state not in TERMINAL_STATES and self.state != P2FState.CLEANUP:
            try:
                self._transition(P2FState.FAILED)
            except ValueError as transition_exc:
                self.failure_reasons.append(f"forced terminal diagnostic after invalid transition: {transition_exc}")
                self.state = P2FState.FAILED
                self.state_transitions.append(
                    {
                        "state": P2FState.FAILED.value,
                        "elapsed_sec": self._elapsed(),
                        "forced": True,
                    }
                )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = Phase2FailureTestRunner(executor=executor)
    executor.add_node(node)
    try:
        result = node.run()
        if not result.get("pass", False):
            raise SystemExit(2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node._finished:
            node.failure_reasons.append("P2-F runner interrupted before completion")
            if node.state not in TERMINAL_STATES and node.state != P2FState.CLEANUP:
                node._transition(P2FState.FAILED)
            node._finish()
        raise SystemExit(130)
    except Exception as exc:
        if not node._finished:
            node._record_internal_exception(exc)
            node._finish()
        raise SystemExit(2)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
