"""Phase 2 P2-E sequential-goal and cancellation test runner.

This runner is intentionally scoped to the isolated fake-base Nav2 stack. It
uses the hardened NavigateToPose action-client pattern from the accepted P2-D
runner, but keeps P2-E sequencing and cancellation behavior in a dedicated
entry point so the P2-D runner remains stable.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import json
import math
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
import yaml

from parking_robot_bringup.phase2_goal_test_runner import (
    CommandLimits,
    analyze_path_against_map,
    build_goal,
    build_initialpose,
    categorize_twist,
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


class P2ERunnerState(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    SETTING_START = "SETTING_START"
    VERIFYING_START = "VERIFYING_START"
    GOAL_REQUEST_SENT = "GOAL_REQUEST_SENT"
    GOAL_ACTIVE = "GOAL_ACTIVE"
    CANCEL_REQUEST_SENT = "CANCEL_REQUEST_SENT"
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"
    FAILED = "FAILED"
    CLEANUP = "CLEANUP"


TERMINAL_STATES = {
    P2ERunnerState.SUCCEEDED,
    P2ERunnerState.CANCELED,
    P2ERunnerState.ABORTED,
    P2ERunnerState.TIMEOUT,
    P2ERunnerState.FAILED,
}


VALID_TRANSITIONS = {
    P2ERunnerState.IDLE: {P2ERunnerState.WAITING_FOR_NAV2, P2ERunnerState.FAILED},
    P2ERunnerState.WAITING_FOR_NAV2: {
        P2ERunnerState.SETTING_START,
        P2ERunnerState.TIMEOUT,
        P2ERunnerState.FAILED,
    },
    P2ERunnerState.SETTING_START: {P2ERunnerState.VERIFYING_START, P2ERunnerState.FAILED},
    P2ERunnerState.VERIFYING_START: {
        P2ERunnerState.GOAL_REQUEST_SENT,
        P2ERunnerState.TIMEOUT,
        P2ERunnerState.FAILED,
    },
    P2ERunnerState.GOAL_REQUEST_SENT: {
        P2ERunnerState.GOAL_ACTIVE,
        P2ERunnerState.ABORTED,
        P2ERunnerState.TIMEOUT,
        P2ERunnerState.FAILED,
    },
    P2ERunnerState.GOAL_ACTIVE: {
        P2ERunnerState.GOAL_REQUEST_SENT,
        P2ERunnerState.CANCEL_REQUEST_SENT,
        P2ERunnerState.SUCCEEDED,
        P2ERunnerState.ABORTED,
        P2ERunnerState.TIMEOUT,
        P2ERunnerState.FAILED,
    },
    P2ERunnerState.CANCEL_REQUEST_SENT: {
        P2ERunnerState.CANCELED,
        P2ERunnerState.ABORTED,
        P2ERunnerState.TIMEOUT,
        P2ERunnerState.FAILED,
    },
    P2ERunnerState.SUCCEEDED: {P2ERunnerState.CLEANUP},
    P2ERunnerState.CANCELED: {P2ERunnerState.CLEANUP},
    P2ERunnerState.ABORTED: {P2ERunnerState.CLEANUP},
    P2ERunnerState.TIMEOUT: {P2ERunnerState.CLEANUP},
    P2ERunnerState.FAILED: {P2ERunnerState.CLEANUP},
    P2ERunnerState.CLEANUP: set(),
}


def validate_transition(old: P2ERunnerState, new: P2ERunnerState) -> None:
    if new not in VALID_TRANSITIONS[old]:
        raise ValueError(f"invalid P2-E transition {old.value} -> {new.value}")


def pose_from_mapping(data: dict[str, Any]) -> Pose2D:
    return Pose2D(float(data["x"]), float(data["y"]), float(data["yaw"]))


def bool_finite(value: float | None) -> bool:
    return value is not None and math.isfinite(float(value))


@dataclass
class CommandTracker:
    limits: CommandLimits
    message_count: int = 0
    nonzero_count: int = 0
    first_nonzero_elapsed_sec: float | None = None
    last_nonzero_elapsed_sec: float | None = None
    first_zero_after_cancel_elapsed_sec: float | None = None
    max_abs_linear_x: float = 0.0
    max_abs_angular_z: float = 0.0
    invalid_count: int = 0
    unsupported_nonzero_count: int = 0
    limit_violation_count: int = 0
    samples: list[dict[str, float]] = field(default_factory=list)

    def observe(self, msg: Twist, elapsed: float, cancel_elapsed: float | None = None) -> None:
        self.message_count += 1
        if not twist_is_finite(msg):
            self.invalid_count += 1
            return
        if unsupported_twist_fields_nonzero(msg):
            self.unsupported_nonzero_count += 1
        if not twist_within_limits(msg, self.limits):
            self.limit_violation_count += 1
        self.max_abs_linear_x = max(self.max_abs_linear_x, abs(float(msg.linear.x)))
        self.max_abs_angular_z = max(self.max_abs_angular_z, abs(float(msg.angular.z)))
        nonzero = twist_nonzero(msg)
        if nonzero:
            self.nonzero_count += 1
            if self.first_nonzero_elapsed_sec is None:
                self.first_nonzero_elapsed_sec = elapsed
            self.last_nonzero_elapsed_sec = elapsed
        elif cancel_elapsed is not None and elapsed >= cancel_elapsed and self.first_zero_after_cancel_elapsed_sec is None:
            self.first_zero_after_cancel_elapsed_sec = elapsed
        if len(self.samples) < 5000:
            self.samples.append(
                {
                    "elapsed_sec": elapsed,
                    "linear_x": float(msg.linear.x),
                    "angular_z": float(msg.angular.z),
                    "nonzero": 1.0 if nonzero else 0.0,
                    "linear_nonzero": 1.0 if categorize_twist(msg)["linear_nonzero"] else 0.0,
                    "angular_nonzero": 1.0 if categorize_twist(msg)["angular_nonzero"] else 0.0,
                }
            )


@dataclass
class OdomTracker:
    sample_count: int = 0
    nonfinite_count: int = 0
    last_pose: Pose2D | None = None
    total_translation_m: float = 0.0
    max_abs_linear_x: float = 0.0
    max_abs_angular_z: float = 0.0
    last_nonzero_twist_elapsed_sec: float | None = None
    first_zero_twist_after_cancel_elapsed_sec: float | None = None

    def observe(self, msg: Odometry, elapsed: float, cancel_elapsed: float | None = None) -> None:
        pose = pose_from_odom(msg)
        if not finite_pose(pose):
            self.nonfinite_count += 1
        if self.last_pose is not None:
            self.total_translation_m += position_error(self.last_pose, pose)
        self.last_pose = pose
        lin = abs(float(msg.twist.twist.linear.x))
        ang = abs(float(msg.twist.twist.angular.z))
        self.max_abs_linear_x = max(self.max_abs_linear_x, lin)
        self.max_abs_angular_z = max(self.max_abs_angular_z, ang)
        if lin > 1.0e-6 or ang > 1.0e-6:
            self.last_nonzero_twist_elapsed_sec = elapsed
        elif cancel_elapsed is not None and elapsed >= cancel_elapsed and self.first_zero_twist_after_cancel_elapsed_sec is None:
            self.first_zero_twist_after_cancel_elapsed_sec = elapsed
        self.sample_count += 1


@dataclass
class PathSnapshot:
    timestamp_elapsed_sec: float
    pose_count: int
    length_m: float
    frame: str
    start: dict[str, float] | None
    end: dict[str, float] | None
    endpoint_error_m: float | None
    max_gap_m: float | None
    occupied_hits: int
    unknown_hits: int
    out_of_bounds_hits: int


@dataclass
class GoalRecord:
    index: int
    requested_pose: Pose2D
    robot_pose_at_request: Pose2D | None = None
    request_elapsed_sec: float | None = None
    response_elapsed_sec: float | None = None
    response_latency_sec: float | None = None
    goal_id: str | None = None
    accepted: bool = False
    result_request_elapsed_sec: float | None = None
    result_response_elapsed_sec: float | None = None
    result_status: int | None = None
    result_status_name: str | None = None
    action_duration_sec: float | None = None
    cancellation_count: int = 0
    feedback_count: int = 0
    final_pose: Pose2D | None = None
    final_xy_error_m: float | None = None
    final_yaw_error_rad: float | None = None
    translation_during_leg_m: float = 0.0
    command_count: int = 0
    nonzero_command_count: int = 0
    invalid_commands: int = 0
    unsupported_commands: int = 0
    command_limit_violations: int = 0
    max_abs_linear_x: float = 0.0
    max_abs_angular_z: float = 0.0
    post_result_stop_latency_sec: float | None = None
    progress_failures: int = 0
    bt_recoveries: int = 0
    first_path: PathSnapshot | None = None
    longest_path: PathSnapshot | None = None
    final_path: PathSnapshot | None = None
    failure_reasons: list[str] = field(default_factory=list)


class Phase2SequenceCancelTestRunner(Node):
    def __init__(self, executor: SingleThreadedExecutor | None = None) -> None:
        super().__init__("phase2_sequence_cancel_test_runner")
        self._executor = executor
        self._declare_parameters()

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in {"sequential", "cancel"}:
            raise ValueError(f"unsupported mode: {self.mode}")
        self.scenario_name = str(self.get_parameter("scenario_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.output_result_path = Path(str(self.get_parameter("output_result_path").value))
        self.server_wait_timeout_sec = float(self.get_parameter("server_wait_timeout_sec").value)
        self.initialpose_settle_timeout_sec = float(self.get_parameter("initialpose_settle_timeout_sec").value)
        self.goal_response_timeout_sec = float(self.get_parameter("goal_response_timeout_sec").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.command_stop_timeout_sec = float(self.get_parameter("command_stop_timeout_sec").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.yaw_tolerance_rad = float(self.get_parameter("yaw_tolerance_rad").value)
        self.cancel_timing_tolerance_sec = float(self.get_parameter("cancel_timing_tolerance_sec").value)
        self.post_stop_observation_sec = float(self.get_parameter("post_stop_observation_sec").value)
        self.scenario_path = self._resolve_path("scenario_path", "config/phase2_test_scenarios.yaml")
        self.nav2_params_path = self._resolve_path("nav2_params_path", "config/phase2_nav2_params.yaml")
        self.map_yaml_path = self._resolve_path("map_yaml_path", "maps/phase2_clean_map.yaml")
        self.command_limits = load_command_limits(self.nav2_params_path)

        self.start_pose, self.goals, self.cancel_after_acceptance_sec = self._load_scenario()
        if self.mode == "sequential" and len(self.goals) != 3:
            raise ValueError("sequential mode requires exactly three goals")
        if self.mode == "cancel" and len(self.goals) != 1:
            raise ValueError("cancel mode requires exactly one goal")

        self._start_monotonic = time.monotonic()
        self.state = P2ERunnerState.IDLE
        self.state_transitions: list[dict[str, Any]] = [{"state": self.state.value, "elapsed_sec": 0.0}]
        self.failure_reasons: list[str] = []
        self._finished = False

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_phase2_mock", self._cmd_cb, 50)
        self.create_subscription(NavPath, str(self.get_parameter("global_path_topic").value), self._path_cb, 20)

        self._last_odom: Odometry | None = None
        self._last_cmd: Twist | None = None
        self._initialpose_publish_count = 0
        self._start_verified = False
        self._start_error_m: float | None = None
        self._start_yaw_error_rad: float | None = None
        self._goal_send_count = 0
        self._cancel_request_count = 0
        self._active_goal: GoalRecord | None = None
        self._goal_records: list[GoalRecord] = []
        self._current_command_tracker: CommandTracker | None = None
        self._current_odom_tracker: OdomTracker | None = None
        self._current_path_history: list[PathSnapshot] = []
        self._all_path_rows: list[dict[str, Any]] = []
        self._goal_feedback_count = 0
        self._goal_feedback_by_index: dict[int, int] = {}
        self._latest_cancel_elapsed_sec: float | None = None
        self._cancel_response_elapsed_sec: float | None = None
        self._cancel_response_received = False
        self._cancel_exception: str | None = None
        self._terminal_result_elapsed_sec: float | None = None
        self._translation_after_cancel_m: float | None = None
        self._yaw_change_after_cancel_rad: float | None = None
        self._post_stop_motion_m: float | None = None
        self._cancel_pose: Pose2D | None = None
        self._stop_observation_start_pose: Pose2D | None = None
        self._stop_observation_end_pose: Pose2D | None = None

        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._response_processed = False
        self._result_processed = False
        self._cancel_processed = False
        self._goal_response_exception: str | None = None
        self._result_exception: str | None = None

    def _declare_parameters(self) -> None:
        self.declare_parameter("mode", "sequential")
        self.declare_parameter("scenario_name", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("nav2_params_path", "")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("server_wait_timeout_sec", 30.0)
        self.declare_parameter("initialpose_settle_timeout_sec", 10.0)
        self.declare_parameter("goal_response_timeout_sec", 10.0)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("position_tolerance_m", 0.25)
        self.declare_parameter("yaw_tolerance_rad", 0.50)
        self.declare_parameter("command_stop_timeout_sec", 0.5)
        self.declare_parameter("cancel_timing_tolerance_sec", 0.20)
        self.declare_parameter("post_stop_observation_sec", 1.0)
        self.declare_parameter("output_result_path", "/tmp/phase2_p2e_result.json")
        self.declare_parameter("global_path_topic", "/plan")
        self.declare_parameter("source_revision", "")

    def _resolve_path(self, parameter_name: str, package_relative: str) -> Path:
        value = str(self.get_parameter(parameter_name).value)
        if value:
            return Path(value)
        return Path(get_package_share_directory("parking_robot_bringup")) / package_relative

    def _load_scenario(self) -> tuple[Pose2D, list[Pose2D], float | None]:
        data = yaml.safe_load(self.scenario_path.read_text())
        scenario_name = self.scenario_name or (
            "p2e_sequential_forward" if self.mode == "sequential" else "p2e_cancel_forward"
        )
        if scenario_name not in data:
            raise ValueError(f"scenario not found: {scenario_name}")
        scenario = data[scenario_name]
        start = pose_from_mapping(scenario["initial_pose"])
        if self.mode == "sequential":
            goals = [pose_from_mapping(goal) for goal in scenario["goals"]]
            cancel_after = None
        else:
            goals = [pose_from_mapping(scenario["goal"])]
            cancel_after = float(scenario["cancel_after_acceptance_sec"])
        return start, goals, cancel_after

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_monotonic

    def _transition(self, new: P2ERunnerState) -> None:
        validate_transition(self.state, new)
        self.state = new
        self.state_transitions.append({"state": new.value, "elapsed_sec": self._elapsed()})
        self.get_logger().info(f"P2-E state -> {new.value}")

    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        if self._executor is None:
            raise RuntimeError("Phase2SequenceCancelTestRunner requires one explicit executor")
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

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg
        elapsed = self._elapsed()
        if self._current_odom_tracker is not None:
            self._current_odom_tracker.observe(msg, elapsed, self._latest_cancel_elapsed_sec)

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd = msg
        elapsed = self._elapsed()
        if self._current_command_tracker is not None:
            self._current_command_tracker.observe(msg, elapsed, self._latest_cancel_elapsed_sec)

    def _path_cb(self, msg: NavPath) -> None:
        if self._active_goal is None or len(msg.poses) == 0:
            return
        snapshot = self._path_snapshot(msg, self._active_goal.requested_pose)
        self._current_path_history.append(snapshot)
        self._all_path_rows.append(
            {
                "goal_index": self._active_goal.index,
                "kind": "observed",
                **asdict(snapshot),
            }
        )

    def _path_snapshot(self, msg: NavPath, goal: Pose2D) -> PathSnapshot:
        poses = [
            Pose2D(
                float(p.pose.position.x),
                float(p.pose.position.y),
                yaw_from_quaternion(p.pose.orientation),
            )
            for p in msg.poses
        ]
        analysis = analyze_path_against_map(poses, self.map_yaml_path) if poses else {}
        return PathSnapshot(
            timestamp_elapsed_sec=self._elapsed(),
            pose_count=len(poses),
            length_m=path_length(poses),
            frame=msg.header.frame_id,
            start=asdict(poses[0]) if poses else None,
            end=asdict(poses[-1]) if poses else None,
            endpoint_error_m=position_error(poses[-1], goal) if poses else None,
            max_gap_m=max_path_gap(poses) if poses else None,
            occupied_hits=int(analysis.get("occupied_hits", 0)),
            unknown_hits=int(analysis.get("unknown_hits", 0)),
            out_of_bounds_hits=int(analysis.get("out_of_bounds_hits", 0)),
        )

    def _goal_handle_id(self, goal_handle: Any) -> str | None:
        goal_id = getattr(goal_handle, "goal_id", None)
        uuid = getattr(goal_id, "uuid", None)
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
        assert self._active_goal is not None
        try:
            self._goal_handle = self._send_goal_future.result()
            self._response_processed = True
            self._active_goal.response_elapsed_sec = self._elapsed()
            self._active_goal.response_latency_sec = (
                self._active_goal.response_elapsed_sec - self._active_goal.request_elapsed_sec
                if self._active_goal.request_elapsed_sec is not None
                else None
            )
            if self._goal_handle is not None:
                self._active_goal.goal_id = self._goal_handle_id(self._goal_handle)
                self._active_goal.accepted = bool(self._goal_handle.accepted)
        except Exception as exc:
            self._response_processed = True
            self._goal_response_exception = repr(exc)
            self._active_goal.failure_reasons.append(f"goal response exception: {exc!r}")
            self.failure_reasons.append(f"NavigateToPose goal response exception: {exc!r}")

    def _result_response_cb(self, future: Any) -> None:
        self._process_result_response_if_ready()

    def _process_result_response_if_ready(self) -> None:
        if self._result_future is None or not self._result_future.done() or self._result_processed:
            return
        assert self._active_goal is not None
        try:
            result = self._result_future.result()
            self._result_processed = True
            self._active_goal.result_response_elapsed_sec = self._elapsed()
            self._active_goal.result_status = int(result.status)
            self._active_goal.result_status_name = goal_status_name(result.status)
            self._terminal_result_elapsed_sec = self._active_goal.result_response_elapsed_sec
        except Exception as exc:
            self._result_processed = True
            self._result_exception = repr(exc)
            self._active_goal.failure_reasons.append(f"result exception: {exc!r}")
            self.failure_reasons.append(f"NavigateToPose result exception: {exc!r}")

    def _process_cancel_response_if_ready(self) -> None:
        if self._cancel_future is None or not self._cancel_future.done() or self._cancel_processed:
            return
        try:
            self._cancel_future.result()
            self._cancel_response_received = True
            self._cancel_response_elapsed_sec = self._elapsed()
        except Exception as exc:
            self._cancel_exception = repr(exc)
            self.failure_reasons.append(f"NavigateToPose cancel exception: {exc!r}")
        finally:
            self._cancel_processed = True

    def _feedback_cb(self, feedback_msg) -> None:
        if self._active_goal is None:
            return
        self._active_goal.feedback_count += 1
        self._goal_feedback_count += 1
        self._goal_feedback_by_index[self._active_goal.index] = self._active_goal.feedback_count

    def _reset_action_state_for_goal(self) -> None:
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._response_processed = False
        self._result_processed = False
        self._cancel_processed = False
        self._goal_response_exception = None
        self._result_exception = None
        self._cancel_response_received = False
        self._cancel_exception = None
        self._cancel_response_elapsed_sec = None
        self._latest_cancel_elapsed_sec = None
        self._current_command_tracker = CommandTracker(self.command_limits)
        self._current_odom_tracker = OdomTracker()
        self._current_path_history = []

    def run(self) -> dict[str, Any]:
        self._transition(P2ERunnerState.WAITING_FOR_NAV2)
        if not self._spin_until(lambda: self._last_odom is not None, self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for /Odometry")
            self._transition(P2ERunnerState.TIMEOUT)
            return self._finish()
        if not self._action_client.wait_for_server(timeout_sec=self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for /navigate_to_pose action server")
            self._transition(P2ERunnerState.TIMEOUT)
            return self._finish()
        if not self._set_initial_pose_once():
            return self._finish()
        if self.mode == "sequential":
            return self._run_sequential()
        return self._run_cancel()

    def _set_initial_pose_once(self) -> bool:
        self._transition(P2ERunnerState.SETTING_START)
        stamp = self.get_clock().now().to_msg()
        self._initialpose_pub.publish(build_initialpose(self.start_pose, self.frame_id, stamp))
        self._initialpose_publish_count += 1
        self._transition(P2ERunnerState.VERIFYING_START)

        def start_verified() -> bool:
            if self._last_odom is None:
                return False
            pose = pose_from_odom(self._last_odom)
            self._start_error_m = position_error(pose, self.start_pose)
            self._start_yaw_error_rad = yaw_error(pose.yaw, self.start_pose.yaw)
            stopped = (
                abs(float(self._last_odom.twist.twist.linear.x)) <= 1.0e-6
                and abs(float(self._last_odom.twist.twist.angular.z)) <= 1.0e-6
            )
            return self._start_error_m <= 0.10 and self._start_yaw_error_rad <= 0.05 and stopped

        if not self._spin_until(start_verified, self.initialpose_settle_timeout_sec):
            self.failure_reasons.append("Start pose reset was not verified")
            self._transition(P2ERunnerState.FAILED)
            return False
        self._start_verified = True
        return True

    def _run_sequential(self) -> dict[str, Any]:
        for index, goal in enumerate(self.goals, start=1):
            if not self._send_and_wait_goal(index, goal, cancel_mode=False):
                return self._finish()
            self._observe_command_stop()
            assert self._active_goal is not None
            self._finalize_active_goal()
            if not self._evaluate_goal_record(self._active_goal):
                self._transition(P2ERunnerState.FAILED)
                return self._finish()
            if index < len(self.goals):
                self._transition(P2ERunnerState.GOAL_REQUEST_SENT)
        self._evaluate_sequential()
        if not self.failure_reasons:
            self._transition(P2ERunnerState.SUCCEEDED)
        else:
            self._transition(P2ERunnerState.FAILED)
        return self._finish()

    def _send_and_wait_goal(self, index: int, goal_pose: Pose2D, cancel_mode: bool) -> bool:
        if self.state != P2ERunnerState.GOAL_REQUEST_SENT:
            self._transition(P2ERunnerState.GOAL_REQUEST_SENT)
        self._reset_action_state_for_goal()
        self._goal_send_count += 1
        record = GoalRecord(
            index=index,
            requested_pose=goal_pose,
            robot_pose_at_request=pose_from_odom(self._last_odom) if self._last_odom is not None else None,
            request_elapsed_sec=self._elapsed(),
        )
        self._active_goal = record
        self._goal_records.append(record)
        try:
            self._send_goal_future = self._action_client.send_goal_async(
                build_goal(goal_pose, self.frame_id, self.get_clock().now().to_msg()),
                feedback_callback=self._feedback_cb,
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
        except Exception as exc:
            record.failure_reasons.append(f"send_goal_async failed: {exc!r}")
            self.failure_reasons.append(f"NavigateToPose send_goal_async failed: {exc!r}")
            self._transition(P2ERunnerState.FAILED)
            return False
        if not self._spin_until(
            lambda: self._response_processed or self._goal_response_exception is not None,
            self.goal_response_timeout_sec,
            after_spin=self._process_goal_response_if_ready,
        ):
            self._process_goal_response_if_ready()
            record.failure_reasons.append("goal response timeout")
            self.failure_reasons.append(
                f"Goal {index} response timed out after {self.goal_response_timeout_sec:.3f} sec"
            )
            self._transition(P2ERunnerState.TIMEOUT)
            return False
        if self._goal_response_exception:
            self._transition(P2ERunnerState.FAILED)
            return False
        if self._goal_handle is None or not self._goal_handle.accepted:
            record.failure_reasons.append("goal rejected")
            self.failure_reasons.append(f"Goal {index} was rejected")
            self._transition(P2ERunnerState.ABORTED)
            return False
        self._transition(P2ERunnerState.GOAL_ACTIVE)
        record.result_request_elapsed_sec = self._elapsed()
        try:
            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self._result_response_cb)
        except Exception as exc:
            self._result_exception = repr(exc)
            record.failure_reasons.append(f"get_result_async failed: {exc!r}")
            self.failure_reasons.append(f"NavigateToPose get_result_async failed: {exc!r}")
            self._transition(P2ERunnerState.FAILED)
            return False
        if cancel_mode:
            return self._wait_then_cancel(index)
        return self._wait_for_result(index, self.goal_timeout_sec)

    def _wait_for_result(self, index: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and not self._result_processed:
            self._spin_once(0.05)
            self._process_result_response_if_ready()
        self._process_result_response_if_ready()
        if not self._result_processed:
            assert self._active_goal is not None
            self._active_goal.failure_reasons.append("result timeout")
            self.failure_reasons.append(f"Goal {index} result timed out")
            self._transition(P2ERunnerState.TIMEOUT)
            return False
        return True

    def _wait_then_cancel(self, index: int) -> bool:
        assert self.cancel_after_acceptance_sec is not None
        assert self._active_goal is not None
        acceptance_elapsed = self._active_goal.response_elapsed_sec or self._elapsed()
        cancel_due = acceptance_elapsed + self.cancel_after_acceptance_sec
        deadline = time.monotonic() + self.goal_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and self._elapsed() < cancel_due:
            self._spin_once(0.02)
            self._process_result_response_if_ready()
            if self._result_processed:
                self._active_goal.failure_reasons.append("goal reached terminal result before cancel time")
                self.failure_reasons.append("Cancellation goal reached terminal result before cancel time")
                self._transition(P2ERunnerState.FAILED)
                return False
        if self._current_command_tracker is None or self._current_command_tracker.nonzero_count <= 0:
            self._active_goal.failure_reasons.append("no nonzero command before cancel")
            self.failure_reasons.append("No nonzero command observed before cancellation")
            self._transition(P2ERunnerState.FAILED)
            return False
        self._transition(P2ERunnerState.CANCEL_REQUEST_SENT)
        self._cancel_request_count += 1
        self._active_goal.cancellation_count += 1
        self._latest_cancel_elapsed_sec = self._elapsed()
        self._cancel_pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None
        try:
            self._cancel_future = self._goal_handle.cancel_goal_async()
        except Exception as exc:
            self._cancel_exception = repr(exc)
            self.failure_reasons.append(f"NavigateToPose cancel request failed: {exc!r}")
            self._transition(P2ERunnerState.FAILED)
            return False
        if not self._spin_until(
            lambda: self._cancel_processed,
            5.0,
            after_spin=self._process_cancel_response_if_ready,
        ):
            self._process_cancel_response_if_ready()
            self.failure_reasons.append("Cancel response timed out")
            self._transition(P2ERunnerState.TIMEOUT)
            return False
        return self._wait_for_result(index, self.goal_timeout_sec)

    def _run_cancel(self) -> dict[str, Any]:
        if not self._send_and_wait_goal(1, self.goals[0], cancel_mode=True):
            return self._finish()
        self._observe_command_stop()
        assert self._active_goal is not None
        self._finalize_active_goal()
        self._observe_post_stop_motion()
        self._evaluate_cancel()
        if not self.failure_reasons:
            self._transition(P2ERunnerState.CANCELED)
        else:
            self._transition(P2ERunnerState.FAILED)
        return self._finish()

    def _observe_command_stop(self) -> None:
        start = time.monotonic()
        deadline = start + self.command_stop_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.02)
        if self._active_goal is None or self._current_command_tracker is None:
            return
        result_elapsed = self._terminal_result_elapsed_sec or self._elapsed()
        last_nonzero = self._current_command_tracker.last_nonzero_elapsed_sec
        self._active_goal.post_result_stop_latency_sec = (
            0.0 if last_nonzero is None or last_nonzero <= result_elapsed else last_nonzero - result_elapsed
        )

    def _observe_post_stop_motion(self) -> None:
        self._stop_observation_start_pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None
        deadline = time.monotonic() + self.post_stop_observation_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.05)
        self._stop_observation_end_pose = pose_from_odom(self._last_odom) if self._last_odom is not None else None
        if self._stop_observation_start_pose and self._stop_observation_end_pose:
            self._post_stop_motion_m = position_error(
                self._stop_observation_start_pose, self._stop_observation_end_pose
            )

    def _finalize_active_goal(self) -> None:
        assert self._active_goal is not None
        record = self._active_goal
        if self._last_odom is not None:
            record.final_pose = pose_from_odom(self._last_odom)
            record.final_xy_error_m = position_error(record.final_pose, record.requested_pose)
            record.final_yaw_error_rad = yaw_error(record.final_pose.yaw, record.requested_pose.yaw)
        if record.request_elapsed_sec is not None and record.result_response_elapsed_sec is not None:
            record.action_duration_sec = record.result_response_elapsed_sec - record.request_elapsed_sec
        if self._current_odom_tracker is not None:
            record.translation_during_leg_m = self._current_odom_tracker.total_translation_m
        if self._current_command_tracker is not None:
            record.command_count = self._current_command_tracker.message_count
            record.nonzero_command_count = self._current_command_tracker.nonzero_count
            record.invalid_commands = self._current_command_tracker.invalid_count
            record.unsupported_commands = self._current_command_tracker.unsupported_nonzero_count
            record.command_limit_violations = self._current_command_tracker.limit_violation_count
            record.max_abs_linear_x = self._current_command_tracker.max_abs_linear_x
            record.max_abs_angular_z = self._current_command_tracker.max_abs_angular_z
        if self._current_path_history:
            record.first_path = self._current_path_history[0]
            record.longest_path = max(self._current_path_history, key=lambda p: p.length_m)
            record.final_path = self._current_path_history[-1]

    def _evaluate_goal_record(self, record: GoalRecord) -> bool:
        ok = True
        if not record.accepted:
            record.failure_reasons.append("goal not accepted")
            ok = False
        if record.result_status != GoalStatus.STATUS_SUCCEEDED:
            record.failure_reasons.append(f"result was {record.result_status_name}")
            ok = False
        if not bool_finite(record.final_xy_error_m) or float(record.final_xy_error_m) > self.position_tolerance_m:
            record.failure_reasons.append("final XY error exceeded tolerance")
            ok = False
        if not bool_finite(record.final_yaw_error_rad) or float(record.final_yaw_error_rad) > self.yaw_tolerance_rad:
            record.failure_reasons.append("final yaw error exceeded tolerance")
            ok = False
        representative = record.longest_path or record.first_path
        if representative is None or representative.pose_count <= 0:
            record.failure_reasons.append("no path received")
            ok = False
        else:
            if representative.frame != self.frame_id:
                record.failure_reasons.append("path frame mismatch")
                ok = False
            if representative.occupied_hits > 0 or representative.unknown_hits > 0 or representative.out_of_bounds_hits > 0:
                record.failure_reasons.append("representative path was not map-compatible")
                ok = False
        if record.nonzero_command_count <= 0:
            record.failure_reasons.append("no nonzero command")
            ok = False
        if record.invalid_commands or record.unsupported_commands or record.command_limit_violations:
            record.failure_reasons.append("command stream invalid or outside limits")
            ok = False
        if record.translation_during_leg_m <= 0.05:
            record.failure_reasons.append("no material progress")
            ok = False
        if record.post_result_stop_latency_sec is None or record.post_result_stop_latency_sec > self.command_stop_timeout_sec:
            record.failure_reasons.append("command stop latency exceeded limit")
            ok = False
        if not ok:
            self.failure_reasons.extend(f"Goal {record.index}: {r}" for r in record.failure_reasons)
        return ok

    def _evaluate_sequential(self) -> None:
        if self._initialpose_publish_count != 1:
            self.failure_reasons.append(f"Expected one initialpose, published {self._initialpose_publish_count}")
        if self._goal_send_count != 3:
            self.failure_reasons.append(f"Expected three NavigateToPose requests, sent {self._goal_send_count}")
        if [r.index for r in self._goal_records] != [1, 2, 3]:
            self.failure_reasons.append("Sequential goal records are not in strict order")
        if any(r.cancellation_count for r in self._goal_records):
            self.failure_reasons.append("Unexpected cancellation during sequential mode")

    def _evaluate_cancel(self) -> None:
        record = self._goal_records[0]
        if self._initialpose_publish_count != 1:
            self.failure_reasons.append(f"Expected one initialpose, published {self._initialpose_publish_count}")
        if self._goal_send_count != 1:
            self.failure_reasons.append(f"Expected one NavigateToPose request, sent {self._goal_send_count}")
        if not record.accepted:
            self.failure_reasons.append("Cancellation goal was not accepted")
        if self._cancel_request_count != 1 or record.cancellation_count != 1:
            self.failure_reasons.append("Expected exactly one cancellation request")
        if not self._cancel_response_received:
            self.failure_reasons.append("Cancel response was not received")
        if record.result_status != GoalStatus.STATUS_CANCELED:
            self.failure_reasons.append(f"Expected status 5 CANCELED, got {record.result_status}")
        if record.response_elapsed_sec is None or self._latest_cancel_elapsed_sec is None:
            self.failure_reasons.append("Missing cancellation timing")
        else:
            actual = self._latest_cancel_elapsed_sec - record.response_elapsed_sec
            if abs(actual - float(self.cancel_after_acceptance_sec or 0.0)) > self.cancel_timing_tolerance_sec:
                self.failure_reasons.append(f"Cancel timing out of tolerance: {actual:.3f} sec")
        if self._current_command_tracker is None or self._current_command_tracker.nonzero_count <= 0:
            self.failure_reasons.append("No nonzero command before cancellation")
        if self._current_command_tracker is not None:
            zero_elapsed = self._current_command_tracker.first_zero_after_cancel_elapsed_sec
            if zero_elapsed is None or self._latest_cancel_elapsed_sec is None:
                self.failure_reasons.append("No zero command observed after cancellation")
            elif zero_elapsed - self._latest_cancel_elapsed_sec > 0.5:
                self.failure_reasons.append("Command zero latency exceeded 0.5 sec")
            if (
                self._current_command_tracker.invalid_count
                or self._current_command_tracker.unsupported_nonzero_count
                or self._current_command_tracker.limit_violation_count
            ):
                self.failure_reasons.append("Cancellation command stream invalid or outside limits")
        if self._current_odom_tracker is not None:
            zero_twist = self._current_odom_tracker.first_zero_twist_after_cancel_elapsed_sec
            if zero_twist is None or self._latest_cancel_elapsed_sec is None:
                self.failure_reasons.append("No zero fake-base velocity observed after cancellation")
            elif zero_twist - self._latest_cancel_elapsed_sec > 0.5:
                self.failure_reasons.append("Fake-base velocity zero latency exceeded 0.5 sec")
        if record.post_result_stop_latency_sec is None or record.post_result_stop_latency_sec > self.command_stop_timeout_sec:
            self.failure_reasons.append("Command stop after cancellation exceeded timeout")
        if self._cancel_pose is not None and self._last_odom is not None:
            final = pose_from_odom(self._last_odom)
            self._translation_after_cancel_m = position_error(self._cancel_pose, final)
            self._yaw_change_after_cancel_rad = yaw_error(self._cancel_pose.yaw, final.yaw)
        if self._post_stop_motion_m is not None and self._post_stop_motion_m > 0.02:
            self.failure_reasons.append("Material post-stop odometry motion observed")

    def _finish(self) -> dict[str, Any]:
        if self._finished:
            return self._result_dict()
        self._finished = True
        if self.state in TERMINAL_STATES:
            self._transition(P2ERunnerState.CLEANUP)
        result = self._result_dict()
        self.output_result_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_result_path.write_text(json.dumps(result, indent=2, sort_keys=True))
        return result

    def _result_dict(self) -> dict[str, Any]:
        return {
            "test_name": "phase2_p2e_sequence_cancel",
            "mode": self.mode,
            "scenario_name": self.scenario_name or (
                "p2e_sequential_forward" if self.mode == "sequential" else "p2e_cancel_forward"
            ),
            "source_revision": str(self.get_parameter("source_revision").value),
            "frame_id": self.frame_id,
            "start_pose": asdict(self.start_pose),
            "initialpose_publish_count": self._initialpose_publish_count,
            "initialpose_verified": self._start_verified,
            "initialpose_position_error_m": self._start_error_m,
            "initialpose_yaw_error_rad": self._start_yaw_error_rad,
            "goal_send_count": self._goal_send_count,
            "cancel_request_count": self._cancel_request_count,
            "cancel_after_acceptance_sec": self.cancel_after_acceptance_sec,
            "latest_cancel_elapsed_sec": self._latest_cancel_elapsed_sec,
            "cancel_response_received": self._cancel_response_received,
            "cancel_response_elapsed_sec": self._cancel_response_elapsed_sec,
            "cancel_exception": self._cancel_exception,
            "translation_after_cancel_m": self._translation_after_cancel_m,
            "yaw_change_after_cancel_rad": self._yaw_change_after_cancel_rad,
            "post_stop_motion_m": self._post_stop_motion_m,
            "goals": [self._goal_record_dict(record) for record in self._goal_records],
            "path_history": self._all_path_rows,
            "state_transitions": self.state_transitions,
            "elapsed_sec": self._elapsed(),
            "pass": not self.failure_reasons and self.state in {P2ERunnerState.SUCCEEDED, P2ERunnerState.CANCELED, P2ERunnerState.CLEANUP},
            "decision": self._decision(),
            "failure_reasons": self.failure_reasons,
            "configured_command_limits": asdict(self.command_limits),
        }

    def _goal_record_dict(self, record: GoalRecord) -> dict[str, Any]:
        data = asdict(record)
        data["requested_pose"] = asdict(record.requested_pose)
        data["robot_pose_at_request"] = asdict(record.robot_pose_at_request) if record.robot_pose_at_request else None
        data["final_pose"] = asdict(record.final_pose) if record.final_pose else None
        return data

    def _decision(self) -> str:
        if self.failure_reasons:
            return "P2E_SEQUENTIAL_GOALS_NEEDS_REVIEW" if self.mode == "sequential" else "P2E_CANCELLATION_NEEDS_REVIEW"
        return "P2E_SEQUENTIAL_GOALS_PASS" if self.mode == "sequential" else "P2E_CANCELLATION_PASS"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = Phase2SequenceCancelTestRunner(executor=executor)
    executor.add_node(node)
    try:
        result = node.run()
        if not result.get("pass", False):
            raise SystemExit(2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node._finished:
            node.failure_reasons.append("P2-E runner interrupted before completion")
            if node.state not in TERMINAL_STATES and node.state != P2ERunnerState.CLEANUP:
                node._transition(P2ERunnerState.FAILED)
            node._finish()
        raise SystemExit(130)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
