"""Phase 2 one-goal NavigateToPose test runner.

This is a Phase 2 validation utility. It publishes one start reset on
``/initialpose`` and sends one ``NavigateToPose`` goal to the isolated fake-base
Nav2 stack. It is intentionally limited to the Phase 2 test scope and does not
import or publish any later-phase typed mission interface.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import json
import math
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
import yaml


class RunnerState(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    SETTING_START = "SETTING_START"
    VERIFYING_START = "VERIFYING_START"
    GOAL_REQUEST_SENT = "GOAL_REQUEST_SENT"
    GOAL_ACTIVE = "GOAL_ACTIVE"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"
    FAILED = "FAILED"
    CLEANUP = "CLEANUP"


TERMINAL_STATES = {
    RunnerState.SUCCEEDED,
    RunnerState.ABORTED,
    RunnerState.TIMEOUT,
    RunnerState.FAILED,
}


VALID_TRANSITIONS = {
    RunnerState.IDLE: {RunnerState.WAITING_FOR_NAV2, RunnerState.FAILED},
    RunnerState.WAITING_FOR_NAV2: {RunnerState.SETTING_START, RunnerState.TIMEOUT, RunnerState.FAILED},
    RunnerState.SETTING_START: {RunnerState.VERIFYING_START, RunnerState.FAILED},
    RunnerState.VERIFYING_START: {RunnerState.GOAL_REQUEST_SENT, RunnerState.TIMEOUT, RunnerState.FAILED},
    RunnerState.GOAL_REQUEST_SENT: {RunnerState.GOAL_ACTIVE, RunnerState.ABORTED, RunnerState.TIMEOUT, RunnerState.FAILED},
    RunnerState.GOAL_ACTIVE: {RunnerState.SUCCEEDED, RunnerState.ABORTED, RunnerState.TIMEOUT, RunnerState.FAILED},
    RunnerState.SUCCEEDED: {RunnerState.CLEANUP},
    RunnerState.ABORTED: {RunnerState.CLEANUP},
    RunnerState.TIMEOUT: {RunnerState.CLEANUP},
    RunnerState.FAILED: {RunnerState.CLEANUP},
    RunnerState.CLEANUP: set(),
}

REQUIRED_RESULT_FIELDS = {
    "test_name",
    "source_revision",
    "start_pose",
    "goal_pose",
    "action_goal_accepted",
    "action_result",
    "elapsed_sec",
    "final_pose",
    "final_position_error_m",
    "initialpose_verified",
    "global_path_received",
    "global_path_pose_count",
    "global_path_length_m",
    "command_message_count",
    "nonzero_command_count",
    "max_abs_linear_x",
    "max_abs_angular_z",
    "invalid_command_count",
    "post_result_stop_latency_sec",
    "state_transitions",
    "pass",
    "failure_reasons",
    "validation_mode",
    "goal_request_sent",
    "goal_response_received",
    "result_response_received",
}


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CommandLimits:
    max_abs_linear_x: float
    max_abs_angular_z: float
    tolerance: float = 1.0e-6


@dataclass
class CommandMetrics:
    message_count: int = 0
    nonzero_count: int = 0
    first_nonzero_elapsed_sec: float | None = None
    last_nonzero_elapsed_sec: float | None = None
    max_abs_linear_x: float = 0.0
    max_abs_angular_z: float = 0.0
    invalid_count: int = 0
    unsupported_nonzero_count: int = 0
    limit_violation_count: int = 0
    linear_nonzero_count: int = 0
    angular_nonzero_count: int = 0
    rotation_only_count: int = 0
    translation_command_count: int = 0
    linear_nonzero_sec: float = 0.0
    angular_nonzero_sec: float = 0.0
    rotation_only_sec: float = 0.0
    translation_command_sec: float = 0.0


@dataclass
class OdomMetrics:
    sample_count: int = 0
    nonfinite_count: int = 0
    timestamp_nonmonotonic_count: int = 0
    max_linear_velocity: float = 0.0
    max_angular_velocity: float = 0.0
    total_translation_m: float = 0.0
    min_quaternion_norm: float | None = None
    max_quaternion_norm: float | None = None


def normalize_yaw(yaw: float) -> float:
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_yaw(yaw: float) -> Quaternion:
    yaw = normalize_yaw(yaw)
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    return normalize_yaw(
        math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
    )


def quaternion_norm(q: Quaternion) -> float:
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def position_error(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def yaw_error(a: float, b: float) -> float:
    return abs(normalize_yaw(a - b))


def finite_pose(pose: Pose2D) -> bool:
    return math.isfinite(pose.x) and math.isfinite(pose.y) and math.isfinite(pose.yaw)


def pose_from_odom(msg: Odometry) -> Pose2D:
    p = msg.pose.pose.position
    return Pose2D(float(p.x), float(p.y), yaw_from_quaternion(msg.pose.pose.orientation))


def twist_is_finite(msg: Twist) -> bool:
    values = [
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    ]
    return all(math.isfinite(float(v)) for v in values)


def unsupported_twist_fields_nonzero(msg: Twist, eps: float = 1.0e-9) -> bool:
    return any(
        abs(float(v)) > eps
        for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
    )


def twist_nonzero(msg: Twist, eps: float = 1.0e-6) -> bool:
    return any(
        abs(float(v)) > eps
        for v in (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
    )


def categorize_twist(msg: Twist, eps: float = 1.0e-6) -> dict[str, bool]:
    linear = abs(float(msg.linear.x)) > eps
    angular = abs(float(msg.angular.z)) > eps
    return {
        "linear_nonzero": linear,
        "angular_nonzero": angular,
        "rotation_only": angular and not linear,
        "translation_command": linear,
    }


def twist_within_limits(msg: Twist, limits: CommandLimits) -> bool:
    return (
        abs(float(msg.linear.x)) <= limits.max_abs_linear_x + limits.tolerance
        and abs(float(msg.angular.z)) <= limits.max_abs_angular_z + limits.tolerance
    )


def path_length(poses: list[Pose2D]) -> float:
    return sum(position_error(a, b) for a, b in zip(poses, poses[1:]))


def max_path_gap(poses: list[Pose2D]) -> float:
    if len(poses) < 2:
        return 0.0
    return max(position_error(a, b) for a, b in zip(poses, poses[1:]))


def goal_status_name(status: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(int(status), f"UNRECOGNIZED_{status}")


def result_has_required_fields(result: dict[str, Any]) -> bool:
    return REQUIRED_RESULT_FIELDS <= set(result)


def exactly_one_goal_sent(goal_send_count: int) -> bool:
    return goal_send_count == 1


def build_initialpose(pose: Pose2D, frame_id: str, stamp: Any) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.pose.position.x = pose.x
    msg.pose.pose.position.y = pose.y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
    return msg


def build_goal(pose: Pose2D, frame_id: str, stamp: Any) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = frame_id
    goal.pose.header.stamp = stamp
    goal.pose.pose.position.x = pose.x
    goal.pose.pose.position.y = pose.y
    goal.pose.pose.position.z = 0.0
    goal.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
    return goal


def load_command_limits(params_path: Path) -> CommandLimits:
    data = yaml.safe_load(params_path.read_text())
    follow_path = data["controller_server"]["ros__parameters"]["FollowPath"]
    return CommandLimits(
        max_abs_linear_x=float(max(abs(follow_path.get("vx_min", 0.0)), abs(follow_path["vx_max"]))),
        max_abs_angular_z=float(abs(follow_path["wz_max"])),
    )


def parse_pgm(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"unsupported PGM magic {magic!r}")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = map(int, line.split())
        max_value = int(f.readline())
        data = f.read()
    if len(data) != width * height:
        raise ValueError("PGM data length does not match dimensions")
    return width, height, max_value, data


def world_to_pixel(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    image_height: int,
) -> tuple[int, int]:
    px = int(math.floor((x - origin_x) / resolution))
    map_y = int(math.floor((y - origin_y) / resolution))
    return px, image_height - 1 - map_y


def classify_map_value(value: int, max_value: int, occupied_thresh: float, free_thresh: float, negate: int) -> str:
    if negate:
        occ = float(value) / float(max_value)
    else:
        occ = (float(max_value) - float(value)) / float(max_value)
    if occ >= occupied_thresh:
        return "occupied"
    if occ <= free_thresh:
        return "free"
    return "unknown"


def analyze_path_against_map(
    poses: list[Pose2D],
    map_yaml_path: Path,
    sample_step_m: float | None = None,
) -> dict[str, Any]:
    map_yaml = yaml.safe_load(map_yaml_path.read_text())
    resolution = float(map_yaml["resolution"])
    origin_x = float(map_yaml["origin"][0])
    origin_y = float(map_yaml["origin"][1])
    occupied_thresh = float(map_yaml["occupied_thresh"])
    free_thresh = float(map_yaml["free_thresh"])
    negate = int(map_yaml.get("negate", 0))
    image_path = map_yaml_path.parent / map_yaml["image"]
    width, height, max_value, data = parse_pgm(image_path)
    step = sample_step_m if sample_step_m is not None else max(resolution * 0.5, 0.01)

    occupied_hits = 0
    unknown_hits = 0
    out_of_bounds_hits = 0
    samples = 0

    def classify_point(x: float, y: float) -> None:
        nonlocal occupied_hits, unknown_hits, out_of_bounds_hits, samples
        samples += 1
        px, py = world_to_pixel(x, y, origin_x, origin_y, resolution, height)
        if not (0 <= px < width and 0 <= py < height):
            out_of_bounds_hits += 1
            return
        cls = classify_map_value(data[py * width + px], max_value, occupied_thresh, free_thresh, negate)
        if cls == "occupied":
            occupied_hits += 1
        elif cls == "unknown":
            unknown_hits += 1

    for a, b in zip(poses, poses[1:]):
        dist = position_error(a, b)
        n = max(1, int(math.ceil(dist / step)))
        for i in range(n + 1):
            t = i / n
            classify_point(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
    if len(poses) == 1:
        classify_point(poses[0].x, poses[0].y)

    return {
        "samples": samples,
        "occupied_hits": occupied_hits,
        "unknown_hits": unknown_hits,
        "out_of_bounds_hits": out_of_bounds_hits,
        "map_yaml": str(map_yaml_path),
    }


def validate_transition(current: RunnerState, new: RunnerState) -> None:
    if new not in VALID_TRANSITIONS[current]:
        raise ValueError(f"invalid transition {current.value}->{new.value}")


class Phase2GoalTestRunner(Node):
    """Run one isolated Phase 2 NavigateToPose goal and write JSON evidence."""

    def __init__(self, executor: SingleThreadedExecutor | None = None) -> None:
        super().__init__("phase2_goal_test_runner")
        self._executor = executor
        self._declare_parameters()
        self.start_pose = Pose2D(
            float(self.get_parameter("start_x").value),
            float(self.get_parameter("start_y").value),
            float(self.get_parameter("start_yaw").value),
        )
        self.goal_pose = Pose2D(
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_yaw").value),
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.output_result_path = Path(str(self.get_parameter("output_result_path").value))
        self.test_name = str(self.get_parameter("test_name").value)
        self.server_wait_timeout_sec = float(self.get_parameter("server_wait_timeout_sec").value)
        self.initialpose_settle_timeout_sec = float(
            self.get_parameter("initialpose_settle_timeout_sec").value
        )
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.command_stop_timeout_sec = float(self.get_parameter("command_stop_timeout_sec").value)
        self.goal_response_timeout_sec = float(self.get_parameter("goal_response_timeout_sec").value)
        self.global_path_topic = str(self.get_parameter("global_path_topic").value)
        self.validation_mode = str(self.get_parameter("validation_mode").value)
        if self.validation_mode not in {"navigation", "action_response_probe"}:
            raise ValueError(f"unsupported validation_mode: {self.validation_mode}")

        pkg_share = Path(get_package_share_directory("parking_robot_bringup"))
        self.params_path = Path(
            str(self.get_parameter("nav2_params_path").value)
            or str(pkg_share / "config" / "phase2_nav2_params.yaml")
        )
        self.map_yaml_path = Path(
            str(self.get_parameter("map_yaml_path").value)
            or str(pkg_share / "maps" / "phase2_clean_map.yaml")
        )
        self.command_limits = load_command_limits(self.params_path)
        nav_params = yaml.safe_load(self.params_path.read_text())
        self.goal_checker_xy_tolerance = float(
            nav_params["controller_server"]["ros__parameters"]["general_goal_checker"][
                "xy_goal_tolerance"
            ]
        )
        self.allow_unknown = bool(
            nav_params["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]
        )

        self.state = RunnerState.IDLE
        self.state_transitions: list[dict[str, Any]] = []
        self.failure_reasons: list[str] = []
        self._start_monotonic = time.monotonic()
        self._result_monotonic: float | None = None
        self._goal_send_count = 0
        self._goal_request_sent = False
        self._goal_request_elapsed_sec: float | None = None
        self._goal_response_received = False
        self._goal_response_elapsed_sec: float | None = None
        self._goal_response_exception: str | None = None
        self._result_request_sent = False
        self._result_request_elapsed_sec: float | None = None
        self._result_response_received = False
        self._result_response_elapsed_sec: float | None = None
        self._result_exception: str | None = None
        self._goal_id: str | None = None
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._response_processed = False
        self._result_processed = False
        self._finished = False
        self._goal_feedback: list[dict[str, Any]] = []
        self._last_odom: Odometry | None = None
        self._first_odom_after_reset: Odometry | None = None
        self._settled_odom: Odometry | None = None
        self._path_msg: NavPath | None = None
        self._odom_metrics = OdomMetrics()
        self._last_odom_pose: Pose2D | None = None
        self._last_odom_stamp_ns: int | None = None
        self._command_metrics = CommandMetrics()
        self._initialpose_stamp: dict[str, int] | None = None
        self._start_verified = False
        self._start_error_m: float | None = None
        self._start_yaw_error_rad: float | None = None
        self._settle_duration_sec: float | None = None
        self._final_pose: Pose2D | None = None
        self._final_position_error_m: float | None = None
        self._action_goal_accepted = False
        self._action_result_status: int | None = None
        self._action_result_name: str | None = None
        self._post_result_stop_latency_sec: float | None = None
        self._cancel_requested = False
        self._cancel_response_received = False
        self._cancel_goal_count = 0
        self._last_cmd_elapsed: float | None = None
        self._last_cmd_category: dict[str, bool] | None = None
        self._odom_buckets: list[dict[str, Any]] = []
        self._next_odom_bucket_elapsed = 0.0

        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 50)
        self.create_subscription(Twist, "/cmd_vel_phase2_mock", self._cmd_cb, 50)
        self.create_subscription(NavPath, self.global_path_topic, self._path_cb, 10)
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _declare_parameters(self) -> None:
        self.declare_parameter("start_x", 5.425)
        self.declare_parameter("start_y", -53.725)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("goal_x", 8.425)
        self.declare_parameter("goal_y", -53.725)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("server_wait_timeout_sec", 30.0)
        self.declare_parameter("initialpose_settle_timeout_sec", 10.0)
        self.declare_parameter("goal_response_timeout_sec", 10.0)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("position_tolerance_m", 1.0)
        self.declare_parameter("command_stop_timeout_sec", 5.0)
        self.declare_parameter("output_result_path", "/tmp/phase2_p2d_result.json")
        self.declare_parameter("test_name", "p2d_one_goal")
        self.declare_parameter("global_path_topic", "/plan")
        self.declare_parameter("nav2_params_path", "")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("source_revision", "")
        self.declare_parameter("validation_mode", "navigation")

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_monotonic

    def _transition(self, new: RunnerState) -> None:
        validate_transition(self.state, new)
        self.state = new
        self.state_transitions.append({"state": new.value, "elapsed_sec": self._elapsed()})
        self.get_logger().info(f"test state -> {new.value}")

    def _odom_cb(self, msg: Odometry) -> None:
        pose = pose_from_odom(msg)
        if not finite_pose(pose):
            self._odom_metrics.nonfinite_count += 1
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._last_odom_stamp_ns is not None and stamp_ns < self._last_odom_stamp_ns:
            self._odom_metrics.timestamp_nonmonotonic_count += 1
        self._last_odom_stamp_ns = stamp_ns
        qnorm = quaternion_norm(msg.pose.pose.orientation)
        if self._odom_metrics.min_quaternion_norm is None:
            self._odom_metrics.min_quaternion_norm = qnorm
            self._odom_metrics.max_quaternion_norm = qnorm
        else:
            self._odom_metrics.min_quaternion_norm = min(self._odom_metrics.min_quaternion_norm, qnorm)
            self._odom_metrics.max_quaternion_norm = max(self._odom_metrics.max_quaternion_norm or qnorm, qnorm)
        if self._last_odom_pose is not None:
            self._odom_metrics.total_translation_m += position_error(self._last_odom_pose, pose)
        self._last_odom_pose = pose
        self._odom_metrics.max_linear_velocity = max(
            self._odom_metrics.max_linear_velocity, abs(float(msg.twist.twist.linear.x))
        )
        self._odom_metrics.max_angular_velocity = max(
            self._odom_metrics.max_angular_velocity, abs(float(msg.twist.twist.angular.z))
        )
        self._odom_metrics.sample_count += 1
        self._last_odom = msg
        elapsed = self._elapsed()
        if elapsed >= self._next_odom_bucket_elapsed:
            self._odom_buckets.append(
                {
                    "elapsed_sec": elapsed,
                    "x": pose.x,
                    "y": pose.y,
                    "yaw": pose.yaw,
                    "distance_from_start_m": position_error(pose, self.start_pose),
                    "distance_to_goal_m": position_error(pose, self.goal_pose),
                    "total_translation_m": self._odom_metrics.total_translation_m,
                }
            )
            self._next_odom_bucket_elapsed += 5.0
        if self.state == RunnerState.VERIFYING_START and self._first_odom_after_reset is None:
            self._first_odom_after_reset = msg

    def _cmd_cb(self, msg: Twist) -> None:
        elapsed = self._elapsed()
        if self._last_cmd_elapsed is not None and self._last_cmd_category is not None:
            dt = max(0.0, elapsed - self._last_cmd_elapsed)
            if self._last_cmd_category["linear_nonzero"]:
                self._command_metrics.linear_nonzero_sec += dt
            if self._last_cmd_category["angular_nonzero"]:
                self._command_metrics.angular_nonzero_sec += dt
            if self._last_cmd_category["rotation_only"]:
                self._command_metrics.rotation_only_sec += dt
            if self._last_cmd_category["translation_command"]:
                self._command_metrics.translation_command_sec += dt
        self._command_metrics.message_count += 1
        if not twist_is_finite(msg):
            self._command_metrics.invalid_count += 1
            return
        category = categorize_twist(msg)
        self._last_cmd_elapsed = elapsed
        self._last_cmd_category = category
        if category["linear_nonzero"]:
            self._command_metrics.linear_nonzero_count += 1
        if category["angular_nonzero"]:
            self._command_metrics.angular_nonzero_count += 1
        if category["rotation_only"]:
            self._command_metrics.rotation_only_count += 1
        if category["translation_command"]:
            self._command_metrics.translation_command_count += 1
        if unsupported_twist_fields_nonzero(msg):
            self._command_metrics.unsupported_nonzero_count += 1
        if not twist_within_limits(msg, self.command_limits):
            self._command_metrics.limit_violation_count += 1
        self._command_metrics.max_abs_linear_x = max(
            self._command_metrics.max_abs_linear_x, abs(float(msg.linear.x))
        )
        self._command_metrics.max_abs_angular_z = max(
            self._command_metrics.max_abs_angular_z, abs(float(msg.angular.z))
        )
        if twist_nonzero(msg):
            self._command_metrics.nonzero_count += 1
            if self._command_metrics.first_nonzero_elapsed_sec is None:
                self._command_metrics.first_nonzero_elapsed_sec = elapsed
            self._command_metrics.last_nonzero_elapsed_sec = elapsed

    def _path_cb(self, msg: NavPath) -> None:
        self._path_msg = msg

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

    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        if self._executor is None:
            raise RuntimeError("Phase2GoalTestRunner requires one explicit executor")
        self._executor.spin_once(timeout_sec=timeout_sec)

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
        if self._send_goal_future is None:
            return
        if not self._send_goal_future.done():
            return
        if self._response_processed:
            return
        try:
            self._goal_handle = self._send_goal_future.result()
            self._response_processed = True
            self._goal_response_elapsed_sec = self._elapsed()
            self._goal_response_received = True
            if self._goal_handle is not None:
                self._goal_id = self._goal_handle_id(self._goal_handle)
        except Exception as exc:  # future exceptions are evidence, not hidden success
            self._response_processed = True
            self._goal_response_elapsed_sec = self._elapsed()
            self._goal_response_exception = repr(exc)
            self.failure_reasons.append(f"NavigateToPose goal response exception: {exc!r}")

    def _result_response_cb(self, future: Any) -> None:
        self._process_result_response_if_ready()

    def _process_result_response_if_ready(self) -> None:
        if self._result_future is None:
            return
        if not self._result_future.done():
            return
        if self._result_processed:
            return
        try:
            result = self._result_future.result()
            self._result_processed = True
            self._result_response_elapsed_sec = self._elapsed()
            self._result_response_received = True
            self._action_result_status = int(result.status)
            self._action_result_name = goal_status_name(result.status)
        except Exception as exc:
            self._result_processed = True
            self._result_response_elapsed_sec = self._elapsed()
            self._result_exception = repr(exc)
            self.failure_reasons.append(f"NavigateToPose result exception: {exc!r}")

    def _process_cancel_response_if_ready(self) -> None:
        if self._cancel_future is None or not self._cancel_future.done():
            return
        if self._cancel_response_received:
            return
        try:
            self._cancel_future.result()
            self._cancel_response_received = True
        except Exception as exc:
            self._cancel_response_received = True
            self.failure_reasons.append(f"NavigateToPose cancel exception: {exc!r}")

    def run(self) -> dict[str, Any]:
        self._transition(RunnerState.WAITING_FOR_NAV2)
        if not self._spin_until(lambda: self._last_odom is not None, self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for /Odometry")
            self._transition(RunnerState.TIMEOUT)
            return self._finish()
        if not self._action_client.wait_for_server(timeout_sec=self.server_wait_timeout_sec):
            self.failure_reasons.append("Timed out waiting for /navigate_to_pose action server")
            self._transition(RunnerState.TIMEOUT)
            return self._finish()

        self._transition(RunnerState.SETTING_START)
        stamp = self.get_clock().now().to_msg()
        self._initialpose_stamp = {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}
        self._initialpose_pub.publish(build_initialpose(self.start_pose, self.frame_id, stamp))
        reset_start = time.monotonic()
        self._transition(RunnerState.VERIFYING_START)

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
            if self._start_error_m <= 0.10 and self._start_yaw_error_rad <= 0.05 and stopped:
                self._settled_odom = self._last_odom
                return True
            return False

        if not self._spin_until(start_verified, self.initialpose_settle_timeout_sec):
            self.failure_reasons.append("Start pose reset was not verified")
            self._transition(RunnerState.FAILED)
            return self._finish()
        self._start_verified = True
        self._settle_duration_sec = time.monotonic() - reset_start

        self._transition(RunnerState.GOAL_REQUEST_SENT)
        self._goal_send_count += 1
        self._goal_request_sent = True
        self._goal_request_elapsed_sec = self._elapsed()
        goal = build_goal(self.goal_pose, self.frame_id, self.get_clock().now().to_msg())
        try:
            self._send_goal_future = self._action_client.send_goal_async(
                goal, feedback_callback=self._feedback_cb
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
        except Exception as exc:
            self._goal_response_exception = repr(exc)
            self.failure_reasons.append(f"NavigateToPose send_goal_async failed: {exc!r}")
            self._transition(RunnerState.FAILED)
            return self._finish()
        if not self._spin_until(
            lambda: self._response_processed or self._goal_response_exception is not None,
            self.goal_response_timeout_sec,
            after_spin=self._process_goal_response_if_ready,
        ):
            self._process_goal_response_if_ready()
            self.failure_reasons.append(
                f"NavigateToPose goal response timed out after {self.goal_response_timeout_sec:.3f} sec"
            )
            self._transition(RunnerState.TIMEOUT)
            return self._finish()
        if self._goal_response_exception:
            self._transition(RunnerState.FAILED)
            return self._finish()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self.failure_reasons.append("NavigateToPose goal was rejected")
            self._transition(RunnerState.ABORTED)
            return self._finish()
        self._action_goal_accepted = True
        self._transition(RunnerState.GOAL_ACTIVE)

        self._result_request_sent = True
        self._result_request_elapsed_sec = self._elapsed()
        try:
            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self._result_response_cb)
        except Exception as exc:
            self._result_exception = repr(exc)
            self.failure_reasons.append(f"NavigateToPose get_result_async failed: {exc!r}")
            self._transition(RunnerState.FAILED)
            return self._finish()
        goal_deadline = time.monotonic() + self.goal_timeout_sec
        while rclpy.ok() and time.monotonic() < goal_deadline and not self._result_processed:
            self._spin_once(0.05)
            self._process_result_response_if_ready()
        self._process_result_response_if_ready()
        if not self._result_processed:
            self.failure_reasons.append("NavigateToPose result timed out")
            if self._last_odom is not None:
                self._final_pose = pose_from_odom(self._last_odom)
                self._final_position_error_m = position_error(self._final_pose, self.goal_pose)
            self._cancel_goal_count += 1
            self._cancel_requested = True
            try:
                self._cancel_future = self._goal_handle.cancel_goal_async()
                while rclpy.ok() and not self._cancel_response_received and self._cancel_future is not None:
                    self._spin_once(0.05)
                    self._process_cancel_response_if_ready()
                self._process_cancel_response_if_ready()
            except Exception as exc:
                self.failure_reasons.append(f"NavigateToPose cancel exception: {exc!r}")
            self._result_monotonic = time.monotonic()
            self._observe_command_stop()
            self._transition(RunnerState.TIMEOUT)
            return self._finish()

        self._result_monotonic = time.monotonic()
        if self._last_odom is not None:
            self._final_pose = pose_from_odom(self._last_odom)
            self._final_position_error_m = position_error(self._final_pose, self.goal_pose)

        self._observe_command_stop()

        self._evaluate_pass_fail()
        if not self.failure_reasons:
            self._transition(RunnerState.SUCCEEDED)
        elif self._action_result_status == GoalStatus.STATUS_ABORTED:
            self._transition(RunnerState.ABORTED)
        else:
            self._transition(RunnerState.FAILED)
        return self._finish()

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose.position
        self._goal_feedback.append(
            {
                "elapsed_sec": self._elapsed(),
                "distance_remaining": float(feedback.distance_remaining),
                "current_x": float(pose.x),
                "current_y": float(pose.y),
            }
        )

    def _observe_command_stop(self) -> None:
        if self._result_monotonic is None:
            return
        deadline = time.monotonic() + self.command_stop_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_once(0.05)
        last_nonzero = self._command_metrics.last_nonzero_elapsed_sec
        result_elapsed = self._result_monotonic - self._start_monotonic
        if last_nonzero is None or last_nonzero <= result_elapsed:
            self._post_result_stop_latency_sec = 0.0
        else:
            self._post_result_stop_latency_sec = last_nonzero - result_elapsed

    def _path_metrics(self) -> dict[str, Any]:
        if self._path_msg is None:
            return {
                "received": False,
                "pose_count": 0,
                "length_m": 0.0,
                "finite": False,
                "frame": None,
                "start": None,
                "end": None,
                "max_gap_m": None,
                "map_analysis": None,
                "endpoint_error_m": None,
            }
        poses = [
            Pose2D(
                float(p.pose.position.x),
                float(p.pose.position.y),
                yaw_from_quaternion(p.pose.orientation),
            )
            for p in self._path_msg.poses
        ]
        finite = all(finite_pose(p) for p in poses)
        analysis = analyze_path_against_map(poses, self.map_yaml_path) if poses else None
        endpoint_error = position_error(poses[-1], self.goal_pose) if poses else None
        return {
            "received": True,
            "message_type": "nav_msgs/msg/Path",
            "topic": self.global_path_topic,
            "frame": self._path_msg.header.frame_id,
            "pose_count": len(poses),
            "length_m": path_length(poses),
            "finite": finite,
            "start": asdict(poses[0]) if poses else None,
            "end": asdict(poses[-1]) if poses else None,
            "max_gap_m": max_path_gap(poses),
            "map_analysis": analysis,
            "endpoint_error_m": endpoint_error,
        }

    def _evaluate_pass_fail(self) -> None:
        if self.validation_mode == "action_response_probe":
            self._evaluate_action_response_probe_pass_fail()
            return
        path_metrics = self._path_metrics()
        if not exactly_one_goal_sent(self._goal_send_count):
            self.failure_reasons.append(f"Expected exactly one goal, sent {self._goal_send_count}")
        if not self._start_verified:
            self.failure_reasons.append("Initial pose was not verified")
        if not self._action_goal_accepted:
            self.failure_reasons.append("Action goal was not accepted")
        if self._action_result_status != GoalStatus.STATUS_SUCCEEDED:
            self.failure_reasons.append(f"Action result was {self._action_result_name}")
        if self._final_position_error_m is None or self._final_position_error_m > self.position_tolerance_m:
            self.failure_reasons.append("Final position error exceeded tolerance")
        if self._odom_metrics.nonfinite_count:
            self.failure_reasons.append("Odometry contained non-finite samples")
        if self._odom_metrics.total_translation_m <= 0.1:
            self.failure_reasons.append("Fake base did not move materially")
        if not path_metrics["received"] or path_metrics["pose_count"] <= 0:
            self.failure_reasons.append("No non-empty global path received")
        else:
            if path_metrics["frame"] != self.frame_id:
                self.failure_reasons.append("Global path frame mismatch")
            if not path_metrics["finite"]:
                self.failure_reasons.append("Global path contained non-finite pose")
            if path_metrics["endpoint_error_m"] is None or path_metrics["endpoint_error_m"] > max(
                self.goal_checker_xy_tolerance, 0.5
            ):
                self.failure_reasons.append("Global path endpoint was far from goal")
            map_analysis = path_metrics["map_analysis"] or {}
            if map_analysis.get("occupied_hits", 0) > 0:
                self.failure_reasons.append("Global path crosses occupied cells")
            if not self.allow_unknown and map_analysis.get("unknown_hits", 0) > 0:
                self.failure_reasons.append("Global path crosses unknown cells while allow_unknown=false")
            if map_analysis.get("out_of_bounds_hits", 0) > 0:
                self.failure_reasons.append("Global path leaves map bounds")
        if self._command_metrics.message_count <= 0 or self._command_metrics.nonzero_count <= 0:
            self.failure_reasons.append("No MPPI nonzero command observed")
        if self._command_metrics.invalid_count:
            self.failure_reasons.append("Command stream contained NaN or infinity")
        if self._command_metrics.unsupported_nonzero_count:
            self.failure_reasons.append("Command stream used unsupported Twist fields")
        if self._command_metrics.limit_violation_count:
            self.failure_reasons.append("Command stream exceeded configured limits")
        if (
            self._post_result_stop_latency_sec is None
            or self._post_result_stop_latency_sec > self.command_stop_timeout_sec
        ):
            self.failure_reasons.append("Command did not stop within timeout")

    def _evaluate_action_response_probe_pass_fail(self) -> None:
        if not exactly_one_goal_sent(self._goal_send_count):
            self.failure_reasons.append(f"Expected exactly one goal, sent {self._goal_send_count}")
        if position_error(self.start_pose, self.goal_pose) > 1.0e-9 or yaw_error(
            self.start_pose.yaw, self.goal_pose.yaw
        ) > 1.0e-9:
            self.failure_reasons.append("action_response_probe requires identical start and goal")
        if not self._start_verified:
            self.failure_reasons.append("Initial pose was not verified")
        if not self._goal_response_received:
            self.failure_reasons.append("Goal response was not received")
        if not self._action_goal_accepted:
            self.failure_reasons.append("Action goal was not accepted")
        if not self._result_response_received:
            self.failure_reasons.append("Result response was not received")
        if self._action_result_status != GoalStatus.STATUS_SUCCEEDED:
            self.failure_reasons.append(f"Action result was {self._action_result_name}")
        if self._final_pose is None:
            self.failure_reasons.append("Final pose was unavailable")
        else:
            if position_error(self._final_pose, self.start_pose) > 0.05:
                self.failure_reasons.append("Fake base translated during action_response_probe")
            if yaw_error(self._final_pose.yaw, self.start_pose.yaw) > 0.05:
                self.failure_reasons.append("Fake base yaw changed during action_response_probe")
        if self._command_metrics.invalid_count:
            self.failure_reasons.append("Command stream contained NaN or infinity")
        if self._command_metrics.unsupported_nonzero_count:
            self.failure_reasons.append("Command stream used unsupported Twist fields")
        if self._command_metrics.limit_violation_count:
            self.failure_reasons.append("Command stream exceeded configured limits")
        if (
            self._post_result_stop_latency_sec is None
            or self._post_result_stop_latency_sec > self.command_stop_timeout_sec
        ):
            self.failure_reasons.append("Command did not stop within timeout")

    def _finish(self) -> dict[str, Any]:
        if self._finished:
            return self._result_dict()
        self._finished = True
        if self.state in TERMINAL_STATES:
            self._transition(RunnerState.CLEANUP)
        result = self._result_dict()
        self.output_result_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_result_path.write_text(json.dumps(result, indent=2, sort_keys=True))
        return result

    def _result_dict(self) -> dict[str, Any]:
        path_metrics = self._path_metrics()
        return {
            "test_name": self.test_name,
            "source_revision": str(self.get_parameter("source_revision").value)
            if self.has_parameter("source_revision")
            else "",
            "start_pose": asdict(self.start_pose),
            "goal_pose": asdict(self.goal_pose),
            "frame_id": self.frame_id,
            "validation_mode": self.validation_mode,
            "goal_request_sent": self._goal_request_sent,
            "goal_request_elapsed_sec": self._goal_request_elapsed_sec,
            "goal_response_received": self._goal_response_received,
            "goal_response_elapsed_sec": self._goal_response_elapsed_sec,
            "goal_response_exception": self._goal_response_exception,
            "goal_accepted": self._action_goal_accepted,
            "goal_id": self._goal_id,
            "result_request_sent": self._result_request_sent,
            "result_request_elapsed_sec": self._result_request_elapsed_sec,
            "result_response_received": self._result_response_received,
            "result_response_elapsed_sec": self._result_response_elapsed_sec,
            "result_exception": self._result_exception,
            "action_goal_accepted": self._action_goal_accepted,
            "action_result": self._action_result_name,
            "action_result_status": self._action_result_status,
            "elapsed_sec": self._elapsed(),
            "final_pose": asdict(self._final_pose) if self._final_pose else None,
            "final_position_error_m": self._final_position_error_m,
            "initialpose_verified": self._start_verified,
            "initialpose_publication_stamp": self._initialpose_stamp,
            "first_odom_after_reset": asdict(pose_from_odom(self._first_odom_after_reset))
            if self._first_odom_after_reset
            else None,
            "settled_odom": asdict(pose_from_odom(self._settled_odom)) if self._settled_odom else None,
            "initialpose_position_error_m": self._start_error_m,
            "initialpose_yaw_error_rad": self._start_yaw_error_rad,
            "initialpose_settle_duration_sec": self._settle_duration_sec,
            "global_path_received": path_metrics["received"],
            "global_path_pose_count": path_metrics["pose_count"],
            "global_path_length_m": path_metrics["length_m"],
            "global_path_metrics": path_metrics,
            "command_message_count": self._command_metrics.message_count,
            "nonzero_command_count": self._command_metrics.nonzero_count,
            "max_abs_linear_x": self._command_metrics.max_abs_linear_x,
            "max_abs_angular_z": self._command_metrics.max_abs_angular_z,
            "invalid_command_count": self._command_metrics.invalid_count,
            "unsupported_command_count": self._command_metrics.unsupported_nonzero_count,
            "command_limit_violation_count": self._command_metrics.limit_violation_count,
            "linear_nonzero_command_count": self._command_metrics.linear_nonzero_count,
            "angular_nonzero_command_count": self._command_metrics.angular_nonzero_count,
            "rotation_only_command_count": self._command_metrics.rotation_only_count,
            "translation_command_count": self._command_metrics.translation_command_count,
            "linear_nonzero_command_sec": self._command_metrics.linear_nonzero_sec,
            "angular_nonzero_command_sec": self._command_metrics.angular_nonzero_sec,
            "rotation_only_command_sec": self._command_metrics.rotation_only_sec,
            "translation_command_sec": self._command_metrics.translation_command_sec,
            "first_nonzero_command_elapsed_sec": self._command_metrics.first_nonzero_elapsed_sec,
            "last_nonzero_command_elapsed_sec": self._command_metrics.last_nonzero_elapsed_sec,
            "post_result_stop_latency_sec": self._post_result_stop_latency_sec,
            "configured_command_limits": asdict(self.command_limits),
            "odom_metrics": asdict(self._odom_metrics),
            "odom_buckets": self._odom_buckets,
            "feedback_sample_count": len(self._goal_feedback),
            "feedback_samples": self._goal_feedback[:20],
            "goal_send_count": self._goal_send_count,
            "cancel_goal_count": self._cancel_goal_count,
            "cancel_requested": self._cancel_requested,
            "cancel_response_received": self._cancel_response_received,
            "state_transitions": self.state_transitions,
            "pass": not self.failure_reasons and self.state in {RunnerState.SUCCEEDED, RunnerState.CLEANUP},
            "failure_reasons": self.failure_reasons,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = Phase2GoalTestRunner(executor=executor)
    executor.add_node(node)
    try:
        result = node.run()
        if not result.get("pass", False):
            raise SystemExit(2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node._finished:
            node.failure_reasons.append("Runner interrupted before completion")
            if node.state not in TERMINAL_STATES and node.state != RunnerState.CLEANUP:
                node._transition(RunnerState.FAILED)
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
