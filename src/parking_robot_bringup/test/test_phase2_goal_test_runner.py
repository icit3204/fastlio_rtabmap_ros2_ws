from pathlib import Path
import math

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
import pytest

from parking_robot_bringup.phase2_goal_test_runner import (
    CommandLimits,
    Phase2GoalTestRunner,
    Pose2D,
    RunnerState,
    VALID_TRANSITIONS,
    analyze_path_against_map,
    build_goal,
    categorize_twist,
    exactly_one_goal_sent,
    goal_status_name,
    load_command_limits,
    max_path_gap,
    normalize_yaw,
    path_length,
    position_error,
    quaternion_from_yaw,
    result_has_required_fields,
    twist_is_finite,
    twist_within_limits,
    unsupported_twist_fields_nonzero,
    validate_transition,
    yaw_error,
    yaw_from_quaternion,
)


PKG = Path(__file__).resolve().parents[1]
PARAMS = PKG / "config" / "phase2_nav2_params.yaml"
MAP_YAML = PKG / "maps" / "phase2_clean_map.yaml"
RUNNER = PKG / "parking_robot_bringup" / "phase2_goal_test_runner.py"


def assert_close(a, b, tol=1e-6):
    assert abs(a - b) <= tol


class _CountingFuture:
    def __init__(self, value=None, exc=None, done=True):
        self.value = value
        self.exc = exc
        self._done = done
        self.result_count = 0

    def done(self):
        return self._done

    def result(self):
        self.result_count += 1
        if self.exc is not None:
            raise self.exc
        return self.value


class _GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.goal_id = None


class _Result:
    def __init__(self, status):
        self.status = status


def _runner_shell():
    runner = object.__new__(Phase2GoalTestRunner)
    runner._start_monotonic = 0.0
    runner._elapsed = lambda: 1.25
    runner.failure_reasons = []
    runner._send_goal_future = None
    runner._goal_handle = None
    runner._result_future = None
    runner._cancel_future = None
    runner._response_processed = False
    runner._result_processed = False
    runner._cancel_response_received = False
    runner._goal_response_received = False
    runner._goal_response_elapsed_sec = None
    runner._goal_response_exception = None
    runner._action_goal_accepted = False
    runner._goal_id = None
    runner._result_response_received = False
    runner._result_response_elapsed_sec = None
    runner._result_exception = None
    runner._action_result_status = None
    runner._action_result_name = None
    return runner


def test_yaw_quaternion_round_trip():
    yaw = normalize_yaw(4.0)
    q = quaternion_from_yaw(yaw)
    assert_close(yaw_from_quaternion(q), yaw)
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    assert_close(norm, 1.0)


def test_position_and_yaw_error():
    assert_close(position_error(Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 4.0, 0.0)), 5.0)
    assert yaw_error(math.pi - 0.1, -math.pi + 0.1) < 0.21


def test_goal_construction():
    pose = Pose2D(0.325, -55.175, 0.0)
    goal = build_goal(pose, "map", Time(sec=1, nanosec=2))
    assert goal.pose.header.frame_id == "map"
    assert goal.pose.header.stamp.sec == 1
    assert_close(goal.pose.pose.position.x, pose.x)
    assert_close(goal.pose.pose.position.y, pose.y)
    assert_close(goal.pose.pose.orientation.w, 1.0)


def test_runner_default_goal_is_authoritative_forward_scenario():
    text = RUNNER.read_text()
    assert 'self.declare_parameter("start_x", 5.425)' in text
    assert 'self.declare_parameter("start_y", -53.725)' in text
    assert 'self.declare_parameter("goal_x", 8.425)' in text
    assert 'self.declare_parameter("goal_y", -53.725)' in text


def test_state_transition_validity():
    validate_transition(RunnerState.IDLE, RunnerState.WAITING_FOR_NAV2)
    validate_transition(RunnerState.VERIFYING_START, RunnerState.GOAL_REQUEST_SENT)
    validate_transition(RunnerState.GOAL_REQUEST_SENT, RunnerState.GOAL_ACTIVE)
    validate_transition(RunnerState.GOAL_ACTIVE, RunnerState.SUCCEEDED)
    validate_transition(RunnerState.SUCCEEDED, RunnerState.CLEANUP)
    with pytest.raises(ValueError):
        validate_transition(RunnerState.VERIFYING_START, RunnerState.GOAL_ACTIVE)
    with pytest.raises(ValueError):
        validate_transition(RunnerState.IDLE, RunnerState.SUCCEEDED)


def test_goal_request_state_is_distinct_from_active_state():
    assert RunnerState.GOAL_REQUEST_SENT.value == "GOAL_REQUEST_SENT"
    assert RunnerState.GOAL_ACTIVE not in VALID_TRANSITIONS[RunnerState.VERIFYING_START]
    assert RunnerState.GOAL_ACTIVE in VALID_TRANSITIONS[RunnerState.GOAL_REQUEST_SENT]


def test_result_status_interpretation():
    assert goal_status_name(GoalStatus.STATUS_SUCCEEDED) == "SUCCEEDED"
    assert goal_status_name(GoalStatus.STATUS_ABORTED) == "ABORTED"
    assert goal_status_name(999) == "UNRECOGNIZED_999"


def test_command_finite_and_unsupported_field_validation():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.2
    assert twist_is_finite(msg)
    assert not unsupported_twist_fields_nonzero(msg)
    msg.linear.y = 0.01
    assert unsupported_twist_fields_nonzero(msg)
    msg.linear.y = 0.0
    msg.angular.z = float("nan")
    assert not twist_is_finite(msg)


def test_command_category_validation():
    msg = Twist()
    assert categorize_twist(msg) == {
        "linear_nonzero": False,
        "angular_nonzero": False,
        "rotation_only": False,
        "translation_command": False,
    }
    msg.angular.z = 0.2
    assert categorize_twist(msg)["rotation_only"]
    assert not categorize_twist(msg)["translation_command"]
    msg.linear.x = 0.1
    category = categorize_twist(msg)
    assert category["linear_nonzero"]
    assert category["angular_nonzero"]
    assert not category["rotation_only"]
    assert category["translation_command"]


def test_configured_limit_validation():
    limits = load_command_limits(PARAMS)
    assert_close(limits.max_abs_linear_x, 0.25)
    assert_close(limits.max_abs_angular_z, 0.8)
    msg = Twist()
    msg.linear.x = 0.25
    msg.angular.z = -0.8
    assert twist_within_limits(msg, limits)
    msg.linear.x = 0.251
    assert not twist_within_limits(msg, limits)


def test_path_length_and_gap_calculation():
    poses = [Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 4.0, 0.0), Pose2D(6.0, 4.0, 0.0)]
    assert_close(path_length(poses), 8.0)
    assert_close(max_path_gap(poses), 5.0)


def test_output_json_schema_helper():
    result = {
        "test_name": "p2d_one_goal",
        "source_revision": "abc",
        "start_pose": {},
        "goal_pose": {},
        "action_goal_accepted": True,
        "action_result": "SUCCEEDED",
        "elapsed_sec": 1.0,
        "final_pose": {},
        "final_position_error_m": 0.1,
        "initialpose_verified": True,
        "global_path_received": True,
        "global_path_pose_count": 2,
        "global_path_length_m": 1.0,
        "command_message_count": 10,
        "nonzero_command_count": 5,
        "max_abs_linear_x": 0.2,
        "max_abs_angular_z": 0.1,
        "invalid_command_count": 0,
        "post_result_stop_latency_sec": 0.0,
        "linear_nonzero_command_count": 1,
        "angular_nonzero_command_count": 1,
        "rotation_only_command_count": 0,
        "translation_command_count": 1,
        "linear_nonzero_command_sec": 1.0,
        "angular_nonzero_command_sec": 1.0,
        "rotation_only_command_sec": 0.0,
        "translation_command_sec": 1.0,
        "cancel_goal_count": 0,
        "cancel_requested": False,
        "cancel_response_received": False,
        "state_transitions": [],
        "pass": True,
        "failure_reasons": [],
        "validation_mode": "navigation",
        "goal_request_sent": True,
        "goal_response_received": True,
        "result_response_received": True,
    }
    assert result_has_required_fields(result)
    del result["goal_pose"]
    assert not result_has_required_fields(result)


def test_exactly_one_goal_per_p2d_execution():
    assert exactly_one_goal_sent(1)
    assert not exactly_one_goal_sent(0)
    assert not exactly_one_goal_sent(2)


def test_path_map_analysis_for_known_free_static_pose_is_free():
    poses = [Pose2D(5.425, -53.725, 0.0)]
    analysis = analyze_path_against_map(poses, MAP_YAML)
    assert analysis["samples"] > 0
    assert analysis["occupied_hits"] == 0
    assert analysis["unknown_hits"] == 0
    assert analysis["out_of_bounds_hits"] == 0


def test_no_phase3_interface_import_or_topic():
    text = RUNNER.read_text()
    forbidden = [
        "RouteMission",
        "MissionState",
        "Mission Manager",
        "/vehicle_cmd_safe",
        "/wheelchair_control_command",
        "/cmd_vel_nav",
        "can0",
    ]
    for term in forbidden:
        assert term not in text


def test_runner_retains_action_lifecycle_objects_as_attributes():
    text = RUNNER.read_text()
    for attr in (
        "self._send_goal_future = None",
        "self._goal_handle = None",
        "self._result_future = None",
        "self._cancel_future = None",
    ):
        assert attr in text
    assert "send_future = self._action_client.send_goal_async" not in text
    assert "goal_handle = send_future.result()" not in text
    assert "result_future = goal_handle.get_result_async()" not in text


def test_runner_uses_explicit_single_threaded_executor():
    text = RUNNER.read_text()
    assert "SingleThreadedExecutor" in text
    assert "executor.add_node(node)" in text
    assert "self._executor.spin_once" in text
    assert "rclpy.spin_once(self" not in text


def test_runner_goal_response_timeout_and_terminal_fields():
    text = RUNNER.read_text()
    assert 'self.declare_parameter("goal_response_timeout_sec", 10.0)' in text
    assert "NavigateToPose goal response timed out" in text
    for field in (
        '"goal_request_sent"',
        '"goal_response_received"',
        '"goal_response_elapsed_sec"',
        '"result_response_received"',
        '"result_response_elapsed_sec"',
        '"goal_response_exception"',
        '"result_exception"',
        '"goal_id"',
    ):
        assert field in text


def test_validation_mode_defaults_and_probe_rules_are_separate():
    text = RUNNER.read_text()
    assert 'self.declare_parameter("validation_mode", "navigation")' in text
    assert '"action_response_probe"' in text
    assert "unsupported validation_mode" in text
    assert "action_response_probe requires identical start and goal" in text
    assert "Fake base translated during action_response_probe" in text
    assert "_evaluate_action_response_probe_pass_fail" in text


def test_navigation_mode_acceptance_still_requires_movement_path_and_commands():
    text = RUNNER.read_text()
    assert "Fake base did not move materially" in text
    assert "No non-empty global path received" in text
    assert "No MPPI nonzero command observed" in text


def test_done_goal_future_is_consumed_before_callback_runs():
    runner = _runner_shell()
    handle = _GoalHandle(accepted=True)
    future = _CountingFuture(value=handle, done=True)
    runner._send_goal_future = future

    Phase2GoalTestRunner._process_goal_response_if_ready(runner)

    assert runner._goal_handle is handle
    assert runner._goal_response_received is True
    assert runner._response_processed is True
    assert future.result_count == 1
    validate_transition(RunnerState.GOAL_REQUEST_SENT, RunnerState.GOAL_ACTIVE)


def test_goal_response_callback_after_helper_is_idempotent():
    runner = _runner_shell()
    future = _CountingFuture(value=_GoalHandle(accepted=True), done=True)
    runner._send_goal_future = future

    Phase2GoalTestRunner._process_goal_response_if_ready(runner)
    Phase2GoalTestRunner._goal_response_cb(runner, future)

    assert future.result_count == 1
    assert runner._goal_response_received is True
    assert runner.failure_reasons == []


def test_goal_response_callback_first_makes_helper_noop():
    runner = _runner_shell()
    future = _CountingFuture(value=_GoalHandle(accepted=True), done=True)
    runner._send_goal_future = future

    Phase2GoalTestRunner._goal_response_cb(runner, future)
    Phase2GoalTestRunner._process_goal_response_if_ready(runner)

    assert future.result_count == 1
    assert runner._goal_response_received is True


def test_done_result_future_is_consumed_before_callback_runs():
    runner = _runner_shell()
    future = _CountingFuture(value=_Result(GoalStatus.STATUS_SUCCEEDED), done=True)
    runner._result_future = future

    Phase2GoalTestRunner._process_result_response_if_ready(runner)

    assert runner._result_response_received is True
    assert runner._action_result_status == GoalStatus.STATUS_SUCCEEDED
    assert runner._action_result_name == "SUCCEEDED"
    assert future.result_count == 1


def test_future_exception_is_recorded_once():
    runner = _runner_shell()
    future = _CountingFuture(exc=RuntimeError("response exploded"), done=True)
    runner._send_goal_future = future

    Phase2GoalTestRunner._process_goal_response_if_ready(runner)
    Phase2GoalTestRunner._goal_response_cb(runner, future)

    assert future.result_count == 1
    assert runner._goal_response_exception is not None
    assert runner._goal_response_received is False
    assert runner._goal_handle is None
    assert len(runner.failure_reasons) == 1


def test_unresolved_response_timeout_remains_possible_after_final_helper_attempt():
    runner = _runner_shell()
    future = _CountingFuture(value=_GoalHandle(accepted=True), done=False)
    runner._send_goal_future = future

    Phase2GoalTestRunner._process_goal_response_if_ready(runner)

    assert runner._response_processed is False
    assert runner._goal_response_received is False
    assert future.result_count == 0
    validate_transition(RunnerState.GOAL_REQUEST_SENT, RunnerState.TIMEOUT)
