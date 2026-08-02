import inspect
import json

import pytest
from action_msgs.msg import GoalStatus

import parking_robot_bringup.phase2_failure_test_runner as p2f_module
from parking_robot_bringup.phase2_failure_test_runner import (
    P2FState,
    Phase2FailureTestRunner,
    StopEvidence,
    TERMINAL_STATES,
    VALID_TRANSITIONS,
    validate_transition,
)
from parking_robot_bringup.phase2_goal_test_runner import Pose2D


def bare_runner(mode="planner_failure"):
    runner = object.__new__(Phase2FailureTestRunner)
    runner.mode = mode
    runner.scenario_name = f"test_{mode}"
    runner.start_pose = Pose2D(0.0, 0.0, 0.0)
    runner.goal_pose = Pose2D(1.0, 0.0, 0.0)
    runner.get_parameter = lambda name: type("P", (), {"value": "test"})()
    runner.failure_reasons = []
    runner._goal_send_count = 1
    runner._cancel_count = 0
    runner._accepted = True
    runner._goal_response_received = False
    runner._result_received = False
    runner._cancel_response_received = False
    runner._goal_request_elapsed_sec = None
    runner._goal_response_elapsed_sec = None
    runner._result_elapsed_sec = None
    runner._goal_id = None
    runner._action_status = GoalStatus.STATUS_ABORTED
    runner._action_status_name = "ABORTED"
    runner._nonzero_cmd_count = 0
    runner._invalid_cmd_count = 0
    runner._unsupported_cmd_count = 0
    runner._cmd_limit_violations = 0
    runner._first_nonzero_cmd_elapsed_sec = None
    runner._last_nonzero_cmd_elapsed_sec = None
    runner._pose_clamp_started = False
    runner._pose_clamp_start_elapsed_sec = None
    runner._pose_clamp_publish_count = 0
    runner._pose_clamp_publish_elapsed_sec = []
    runner._pose_clamp_measured_frequency_hz = None
    runner._natural_result_observation_deadline_sec = 360.0 if mode == "controller_no_progress" else 45.0
    runner._cleanup_cancellation_deadline_sec = 390.0 if mode == "controller_no_progress" else 45.0
    runner._cleanup_cancellation_allowed_elapsed_sec = None
    runner._cancel_request_elapsed_sec = None
    runner._goal_active_monotonic = 100.0
    runner._goal_active_elapsed_sec = 5.0
    runner._natural_result_deadline_monotonic = 460.0 if mode == "controller_no_progress" else 145.0
    runner._cleanup_cancellation_deadline_monotonic = 490.0 if mode == "controller_no_progress" else 145.0
    runner._natural_terminal_result_received_before_cleanup = runner._cancel_count == 0
    runner._fake_base_pid = None
    runner._fake_base_exit_observed = True
    runner._odom_stale_before_goal = True
    runner._tf_stale_before_goal = True
    runner._lifecycle_startup_attempted = False
    runner._lifecycle_startup_success = False
    runner._managed_node_states = {}
    runner._navigate_to_pose_server_available = False
    runner._tf_checks = {
        "map_to_odom": {"available": True, "error": None},
        "odom_to_base": {"available": False, "error": "missing dynamic tf"},
        "map_to_base": {"available": False, "error": "missing dynamic tf"},
    }
    runner._cold_start_final_classification = None
    runner._shutdown_result = None
    runner._runner_exception_type = None
    runner._runner_exception_message = None
    runner._runner_exception_state = None
    runner._runner_exception_traceback = None
    runner._initialpose_publish_count = 0
    runner._start_verified = False
    runner._start_error_m = None
    runner._start_yaw_error_rad = None
    runner._odom_count = 0
    runner._cmd_count = 0
    runner._max_abs_linear_x = 0.0
    runner._max_abs_angular_z = 0.0
    runner._path_message_count = 0
    runner._paths = []
    runner.state = P2FState.ABORTED
    runner.state_transitions = [{"state": runner.state.value, "elapsed_sec": 0.0}]
    runner._finished = False
    runner._elapsed = lambda: 1.0
    runner.get_logger = lambda: type("Logger", (), {"info": lambda self, msg: None})()
    runner.stop_evidence = StopEvidence(
        failure_elapsed_sec=10.0,
        first_zero_command_after_failure_elapsed_sec=10.1,
        command_stop_latency_sec=0.1,
        last_nonzero_command_elapsed_sec=9.9,
        first_zero_twist_after_failure_elapsed_sec=10.1,
        odom_velocity_stop_latency_sec=0.1,
        last_nonzero_odom_twist_elapsed_sec=9.9,
        translation_after_failure_m=0.0,
        yaw_change_after_failure_rad=0.0,
        post_stop_motion_m=0.0,
    )
    return runner


class FakeFuture:
    def __init__(self, value=None, exc=None, done=True):
        self.value = value
        self.exc = exc
        self._done = done
        self.result_calls = 0

    def done(self):
        return self._done

    def result(self):
        self.result_calls += 1
        if self.exc is not None:
            raise self.exc
        return self.value


class FakeGoalHandle:
    def __init__(self, accepted=True, uuid=(1, 2, 3)):
        self.accepted = accepted
        self.goal_id = type("GoalID", (), {"uuid": uuid})()


class FakeResult:
    def __init__(self, status):
        self.status = status


def bare_action_runner():
    runner = object.__new__(Phase2FailureTestRunner)
    runner._send_goal_future = None
    runner._goal_handle = None
    runner._result_future = None
    runner._cancel_future = None
    runner._response_processed = False
    runner._result_processed = False
    runner._cancel_processed = False
    runner._goal_response_received = False
    runner._result_received = False
    runner._cancel_response_received = False
    runner._goal_response_exception = None
    runner._result_exception = None
    runner._cancel_exception = None
    runner._goal_response_elapsed_sec = None
    runner._result_elapsed_sec = None
    runner._cancel_response_elapsed_sec = None
    runner._goal_request_elapsed_sec = None
    runner._result_request_elapsed_sec = None
    runner._cancel_request_elapsed_sec = None
    runner._accepted = False
    runner._goal_id = None
    runner._action_status = None
    runner._action_status_name = None
    runner.failure_reasons = []
    runner._elapsed = lambda: 1.25
    return runner


def test_supported_failure_modes_and_rejection_contract():
    source = inspect.getsource(Phase2FailureTestRunner)
    assert "planner_failure" in source
    assert "controller_no_progress" in source
    assert "tf_loss" in source
    assert "cold_start_missing_tf" in source
    assert "action_response_probe" in source
    assert "unsupported mode" in source
    assert "rclpy.spin_once" not in source
    assert "SingleThreadedExecutor" in source


def test_goal_active_requires_goal_request_sent():
    assert P2FState.GOAL_ACTIVE not in VALID_TRANSITIONS[P2FState.VERIFYING_START]
    assert P2FState.GOAL_ACTIVE in VALID_TRANSITIONS[P2FState.GOAL_REQUEST_SENT]
    validate_transition(P2FState.GOAL_REQUEST_SENT, P2FState.GOAL_ACTIVE)
    with pytest.raises(ValueError):
        validate_transition(P2FState.VERIFYING_START, P2FState.GOAL_ACTIVE)


def test_cold_start_waiting_for_nav2_to_pre_fault_is_valid():
    assert P2FState.PRE_FAULT in VALID_TRANSITIONS[P2FState.WAITING_FOR_NAV2]
    validate_transition(P2FState.WAITING_FOR_NAV2, P2FState.PRE_FAULT)


def test_unrelated_invalid_transitions_remain_rejected():
    assert P2FState.GOAL_ACTIVE not in VALID_TRANSITIONS[P2FState.WAITING_FOR_NAV2]
    assert P2FState.CANCEL_REQUEST_SENT not in VALID_TRANSITIONS[P2FState.PRE_FAULT]
    with pytest.raises(ValueError):
        validate_transition(P2FState.WAITING_FOR_NAV2, P2FState.GOAL_ACTIVE)
    with pytest.raises(ValueError):
        validate_transition(P2FState.PRE_FAULT, P2FState.CANCEL_REQUEST_SENT)


def test_future_done_before_goal_callback_is_consumed_by_helper():
    runner = bare_action_runner()
    goal_handle = FakeGoalHandle(accepted=True)
    runner._send_goal_future = FakeFuture(goal_handle)

    Phase2FailureTestRunner._process_goal_response_if_ready(runner)

    assert runner._goal_handle is goal_handle
    assert runner._goal_response_received is True
    assert runner._accepted is True
    assert runner._goal_id == "010203"
    assert runner._response_processed is True
    assert runner._send_goal_future.result_calls == 1


def test_goal_callback_after_helper_is_idempotent_noop():
    runner = bare_action_runner()
    runner._send_goal_future = FakeFuture(FakeGoalHandle(accepted=True))

    Phase2FailureTestRunner._process_goal_response_if_ready(runner)
    Phase2FailureTestRunner._goal_response_cb(runner, runner._send_goal_future)

    assert runner._send_goal_future.result_calls == 1
    assert runner._goal_response_received is True


def test_goal_callback_first_then_helper_is_idempotent_noop():
    runner = bare_action_runner()
    runner._send_goal_future = FakeFuture(FakeGoalHandle(accepted=True))

    Phase2FailureTestRunner._goal_response_cb(runner, runner._send_goal_future)
    Phase2FailureTestRunner._process_goal_response_if_ready(runner)

    assert runner._send_goal_future.result_calls == 1
    assert runner._goal_response_received is True


def test_result_future_done_before_callback_is_consumed_by_helper():
    runner = bare_action_runner()
    runner._result_future = FakeFuture(FakeResult(GoalStatus.STATUS_SUCCEEDED))

    Phase2FailureTestRunner._process_result_response_if_ready(runner)
    Phase2FailureTestRunner._result_response_cb(runner, runner._result_future)

    assert runner._result_future.result_calls == 1
    assert runner._result_received is True
    assert runner._action_status == GoalStatus.STATUS_SUCCEEDED
    assert runner._action_status_name == "SUCCEEDED"


def test_future_exception_is_recorded_once():
    runner = bare_action_runner()
    runner._send_goal_future = FakeFuture(exc=RuntimeError("boom"))

    Phase2FailureTestRunner._process_goal_response_if_ready(runner)
    Phase2FailureTestRunner._goal_response_cb(runner, runner._send_goal_future)

    assert runner._send_goal_future.result_calls == 1
    assert runner._goal_response_received is False
    assert runner._goal_response_exception is not None
    assert len(runner.failure_reasons) == 1


def test_planner_failure_requires_status_6_and_zero_nonzero_commands():
    runner = bare_runner("planner_failure")
    Phase2FailureTestRunner._evaluate(runner)
    assert runner.failure_reasons == []

    runner = bare_runner("planner_failure")
    runner._action_status = GoalStatus.STATUS_CANCELED
    Phase2FailureTestRunner._evaluate(runner)
    assert any("expected ABORTED status 6" in reason for reason in runner.failure_reasons)

    runner = bare_runner("planner_failure")
    runner._nonzero_cmd_count = 1
    Phase2FailureTestRunner._evaluate(runner)
    assert any("nonzero commands" in reason for reason in runner.failure_reasons)


def test_controller_no_progress_requires_nonzero_command_before_clamp():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = None
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 1
    Phase2FailureTestRunner._evaluate(runner)
    assert any("no nonzero command" in reason for reason in runner.failure_reasons)


def test_controller_no_progress_requires_clamp_started():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 2.0
    runner._pose_clamp_started = False
    runner._pose_clamp_publish_count = 0
    Phase2FailureTestRunner._evaluate(runner)
    assert any("pose clamp did not start" in reason for reason in runner.failure_reasons)


def test_controller_no_progress_requires_clamp_timestamps_for_each_publish():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 2.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [2.1]

    Phase2FailureTestRunner._evaluate(runner)

    assert any("timestamp count" in reason for reason in runner.failure_reasons)


def test_bare_runner_fixture_matches_constructor_cancel_default():
    runner = bare_runner("controller_no_progress")
    assert runner._cancel_count == 0


def test_production_constructor_initializes_cancel_count():
    source = inspect.getsource(Phase2FailureTestRunner.__init__)
    assert "self._cancel_count = 0" in source


def test_pose_clamp_frequency_is_computed_from_exact_timestamps():
    runner = bare_runner("controller_no_progress")
    runner._pose_clamp_publish_elapsed_sec = [2.0, 2.1, 2.2, 2.3]

    Phase2FailureTestRunner._update_pose_clamp_frequency(runner)

    assert runner._pose_clamp_measured_frequency_hz == pytest.approx(10.0)


def test_controller_no_progress_serializes_clamp_timestamps_and_frequency():
    runner = bare_runner("controller_no_progress")
    runner.scenario_name = "p2f_controller_no_progress"
    runner.get_parameter = lambda name: type("P", (), {"value": ""})()
    runner.start_pose = Pose2D(0.0, 0.0, 0.0)
    runner.goal_pose = Pose2D(1.0, 0.0, 0.0)
    runner._initialpose_publish_count = 1
    runner._start_verified = True
    runner._start_error_m = 0.0
    runner._start_yaw_error_rad = 0.0
    runner._goal_response_received = True
    runner._goal_request_elapsed_sec = 1.0
    runner._goal_response_elapsed_sec = 1.1
    runner._goal_id = "abc"
    runner._result_received = True
    runner._result_elapsed_sec = 10.0
    runner._cancel_count = 0
    runner._cancel_request_elapsed_sec = None
    runner._cancel_response_received = False
    runner._goal_active_elapsed_sec = 2.0
    runner._natural_result_deadline_monotonic = 362.0
    runner._cleanup_cancellation_deadline_monotonic = 392.0
    runner._natural_terminal_result_received_before_cleanup = True
    runner.state_transitions = []
    runner._paths = []
    runner._cmd_count = 2
    runner._max_abs_linear_x = 0.1
    runner._max_abs_angular_z = 0.0
    runner._odom_count = 2
    runner._tf_stale_before_goal = False
    runner._pose_clamp_started = True
    runner._pose_clamp_start_elapsed_sec = 2.0
    runner._pose_clamp_publish_count = 4
    runner._pose_clamp_publish_elapsed_sec = [2.0, 2.1, 2.2, 2.3]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner.failure_reasons = []

    result = Phase2FailureTestRunner._result_dict(runner)

    assert result["pose_clamp_publish_elapsed_sec"] == [2.0, 2.1, 2.2, 2.3]
    assert result["pose_clamp_measured_frequency_hz"] == 10.0
    assert result["natural_result_observation_deadline_sec"] == 360.0
    assert result["cleanup_cancellation_deadline_sec"] == 390.0
    assert result["goal_active_elapsed_sec"] == 2.0
    assert result["natural_result_deadline_elapsed_sec"] == 362.0
    assert result["cleanup_cancellation_deadline_elapsed_sec"] == 392.0
    assert result["cleanup_cancel_sent"] is False
    assert result["cleanup_cancel_count"] == 0
    assert result["natural_terminal_result_received_before_cleanup"] is True


def test_controller_no_progress_deadlines_are_anchored_to_goal_active(monkeypatch):
    runner = bare_runner("controller_no_progress")
    ticks = iter([200.0, 200.0])
    monkeypatch.setattr(p2f_module.time, "monotonic", lambda: next(ticks))
    runner._start_monotonic = 150.0
    runner._elapsed = lambda: p2f_module.time.monotonic() - runner._start_monotonic

    Phase2FailureTestRunner._mark_goal_active_time(runner)

    assert runner._goal_active_monotonic == 200.0
    assert runner._goal_active_elapsed_sec == 50.0
    assert runner._natural_result_deadline_monotonic == 560.0
    assert runner._cleanup_cancellation_deadline_monotonic == 590.0


def test_runner_startup_delay_does_not_shorten_cleanup_after_goal_active(monkeypatch):
    runner = bare_runner("controller_no_progress")
    ticks = iter([1000.0, 1000.0])
    monkeypatch.setattr(p2f_module.time, "monotonic", lambda: next(ticks))
    runner._start_monotonic = 10.0
    runner._elapsed = lambda: p2f_module.time.monotonic() - runner._start_monotonic

    Phase2FailureTestRunner._mark_goal_active_time(runner)

    assert runner._cleanup_cancellation_deadline_monotonic - runner._goal_active_monotonic == 390.0


def test_natural_aborted_before_360_after_goal_active_passes_without_cancel():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 6.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [6.0, 6.1]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner._goal_active_elapsed_sec = 5.0
    runner._result_elapsed_sec = 364.9
    runner._cancel_count = 0
    runner._action_status = GoalStatus.STATUS_ABORTED

    Phase2FailureTestRunner._evaluate(runner)

    assert runner._result_elapsed_sec - runner._goal_active_elapsed_sec == pytest.approx(359.9)
    assert runner.failure_reasons == []


def test_natural_aborted_between_360_and_390_after_goal_active_passes_without_cancel():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 6.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [6.0, 6.1]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner._goal_active_elapsed_sec = 5.0
    runner._result_elapsed_sec = 380.0
    runner._cancel_count = 0
    runner._action_status = GoalStatus.STATUS_ABORTED

    Phase2FailureTestRunner._evaluate(runner)

    assert runner._result_elapsed_sec - runner._goal_active_elapsed_sec == pytest.approx(375.0)
    assert runner.failure_reasons == []


def test_controller_no_progress_cleanup_status_5_cannot_pass():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 2.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [2.1, 2.2]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner._cancel_count = 1
    runner._action_status = GoalStatus.STATUS_CANCELED
    runner._goal_active_elapsed_sec = 5.0
    runner._cancel_request_elapsed_sec = 395.1

    Phase2FailureTestRunner._evaluate(runner)

    assert any("cleanup-generated CANCELED status 5" in reason for reason in runner.failure_reasons)


def test_controller_no_progress_cleanup_cannot_occur_before_390_seconds():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 2.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [2.1, 2.2]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner._cancel_count = 1
    runner._goal_active_elapsed_sec = 5.0
    runner._cancel_request_elapsed_sec = 394.9

    Phase2FailureTestRunner._evaluate(runner)

    assert any("before 390 sec after GOAL_ACTIVE" in reason for reason in runner.failure_reasons)


def test_cleanup_may_occur_at_or_after_390_only_without_result():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 2.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [2.1, 2.2]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner._cancel_count = 1
    runner._goal_active_elapsed_sec = 5.0
    runner._cancel_request_elapsed_sec = 395.0
    runner._action_status = GoalStatus.STATUS_ABORTED

    Phase2FailureTestRunner._evaluate(runner)

    assert not any("before 390 sec" in reason for reason in runner.failure_reasons)


def test_cancel_timing_must_be_provable():
    runner = bare_runner("controller_no_progress")
    runner._first_nonzero_cmd_elapsed_sec = 2.0
    runner._pose_clamp_started = True
    runner._pose_clamp_publish_count = 2
    runner._pose_clamp_publish_elapsed_sec = [2.1, 2.2]
    runner._pose_clamp_measured_frequency_hz = 10.0
    runner._cancel_count = 1
    runner._goal_active_elapsed_sec = None

    Phase2FailureTestRunner._evaluate(runner)

    assert any("cannot be proven" in reason for reason in runner.failure_reasons)


def test_planner_tf_loss_and_probe_deadlines_remain_default():
    for mode in ("planner_failure", "tf_loss", "cold_start_missing_tf", "action_response_probe"):
        runner = bare_runner(mode)
        assert runner._natural_result_observation_deadline_sec == 45.0
        assert runner._cleanup_cancellation_deadline_sec == 45.0


def test_tf_loss_requires_staleness_before_goal():
    runner = bare_runner("tf_loss")
    runner._accepted = False
    runner._nonzero_cmd_count = 0
    runner._fake_base_exit_observed = False
    runner._odom_stale_before_goal = False
    Phase2FailureTestRunner._evaluate(runner)
    assert any("staleness was not proven" in reason for reason in runner.failure_reasons)


def test_tf_loss_timeout_disposition_is_bounded_and_accepted():
    runner = bare_runner("tf_loss")
    runner._accepted = True
    runner._action_status = None
    runner.state = P2FState.TIMEOUT
    runner._nonzero_cmd_count = 0
    Phase2FailureTestRunner._evaluate(runner)
    assert runner.failure_reasons == []


def test_action_response_probe_does_not_weaken_failure_mode_decisions():
    source = inspect.getsource(Phase2FailureTestRunner._evaluate)
    assert 'self.mode == "planner_failure"' in source
    assert 'self.mode == "controller_no_progress"' in source
    assert 'self.mode == "tf_loss"' in source
    assert "action-response probe expected SUCCEEDED" in source
    assert "planner failure produced nonzero commands" in source


def test_one_cancellation_maximum_is_source_contract():
    source = inspect.getsource(Phase2FailureTestRunner)
    assert "self._cancel_count += 1" in source
    assert source.count("cancel_goal_async") == 1


def test_terminal_result_serializes_exact_stop_timestamps():
    runner = bare_runner("controller_no_progress")
    runner.scenario_name = "p2f_controller_no_progress"
    runner.get_parameter = lambda name: type("P", (), {"value": ""})()
    runner.start_pose = Pose2D(0.0, 0.0, 0.0)
    runner.goal_pose = Pose2D(1.0, 0.0, 0.0)
    runner._initialpose_publish_count = 1
    runner._start_verified = True
    runner._start_error_m = 0.0
    runner._start_yaw_error_rad = 0.0
    runner._goal_response_received = True
    runner._goal_request_elapsed_sec = 1.0
    runner._goal_response_elapsed_sec = 1.1
    runner._goal_id = "abc"
    runner._result_received = True
    runner._result_elapsed_sec = 10.0
    runner._cancel_count = 0
    runner._cancel_response_received = False
    runner.state_transitions = []
    runner._paths = []
    runner._cmd_count = 2
    runner._max_abs_linear_x = 0.1
    runner._max_abs_angular_z = 0.0
    runner._odom_count = 2
    runner._tf_stale_before_goal = False
    runner.failure_reasons = []

    result = Phase2FailureTestRunner._result_dict(runner)
    assert result["first_zero_command_after_failure_elapsed_sec"] == 10.1
    assert result["first_zero_odom_twist_after_failure_elapsed_sec"] == 10.1
    assert result["command_stop_latency_sec"] == 0.1
    assert result["odometry_velocity_stop_latency_sec"] == 0.1


def test_first_longest_final_path_tracking_contract():
    source = inspect.getsource(Phase2FailureTestRunner)
    assert 'self._path_dict("first")' in source
    assert 'self._path_dict("longest")' in source
    assert 'self._path_dict("final")' in source


def test_no_physical_command_topics_in_runner_source():
    source = inspect.getsource(Phase2FailureTestRunner)
    assert "/cmd_vel_phase2_mock" in source
    for forbidden in ("/vehicle_cmd_safe", "/wheelchair_control_command", "/cmd_vel_nav"):
        assert forbidden not in source


def cold_start_runner():
    runner = bare_runner("cold_start_missing_tf")
    runner._goal_send_count = 0
    runner._accepted = False
    runner._action_status = None
    runner._action_status_name = None
    runner.state = P2FState.TIMEOUT
    runner._lifecycle_startup_attempted = True
    runner._lifecycle_startup_success = False
    runner._navigate_to_pose_server_available = False
    runner._natural_terminal_result_received_before_cleanup = False
    return runner


class FakeActionClient:
    def __init__(self, available):
        self.available = available

    def wait_for_server(self, timeout_sec):
        return self.available


def runnable_cold_start_runner(server_available=True, lifecycle_active=True):
    runner = cold_start_runner()
    runner.state = P2FState.IDLE
    runner.state_transitions = [{"state": P2FState.IDLE.value, "elapsed_sec": 0.0}]
    runner._lifecycle_startup_success = lifecycle_active
    runner._navigate_to_pose_server_available = server_available
    runner._action_client = FakeActionClient(server_available)
    runner.server_wait_timeout_sec = 0.01
    runner._query_managed_node_states = lambda: None
    runner._all_managed_nodes_active = lambda: lifecycle_active
    runner._spin_for = lambda timeout_sec: None
    runner._verify_cold_start_tf_absence = lambda: None
    runner._mark_failure_time = lambda: None
    runner._observe_stop_and_post_motion = lambda: None

    def finish():
        if runner.state in TERMINAL_STATES:
            Phase2FailureTestRunner._transition(runner, P2FState.CLEANUP)
        return Phase2FailureTestRunner._result_dict(runner)

    runner._finish = finish
    return runner


def transition_states(runner):
    return [entry["state"] for entry in runner.state_transitions]


def test_cold_start_server_unavailable_path_reaches_cleanup():
    runner = runnable_cold_start_runner(server_available=False, lifecycle_active=True)

    result = Phase2FailureTestRunner.run(runner)

    assert transition_states(runner) == [
        "IDLE",
        "WAITING_FOR_NAV2",
        "PRE_FAULT",
        "TIMEOUT",
        "CLEANUP",
    ]
    assert result["navigate_to_pose_request_count"] == 0


def test_cold_start_goal_rejected_path_executes_through_pre_fault():
    runner = runnable_cold_start_runner()

    def send_goal():
        Phase2FailureTestRunner._transition(runner, P2FState.GOAL_REQUEST_SENT)
        Phase2FailureTestRunner._transition(runner, P2FState.REJECTED)
        return runner._finish()

    runner._send_goal_and_observe = send_goal

    Phase2FailureTestRunner.run(runner)

    assert transition_states(runner) == ["IDLE", "WAITING_FOR_NAV2", "PRE_FAULT", "GOAL_REQUEST_SENT", "REJECTED", "CLEANUP"]


def test_cold_start_accepted_aborted_path_executes_through_pre_fault():
    runner = runnable_cold_start_runner()

    def send_goal():
        runner._goal_send_count = 1
        runner._accepted = True
        runner._action_status = GoalStatus.STATUS_ABORTED
        Phase2FailureTestRunner._transition(runner, P2FState.GOAL_REQUEST_SENT)
        Phase2FailureTestRunner._transition(runner, P2FState.GOAL_ACTIVE)
        Phase2FailureTestRunner._transition(runner, P2FState.ABORTED)
        return runner._finish()

    runner._send_goal_and_observe = send_goal

    Phase2FailureTestRunner.run(runner)

    assert "PRE_FAULT" in transition_states(runner)
    assert transition_states(runner)[-2:] == ["ABORTED", "CLEANUP"]


def test_cold_start_timeout_cleanup_cancel_path_has_one_cancel():
    runner = runnable_cold_start_runner()

    def send_goal():
        runner._goal_send_count = 1
        runner._accepted = True
        runner._action_status = GoalStatus.STATUS_CANCELED
        runner._cancel_count = 1
        Phase2FailureTestRunner._transition(runner, P2FState.GOAL_REQUEST_SENT)
        Phase2FailureTestRunner._transition(runner, P2FState.GOAL_ACTIVE)
        Phase2FailureTestRunner._transition(runner, P2FState.CANCEL_REQUEST_SENT)
        Phase2FailureTestRunner._transition(runner, P2FState.TIMEOUT)
        return runner._finish()

    runner._send_goal_and_observe = send_goal

    Phase2FailureTestRunner.run(runner)

    assert "PRE_FAULT" in transition_states(runner)
    assert runner._cancel_count == 1
    assert transition_states(runner)[-2:] == ["TIMEOUT", "CLEANUP"]


def runnable_existing_mode_runner(mode):
    runner = bare_runner(mode)
    runner.state = P2FState.IDLE
    runner.state_transitions = [{"state": P2FState.IDLE.value, "elapsed_sec": 0.0}]
    runner._last_odom = object()
    runner.server_wait_timeout_sec = 0.01
    runner._spin_until = lambda predicate, timeout_sec, after_spin=None: True
    runner._action_client = FakeActionClient(True)
    runner._mark_failure_time = lambda: None
    runner._observe_stop_and_post_motion = lambda: None
    runner._evaluate = lambda: None

    def set_initial_pose_once():
        Phase2FailureTestRunner._transition(runner, P2FState.SETTING_START)
        runner._initialpose_publish_count += 1
        Phase2FailureTestRunner._transition(runner, P2FState.VERIFYING_START)
        return True

    def send_goal():
        Phase2FailureTestRunner._transition(runner, P2FState.GOAL_REQUEST_SENT)
        return Phase2FailureTestRunner._result_dict(runner)

    runner._set_initial_pose_once = set_initial_pose_once
    runner._send_goal_and_observe = send_goal
    runner._perform_tf_loss_fault = lambda: (Phase2FailureTestRunner._transition(runner, P2FState.PRE_FAULT) or True)
    return runner


def test_existing_planner_failure_state_sequence_remains_unchanged():
    runner = runnable_existing_mode_runner("planner_failure")
    Phase2FailureTestRunner.run(runner)
    assert transition_states(runner) == ["IDLE", "WAITING_FOR_NAV2", "SETTING_START", "VERIFYING_START", "GOAL_REQUEST_SENT"]


def test_existing_controller_no_progress_state_sequence_remains_unchanged():
    runner = runnable_existing_mode_runner("controller_no_progress")
    Phase2FailureTestRunner.run(runner)
    assert transition_states(runner) == ["IDLE", "WAITING_FOR_NAV2", "SETTING_START", "VERIFYING_START", "GOAL_REQUEST_SENT"]


def test_existing_runtime_tf_loss_state_sequence_remains_unchanged():
    runner = runnable_existing_mode_runner("tf_loss")
    Phase2FailureTestRunner.run(runner)
    assert transition_states(runner) == [
        "IDLE",
        "WAITING_FOR_NAV2",
        "SETTING_START",
        "VERIFYING_START",
        "PRE_FAULT",
        "GOAL_REQUEST_SENT",
    ]


def test_existing_action_response_probe_state_sequence_remains_unchanged():
    runner = runnable_existing_mode_runner("action_response_probe")
    Phase2FailureTestRunner.run(runner)
    assert transition_states(runner) == ["IDLE", "WAITING_FOR_NAV2", "SETTING_START", "VERIFYING_START", "GOAL_REQUEST_SENT"]


def test_internal_exception_records_failure_classification_and_zero_cancel():
    runner = runnable_cold_start_runner()
    runner.state = P2FState.WAITING_FOR_NAV2
    exc = RuntimeError("injected cold-start failure")

    Phase2FailureTestRunner._record_internal_exception(runner, exc)

    assert runner._runner_exception_type == "RuntimeError"
    assert runner._runner_exception_message == "injected cold-start failure"
    assert runner._runner_exception_state == "WAITING_FOR_NAV2"
    assert "RuntimeError: injected cold-start failure" in runner._runner_exception_traceback
    assert runner._cold_start_final_classification == "RUNNER_INTERNAL_EXCEPTION"
    assert runner._cancel_count == 0
    assert transition_states(runner)[-1] == "FAILED"


def test_internal_exception_finish_writes_terminal_json_once(tmp_path):
    runner = runnable_cold_start_runner()
    runner.state = P2FState.WAITING_FOR_NAV2
    runner.state_transitions = [{"state": P2FState.WAITING_FOR_NAV2.value, "elapsed_sec": 0.0}]
    runner.output_result_path = tmp_path / "result.json"
    runner.get_parameter = lambda name: type("P", (), {"value": "test"})()
    exc = ValueError("bad transition")

    Phase2FailureTestRunner._record_internal_exception(runner, exc)
    first = Phase2FailureTestRunner._finish(runner)
    second = Phase2FailureTestRunner._finish(runner)
    data = json.loads(runner.output_result_path.read_text())

    assert first == second
    assert data["pass"] is False
    assert data["final_classification"] == "RUNNER_INTERNAL_EXCEPTION"
    assert data["runner_exception_type"] == "ValueError"
    assert data["runner_exception_message"] == "bad transition"
    assert data["runner_exception_state"] == "WAITING_FOR_NAV2"
    assert "ValueError: bad transition" in data["runner_exception_traceback"]
    assert data["cleanup_cancel_count"] == 0
    assert transition_states(runner).count("CLEANUP") == 1


def test_cold_start_mode_publishes_no_initial_pose_and_requires_zero_odometry():
    runner = cold_start_runner()
    Phase2FailureTestRunner._evaluate(runner)
    assert runner.failure_reasons == []

    runner = cold_start_runner()
    runner._initialpose_publish_count = 1
    runner._odom_count = 1
    Phase2FailureTestRunner._evaluate(runner)
    assert any("must not publish /initialpose" in reason for reason in runner.failure_reasons)
    assert any("expected zero /Odometry" in reason for reason in runner.failure_reasons)


def test_cold_start_tf_absence_is_measured_independently():
    runner = cold_start_runner()
    runner._tf_checks = {
        "map_to_odom": {"available": True},
        "odom_to_base": {"available": False, "error": "lookup failed"},
        "map_to_base": {"available": False, "error": "connectivity failed"},
    }
    Phase2FailureTestRunner._evaluate(runner)
    assert runner.failure_reasons == []

    runner = cold_start_runner()
    runner._odom_count = 0
    runner._tf_checks["odom_to_base"] = {"available": True}
    Phase2FailureTestRunner._evaluate(runner)
    assert any("unexpectedly resolved odom->base_footprint" in reason for reason in runner.failure_reasons)


def test_cold_start_odom_absence_is_not_labeled_tf_absence():
    source = inspect.getsource(Phase2FailureTestRunner._run_cold_start_missing_tf)
    assert "_verify_cold_start_tf_absence" in source
    assert "_tf_stale_before_goal = self._odom_stale_before_goal" not in source


def test_cold_start_no_goal_before_missing_tf_is_proven():
    source = inspect.getsource(Phase2FailureTestRunner._run_cold_start_missing_tf)
    assert source.index("_verify_cold_start_tf_absence") < source.index("return self._send_goal_and_observe()")


def test_cold_start_at_most_one_navigate_to_pose_request_and_no_followpath():
    runner = cold_start_runner()
    runner._goal_send_count = 2
    Phase2FailureTestRunner._evaluate(runner)
    assert any("more than one goal" in reason for reason in runner.failure_reasons)
    source = inspect.getsource(Phase2FailureTestRunner)
    assert 'ActionClient(self, NavigateToPose, "/navigate_to_pose")' in source
    assert "FollowPath" not in source


def test_cold_start_safe_pass_dispositions():
    for state, accepted, status, cancel_count in (
        (P2FState.TIMEOUT, False, None, 0),
        (P2FState.REJECTED, False, None, 0),
        (P2FState.ABORTED, True, GoalStatus.STATUS_ABORTED, 0),
        (P2FState.TIMEOUT, True, GoalStatus.STATUS_CANCELED, 1),
    ):
        runner = cold_start_runner()
        runner.state = state
        runner._goal_send_count = 1 if accepted or state == P2FState.REJECTED else 0
        runner._accepted = accepted
        runner._action_status = status
        runner._cancel_count = cancel_count
        Phase2FailureTestRunner._evaluate(runner)
        assert runner.failure_reasons == []
        assert runner._cold_start_final_classification == "COLD_START_MISSING_TF_SAFE_FAILURE_CONFIRMED"


def test_cold_start_nonzero_command_and_unknown_dynamic_tf_fail():
    runner = cold_start_runner()
    runner._nonzero_cmd_count = 1
    runner._tf_checks["map_to_base"] = {"available": True}
    Phase2FailureTestRunner._evaluate(runner)
    assert any("produced nonzero commands" in reason for reason in runner.failure_reasons)
    assert any("unexpectedly resolved map->base_footprint" in reason for reason in runner.failure_reasons)


def test_cold_start_serializes_terminal_json_fields():
    runner = cold_start_runner()
    runner.scenario_name = "p2f_cold_start_missing_tf_internal"
    runner.get_parameter = lambda name: type("P", (), {"value": ""})()
    runner.start_pose = Pose2D(0.0, 0.0, 0.0)
    runner.goal_pose = Pose2D(8.425, -53.725, 0.0)
    runner._goal_response_received = False
    runner._goal_request_elapsed_sec = None
    runner._goal_response_elapsed_sec = None
    runner._goal_id = None
    runner._result_received = False
    runner._result_elapsed_sec = None
    runner._cancel_response_received = False
    runner.state_transitions = []
    runner.failure_reasons = []
    Phase2FailureTestRunner._evaluate(runner)

    result = Phase2FailureTestRunner._result_dict(runner)

    assert result["mode"] == "cold_start_missing_tf"
    assert result["request_count"] == 0
    assert result["odometry_message_count"] == 0
    assert result["map_to_odom_available"] is True
    assert result["odom_to_base_available"] is False
    assert result["map_to_base_available"] is False
    assert result["path_message_count"] == 0
    assert result["nonempty_path_count"] == 0
    assert result["command_publishers_by_gid"] == {}
    assert result["unknown_publisher_count"] == 0
    assert result["final_classification"] == "COLD_START_MISSING_TF_SAFE_FAILURE_CONFIRMED"


def test_existing_failure_modes_remain_separate_from_cold_start():
    source = inspect.getsource(Phase2FailureTestRunner._evaluate)
    assert 'self.mode == "planner_failure"' in source
    assert 'self.mode == "controller_no_progress"' in source
    assert 'self.mode == "tf_loss"' in source
    assert 'self.mode == "cold_start_missing_tf"' in source
    assert "tf-loss fake-base/odom staleness was not proven" in source
