import pytest

from action_msgs.msg import GoalStatus

from parking_robot_bringup.phase2_goal_test_runner import CommandLimits, Pose2D
from parking_robot_bringup.phase2_sequence_cancel_test_runner import (
    CommandTracker,
    GoalRecord,
    P2ERunnerState,
    Phase2SequenceCancelTestRunner,
    VALID_TRANSITIONS,
    validate_transition,
)


class CountingFuture:
    def __init__(self, result=None, *, done=True, exc=None):
        self._result = result
        self._done = done
        self._exc = exc
        self.result_calls = 0

    def done(self):
        return self._done

    def result(self):
        self.result_calls += 1
        if self._exc is not None:
            raise self._exc
        return self._result


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.goal_id = type("GoalID", (), {"uuid": [1, 2, 3, 4]})()


class FakeResult:
    def __init__(self, status):
        self.status = status


def bare_runner():
    runner = object.__new__(Phase2SequenceCancelTestRunner)
    runner._send_goal_future = None
    runner._goal_handle = None
    runner._result_future = None
    runner._cancel_future = None
    runner._response_processed = False
    runner._result_processed = False
    runner._cancel_processed = False
    runner._goal_response_exception = None
    runner._result_exception = None
    runner._cancel_response_received = False
    runner._cancel_exception = None
    runner._cancel_response_elapsed_sec = None
    runner._start_monotonic = 0.0
    runner._active_goal = GoalRecord(index=1, requested_pose=Pose2D(1.0, 0.0, 0.0), request_elapsed_sec=1.0)
    runner.failure_reasons = []
    runner._goal_handle_id = lambda goal_handle: "01020304"
    runner._elapsed = lambda: 2.0
    return runner


def test_p2e_state_machine_requires_request_before_active():
    assert P2ERunnerState.GOAL_REQUEST_SENT in VALID_TRANSITIONS[P2ERunnerState.VERIFYING_START]
    assert P2ERunnerState.GOAL_ACTIVE not in VALID_TRANSITIONS[P2ERunnerState.VERIFYING_START]
    validate_transition(P2ERunnerState.GOAL_REQUEST_SENT, P2ERunnerState.GOAL_ACTIVE)
    with pytest.raises(ValueError):
        validate_transition(P2ERunnerState.VERIFYING_START, P2ERunnerState.GOAL_ACTIVE)


def test_sequential_mode_allows_next_goal_only_from_active_state():
    assert P2ERunnerState.GOAL_REQUEST_SENT in VALID_TRANSITIONS[P2ERunnerState.GOAL_ACTIVE]
    assert P2ERunnerState.GOAL_REQUEST_SENT not in VALID_TRANSITIONS[P2ERunnerState.IDLE]
    assert P2ERunnerState.GOAL_REQUEST_SENT not in VALID_TRANSITIONS[P2ERunnerState.CANCEL_REQUEST_SENT]


def test_cancel_request_cannot_occur_before_goal_active():
    assert P2ERunnerState.CANCEL_REQUEST_SENT in VALID_TRANSITIONS[P2ERunnerState.GOAL_ACTIVE]
    assert P2ERunnerState.CANCEL_REQUEST_SENT not in VALID_TRANSITIONS[P2ERunnerState.GOAL_REQUEST_SENT]
    assert P2ERunnerState.CANCEL_REQUEST_SENT not in VALID_TRANSITIONS[P2ERunnerState.VERIFYING_START]


def test_goal_future_done_before_callback_is_consumed_once():
    runner = bare_runner()
    future = CountingFuture(FakeGoalHandle(accepted=True))
    runner._send_goal_future = future

    Phase2SequenceCancelTestRunner._process_goal_response_if_ready(runner)
    Phase2SequenceCancelTestRunner._process_goal_response_if_ready(runner)

    assert future.result_calls == 1
    assert runner._response_processed is True
    assert runner._goal_handle.accepted is True
    assert runner._active_goal.accepted is True
    assert runner._active_goal.goal_id == "01020304"
    assert runner._active_goal.response_elapsed_sec == 2.0


def test_goal_callback_after_sync_consumption_is_noop():
    runner = bare_runner()
    future = CountingFuture(FakeGoalHandle(accepted=True))
    runner._send_goal_future = future

    Phase2SequenceCancelTestRunner._process_goal_response_if_ready(runner)
    Phase2SequenceCancelTestRunner._goal_response_cb(runner, future)

    assert future.result_calls == 1
    assert runner._active_goal.accepted is True


def test_result_future_done_before_callback_is_consumed_once():
    runner = bare_runner()
    future = CountingFuture(FakeResult(GoalStatus.STATUS_SUCCEEDED))
    runner._result_future = future

    Phase2SequenceCancelTestRunner._process_result_response_if_ready(runner)
    Phase2SequenceCancelTestRunner._result_response_cb(runner, future)

    assert future.result_calls == 1
    assert runner._result_processed is True
    assert runner._active_goal.result_status == GoalStatus.STATUS_SUCCEEDED
    assert runner._active_goal.result_status_name == "SUCCEEDED"


def test_cancel_future_is_consumed_once():
    runner = bare_runner()
    future = CountingFuture(object())
    runner._cancel_future = future

    Phase2SequenceCancelTestRunner._process_cancel_response_if_ready(runner)
    Phase2SequenceCancelTestRunner._process_cancel_response_if_ready(runner)

    assert future.result_calls == 1
    assert runner._cancel_processed is True
    assert runner._cancel_response_received is True


def test_future_exception_is_recorded_once():
    runner = bare_runner()
    future = CountingFuture(exc=RuntimeError("boom"))
    runner._send_goal_future = future

    Phase2SequenceCancelTestRunner._process_goal_response_if_ready(runner)
    Phase2SequenceCancelTestRunner._process_goal_response_if_ready(runner)

    assert future.result_calls == 1
    assert runner._goal_response_exception is not None
    assert runner._active_goal.accepted is False
    assert len(runner.failure_reasons) == 1


def test_command_tracker_records_stop_after_cancel():
    from geometry_msgs.msg import Twist

    tracker = CommandTracker(CommandLimits(max_abs_linear_x=0.25, max_abs_angular_z=0.8))
    moving = Twist()
    moving.linear.x = 0.1
    stopped = Twist()

    tracker.observe(moving, 1.0, cancel_elapsed=None)
    tracker.observe(stopped, 3.2, cancel_elapsed=3.0)

    assert tracker.message_count == 2
    assert tracker.nonzero_count == 1
    assert tracker.first_zero_after_cancel_elapsed_sec == 3.2
    assert tracker.limit_violation_count == 0


def test_goal_record_sequential_acceptance_rejects_missing_path():
    runner = bare_runner()
    runner.position_tolerance_m = 0.25
    runner.yaw_tolerance_rad = 0.5
    runner.command_stop_timeout_sec = 0.5
    runner.frame_id = "map"
    record = GoalRecord(index=1, requested_pose=Pose2D(1.0, 0.0, 0.0))
    record.accepted = True
    record.result_status = GoalStatus.STATUS_SUCCEEDED
    record.result_status_name = "SUCCEEDED"
    record.final_xy_error_m = 0.1
    record.final_yaw_error_rad = 0.0
    record.nonzero_command_count = 10
    record.translation_during_leg_m = 0.5
    record.post_result_stop_latency_sec = 0.0

    assert Phase2SequenceCancelTestRunner._evaluate_goal_record(runner, record) is False
    assert any("no path received" in reason for reason in record.failure_reasons)


def test_unsupported_mode_is_rejected_by_source_contract():
    import inspect

    source = inspect.getsource(Phase2SequenceCancelTestRunner)
    assert 'self.mode not in {"sequential", "cancel"}' in source
    assert "unsupported mode" in source
    assert "SingleThreadedExecutor" in source
    assert "rclpy.spin_once" not in source
