from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import RouteMission

from parking_robot_mission_manager.mission_state_machine import (
    GoalResultCode,
    MissionStateCode,
    MissionStateMachine,
)
from parking_robot_mission_manager.nav2_goal_executor import ScriptedFakeGoalExecutor


EXPECTED_TOPOLOGY = "topology-v1"


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def pose(x):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.orientation.w = 1.0
    return msg


def mission(count=2, *, topology=EXPECTED_TOPOLOGY):
    msg = RouteMission()
    msg.header.frame_id = "map"
    msg.mission_id = "m1"
    msg.route_id = "r1"
    msg.topology_version = topology
    msg.node_ids = [f"n{i}" for i in range(count)]
    msg.poses = [pose(float(i)) for i in range(count)]
    msg.edge_ids = [f"e{i}" for i in range(max(count - 1, 0))]
    msg.edge_directions = [1 for _ in msg.edge_ids]
    return msg


def machine(executor, clock=None):
    return MissionStateMachine(
        executor,
        expected_topology_version=EXPECTED_TOPOLOGY,
        cancel_response_timeout_sec=2.0,
        cancel_result_timeout_sec=5.0,
        steady_clock=clock or FakeClock(),
    )


def active_machine(outcomes=None, count=2, clock=None):
    executor = ScriptedFakeGoalExecutor(outcomes or ["delayed"])
    sm = machine(executor, clock)
    sm.receive_mission(mission(count))
    assert sm.start().valid
    assert sm.state == MissionStateCode.NAVIGATING
    assert sm.active_goal_uuid
    return sm, executor


def states(sm):
    return [snap.state for snap in sm.snapshots]


def reason_codes(sm):
    return [snap.reason_code for snap in sm.snapshots]


def test_route_receipt_causes_received_and_sends_zero_goals():
    executor = ScriptedFakeGoalExecutor(["succeeded"])
    sm = machine(executor)
    assert sm.receive_mission(mission()).valid
    assert sm.state == MissionStateCode.RECEIVED
    assert executor.sent_waypoint_ids == []


def test_start_sequence_and_two_waypoint_success():
    executor = ScriptedFakeGoalExecutor(["succeeded", "succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    assert sm.start().valid
    observed = states(sm)
    for state in (MissionStateCode.RECEIVED, MissionStateCode.VALIDATING, MissionStateCode.PLANNING, MissionStateCode.NAVIGATING):
        assert state in observed
    assert sm.state == MissionStateCode.SUCCEEDED
    assert sm.completed_waypoint_count == 2
    assert sm.snapshot().progress == 1.0


def test_invalid_topology_version_fails_with_zero_goals():
    executor = ScriptedFakeGoalExecutor(["succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(topology="wrong"))
    result = sm.start()
    assert not result.valid
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "TOPOLOGY_VERSION_MISMATCH"
    assert executor.sent_waypoint_ids == []


def test_middle_waypoint_aborted_sends_no_later_waypoint():
    executor = ScriptedFakeGoalExecutor(["succeeded", "aborted", "succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(3))
    sm.start()
    assert sm.state == MissionStateCode.FAILED
    assert sm.current_waypoint_index == 1
    assert sm.completed_waypoint_count == 1
    assert len(executor.sent_waypoint_ids) == 2


def test_second_mission_during_navigation_does_not_corrupt_active_mission():
    sm, _ = active_machine(["delayed"])
    active = sm.snapshot()
    result = sm.receive_mission(mission(2))
    assert not result.valid
    assert sm.snapshot().mission_id == active.mission_id
    assert sm.state == MissionStateCode.NAVIGATING


def test_cancel_acknowledged_and_canceled_result_reaches_cancelled():
    sm, executor = active_machine(["delayed"])
    assert sm.request_cancel()
    assert sm.state == MissionStateCode.CANCELLING
    assert executor.cancel_count == 1
    sm.on_cancel_response_accepted()
    assert sm.state == MissionStateCode.CANCELLING
    sm.on_cancel_result_canceled()
    assert sm.state == MissionStateCode.CANCELLED
    assert sm.active_goal_uuid == ""


def test_immediate_cancel_rejection_is_not_timeout():
    sm, executor = active_machine(["delayed", "cancel_reject"])
    assert sm.request_cancel()
    assert executor.cancel_count == 1
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "CANCEL_ACK_REJECTED"


def test_never_completing_cancel_response_times_out_after_steady_deadline():
    clock = FakeClock()
    sm, executor = active_machine(["delayed"], clock=clock)
    assert sm.request_cancel()
    assert executor.cancel_count == 1
    clock.advance(1.9)
    sm.tick()
    assert sm.state == MissionStateCode.CANCELLING
    clock.advance(0.1)
    sm.tick()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "CANCEL_ACK_TIMEOUT"
    assert sm.active_goal_uuid


def test_cancel_response_accepted_but_no_result_times_out():
    clock = FakeClock()
    sm, _ = active_machine(["delayed"], clock=clock)
    assert sm.request_cancel()
    sm.on_cancel_response_accepted()
    clock.advance(4.9)
    sm.tick()
    assert sm.state == MissionStateCode.CANCELLING
    clock.advance(0.1)
    sm.tick()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "CANCEL_RESULT_TIMEOUT"
    assert sm.active_goal_uuid


def test_pause_acknowledged_and_canceled_result_reaches_paused_preserving_index():
    sm, executor = active_machine(["delayed"])
    assert sm.request_pause()
    assert executor.cancel_count == 1
    assert sm.state == MissionStateCode.CANCELLING
    sm.on_cancel_response_accepted()
    sm.on_cancel_result_canceled()
    assert sm.state == MissionStateCode.PAUSED
    assert sm.current_waypoint_index == 0
    assert sm.completed_waypoint_count == 0
    assert sm.active_goal_uuid == ""


def test_immediate_pause_rejection_is_not_timeout():
    sm, executor = active_machine(["delayed", "cancel_reject"])
    assert sm.request_pause()
    assert executor.cancel_count == 1
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "PAUSE_CANCEL_ACK_REJECTED"


def test_never_completing_pause_response_times_out():
    clock = FakeClock()
    sm, _ = active_machine(["delayed"], clock=clock)
    assert sm.request_pause()
    clock.advance(2.0)
    sm.tick()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "PAUSE_CANCEL_ACK_TIMEOUT"


def test_pause_response_accepted_but_no_result_times_out():
    clock = FakeClock()
    sm, _ = active_machine(["delayed"], clock=clock)
    assert sm.request_pause()
    sm.on_cancel_response_accepted()
    clock.advance(5.0)
    sm.tick()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "PAUSE_CANCEL_RESULT_TIMEOUT"


def test_resume_resends_same_waypoint_and_completes():
    sm, executor = active_machine(["delayed", "succeeded", "succeeded"])
    assert sm.request_pause()
    sm.on_cancel_response_accepted()
    sm.on_cancel_result_canceled()
    assert sm.resume()
    assert sm.state == MissionStateCode.SUCCEEDED
    assert executor.sent_waypoint_ids[:2] == ["pose-0", "pose-1"]
    assert sm.completed_waypoint_count == 2


def test_pause_resume_idempotence_and_one_cancel_request():
    sm, executor = active_machine(["delayed"])
    assert sm.request_pause()
    assert sm.request_pause()
    assert sm.state == MissionStateCode.CANCELLING
    assert executor.cancel_count == 1
    sm.on_cancel_response_accepted()
    sm.on_cancel_result_canceled()
    assert sm.request_pause()
    assert sm.state == MissionStateCode.PAUSED


def test_late_succeeded_after_cancel_timeout_does_not_become_succeeded():
    clock = FakeClock()
    sm, _ = active_machine(["delayed"], clock=clock)
    sm.request_cancel()
    clock.advance(2.0)
    sm.tick()
    assert sm.state == MissionStateCode.FAILED
    sm.on_goal_result(GoalResultCode.SUCCEEDED, "late success")
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "CANCEL_ACK_TIMEOUT"
    assert sm.late_action_results == ["SUCCEEDED:late success"]


def test_late_canceled_after_timeout_does_not_become_cancelled():
    clock = FakeClock()
    sm, _ = active_machine(["delayed"], clock=clock)
    sm.request_cancel()
    sm.on_cancel_response_accepted()
    clock.advance(5.0)
    sm.tick()
    assert sm.state == MissionStateCode.FAILED
    sm.on_goal_result(GoalResultCode.CANCELED, "late canceled")
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "CANCEL_RESULT_TIMEOUT"
    assert sm.late_action_results == ["CANCELED:late canceled"]


def test_no_later_waypoint_after_timeout_or_rejection():
    clock = FakeClock()
    sm, executor = active_machine(["delayed", "succeeded"], count=2, clock=clock)
    sm.request_cancel()
    clock.advance(2.0)
    sm.tick()
    sm.on_goal_result(GoalResultCode.SUCCEEDED, "late")
    assert len(executor.sent_waypoint_ids) == 1

    sm, executor = active_machine(["delayed", "cancel_reject", "succeeded"], count=2)
    sm.request_pause()
    assert sm.state == MissionStateCode.FAILED
    assert len(executor.sent_waypoint_ids) == 1


def test_no_goal_dispatched_before_start_and_start_refusals():
    executor = ScriptedFakeGoalExecutor(["succeeded"])
    sm = machine(executor)
    assert not sm.start().valid
    sm.receive_mission(mission(1))
    assert executor.sent_waypoint_ids == []


def test_progress_monotonicity_and_completed_count_bounds():
    executor = ScriptedFakeGoalExecutor(["succeeded", "succeeded", "succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(3))
    sm.start()
    progresses = [snap.progress for snap in sm.snapshots]
    assert progresses == sorted(progresses)
    assert all(snap.completed_waypoint_count <= snap.total_waypoint_count for snap in sm.snapshots)


def test_exceptions_become_failed_with_terminal_evidence():
    executor = ScriptedFakeGoalExecutor(["exception"])
    sm = machine(executor)
    sm.receive_mission(mission(1))
    sm.start()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "EXECUTOR_EXCEPTION"


def test_terminal_to_idle_to_received_transition_is_published():
    executor = ScriptedFakeGoalExecutor(["succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(1))
    sm.start()
    assert sm.state == MissionStateCode.SUCCEEDED
    sm.receive_mission(mission(1))
    observed = states(sm)
    assert observed[-2:] == [MissionStateCode.IDLE, MissionStateCode.RECEIVED]


def test_no_later_waypoint_after_failed_cancelled_blocked_or_help_required():
    executor = ScriptedFakeGoalExecutor(["aborted", "succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert sm.state == MissionStateCode.FAILED
    assert len(executor.sent_waypoint_ids) == 1

    sm, executor = active_machine(["delayed"])
    sm.request_cancel()
    sm.on_cancel_response_accepted()
    sm.on_cancel_result_canceled()
    assert sm.state == MissionStateCode.CANCELLED
    assert len(executor.sent_waypoint_ids) == 1

    assert MissionStateCode.BLOCKED.name == "BLOCKED"
    assert MissionStateCode.HELP_REQUIRED.name == "HELP_REQUIRED"
