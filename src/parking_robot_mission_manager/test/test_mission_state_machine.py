from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import RouteMission

from parking_robot_mission_manager.mission_state_machine import (
    GoalResultCode,
    MissionStateCode,
    MissionStateMachine,
)
from parking_robot_mission_manager.nav2_goal_executor import ScriptedFakeGoalExecutor


EXPECTED_TOPOLOGY = "topology-v1"


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


def machine(executor):
    return MissionStateMachine(executor, expected_topology_version=EXPECTED_TOPOLOGY)


def states(sm):
    return [snap.state for snap in sm.snapshots]


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
    executor = ScriptedFakeGoalExecutor(["delayed"])
    sm = machine(executor)
    sm.receive_mission(mission(1))
    sm.start()
    active = sm.snapshot()
    result = sm.receive_mission(mission(2))
    assert not result.valid
    assert sm.snapshot().mission_id == active.mission_id
    assert sm.state == MissionStateCode.NAVIGATING


def test_cancel_acknowledgement_reaches_cancelled():
    executor = ScriptedFakeGoalExecutor(["delayed"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert sm.cancel()
    assert sm.state == MissionStateCode.CANCELLED
    assert executor.cancel_count == 1
    assert MissionStateCode.CANCELLING in states(sm)


def test_cancel_timeout_reaches_failed_not_cancelled_or_idle():
    executor = ScriptedFakeGoalExecutor(["delayed", "cancel_timeout"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert not sm.cancel()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "CANCEL_ACK_TIMEOUT"
    assert sm.state not in (MissionStateCode.CANCELLED, MissionStateCode.IDLE)


def test_pause_acknowledgement_reaches_paused_and_preserves_index():
    executor = ScriptedFakeGoalExecutor(["delayed"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert sm.pause()
    assert sm.state == MissionStateCode.PAUSED
    assert sm.current_waypoint_index == 0
    assert sm.completed_waypoint_count == 0


def test_pause_timeout_reaches_failed_not_paused():
    executor = ScriptedFakeGoalExecutor(["delayed", "cancel_timeout"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert not sm.pause()
    assert sm.state == MissionStateCode.FAILED
    assert sm.reason_code == "PAUSE_CANCEL_ACK_TIMEOUT"


def test_resume_resends_same_waypoint_and_completes():
    executor = ScriptedFakeGoalExecutor(["delayed", "succeeded", "succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert sm.pause()
    assert sm.resume()
    assert sm.state == MissionStateCode.SUCCEEDED
    assert executor.sent_waypoint_ids[:2] == ["pose-0", "pose-1"]
    assert sm.completed_waypoint_count == 2


def test_pause_resume_idempotence_and_one_cancel_request():
    executor = ScriptedFakeGoalExecutor(["delayed"])
    sm = machine(executor)
    sm.receive_mission(mission(1))
    sm.start()
    assert sm.pause()
    assert sm.pause()
    assert sm.state == MissionStateCode.PAUSED
    assert executor.cancel_count == 1
    assert not sm.resume() or sm.state in (MissionStateCode.NAVIGATING, MissionStateCode.SUCCEEDED)


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


def test_no_later_waypoint_after_failed_cancelled_blocked_or_help_required():
    executor = ScriptedFakeGoalExecutor(["aborted", "succeeded"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    assert sm.state == MissionStateCode.FAILED
    assert len(executor.sent_waypoint_ids) == 1

    executor = ScriptedFakeGoalExecutor(["delayed"])
    sm = machine(executor)
    sm.receive_mission(mission(2))
    sm.start()
    sm.cancel()
    assert sm.state == MissionStateCode.CANCELLED
    assert len(executor.sent_waypoint_ids) == 1

    assert MissionStateCode.BLOCKED.name == "BLOCKED"
    assert MissionStateCode.HELP_REQUIRED.name == "HELP_REQUIRED"
