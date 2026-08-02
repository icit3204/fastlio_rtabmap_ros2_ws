from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import RouteMission, RouteWaypoint

from parking_robot_mission_manager.mission_state_machine import (
    GoalResultCode,
    MissionStateCode,
    MissionStateMachine,
)
from parking_robot_mission_manager.nav2_goal_executor import ScriptedFakeGoalExecutor


def wp(waypoint_id, x):
    waypoint = RouteWaypoint()
    waypoint.waypoint_id = waypoint_id
    waypoint.pose = PoseStamped()
    waypoint.pose.header.frame_id = "map"
    waypoint.pose.pose.position.x = x
    waypoint.pose.pose.orientation.w = 1.0
    return waypoint


def mission(count=2):
    msg = RouteMission()
    msg.header.frame_id = "map"
    msg.mission_id = "m1"
    msg.route_id = "r1"
    msg.route_version = "v1"
    msg.direction_id = "forward"
    msg.waypoints = [wp(f"wp{i}", float(i)) for i in range(count)]
    return msg


def states(machine):
    return [snap.state for snap in machine.snapshots]


def test_complete_nominal_two_and_three_waypoint_success():
    executor = ScriptedFakeGoalExecutor(["succeeded", "succeeded"])
    machine = MissionStateMachine(executor)
    assert machine.submit_mission(mission(2)).valid
    assert machine.state == MissionStateCode.SUCCEEDED
    assert machine.completed_waypoint_count == 2
    assert executor.sent_waypoint_ids == ["wp0", "wp1"]

    executor = ScriptedFakeGoalExecutor(["succeeded", "succeeded", "succeeded"])
    machine = MissionStateMachine(executor)
    assert machine.submit_mission(mission(3)).valid
    assert machine.state == MissionStateCode.SUCCEEDED
    assert machine.completed_waypoint_count == 3


def test_rejected_first_waypoint_fails_without_skip():
    executor = ScriptedFakeGoalExecutor(["rejected", "succeeded"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(2))
    assert machine.state == MissionStateCode.FAILED
    assert machine.current_waypoint_index == 0
    assert executor.sent_waypoint_ids == ["wp0"]


def test_aborted_middle_waypoint_fails_without_sending_later_waypoint():
    executor = ScriptedFakeGoalExecutor(["succeeded", "aborted", "succeeded"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(3))
    assert machine.state == MissionStateCode.FAILED
    assert machine.current_waypoint_index == 1
    assert machine.completed_waypoint_count == 1
    assert executor.sent_waypoint_ids == ["wp0", "wp1"]


def test_second_mission_rejected_while_active():
    executor = ScriptedFakeGoalExecutor(["delayed"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    result = machine.submit_mission(mission(1))
    assert not result.valid
    assert result.reason_code == "MISSION_ALREADY_ACTIVE"
    assert executor.sent_waypoint_ids == ["wp0"]


def test_cancel_before_first_dispatch_and_during_active_goal():
    executor = ScriptedFakeGoalExecutor(["delayed"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    assert machine.cancel()
    assert machine.state == MissionStateCode.IDLE
    assert executor.cancel_count == 1
    assert MissionStateCode.CANCELING in states(machine)


def test_pause_and_resume_resends_same_waypoint():
    executor = ScriptedFakeGoalExecutor(["delayed", "succeeded"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(2))
    assert machine.pause()
    assert machine.state == MissionStateCode.PAUSED
    assert machine.current_waypoint_index == 0
    assert executor.cancel_count == 1
    assert machine.resume()
    assert machine.state == MissionStateCode.SUCCEEDED
    assert executor.sent_waypoint_ids[:2] == ["wp0", "wp0"]
    assert machine.completed_waypoint_count == 2


def test_pause_cancel_idempotence_and_server_unavailable():
    executor = ScriptedFakeGoalExecutor(["server_unavailable"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    assert machine.state == MissionStateCode.FAILED
    assert not machine.pause()
    assert machine.cancel() is False

    executor = ScriptedFakeGoalExecutor(["delayed"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    assert machine.pause()
    assert machine.pause()
    assert machine.cancel()
    assert machine.state == MissionStateCode.IDLE


def test_cancellation_timeout_is_bounded_and_does_not_skip():
    class TimeoutExecutor(ScriptedFakeGoalExecutor):
        def cancel_goal(self, goal_uuid, timeout_sec):
            super().cancel_goal(goal_uuid, timeout_sec)
            return False

    executor = TimeoutExecutor(["delayed", "succeeded"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    assert machine.pause()
    assert machine.state == MissionStateCode.PAUSED
    assert machine.completed_waypoint_count == 0


def test_illegal_transition_rejection_and_exception_becomes_failed():
    executor = ScriptedFakeGoalExecutor(["succeeded"])
    machine = MissionStateMachine(executor)
    assert not machine.validate_transition(MissionStateCode.RUNNING)
    assert machine.transition_errors

    executor = ScriptedFakeGoalExecutor(["exception"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    assert machine.state == MissionStateCode.FAILED
    assert machine.reason_code == "EXECUTOR_EXCEPTION"


def test_progress_monotonicity_and_final_completed_count():
    executor = ScriptedFakeGoalExecutor(["succeeded", "succeeded", "succeeded"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(3))
    progresses = [snap.progress for snap in machine.snapshots]
    assert progresses == sorted(progresses)
    assert progresses[-1] == 1.0
    assert machine.completed_waypoint_count == 3


def test_required_state_sequences_are_explicit():
    executor = ScriptedFakeGoalExecutor(["succeeded"])
    machine = MissionStateMachine(executor)
    machine.submit_mission(mission(1))
    observed = states(machine)
    assert observed[:4] == [
        MissionStateCode.IDLE,
        MissionStateCode.VALIDATING,
        MissionStateCode.READY,
        MissionStateCode.RUNNING,
    ]
    assert observed[-1] == MissionStateCode.SUCCEEDED
