from geometry_msgs.msg import PoseStamped
from parking_robot_interfaces.msg import RouteMission

from parking_robot_mission_manager.mission_state_machine import (
    GoalResultCode,
    MissionStateCode,
    MissionStateMachine,
    TerminationIntent,
    VALID_TRANSITIONS,
)
from parking_robot_mission_manager.mission_progress_supervisor_core import (
    CollisionClassification,
    ProgressClassification,
    SupervisorEvent,
    SupervisorResult,
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


def supervisor_result(event, reason=None):
    return SupervisorResult(
        primary_event=event,
        collision=CollisionClassification.CLEAR,
        progress=ProgressClassification.NOT_MEASURABLE,
        movement_intent=False,
        ages_sec={},
        stop_age_sec=None,
        no_progress_age_sec=None,
        clear_age_sec=None,
        recovery_delta=0,
        reason=reason or event.name,
        diagnostics={},
    )


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


def test_navigating_with_accepted_goal_always_accepts_user_cancel():
    sm, executor = active_machine(["delayed"])
    assert sm.snapshot().state is MissionStateCode.NAVIGATING
    assert sm.snapshot().active_goal_uuid
    assert sm.request_cancel()
    assert sm.state is MissionStateCode.CANCELLING
    assert executor.cancel_count == 1


def test_goal_ownership_clear_cannot_leave_reported_state_navigating():
    sm, _ = active_machine(["delayed"])
    sm.on_goal_result(GoalResultCode.CANCELED, "unexpected cancellation")
    snapshot = sm.snapshot()
    assert snapshot.active_goal_uuid == ""
    assert snapshot.state is MissionStateCode.FAILED
    assert snapshot.reason_code == "GOAL_CANCELED"


def test_pause_resume_does_not_poison_later_user_cancel():
    sm, executor = active_machine(["delayed", "delayed"])
    assert sm.request_pause()
    sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    preserved_index = sm.current_waypoint_index
    assert sm.resume()
    assert sm.state is MissionStateCode.NAVIGATING
    assert sm.current_waypoint_index == preserved_index
    assert sm.active_goal_uuid
    assert sm.request_cancel()
    assert sm.state is MissionStateCode.CANCELLING
    assert executor.cancel_count == 2
    sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert sm.state is MissionStateCode.CANCELLED
    assert len(executor.sent_waypoint_ids) == 2


def test_second_independent_mission_after_cancel_is_cancellable_without_next_goal():
    sm, executor = active_machine(["delayed", "delayed"], count=2)
    assert sm.request_cancel()
    sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert sm.receive_mission(mission(2)).valid
    assert sm.start().valid
    assert sm.state is MissionStateCode.NAVIGATING
    assert sm.request_cancel()
    sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert sm.state is MissionStateCode.CANCELLED
    assert sm.completed_waypoint_count == 0
    assert len(executor.sent_waypoint_ids) == 2


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


def test_p4e5a_exact_new_transition_legality_matrix():
    expected = {
        (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.NAVIGATING): True,
        (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.PLANNING): True,
        (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.CANCELLING): True,
        (MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.SUCCEEDED): True,
        (MissionStateCode.CANCELLING, MissionStateCode.BLOCKED): True,
        (MissionStateCode.CANCELLING, MissionStateCode.FAILED): True,
        (MissionStateCode.CANCELLING, MissionStateCode.CANCELLED): True,
        (MissionStateCode.CANCELLING, MissionStateCode.PAUSED): True,
    }
    assert {(source, target): target in VALID_TRANSITIONS[source] for source, target in expected} == expected
    assert MissionStateCode.HELP_REQUIRED not in VALID_TRANSITIONS[MissionStateCode.TEMPORARILY_BLOCKED]
    assert VALID_TRANSITIONS[MissionStateCode.CANCELLING] == {
        MissionStateCode.PAUSED, MissionStateCode.CANCELLED,
        MissionStateCode.BLOCKED, MissionStateCode.FAILED,
    }


def test_temporary_stop_is_idempotent_and_stable_clear_recovers():
    sm, executor = active_machine(["delayed"])
    stop = supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY)
    sm.apply_progress_supervisor_result(stop)
    assert sm.state is MissionStateCode.TEMPORARILY_BLOCKED
    assert sm.block_reason == "COLLISION_STOP_TEMPORARY"
    snapshot_count = len(sm.snapshots)
    sm.apply_progress_supervisor_result(stop)
    assert len(sm.snapshots) == snapshot_count
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_SLOWDOWN))
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.PROGRESSING, "measurable progress"))
    assert sm.state is MissionStateCode.TEMPORARILY_BLOCKED
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.PROGRESSING, "clear recovery stable"))
    assert sm.state is MissionStateCode.NAVIGATING
    assert sm.reason_code == "MISSION_PROGRESS_RESUMED"
    assert sm.block_reason == ""
    assert executor.cancel_count == 0


def test_persistent_block_from_temporary_is_one_shot_and_clear_cannot_reverse():
    sm, executor = active_machine(["delayed"])
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY))
    persistent = supervisor_result(SupervisorEvent.PERSISTENT_COLLISION_STOP)
    sm.apply_progress_supervisor_result(persistent)
    assert sm.state is MissionStateCode.CANCELLING
    assert executor.cancel_count == 0
    assert sm._pending_cancel_operation.intent is TerminationIntent.BLOCK_TERMINATION
    sm.tick()
    assert executor.cancel_count == 1
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.PROGRESSING, "clear recovery stable"))
    sm.tick()
    assert executor.cancel_count == 1
    sm.on_cancel_response_accepted()
    sm.on_cancel_result_canceled()
    assert sm.state is MissionStateCode.BLOCKED
    assert sm.reason_code == "PERSISTENT_COLLISION_STOP"
    assert sm.block_reason == "PERSISTENT_COLLISION_STOP"
    assert len(executor.sent_waypoint_ids) == 1


def test_persistent_block_from_navigating_preserves_visible_sequence():
    sm, executor = active_machine(["delayed"])
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.CONTROLLER_NO_PROGRESS))
    tail = states(sm)[-2:]
    assert tail == [MissionStateCode.TEMPORARILY_BLOCKED, MissionStateCode.CANCELLING]
    sm.tick(); sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert executor.cancel_count == 1
    assert sm.state is MissionStateCode.BLOCKED
    assert sm.reason_code == "CONTROLLER_NO_PROGRESS"


def test_user_cancel_and_pause_are_accepted_from_temporary_blocked():
    for method, final in (("request_cancel", MissionStateCode.CANCELLED), ("request_pause", MissionStateCode.PAUSED)):
        sm, executor = active_machine(["delayed"])
        sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY))
        assert getattr(sm, method)()
        sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
        assert executor.cancel_count == 1
        assert sm.state is final
        assert sm.completed_waypoint_count == 0
        assert len(executor.sent_waypoint_ids) == 1


def test_user_overrides_internal_cancel_before_and_after_submission_without_duplicate():
    for submit_first in (False, True):
        sm, executor = active_machine(["delayed"])
        sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.PERSISTENT_COLLISION_STOP))
        if submit_first:
            sm.tick()
        assert sm.request_cancel()
        sm.tick()
        assert executor.cancel_count == 1
        sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
        assert sm.state is MissionStateCode.CANCELLED


def test_user_pause_is_below_user_cancel_and_overrides_internal_only():
    sm, executor = active_machine(["delayed"])
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.PERSISTENT_COLLISION_STOP))
    assert sm.request_pause()
    assert sm._pending_cancel_operation.intent is TerminationIntent.USER_PAUSE
    assert sm.request_cancel()
    assert sm._pending_cancel_operation.intent is TerminationIntent.USER_CANCEL
    assert sm.request_pause()
    assert sm._pending_cancel_operation.intent is TerminationIntent.USER_CANCEL
    sm.tick(); sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert executor.cancel_count == 1
    assert sm.state is MissionStateCode.CANCELLED


def test_health_termination_from_navigating_and_temporary_never_blocks():
    for temporarily_blocked in (False, True):
        sm, executor = active_machine(["delayed"])
        if temporarily_blocked:
            sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY))
        sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.GATE_FAULT))
        assert sm.state is MissionStateCode.CANCELLING
        assert executor.cancel_count == 1
        sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
        assert sm.state is MissionStateCode.FAILED
        assert sm.reason_code == "GATE_FAULT"
        assert sm.block_reason == ""
        assert len(executor.sent_waypoint_ids) == 1


def test_health_without_goal_uses_direct_failed_protocol():
    sm, _ = active_machine(["delayed"])
    sm.active_goal_uuid = ""
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.FEEDBACK_STALE))
    assert sm.state is MissionStateCode.FAILED
    assert sm.reason_code == "FEEDBACK_STALE"


def test_initial_command_acquisition_timeout_has_one_health_cancel_and_same_terminal_reason():
    sm, executor = active_machine(["delayed"])
    timeout_event = type("InitialTimeoutEvent", (), {
        "name": "INITIAL_COMMAND_ACQUISITION_TIMEOUT"
    })()
    result = SupervisorResult(
        primary_event=timeout_event,
        collision=CollisionClassification.UNAVAILABLE,
        progress=ProgressClassification.NOT_MEASURABLE,
        movement_intent=False,
        ages_sec={}, stop_age_sec=None, no_progress_age_sec=None,
        clear_age_sec=None, recovery_delta=0,
        reason="INITIAL_COMMAND_ACQUISITION_TIMEOUT", diagnostics={},
    )
    sm.apply_progress_supervisor_result(result)
    assert sm.state is MissionStateCode.CANCELLING
    assert sm.reason_code == "INITIAL_COMMAND_ACQUISITION_TIMEOUT"
    assert executor.cancel_count == 1
    for _ in range(3):
        sm.apply_progress_supervisor_result(result); sm.tick()
    assert executor.cancel_count == 1
    sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert sm.state is MissionStateCode.FAILED
    assert sm.reason_code == "INITIAL_COMMAND_ACQUISITION_TIMEOUT"


import pytest


@pytest.mark.parametrize("event,prefix", [
    (SupervisorEvent.PERSISTENT_COLLISION_STOP, "BLOCK"),
    (SupervisorEvent.GATE_FAULT, "HEALTH"),
])
@pytest.mark.parametrize("failure,suffix", [
    ("reject", "ACK_REJECTED"),
    ("ack_timeout", "ACK_TIMEOUT"),
    ("unexpected", "RESULT_UNEXPECTED"),
    ("result_timeout", "RESULT_TIMEOUT"),
])
def test_internal_intent_protocol_failures_are_failed(event, prefix, failure, suffix):
    clock = FakeClock()
    outcomes = ["delayed", "cancel_reject"] if failure == "reject" else ["delayed"]
    sm, executor = active_machine(outcomes, clock=clock)
    sm.apply_progress_supervisor_result(supervisor_result(event))
    if event is SupervisorEvent.PERSISTENT_COLLISION_STOP:
        sm.tick()
    if failure == "ack_timeout":
        clock.advance(2.0); sm.tick()
    elif failure == "unexpected":
        sm.on_cancel_response_accepted(); sm.on_goal_result(GoalResultCode.ABORTED)
    elif failure == "result_timeout":
        sm.on_cancel_response_accepted(); clock.advance(5.0); sm.tick()
    assert executor.cancel_count == 1
    assert sm.state is MissionStateCode.FAILED
    assert sm.reason_code == f"{prefix}_CANCEL_{suffix}"


def test_waypoint_success_while_temporarily_blocked_nonfinal_and_final():
    sm, executor = active_machine(["delayed", "delayed"], count=2)
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY))
    sm.on_goal_result(GoalResultCode.SUCCEEDED, "first")
    assert MissionStateCode.PLANNING in states(sm)
    assert sm.state is MissionStateCode.NAVIGATING
    assert sm.completed_waypoint_count == 1
    assert executor.cancel_count == 0
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY))
    before = len(executor.sent_waypoint_ids)
    sm.on_goal_result(GoalResultCode.SUCCEEDED, "final")
    assert sm.state is MissionStateCode.SUCCEEDED
    assert sm.completed_waypoint_count == 2
    assert executor.cancel_count == 0
    assert len(executor.sent_waypoint_ids) == before


def test_action_success_wins_before_same_tick_persistent_evidence():
    sm, executor = active_machine(["delayed"], count=1)
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.COLLISION_STOP_TEMPORARY))
    sm.on_goal_result(GoalResultCode.SUCCEEDED)
    sm.apply_progress_supervisor_result(supervisor_result(SupervisorEvent.PERSISTENT_COLLISION_STOP))
    sm.tick()
    assert sm.state is MissionStateCode.SUCCEEDED
    assert executor.cancel_count == 0


def test_repeated_internal_event_and_watchdog_ticks_keep_uuid_and_one_cancel():
    sm, executor = active_machine(["delayed"])
    uuid = sm.active_goal_uuid
    event = supervisor_result(SupervisorEvent.RECOVERY_EXHAUSTED_NO_PROGRESS)
    sm.apply_progress_supervisor_result(event)
    for _ in range(5):
        sm.apply_progress_supervisor_result(event)
        sm.tick()
    assert executor.cancel_count == 1
    assert sm.active_goal_uuid == uuid
    assert sm._pending_cancel_operation.active_goal_uuid == uuid
    sm.on_cancel_response_accepted(); sm.on_cancel_result_canceled()
    assert sm.active_goal_uuid == ""
    assert sm.state is MissionStateCode.BLOCKED
