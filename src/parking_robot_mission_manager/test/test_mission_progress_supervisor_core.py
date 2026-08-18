from dataclasses import FrozenInstanceError
import math

import pytest

from parking_robot_mission_manager.mission_progress_supervisor_core import (
    ActionTerminal, BoolHealthSample, CommandSample, FeedbackSample, GateHealthSample,
    GateState, InternalIntent, MissionProgressSupervisorCore, OdometrySample,
    OptionalAdapterHealthSample, ProgressClassification, SupervisorEvent,
    SupervisorInputs, SupervisorThresholds, TfSample,
)


def sample_inputs(now=10.0, *, raw=(0.2, 0.0), safe=(0.2, 0.0), distance=5.0,
                  pose=(0.0, 0.0), recoveries=0, collision_valid=True,
                  adapter=None, intent=InternalIntent.NONE, terminal=ActionTerminal.NONE):
    return SupervisorInputs(
        feedback=FeedbackSample(now, pose[0], pose[1], 0.0, 1.0, 10.0, recoveries, distance),
        odometry=OdometrySample(now, pose[0], pose[1], 0.0, 0.0, 0.0),
        transform=TfSample(now, pose[0], pose[1], 0.0),
        raw_command=CommandSample(now, *raw), safe_command=CommandSample(now, *safe),
        gate=GateHealthSample(now, GateState.ARMED, False),
        collision_monitor_valid=BoolHealthSample(now, collision_valid),
        localization_valid=BoolHealthSample(now, True),
        controller_valid=BoolHealthSample(now, True), adapter_health=adapter,
        intent=intent, action_terminal=terminal,
    )


def core(now=10.0):
    result = MissionProgressSupervisorCore()
    result.accept_new_goal(now, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    return result


def event(result):
    return result.primary_event


def test_brief_stop_does_not_become_temporary_and_clear_resets_timer():
    c = core()
    assert event(c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(c.evaluate(10.9, sample_inputs(10.9, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    c.evaluate(10.95, sample_inputs(10.95))
    assert event(c.evaluate(11.8, sample_inputs(11.8, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING


def test_sustained_stop_exact_entry_and_single_persistent_horizon():
    c = core()
    c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert event(c.evaluate(30.0, sample_inputs(30.0, safe=(0.0, 0.0)))) is SupervisorEvent.PERSISTENT_COLLISION_STOP


def test_unknown_pair_breaks_stop_continuity_without_core_change():
    c = core()
    assert event(c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    missing = SupervisorInputs(**{**sample_inputs(10.9).__dict__,
                                  "raw_command": None, "safe_command": None})
    assert event(c.evaluate(10.9, missing)) is SupervisorEvent.COMMAND_PAIR_STALE
    assert event(c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(c.evaluate(11.999, sample_inputs(11.999, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(c.evaluate(12.0, sample_inputs(12.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY


def test_unknown_at_19_8_seconds_resets_persistent_stop_clock():
    c = core(0.0)
    c.evaluate(0.0, sample_inputs(0.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(19.8, sample_inputs(19.8, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    missing = SupervisorInputs(**{**sample_inputs(19.81).__dict__,
                                  "raw_command": None, "safe_command": None})
    assert event(c.evaluate(19.81, missing)) is SupervisorEvent.COMMAND_PAIR_STALE
    resumed = c.evaluate(19.82, sample_inputs(19.82, safe=(0.0, 0.0)))
    assert event(resumed) is SupervisorEvent.COLLISION_STOP_PENDING
    assert resumed.stop_age_sec == 0.0


def test_unknown_during_temporary_stop_is_not_clear_recovery():
    c = core()
    c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    missing = SupervisorInputs(**{**sample_inputs(11.1).__dict__,
                                  "raw_command": None, "safe_command": None})
    assert event(c.evaluate(11.1, missing)) is SupervisorEvent.COMMAND_PAIR_STALE
    assert event(c.evaluate(11.2, sample_inputs(11.2))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(11.699, sample_inputs(11.699))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(11.7, sample_inputs(11.7))) is SupervisorEvent.PROGRESSING


def test_slowdown_with_progress():
    c = core()
    result = c.evaluate(10.1, sample_inputs(10.1, safe=(0.16, 0.0), distance=4.9))
    assert event(result) is SupervisorEvent.COLLISION_SLOWDOWN
    assert result.progress is ProgressClassification.MEASURABLE


@pytest.mark.parametrize("intent, expected", [
    (InternalIntent.USER_CANCEL, SupervisorEvent.USER_CANCEL),
    (InternalIntent.USER_PAUSE, SupervisorEvent.USER_PAUSE),
    (InternalIntent.BLOCK_TERMINATION, SupervisorEvent.BLOCK_TERMINATION),
    (InternalIntent.HEALTH_FAILURE_TERMINATION, SupervisorEvent.HEALTH_FAILURE_TERMINATION),
])
def test_intent_precedence_suppresses_obstacle(intent, expected):
    c = core()
    assert event(c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0), intent=intent))) is expected


def test_raw_zero_explicit_success_wins_and_discards_stop_timer():
    c = core()
    c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    result = c.evaluate(10.5, sample_inputs(10.5, raw=(0.0, 0.0), safe=(0.0, 0.0), terminal=ActionTerminal.SUCCEEDED))
    assert event(result) is SupervisorEvent.GOAL_REACHED
    assert result.stop_age_sec is None


@pytest.mark.parametrize("terminal, expected", [
    (ActionTerminal.GOAL_REJECTED, SupervisorEvent.GOAL_REJECTED),
    (ActionTerminal.ABORTED, SupervisorEvent.NAVIGATION_ABORTED_UNCLASSIFIED),
    (ActionTerminal.PROTOCOL_ERROR, SupervisorEvent.NAVIGATION_PROTOCOL_ERROR),
])
def test_explicit_action_terminal_taxonomy(terminal, expected):
    c = core()
    assert event(c.evaluate(10.0, sample_inputs(10.0, terminal=terminal))) is expected


def test_gate_fault_and_disarmed_are_health_not_blockage():
    c = core()
    data = sample_inputs()
    assert event(c.evaluate(10.0, SupervisorInputs(**{**data.__dict__, "gate": GateHealthSample(10.0, GateState.FAULT, True)}))) is SupervisorEvent.GATE_FAULT
    c = core()
    assert event(c.evaluate(10.0, SupervisorInputs(**{**data.__dict__, "gate": GateHealthSample(10.0, GateState.DISARMED, False)}))) is SupervisorEvent.GATE_DISARMED


def test_health_and_freshness_precede_stop():
    data = sample_inputs(10.0, safe=(0.0, 0.0), collision_valid=False)
    assert event(core().evaluate(10.0, data)) is SupervisorEvent.COLLISION_MONITOR_INVALID
    data = sample_inputs(10.0, safe=(0.0, 0.0))
    stale_odom = OdometrySample(9.49, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert event(core().evaluate(10.0, SupervisorInputs(**{**data.__dict__, "odometry": stale_odom}))) is SupervisorEvent.ODOMETRY_STALE


def test_controller_no_progress_at_twenty_seconds():
    c = core()
    assert event(c.evaluate(30.0, sample_inputs(30.0))) is SupervisorEvent.CONTROLLER_NO_PROGRESS


def test_clear_recovery_requires_half_second_stability():
    c = core()
    c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(11.1, sample_inputs(11.1))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(11.6, sample_inputs(11.6))) is SupervisorEvent.PROGRESSING


def test_freshness_exact_boundaries_and_stale_beyond():
    data = sample_inputs(10.0)
    exact_feedback = FeedbackSample(8.0, 0.0, 0.0, 0.0, 1.0, 10.0, 0, 5.0)
    assert event(core().evaluate(10.0, SupervisorInputs(**{**data.__dict__, "feedback": exact_feedback}))) is SupervisorEvent.NO_PROGRESS_PENDING
    c = core(); stale_feedback = FeedbackSample(math.nextafter(8.0, -math.inf), 0.0, 0.0, 0.0, 1.0, 10.0, 0, 5.0)
    assert event(c.evaluate(10.0, SupervisorInputs(**{**data.__dict__, "feedback": stale_feedback}))) is SupervisorEvent.FEEDBACK_STALE
    c = core(); exact_odom = OdometrySample(9.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert event(c.evaluate(10.0, SupervisorInputs(**{**data.__dict__, "odometry": exact_odom}))) is SupervisorEvent.NO_PROGRESS_PENDING
    c = core(); stale_tf = TfSample(9.49, 0.0, 0.0, 0.0)
    assert event(c.evaluate(10.0, SupervisorInputs(**{**data.__dict__, "transform": stale_tf}))) is SupervisorEvent.TF_STALE


def test_command_pair_exact_boundary_and_beyond():
    data = sample_inputs(10.0)
    exact = SupervisorInputs(**{**data.__dict__, "raw_command": CommandSample(9.75, 0.2, 0.0), "safe_command": CommandSample(10.0, 0.2, 0.0)})
    assert event(core().evaluate(10.0, exact)) is SupervisorEvent.NO_PROGRESS_PENDING
    beyond = SupervisorInputs(**{**data.__dict__, "raw_command": CommandSample(9.749, 0.2, 0.0)})
    assert event(core().evaluate(10.0, beyond)) is SupervisorEvent.COMMAND_PAIR_STALE


def test_raw_zero_never_collision_stop():
    result = core().evaluate(10.0, sample_inputs(10.0, raw=(0.0, 0.0), safe=(0.0, 0.0)))
    assert event(result) is SupervisorEvent.NO_PROGRESS_PENDING
    assert not result.movement_intent


def test_slowdown_ratio_boundary_and_above():
    assert event(core().evaluate(10.0, sample_inputs(10.0, safe=(0.16, 0.0)))) is SupervisorEvent.COLLISION_SLOWDOWN
    result = core().evaluate(10.0, sample_inputs(10.0, safe=(0.161, 0.0)))
    assert event(result) is SupervisorEvent.NO_PROGRESS_PENDING


def test_recovery_exhaustion_only_without_progress():
    assert event(core().evaluate(10.0, sample_inputs(10.0, recoveries=6))) is SupervisorEvent.RECOVERY_EXHAUSTED_NO_PROGRESS
    result = core().evaluate(10.0, sample_inputs(10.0, recoveries=6, distance=4.9))
    assert event(result) is SupervisorEvent.PROGRESSING


def test_new_waypoint_reset_clears_history_timers_and_recovery_baseline():
    c = core(); c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0)))
    c.accept_new_goal(12.0, recovery_baseline=8, distance_remaining_m=7.0, pose_xy=(3.0, 4.0))
    result = c.evaluate(12.0, sample_inputs(12.0, distance=7.0, pose=(3.0, 4.0), recoveries=8))
    assert result.recovery_delta == 0 and result.stop_age_sec is None
    assert not result.diagnostics["temporary_stop_reached"]


def test_backward_time_future_sample_and_decreasing_recovery_rejected():
    c = core(); c.evaluate(10.0, sample_inputs(10.0))
    with pytest.raises(ValueError, match="backward"):
        c.evaluate(9.9, sample_inputs(9.9))
    c = core()
    with pytest.raises(ValueError, match="future"):
        c.evaluate(10.0, sample_inputs(10.1))
    c = MissionProgressSupervisorCore(); c.accept_new_goal(10.0, recovery_baseline=2)
    with pytest.raises(ValueError, match="decreased"):
        c.evaluate(10.0, sample_inputs(10.0, recoveries=1))


@pytest.mark.parametrize("constructor", [
    lambda: CommandSample(0.0, math.nan, 0.0),
    lambda: TfSample(0.0, math.inf, 0.0, 0.0),
    lambda: FeedbackSample(0.0, 0.0, 0.0, 0.0, 0.0, math.inf, 0, 1.0),
    lambda: BoolHealthSample(-1.0, True),
])
def test_invalid_samples_rejected(constructor):
    with pytest.raises(ValueError):
        constructor()


def test_immutable_samples():
    sample = CommandSample(0.0, 1.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        sample.linear_x = 2.0


def test_optional_adapter_semantics():
    invalid = OptionalAdapterHealthSample(10.0, False, "bad")
    assert event(core().evaluate(10.0, sample_inputs(10.0, adapter=invalid))) is SupervisorEvent.ADAPTER_INVALID
    assert event(core().evaluate(10.0, sample_inputs(10.0, adapter=None))) is SupervisorEvent.NO_PROGRESS_PENDING


def test_explicit_command_authority_only():
    data = sample_inputs()
    invalid = SupervisorInputs(**{**data.__dict__, "command_authority_valid": BoolHealthSample(10.0, False)})
    assert event(core().evaluate(10.0, invalid)) is SupervisorEvent.COMMAND_AUTHORITY_INVALID
    assert event(core().evaluate(10.0, data)) is SupervisorEvent.NO_PROGRESS_PENDING


def test_pose_and_distance_progress_families_are_independent():
    assert event(core().evaluate(10.1, sample_inputs(10.1, distance=4.9))) is SupervisorEvent.PROGRESSING
    assert event(core().evaluate(10.1, sample_inputs(10.1, pose=(0.1, 0.0)))) is SupervisorEvent.PROGRESSING


def test_twist_and_command_alone_are_not_progress():
    data = sample_inputs()
    moving_odom = OdometrySample(10.0, 0.0, 0.0, 0.0, 1.0, 1.0)
    result = core().evaluate(10.0, SupervisorInputs(**{**data.__dict__, "odometry": moving_odom}))
    assert result.progress is ProgressClassification.NOT_MEASURABLE


@pytest.mark.parametrize("kwargs", [
    {"command_pair_freshness_sec": 0.6},
    {"clear_stability_sec": 1.1},
    {"temporary_block_entry_sec": 20.0},
    {"minimum_improvement_m": 0.11},
    {"persistent_block_sec": 19.0},
    {"maximum_recovery_increase": 5},
    {"mission_replan_attempts": 1},
    {"feedback_freshness_sec": math.nan},
    {"tf_freshness_sec": -1.0},
])
def test_threshold_consistency_rejected(kwargs):
    with pytest.raises(ValueError):
        SupervisorThresholds(**kwargs)


def test_persistent_stop_uses_collision_clock_not_five_second_older_no_progress_clock():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    assert event(c.evaluate(4.999, sample_inputs(4.999))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(5.0, sample_inputs(5.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    result = c.evaluate(20.0, sample_inputs(20.0, safe=(0.0, 0.0)))
    assert event(result) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert result.no_progress_age_sec == 20.0 and result.stop_age_sec == 15.0
    result = c.evaluate(25.0, sample_inputs(25.0, safe=(0.0, 0.0)))
    assert event(result) is SupervisorEvent.PERSISTENT_COLLISION_STOP
    assert result.no_progress_age_sec == 25.0 and result.stop_age_sec == 20.0


def test_nineteen_point_eight_second_general_history_cannot_bypass_stop_entry():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    assert event(c.evaluate(19.8, sample_inputs(19.8, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(c.evaluate(20.0, sample_inputs(20.0, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(c.evaluate(20.8, sample_inputs(20.8, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert event(c.evaluate(39.799999, sample_inputs(39.799999, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    # Binary subtraction at decimal 39.8 is just below 20.0; never weaken the
    # safety lower bound with an early-trigger tolerance.
    assert event(c.evaluate(39.8, sample_inputs(39.8, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    boundary = math.nextafter(39.8, math.inf)
    assert event(c.evaluate(boundary, sample_inputs(boundary, safe=(0.0, 0.0)))) is SupervisorEvent.PERSISTENT_COLLISION_STOP


def test_mature_general_no_progress_cannot_make_new_stop_instantly_persistent():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    assert event(c.evaluate(19.999999, sample_inputs(19.999999))) is SupervisorEvent.NO_PROGRESS_PENDING
    result = c.evaluate(20.0, sample_inputs(20.0, safe=(0.0, 0.0)))
    assert event(result) is SupervisorEvent.COLLISION_STOP_PENDING
    assert result.no_progress_age_sec == 20.0 and result.stop_age_sec == 0.0


def test_measurable_progress_one_second_before_stop_does_not_shorten_stop_clock():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    assert event(c.evaluate(10.0, sample_inputs(10.0, distance=4.9))) is SupervisorEvent.PROGRESSING
    assert event(c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0), distance=4.9))) is SupervisorEvent.COLLISION_STOP_PENDING
    result = c.evaluate(30.0, sample_inputs(30.0, safe=(0.0, 0.0), distance=4.9))
    assert event(result) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert result.no_progress_age_sec == 20.0 and result.stop_age_sec == 19.0
    assert event(c.evaluate(31.0, sample_inputs(31.0, safe=(0.0, 0.0), distance=4.9))) is SupervisorEvent.PERSISTENT_COLLISION_STOP


def test_persistent_stop_exact_boundary_and_just_before():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    c.evaluate(0.0, sample_inputs(0.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(19.999999, sample_inputs(19.999999, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    result = c.evaluate(20.0, sample_inputs(20.0, safe=(0.0, 0.0)))
    assert event(result) is SupervisorEvent.PERSISTENT_COLLISION_STOP
    assert result.stop_age_sec == 20.0


def test_accepted_clear_progress_gives_second_stop_a_full_new_episode():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    c.evaluate(0.0, sample_inputs(0.0, safe=(0.0, 0.0)))
    c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(10.1, sample_inputs(10.1, distance=4.9))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(10.6, sample_inputs(10.6, distance=4.8))) is SupervisorEvent.PROGRESSING
    assert event(c.evaluate(10.7, sample_inputs(10.7, safe=(0.0, 0.0), distance=4.8))) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(c.evaluate(30.699999, sample_inputs(30.699999, safe=(0.0, 0.0), distance=4.8))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert event(c.evaluate(30.7, sample_inputs(30.7, safe=(0.0, 0.0), distance=4.8))) is SupervisorEvent.PERSISTENT_COLLISION_STOP


def test_any_nonstop_evaluation_discards_near_horizon_collision_episode():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    c.evaluate(0.0, sample_inputs(0.0, safe=(0.0, 0.0)))
    assert event(c.evaluate(19.5, sample_inputs(19.5, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    clear = c.evaluate(19.6, sample_inputs(19.6))
    assert event(clear) is SupervisorEvent.NO_PROGRESS_PENDING and clear.stop_age_sec is None
    resumed = c.evaluate(19.7, sample_inputs(19.7, safe=(0.0, 0.0)))
    assert event(resumed) is SupervisorEvent.COLLISION_STOP_PENDING and resumed.stop_age_sec == 0.0
    assert resumed.diagnostics["temporary_stop_reached"]
    assert event(c.evaluate(39.699999, sample_inputs(39.699999, safe=(0.0, 0.0)))) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert event(c.evaluate(39.7, sample_inputs(39.7, safe=(0.0, 0.0)))) is SupervisorEvent.PERSISTENT_COLLISION_STOP


def test_temporary_threshold_does_not_restart_collision_clock():
    c = core()
    first = c.evaluate(10.0, sample_inputs(10.0, safe=(0.0, 0.0)))
    temporary = c.evaluate(11.0, sample_inputs(11.0, safe=(0.0, 0.0)))
    later = c.evaluate(15.0, sample_inputs(15.0, safe=(0.0, 0.0)))
    assert event(first) is SupervisorEvent.COLLISION_STOP_PENDING
    assert event(temporary) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert event(later) is SupervisorEvent.COLLISION_STOP_TEMPORARY
    assert first.stop_age_sec == 0.0 and temporary.stop_age_sec == 1.0 and later.stop_age_sec == 5.0


def test_new_goal_resets_collision_and_general_no_progress_clocks():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    c.evaluate(0.0, sample_inputs(0.0, safe=(0.0, 0.0)))
    c.evaluate(5.0, sample_inputs(5.0, safe=(0.0, 0.0)))
    c.accept_new_goal(6.0, recovery_baseline=3, distance_remaining_m=7.0, pose_xy=(1.0, 0.0))
    result = c.evaluate(6.0, sample_inputs(6.0, distance=7.0, pose=(1.0, 0.0), recoveries=3))
    assert event(result) is SupervisorEvent.NO_PROGRESS_PENDING
    assert result.stop_age_sec is None and result.no_progress_age_sec == 0.0
    assert result.recovery_delta == 0 and not result.diagnostics["temporary_stop_reached"]


def test_controller_no_progress_exact_boundary_and_progress_reset_are_unchanged():
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    assert event(c.evaluate(19.999999, sample_inputs(19.999999))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(20.0, sample_inputs(20.0))) is SupervisorEvent.CONTROLLER_NO_PROGRESS
    c = MissionProgressSupervisorCore()
    c.accept_new_goal(0.0, recovery_baseline=0, distance_remaining_m=5.0, pose_xy=(0.0, 0.0))
    assert event(c.evaluate(10.0, sample_inputs(10.0, distance=4.9))) is SupervisorEvent.PROGRESSING
    assert event(c.evaluate(29.999999, sample_inputs(29.999999, distance=4.9))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(30.0, sample_inputs(30.0, distance=4.9))) is SupervisorEvent.CONTROLLER_NO_PROGRESS


def test_recovery_exhaustion_boundary_remains_independent_of_collision_age():
    c = core()
    assert event(c.evaluate(10.0, sample_inputs(10.0, recoveries=5))) is SupervisorEvent.NO_PROGRESS_PENDING
    assert event(c.evaluate(10.1, sample_inputs(10.1, recoveries=6))) is SupervisorEvent.RECOVERY_EXHAUSTED_NO_PROGRESS
    c = core()
    assert event(c.evaluate(10.0, sample_inputs(10.0, recoveries=6, distance=4.9))) is SupervisorEvent.PROGRESSING
