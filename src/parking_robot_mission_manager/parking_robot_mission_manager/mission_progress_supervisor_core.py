"""Pure deterministic mission progress and navigation-health observations.

This module deliberately has no ROS, clock, transport, publication, or mission
state authority.  Callers own sample collection and supply monotonic steady time.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Mapping, Optional


def _finite(name: str, value: float, *, nonnegative: bool = False) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if nonnegative and value < 0.0:
        raise ValueError(f"{name} must be nonnegative")


class InternalIntent(Enum):
    NONE = auto()
    USER_CANCEL = auto()
    USER_PAUSE = auto()
    BLOCK_TERMINATION = auto()
    HEALTH_FAILURE_TERMINATION = auto()


class ActionTerminal(Enum):
    NONE = auto()
    SUCCEEDED = auto()
    GOAL_REJECTED = auto()
    ABORTED = auto()
    PROTOCOL_ERROR = auto()


class GateState(Enum):
    ARMED = auto()
    DISARMED = auto()
    FAULT = auto()


class SupervisorEvent(Enum):
    PROGRESSING = auto()
    COLLISION_SLOWDOWN = auto()
    COLLISION_STOP_PENDING = auto()
    COLLISION_STOP_TEMPORARY = auto()
    PERSISTENT_COLLISION_STOP = auto()
    NO_PROGRESS_PENDING = auto()
    CONTROLLER_NO_PROGRESS = auto()
    RECOVERY_EXHAUSTED_NO_PROGRESS = auto()
    FEEDBACK_STALE = auto()
    ODOMETRY_STALE = auto()
    TF_STALE = auto()
    COMMAND_PAIR_STALE = auto()
    COLLISION_MONITOR_INVALID = auto()
    LOCALIZATION_INVALID = auto()
    CONTROLLER_INVALID = auto()
    GATE_DISARMED = auto()
    GATE_FAULT = auto()
    COMMAND_AUTHORITY_INVALID = auto()
    ADAPTER_INVALID = auto()
    GOAL_REACHED = auto()
    GOAL_REJECTED = auto()
    NAVIGATION_ABORTED_UNCLASSIFIED = auto()
    NAVIGATION_PROTOCOL_ERROR = auto()
    USER_CANCEL = auto()
    USER_PAUSE = auto()
    BLOCK_TERMINATION = auto()
    HEALTH_FAILURE_TERMINATION = auto()


class CollisionClassification(Enum):
    UNAVAILABLE = auto()
    CLEAR = auto()
    SLOWDOWN = auto()
    STOP = auto()


class ProgressClassification(Enum):
    MEASURABLE = auto()
    NOT_MEASURABLE = auto()


@dataclass(frozen=True)
class SupervisorThresholds:
    feedback_freshness_sec: float = 2.0
    odometry_freshness_sec: float = 0.50
    tf_freshness_sec: float = 0.50
    command_pair_freshness_sec: float = 0.25
    linear_intent_mps: float = 0.01
    angular_intent_radps: float = 0.02
    minimum_improvement_m: float = 0.10
    temporary_block_entry_sec: float = 1.0
    clear_stability_sec: float = 0.50
    persistent_block_sec: float = 20.0
    active_nav2_movement_allowance_sec: float = 20.0
    maximum_recovery_increase: int = 6
    accepted_bt_outer_retry_count: int = 6
    mission_replan_attempts: int = 0
    slowdown_ratio: float = 0.80

    def __post_init__(self) -> None:
        duration_names = (
            "feedback_freshness_sec", "odometry_freshness_sec", "tf_freshness_sec",
            "command_pair_freshness_sec", "temporary_block_entry_sec",
            "clear_stability_sec", "persistent_block_sec",
            "active_nav2_movement_allowance_sec",
        )
        for name in duration_names:
            _finite(name, float(getattr(self, name)), nonnegative=True)
        for name in ("linear_intent_mps", "angular_intent_radps", "minimum_improvement_m", "slowdown_ratio"):
            _finite(name, float(getattr(self, name)), nonnegative=True)
        if self.command_pair_freshness_sec > self.odometry_freshness_sec:
            raise ValueError("command-pair freshness must not exceed odometry freshness")
        if self.command_pair_freshness_sec > self.tf_freshness_sec:
            raise ValueError("command-pair freshness must not exceed TF freshness")
        if self.clear_stability_sec > self.temporary_block_entry_sec:
            raise ValueError("clear stability must not exceed temporary-block entry")
        if self.temporary_block_entry_sec >= self.persistent_block_sec:
            raise ValueError("temporary-block entry must be less than persistent block")
        if self.minimum_improvement_m != 0.10:
            raise ValueError("minimum improvement is frozen at 0.10 m")
        if self.persistent_block_sec != self.active_nav2_movement_allowance_sec:
            raise ValueError("persistent block must equal active Nav2 movement allowance")
        if self.maximum_recovery_increase != self.accepted_bt_outer_retry_count:
            raise ValueError("maximum recovery increase must equal accepted BT outer retry count")
        if self.maximum_recovery_increase != 6:
            raise ValueError("maximum recovery increase is frozen at 6")
        if self.mission_replan_attempts != 0:
            raise ValueError("Mission Manager replan attempts are frozen at zero")
        if not 0.0 < self.slowdown_ratio <= 1.0:
            raise ValueError("slowdown ratio must be in (0, 1]")


@dataclass(frozen=True)
class FeedbackSample:
    steady_receipt_sec: float
    x: float
    y: float
    yaw: float
    navigation_time_sec: float
    estimated_time_remaining_sec: float
    number_of_recoveries: int
    distance_remaining_m: float

    def __post_init__(self) -> None:
        for name in ("steady_receipt_sec", "navigation_time_sec", "estimated_time_remaining_sec", "distance_remaining_m"):
            _finite(name, float(getattr(self, name)), nonnegative=True)
        for name in ("x", "y", "yaw"):
            _finite(name, float(getattr(self, name)))
        if isinstance(self.number_of_recoveries, bool) or not isinstance(self.number_of_recoveries, int) or self.number_of_recoveries < 0:
            raise ValueError("number_of_recoveries must be a nonnegative integer")


@dataclass(frozen=True)
class OdometrySample:
    steady_receipt_sec: float
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float

    def __post_init__(self) -> None:
        _validate_sample_numbers(self, ("x", "y", "yaw", "linear_x", "angular_z"))


@dataclass(frozen=True)
class TfSample:
    steady_receipt_sec: float
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        _validate_sample_numbers(self, ("x", "y", "yaw"))


@dataclass(frozen=True)
class CommandSample:
    steady_receipt_sec: float
    linear_x: float
    angular_z: float

    def __post_init__(self) -> None:
        _validate_sample_numbers(self, ("linear_x", "angular_z"))


@dataclass(frozen=True)
class GateHealthSample:
    steady_receipt_sec: float
    state: GateState
    fault_latched: bool
    reason: str = ""

    def __post_init__(self) -> None:
        _finite("steady_receipt_sec", self.steady_receipt_sec, nonnegative=True)
        if not isinstance(self.state, GateState) or not isinstance(self.fault_latched, bool):
            raise ValueError("invalid parsed gate health")


@dataclass(frozen=True)
class BoolHealthSample:
    steady_receipt_sec: float
    value: bool

    def __post_init__(self) -> None:
        _finite("steady_receipt_sec", self.steady_receipt_sec, nonnegative=True)
        if not isinstance(self.value, bool):
            raise ValueError("health value must be bool")


@dataclass(frozen=True)
class OptionalAdapterHealthSample:
    steady_receipt_sec: float
    valid: bool
    reason: str = ""

    def __post_init__(self) -> None:
        _finite("steady_receipt_sec", self.steady_receipt_sec, nonnegative=True)
        if not isinstance(self.valid, bool):
            raise ValueError("adapter valid must be bool")


def _validate_sample_numbers(sample: object, names: tuple[str, ...]) -> None:
    _finite("steady_receipt_sec", float(getattr(sample, "steady_receipt_sec")), nonnegative=True)
    for name in names:
        _finite(name, float(getattr(sample, name)))


@dataclass(frozen=True)
class SupervisorInputs:
    feedback: Optional[FeedbackSample]
    odometry: Optional[OdometrySample]
    transform: Optional[TfSample]
    raw_command: Optional[CommandSample]
    safe_command: Optional[CommandSample]
    gate: Optional[GateHealthSample]
    collision_monitor_valid: Optional[BoolHealthSample]
    localization_valid: Optional[BoolHealthSample]
    controller_valid: Optional[BoolHealthSample]
    adapter_health: Optional[OptionalAdapterHealthSample] = None
    command_authority_valid: Optional[BoolHealthSample] = None
    intent: InternalIntent = InternalIntent.NONE
    action_terminal: ActionTerminal = ActionTerminal.NONE


@dataclass(frozen=True)
class SupervisorResult:
    primary_event: SupervisorEvent
    collision: CollisionClassification
    progress: ProgressClassification
    movement_intent: bool
    ages_sec: Mapping[str, Optional[float]]
    stop_age_sec: Optional[float]
    no_progress_age_sec: Optional[float]
    clear_age_sec: Optional[float]
    recovery_delta: int
    reason: str
    diagnostics: Mapping[str, object]


class MissionProgressSupervisorCore:
    """Stateful per-waypoint evidence window with caller-controlled time."""

    def __init__(self, thresholds: SupervisorThresholds = SupervisorThresholds()) -> None:
        self.thresholds = thresholds
        self._last_now: Optional[float] = None
        self._goal_started: Optional[float] = None
        self._baseline_recoveries: Optional[int] = None
        self._baseline_distance: Optional[float] = None
        self._baseline_pose: Optional[tuple[float, float]] = None
        self._stop_started: Optional[float] = None
        self._no_progress_started: Optional[float] = None
        self._clear_started: Optional[float] = None
        self._temporary_stop_reached = False

    def accept_new_goal(
        self,
        steady_now: float,
        *,
        recovery_baseline: int,
        distance_remaining_m: Optional[float] = None,
        pose_xy: Optional[tuple[float, float]] = None,
    ) -> None:
        self._validate_now(steady_now)
        if isinstance(recovery_baseline, bool) or not isinstance(recovery_baseline, int) or recovery_baseline < 0:
            raise ValueError("recovery baseline must be a nonnegative integer")
        if distance_remaining_m is not None:
            _finite("distance_remaining_m", distance_remaining_m, nonnegative=True)
        if pose_xy is not None:
            if len(pose_xy) != 2:
                raise ValueError("pose_xy must contain x and y")
            _finite("pose x", pose_xy[0]); _finite("pose y", pose_xy[1])
        self._goal_started = steady_now
        self._baseline_recoveries = recovery_baseline
        self._baseline_distance = distance_remaining_m
        self._baseline_pose = pose_xy
        self._stop_started = None
        self._no_progress_started = steady_now
        self._clear_started = None
        self._temporary_stop_reached = False

    def evaluate(self, steady_now: float, inputs: SupervisorInputs) -> SupervisorResult:
        self._validate_now(steady_now)
        ages = self._ages(steady_now, inputs)

        intent_event = {
            InternalIntent.USER_CANCEL: SupervisorEvent.USER_CANCEL,
            InternalIntent.USER_PAUSE: SupervisorEvent.USER_PAUSE,
            InternalIntent.BLOCK_TERMINATION: SupervisorEvent.BLOCK_TERMINATION,
            InternalIntent.HEALTH_FAILURE_TERMINATION: SupervisorEvent.HEALTH_FAILURE_TERMINATION,
        }.get(inputs.intent)
        if intent_event is not None:
            self._stop_started = None
            self._clear_started = None
            return self._result(intent_event, ages, reason=inputs.intent.name)

        terminal_event = {
            ActionTerminal.SUCCEEDED: SupervisorEvent.GOAL_REACHED,
            ActionTerminal.GOAL_REJECTED: SupervisorEvent.GOAL_REJECTED,
            ActionTerminal.ABORTED: SupervisorEvent.NAVIGATION_ABORTED_UNCLASSIFIED,
            ActionTerminal.PROTOCOL_ERROR: SupervisorEvent.NAVIGATION_PROTOCOL_ERROR,
        }.get(inputs.action_terminal)
        if terminal_event is not None:
            self._stop_started = None
            self._clear_started = None
            if inputs.action_terminal is ActionTerminal.SUCCEEDED:
                self._no_progress_started = None
                self._temporary_stop_reached = False
            return self._result(terminal_event, ages, reason=inputs.action_terminal.name)

        health_event = self._health_failure(inputs, ages)
        if health_event is not None:
            self._reset_unsafe_evidence()
            return self._result(health_event, ages, reason=health_event.name)

        stale_event = self._required_stale(inputs, ages)
        if stale_event is not None:
            self._reset_unsafe_evidence()
            return self._result(stale_event, ages, reason=stale_event.name)

        assert inputs.feedback is not None and inputs.odometry is not None and inputs.transform is not None
        if self._baseline_recoveries is None:
            raise ValueError("accept_new_goal must establish waypoint ownership before evaluation")
        recovery_delta = inputs.feedback.number_of_recoveries - self._baseline_recoveries
        if recovery_delta < 0:
            raise ValueError("recovery count decreased without explicit new-goal reset")

        progress = self._measure_progress(inputs.feedback, inputs.odometry, inputs.transform)
        collision, movement_intent, ratio = self._collision(inputs)
        if progress:
            self._record_progress(steady_now, inputs.feedback, inputs.odometry, inputs.transform)

        if recovery_delta >= self.thresholds.maximum_recovery_increase and not progress:
            return self._result(SupervisorEvent.RECOVERY_EXHAUSTED_NO_PROGRESS, ages,
                                collision, progress, movement_intent, recovery_delta,
                                reason="recovery increase reached frozen maximum", ratio=ratio)

        if collision is CollisionClassification.STOP:
            self._clear_started = None
            if self._stop_started is None:
                self._stop_started = steady_now
            if self._no_progress_started is None:
                self._no_progress_started = self._stop_started
            stop_age = steady_now - self._stop_started
            if stop_age >= self.thresholds.persistent_block_sec:
                event = SupervisorEvent.PERSISTENT_COLLISION_STOP
            elif stop_age >= self.thresholds.temporary_block_entry_sec:
                self._temporary_stop_reached = True
                event = SupervisorEvent.COLLISION_STOP_TEMPORARY
            else:
                event = SupervisorEvent.COLLISION_STOP_PENDING
            return self._result(event, ages, collision, progress, movement_intent,
                                recovery_delta, "qualifying continuous collision stop", ratio)

        self._stop_started = None
        if self._temporary_stop_reached:
            qualifying_clear = collision is CollisionClassification.CLEAR or progress
            if qualifying_clear:
                if self._clear_started is None:
                    self._clear_started = steady_now
                if steady_now - self._clear_started < self.thresholds.clear_stability_sec:
                    return self._result(SupervisorEvent.NO_PROGRESS_PENDING, ages, collision, progress,
                                        movement_intent, recovery_delta, "clear recovery stability pending", ratio)
                self._temporary_stop_reached = False
                self._clear_started = None
                self._no_progress_started = steady_now
                return self._result(SupervisorEvent.PROGRESSING, ages, collision, True,
                                    movement_intent, recovery_delta, "clear recovery stable", ratio)
            self._clear_started = None

        if progress:
            return self._result(
                SupervisorEvent.COLLISION_SLOWDOWN if collision is CollisionClassification.SLOWDOWN else SupervisorEvent.PROGRESSING,
                ages, collision, True, movement_intent, recovery_delta, "measurable progress", ratio)

        if self._no_progress_started is None:
            self._no_progress_started = self._goal_started if self._goal_started is not None else steady_now
        if steady_now - self._no_progress_started >= self.thresholds.persistent_block_sec:
            event = SupervisorEvent.CONTROLLER_NO_PROGRESS
        elif collision is CollisionClassification.SLOWDOWN:
            event = SupervisorEvent.COLLISION_SLOWDOWN
        else:
            event = SupervisorEvent.NO_PROGRESS_PENDING
        return self._result(event, ages, collision, False, movement_intent,
                            recovery_delta, event.name, ratio)

    def _validate_now(self, steady_now: float) -> None:
        _finite("steady_now", steady_now, nonnegative=True)
        if self._last_now is not None and steady_now < self._last_now:
            raise ValueError("steady time moved backward")
        self._last_now = steady_now

    def _age(self, now: float, sample: object) -> float:
        age = now - float(getattr(sample, "steady_receipt_sec"))
        if age < 0.0:
            raise ValueError("sample receipt time is in the future")
        return age

    def _ages(self, now: float, inputs: SupervisorInputs) -> dict[str, Optional[float]]:
        pairs = {
            "feedback": inputs.feedback, "odometry": inputs.odometry, "tf": inputs.transform,
            "raw_command": inputs.raw_command, "safe_command": inputs.safe_command,
            "gate": inputs.gate, "collision_monitor": inputs.collision_monitor_valid,
            "localization": inputs.localization_valid, "controller": inputs.controller_valid,
            "adapter": inputs.adapter_health, "command_authority": inputs.command_authority_valid,
        }
        return {name: None if sample is None else self._age(now, sample) for name, sample in pairs.items()}

    def _health_failure(self, inputs: SupervisorInputs, ages: Mapping[str, Optional[float]]) -> Optional[SupervisorEvent]:
        health_timeout = self.thresholds.odometry_freshness_sec
        if inputs.command_authority_valid is not None and (
            ages["command_authority"] > health_timeout or not inputs.command_authority_valid.value
        ):
            return SupervisorEvent.COMMAND_AUTHORITY_INVALID
        if inputs.gate is None or ages["gate"] > health_timeout:
            return SupervisorEvent.GATE_FAULT
        if inputs.gate.fault_latched or inputs.gate.state is GateState.FAULT:
            return SupervisorEvent.GATE_FAULT
        if inputs.gate.state is not GateState.ARMED:
            return SupervisorEvent.GATE_DISARMED
        for sample, age_name, event in (
            (inputs.localization_valid, "localization", SupervisorEvent.LOCALIZATION_INVALID),
            (inputs.controller_valid, "controller", SupervisorEvent.CONTROLLER_INVALID),
            (inputs.collision_monitor_valid, "collision_monitor", SupervisorEvent.COLLISION_MONITOR_INVALID),
        ):
            if sample is None or ages[age_name] > health_timeout or not sample.value:
                return event
        if inputs.adapter_health is not None and (ages["adapter"] > health_timeout or not inputs.adapter_health.valid):
            return SupervisorEvent.ADAPTER_INVALID
        return None

    def _required_stale(self, inputs: SupervisorInputs, ages: Mapping[str, Optional[float]]) -> Optional[SupervisorEvent]:
        if inputs.odometry is None or ages["odometry"] > self.thresholds.odometry_freshness_sec:
            return SupervisorEvent.ODOMETRY_STALE
        if inputs.transform is None or ages["tf"] > self.thresholds.tf_freshness_sec:
            return SupervisorEvent.TF_STALE
        if inputs.feedback is None or ages["feedback"] > self.thresholds.feedback_freshness_sec:
            return SupervisorEvent.FEEDBACK_STALE
        if inputs.raw_command is None or inputs.safe_command is None:
            return SupervisorEvent.COMMAND_PAIR_STALE
        if (ages["raw_command"] > self.thresholds.command_pair_freshness_sec or
                ages["safe_command"] > self.thresholds.command_pair_freshness_sec or
                abs(inputs.raw_command.steady_receipt_sec - inputs.safe_command.steady_receipt_sec) >
                self.thresholds.command_pair_freshness_sec):
            return SupervisorEvent.COMMAND_PAIR_STALE
        return None

    def _measure_progress(self, feedback: FeedbackSample, odom: OdometrySample, transform: TfSample) -> bool:
        def reached(value: float) -> bool:
            return value > self.thresholds.minimum_improvement_m or math.isclose(
                value, self.thresholds.minimum_improvement_m, rel_tol=0.0, abs_tol=1e-12
            )

        distance_progress = self._baseline_distance is not None and reached(
            self._baseline_distance - feedback.distance_remaining_m
        )
        pose_progress = False
        if self._baseline_pose is not None:
            pose_progress = reached(max(
                math.hypot(odom.x - self._baseline_pose[0], odom.y - self._baseline_pose[1]),
                math.hypot(transform.x - self._baseline_pose[0], transform.y - self._baseline_pose[1]),
                math.hypot(feedback.x - self._baseline_pose[0], feedback.y - self._baseline_pose[1]),
            ))
        return bool(distance_progress or pose_progress)

    def _record_progress(self, now: float, feedback: FeedbackSample, odom: OdometrySample, transform: TfSample) -> None:
        self._baseline_distance = feedback.distance_remaining_m
        # Independent pose evidence uses the freshest caller-qualified family; TF is the base pose contract.
        self._baseline_pose = (transform.x, transform.y)
        self._no_progress_started = now

    def _collision(self, inputs: SupervisorInputs) -> tuple[CollisionClassification, bool, Optional[float]]:
        assert inputs.raw_command is not None and inputs.safe_command is not None
        raw, safe = inputs.raw_command, inputs.safe_command
        movement = abs(raw.linear_x) >= self.thresholds.linear_intent_mps or abs(raw.angular_z) >= self.thresholds.angular_intent_radps
        if not movement:
            return CollisionClassification.CLEAR, False, None
        safe_moving = abs(safe.linear_x) >= self.thresholds.linear_intent_mps or abs(safe.angular_z) >= self.thresholds.angular_intent_radps
        if not safe_moving:
            return CollisionClassification.STOP, True, 0.0
        raw_components = (raw.linear_x, raw.angular_z)
        safe_components = (safe.linear_x, safe.angular_z)
        compatible = all(r == 0.0 or s == 0.0 or math.copysign(1.0, r) == math.copysign(1.0, s)
                         for r, s in zip(raw_components, safe_components))
        component_thresholds = (self.thresholds.linear_intent_mps, self.thresholds.angular_intent_radps)
        ratios = [abs(s) / abs(r) for r, s, threshold in zip(raw_components, safe_components, component_thresholds)
                  if abs(r) >= threshold]
        ratio = max(ratios) if ratios else None
        if compatible and ratio is not None and ratio <= self.thresholds.slowdown_ratio:
            return CollisionClassification.SLOWDOWN, True, ratio
        return CollisionClassification.CLEAR, True, ratio

    def _reset_unsafe_evidence(self) -> None:
        self._stop_started = None
        self._clear_started = None

    def _result(self, event: SupervisorEvent, ages: Mapping[str, Optional[float]],
                collision: CollisionClassification = CollisionClassification.UNAVAILABLE,
                progress: bool = False, movement: bool = False, recovery_delta: int = 0,
                reason: str = "", ratio: Optional[float] = None) -> SupervisorResult:
        now = self._last_now
        return SupervisorResult(
            primary_event=event, collision=collision,
            progress=ProgressClassification.MEASURABLE if progress else ProgressClassification.NOT_MEASURABLE,
            movement_intent=movement, ages_sec=dict(ages),
            stop_age_sec=None if self._stop_started is None else now - self._stop_started,
            no_progress_age_sec=None if self._no_progress_started is None else now - self._no_progress_started,
            clear_age_sec=None if self._clear_started is None else now - self._clear_started,
            recovery_delta=recovery_delta, reason=reason,
            diagnostics={"safe_raw_ratio": ratio, "temporary_stop_reached": self._temporary_stop_reached},
        )
