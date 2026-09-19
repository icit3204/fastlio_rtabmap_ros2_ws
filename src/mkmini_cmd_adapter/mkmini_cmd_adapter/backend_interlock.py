"""MK-mini physical-backend motion admission below the Generic Gate.

The core is deliberately transport and ROS independent.  It never creates
motion authority from a Generic Gate state: an already-safe command is only
admitted after installed-unit feedback and a continuous N+0+0 heartbeat have
qualified the chassis.  Any loss is latched and requires an explicit re-arm.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .codec import RunningMode
from .model import Gear


class BackendInterlockState(str, Enum):
    DISARMED = "DISARMED"
    N_ZERO_PRIMING = "N_ZERO_PRIMING"
    WAITING_FOR_VALID_FEEDBACK = "WAITING_FOR_VALID_FEEDBACK"
    MOTION_PERMITTED = "MOTION_PERMITTED"
    MOTION_NOT_PERMITTED = "MOTION_NOT_PERMITTED"


@dataclass(frozen=True)
class BackendInterlockConfig:
    feedback_timeout_sec: float = 0.050
    heartbeat_timeout_sec: float = 0.030
    priming_duration_sec: float = 0.250
    stable_feedback_duration_sec: float = 0.500
    zero_speed_tolerance_mps: float = 0.005
    zero_steering_tolerance_deg: float = 0.10
    # Temporary MK-mini commissioning contract established by the R16 rig
    # evidence: an actual command transition may lag, but an uncommanded
    # deviation, wrong sign, excessive error, or a transition which never
    # settles still fails closed.
    maximum_transition_steering_error_deg: float = 15.0
    steering_transition_timeout_sec: float = 1.5
    maximum_motion_speed_mps: float = 0.05

    def __post_init__(self) -> None:
        positive = (
            self.feedback_timeout_sec,
            self.heartbeat_timeout_sec,
            self.priming_duration_sec,
            self.stable_feedback_duration_sec,
            self.maximum_transition_steering_error_deg,
            self.steering_transition_timeout_sec,
            self.maximum_motion_speed_mps,
        )
        nonnegative = (self.zero_speed_tolerance_mps, self.zero_steering_tolerance_deg)
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("interlock timing and motion limits must be finite and positive")
        if any(not math.isfinite(value) or value < 0.0 for value in nonnegative):
            raise ValueError("interlock zero tolerances must be finite and non-negative")


@dataclass(frozen=True)
class BackendFeedback:
    ctrl_age_sec: float | None = None
    diagnostic_age_sec: float | None = None
    ctrl_checksum_valid: bool = False
    diagnostic_checksum_valid: bool = False
    ctrl_alive_contiguous: bool = False
    diagnostic_alive_contiguous: bool = False
    gear: int | Gear | None = None
    speed_mps: float | None = None
    steering_deg: float | None = None
    mode: RunningMode | int | None = None
    vehicle_fault_level: int | None = None
    auto_can_error: bool | None = None
    can_state: str = "UNKNOWN"
    hard_fault: bool = False


@dataclass(frozen=True)
class BackendCommand:
    gear: Gear
    speed_mps: float
    steering_deg: float

    @classmethod
    def safe_neutral(cls) -> "BackendCommand":
        return cls(Gear.N, 0.0, 0.0)


@dataclass(frozen=True)
class BackendInterlockResult:
    state: BackendInterlockState
    motion_permitted: bool
    command_admitted: bool
    output: BackendCommand
    reasons: tuple[str, ...]
    stable_for_sec: float


class MkminiBackendInterlock:
    """Latched chassis-specific admission state machine.

    ``step`` must be called for every command heartbeat.  Silence therefore
    cannot preserve qualification: ``watchdog`` must also be called by the
    backend scheduler while no command is available.
    """

    def __init__(self, config: BackendInterlockConfig | None = None) -> None:
        self.config = config or BackendInterlockConfig()
        self.state = BackendInterlockState.DISARMED
        self._last_now: float | None = None
        self._last_heartbeat: float | None = None
        self._priming_since: float | None = None
        self._stable_since: float | None = None
        self._trip_reasons: tuple[str, ...] = ()
        self._last_admitted_command = BackendCommand.safe_neutral()
        self._steering_transition_started: float | None = None
        self._steering_transition_target: float | None = None

    def arm(self, now_sec: float) -> BackendInterlockResult:
        if not math.isfinite(now_sec):
            raise ValueError("arm timestamp must be finite")
        self.state = BackendInterlockState.N_ZERO_PRIMING
        self._last_now = now_sec
        self._last_heartbeat = None
        self._priming_since = None
        self._stable_since = None
        self._trip_reasons = ()
        self._last_admitted_command = BackendCommand.safe_neutral()
        self._steering_transition_started = None
        self._steering_transition_target = None
        return self._result(False, False, ("N_ZERO_PRIMING",), 0.0)

    def disarm(self) -> BackendInterlockResult:
        self.state = BackendInterlockState.DISARMED
        self._last_heartbeat = None
        self._priming_since = None
        self._stable_since = None
        self._trip_reasons = ()
        self._last_admitted_command = BackendCommand.safe_neutral()
        self._steering_transition_started = None
        self._steering_transition_target = None
        return self._result(False, False, ("DISARMED",), 0.0)

    def _result(
        self, permitted: bool, admitted: bool, reasons: tuple[str, ...], stable: float
    ) -> BackendInterlockResult:
        return BackendInterlockResult(
            self.state, permitted, admitted, BackendCommand.safe_neutral(), reasons, stable
        )

    def _trip(self, reasons: tuple[str, ...]) -> BackendInterlockResult:
        self.state = BackendInterlockState.MOTION_NOT_PERMITTED
        self._stable_since = None
        self._trip_reasons = tuple(dict.fromkeys(reasons)) or ("INTERLOCK_INVALID",)
        self._steering_transition_started = None
        self._steering_transition_target = None
        return self._result(False, False, self._trip_reasons, 0.0)

    @staticmethod
    def _finite_nonnegative(value: float | None) -> bool:
        return value is not None and math.isfinite(value) and value >= 0.0

    def _base_feedback_reasons(self, feedback: BackendFeedback) -> tuple[str, ...]:
        reasons: list[str] = []
        if not self._finite_nonnegative(feedback.ctrl_age_sec):
            reasons.append("CTRL_FEEDBACK_MISSING_OR_MALFORMED")
        elif float(feedback.ctrl_age_sec) > self.config.feedback_timeout_sec:
            reasons.append("CTRL_FEEDBACK_STALE")
        if not self._finite_nonnegative(feedback.diagnostic_age_sec):
            reasons.append("DIAGNOSTIC_FEEDBACK_MISSING_OR_MALFORMED")
        elif float(feedback.diagnostic_age_sec) > self.config.feedback_timeout_sec:
            reasons.append("DIAGNOSTIC_FEEDBACK_STALE")
        if feedback.ctrl_checksum_valid is not True or feedback.diagnostic_checksum_valid is not True:
            reasons.append("FEEDBACK_CHECKSUM_INVALID")
        if feedback.ctrl_alive_contiguous is not True or feedback.diagnostic_alive_contiguous is not True:
            reasons.append("FEEDBACK_ALIVE_INVALID")
        if feedback.can_state != "ERROR-ACTIVE":
            reasons.append("CAN_UNHEALTHY")
        level = feedback.vehicle_fault_level
        if isinstance(level, bool) or not isinstance(level, int) or level not in (0, 1, 2, 3):
            reasons.append("FAULT_LEVEL_MALFORMED")
        elif level >= 2:
            reasons.append(f"HARD_FAULT_LEVEL_{level}")
        if feedback.hard_fault is not False:
            reasons.append("HARD_DIAGNOSTIC_FAULT")
        return tuple(dict.fromkeys(reasons))

    def _qualified_feedback_reasons(self, feedback: BackendFeedback) -> tuple[str, ...]:
        reasons = list(self._base_feedback_reasons(feedback))
        if feedback.gear != Gear.N:
            reasons.append("FEEDBACK_GEAR_NOT_N")
        if not self._finite_nonnegative(feedback.speed_mps):
            reasons.append("FEEDBACK_SPEED_MALFORMED")
        elif abs(float(feedback.speed_mps)) > self.config.zero_speed_tolerance_mps:
            reasons.append("FEEDBACK_SPEED_NOT_ZERO")
        if feedback.steering_deg is None or not math.isfinite(feedback.steering_deg):
            reasons.append("FEEDBACK_STEERING_MALFORMED")
        elif abs(feedback.steering_deg) > self.config.zero_steering_tolerance_deg:
            reasons.append("FEEDBACK_STEERING_NOT_ZERO")
        if feedback.mode != RunningMode.AUTO:
            reasons.append("FEEDBACK_MODE_NOT_AUTO")
        if feedback.auto_can_error is not False:
            reasons.append("AUTO_CAN_NOT_FALSE")
        return tuple(dict.fromkeys(reasons))

    def assess_feedback(
        self, feedback: BackendFeedback, *, require_n_auto: bool = True
    ) -> tuple[str, ...]:
        """Return current fail-closed reasons without changing interlock state."""

        if not isinstance(feedback, BackendFeedback):
            return ("FEEDBACK_MALFORMED",)
        return (
            self._qualified_feedback_reasons(feedback)
            if require_n_auto
            else self._base_feedback_reasons(feedback)
        )

    def _motion_feedback_reasons(
        self, now_sec: float, feedback: BackendFeedback, command: BackendCommand
    ) -> tuple[str, ...]:
        reasons = list(self._base_feedback_reasons(feedback))
        if feedback.mode != RunningMode.AUTO:
            reasons.append("FEEDBACK_MODE_NOT_AUTO")
        if feedback.auto_can_error is not False:
            reasons.append("AUTO_CAN_NOT_FALSE")
        # N is accepted during the command/feedback transition and D is
        # accepted during forward motion or its zero-speed stopping phase.
        if feedback.gear not in (Gear.N, Gear.D):
            reasons.append("FEEDBACK_GEAR_NOT_FORWARD_SAFE")
        if not self._finite_nonnegative(feedback.speed_mps):
            reasons.append("FEEDBACK_SPEED_MALFORMED")
        elif float(feedback.speed_mps) > self.config.maximum_motion_speed_mps + 1.0e-9:
            reasons.append("FEEDBACK_MOTION_OVERSPEED")
        if feedback.steering_deg is None or not math.isfinite(feedback.steering_deg):
            reasons.append("FEEDBACK_STEERING_MALFORMED")
        else:
            steering_error = abs(feedback.steering_deg - command.steering_deg)
            if steering_error <= self.config.zero_steering_tolerance_deg:
                self._steering_transition_started = None
                self._steering_transition_target = None
            else:
                # Stationary priming may establish a new target. During
                # motion, only an actual commanded transition gets a short,
                # bounded actuator-response interval; an uncommanded error,
                # wrong sign, excessive deviation, or failure to settle is
                # still fail-closed.
                stationary_priming = (
                    command.speed_mps == 0.0
                    and self._finite_nonnegative(feedback.speed_mps)
                    and abs(float(feedback.speed_mps)) <= self.config.zero_speed_tolerance_mps
                )
                commanded_transition = (
                    abs(command.steering_deg - self._last_admitted_command.steering_deg)
                    > self.config.zero_steering_tolerance_deg
                )
                continuing_transition = self._steering_transition_started is not None
                if stationary_priming:
                    if not continuing_transition:
                        self._steering_transition_started = now_sec
                    self._steering_transition_target = command.steering_deg
                elif commanded_transition or continuing_transition:
                    if not continuing_transition:
                        self._steering_transition_started = now_sec
                    self._steering_transition_target = command.steering_deg
                    elapsed = now_sec - (
                        self._steering_transition_started
                        if self._steering_transition_started is not None else now_sec)
                    wrong_sign = (abs(command.steering_deg) >= 2.0
                                  and abs(feedback.steering_deg) >= 2.0
                                  and command.steering_deg * feedback.steering_deg < 0.0)
                    if wrong_sign:
                        reasons.append("FEEDBACK_STEERING_SIGN_INVALID")
                    elif steering_error > self.config.maximum_transition_steering_error_deg:
                        reasons.append("FEEDBACK_STEERING_TRANSITION_ERROR")
                    elif elapsed > self.config.steering_transition_timeout_sec:
                        reasons.append("FEEDBACK_STEERING_TRANSITION_TIMEOUT")
                else:
                    reasons.append("UNEXPECTED_FEEDBACK_STEERING")
        return tuple(dict.fromkeys(reasons))

    @staticmethod
    def _command_reasons(command: BackendCommand) -> tuple[str, ...]:
        if not isinstance(command.gear, Gear):
            return ("COMMAND_GEAR_MALFORMED",)
        if not math.isfinite(command.speed_mps) or command.speed_mps < 0.0:
            return ("COMMAND_SPEED_MALFORMED",)
        if not math.isfinite(command.steering_deg):
            return ("COMMAND_STEERING_MALFORMED",)
        return ()

    @staticmethod
    def _is_n_zero(command: BackendCommand) -> bool:
        return command.gear is Gear.N and command.speed_mps == 0.0 and command.steering_deg == 0.0

    def _time_reasons(self, now_sec: float) -> tuple[str, ...]:
        if not math.isfinite(now_sec):
            return ("TIMESTAMP_INVALID",)
        if self._last_now is not None and now_sec < self._last_now:
            return ("TIMESTAMP_REVERSAL",)
        return ()

    def watchdog(self, now_sec: float, feedback: BackendFeedback) -> BackendInterlockResult:
        time_reasons = self._time_reasons(now_sec)
        if time_reasons:
            return self._trip(time_reasons)
        self._last_now = now_sec
        if self.state is BackendInterlockState.DISARMED:
            return self._result(False, False, ("DISARMED",), 0.0)
        if self.state is BackendInterlockState.MOTION_NOT_PERMITTED:
            return self._result(False, False, self._trip_reasons, 0.0)
        feedback_reasons = self._base_feedback_reasons(feedback)
        if feedback_reasons:
            return self._trip(feedback_reasons)
        if self._last_heartbeat is None or now_sec - self._last_heartbeat > self.config.heartbeat_timeout_sec:
            return self._trip(("N_ZERO_HEARTBEAT_STALE",))
        if self.state in (BackendInterlockState.WAITING_FOR_VALID_FEEDBACK, BackendInterlockState.MOTION_PERMITTED):
            qualified_reasons = (
                self._motion_feedback_reasons(now_sec, feedback, self._last_admitted_command)
                if self.state is BackendInterlockState.MOTION_PERMITTED
                else self._qualified_feedback_reasons(feedback)
            )
            if qualified_reasons:
                if self.state is BackendInterlockState.MOTION_PERMITTED or self._stable_since is not None:
                    return self._trip(qualified_reasons)
                return self._result(False, False, qualified_reasons, 0.0)
        stable = 0.0 if self._stable_since is None else max(0.0, now_sec - self._stable_since)
        return self._result(self.state is BackendInterlockState.MOTION_PERMITTED, False, (self.state.value,), stable)

    def note_heartbeat_emitted(self, now_sec: float) -> None:
        """Record a heartbeat only after the transport has actually sent it."""
        self._record_heartbeat(now_sec)

    def note_native_sender_healthy(self, now_sec: float) -> None:
        """Record a supervisor tick after validating the native sender.

        The native process owns the physical 30 ms CAN watchdog and exits on
        a real transmit gap. This timestamp tracks the Python interlock loop,
        without treating delayed native status datagrams as missing CAN TX.
        """
        self._record_heartbeat(now_sec)

    def _record_heartbeat(self, now_sec: float) -> None:
        if not math.isfinite(now_sec):
            raise ValueError("heartbeat timestamp must be finite")
        # Native sender reports may be consumed by the supervisor after its
        # own newer sample timestamp.  Heartbeat ordering is independent of
        # supervisor sampling; only a reversal relative to the prior physical
        # send is invalid.
        if self._last_heartbeat is not None and now_sec < self._last_heartbeat:
            self._trip(("TIMESTAMP_REVERSAL",))
            return
        self._last_heartbeat = now_sec

    def step(
        self, now_sec: float, command: BackendCommand, feedback: BackendFeedback,
        *, heartbeat_emitted: bool = True,
    ) -> BackendInterlockResult:
        if self.state is BackendInterlockState.DISARMED:
            return self._result(False, False, ("DISARMED",), 0.0)
        if self.state is BackendInterlockState.MOTION_NOT_PERMITTED:
            return self._result(False, False, self._trip_reasons, 0.0)
        time_reasons = self._time_reasons(now_sec)
        command_reasons = self._command_reasons(command)
        if time_reasons or command_reasons:
            return self._trip(time_reasons + command_reasons)
        if self._last_heartbeat is not None and now_sec - self._last_heartbeat > self.config.heartbeat_timeout_sec:
            return self._trip(("N_ZERO_HEARTBEAT_STALE",))
        self._last_now = now_sec
        # The legacy standalone runner emits and evaluates in one call.  The
        # ROS physical backend supplies ``False`` and records the timestamp
        # from its dedicated sender after a successful SocketCAN send.
        if heartbeat_emitted:
            self._last_heartbeat = now_sec

        base_reasons = self._base_feedback_reasons(feedback)
        if base_reasons:
            return self._trip(base_reasons)

        if self.state is not BackendInterlockState.MOTION_PERMITTED and not self._is_n_zero(command):
            return self._trip(("NONZERO_OR_DRIVE_BEFORE_PERMISSION",))

        if self.state is BackendInterlockState.N_ZERO_PRIMING:
            if self._priming_since is None:
                self._priming_since = now_sec
            primed_for = now_sec - self._priming_since
            if primed_for >= self.config.priming_duration_sec:
                self.state = BackendInterlockState.WAITING_FOR_VALID_FEEDBACK
                self._stable_since = None
            return self._result(False, True, (self.state.value,), 0.0)

        qualified_reasons = (
            self._motion_feedback_reasons(now_sec, feedback, command)
            if self.state is BackendInterlockState.MOTION_PERMITTED
            else self._qualified_feedback_reasons(feedback)
        )
        if qualified_reasons:
            if self._stable_since is not None:
                return self._trip(qualified_reasons)
            return self._result(False, True, qualified_reasons, 0.0)

        if self.state is BackendInterlockState.WAITING_FOR_VALID_FEEDBACK:
            if self._stable_since is None:
                self._stable_since = now_sec
            stable_for = now_sec - self._stable_since
            if stable_for >= self.config.stable_feedback_duration_sec:
                self.state = BackendInterlockState.MOTION_PERMITTED
                return self._result(True, True, ("QUALIFIED",), stable_for)
            return self._result(False, True, ("STABLE_FEEDBACK_PENDING",), stable_for)

        # Permission exists only while the same feedback contract remains true.
        if command.gear not in (Gear.N, Gear.D):
            return self._trip(("GEAR_NOT_ADMITTED",))
        if command.speed_mps > self.config.maximum_motion_speed_mps:
            return self._trip(("MOTION_SPEED_LIMIT_EXCEEDED",))
        if command.gear is Gear.N and not self._is_n_zero(command):
            return self._trip(("NONZERO_NEUTRAL_COMMAND",))
        self._last_admitted_command = command
        return BackendInterlockResult(
            self.state, True, True, command, ("MOTION_PERMITTED",),
            max(0.0, now_sec - (self._stable_since or now_sec)),
        )


__all__ = [
    "BackendCommand",
    "BackendFeedback",
    "BackendInterlockConfig",
    "BackendInterlockResult",
    "BackendInterlockState",
    "MkminiBackendInterlock",
]
