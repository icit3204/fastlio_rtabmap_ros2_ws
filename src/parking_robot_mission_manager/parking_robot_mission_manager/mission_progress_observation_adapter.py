"""ROS observation conversion for passive mission progress supervision.

All converters copy scalar values into the immutable pure-core records.  A
malformed ROS observation is represented by ``None`` and never by a fabricated
zero-valued sample.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from .mission_progress_supervisor_core import (
    BoolHealthSample,
    CommandSample,
    FeedbackSample,
    GateHealthSample,
    GateState,
    OdometrySample,
    OptionalAdapterHealthSample,
    TfSample,
)


class CausalPairState(Enum):
    VALID = auto()
    PENDING = auto()
    AMBIGUOUS = auto()
    STALE = auto()
    # Compatibility aliases for evidence tooling written against A.7M.
    UNKNOWN = AMBIGUOUS
    NO_PAIR = PENDING


class CausalCollisionMeaning(Enum):
    CLEAR = auto()
    SLOWDOWN = auto()
    STOP = auto()
    NO_MOVEMENT = auto()


class CommandStreamPhase(Enum):
    ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW = auto()
    ACQUIRING_FIRST_COMMAND_PAIR_RAW_PENDING = auto()
    STREAM_ESTABLISHED = auto()


@dataclass(frozen=True)
class CommandObservationRecord:
    steady_receipt_sec: float
    linear_x: float
    angular_z: float
    epoch: int
    ordinal: int


@dataclass(frozen=True)
class CausalPairResult:
    state: CausalPairState
    raw: Optional[CommandObservationRecord] = None
    safe: Optional[CommandObservationRecord] = None
    meaning: Optional[CausalCollisionMeaning] = None
    drain_only: bool = False


class CausalCommandPairer:
    """Bounded semantic alignment for independently observed Twist streams.

    ``_frontier`` contains the possible last-matched raw ordinal for every
    live monotone alignment.  It never identifies one physical raw merely
    because all candidate pairs have the same collision meaning.
    """

    def __init__(self, freshness_sec: float = 0.25, capacity: int = 32,
                 linear_intent: float = 0.01, angular_intent: float = 0.02,
                 slowdown_ratio: float = 0.80) -> None:
        if not math.isfinite(freshness_sec) or freshness_sec <= 0.0:
            raise ValueError("freshness_sec must be positive and finite")
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 2:
            raise ValueError("capacity must be an integer of at least two")
        self.freshness_sec = freshness_sec
        self.capacity = capacity
        self.linear_intent = linear_intent
        self.angular_intent = angular_intent
        self.slowdown_ratio = slowdown_ratio
        self.epoch = 0
        self.active = False
        self.raw_history = deque(maxlen=capacity)
        self.safe_history = deque(maxlen=capacity)
        self._ordinal = 0
        self._frontier = {-1}
        self._epoch_floor_ordinal = -1
        self._prior_epoch: Optional[int] = None
        self._prior_tail_ordinals = deque(maxlen=capacity)
        self._last_safe_receipt_sec: Optional[float] = None
        self.current = CausalPairResult(CausalPairState.PENDING)
        self.unavailable_since_sec: Optional[float] = None
        self.command_stream_phase: Optional[CommandStreamPhase] = None
        self.no_raw_acquisition_start_sec: Optional[float] = None

    def start_epoch(self, now: float) -> int:
        self._validate_time(now)
        self._prune(now)
        self._prior_epoch = self.epoch if self.active else None
        self._prior_tail_ordinals.clear()
        if self._prior_epoch is not None:
            self._prior_tail_ordinals.extend(
                raw.ordinal for raw in self.raw_history
                if raw.epoch == self._prior_epoch and (
                    self._last_safe_receipt_sec is None
                    or raw.steady_receipt_sec > self._last_safe_receipt_sec))
            if not self._prior_tail_ordinals:
                self._prior_epoch = None
        self.epoch += 1
        self.active = True
        self._epoch_floor_ordinal = self._ordinal
        self._frontier = {self._epoch_floor_ordinal}
        self.current = CausalPairResult(CausalPairState.PENDING)
        self.unavailable_since_sec = None
        self.command_stream_phase = CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW
        self.no_raw_acquisition_start_sec = now
        return self.epoch

    def end_epoch(self, now: float) -> None:
        self._validate_time(now)
        self._prune(now)
        self.active = False
        self.current = CausalPairResult(CausalPairState.PENDING)
        self.unavailable_since_sec = None
        self.command_stream_phase = None
        self.no_raw_acquisition_start_sec = None

    def reset(self) -> None:
        self.raw_history.clear(); self.safe_history.clear()
        self.active = False
        self.current = CausalPairResult(CausalPairState.PENDING)
        self.unavailable_since_sec = None
        self._frontier = {-1}
        self._epoch_floor_ordinal = -1
        self._prior_epoch = None
        self._prior_tail_ordinals.clear()
        self._last_safe_receipt_sec = None
        self.command_stream_phase = None
        self.no_raw_acquisition_start_sec = None

    def observe_raw(self, sample: CommandSample) -> CommandObservationRecord:
        self._ordinal += 1
        record = CommandObservationRecord(sample.steady_receipt_sec, sample.linear_x,
                                          sample.angular_z, self.epoch, self._ordinal)
        self._prune(sample.steady_receipt_sec)
        self.raw_history.append(record)
        # The formerly committed pair cannot describe this newer command.
        # Enter bounded acquisition until a causally following safe arrives.
        self.current = CausalPairResult(CausalPairState.PENDING)
        if (self.active and self.command_stream_phase
                is CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW):
            self.command_stream_phase = CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_RAW_PENDING
            self.no_raw_acquisition_start_sec = None
            self.unavailable_since_sec = sample.steady_receipt_sec
        elif self.active and self.unavailable_since_sec is None:
            self.unavailable_since_sec = sample.steady_receipt_sec
        return record

    def observe_safe(self, sample: CommandSample) -> CausalPairResult:
        self._ordinal += 1
        safe = CommandObservationRecord(sample.steady_receipt_sec, sample.linear_x,
                                        sample.angular_z, -1, self._ordinal)
        self._prune(sample.steady_receipt_sec)
        self.safe_history.append(safe)
        prior = [item for item in self._eligible_candidates(safe, self._prior_epoch, None)
                 if item[0].ordinal in self._prior_tail_ordinals]
        if prior:
            meanings = {meaning for _, meaning in prior}
            if len(meanings) != 1:
                return self._unavailable(CausalPairState.AMBIGUOUS, safe.steady_receipt_sec)
            raw, meaning = min(prior, key=lambda item: item[0].ordinal)
            self._prior_tail_ordinals.remove(raw.ordinal)
            if not self._prior_tail_ordinals:
                self._prior_epoch = None
            self._last_safe_receipt_sec = safe.steady_receipt_sec
            # Prior output is deliberately drain-only; no physical identity is
            # committed and no current policy class is established.
            return CausalPairResult(CausalPairState.VALID, raw, safe, meaning, True)

        candidates = self._eligible_candidates(safe, self.epoch, self._frontier)
        if not candidates:
            self._last_safe_receipt_sec = safe.steady_receipt_sec
            return self._unavailable(CausalPairState.PENDING, safe.steady_receipt_sec)

        meanings = {meaning for _, meaning in candidates}
        # Every observed safe must be assigned to one observed raw in a live
        # alignment.  Raw observations may be skipped (safe observer loss),
        # while a safe whose raw was not observed remains PENDING rather than
        # fabricating identity.  Retain every feasible last-raw state; never
        # select one merely because their semantic meanings agree.
        self._frontier = {raw.ordinal for raw, _ in candidates}
        self._normalize_frontier()
        if len(meanings) != 1:
            self._last_safe_receipt_sec = safe.steady_receipt_sec
            return self._unavailable(CausalPairState.AMBIGUOUS, safe.steady_receipt_sec)

        witness, meaning = min(candidates, key=lambda item: item[0].ordinal)
        result = CausalPairResult(CausalPairState.VALID, witness, safe, meaning, not self.active)
        if not self.active:
            self.current = CausalPairResult(CausalPairState.PENDING)
            self._last_safe_receipt_sec = safe.steady_receipt_sec
            return result
        self.current = result
        self.unavailable_since_sec = None
        self.command_stream_phase = CommandStreamPhase.STREAM_ESTABLISHED
        self.no_raw_acquisition_start_sec = None
        self._last_safe_receipt_sec = safe.steady_receipt_sec
        return result

    def no_raw_acquisition_age(self, now: float) -> Optional[float]:
        self._validate_time(now)
        if (self.command_stream_phase
                is not CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW
                or self.no_raw_acquisition_start_sec is None):
            return None
        return max(0.0, now - self.no_raw_acquisition_start_sec)

    def pair_liveness_age(self, now: float) -> Optional[float]:
        self._validate_time(now)
        if self.unavailable_since_sec is None:
            return None
        return max(0.0, now - self.unavailable_since_sec)

    def pending_within_budget(self, now: float) -> bool:
        self._validate_time(now)
        return (self.current.state in (CausalPairState.PENDING, CausalPairState.AMBIGUOUS)
                and self.unavailable_since_sec is not None
                and now - self.unavailable_since_sec < self.freshness_sec)

    def adjudicate(self, now: float) -> CausalPairResult:
        """Advance only the non-sliding liveness deadline."""
        self._validate_time(now)
        self._prune(now)
        if (self.current.state is not CausalPairState.VALID
                and self.unavailable_since_sec is not None
                and now - self.unavailable_since_sec >= self.freshness_sec):
            self.current = CausalPairResult(CausalPairState.STALE)
        return self.current

    @property
    def frontier_state_count(self) -> int:
        return len(self._frontier)

    def _unavailable(self, state: CausalPairState, now: float) -> CausalPairResult:
        self.current = CausalPairResult(state)
        if (self.unavailable_since_sec is None
                and self.command_stream_phase
                is not CommandStreamPhase.ACQUIRING_FIRST_COMMAND_PAIR_NO_RAW):
            self.unavailable_since_sec = now
        return self.current

    def _prune(self, now: float) -> None:
        cutoff = now - self.freshness_sec
        while self.raw_history and self.raw_history[0].steady_receipt_sec < cutoff:
            self.raw_history.popleft()
        while self.safe_history and self.safe_history[0].steady_receipt_sec < cutoff:
            self.safe_history.popleft()
        if self._prior_epoch is not None and not any(
                raw.epoch == self._prior_epoch for raw in self.raw_history):
            self._prior_epoch = None
            self._prior_tail_ordinals.clear()
        self._normalize_frontier()

    def _normalize_frontier(self) -> None:
        current_ordinals = [raw.ordinal for raw in self.raw_history if raw.epoch == self.epoch]
        floor = self._epoch_floor_ordinal if not current_ordinals else min(current_ordinals) - 1
        self._frontier = {floor if state < floor else state for state in self._frontier}
        # There is one state per retained raw plus the skip-prefix floor.
        if len(self._frontier) > self.capacity + 1:
            raise RuntimeError("causal alignment frontier exceeded deterministic bound")

    def _eligible_candidates(self, safe: CommandObservationRecord, epoch: Optional[int],
                             frontier: Optional[set[int]]) -> list[tuple[CommandObservationRecord,
                                                                        CausalCollisionMeaning]]:
        if epoch is None:
            return []
        candidates = []
        for raw in self.raw_history:
            if raw.epoch != epoch or raw.steady_receipt_sec > safe.steady_receipt_sec:
                continue
            if safe.steady_receipt_sec - raw.steady_receipt_sec > self.freshness_sec:
                continue
            if frontier is not None and not any(raw.ordinal > state for state in frontier):
                continue
            if not self._transformation_feasible(raw, safe):
                continue
            candidates.append((raw, self._meaning(raw, safe)))
        return candidates

    @staticmethod
    def _validate_time(now: float) -> None:
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("receipt time must be finite and nonnegative")

    def _moving(self, record: CommandObservationRecord) -> bool:
        return (abs(record.linear_x) >= self.linear_intent
                or abs(record.angular_z) >= self.angular_intent)

    def _transformation_feasible(self, raw: CommandObservationRecord,
                                 safe: CommandObservationRecord) -> bool:
        """Reject assignments Collision Monitor cannot causally produce.

        This is deliberately separate from semantic classification: the
        unchanged supervisor classifies an already-supplied sign-changing
        pair as CLEAR, but Collision Monitor cannot introduce such a sign
        change and the alignment frontier must not use it as an explanation.
        """
        raw_moving, safe_moving = self._moving(raw), self._moving(safe)
        if not raw_moving:
            return not safe_moving
        if not safe_moving:
            return True
        return all(
            not (abs(r) >= threshold and abs(s) >= threshold)
            or math.copysign(1.0, r) == math.copysign(1.0, s)
            for r, s, threshold in (
                (raw.linear_x, safe.linear_x, self.linear_intent),
                (raw.angular_z, safe.angular_z, self.angular_intent)))

    def _meaning(self, raw: CommandObservationRecord,
                 safe: CommandObservationRecord) -> CausalCollisionMeaning:
        raw_moving, safe_moving = self._moving(raw), self._moving(safe)
        if not raw_moving:
            return CausalCollisionMeaning.NO_MOVEMENT
        if not safe_moving:
            return CausalCollisionMeaning.STOP
        components = ((raw.linear_x, safe.linear_x, self.linear_intent),
                      (raw.angular_z, safe.angular_z, self.angular_intent))
        compatible = all(r == 0.0 or s == 0.0 or math.copysign(1.0, r) == math.copysign(1.0, s)
                         for r, s, _ in components)
        ratios = [abs(s) / abs(r) for r, s, threshold in components if abs(r) >= threshold]
        return (CausalCollisionMeaning.SLOWDOWN if compatible and ratios
                and max(ratios) <= self.slowdown_ratio
                else CausalCollisionMeaning.CLEAR)


def _yaw(orientation) -> float:
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite quaternion")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _duration_sec(duration) -> float:
    sec = int(duration.sec)
    nanosec = int(duration.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("invalid duration")
    return sec + nanosec / 1_000_000_000.0


def feedback_sample(msg, steady_receipt_sec: float) -> Optional[FeedbackSample]:
    try:
        feedback = msg.feedback if hasattr(msg, "feedback") else msg
        pose = feedback.current_pose.pose
        return FeedbackSample(
            steady_receipt_sec=float(steady_receipt_sec),
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=_yaw(pose.orientation),
            navigation_time_sec=_duration_sec(feedback.navigation_time),
            estimated_time_remaining_sec=_duration_sec(feedback.estimated_time_remaining),
            number_of_recoveries=int(feedback.number_of_recoveries),
            distance_remaining_m=float(feedback.distance_remaining),
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def odometry_sample(msg, steady_receipt_sec: float) -> Optional[OdometrySample]:
    try:
        return OdometrySample(
            steady_receipt_sec=float(steady_receipt_sec),
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=_yaw(msg.pose.pose.orientation),
            linear_x=float(msg.twist.twist.linear.x),
            angular_z=float(msg.twist.twist.angular.z),
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def transform_sample(msg, steady_receipt_sec: float) -> Optional[TfSample]:
    try:
        return TfSample(
            steady_receipt_sec=float(steady_receipt_sec),
            x=float(msg.transform.translation.x),
            y=float(msg.transform.translation.y),
            yaw=_yaw(msg.transform.rotation),
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def command_sample(msg, steady_receipt_sec: float) -> Optional[CommandSample]:
    try:
        return CommandSample(
            steady_receipt_sec=float(steady_receipt_sec),
            linear_x=float(msg.linear.x),
            angular_z=float(msg.angular.z),
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def bool_health_sample(msg, steady_receipt_sec: float) -> Optional[BoolHealthSample]:
    try:
        if not isinstance(msg.data, bool):
            return None
        return BoolHealthSample(float(steady_receipt_sec), msg.data)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def gate_health_sample(msg: DiagnosticStatus, steady_receipt_sec: float) -> Optional[GateHealthSample]:
    try:
        fields = {str(item.key): str(item.value) for item in msg.values}
        state_text = fields["state"]
        fault_text = fields["fault_latched"].lower()
        if state_text not in GateState.__members__ or fault_text not in ("true", "false"):
            return None
        return GateHealthSample(
            float(steady_receipt_sec), GateState[state_text], fault_text == "true",
            fields.get("reason_code", str(msg.message)),
        )
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return None


def adapter_health_sample(msg: DiagnosticArray, steady_receipt_sec: float) -> Optional[OptionalAdapterHealthSample]:
    try:
        matching = [status for status in msg.status if status.name == "mock_wheelchair_cmd_adapter"]
        if len(matching) != 1:
            return None
        status = matching[0]
        level = status.level[0] if isinstance(status.level, bytes) and len(status.level) == 1 else int(status.level)
        ok_level = DiagnosticStatus.OK[0] if isinstance(DiagnosticStatus.OK, bytes) else int(DiagnosticStatus.OK)
        warn_level = DiagnosticStatus.WARN[0] if isinstance(DiagnosticStatus.WARN, bytes) else int(DiagnosticStatus.WARN)
        if level not in (ok_level, warn_level):
            return None
        return OptionalAdapterHealthSample(
            float(steady_receipt_sec), level == ok_level, str(status.message)
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def transform_stamp_key(msg) -> Optional[tuple[int, int]]:
    """Return source evidence identity; zero denotes static/non-refreshing TF."""
    try:
        key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if key[0] < 0 or not 0 <= key[1] < 1_000_000_000 or key == (0, 0):
            return None
        return key
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
