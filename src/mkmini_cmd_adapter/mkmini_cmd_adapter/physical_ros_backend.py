"""The sole physical MK-mini ROS endpoint below the Generic Gate.

This node accepts only the Gate's stamped safe Twist.  It has no direct Nav2
or ``/cmd_vel`` input and owns the only SocketCAN writer in physical mode.
The normal mode retains the feedback interlock.  An explicit professor-
authorized ``TEMPORARY_MKMINI_OPEN_LOOP`` mode exists only for the development
MK-mini; it keeps feedback diagnostic and admits motion solely from the Gate's
bounded, fresh command.  It is not the final-robot policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import json
import math
import os
import threading
import time

try:  # Keep the pure command policy importable by non-ROS unit tests.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from std_srvs.srv import SetBool, Trigger
except ImportError:  # pragma: no cover - exercised only outside ROS runtime
    rclpy = None
    DiagnosticStatus = KeyValue = TwistStamped = SetBool = Trigger = None
    Node = object

from .backend_interlock import (
    BackendCommand, BackendFeedback, BackendInterlockConfig,
    BackendInterlockState, MkminiBackendInterlock,
)
from .backend_zero_heartbeat import LiveCanMonitor
from .heartbeat_scheduler import timing_summary
from .kinematics import MkminiKinematicsCore
from .model import Gear
from .native_sender import NativeSender


SAFE_COMMAND_TOPIC = "/vehicle_cmd_safe"
STATE_TOPIC = "/mkmini_physical_backend/state"
ENABLE_SERVICE = "/mkmini_physical_backend/enable"
R16_STATIC_STEERING_SERVICE = "/mkmini_physical_backend/r16_static_steering"
R18_POSITIVE_STEERING_SERVICE = "/mkmini_physical_backend/r18_positive_steering"
TEMPORARY_OPEN_LOOP_MODE = "TEMPORARY_MKMINI_OPEN_LOOP"
CLOSED_LOOP_MODE = "FEEDBACK_INTERLOCKED"
EPSILON = 1.0e-9
# The Gate's qualified safe-command heartbeat is 20 Hz.  A D output cannot
# survive a missing backend supervisor for longer than the unchanged 50 ms
# feedback-freshness bound; thereafter the independent sender emits N/zero.
SUPERVISION_OUTPUT_MAX_AGE_SEC = .050


@dataclass(frozen=True)
class SafeTwistPolicy:
    command_timeout_sec: float = .25
    command_speed_ceiling_mps: float = .040
    command_steering_ceiling_deg: float = 30.0


class PhysicalCommandPolicy:
    """ROS-free safe-Twist validation and 1.75m Ackermann conversion."""

    def __init__(self, policy: SafeTwistPolicy | None = None) -> None:
        self.policy = policy or SafeTwistPolicy()
        self._kinematics = MkminiKinematicsCore()
        self._received: float | None = None
        self._command = BackendCommand.safe_neutral()
        self._reason = "SAFE_COMMAND_MISSING"

    def accept(self, *, linear_x: float, angular_z: float, unsupported_axes: tuple[float, ...], now: float) -> None:
        values = (linear_x, angular_z, *unsupported_axes, now)
        self._received = now
        if not all(math.isfinite(value) for value in values):
            self._command, self._reason = BackendCommand.safe_neutral(), "SAFE_COMMAND_NONFINITE"
            return
        if any(abs(value) > EPSILON for value in unsupported_axes):
            self._command, self._reason = BackendCommand.safe_neutral(), "SAFE_COMMAND_UNSUPPORTED_AXIS"
            return
        if linear_x < -EPSILON:
            self._command, self._reason = BackendCommand.safe_neutral(), "SAFE_COMMAND_REVERSE_REJECTED"
            return
        if linear_x > self.policy.command_speed_ceiling_mps + EPSILON:
            self._command, self._reason = BackendCommand.safe_neutral(), "SAFE_COMMAND_SPEED_CEILING"
            return
        result = self._kinematics.compute(linear_x, angular_z)
        if not result.valid or result.direction.value not in {"STATIONARY", "FORWARD"}:
            self._command, self._reason = BackendCommand.safe_neutral(), "SAFE_COMMAND_KINEMATICS_INVALID"
            return
        if abs(result.inner_steering_deg) > self.policy.command_steering_ceiling_deg + EPSILON:
            self._command, self._reason = BackendCommand.safe_neutral(), "SAFE_COMMAND_STEERING_CEILING"
            return
        self._command = (BackendCommand.safe_neutral() if abs(linear_x) <= EPSILON
                         else BackendCommand(Gear.D, result.speed_magnitude_mps, result.inner_steering_deg))
        self._reason = "SAFE_COMMAND_VALID"

    def desired(self, now: float) -> tuple[BackendCommand, str]:
        if self._received is None or now - self._received > self.policy.command_timeout_sec:
            return BackendCommand.safe_neutral(), "SAFE_COMMAND_STALE"
        return self._command, self._reason


def stationary_steering_priming_command(
    command: BackendCommand, feedback, *, steering_tolerance_deg: float = .10,
    zero_speed_tolerance_mps: float = .005,
) -> BackendCommand:
    """Hold D at zero until measured steering has reached a new target.

    This is the physical backend's actuator-transition rule below the Generic
    Gate.  It cannot create motion: only a validated forward request is
    changed, and only its speed is reduced to exactly zero.  The interlock
    separately rejects invalid feedback and any steering mismatch once the
    chassis is moving.
    """
    if command.gear is not Gear.D or command.speed_mps <= EPSILON:
        return command
    measured_steering = getattr(feedback, "steering_deg", None)
    measured_speed = getattr(feedback, "speed_mps", None)
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in (measured_steering, measured_speed)):
        return command
    if (abs(float(measured_speed)) <= zero_speed_tolerance_mps
            and abs(float(measured_steering) - command.steering_deg) > steering_tolerance_deg):
        return BackendCommand(Gear.D, 0.0, command.steering_deg)
    return command


class MkminiPhysicalRosBackend(Node):
    def __init__(self) -> None:
        super().__init__("mkmini_physical_ros_backend")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("physical_enabled", False)
        self.declare_parameter("command_timeout_sec", .25)
        self.declare_parameter("command_speed_ceiling_mps", .040)
        self.declare_parameter("actual_speed_ceiling_mps", .050)
        self.declare_parameter("backend_policy_mode", CLOSED_LOOP_MODE)
        self.declare_parameter("command_steering_ceiling_deg", 30.0)
        self.declare_parameter("safe_command_topic", SAFE_COMMAND_TOPIC)
        self.declare_parameter("state_topic", STATE_TOPIC)
        self.declare_parameter("enable_service", ENABLE_SERVICE)
        self.declare_parameter("native_sender_cpu", 6)
        self._policy = PhysicalCommandPolicy(SafeTwistPolicy(
            command_timeout_sec=float(self.get_parameter("command_timeout_sec").value),
            command_speed_ceiling_mps=float(self.get_parameter("command_speed_ceiling_mps").value),
            command_steering_ceiling_deg=float(
                self.get_parameter("command_steering_ceiling_deg").value),
        ))
        self._backend_policy_mode = str(self.get_parameter("backend_policy_mode").value)
        if self._backend_policy_mode not in {CLOSED_LOOP_MODE, TEMPORARY_OPEN_LOOP_MODE}:
            raise ValueError(f"unsupported backend_policy_mode: {self._backend_policy_mode}")
        self._temporary_open_loop = self._backend_policy_mode == TEMPORARY_OPEN_LOOP_MODE
        self._interface = str(self.get_parameter("can_interface").value)
        self._interlock = MkminiBackendInterlock(BackendInterlockConfig(
            maximum_motion_speed_mps=float(self.get_parameter("actual_speed_ceiling_mps").value)))
        self._can_monitor = LiveCanMonitor(self._interface)
        # This controller never opens a CAN TX socket.  The isolated native
        # child is the sole physical writer and has its own bounded command
        # lease; Python retains interlock and lifecycle authority.
        self._native = NativeSender(
            self._interface, cpu=int(self.get_parameter("native_sender_cpu").value))
        self._command_lock = threading.Lock()
        self._interlock_lock = threading.Lock()
        self._scheduled = BackendCommand.safe_neutral()
        self._requested = BackendCommand.safe_neutral()
        self._supervision_stop = threading.Event()
        self._supervision_thread: threading.Thread | None = None
        self._last_supervision_success: float | None = None
        self._safe_command_stamps = deque(maxlen=1000)
        self._supervision_stamps = deque(maxlen=2000)
        self._native_started_at: float | None = None
        self._last_native_heartbeat_seen: float | None = None
        self._enabled = False
        self._failure: str | None = None
        self._last_scheduler_fault: str | None = None
        self._last_timing: dict = timing_summary([])
        self._last_reason = "DISABLED"
        self._last_state = BackendInterlockState.DISARMED
        self._last_alive_anomaly: dict = {}
        self._last_fault_native_command: dict | None = None
        self._last_fault_feedback: dict | None = None
        self._last_feedback = BackendFeedback(hard_fault=True)
        self._last_feedback_detail: dict = {
            "source": "native_mmap", "valid": False, "reason": "SENDER_NOT_OPEN"}
        # A fixed, zero-speed commissioning sequence.  This is intentionally
        # not an arbitrary steering-command endpoint: it runs through this
        # node's existing interlock and sole native SocketCAN authority.
        self._static_steering_target: float | None = None
        self._static_steering_active = False
        self._static_steering_result = "NOT_RUN"
        self._static_steering_records: list[dict] = []
        self._static_steering_thread: threading.Thread | None = None
        self._can_monitor.open()
        self.create_subscription(TwistStamped, str(self.get_parameter("safe_command_topic").value), self._safe_cb, 20)
        self._state_pub = self.create_publisher(DiagnosticStatus, str(self.get_parameter("state_topic").value), 10)
        self.create_service(SetBool, str(self.get_parameter("enable_service").value), self._enable_cb)
        self.create_service(Trigger, R16_STATIC_STEERING_SERVICE, self._r16_static_steering_cb)
        self.create_service(Trigger, R18_POSITIVE_STEERING_SERVICE,
                            self._r18_positive_steering_cb)
        # ROS only observes the dedicated application scheduler. The
        # supervision loop is deliberately a node-owned monotonic thread so
        # DDS/executor work cannot delay either its lease or CAN TX.
        self.create_timer(.050, self._supervise)
        self.create_timer(.100, self._publish_state)
        if bool(self.get_parameter("physical_enabled").value):
            self._set_enabled(True)

    def _safe_cb(self, msg: TwistStamped) -> None:
        received = time.monotonic()
        with self._command_lock:
            self._safe_command_stamps.append(received)
        twist = msg.twist
        self._policy.accept(linear_x=float(twist.linear.x), angular_z=float(twist.angular.z),
                            unsupported_axes=(float(twist.linear.y), float(twist.linear.z),
                                              float(twist.angular.x), float(twist.angular.y)),
                            now=time.monotonic())

    @staticmethod
    def _can_state(link: dict) -> str:
        info = link.get("linkinfo", {}).get("info_data", {})
        counters = info.get("berr_counter", {})
        xstats = link.get("linkinfo", {}).get("info_xstats", {})
        healthy = (
            info.get("state") == "ERROR-ACTIVE"
            and counters.get("tx") == 0
            and counters.get("rx") == 0
            and all(value == 0 for value in xstats.values())
        )
        return "ERROR-ACTIVE" if healthy else str(
            info.get("state", "UNKNOWN")) + "_WITH_ERRORS"

    def _feedback_snapshot(self, now: float) -> tuple[BackendFeedback, dict]:
        """Consume the isolated native RX snapshot; never decode CAN in ROS."""
        if not self._native.is_open:
            return self._last_feedback, dict(self._last_feedback_detail)
        feedback, detail = self._native.feedback_snapshot(
            max(now, time.monotonic()), self._can_state(self._can_monitor.snapshot()))
        self._last_feedback, self._last_feedback_detail = feedback, detail
        return feedback, detail

    def _set_enabled(self, enabled: bool) -> tuple[bool, str]:
        if enabled == self._enabled:
            return True, "ALREADY_ENABLED" if enabled else "ALREADY_DISABLED"
        if not enabled:
            self._fail_closed("OPERATOR_DISABLE", disarm=True)
            return True, "DISABLED"
        self._failure = None
        self._last_fault_native_command = None
        self._last_fault_feedback = None
        try:
            self._native.open()
        except Exception as exc:
            # Enabling is an operator-controlled boundary.  Transport setup
            # failures must leave the node alive, disabled, and observable so
            # a corrected restart can re-qualify; they must never open a
            # second writer or terminate the ROS process.
            self._failure = f"NATIVE_SENDER_START_FAILED:{type(exc).__name__}:{exc}"
            self._last_reason = self._failure
            self._interlock.disarm()
            self._enabled = False
            return False, self._failure
        if self._temporary_open_loop:
            # Professor-authorized development policy: chassis feedback is
            # diagnostic only. Native starts in N/zero and motion remains
            # bounded by fresh /vehicle_cmd_safe plus native command TTL.
            self._last_state = BackendInterlockState.MOTION_PERMITTED
        # Native construction defines the closed-loop qualification epoch: its first
        # checksum-valid frame establishes each alive baseline, and a second
        # forward sample is required before the ROS interlock can arm.  The
        # sender emits only fail-safe N/zero during this bounded bootstrap.
        if not self._temporary_open_loop:
            deadline = time.monotonic() + .500
            failures: tuple[str, ...] = ("FEEDBACK_ALIVE_BOOTSTRAP_PENDING",)
            while time.monotonic() < deadline:
                feedback, _detail = self._feedback_snapshot(time.monotonic())
                failures = self._interlock.assess_feedback(feedback, require_n_auto=False)
                if not failures:
                    break
                time.sleep(.005)
            if failures:
                self._failure = ";".join(failures)
                self._last_reason = self._failure
                self._native.update(BackendCommand.safe_neutral(), allowed=False)
                self._last_timing = self._native.timing()
                self._native.close()
                self._interlock.disarm()
                self._enabled = False
                return False, self._failure
            self._interlock.arm(time.monotonic())
        self._enabled = True
        self._supervision_stop.clear()
        self._native_started_at = self._last_supervision_success = time.monotonic()
        self._last_native_heartbeat_seen = None
        self._supervision_thread = threading.Thread(target=self._supervision_loop,
                                                    name="mkmini-can-supervision", daemon=True)
        self._supervision_thread.start()
        try:
            # Pin the supervisor independently of the ROS executor thread.
            # The launch reserves CPU 7 for this supervisor; CPU 6 owns
            # native TX/RX and decoding; the ROS executor stays on
            # CPU 5.  No safety-critical CAN callback runs in Python.
            os.sched_setaffinity(self._supervision_thread.native_id, {7})
        except (AttributeError, OSError) as exc:
            self._fail_closed(f"SUPERVISION_CPU_AFFINITY_FAILED:{type(exc).__name__}:{exc}")
            return False, self._failure
        self._last_reason = (TEMPORARY_OPEN_LOOP_MODE if self._temporary_open_loop
                             else "N_ZERO_PRIMING")
        return True, self._last_reason

    def _open_loop_command_is_safe(self, command: BackendCommand) -> bool:
        return all((
            command.gear in {Gear.N, Gear.D},
            math.isfinite(command.speed_mps),
            math.isfinite(command.steering_deg),
            0.0 <= command.speed_mps <= self._policy.policy.command_speed_ceiling_mps,
            abs(command.steering_deg) <= self._policy.policy.command_steering_ceiling_deg,
            command.gear is Gear.D or (
                command.speed_mps == 0.0 and command.steering_deg == 0.0),
        ))

    def _supervision_loop(self) -> None:
        deadline = time.monotonic()
        while not self._supervision_stop.is_set() and self._enabled:
            now = time.monotonic()
            with self._command_lock:
                self._supervision_stamps.append(now)
            feedback, feedback_detail = self._feedback_snapshot(now)
            with self._interlock_lock:
                # Do not sort or scan the TX history in this 100 Hz loop.
                # Percentiles are computed only by the slower diagnostics timer.
                native_timing = self._native.watchdog_timing()
                if not self._native.is_open:
                    evidence = self._native.exit_reason()
                    self._failure = "NATIVE_SENDER_EXITED" + (f":{evidence}" if evidence else "")
                    return
                if (native_timing["max_interval_sec"] is not None
                      and native_timing["max_interval_sec"] > .030):
                    self._failure = "NATIVE_HEARTBEAT_GAP"
                    self._native.update(BackendCommand.safe_neutral(), allowed=False)
                    return
                command, command_reason = self._policy.desired(now)
                with self._command_lock:
                    static_target = self._static_steering_target if self._static_steering_active else None
                if static_target is not None:
                    # The fixed rig test may move steering only.  It can
                    # never request traction speed or reverse.
                    command = BackendCommand(Gear.D, 0.0, static_target)
                    command_reason = "R16_STATIC_STEERING"
                with self._command_lock:
                    self._requested = command
                if self._temporary_open_loop:
                    if not self._open_loop_command_is_safe(command):
                        self._failure = "OPEN_LOOP_COMMAND_ENVELOPE_REJECTED"
                        self._native.update(BackendCommand.safe_neutral(), allowed=False)
                        return
                    with self._command_lock:
                        self._scheduled = command
                    self._last_state = BackendInterlockState.MOTION_PERMITTED
                    self._last_reason = (TEMPORARY_OPEN_LOOP_MODE + ":" + command_reason)
                    self._native.update(command, allowed=True)
                    self._last_supervision_success = now
                    # Feedback remains logged above, but cannot revoke motion.
                    deadline += .010
                    delay = deadline - time.monotonic()
                    if delay > 0:
                        self._supervision_stop.wait(delay)
                    else:
                        deadline = time.monotonic()
                    continue
                # Do not apply a nonzero forward speed until the chassis has
                # acknowledged a newly requested steering angle.  The D/zero
                # priming command is explicitly permitted by the qualified
                # in-motion interlock while actual speed remains zero.
                command = stationary_steering_priming_command(command, feedback)
                # A delayed Python supervisor tick is not evidence of a CAN
                # heartbeat gap: the native sender independently times every
                # physical TX and exits (after sending N/zero) if its real
                # interval exceeds 30 ms. Refresh the interlock's witness
                # only after checking that native authority is still alive
                # and its measured CAN interval remains within that bound.
                # Do this immediately before step(); doing it afterward made
                # a harmless >30 ms supervisor scheduling pause look like a
                # missing chassis heartbeat even while native TX was healthy.
                now = time.monotonic()
                self._interlock.note_native_sender_healthy(now)
                result = self._interlock.step(now, command, feedback, heartbeat_emitted=False)
                self._last_state, self._last_reason = result.state, ";".join(result.reasons)
                if result.state is BackendInterlockState.MOTION_NOT_PERMITTED:
                    self._failure = self._last_reason
                    self._last_fault_native_command = self._native.transmitted_command()
                    self._last_fault_feedback = {
                        "ctrl_age_sec": feedback.ctrl_age_sec,
                        "gear": None if feedback.gear is None else int(feedback.gear),
                        "speed_mps": feedback.speed_mps,
                        "steering_deg": feedback.steering_deg,
                    }
                    self._last_alive_anomaly = (dict(feedback_detail)
                                                if (not feedback.ctrl_alive_contiguous
                                                    or not feedback.diagnostic_alive_contiguous)
                                                else {})
                    with self._command_lock:
                        self._scheduled = BackendCommand.safe_neutral()
                    self._native.update(BackendCommand.safe_neutral(), allowed=False)
                    return
                with self._command_lock:
                    self._scheduled = result.output if result.command_admitted else BackendCommand.safe_neutral()
                    scheduled = self._scheduled
                self._native.update(scheduled, allowed=True)
                self._last_supervision_success = now
            deadline += .010
            delay = deadline - time.monotonic()
            if delay > 0:
                self._supervision_stop.wait(delay)
            else:
                deadline = time.monotonic()

    def _enable_cb(self, request, response):
        response.success, response.message = self._set_enabled(bool(request.data))
        return response

    def _r16_static_steering_cb(self, _request, response):
        feedback, _detail = self._feedback_snapshot(time.monotonic())
        command, _reason = self._policy.desired(time.monotonic())
        refusal = None
        if not self._enabled or self._failure is not None:
            refusal = "BACKEND_NOT_HEALTHY"
        elif (not self._temporary_open_loop
              and self._last_state is not BackendInterlockState.MOTION_PERMITTED):
            refusal = "INTERLOCK_NOT_MOTION_PERMITTED"
        elif self._static_steering_active:
            refusal = "STATIC_STEERING_ALREADY_ACTIVE"
        elif command != BackendCommand.safe_neutral():
            refusal = "NONZERO_SAFE_COMMAND_PRESENT"
        elif (not self._temporary_open_loop
              and (feedback.speed_mps is None or abs(float(feedback.speed_mps)) > .005)):
            refusal = "CHASSIS_NOT_STATIONARY"
        if refusal is not None:
            response.success, response.message = False, refusal
            return response
        with self._command_lock:
            self._static_steering_records = []
            self._static_steering_result = "RUNNING"
            self._static_steering_active = True
            self._static_steering_target = 0.0
        self._static_steering_thread = threading.Thread(
            target=self._run_r16_static_steering,
            name="r16-static-steering", daemon=True)
        self._static_steering_thread.start()
        response.success, response.message = True, "R16_STATIC_STEERING_STARTED"
        return response

    def _r18_positive_steering_cb(self, _request, response):
        command, _reason = self._policy.desired(time.monotonic())
        refusal = None
        if not self._temporary_open_loop:
            refusal = "R18_SERVICE_REQUIRES_TEMPORARY_OPEN_LOOP"
        elif not self._enabled or self._failure is not None:
            refusal = "BACKEND_NOT_HEALTHY"
        elif self._static_steering_active:
            refusal = "STATIC_STEERING_ALREADY_ACTIVE"
        elif command != BackendCommand.safe_neutral():
            refusal = "NONZERO_SAFE_COMMAND_PRESENT"
        if refusal is not None:
            response.success, response.message = False, refusal
            return response
        with self._command_lock:
            self._static_steering_records = []
            self._static_steering_result = "R18_POSITIVE_RUNNING"
            self._static_steering_active = True
            self._static_steering_target = 5.0
        self._static_steering_thread = threading.Thread(
            target=self._run_r18_positive_steering,
            name="r18-positive-steering", daemon=True)
        self._static_steering_thread.start()
        response.success, response.message = True, "R18_POSITIVE_5_DEG_STARTED"
        return response

    def _run_r18_positive_steering(self) -> None:
        try:
            # Hold long enough for an unambiguous operator observation, then
            # automatically return to center before releasing the override.
            if self._supervision_stop.wait(3.0):
                raise RuntimeError("BACKEND_STOPPED_DURING_R18_POSITIVE")
            with self._command_lock:
                self._static_steering_target = 0.0
            if self._supervision_stop.wait(1.0):
                raise RuntimeError("BACKEND_STOPPED_DURING_R18_CENTER")
            with self._command_lock:
                self._static_steering_result = "R18_POSITIVE_PASS"
        except Exception as exc:
            with self._command_lock:
                self._static_steering_result = f"FAIL:{exc}"
            self._fail_closed(f"R18_{exc}")
        finally:
            with self._command_lock:
                self._static_steering_target = None
                self._static_steering_active = False

    def _run_r16_static_steering(self) -> None:
        # Centre between opposite signs to make sign and settling behaviour
        # unambiguous.  The final target is always centre.
        targets = ((0.0, 5.0, 0.0, -5.0, 0.0) if self._temporary_open_loop else
                   (0.0, 5.0, 0.0, -5.0, 0.0, 10.0, 0.0, -10.0,
                    0.0, 15.0, 0.0, -15.0, 0.0))
        try:
            for target in targets:
                started = time.monotonic()
                with self._command_lock:
                    self._static_steering_target = target
                stable_since = None
                extrema: list[float] = []
                native_sample = None
                last_feedback = None
                while time.monotonic() - started <= 2.5:
                    now = time.monotonic()
                    if not self._enabled or self._failure is not None:
                        raise RuntimeError(self._failure or "BACKEND_DISABLED")
                    feedback, _detail = self._feedback_snapshot(now)
                    if (not self._temporary_open_loop and
                            (feedback.speed_mps is None or abs(float(feedback.speed_mps)) > .005)):
                        raise RuntimeError("STATIC_STEERING_NONZERO_SPEED")
                    if (not self._temporary_open_loop and
                            (feedback.steering_deg is None or not math.isfinite(float(feedback.steering_deg)))):
                        raise RuntimeError("STATIC_STEERING_FEEDBACK_INVALID")
                    value = (None if feedback.steering_deg is None else float(feedback.steering_deg))
                    if value is not None and math.isfinite(value):
                        extrema.append(value)
                        last_feedback = value
                    native_sample = self._native.transmitted_command()
                    native_target_ok = (native_sample is not None
                                        and native_sample.get("gear") == int(Gear.D)
                                        and native_sample.get("speed_raw_mmps") == 0
                                        and abs(float(native_sample.get("steering_deg", math.inf)) - target) <= .011)
                    if self._temporary_open_loop and native_target_ok:
                        stable_since = stable_since or now
                        if now - stable_since >= .70:
                            break
                    elif native_target_ok and value is not None and abs(value - target) <= .50:
                        stable_since = stable_since or now
                        if now - stable_since >= .20:
                            break
                    else:
                        stable_since = None
                    time.sleep(.01)
                required_stable = .70 if self._temporary_open_loop else .20
                settled = stable_since is not None and time.monotonic() - stable_since >= required_stable
                record = {
                    "target_deg": target,
                    "native_tx": native_sample,
                    "feedback_deg": last_feedback,
                    "settled": settled,
                    "settling_sec": time.monotonic() - started,
                    "feedback_min_deg": min(extrema) if extrema else None,
                    "feedback_max_deg": max(extrema) if extrema else None,
                }
                with self._command_lock:
                    self._static_steering_records.append(record)
                if not settled:
                    raise RuntimeError(f"STATIC_STEERING_NOT_SETTLED:{target:+.1f}")
            with self._command_lock:
                self._static_steering_result = "PASS"
        except Exception as exc:
            with self._command_lock:
                self._static_steering_result = f"FAIL:{exc}"
            self._fail_closed(f"R16_{exc}")
        finally:
            with self._command_lock:
                self._static_steering_target = None
                self._static_steering_active = False

    def _fail_closed(self, reason: str, *, disarm: bool = False) -> None:
        self._failure = reason
        with self._command_lock:
            self._scheduled = BackendCommand.safe_neutral()
            self._static_steering_target = None
            self._static_steering_active = False
        self._supervision_stop.set()
        if self._supervision_thread is not None and self._supervision_thread is not threading.current_thread():
            self._supervision_thread.join(timeout=.25)
        self._supervision_thread = None
        if self._native.is_open:
            try:
                self._native.update(BackendCommand.safe_neutral(), allowed=False)
            finally:
                self._last_timing = self._native.timing()
                self._native.close()
        if disarm:
            self._interlock.disarm()
        self._enabled = False
        self._last_reason = reason

    def _supervise(self) -> None:
        if not self._enabled:
            return
        if self._failure is not None or not self._native.is_open:
            evidence = self._native.exit_reason() if not self._native.is_open else ""
            self._fail_closed(self._failure or ("NATIVE_SENDER_EXITED" + (f":{evidence}" if evidence else "")))

    def _publish_state(self) -> None:
        diag = DiagnosticStatus()
        diag.name = "mkmini_cmd_adapter/physical_backend"
        diag.hardware_id = "mkmini_physical_can"
        diag.level = DiagnosticStatus.OK if self._enabled and self._last_state is BackendInterlockState.MOTION_PERMITTED else DiagnosticStatus.WARN
        diag.message = self._last_reason
        timing = self._native.timing() if self._native.is_open else self._last_timing
        now_monotonic = time.monotonic()
        native_status_stamp = self._native.latest_send()
        native_status_age = (None if native_status_stamp is None
                             else max(0.0, now_monotonic - native_status_stamp))
        native_count = self._native.watchdog_timing()["count"]
        native_command = self._native.transmitted_command()
        with self._interlock_lock:
            interlock_stamp = self._interlock._last_heartbeat
        interlock_age = (None if interlock_stamp is None
                         else max(0.0, now_monotonic - interlock_stamp))
        feedback, feedback_detail = self._feedback_snapshot(time.monotonic())
        with self._command_lock:
            safe_command_stamps = list(self._safe_command_stamps)
            supervision_stamps = list(self._supervision_stamps)
            requested = self._requested
            scheduled = self._scheduled
            static_steering_active = self._static_steering_active
            static_steering_target = self._static_steering_target
            static_steering_result = self._static_steering_result
            static_steering_records = list(self._static_steering_records)
        def rate_and_max_gap(stamps: list[float]) -> tuple[str, str]:
            if len(stamps) < 2:
                return "0.0", "None"
            gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
            duration = stamps[-1] - stamps[0]
            return str((len(stamps) - 1) / duration if duration > 0 else 0.0), str(max(gaps))
        safe_rate, safe_gap = rate_and_max_gap(safe_command_stamps)
        supervisor_rate, supervisor_gap = rate_and_max_gap(supervision_stamps)
        sender_rate = "0.0" if timing["mean_interval_sec"] in (None, 0) else str(1.0 / timing["mean_interval_sec"])
        sender_gap = str(timing["max_interval_sec"])
        diag.values = [KeyValue(key="input_topic", value=SAFE_COMMAND_TOPIC),
                       KeyValue(key="backend_policy_mode", value=self._backend_policy_mode),
                       KeyValue(key="feedback_motion_authority", value=str(not self._temporary_open_loop).lower()),
                       KeyValue(key="interlock_state", value=self._last_state.value),
                       KeyValue(key="physical_enabled", value=str(self._enabled).lower()),
                       KeyValue(key="socketcan_open", value=str(self._native.is_open).lower()),
                       KeyValue(key="heartbeat_max_interval_sec", value=str(timing["max_interval_sec"])),
                       KeyValue(key="heartbeat_mean_interval_sec", value=str(timing["mean_interval_sec"])),
                       KeyValue(key="heartbeat_p99_interval_sec", value=str(timing["p99_interval_sec"])),
                       KeyValue(key="heartbeat_scheduler_fault", value=self._last_scheduler_fault or ""),
                       KeyValue(key="ctrl_feedback_age_sec", value=str(feedback.ctrl_age_sec)),
                       KeyValue(key="diagnostic_feedback_age_sec", value=str(feedback.diagnostic_age_sec)),
                       KeyValue(key="feedback_source", value=str(feedback_detail.get("source"))),
                       KeyValue(key="native_ctrl_rx_count", value=str(feedback_detail.get("ctrl_count"))),
                       KeyValue(key="native_diagnostic_rx_count", value=str(feedback_detail.get("diagnostic_count"))),
                       KeyValue(key="native_ctrl_alive_delta", value=str(feedback_detail.get("ctrl_delta"))),
                       KeyValue(key="native_diagnostic_alive_delta", value=str(feedback_detail.get("diagnostic_delta"))),
                       KeyValue(key="actual_speed_mps", value=str(feedback.speed_mps)),
                       KeyValue(key="actual_steering_deg", value=str(feedback.steering_deg)),
                       KeyValue(key="requested_gear", value=str(int(requested.gear))),
                       KeyValue(key="requested_speed_mps", value=str(requested.speed_mps)),
                       KeyValue(key="requested_steering_deg", value=str(requested.steering_deg)),
                       KeyValue(key="scheduled_gear", value=str(int(scheduled.gear))),
                       KeyValue(key="scheduled_speed_mps", value=str(scheduled.speed_mps)),
                       KeyValue(key="scheduled_steering_deg", value=str(scheduled.steering_deg)),
                       KeyValue(key="can_state", value=feedback.can_state),
                       KeyValue(key="safe_command_rate_hz", value=safe_rate),
                       KeyValue(key="safe_command_max_gap_sec", value=safe_gap),
                       KeyValue(key="supervision_rate_hz", value=supervisor_rate),
                       KeyValue(key="supervision_max_gap_sec", value=supervisor_gap),
                       KeyValue(key="heartbeat_sender_rate_hz", value=sender_rate),
                       KeyValue(key="heartbeat_sender_max_gap_sec", value=sender_gap),
                       KeyValue(key="native_tx_status_age_sec", value=str(native_status_age)),
                       KeyValue(key="native_tx_status_count", value=str(native_count)),
                       KeyValue(key="native_tx_gear", value=str(None if native_command is None else native_command["gear"])),
                       KeyValue(key="native_tx_speed_raw_mmps", value=str(None if native_command is None else native_command["speed_raw_mmps"])),
                       KeyValue(key="native_tx_steering_raw_cdeg", value=str(None if native_command is None else native_command["steering_raw_cdeg"])),
                       KeyValue(key="native_tx_steering_deg", value=str(None if native_command is None else native_command["steering_deg"])),
                       KeyValue(key="interlock_heartbeat_age_sec", value=str(interlock_age)),
                       KeyValue(key="last_alive_anomaly", value=str(self._last_alive_anomaly)),
                       KeyValue(key="last_fault_native_command", value=str(self._last_fault_native_command)),
                       KeyValue(key="last_fault_feedback", value=str(self._last_fault_feedback)),
                       KeyValue(key="r16_static_steering_active", value=str(static_steering_active).lower()),
                       KeyValue(key="r16_static_steering_target_deg", value=str(static_steering_target)),
                       KeyValue(key="r16_static_steering_result", value=static_steering_result),
                       KeyValue(key="r16_static_steering_records_json", value=json.dumps(static_steering_records)),
                       KeyValue(key="failure", value=self._failure or "")]
        self._state_pub.publish(diag)

    def destroy_node(self):
        self._fail_closed("NODE_SHUTDOWN", disarm=True)
        self._can_monitor.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required for the physical ROS backend")
    rclpy.init(args=args)
    node = MkminiPhysicalRosBackend()
    try:
        # Native TX/RX owns CPU 6, supervision is pinned to 7, and the ROS
        # executor remains on CPU 5.
        try:
            os.sched_setaffinity(0, {5})
        except (AttributeError, OSError):
            pass
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["PhysicalCommandPolicy", "SafeTwistPolicy", "MkminiPhysicalRosBackend"]
