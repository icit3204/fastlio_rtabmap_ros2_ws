import pytest
import mmap

from mkmini_cmd_adapter import BackendCommand, BackendFeedback, Gear
from mkmini_cmd_adapter.physical_ros_backend import (
    CLOSED_LOOP_MODE, TEMPORARY_OPEN_LOOP_MODE, PhysicalCommandPolicy,
    SafeTwistPolicy, stationary_steering_priming_command,
)


def accept(policy, linear_x, angular_z=0.0, unsupported=(0.0, 0.0, 0.0, 0.0), now=1.0):
    policy.accept(linear_x=linear_x, angular_z=angular_z, unsupported_axes=unsupported, now=now)
    return policy.desired(now)


def test_only_safe_twist_policy_admits_bounded_forward_ackermann_command():
    policy = PhysicalCommandPolicy(SafeTwistPolicy(command_timeout_sec=.25, command_speed_ceiling_mps=.04))
    command, reason = accept(policy, .04, .01)
    assert reason == "SAFE_COMMAND_VALID"
    assert command.gear is Gear.D and command.speed_mps == pytest.approx(.04)


@pytest.mark.parametrize("linear,unsupported,reason", [
    (-.001, (0., 0., 0., 0.), "SAFE_COMMAND_REVERSE_REJECTED"),
    (.0401, (0., 0., 0., 0.), "SAFE_COMMAND_SPEED_CEILING"),
    (.01, (.1, 0., 0., 0.), "SAFE_COMMAND_UNSUPPORTED_AXIS"),
])
def test_directly_unsafe_or_out_of_envelope_twists_become_neutral(linear, unsupported, reason):
    policy = PhysicalCommandPolicy()
    command, observed = accept(policy, linear, unsupported=unsupported)
    assert command == command.safe_neutral()
    assert observed == reason


def test_stale_safe_command_is_neutral():
    policy = PhysicalCommandPolicy(SafeTwistPolicy(command_timeout_sec=.25))
    accept(policy, .04, now=1.0)
    command, reason = policy.desired(1.251)
    assert command == command.safe_neutral()
    assert reason == "SAFE_COMMAND_STALE"


def test_temporary_open_loop_command_policy_keeps_speed_and_steering_bounds():
    policy = PhysicalCommandPolicy(SafeTwistPolicy(
        command_speed_ceiling_mps=.04, command_steering_ceiling_deg=30.0))
    command, reason = accept(policy, .04, .020)
    assert reason == "SAFE_COMMAND_VALID"
    assert command.speed_mps == pytest.approx(.04)
    assert 0.0 < command.steering_deg <= 30.0
    command, reason = accept(policy, .04, .031)
    assert command == BackendCommand.safe_neutral()
    assert reason == "SAFE_COMMAND_STEERING_CEILING"


def test_nonzero_forward_waits_at_d_zero_for_stationary_steering_feedback():
    requested = BackendCommand(Gear.D, .04, 5.0)
    waiting = stationary_steering_priming_command(
        requested, BackendFeedback(speed_mps=0.0, steering_deg=0.0))
    assert waiting == BackendCommand(Gear.D, 0.0, 5.0)
    reached = stationary_steering_priming_command(
        requested, BackendFeedback(speed_mps=0.0, steering_deg=5.0))
    assert reached == requested
    # Once moving, the backend never masks a steering disagreement.
    moving = stationary_steering_priming_command(
        requested, BackendFeedback(speed_mps=.01, steering_deg=0.0))
    assert moving == requested


def test_physical_ros_backend_has_only_gate_input_and_no_cmd_vel_subscription():
    source = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/physical_ros_backend.py", encoding="utf-8").read()
    assert 'SAFE_COMMAND_TOPIC = "/vehicle_cmd_safe"' in source
    assert "create_subscription(TwistStamped, str(self.get_parameter(\"safe_command_topic\").value)" in source
    assert "NativeSender" in source
    assert "MkminiBackendInterlock" in source
    assert "self._native.feedback_snapshot" in source
    assert "LiveFeedbackMonitor" not in source
    assert "self._monitor.snapshot" not in source
    assert "while time.monotonic() < deadline" in source
    assert "SocketCanTxTransport" not in source
    assert "self._native.update(BackendCommand.safe_neutral(), allowed=False)" in source
    assert 'R16_STATIC_STEERING_SERVICE = "/mkmini_physical_backend/r16_static_steering"' in source
    assert 'R18_POSITIVE_STEERING_SERVICE = "/mkmini_physical_backend/r18_positive_steering"' in source
    assert "R18_POSITIVE_5_DEG_STARTED" in source
    assert "self._static_steering_target = 5.0" in source
    assert "BackendCommand(Gear.D, 0.0, static_target)" in source
    assert "STATIC_STEERING_NONZERO_SPEED" in source
    assert "NATIVE_SENDER_START_FAILED" in source
    assert "os.sched_setaffinity(self._supervision_thread.native_id, {7})" in source
    assert "deque(maxlen=2000)" in source
    native_gap_check = source.index('native_timing["max_interval_sec"] > .030')
    native_health_witness = source.index("self._interlock.note_native_sender_healthy(now)")
    interlock_evaluation = source.index("self._interlock.step(now, command, feedback, heartbeat_emitted=False)")
    assert native_gap_check < native_health_witness < interlock_evaluation
    native_source = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/native_sender.py", encoding="utf-8").read()
    assert "self._first_status.wait(timeout=1.0)" in native_source
    assert "native sender produced no initial heartbeat status" in native_source


def test_explicit_temporary_open_loop_preserves_closed_loop_code_and_feedback_is_diagnostic():
    source = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/physical_ros_backend.py", encoding="utf-8").read()
    config = open("src/mkmini_cmd_adapter/config/r11_physical_backend_commissioning.yaml", encoding="utf-8").read()
    assert TEMPORARY_OPEN_LOOP_MODE == "TEMPORARY_MKMINI_OPEN_LOOP"
    assert CLOSED_LOOP_MODE == "FEEDBACK_INTERLOCKED"
    assert "backend_policy_mode: TEMPORARY_MKMINI_OPEN_LOOP" in config
    assert "if self._temporary_open_loop:" in source
    assert "Feedback remains logged above, but cannot revoke motion" in source
    assert "if not self._temporary_open_loop:" in source
    assert "self._interlock.step" in source
    assert "self._native.feedback_snapshot" in source
    assert "OPEN_LOOP_COMMAND_ENVELOPE_REJECTED" in source
    assert "self._native.update(command, allowed=True)" in source
    assert "command_steering_ceiling_deg: 30.0" in config


def test_final_robot_feedback_restore_requirement_is_documented():
    text = open("src/mkmini_cmd_adapter/docs/TEMPORARY_MKMINI_OPEN_LOOP.md", encoding="utf-8").read()
    assert "FINAL_ROBOT_REQUIREMENT" in text
    assert "restore `FEEDBACK_INTERLOCKED`" in text


def test_native_sender_restart_paths_are_nonce_scoped_and_cleaned_before_bind():
    source = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/native_sender.py", encoding="utf-8").read()
    assert "uuid.uuid4().hex" in source
    assert "self._client_path.unlink(missing_ok=True)" in source
    assert "self._server_path.unlink(missing_ok=True)" in source


def test_native_sender_exposes_constant_time_watchdog_snapshot():
    from mkmini_cmd_adapter.native_sender import NativeSender

    sender = NativeSender("can-test")
    sender._native_count = 42
    sender._native_max = .012
    assert sender.watchdog_timing() == {"count": 42, "max_interval_sec": .012}
    backend = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/physical_ros_backend.py", encoding="utf-8").read()
    assert "native_timing = self._native.watchdog_timing()" in backend
    assert "note_native_sender_healthy(now)" in backend
    assert "self.create_timer(.100, self._publish_state)" in backend


def test_native_writer_reports_the_exact_transmitted_command_domain():
    native = open("src/mkmini_native_sender/src/mkmini_native_sender.cpp", encoding="utf-8").read()
    sender = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/native_sender.py", encoding="utf-8").read()
    backend = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/physical_ros_backend.py", encoding="utf-8").read()
    assert '"STAT %.9f %.9f %llu %.9f %.9f %d %d %d %d"' in native
    assert '"steering_raw_cdeg": steering_raw' in sender
    assert "def transmitted_command" in sender
    assert 'KeyValue(key="last_fault_native_command"' in backend


def test_physical_backend_uses_one_native_cpu_for_tx_rx_and_no_python_can_rx():
    source = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/physical_ros_backend.py", encoding="utf-8").read()
    native = open("src/mkmini_native_sender/src/mkmini_native_sender.cpp", encoding="utf-8").read()
    assert 'self.declare_parameter("native_sender_cpu", 6)' in source
    assert "LiveFeedbackMonitor" not in source
    assert "receive_feedback(fd, shared, ctrl_alive, diag_alive)" in native
    assert "can_filter filters[2]" in native
    assert "kCtrlFbId | CAN_EFF_FLAG" in native
    assert "kDiagFbId | CAN_EFF_FLAG" in native
    assert 'cpu=int(self.get_parameter("native_sender_cpu").value)' in source


def test_native_feedback_mmap_decodes_fresh_snapshot_without_python_can_queue():
    from mkmini_cmd_adapter.native_sender import (
        FEEDBACK_ABI, FEEDBACK_MAGIC, FEEDBACK_VERSION, NativeSender,
    )

    sender = NativeSender("can-test")
    mapped = mmap.mmap(-1, FEEDBACK_ABI.size)
    # magic/version/sequence, timestamps/counts, 14 ints, two intervals
    mapped[:] = FEEDBACK_ABI.pack(
        FEEDBACK_MAGIC, FEEDBACK_VERSION, 8,
        10.000, 10.005, 42, 21,
        int(Gear.N), 0, -503, 1, 7,
        1, 0, 0, 1, 1, 1, 1, 2, 1,
        .010, .020,
    )
    sender._feedback_mmap = mapped
    feedback, detail = sender.feedback_snapshot(10.020, "ERROR-ACTIVE")
    assert feedback.ctrl_age_sec == pytest.approx(.020)
    assert feedback.diagnostic_age_sec == pytest.approx(.015)
    assert feedback.gear is Gear.N
    assert feedback.speed_mps == 0.0
    assert feedback.steering_deg == pytest.approx(-5.03)
    assert feedback.ctrl_alive_contiguous is True
    assert feedback.diagnostic_alive_contiguous is True
    assert feedback.hard_fault is False
    assert detail["source"] == "native_mmap"
    assert detail["ctrl_count"] == 42
    sender._feedback_mmap = None
    mapped.close()


def test_native_sender_exit_evidence_reaches_the_fail_closed_backend():
    source = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/native_sender.py", encoding="utf-8").read()
    backend = open("src/mkmini_cmd_adapter/mkmini_cmd_adapter/physical_ros_backend.py", encoding="utf-8").read()
    assert "def exit_reason" in source
    assert "self._native.exit_reason()" in backend


def test_native_writer_has_monotonic_timer_and_bounded_motion_lease():
    source = open("src/mkmini_native_sender/src/mkmini_native_sender.cpp", encoding="utf-8").read()
    assert "timerfd_create(CLOCK_MONOTONIC" in source
    assert "kCommandTtlSec = .050" in source
    assert "mono() - command.received > kCommandTtlSec" in source
    assert 'std::string(buffer, length) == "STOP"' in source
    assert "send_can(fd, Command{}, alive)" in source


def test_native_rx_preserves_50ms_freshness_checksum_alive_and_hard_fault_checks():
    source = open("src/mkmini_native_sender/src/mkmini_native_sender.cpp", encoding="utf-8").read()
    assert "kFeedbackFreshnessSec = .050" in source
    assert "frame.data[7] == checksum(frame.data)" in source
    assert "delta == 0 || delta > 5" in source
    assert "ctrl_count >= 2 and ctrl_alive_valid" in open(
        "src/mkmini_cmd_adapter/mkmini_cmd_adapter/native_sender.py", encoding="utf-8").read()
    assert "eps != 0 || left_drive != 0 || right_drive != 0 || bms_loss || estop" in source
