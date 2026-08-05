import math

from vehicle_cmd_safety.gate_core import (
    AuthoritySnapshot,
    GateConfig,
    GateCore,
    STATE_ARMED,
    STATE_DISARMED,
    STATE_FAULT,
    Twist6,
)


def configured_gate() -> GateCore:
    return GateCore(
        GateConfig(
            max_forward_velocity=0.20,
            max_angular_velocity=0.50,
            max_linear_increase_rate=0.50,
            max_angular_increase_rate=1.00,
        )
    )


def make_ready(core: GateCore, now: float) -> None:
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), now)
    core.set_safe_command(Twist6(linear_x=0.10), now + 1.1)
    core.set_permission("localization", True, now + 1.2)
    core.set_permission("controller", True, now + 1.2)
    core.set_permission("collision", True, now + 1.2)


def test_defaults_deny_motion_and_explicit_limits_are_required():
    core = GateCore(GateConfig())
    make_ready(core, 10.0)
    ok, reason = core.request_arm(True, 10.0)
    assert not ok
    assert reason == "MISSING_EXPLICIT_LINEAR_LIMIT"


def test_startup_disarmed_repeated_zero_and_diagnostics():
    core = configured_gate()
    status = core.tick(1.0)
    assert status.state == STATE_DISARMED
    assert status.output.is_zero()
    assert status.diagnostics["mode"] == "MOCK"
    assert status.diagnostics["state"] == STATE_DISARMED
    assert status.diagnostics["configured_frame"] == "base_footprint"
    later = core.tick(1.5)
    assert later.state == STATE_DISARMED
    assert later.output.is_zero()
    assert later.diagnostics["reason_code"] == "DISARMED_ZERO"


def test_arm_requires_fresh_prerequisites_and_stable_authority():
    core = configured_gate()
    now = 10.0
    core.set_safe_command(Twist6(linear_x=0.10), now)
    core.set_permission("localization", True, now)
    core.set_permission("controller", True, now)
    core.set_permission("collision", True, now)
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), now)
    ok, reason = core.request_arm(True, now + 0.5)
    assert not ok
    assert reason == "AUTHORITY_NOT_STABLE"
    core.set_safe_command(Twist6(linear_x=0.10), now + 0.8)
    core.set_permission("localization", True, now + 0.8)
    core.set_permission("controller", True, now + 0.8)
    core.set_permission("collision", True, now + 0.8)
    ok, reason = core.request_arm(True, now + 1.1)
    assert ok
    assert reason == "ARMED"
    assert core.state == STATE_ARMED


def test_output_stamp_frame_fields_are_core_supported_by_zeroing_unsupported_axes():
    core = configured_gate()
    make_ready(core, 1.0)
    ok, _ = core.request_arm(True, 2.2)
    assert ok
    status = core.tick(2.25)
    assert status.output.linear_y == 0.0
    assert status.output.linear_z == 0.0
    assert status.output.angular_x == 0.0
    assert status.output.angular_y == 0.0


def test_velocity_clamping_and_increase_only_slew_then_immediate_reduction():
    core = configured_gate()
    make_ready(core, 1.0)
    assert core.request_arm(True, 2.2)[0]
    core.set_safe_command(Twist6(linear_x=0.30, angular_z=0.70), 2.21)
    first = core.tick(2.30).output
    assert first.linear_x <= 0.05 + 1e-9
    assert first.angular_z <= 0.10 + 1e-9
    later = core.tick(2.40).output
    assert later.linear_x <= 0.10 + 1e-9
    assert later.angular_z <= 0.20 + 1e-9
    core.set_safe_command(Twist6(linear_x=0.02, angular_z=0.01), 2.41)
    reduced = core.tick(2.42).output
    assert reduced.linear_x == 0.02
    assert reduced.angular_z == 0.01


def test_upstream_zero_and_disarm_zero_are_immediate():
    core = configured_gate()
    make_ready(core, 1.0)
    assert core.request_arm(True, 2.2)[0]
    core.tick(2.25)
    core.set_safe_command(Twist6(), 2.26)
    assert core.tick(2.27).output.is_zero()
    ok, reason = core.request_arm(False, 2.28)
    assert ok
    assert reason in {"DISARMED", "DISARMED_FAULT_CLEARED"}
    assert core.tick(2.29).output.is_zero()


def assert_fault_for(command: Twist6, reason: str) -> None:
    core = configured_gate()
    make_ready(core, 1.0)
    assert core.request_arm(True, 2.2)[0]
    core.set_safe_command(command, 2.21)
    status = core.tick(2.22)
    assert status.state == STATE_FAULT
    assert status.reason_code == reason
    assert status.output.is_zero()


def test_nonfinite_unsupported_reverse_and_in_place_faults_latch():
    assert_fault_for(Twist6(linear_x=math.nan), "NONFINITE_COMMAND")
    assert_fault_for(Twist6(linear_x=math.inf), "NONFINITE_COMMAND")
    assert_fault_for(Twist6(linear_x=0.1, linear_y=1e-4), "UNSUPPORTED_AXIS")
    assert_fault_for(Twist6(linear_x=-1e-4), "REVERSE_COMMAND")
    assert_fault_for(Twist6(linear_x=0.0, angular_z=0.10), "IN_PLACE_ROTATION")


def test_stale_safe_and_permissions_fault_while_armed():
    cases = [
        ("safe", 2.6, "SAFE_TWIST_STALE"),
        ("localization", 2.6, "LOCALIZATION_PERMISSION_INVALID"),
        ("controller", 2.6, "CONTROLLER_PERMISSION_INVALID"),
        ("collision", 2.6, "COLLISION_MONITOR_VALID_INVALID"),
    ]
    for target, tick_time, reason in cases:
        core = configured_gate()
        make_ready(core, 1.0)
        assert core.request_arm(True, 2.2)[0]
        if target == "safe":
            status = core.tick(tick_time)
        else:
            core.set_safe_command(Twist6(linear_x=0.1), tick_time - 0.01)
            if target == "localization":
                core.set_permission("localization", False, tick_time)
            elif target == "controller":
                core.set_permission("controller", False, tick_time)
            else:
                core.set_permission("collision", False, tick_time)
            status = core.tick(tick_time)
        assert status.state == STATE_FAULT
        assert status.reason_code == reason
        assert status.output.is_zero()


def test_monotonic_permission_age_faults_despite_frozen_ros_stamp_context():
    core = configured_gate()
    now = 10.0
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), now)
    core.set_safe_command(Twist6(linear_x=0.10), now + 1.0)
    core.set_permission("localization", True, now + 1.0)
    core.set_permission("controller", True, now + 1.0)
    core.set_permission("collision", True, now + 1.0)
    assert core.request_arm(True, now + 1.1)[0]

    frozen_ros_stamp = 123.456
    core.set_safe_command(Twist6(linear_x=0.10), now + 1.55)
    core.set_permission("localization", True, now + 1.55)
    core.set_permission("controller", True, now + 1.55)
    fault = core.tick(now + 1.61)

    assert frozen_ros_stamp == 123.456
    assert fault.state == STATE_FAULT
    assert fault.reason_code == "COLLISION_MONITOR_VALID_INVALID"
    assert fault.output.is_zero()
    assert fault.diagnostics["collision_monitor_valid_age_sec"] == "0.610000"

    later = core.tick(now + 2.0)
    assert later.state == STATE_FAULT
    assert later.output.is_zero()
    ok, reason = core.request_arm(True, now + 2.01)
    assert not ok
    assert reason == "COLLISION_MONITOR_VALID_INVALID"


def test_false_permissions_fault_while_armed():
    for name, reason in [
        ("localization", "LOCALIZATION_PERMISSION_INVALID"),
        ("controller", "CONTROLLER_PERMISSION_INVALID"),
        ("collision", "COLLISION_MONITOR_VALID_INVALID"),
    ]:
        core = configured_gate()
        make_ready(core, 1.0)
        assert core.request_arm(True, 2.2)[0]
        core.set_permission(name, False, 2.21)
        status = core.tick(2.22)
        assert status.state == STATE_FAULT
        assert status.reason_code == reason


def test_authority_conflicts_fault_and_block_arm():
    snapshots = [
        (AuthoritySnapshot(2, 1, 1, 1, 1), "SAFE_INPUT_AUTHORITY_CONFLICT"),
        (AuthoritySnapshot(1, 2, 1, 1, 1), "OUTPUT_AUTHORITY_CONFLICT"),
        (AuthoritySnapshot(1, 1, 2, 1, 1), "LOCALIZATION_AUTHORITY_CONFLICT"),
        (AuthoritySnapshot(1, 1, 1, 2, 1), "CONTROLLER_AUTHORITY_CONFLICT"),
        (AuthoritySnapshot(1, 1, 1, 1, 2), "COLLISION_VALID_AUTHORITY_CONFLICT"),
    ]
    for snapshot, reason in snapshots:
        core = configured_gate()
        make_ready(core, 1.0)
        core.set_authority(snapshot, 2.0)
        ok, block = core.request_arm(True, 3.2)
        assert not ok
        assert block == reason
        core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), 3.3)
        core.set_safe_command(Twist6(linear_x=0.10), 4.4)
        core.set_permission("localization", True, 4.4)
        core.set_permission("controller", True, 4.4)
        core.set_permission("collision", True, 4.4)
        assert core.request_arm(True, 4.5)[0]
        core.set_authority(snapshot, 4.6)
        core.set_safe_command(Twist6(linear_x=0.10), 4.7)
        status = core.tick(4.8)
        assert status.state == STATE_FAULT
        assert status.reason_code == reason


def test_fault_recovery_requires_cause_removed_disarm_and_new_arm():
    core = configured_gate()
    make_ready(core, 1.0)
    assert core.request_arm(True, 2.2)[0]
    core.set_permission("collision", False, 2.3)
    assert core.tick(2.31).state == STATE_FAULT
    ok, reason = core.request_arm(True, 2.32)
    assert not ok
    assert reason == "COLLISION_MONITOR_VALID_INVALID"
    ok, reason = core.request_arm(False, 2.33)
    assert not ok
    assert reason == "COLLISION_MONITOR_VALID_INVALID"
    core.set_permission("collision", True, 2.34)
    ok, reason = core.request_arm(False, 2.35)
    assert ok
    assert reason == "DISARMED_FAULT_CLEARED"
    assert core.state == STATE_DISARMED
    core.set_safe_command(Twist6(linear_x=0.10), 4.0)
    core.set_permission("localization", True, 4.0)
    core.set_permission("controller", True, 4.0)
    core.set_permission("collision", True, 4.0)
    ok, reason = core.request_arm(True, 2.6)
    assert not ok
    assert reason == "AUTHORITY_NOT_STABLE"
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), 3.6)
    core.set_safe_command(Twist6(linear_x=0.10), 4.8)
    core.set_permission("localization", True, 4.8)
    core.set_permission("controller", True, 4.8)
    core.set_permission("collision", True, 4.8)
    ok, reason = core.request_arm(True, 4.9)
    assert ok
    assert reason == "ARMED"
