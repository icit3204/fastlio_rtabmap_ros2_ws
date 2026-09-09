"""P5-3F5J2 arm-pending fault-latch immutability tests."""

from vehicle_cmd_safety.gate_core import (
    AuthoritySnapshot,
    GateConfig,
    GateCore,
    REASON_ARM_PENDING_SAFE,
    REASON_SAFE_ZERO_QUIESCENT,
    STATE_ARMED,
    STATE_DISARMED,
    STATE_FAULT,
    Twist6,
)


ZERO = Twist6()


def phase5_gate() -> GateCore:
    return GateCore(
        GateConfig(
            max_forward_velocity=0.20,
            max_angular_velocity=0.50,
            max_linear_increase_rate=0.50,
            max_angular_increase_rate=0.50,
            safe_zero_quiescence_enabled=True,
        )
    )


def permissions(core: GateCore, now: float, *, collision: bool = True,
                localization: bool = True, controller: bool = True) -> None:
    core.set_permission("localization", localization, now)
    core.set_permission("controller", controller, now)
    core.set_permission("collision", collision, now)


def prepared(core: GateCore, command: Twist6 = ZERO) -> None:
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), 0.0)
    core.set_safe_command(command, 1.0)
    permissions(core, 1.9)


def pending(core: GateCore, arm_time: float = 2.0) -> float:
    assert core.request_arm(True, arm_time) == (True, REASON_ARM_PENDING_SAFE)
    assert core.state == STATE_DISARMED
    assert core.arm_pending
    return arm_time


def accept_after_arm(core: GateCore, now: float = 2.1,
                     command: Twist6 = ZERO) -> None:
    permissions(core, now)
    core.set_safe_command(command, now)


def pending_collision_fault(core: GateCore) -> None:
    pending(core)
    permissions(core, 2.1, collision=False)
    status = core.tick(2.1)
    assert status.state == STATE_FAULT
    assert status.reason_code == "COLLISION_MONITOR_VALID_INVALID"
    assert core.arm_pending


def test_j2_t1_normal_pending_path_is_unchanged():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core)
    assert core.state == STATE_ARMED
    assert not core.arm_pending


def test_j2_t2_pending_collision_fault_latches_and_keeps_pending_epoch():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    assert core.fault_reason == "COLLISION_MONITOR_VALID_INVALID"


def test_j2_t3_recovery_and_fresh_zero_cannot_arm_from_fault():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    core.set_safe_command(ZERO, 3.1)
    assert core.state == STATE_FAULT
    assert core.arm_pending
    assert not core.safe_command_accepted_after_arm


def test_j2_t4_recovery_and_fresh_nonzero_cannot_arm_from_fault():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    core.set_safe_command(Twist6(linear_x=0.10), 3.1)
    assert core.state == STATE_FAULT
    assert core.arm_pending
    assert not core.safe_command_accepted_after_arm


def test_j2_t5_arm_true_is_refused_while_fault_is_latched():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    assert core.request_arm(True, 3.1) == (
        False, "COLLISION_MONITOR_VALID_INVALID"
    )


def test_j2_t6_arm_false_is_refused_while_collision_cause_is_active():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    assert core.request_arm(False, 2.2) == (
        False, "COLLISION_MONITOR_VALID_INVALID"
    )
    assert core.state == STATE_FAULT


def test_j2_t7_cleared_pending_fault_requires_explicit_cancel_reset():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    assert core.request_arm(False, 3.1) == (
        True, "DISARMED_ARM_PENDING_CANCELLED"
    )
    assert core.state == STATE_DISARMED
    assert not core.arm_pending
    assert core.fault_reason is None


def test_j2_t8_cached_safe_cannot_satisfy_next_arm_after_reset():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    core.set_safe_command(ZERO, 3.1)
    assert core.request_arm(False, 3.2)[0]
    permissions(core, 4.4)
    pending(core, 4.5)
    status = core.tick(4.6)
    assert status.state == STATE_DISARMED
    assert status.reason_code == REASON_ARM_PENDING_SAFE
    assert not core.safe_command_accepted_after_arm


def test_j2_t9_new_arm_starts_a_new_pending_epoch():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    assert core.request_arm(False, 3.1)[0]
    permissions(core, 4.4)
    assert core.request_arm(True, 4.5) == (True, REASON_ARM_PENDING_SAFE)
    assert core.state == STATE_DISARMED
    assert core.arm_pending


def test_j2_t10_new_post_arm_safe_zero_arms_normally():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    assert core.request_arm(False, 3.1)[0]
    permissions(core, 4.4)
    pending(core, 4.5)
    accept_after_arm(core, 4.6, ZERO)
    assert core.state == STATE_ARMED
    assert not core.arm_pending


def test_j2_t11_ordinary_armed_fault_latch_is_unchanged():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, Twist6(linear_x=0.10))
    permissions(core, 2.5, collision=False)
    status = core.tick(2.5)
    assert status.state == STATE_FAULT
    assert status.reason_code == "COLLISION_MONITOR_VALID_INVALID"
    assert status.output.is_zero()


def test_j2_t12_stale_zero_reset_after_independent_fault_is_preserved():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, ZERO)
    permissions(core, 10.0)
    assert core.tick(10.0).reason_code == REASON_SAFE_ZERO_QUIESCENT
    permissions(core, 10.1, collision=False)
    assert core.tick(10.1).state == STATE_FAULT
    permissions(core, 11.0)
    assert core.request_arm(False, 11.1) == (True, "DISARMED_FAULT_CLEARED")
    assert core.state == STATE_DISARMED


def test_j2_t13_stale_nonzero_remains_strict_until_command_recovers():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, Twist6(linear_x=0.10))
    permissions(core, 2.5)
    assert core.tick(2.5).reason_code == "SAFE_TWIST_STALE"
    assert core.request_arm(False, 2.6) == (False, "SAFE_TWIST_STALE")
    permissions(core, 3.0)
    core.set_safe_command(Twist6(linear_x=0.10), 3.0)
    assert core.request_arm(False, 3.1) == (True, "DISARMED_FAULT_CLEARED")


def test_j2_t14_safe_zero_quiescence_is_unchanged():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, ZERO)
    permissions(core, 10.0)
    status = core.tick(10.0)
    assert status.state == STATE_ARMED
    assert status.reason_code == REASON_SAFE_ZERO_QUIESCENT
    assert status.output.is_zero()
    assert status.diagnostics["safe_zero_quiescent"] == "true"


def test_j2_t15_explicit_reset_clears_fault_reason():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    assert core.request_arm(False, 3.1)[0]
    assert core.fault_reason is None
    assert not core.contributing_faults


def test_j2_t16_new_fault_after_reset_gets_its_own_primary_reason():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    assert core.request_arm(False, 3.1)[0]
    permissions(core, 4.4)
    pending(core, 4.5)
    permissions(core, 4.6, localization=False)
    status = core.tick(4.6)
    assert status.state == STATE_FAULT
    assert status.reason_code == "LOCALIZATION_PERMISSION_INVALID"
    assert core.fault_reason == "LOCALIZATION_PERMISSION_INVALID"


def test_j2_t17_exact_pending_fault_reproduction_is_immutably_latched():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    core.set_safe_command(ZERO, 3.1)
    assert core.state == STATE_FAULT
    assert core.fault_reason == "COLLISION_MONITOR_VALID_INVALID"
    assert core.arm_pending


def test_j2_t18_historical_reason_cannot_auto_rearm_or_contaminate_new_epoch():
    core = phase5_gate()
    prepared(core)
    pending_collision_fault(core)
    permissions(core, 3.0)
    core.set_safe_command(Twist6(linear_x=0.10), 3.1)
    assert core.state == STATE_FAULT
    assert core.fault_reason == "COLLISION_MONITOR_VALID_INVALID"
    permissions(core, 4.0)
    assert core.request_arm(False, 4.1)[0]
    permissions(core, 5.0)
    pending(core, 5.1)
    permissions(core, 5.2, localization=False)
    status = core.tick(5.2)
    assert status.state == STATE_FAULT
    assert status.reason_code == "LOCALIZATION_PERMISSION_INVALID"
    assert core.fault_reason == "LOCALIZATION_PERMISSION_INVALID"


def test_j2_fault_reason_audit_a_safe_twist_stale_is_primary():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, Twist6(linear_x=0.10))
    permissions(core, 2.5)
    status = core.tick(2.5)
    assert status.reason_code == "SAFE_TWIST_STALE"
    assert core.fault_reason == "SAFE_TWIST_STALE"


def test_j2_fault_reason_audit_b_localization_invalid_is_primary():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, ZERO)
    permissions(core, 2.5, localization=False)
    status = core.tick(2.5)
    assert status.reason_code == "LOCALIZATION_PERMISSION_INVALID"
    assert core.fault_reason == "LOCALIZATION_PERMISSION_INVALID"


def test_j2_fault_reason_audit_c_collision_invalid_is_primary():
    core = phase5_gate()
    prepared(core)
    pending(core)
    accept_after_arm(core, 2.1, ZERO)
    permissions(core, 2.5, collision=False)
    status = core.tick(2.5)
    assert status.reason_code == "COLLISION_MONITOR_VALID_INVALID"
    assert core.fault_reason == "COLLISION_MONITOR_VALID_INVALID"
