"""P5-3F5J1 stale-zero fault-reset completeness tests."""

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


def set_permissions(core: GateCore, now: float, *, collision: bool = True,
                    localization: bool = True, controller: bool = True) -> None:
    core.set_permission("localization", localization, now)
    core.set_permission("controller", controller, now)
    core.set_permission("collision", collision, now)


def prepare(core: GateCore, command: Twist6 = ZERO) -> None:
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), 0.0)
    core.set_safe_command(command, 1.0)
    set_permissions(core, 1.0)


def arm_and_accept(core: GateCore, command: Twist6 = ZERO) -> None:
    assert core.request_arm(True, 1.1) == (True, REASON_ARM_PENDING_SAFE)
    set_permissions(core, 1.2)
    core.set_safe_command(command, 1.2)
    assert core.state == STATE_ARMED


def enter_zero_quiescence(core: GateCore, now: float = 10.0) -> None:
    set_permissions(core, now)
    status = core.tick(now)
    assert status.state == STATE_ARMED
    assert status.reason_code == REASON_SAFE_ZERO_QUIESCENT
    assert status.output.is_zero()


def fault_on_permission(core: GateCore, name: str, now: float) -> None:
    set_permissions(core, now)
    core.set_permission(name, False, now)
    status = core.tick(now)
    assert status.state == STATE_FAULT
    assert status.output.is_zero()


def recover_permissions(core: GateCore, now: float) -> None:
    set_permissions(core, now)
    core.tick(now)


def test_j1_t1_cleared_collision_fault_with_stale_zero_resets():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "collision", 10.1)
    recover_permissions(core, 11.0)
    assert core.request_arm(False, 11.1) == (True, "DISARMED_FAULT_CLEARED")
    assert core.state == STATE_DISARMED
    assert core.tick(11.2).output.is_zero()


def test_j1_t2_cleared_localization_fault_with_stale_zero_resets():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "localization", 10.1)
    recover_permissions(core, 11.0)
    assert core.request_arm(False, 11.1)[0]
    assert core.state == STATE_DISARMED


def test_j1_t3_cleared_controller_fault_with_stale_zero_resets():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "controller", 10.1)
    recover_permissions(core, 11.0)
    assert core.request_arm(False, 11.1)[0]
    assert core.state == STATE_DISARMED


def test_j1_t4_cleared_authority_fault_with_stale_zero_resets_after_stability():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    core.set_authority(AuthoritySnapshot(2, 1, 1, 1, 1), 10.1)
    assert core.tick(10.1).state == STATE_FAULT
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), 11.0)
    set_permissions(core, 12.1)
    assert core.tick(12.1).state == STATE_FAULT
    assert core.request_arm(False, 12.2)[0]
    assert core.state == STATE_DISARMED


def test_j1_t5_active_collision_fault_still_refuses_reset_with_stale_zero():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "collision", 10.1)
    assert core.request_arm(False, 10.2) == (
        False, "COLLISION_MONITOR_VALID_INVALID"
    )
    assert core.state == STATE_FAULT


def test_j1_t6_stale_nonzero_fault_still_refuses_reset():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core, Twist6(linear_x=0.10))
    set_permissions(core, 1.6)
    assert core.tick(1.6).reason_code == "SAFE_TWIST_STALE"
    assert core.request_arm(False, 1.7) == (False, "SAFE_TWIST_STALE")
    assert core.state == STATE_FAULT


def test_j1_t7_fresh_nonzero_restores_stale_nonzero_reset():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core, Twist6(linear_x=0.10))
    set_permissions(core, 1.6)
    assert core.tick(1.6).state == STATE_FAULT
    set_permissions(core, 2.0)
    core.set_safe_command(Twist6(linear_x=0.10), 2.0)
    assert core.request_arm(False, 2.1) == (True, "DISARMED_FAULT_CLEARED")
    assert core.state == STATE_DISARMED


def test_j1_t8_pending_arm_fault_recovers_without_safe_command():
    core = phase5_gate()
    prepare(core)
    assert core.request_arm(True, 1.1) == (True, REASON_ARM_PENDING_SAFE)
    core.set_permission("collision", False, 1.2)
    assert core.tick(1.2).state == STATE_FAULT
    set_permissions(core, 2.0)
    assert core.request_arm(False, 2.1) == (True, "DISARMED_ARM_PENDING_CANCELLED")
    assert core.state == STATE_DISARMED


def test_j1_t9_reset_requires_new_pending_arm_cycle():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "collision", 10.1)
    recover_permissions(core, 11.0)
    assert core.request_arm(False, 11.1)[0]
    set_permissions(core, 12.2)
    assert core.request_arm(True, 12.3) == (True, REASON_ARM_PENDING_SAFE)
    assert core.state == STATE_DISARMED
    assert core.arm_pending


def test_j1_t10_previous_stale_zero_does_not_satisfy_next_arm():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "collision", 10.1)
    recover_permissions(core, 11.0)
    assert core.request_arm(False, 11.1)[0]
    set_permissions(core, 12.2)
    assert core.request_arm(True, 12.3)[0]
    set_permissions(core, 20.0)
    status = core.tick(20.0)
    assert status.state == STATE_DISARMED
    assert status.reason_code == REASON_ARM_PENDING_SAFE
    assert status.output.is_zero()


def test_j1_t11_only_strictly_new_post_arm_receipt_completes_readiness():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    enter_zero_quiescence(core)
    fault_on_permission(core, "collision", 10.1)
    recover_permissions(core, 11.0)
    assert core.request_arm(False, 11.1)[0]
    set_permissions(core, 12.2)
    arm_time = 12.3
    assert core.request_arm(True, arm_time)[0]
    set_permissions(core, arm_time)
    core.set_safe_command(ZERO, arm_time)
    assert core.state == STATE_DISARMED
    core.set_safe_command(ZERO, arm_time + 0.01)
    assert core.state == STATE_ARMED
    assert core.safe_command_accepted_after_arm


def test_j1_t12_default_mode_keeps_legacy_reset_behavior():
    core = GateCore(GateConfig(
        max_forward_velocity=0.20,
        max_angular_velocity=0.50,
        max_linear_increase_rate=0.50,
        max_angular_increase_rate=0.50,
        safe_zero_quiescence_enabled=False,
    ))
    prepare(core, Twist6(linear_x=0.10))
    assert core.request_arm(True, 1.1) == (True, "ARMED")
    set_permissions(core, 1.6)
    assert core.tick(1.6).reason_code == "SAFE_TWIST_STALE"
    assert core.request_arm(False, 1.7) == (False, "SAFE_TWIST_STALE")


def test_j1_full_final_fault_cycle_and_post_reset_readiness():
    core = phase5_gate()
    prepare(core)
    arm_and_accept(core)
    core.set_safe_command(Twist6(linear_x=0.20), 1.3)
    assert core.tick(1.31).output.linear_x > 0.0

    core.set_permission("collision", False, 1.4)
    assert core.tick(1.4).state == STATE_FAULT
    core.set_safe_command(ZERO, 1.5)
    assert core.request_arm(False, 1.6) == (
        False, "COLLISION_MONITOR_VALID_INVALID"
    )

    set_permissions(core, 3.0)
    assert core.tick(3.0).state == STATE_FAULT
    assert core.request_arm(True, 3.1) == (
        False, "COLLISION_MONITOR_VALID_INVALID"
    )
    assert core.request_arm(False, 3.2) == (True, "DISARMED_FAULT_CLEARED")
    assert core.state == STATE_DISARMED
    assert core.tick(3.3).output.is_zero()

    set_permissions(core, 4.3)
    assert core.request_arm(True, 4.4) == (True, REASON_ARM_PENDING_SAFE)
    set_permissions(core, 8.0)
    assert core.tick(8.0).reason_code == REASON_ARM_PENDING_SAFE
    core.set_safe_command(ZERO, 8.01)
    assert core.state == STATE_ARMED
    core.set_safe_command(Twist6(linear_x=0.20), 8.02)
    assert core.tick(8.03).output.linear_x > 0.0
