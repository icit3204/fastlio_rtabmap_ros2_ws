from pathlib import Path

import yaml

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


def prepare(core: GateCore, now: float = 1.0, command: Twist6 = Twist6()) -> None:
    core.set_authority(AuthoritySnapshot(1, 1, 1, 1, 1), now - 1.0)
    core.set_safe_command(command, now)
    core.set_permission("localization", True, now)
    core.set_permission("controller", True, now)
    core.set_permission("collision", True, now)


def arm_pending(core: GateCore) -> float:
    arm_time = 1.1
    ok, reason = core.request_arm(True, arm_time)
    assert ok and reason == REASON_ARM_PENDING_SAFE
    assert core.state == STATE_DISARMED
    assert core.arm_pending
    return arm_time


def refresh_permissions(core: GateCore, now: float) -> None:
    core.set_permission("localization", True, now)
    core.set_permission("controller", True, now)
    core.set_permission("collision", True, now)


def accept_after_arm(core: GateCore, command: Twist6 = Twist6(), now: float = 1.2) -> None:
    refresh_permissions(core, now)
    core.set_safe_command(command, now)
    assert core.state == STATE_ARMED
    assert not core.arm_pending


def test_t1_default_mode_retains_legacy_stale_safe_behavior():
    core = GateCore(GateConfig(
        max_forward_velocity=0.20, max_angular_velocity=0.50,
        max_linear_increase_rate=0.50, max_angular_increase_rate=0.50,
        safe_zero_quiescence_enabled=False,
    ))
    prepare(core, 1.0, Twist6(linear_x=0.10))
    assert core.request_arm(True, 1.1) == (True, "ARMED")
    status = core.tick(1.5)
    assert status.state == STATE_FAULT
    assert status.reason_code == "SAFE_TWIST_STALE"
    assert status.output.is_zero()


def test_t2_arm_without_post_arm_safe_message_is_pending_and_zero():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    status = core.tick(1.2)
    assert status.state == STATE_DISARMED
    assert status.reason_code == REASON_ARM_PENDING_SAFE
    assert status.output.is_zero()


def test_t3_pre_arm_cached_zero_does_not_satisfy_arm():
    core = phase5_gate()
    prepare(core, command=Twist6())
    arm_pending(core)
    refresh_permissions(core, 2.0)
    assert core.tick(2.1).reason_code == REASON_ARM_PENDING_SAFE
    accept_after_arm(core, Twist6(), 2.2)
    assert core.state == STATE_ARMED


def test_t4_pre_arm_cached_nonzero_does_not_satisfy_arm():
    core = phase5_gate()
    prepare(core, command=Twist6(linear_x=0.10))
    arm_pending(core)
    refresh_permissions(core, 2.0)
    assert core.tick(2.1).reason_code == REASON_ARM_PENDING_SAFE
    accept_after_arm(core, Twist6(linear_x=0.10), 2.2)
    assert core.state == STATE_ARMED


def test_t5_fresh_post_arm_zero_completes_readiness():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    status = core.tick(1.21)
    assert status.state == STATE_ARMED
    assert status.output.is_zero()
    assert status.reason_code == "ARMED_COMMAND"


def test_t6_fresh_post_arm_nonzero_uses_normal_slew():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(linear_x=0.20), 1.2)
    status = core.tick(1.21)
    assert status.state == STATE_ARMED
    assert 0.0 < status.output.linear_x <= 0.005 + 1e-12


def test_t7_stale_accepted_zero_enters_quiescence_without_fault():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    status = core.tick(10.0)
    assert status.state == STATE_ARMED
    assert status.reason_code == REASON_SAFE_ZERO_QUIESCENT
    assert status.output.is_zero()
    assert status.diagnostics["safe_zero_quiescent"] == "true"


def test_t8_stale_accepted_nonzero_faults():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(linear_x=0.10), 1.2)
    refresh_permissions(core, 1.6)
    status = core.tick(1.6)
    assert status.state == STATE_FAULT
    assert status.reason_code == "SAFE_TWIST_STALE"
    assert status.output.is_zero()


def test_t9_collision_validity_loss_during_quiescence_faults():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    core.set_permission("collision", False, 10.0)
    status = core.tick(10.0)
    assert status.state == STATE_FAULT
    assert status.reason_code == "COLLISION_MONITOR_VALID_INVALID"
    assert status.output.is_zero()


def test_t10_localization_validity_loss_during_quiescence_faults():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    core.set_permission("localization", False, 10.0)
    status = core.tick(10.0)
    assert status.state == STATE_FAULT
    assert status.reason_code == "LOCALIZATION_PERMISSION_INVALID"


def test_t11_controller_validity_loss_during_quiescence_faults():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    core.set_permission("controller", False, 10.0)
    status = core.tick(10.0)
    assert status.state == STATE_FAULT
    assert status.reason_code == "CONTROLLER_PERMISSION_INVALID"


def test_t12_pending_arm_validity_failure_faults_closed():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    core.set_permission("collision", False, 1.2)
    status = core.tick(1.2)
    assert status.state == STATE_FAULT
    assert status.output.is_zero()


def test_t13_arm_false_cancels_pending():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    ok, reason = core.request_arm(False, 1.2)
    assert ok and reason == "DISARMED_ARM_PENDING_CANCELLED"
    assert core.state == STATE_DISARMED
    assert not core.arm_pending
    assert core.tick(2.0).output.is_zero()


def test_t14_healthy_disarm_from_quiescence():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    assert core.tick(10.0).reason_code == REASON_SAFE_ZERO_QUIESCENT
    ok, reason = core.request_arm(False, 10.1)
    assert ok and reason == "DISARMED_FAULT_CLEARED"
    assert core.state == STATE_DISARMED
    assert core.tick(10.2).output.is_zero()


def test_t15_fresh_nonzero_after_quiescence_resumes_protected_slew():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    assert core.tick(10.0).reason_code == REASON_SAFE_ZERO_QUIESCENT
    core.set_safe_command(Twist6(linear_x=0.20), 10.1)
    status = core.tick(10.11)
    assert status.state == STATE_ARMED
    assert status.output.linear_x <= 0.05 + 1e-12


def test_t16_fresh_zero_after_quiescence_refreshes_normal_state():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    assert core.tick(10.0).reason_code == REASON_SAFE_ZERO_QUIESCENT
    core.set_safe_command(Twist6(), 10.1)
    status = core.tick(10.11)
    assert status.state == STATE_ARMED
    assert status.reason_code == "ARMED_COMMAND"
    assert status.output.is_zero()


def test_t17_repeated_zero_silence_never_creates_nonzero_output():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    for now in (3.0, 6.0, 12.0, 30.0):
        refresh_permissions(core, now)
        status = core.tick(now)
        assert status.state == STATE_ARMED
        assert status.output.is_zero()


def test_t18_fault_latch_and_reset_semantics_remain_explicit():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(linear_x=0.10), 1.2)
    refresh_permissions(core, 1.6)
    assert core.tick(1.6).state == STATE_FAULT
    ok, _ = core.request_arm(False, 1.7)
    assert not ok
    refresh_permissions(core, 2.0)
    core.set_safe_command(Twist6(linear_x=0.10), 2.0)
    ok, reason = core.request_arm(False, 2.1)
    assert ok and reason == "DISARMED_FAULT_CLEARED"
    assert core.state == STATE_DISARMED


def test_t19_zero_classification_uses_all_six_components_and_existing_epsilon():
    assert Twist6(linear_x=1e-10, angular_z=-1e-10).is_zero()
    assert not Twist6(angular_y=1e-8).is_zero()
    assert not Twist6(linear_z=1e-8).is_zero()


def test_t20_default_and_phase5_configurations_are_distinct_and_valid():
    root = Path(__file__).resolve().parents[2]
    default = yaml.safe_load((root / "vehicle_cmd_safety" / "config" / "phase4_p4c_gate_mock.yaml").read_text())
    phase5 = yaml.safe_load((root / "parking_robot_bringup" / "config" / "phase5_gate_mock.yaml").read_text())
    default_params = default["guarded_vehicle_cmd_gate"]["ros__parameters"]
    phase5_params = phase5["guarded_vehicle_cmd_gate"]["ros__parameters"]
    assert default_params["safe_zero_quiescence_enabled"] is False
    assert phase5_params["safe_zero_quiescence_enabled"] is True
    assert GateConfig(
        max_forward_velocity=0.20,
        max_angular_velocity=0.50,
        max_linear_increase_rate=0.50,
        max_angular_increase_rate=0.50,
        safe_zero_quiescence_enabled=True,
    ).validation_error() is None


def test_cm_contract_simulation_zero_for_two_seconds_then_silence_stays_quiescent():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    for now in (1.3, 1.8, 2.7, 3.2):
        refresh_permissions(core, now)
        status = core.tick(now)
        assert status.output.is_zero()
    refresh_permissions(core, 10.0)
    assert core.tick(10.0).reason_code == REASON_SAFE_ZERO_QUIESCENT


def test_startup_after_collision_monitor_already_quiet_requires_new_message():
    core = phase5_gate()
    prepare(core, command=Twist6())
    arm_pending(core)
    for now in (2.0, 5.0, 8.0):
        refresh_permissions(core, now)
        status = core.tick(now)
        assert status.state == STATE_DISARMED
        assert status.reason_code == REASON_ARM_PENDING_SAFE
        assert status.output.is_zero()
    accept_after_arm(core, Twist6(), 8.1)
    assert core.tick(8.11).state == STATE_ARMED


def test_validity_loss_during_zero_silence_is_not_hidden():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    refresh_permissions(core, 10.0)
    assert core.tick(10.0).reason_code == REASON_SAFE_ZERO_QUIESCENT
    core.set_permission("collision", False, 10.1)
    assert core.tick(10.1).state == STATE_FAULT


def test_raw_nonzero_without_new_safe_message_remains_zero():
    core = phase5_gate()
    prepare(core)
    arm_pending(core)
    accept_after_arm(core, Twist6(), 1.2)
    for now in (5.0, 10.0):
        refresh_permissions(core, now)
        status = core.tick(now)
        assert status.output.is_zero()


def test_node_and_phase5_launch_expose_the_opt_in_contract():
    root = Path(__file__).resolve().parents[2]
    node = (root / "vehicle_cmd_safety" / "vehicle_cmd_safety" / "guarded_vehicle_cmd_gate.py").read_text()
    launch = (root / "parking_robot_bringup" / "launch" / "phase5_no_motion.launch.py").read_text()
    assert 'declare_parameter("safe_zero_quiescence_enabled", False)' in node
    assert 'safe_zero_quiescence_enabled=bool(' in node
    assert '"phase5_gate_mock.yaml"' in launch
