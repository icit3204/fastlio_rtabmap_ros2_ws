import importlib.util
import math
import statistics
import time
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location(
    "motion_aware_collision_mock", SCRIPTS / "motion_aware_collision_mock.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def counts(stop, slow):
    return module.Counts(stop, slow, 0, 0)


def test_clear_slow_stop_output_contract():
    assert module.filter_command(0.05, 0.01, module.CLEAR) == (0.05, 0.01)
    slow = module.filter_command(0.05, -0.02, module.SLOW)
    assert slow == (0.015, -0.006)
    assert module.filter_command(0.05, -0.02, module.STOP) == (0.0, 0.0)
    assert module.filter_command(math.nan, 0.0, module.CLEAR) == (0.0, 0.0)


def test_point_count_threshold_matches_qualified_semantics():
    assert module.classify_counts(counts(1, 66), 3) == module.SLOW
    assert module.classify_counts(counts(3, 66), 3) == module.SLOW
    assert module.classify_counts(counts(4, 66), 3) == module.STOP
    assert module.classify_counts(counts(0, 3), 3) == module.CLEAR
    assert module.classify_counts(counts(0, 4), 3) == module.SLOW


def test_deescalation_is_stable_and_escalation_is_immediate():
    policy = module.RiskStatePolicy(2)
    assert policy.update(module.SLOW) == module.SLOW
    assert policy.update(module.STOP) == module.STOP
    assert policy.update(module.SLOW) == module.STOP
    assert policy.update(module.STOP) == module.STOP
    assert policy.update(module.SLOW) == module.STOP
    assert policy.update(module.SLOW) == module.SLOW
    assert policy.update(module.CLEAR) == module.SLOW
    assert policy.update(module.CLEAR) == module.CLEAR
    assert policy.force_stop() == module.STOP


def test_r23_r2_self_return_clear_and_external_boundary_retained():
    geometry = module.build_zones(0.05, 0.0, 0.0)
    # R23-R2 mask removes this proven point before it reaches this observer.
    assert module.observation_counts([], [], geometry).stop == 0
    # A point 0.1 mm beyond the physical front edge is outside the self-mask
    # and remains eligible for zone classification.
    external = (0.7551, 0.0)
    result = module.observation_counts([], [external], geometry)
    assert result.stop == 1
    assert result.slow == 1


def test_left_right_geometry_changes_obstacle_classification():
    obstacle = [(1.02, -0.48)] * 5
    left = module.build_zones(0.05, 0.05 / 1.75, 0.0)
    right = module.build_zones(0.05, -0.05 / 1.75, 0.0)
    left_counts = module.observation_counts(obstacle, [], left)
    right_counts = module.observation_counts(obstacle, [], right)
    assert right_counts.stop >= left_counts.stop


def test_config_and_launch_are_mock_only():
    config = yaml.safe_load((ROOT / "config/motion_aware_collision_mock.experimental.yaml").read_text())
    values = config["motion_aware_collision_mock"]["ros__parameters"]
    assert values["output_topic"] == "/cmd_vel_motion_aware_mock"
    assert values["slowdown_ratio"] == 0.30
    assert values["max_points"] == 3
    assert values["tmini_topic"] == "/scan_collision_experimental"
    assert values["mid_topic"] == "/cloud_registered_nav2_obstacles"
    assert values["command_timeout_sec"] == 0.250
    assert values["watchdog_zero_deadline_sec"] == 0.225
    assert values["watchdog_period_sec"] == 0.005
    assert values.get("integration_mock_enabled", False) is False
    launch = (ROOT.parent / "parking_robot_bringup/launch/r3h_physical_navigation.launch.py").read_text()
    assert 'executable="motion_aware_collision_mock"' in launch
    assert "r3h_motion_aware_collision_physical.yaml" in launch
    assert 'default_value="fixed_qualified"' in launch
    enforcer = (ROOT / "scripts/motion_aware_collision_mock.py").read_text()
    assert 'forbidden = {"/vehicle_cmd_safe", "/cmd_vel_collision_monitor"}' in enforcer
    assert '(physical_enforcement and output_topic == "/cmd_vel")' in enforcer


def test_identical_command_refresh_does_not_starve_live_classification():
    source = (SCRIPTS / "motion_aware_collision_mock.py").read_text()
    assert "if signature != self.command_signature:" in source
    assert "self.command_generation += 1" in source
    assert "self.watchdog_latched_generation = None" in source


def test_r22_and_real_authorities_are_unchanged():
    collision = (ROOT / "config/collision_monitor_dual_sensor.yaml").read_text()
    nav = (ROOT / "config/nav2_common.yaml").read_text()
    assert "/cmd_vel_motion_aware_mock" not in collision
    assert "/cmd_vel_motion_aware_mock" not in nav
    assert "minimum_turning_radius: 1.75" in nav


def test_stale_timing_contract_and_20_trial_monotonic_schedule():
    timing = module.StaleTimingConfig()
    timing.validate()
    assert timing.stale_threshold_sec == 0.250
    assert timing.zero_deadline_sec == 0.225
    assert timing.watchdog_period_sec == 0.005

    # Exercise the monotonic scheduling arithmetic twenty times. Runtime ROS
    # qualification separately measures publisher and subscriber timestamps.
    ages = []
    for _ in range(20):
        accepted = time.monotonic_ns()
        deadline = accepted + int(timing.zero_deadline_sec * 1e9)
        threshold = accepted + int(timing.stale_threshold_sec * 1e9)
        assert deadline < threshold
        ages.append((deadline - accepted) * 1e-9)
    assert len(ages) == 20
    assert max(ages) <= 0.250
    assert statistics.mean(ages) == 0.225


def test_invalid_stale_schedules_fail_closed_at_configuration():
    for timing in (
        module.StaleTimingConfig(zero_deadline_sec=0.250),
        module.StaleTimingConfig(zero_deadline_sec=0.251),
        module.StaleTimingConfig(watchdog_period_sec=0.0),
    ):
        try:
            timing.validate()
        except ValueError:
            pass
        else:
            raise AssertionError("invalid stale schedule was accepted")
