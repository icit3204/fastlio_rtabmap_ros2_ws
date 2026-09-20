import importlib.util
import math
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
    launch = (ROOT.parent / "parking_robot_bringup/launch/r3h_physical_navigation.launch.py").read_text()
    assert 'executable="motion_aware_collision_mock"' in launch
    assert "R23-R3 MOCK ONLY" in launch
    assert 'default_value="fixed_qualified"' in launch
    assert "/vehicle_cmd_safe" not in (ROOT / "scripts/motion_aware_collision_mock.py").read_text().split("forbidden =", 1)[0]


def test_r22_and_real_authorities_are_unchanged():
    collision = (ROOT / "config/collision_monitor_dual_sensor.yaml").read_text()
    nav = (ROOT / "config/nav2_common.yaml").read_text()
    assert "/cmd_vel_motion_aware_mock" not in collision
    assert "/cmd_vel_motion_aware_mock" not in nav
    assert "minimum_turning_radius: 1.75" in nav
