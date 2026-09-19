"""R23 software-only motion-aware geometry and R22 isolation contracts."""

import importlib.util
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/motion_aware_collision_geometry.py"
SPEC = importlib.util.spec_from_file_location("motion_geometry", SCRIPT)
GEOMETRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEOMETRY)
CONFIG = GEOMETRY.GeometryConfig()


def assert_nested(result):
    assert all(GEOMETRY.point_in_polygon(point, result.slow) for point in result.stop)


def assert_sweep_protected(result, distance, curvature):
    footprint = GEOMETRY._expanded_footprint(CONFIG, CONFIG.stop_extra_margin)
    steps = max(1, int(math.ceil(distance / CONFIG.sample_step)))
    for index in range(steps + 1):
        pose = GEOMETRY.trajectory_pose(distance * index / steps, curvature)
        for point in GEOMETRY.transform_polygon(footprint, pose):
            assert GEOMETRY.point_in_polygon(point, result.stop)


def test_zero_and_stale_commands_use_conservative_fallback():
    for linear, angular, age in ((0.0, 0.0, 0.0), (0.05, 0.0, 1.0),
                                 (float("nan"), 0.0, 0.0), (-0.05, 0.0, 0.0)):
        result = GEOMETRY.build_zones(linear, angular, age, CONFIG)
        assert result.state == "CONSERVATIVE_FIXED_FALLBACK"
        assert_nested(result)
        assert GEOMETRY.point_in_polygon((1.04, 0.52), result.stop)
        assert GEOMETRY.point_in_polygon((1.34, 0.52), result.slow)


def test_straight_forward_is_symmetric_nested_and_body_protecting():
    result = GEOMETRY.build_zones(0.05, 0.0, 0.0, CONFIG)
    assert result.state == "MOTION_AWARE"
    assert_nested(result)
    assert_sweep_protected(result, result.stop_distance, 0.0)
    assert math.isclose(min(y for _, y in result.stop),
                        -max(y for _, y in result.stop), abs_tol=1e-12)
    assert max(x for x, _ in result.slow) > max(x for x, _ in result.stop)


def test_left_and_right_turns_protect_the_correct_swept_side():
    for sign in (1.0, -1.0):
        curvature = sign / CONFIG.minimum_turning_radius
        result = GEOMETRY.build_zones(0.05, 0.05 * curvature, 0.0, CONFIG)
        assert result.state == "MOTION_AWARE"
        assert math.isclose(result.curvature, curvature, abs_tol=1e-12)
        assert_nested(result)
        assert_sweep_protected(result, result.stop_distance, curvature)
        if sign > 0:
            assert max(y for _, y in result.stop) > abs(min(y for _, y in result.stop))
        else:
            assert abs(min(y for _, y in result.stop)) > max(y for _, y in result.stop)


def test_gentle_turn_matrix_and_invalid_curvature_fallback():
    for curvature in (1.0 / 3.5, -1.0 / 3.5):
        result = GEOMETRY.build_zones(0.05, 0.05 * curvature, 0.0, CONFIG)
        assert result.state == "MOTION_AWARE"
        assert_nested(result)
    invalid = GEOMETRY.build_zones(0.05, 0.05 / 1.60, 0.0, CONFIG)
    assert invalid.state == "CONSERVATIVE_FIXED_FALLBACK"


def test_r20_exact_right_command_excludes_table_from_stop_not_slow():
    result = GEOMETRY.build_zones(0.04, -0.022857142857142857, 0.0, CONFIG)
    closest = (0.9865832492735351, -0.4392577719820669)
    assert not GEOMETRY.point_in_polygon(closest, result.stop)
    assert GEOMETRY.point_in_polygon(closest, result.slow)
    # Independent R20 full-path audit retained 0.182559 m padded-body clearance.
    assert 0.18255917527812265 > CONFIG.stop_extra_margin


def test_r22_sensor_contract_and_fixed_default_are_unchanged():
    fixed = yaml.safe_load((ROOT / "config/collision_monitor_dual_sensor.yaml").read_text())
    params = fixed["collision_monitor"]["ros__parameters"]
    assert params["polygons"] == ["Phase5Stop", "Phase5Slow"]
    assert params["Phase5Stop"]["points"] == [
        0.785, 0.53, 1.05, 0.53, 1.05, -0.53, 0.785, -0.53]
    assert params["Phase5Slow"]["points"] == [
        0.785, 0.53, 1.35, 0.53, 1.35, -0.53, 0.785, -0.53]
    assert params["mid360_body_cloud"]["topic"] == "/cloud_registered_nav2_obstacles"
    assert params["tmini_scan"]["topic"] == "/scan"
    launch = (ROOT.parent / "parking_robot_bringup/launch/r3h_physical_navigation.launch.py").read_text()
    assert 'DeclareLaunchArgument("collision_monitor_mode", default_value="fixed_qualified")' in launch
    assert "motion_aware_experimental" in launch
    assert "R23_SHADOW_ONLY" in launch
