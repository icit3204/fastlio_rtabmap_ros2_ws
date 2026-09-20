import importlib.util
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/tmini_collision_self_mask.py"
CONFIG = ROOT / "config/tmini_collision_self_mask.experimental.yaml"
R3H_LAUNCH = ROOT.parent / "parking_robot_bringup/launch/r3h_physical_navigation.launch.py"

spec = importlib.util.spec_from_file_location("tmini_collision_self_mask", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_proven_cluster_is_inside_physical_body_and_removed():
    config = module.SelfMaskConfig()
    for point in ((0.6453, -0.1028), (0.6805, 0.0936), (0.656, 0.0)):
        assert module.point_in_physical_body(*point, config)
        dx, dy = point[0] - config.sensor_x, point[1] - config.sensor_y
        distance, angle = math.hypot(dx, dy), math.atan2(dy, dx)
        ranges, removed = module.mask_ranges(
            [distance], angle, 1.0, 0.01, 16.0, config)
        assert removed == 1
        assert math.isinf(ranges[0])


def test_external_points_immediately_outside_each_relevant_edge_are_retained():
    config = module.SelfMaskConfig()
    points = (
        (config.footprint_x_max + 1e-4, 0.0),
        (0.50, config.footprint_y_max + 1e-4),
        (0.50, config.footprint_y_min - 1e-4),
    )
    for x, y in points:
        assert not module.point_in_physical_body(x, y, config)
        distance = math.hypot(x-config.sensor_x, y-config.sensor_y)
        angle = math.atan2(y-config.sensor_y, x-config.sensor_x)
        ranges, removed = module.mask_ranges(
            [distance], angle, 1.0, 0.00001, 16.0, config)
        assert removed == 0
        assert ranges[0] == distance


def test_nonfinite_and_out_of_range_values_are_not_reinterpreted():
    config = module.SelfMaskConfig()
    ranges, removed = module.mask_ranges(
        [math.inf, math.nan, 0.001, 20.0], -1.0, 0.1, 0.01, 16.0, config)
    assert removed == 0
    assert math.isinf(ranges[0])
    assert math.isnan(ranges[1])
    assert ranges[2:] == (0.001, 20.0)


def test_config_is_unpadded_physical_footprint_and_dedicated_topic():
    values = yaml.safe_load(CONFIG.read_text())["tmini_collision_self_mask"]["ros__parameters"]
    assert values["input_topic"] == "/scan"
    assert values["output_topic"] == "/scan_collision_experimental"
    assert values["expected_frame"] == "laser_frame"
    assert [values[key] for key in ("footprint_x_min", "footprint_x_max",
                                    "footprint_y_min", "footprint_y_max")] == [
        -0.145, 0.755, -0.300, 0.300]


def test_r22_topics_are_not_modified_by_self_mask_contract():
    collision = (ROOT / "config/collision_monitor_dual_sensor.yaml").read_text()
    nav = (ROOT / "config/nav2_common.yaml").read_text()
    assert "/scan_collision_experimental" not in collision
    assert "/scan_collision_experimental" not in nav
    assert 'topic: "/scan"' in collision


def test_mask_is_wired_only_to_experimental_selected_mode():
    launch = R3H_LAUNCH.read_text()
    assert 'executable="tmini_collision_self_mask"' in launch
    assert '"motion_aware_experimental"' in launch
    assert "r3h_motion_aware_collision_physical.yaml" in launch
    assert "fixed_qualified" in launch
