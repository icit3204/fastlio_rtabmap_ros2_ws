import ast
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parent
WORKSPACE = SRC.parent
ROBOT_LAUNCH = SRC / "robot_bringup" / "launch" / "phase5_calibrated_localization.launch.py"
RTAB_BRIDGE = SRC / "robot_bringup" / "launch" / "rtabmap_bridge.launch.py"
PHASE5_LAUNCH = PACKAGE / "launch" / "phase5_no_motion.launch.py"
NAV = PACKAGE / "config" / "phase5_no_motion_nav2.yaml"
COLLISION = PACKAGE / "config" / "collision_monitor_phase5_raw_mid360.yaml"
COLLISION_VALIDITY = PACKAGE / "config" / "phase5_collision_monitor_validity.yaml"
LIVOX_CONFIG = SRC / "livox_ros_driver2" / "config" / "MID360_config.json"
LIVOX_PUBLISHER = SRC / "livox_ros_driver2" / "src" / "comm" / "pub_handler.cpp"
FAST_LIO = SRC / "FAST_LIO_ROS2" / "config" / "mid360.yaml"
CALIBRATION = WORKSPACE / "docs" / "phases" / "phase_05" / "PHASE5_PHYSICAL_FRAME_CALIBRATION_AUTHORITY.yaml"
ADAPTER_CONFIG = SRC / "wheelchair_cmd_adapter" / "config" / "mock_wheelchair_cmd_adapter.yaml"
GATE_SOURCE = SRC / "vehicle_cmd_safety" / "vehicle_cmd_safety" / "gate_core.py"
PACKAGE_XML = PACKAGE / "package.xml"


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def literal_assignment(path, name):
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"missing literal assignment {name}")


def polygon_extents(flat_points):
    xs = flat_points[0::2]
    ys = flat_points[1::2]
    return min(xs), max(xs), min(ys), max(ys)


def test_p5_0c0_calibration_authority_is_preserved_exactly():
    assert hashlib.sha256(CALIBRATION.read_bytes()).hexdigest() == (
        "c706d16af4c6607f73b6f9acdb7e853d5f3807067b1a62781d62d90263041600"
    )


def test_calibrated_static_tf_direction_values_and_single_authority():
    transforms = literal_assignment(ROBOT_LAUNCH, "CALIBRATED_STATIC_TRANSFORMS")
    by_edge = {(item["parent"], item["child"]): item for item in transforms}
    assert set(by_edge) == {
        ("body", "base_footprint"),
        ("base_footprint", "base_link"),
        ("base_link", "livox_frame"),
    }
    assert by_edge[("body", "base_footprint")]["translation"] == (
        0.452466, -0.183290, -1.396285
    )
    assert by_edge[("body", "base_footprint")]["rpy_rad"] == (
        0.0, -math.radians(28.5), 0.0
    )
    assert by_edge[("base_footprint", "base_link")]["translation"] == (0.0, 0.0, 0.125)
    assert by_edge[("base_footprint", "base_link")]["rpy_rad"] == (0.0, 0.0, 0.0)
    assert by_edge[("base_link", "livox_frame")]["translation"] == (0.280, 0.160, 1.362)
    assert by_edge[("base_link", "livox_frame")]["rpy_rad"] == (
        0.0, math.radians(28.5), 0.0
    )
    assert len(transforms) == 3


def test_composed_rigid_chain_matches_internal_lidar_imu_geometry():
    calibration = load_yaml(CALIBRATION)
    inverse = calibration["transforms"]["body_to_base_footprint"]["derived_inverse_for_audit"]
    expected = inverse["translation_m"]
    # base_footprint -> livox minus the body->livox internal offset rotated by
    # the physical mounting pitch yields base_footprint -> body.
    pitch = math.radians(28.5)
    rotation = (
        (math.cos(pitch), 0.0, math.sin(pitch)),
        (0.0, 1.0, 0.0),
        (-math.sin(pitch), 0.0, math.cos(pitch)),
    )
    internal = (-0.011, -0.02329, 0.04412)
    rotated = [sum(rotation[i][j] * internal[j] for j in range(3)) for i in range(3)]
    actual = [0.280 - rotated[0], 0.160 - rotated[1], 1.487 - rotated[2]]
    assert all(math.isclose(a, e, abs_tol=1.0e-6) for a, e in zip(actual, expected))


def test_mounting_pitch_is_applied_once_to_active_sensor_configuration():
    import json

    driver = json.loads(LIVOX_CONFIG.read_text())
    driver_ext = driver["lidar_configs"][0]["extrinsic_parameter"]
    assert driver_ext == {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0}
    fast_lio = load_yaml(FAST_LIO)["/**"]["ros__parameters"]["mapping"]
    assert fast_lio["extrinsic_T"] == [-0.011, -0.02329, 0.04412]
    assert fast_lio["extrinsic_R"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    transforms = literal_assignment(ROBOT_LAUNCH, "CALIBRATED_STATIC_TRANSFORMS")
    mount = next(item for item in transforms if item["child"] == "livox_frame")
    assert mount["rpy_rad"] == (0.0, math.radians(28.5), 0.0)


def test_livox_source_applies_configured_rotation_to_points_and_imu_so_zero_is_native():
    source = LIVOX_PUBLISHER.read_text()
    assert "imu_data.gyro_x = (imu->gyro_x * extrinsic_.rotation[0][0]" in source
    assert "imu_data.acc_x = (imu->acc_x * extrinsic_.rotation[0][0]" in source
    assert "raw[i].x * extrinsic_.rotation[0][0]" in source
    assert "packet.extrinsic_enable = false" in source


def test_selected_launch_has_no_fake_dynamic_tf_or_optional_stale_rtab_packages():
    text = ROBOT_LAUNCH.read_text() + RTAB_BRIDGE.read_text() + PHASE5_LAUNCH.read_text()
    assert "map_to_odom_static" not in text
    assert "odom_to_body" not in text
    assert "odom_to_base_footprint" not in text
    for package in ("rtabmap_odom", "rtabmap_rviz_plugins", "rtabmap_costmap_plugins"):
        assert package not in text
    assert '"rtabmap_viz": "false"' in text
    assert '"localization": "true"' in text
    assert "'publish_tf_map': 'true'" in text
    assert "'publish_tf_odom': 'false'" in text


def test_measured_nav2_footprint_covers_all_physical_extents():
    nav = load_yaml(NAV)
    for section in ("local_costmap", "global_costmap"):
        params = nav[section][section]["ros__parameters"]
        points = yaml.safe_load(params["footprint"])
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        assert max(xs) >= 0.850 and min(xs) <= -0.220
        assert max(ys) >= 0.330 and min(ys) <= -0.330
        assert params["footprint_padding"] == 0.03
        assert "robot_radius" not in params


def test_collision_monitor_humble_pointcloud_schema_and_safe_chain():
    params = load_yaml(COLLISION)["collision_monitor"]["ros__parameters"]
    assert params["cmd_vel_in_topic"] == "/cmd_vel_nav_raw"
    assert params["cmd_vel_out_topic"] == "/cmd_vel_nav_safe"
    assert params["source_timeout"] == 0.5
    assert params["transform_tolerance"] == 0.5
    source = params["mid360_body_cloud"]
    assert params["observation_sources"] == ["mid360_body_cloud"]
    assert source["type"] == "pointcloud"
    assert source["topic"] == "/cloud_registered_body"
    assert source["min_height"] == 0.10 and source["max_height"] == 1.60
    installed = Path("/opt/ros/humble/share/nav2_collision_monitor/params/collision_monitor_params.yaml")
    header = Path("/opt/ros/humble/include/nav2_collision_monitor/pointcloud.hpp")
    assert installed.is_file() and header.is_file()
    installed_params = load_yaml(installed)["collision_monitor"]["ros__parameters"]
    assert installed_params["pointcloud"]["type"] == "pointcloud"
    assert {"topic", "min_height", "max_height", "enabled"} <= set(installed_params["pointcloud"])
    assert "sensor_msgs::msg::PointCloud2" in header.read_text()


def test_stop_and_slow_zones_are_external_to_measured_rigid_envelope():
    params = load_yaml(COLLISION)["collision_monitor"]["ros__parameters"]
    body = (-0.220, 0.850, -0.330, 0.330)
    stop = polygon_extents(params["Phase5Stop"]["points"])
    slow = polygon_extents(params["Phase5Slow"]["points"])
    assert stop[0] < body[0] and stop[1] > body[1] and stop[2] < body[2] and stop[3] > body[3]
    assert slow[0] < stop[0] and slow[1] > stop[1] and slow[2] < stop[2] and slow[3] > stop[3]
    assert params["Phase5Stop"]["action_type"] == "stop"
    assert params["Phase5Slow"]["action_type"] == "slowdown"


def test_localization_and_collision_validity_are_each_integrated_once():
    launch = PHASE5_LAUNCH.read_text()
    assert launch.count('executable="localization_validity_monitor"') == 1
    assert launch.count('executable="collision_monitor_validity_monitor"') == 1
    assert "/Odometry" in (SRC / "vehicle_cmd_safety" / "config" / "phase5_localization_validity.yaml").read_text()
    validity = load_yaml(COLLISION_VALIDITY)["collision_monitor_validity_monitor"]["ros__parameters"]
    assert validity["source_topic"] == "/cloud_registered_body"
    assert validity["expected_frame"] == "body"
    assert validity["expected_observation_source_name"] == "mid360_body_cloud"


def test_selected_composition_is_mock_only_no_can_and_gate_starts_disarmed():
    launch = ROBOT_LAUNCH.read_text() + RTAB_BRIDGE.read_text() + PHASE5_LAUNCH.read_text()
    adapter = load_yaml(ADAPTER_CONFIG)["mock_wheelchair_cmd_adapter"]["ros__parameters"]
    assert launch.count('executable="mock_wheelchair_cmd_adapter"') == 1
    assert adapter["input_topic"] == "/vehicle_cmd_safe"
    assert adapter["output_topic"] == "/wheelchair_control_command_mock"
    for forbidden in (
        "wheelchair_controller", "SocketCAN", "socketcan", "can0",
        "phase4_vehicle_cmd_fake_base", '"/wheelchair_control_command"',
    ):
        assert forbidden not in launch
    gate = GATE_SOURCE.read_text()
    assert "self.state = STATE_DISARMED" in gate
    assert "/vehicle_cmd_safety/arm" not in launch


def test_selected_package_declares_all_runtime_owners():
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert {
        "nav2_collision_monitor", "robot_bringup", "vehicle_cmd_safety",
        "wheelchair_cmd_adapter",
    } <= dependencies


def test_launch_files_are_python_syntax_valid():
    compile(ROBOT_LAUNCH.read_text(), str(ROBOT_LAUNCH), "exec")
    compile(PHASE5_LAUNCH.read_text(), str(PHASE5_LAUNCH), "exec")
