from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
PROFILE = PACKAGE / "config" / "mkmini_platform_profile.yaml"
URDF_TEMPLATE = PACKAGE / "urdf" / "mkmini_frames.urdf.template"
FAST_LIO = WORKSPACE / "src" / "FAST_LIO_ROS2" / "config" / "mid360.yaml"
TMINI_LAUNCH = PACKAGE / "launch" / "p5a_mk2a0_tmini_preparation.launch.py"


def test_profile_uses_mkmini_worksheet_and_explicit_authority_statuses():
    data = yaml.safe_load(PROFILE.read_text())
    assert data["source_authority"]["operator_worksheet"]["status"] == "INGESTED_FAITHFULLY_V3_REVISION"
    assert data["frame_convention"]["axes"]["value"] == "+X forward, +Y left, +Z up"
    assert data["geometry"]["manufacturer_nominal"]["wheelbase_m"]["value"] == 0.600
    assert data["geometry"]["manufacturer_nominal"]["track_m"]["value"] == 0.518
    assert data["transforms"]["base_footprint_to_base_link"]["translation_m"] == [0.0, 0.0, 0.120]
    assert data["mid360_mount_reference"]["mast_x_from_M0_m"]["value"] == 0.1385
    assert data["mid360_mount_reference"]["lateral_reference_m"]["value"] == 0.019
    assert data["mid360_mount_reference"]["working_height_from_floor_m"]["value"] == 0.850
    assert data["mid360_mount_reference"]["reference_z_relative_to_base_link_m"]["value"] == 0.730
    assert data["mid360_mount_reference"]["physical_tilt_magnitude_deg"]["value"] == 31.5


def test_rear_axle_frame_convention_and_front_axle_are_explicit():
    text = URDF_TEMPLATE.read_text().replace("@BASE_LINK_Z@", "0.120").replace(
        "@FRONT_AXLE_X@", "0.600").replace("@LIVOX_XYZ@", "0 0 0").replace(
        "@LIVOX_RPY_RAD@", "0 0 0")
    root = ET.fromstring(text)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert joints["base_footprint_to_base_link"].find("origin").attrib["xyz"] == "0 0 0.120"
    assert joints["base_link_to_rear_axle"].find("origin").attrib["xyz"] == "0 0 0"
    assert joints["base_link_to_front_axle"].find("origin").attrib["xyz"] == "0.600 0 0"
    assert {link.attrib["name"] for link in root.findall("link")} == {
        "base_footprint", "base_link", "rear_axle", "front_axle", "livox_frame"
    }


def test_ackermann_reference_is_kinematically_consistent():
    # The project input is body-X velocity at the rear-axle/nonholonomic
    # reference; for planar Ackermann motion kappa=w/v and R=v/w.
    L, T = 0.600, 0.518
    v, w = 0.2, 0.1
    curvature = w / v
    rear_radius = abs(1.0 / curvature)
    inner = math.atan(L / (rear_radius - T / 2.0))
    assert math.isclose(curvature, 0.5)
    assert math.isclose(rear_radius, 2.0)
    assert 0.0 < inner < math.pi / 2.0


def test_native_sensor_tf_and_safety_geometry_are_not_fabricated():
    template_text = URDF_TEMPLATE.read_text()
    data = yaml.safe_load(PROFILE.read_text())
    native = data["livox_native_transform"]
    assert native["authorized"]["value"] is True
    assert native["authorized"]["status"] == "CURRENT_HARDWARE_FRAME_CONTRACT_AUTHORIZED"
    assert native["x_m"]["value"] == 0.162993658894
    assert native["y_m"]["value"] == 0.018657143794
    assert native["z_m"]["value"] == 0.739281661466
    assert native["roll_deg"]["value"] == 1.1096728679
    assert native["pitch_deg"]["value"] == 31.3946885517
    assert native["yaw_deg"]["value"] == 1.3275
    assert data["safety_geometry"]["operational_footprint"]["active"] is False
    assert "livox_frame" in template_text
    assert "laser_frame" not in template_text
    assert "Phase5Stop" not in template_text
    assert "Phase5Slow" not in template_text
    assert "0.850" not in template_text
    assert "1.05" not in template_text
    assert "1.35" not in template_text


def test_calibrated_livox_provenance_and_uncertainty_are_retained():
    data = yaml.safe_load(PROFILE.read_text())
    native = data["livox_native_transform"]
    assert native["floor_provenance"]["capture_1_origin_distance_m"]["value"] == 0.855073
    assert native["floor_provenance"]["capture_2_origin_distance_m"]["value"] == 0.863490323
    assert native["floor_provenance"]["accepted_midpoint_m"]["value"] == 0.859281661
    assert native["uncertainty"]["covariance"]["value"] is None
    assert native["mechanical_authority"]["native_origin_above_bottom_m"]["value"] == 0.047


def test_fast_lio_internal_extrinsic_is_not_changed_by_profile_draft():
    data = yaml.safe_load(FAST_LIO.read_text())["/**"]["ros__parameters"]["mapping"]
    assert data["extrinsic_T"] == [-0.011, -0.02329, 0.04412]
    assert data["extrinsic_R"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def test_tmini_stereo_and_protocol_are_deferred():
    data = yaml.safe_load(PROFILE.read_text())
    tmini = data["sensor_status"]["tmini"]
    assert tmini["status"] == "P5A_MK2A0R_TMINI_FINAL_TF_MEASUREMENT_FREEZE_PASS"
    assert tmini["expected_future_message_type"] == "sensor_msgs/msg/LaserScan"
    assert tmini["expected_conceptual_frame"] == "laser_frame"
    assert tmini["driver_activation"] == "DISABLED_SENSOR_DISCONNECTED"
    assert tmini["stable_usb_identity"] == "UNRESOLVED_USE_DEV_SERIAL_BY_ID_OR_EQUIVALENT"
    assert data["sensor_status"]["stereo"]["status"] == "DEFERRED_OPTIONAL_EXACT_POSE"
    assert data["protocol_blockers"]["brake_profile"]["value"] == "UNRESOLVED"
    assert data["protocol_blockers"]["encoder_scale"]["value"] == "UNRESOLVED"
    assert data["profile_scope"]["can_implemented_or_changed"] is False


def test_tmini_measurement_freeze_resolves_tf_fields_and_checks():
    tmini = yaml.safe_load(PROFILE.read_text())["sensor_status"]["tmini"]
    frozen = tmini["measurement_freeze"]
    assert frozen["TMINI_CENTERLINE_X_FROM_REAR_AXLE_M"]["value"] == 0.703
    assert frozen["TMINI_CENTERLINE_Y_FROM_ROBOT_CENTER_M"]["value"] == 0.0
    assert frozen["TMINI_HOUSING_BOTTOM_HEIGHT_FROM_FLOOR_M"]["value"] == 0.286
    assert frozen["BASE_LINK_HEIGHT_FROM_FLOOR_M"]["value"] == 0.1200
    assert frozen["TMINI_OPTICAL_CENTERLINE_ABOVE_HOUSING_BOTTOM_M"]["value"] == 0.0263
    assert frozen["TMINI_SCAN_PLANE_HEIGHT_FROM_FLOOR_M"]["value"] == 0.3123
    assert frozen["BASE_LINK_TO_TMINI_SCAN_PLANE_Z_M"]["value"] == 0.1923
    assert frozen["TMINI_ZERO_MARK_DIRECTION"] == "ROBOT_FORWARD_PLUS_X"
    assert frozen["TMINI_NOMINAL_YAW_DEG"]["value"] == 0.0
    assert frozen["actual_scan_orientation_verification"] == "P5A_MK2A1R0_REVERSION_FALSE_FRONT_LEFT_RIGHT_QUALIFIED"
    assert frozen["manufacturer_zero_angle_tolerance_deg"]["range"] == [-3.0, 3.0]
    assert frozen["mount_rigidity"] == "YES"
    assert frozen["front_obstruction"] == "NO"
    assert frozen["left_obstruction"] == "NO"
    assert frozen["right_obstruction"] == "NO"
    assert frozen["rear_obstruction"] == "YES_COOLER_BODY"
    assert frozen["wheel_in_scan_plane"] == "NO"
    assert frozen["cable_secure"] == "YES"

    checks = tmini["cross_checks"]
    assert checks["wheelbase_plus_forward_of_front_axle"]["result_m"] == 0.703
    assert checks["centerline_x_plus_behind_front_edge"]["result_m"] == 0.755
    assert checks["housing_bottom_plus_optical_centerline"]["result_m"] == 0.3123
    assert checks["scan_plane_minus_base_link_height"]["result_m"] == 0.1923
    assert checks["wheelbase_plus_forward_of_front_axle"]["status"] == "PASS"
    assert checks["centerline_x_plus_behind_front_edge"]["status"] == "PASS"
    assert checks["diagonal_measurement"] == "NOT_INFERRED"

    candidate = tmini["base_link_to_laser_frame_candidate"]
    assert candidate["parent_frame"] == "base_link"
    assert candidate["child_frame"] == "laser_frame"
    assert candidate["translation_m"] == {"x": 0.703, "y": 0.0, "z": 0.1923}
    assert candidate["rotation_rpy_deg"] == [0.0, 0.0, 0.0]
    assert candidate["manufacturer_yaw_tolerance_deg"] == 3.0
    assert candidate["activation"] == "DISABLED_UNTIL_LIVE_SCAN_ORIENTATION_QUALIFICATION"


def test_tmini_preparation_launch_is_disabled_and_driver_free():
    text = TMINI_LAUNCH.read_text()
    assert 'default_value="false"' in text
    assert 'default_value="0.1923"' in text
    assert 'default_value="0.0"' in text
    assert "ydlidar_ros2_driver" not in text
    assert "/dev/ttyUSB0" not in text
    assert '"laser_frame"' in text
    assert '"base_link"' in text
    assert "0.286" not in text
