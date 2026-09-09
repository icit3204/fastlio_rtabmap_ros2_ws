"""Compatibility-name tests for the superseding MK2C1R3 frame authority."""

import importlib.util
import math
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE / "config" / "mkmini_platform_profile.yaml"
TEMPLATE = PACKAGE / "urdf" / "mkmini_frames.urdf.template"
LAUNCH = PACKAGE.parent / "robot_bringup" / "launch" / "mkmini_calibrated_localization.launch.py"


def load_launch():
    spec = importlib.util.spec_from_file_location("mkmini_calibrated_localization", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_current_authority_supersedes_only_old_yaw():
    data = yaml.safe_load(PROFILE.read_text())
    native = data["livox_native_transform"]
    assert data["status"] == "MKMINI_MID360_CURRENT_YAW_AUTHORITY_MK2C1R3"
    assert native["roll_deg"]["value"] == 1.1096728679
    assert native["pitch_deg"]["value"] == 31.3946885517
    assert native["yaw_deg"]["value"] == 1.3275
    assert native["history"]["retired_current_hardware_yaw_deg"] == -112.6401945888
    assert native["history"]["retired_current_hardware_yaw_status"] == \
        "RETIRED_FOR_CURRENT_HARDWARE_FRAME_CONTRACT"


def test_profile_is_only_numeric_mid360_mount_authority():
    template = TEMPLATE.read_text()
    launch = LAUNCH.read_text()
    assert "@LIVOX_XYZ@" in template
    assert "@LIVOX_RPY_RAD@" in template
    assert "-1.965942" not in template + launch
    assert "-112.640" not in template + launch
    assert "1.3275" not in template + launch
    assert "mkmini_platform_profile.yaml" in launch
    assert not (PACKAGE / "urdf" / "mkmini_frames.urdf").exists()
    assert not (PACKAGE / "urdf" / "mkmini_frames.urdf.xacro").exists()


def test_dependent_transforms_are_derived_from_profile():
    module = load_launch()
    authority = module.load_current_frame_authority(PACKAGE)
    body_base = authority["body_to_base_footprint"]
    expected_t = (0.297163177464, -0.054016303975, -0.774031175446)
    expected_rpy = tuple(map(math.radians, (-0.489726635719, -31.4088642393,
                                             -0.877933945254)))
    assert max(abs(a - b) for a, b in zip(body_base["translation"], expected_t)) < 1e-9
    assert max(abs(a - b) for a, b in zip(body_base["rpy_rad"], expected_rpy)) < 1e-9
    assert "@" not in authority["robot_description"]
    assert 'name="base_link_to_livox_frame"' in authority["robot_description"]


def test_current_basis_and_recorded_front_targets_are_not_mirrored():
    module = load_launch()
    authority = module.load_current_frame_authority(PACKAGE)
    base_body = authority["base_footprint_to_body"]
    rotation = base_body["rotation"]
    translation = base_body["translation"]

    # Child body basis columns expressed in project base coordinates.
    assert rotation[0][0] > 0.85       # body +X has forward projection
    assert rotation[1][1] > 0.99       # body +Y has left projection
    assert rotation[2][2] > 0.85       # body +Z has up projection
    determinant = (
        rotation[0][0] * (rotation[1][1]*rotation[2][2]-rotation[1][2]*rotation[2][1])
        - rotation[0][1] * (rotation[1][0]*rotation[2][2]-rotation[1][2]*rotation[2][0])
        + rotation[0][2] * (rotation[1][0]*rotation[2][1]-rotation[1][1]*rotation[2][0]))
    assert math.isclose(determinant, 1.0, abs_tol=1e-9)

    raw_centroids = [
        (1.187470, -0.072049, 0.291851),
        (1.237937, -0.047495, 0.175321),
        (1.499054, -0.079221, 0.308524),
        (1.677351, -0.090763, 0.417767),
        (1.873508, -0.058574, 0.531295),
    ]
    projected = [tuple(sum(rotation[i][j]*point[j] for j in range(3)) + translation[i]
                       for i in range(3)) for point in raw_centroids]
    assert all(point[0] > 1.0 and abs(point[1]) < 0.03 and point[2] > 0.0
               for point in projected)
    # The four 430-mm scenes increase with reported physical clearance.
    assert all(projected[i][0] < projected[i+1][0] for i in range(1, 4))


def test_launch_has_one_body_and_one_chassis_odom_publisher_and_no_motion_stack():
    text = LAUNCH.read_text()
    assert text.count('"mkmini_body_to_base_footprint"') == 1
    assert text.count('"mkmini_odom_to_odom_chassis"') == 1
    assert 'package="nav2' not in text
    assert 'package="collision_monitor"' not in text
    assert '"can0"' not in text
