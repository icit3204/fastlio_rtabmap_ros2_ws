from pathlib import Path


def test_phase5_selects_exactly_one_mock_or_physical_backend():
    source = (Path(__file__).parents[1] / "launch" / "phase5_no_motion.launch.py").read_text()
    assert 'DeclareLaunchArgument("backend_mode", default_value="mock"' in source
    assert "' == 'mock'" in source
    assert "' == 'physical_mkmini'" in source
    assert "mkmini_physical_ros_backend" in source
    assert "mock_wheelchair_cmd_adapter" in source


def test_physical_profile_is_disabled_until_explicit_enable_and_r11_bounded():
    profile = (Path(__file__).parents[2] / "mkmini_cmd_adapter" / "config" / "r11_physical_backend_commissioning.yaml").read_text()
    assert "physical_enabled: false" in profile
    assert "command_speed_ceiling_mps: 0.040" in profile
    assert "actual_speed_ceiling_mps: 0.070" in profile
