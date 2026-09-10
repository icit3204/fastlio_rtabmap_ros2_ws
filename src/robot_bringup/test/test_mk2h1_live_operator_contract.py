from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'launch' / 'bringup.launch.py'
H1_LAUNCH = ROOT / 'launch' / 'mk2h1_live_operator_mock.launch.py'


def test_live_profile_uses_real_localization_and_preserves_mock_controller_boundary():
    source = BRINGUP.read_text()
    assert "DeclareLaunchArgument(\n            'stationary_real_localization_validity'" in source
    assert "executable='localization_validity_monitor'" in source
    assert "'phase5_localization_validity.yaml'" in source
    assert "'publish_localization': not real_localization" in source
    assert "'publish_controller': True" in source
    assert "'output_transport': 'mock'" in source
    assert "'can_interface': 'CANONICAL_MOCK_FORBIDDEN'" in source


def test_h1_launch_has_one_real_stack_and_no_demo_fixture():
    source = H1_LAUNCH.read_text()
    assert "'stationary_mission_manager_mock_gate': 'true'" in source
    assert "'stationary_real_localization_validity': 'true'" in source
    assert ("'mission_manager_expected_topology_version':\n"
            "                'sha256:f8bd2b688ce827446ea88f42f19777f97c918c2dc1832091fa9b379a85000d70'") in source
    assert "'start_livox': 'true'" in source
    assert "'start_ydlidar': 'true'" in source
    assert "'use_fast_lio': 'true'" in source
    assert "'start_rtabmap': 'true'" in source
    assert "'rtabmap_use_working_copy': 'true'" in source
    assert "'use_fixture': 'false'" in source
    assert "integrated_demo_fixture" not in source
    assert "operator_gui.launch.py" in source
    assert "navigation_visualization.launch.py" in source


def test_h1_profile_has_no_physical_or_direct_motion_authority():
    source = H1_LAUNCH.read_text()
    forbidden = (
        'SocketCAN', 'output_transport', 'cmd_vel', 'wheelchair_control_command',
        'NavigateToPose', 'integrated_demo_fixture', 'operator_gui_mock_fixture',
    )
    for token in forbidden:
        assert token not in source


def test_live_gui_consumes_canonical_gate_diagnostics():
    source = H1_LAUNCH.read_text()
    assert "'gate_state_topic': '/phase5/gate_test/state'" in source


def test_mission_manager_expected_topology_is_configurable_without_changing_default():
    source = BRINGUP.read_text()
    assert ("'mission_manager_expected_topology_version', default_value='v1'"
            in source)
    assert "'expected_topology_version': mission_manager_expected_topology_version" in source
