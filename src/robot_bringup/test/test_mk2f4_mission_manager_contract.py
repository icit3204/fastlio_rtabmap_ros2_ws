import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "bringup.launch.py"
MANAGER = ROOT.parent / "parking_robot_mission_manager" / "parking_robot_mission_manager" / "mission_manager_node.py"
MACHINE = ROOT.parent / "parking_robot_mission_manager" / "parking_robot_mission_manager" / "mission_state_machine.py"
RUNNER = ROOT / "scripts" / "mk2f4_mission_manager_stationary_runner.py"


def readiness_class():
    spec = importlib.util.spec_from_file_location("mk2f4_runner_contract", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CommandPathReadiness


def test_canonical_mission_mode_uses_real_manager_and_mock_lower_chain():
    source = LAUNCH.read_text()
    assert "DeclareLaunchArgument('stationary_mission_manager_mock_gate'" in source
    section = source.split("def stationary_mission_manager_actions", 1)[1].split("# ── 7.", 1)[0]
    assert "package='parking_robot_mission_manager'" in section
    assert "'navigate_to_pose_action': '/navigate_to_pose'" in section
    assert "'progress_tf_frame': 'odom_chassis'" in section
    lower = source.split("def stationary_command_chain_actions", 1)[1].split("def stationary_mppi_controller_actions", 1)[0]
    assert "'output_transport': 'mock'" in lower
    assert "CANONICAL_MOCK_FORBIDDEN" in lower


def test_manager_observer_binds_to_current_phase5_chain_without_velocity_output():
    source = MANAGER.read_text()
    for parameter in ("raw_command_topic", "safe_command_topic", "gate_state_topic",
                      "adapter_diagnostics_topic", "progress_tf_frame", "progress_base_frame"):
        assert f'declare_parameter("{parameter}"' in source
    assert "create_publisher(Twist" not in source
    assert "ActionClient(node, NavigateToPose, action_name)" in source


def test_pause_resume_cancel_and_failure_are_action_owned_and_bounded():
    source = MACHINE.read_text()
    assert "def request_pause" in source and "def resume" in source
    assert "def request_cancel" in source
    assert "cancel_response_timeout_sec" in source
    assert "cancel_result_timeout_sec" in source
    assert "self._dispatch_current_waypoint()" in source
    assert "GoalResultCode.SUCCEEDED" in source
    assert "MissionStateCode.FAILED" in source


def test_no_direct_motion_authority_or_incompatible_recovery_added():
    launch = LAUNCH.read_text()
    section = launch.split("def stationary_mission_manager_actions", 1)[1].split("# ── 7.", 1)[0]
    assert "cmd_vel" in section  # observer topics only
    assert "create_publisher" not in section
    assert "nav2_behaviors" not in section
    assert "SocketCAN" not in section


def test_one_mppi_sample_never_declares_command_path_ready():
    readiness = readiness_class()()
    readiness.start_epoch(1_000_000_000)
    readiness.observe_command('mppi', 1_010_000_000, .25, 0.0)
    readiness.observe_gate(1_010_000_000, {
        'localization_valid': 'true', 'controller_valid': 'true',
        'collision_monitor_valid': 'true'})
    assert not readiness.ready(1_010_000_000)


def test_readiness_requires_continuously_fresh_mppi_and_cm_plus_validity():
    readiness = readiness_class()()
    readiness.start_epoch(1_000_000_000)
    validity = {'localization_valid': 'true', 'controller_valid': 'true',
                'collision_monitor_valid': 'true'}
    for offset_ms in range(0, 351, 50):
        now = 1_000_000_000 + offset_ms * 1_000_000
        readiness.observe_command('mppi', now, .25, .02)
        readiness.observe_command('cm', now + 2_000_000, .25, .02)
        readiness.observe_gate(now + 3_000_000, validity)
    assert readiness.ready(1_355_000_000)


def test_cm_gap_resets_readiness_until_a_new_stable_tail_exists():
    readiness = readiness_class()()
    readiness.start_epoch(1_000_000_000)
    validity = {'localization_valid': 'true', 'controller_valid': 'true',
                'collision_monitor_valid': 'true'}
    for offset_ms in (0, 50, 100, 300):
        now = 1_000_000_000 + offset_ms * 1_000_000
        readiness.observe_command('mppi', now, .25, 0.0)
        readiness.observe_command('cm', now, .25, 0.0)
        readiness.observe_gate(now, validity)
    assert not readiness.ready(1_300_000_000)
    for offset_ms in range(350, 651, 50):
        now = 1_000_000_000 + offset_ms * 1_000_000
        readiness.observe_command('mppi', now, .25, 0.0)
        readiness.observe_command('cm', now, .25, 0.0)
        readiness.observe_gate(now, validity)
    assert readiness.ready(1_650_000_000)
