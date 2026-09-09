from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "nav2_common.yaml"
LAUNCH = ROOT / "launch" / "bringup.launch.py"
RUNNER = ROOT / "scripts" / "mk2f1_mppi_stationary_runner.py"


def test_mppi_is_ackermann_and_within_canonical_lower_limits():
    config = yaml.safe_load(CONFIG.read_text())
    params = config["controller_server"]["ros__parameters"]
    plugin = params["FollowPath"]
    assert params["controller_frequency"] == 20.0
    assert params["odom_topic"] == "/Odometry"
    assert plugin["plugin"] == "nav2_mppi_controller::MPPIController"
    assert plugin["motion_model"] == "Ackermann"
    assert plugin["vx_min"] == 0.0
    assert plugin["vx_max"] == 0.25
    assert plugin["wz_max"] == 0.50
    assert plugin["AckermannConstraints"]["min_turning_r"] == 1.75
    assert plugin["ObstaclesCritic"]["consider_footprint"] is True
    assert config["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"] == "odom_chassis"
    assert config["local_costmap"]["local_costmap"]["ros__parameters"]["robot_base_frame"] == "base_footprint"


def test_stationary_mppi_launch_is_real_controller_with_production_topics():
    source = LAUNCH.read_text()
    assert "DeclareLaunchArgument('stationary_mppi_mock_gate'" in source
    function = source.split("def stationary_mppi_controller_actions", 1)[1]
    function = function.split("def stationary_planner_actions", 1)[0]
    assert "package='nav2_controller', executable='controller_server'" in function
    assert "remappings=[('/cmd_vel', '/cmd_vel_nav')]" in function
    assert "'node_names': ['controller_server']" in function
    assert "package='nav2_bt_navigator'" not in function
    assert "planner_server" not in function
    assert "velocity_smoother" not in function
    assert "pure_pursuit" not in function
    assert "output_transport': 'mock'" in source
    assert "CANONICAL_MOCK_FORBIDDEN" in source
    assert "params['cmd_vel_in_topic'] = '/cmd_vel_nav'" in source
    assert "params['cmd_vel_out_topic'] = '/cmd_vel'" in source
    assert "'safe_input_topic': '/cmd_vel'" in source
    assert "'output_topic': '/vehicle_cmd_safe'" in source


def test_follow_path_runner_is_direct_and_has_no_motion_transport():
    source = RUNNER.read_text()
    assert 'ActionClient(self, FollowPath, "/follow_path")' in source
    assert 'message.header.frame_id = "odom_chassis"' in source
    assert '"odom_chassis", "base_footprint"' in source
    assert 'self.create_subscription(Twist, "/cmd_vel_nav"' in source
    assert 'self.create_subscription(Twist, "/cmd_vel"' in source
    assert 'self.create_subscription(TwistStamped, "/vehicle_cmd_safe"' in source
    assert '"/phase5/mk2e4/mock_can_decoded"' in source
    for forbidden in ("SocketCAN", "can0", "keyboard", "NavigateToPose"):
        assert forbidden not in source


def test_reverse_is_intentionally_disabled_at_controller_layer():
    plugin = yaml.safe_load(CONFIG.read_text())["controller_server"]["ros__parameters"]["FollowPath"]
    assert plugin["vx_min"] == 0.0
    assert "PreferForwardCritic" not in plugin["critics"]
