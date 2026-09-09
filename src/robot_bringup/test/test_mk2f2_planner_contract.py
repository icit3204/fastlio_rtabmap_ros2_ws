from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "nav2_common.yaml"
LAUNCH = ROOT / "launch" / "bringup.launch.py"
RUNNER = ROOT / "scripts" / "mk2f2_planner_stationary_runner.py"


def test_global_planner_has_the_same_forward_ackermann_contract_as_mppi():
    config = yaml.safe_load(CONFIG.read_text())
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    assert global_costmap["global_frame"] == "map"
    assert global_costmap["robot_base_frame"] == "base_footprint"
    assert global_costmap["static_layer"]["map_topic"] == "/map"
    assert global_costmap["footprint"] == config["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"
    assert planner["motion_model_for_search"] == "DUBIN"
    assert planner["minimum_turning_radius"] == 1.75
    assert planner["allow_unknown"] is False
    assert config["controller_server"]["ros__parameters"]["FollowPath"]["vx_min"] == 0.0


def test_stationary_planner_launch_owns_real_planner_and_keeps_mock_lock():
    source = LAUNCH.read_text()
    assert "DeclareLaunchArgument('stationary_planner_mock_gate'" in source
    section = source.split("def stationary_planner_actions", 1)[1].split("def stationary_bt_navigator_actions", 1)[0]
    assert "package='nav2_planner', executable='planner_server'" in section
    assert "'node_names': ['planner_server']" in section
    assert "start_rtabmap" in source
    assert "output_transport': 'mock'" in source
    for forbidden in ("package='nav2_bt_navigator'", "velocity_smoother", "pure_pursuit"):
        assert forbidden not in section


def test_runner_uses_real_actions_and_analyzes_the_returned_path():
    source = RUNNER.read_text()
    assert "ActionClient(self, ComputePathToPose, \"/compute_path_to_pose\")" in source
    assert "ActionClient(self, FollowPath, \"/follow_path\")" in source
    assert "path = self.compute(start, goal)" in source
    assert "request.path = path" in source
    assert "analyze_path(path)" in source
    for forbidden in ("SocketCAN", "can0", "NavigateToPose", "create_publisher(Twist, \"/cmd_vel_nav\""):
        assert forbidden not in source
