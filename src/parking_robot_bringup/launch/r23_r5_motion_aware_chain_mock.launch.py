"""R23-R5 opt-in collision selector through Gate to MockTransport only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


VALID_MODES = ("fixed_qualified", "motion_aware_experimental")


def generate_launch_description():
    mode = LaunchConfiguration("collision_monitor_mode")
    robot = FindPackageShare("robot_bringup")
    bringup = FindPackageShare("parking_robot_bringup")
    safety = FindPackageShare("vehicle_cmd_safety")
    bridge = FindPackageShare("wheelchair_cmd_adapter")
    controller = FindPackageShare("wheelchair_controller")

    def validate_mode(context):
        selected = mode.perform(context)
        if selected not in VALID_MODES:
            raise RuntimeError(
                "collision_monitor_mode must be fixed_qualified or "
                f"motion_aware_experimental, got {selected!r}")
        return []

    fixed = Node(
        package="nav2_collision_monitor", executable="collision_monitor",
        name="collision_monitor", output="screen",
        parameters=[PathJoinSubstitution([
            robot, "config", "collision_monitor_dual_sensor.yaml"])],
        remappings=[
            ("/cmd_vel_nav", "/r23_r5/nav_cmd"),
            ("/cmd_vel", "/r23_r5/collision_selected"),
            ("/cloud_registered_nav2_obstacles", "/r23_r5/mid_obstacles"),
            ("/scan", "/r23_r5/scan")],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'fixed_qualified'"])))
    fixed_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="r23_r5_fixed_collision_manager", output="screen",
        parameters=[{"autostart": True, "bond_timeout": 0.0,
                     "node_names": ["collision_monitor"]}],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'fixed_qualified'"])))

    experimental_mask = Node(
        package="robot_bringup", executable="tmini_collision_self_mask",
        name="tmini_collision_self_mask", output="screen",
        parameters=[PathJoinSubstitution([
            robot, "config", "tmini_collision_self_mask.experimental.yaml"]), {
                "input_topic": "/r23_r5/scan",
                "output_topic": "/r23_r5/scan_experimental"}],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'motion_aware_experimental'"])))
    experimental = Node(
        package="robot_bringup", executable="motion_aware_collision_mock",
        name="motion_aware_collision_mock", output="screen",
        parameters=[PathJoinSubstitution([
            robot, "config", "motion_aware_collision_mock.experimental.yaml"]), {
                "input_cmd_topic": "/r23_r5/nav_cmd",
                "mid_topic": "/r23_r5/mid_obstacles",
                "tmini_topic": "/r23_r5/scan_experimental",
                "output_topic": "/r23_r5/collision_selected",
                "state_topic": "/r23_r5/experimental_state",
                "integration_mock_enabled": True}],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'motion_aware_experimental'"])))

    common = [
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="r23_r5_odom_base_tf", output="screen",
             arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="r23_r5_base_laser_tf", output="screen",
             arguments=["0.703", "0", "0.1923", "0", "0", "0",
                        "base_footprint", "laser_frame"]),
        Node(package="parking_robot_bringup", executable="selected_collision_status",
             name="selected_collision_status", output="screen", parameters=[{
                 "active_mode": mode,
                 "input_topic": "/r23_r5/nav_cmd",
                 "selected_output_topic": "/r23_r5/collision_selected",
                 "experimental_state_topic": "/r23_r5/experimental_state",
                 "status_topic": "/r23_r5/collision_status"}]),
        Node(package="vehicle_cmd_safety", executable="phase4_p4c_permission_fixture",
             name="r23_r5_permission_fixture", output="screen", parameters=[{
                 "publish_localization": True, "publish_controller": True,
                 "publish_collision": True, "localization_valid": True,
                 "controller_valid": True, "collision_valid": True,
                 "publish_rate_hz": 20.0}]),
        Node(package="vehicle_cmd_safety", executable="guarded_vehicle_cmd_gate",
             name="guarded_vehicle_cmd_gate", output="screen",
             parameters=[PathJoinSubstitution([
                 bringup, "config", "r23_r5_gate_mock.yaml"])]),
        Node(package="wheelchair_cmd_adapter", executable="gate_to_labmate_bridge",
             name="gate_to_labmate_bridge", output="screen",
             parameters=[PathJoinSubstitution([
                 bridge, "config", "gate_to_labmate_bridge.yaml"])]),
        Node(package="wheelchair_controller", executable="wheelchair_controller_node",
             name="wheelchair_controller_node", output="screen",
             parameters=[PathJoinSubstitution([
                 controller, "config", "wheelchair_controller_param.yaml"]), {
                     "output_transport": "mock", "auto_start": True,
                     "can_send_period_ms": 20.0,
                     "wheelbase_mm": 600.0, "track_width_mm": 518.0,
                     "max_steer_angle_deg": 30.0}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "collision_monitor_mode", default_value="fixed_qualified"),
        OpaqueFunction(function=validate_mode),
        fixed, fixed_manager, experimental_mask, experimental, *common,
    ])
