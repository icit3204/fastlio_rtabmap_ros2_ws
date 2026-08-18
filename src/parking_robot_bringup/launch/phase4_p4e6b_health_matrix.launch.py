"""Dedicated P4-E.6B composition; all qualification controls default transparent."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    case_id=LaunchConfiguration("case_id"); enable_runner=LaunchConfiguration("enable_health_runner")
    output_dir=LaunchConfiguration("health_output_dir")
    b=FindPackageShare("parking_robot_bringup"); s=FindPackageShare("vehicle_cmd_safety"); a=FindPackageShare("wheelchair_cmd_adapter")
    nav=PathJoinSubstitution([b,"config","phase4_p4e_chassis_nav2_params.yaml"]); collision=PathJoinSubstitution([b,"config","phase4_p4e_collision_monitor_scan.yaml"])
    validity=PathJoinSubstitution([s,"config","phase4_p4c_collision_validity_scan.yaml"]); gate=PathJoinSubstitution([b,"config","phase4_p4e_gate_mock.yaml"]); fake=PathJoinSubstitution([b,"config","phase4_p4e1a_fake_base.yaml"]); adapter=PathJoinSubstitution([a,"config","mock_wheelchair_cmd_adapter.yaml"])
    ntp=PathJoinSubstitution([b,"behavior_trees","phase4_p4e_chassis_navigate_to_pose.xml"]); ntps=PathJoinSubstitution([b,"behavior_trees","phase4_p4e_chassis_navigate_through_poses.xml"]); map_yaml=PathJoinSubstitution([b,"maps","phase2_clean_map.yaml"])
    nodes=[
      Node(package="nav2_map_server",executable="map_server",name="map_server",parameters=[nav,{"yaml_filename":map_yaml,"use_sim_time":False}]),
      Node(package="tf2_ros",executable="static_transform_publisher",name="phase2_map_to_odom_static_tf",arguments=["0","0","0","0","0","0","map","odom"]),
      Node(package="nav2_planner",executable="planner_server",name="planner_server",parameters=[nav,{"use_sim_time":False}]),
      Node(package="nav2_controller",executable="controller_server",name="controller_server",parameters=[nav,{"use_sim_time":False}],remappings=[("/cmd_vel","/cmd_vel_nav_raw")]),
      Node(package="nav2_behaviors",executable="behavior_server",name="behavior_server",parameters=[nav,{"use_sim_time":False}],remappings=[("/cmd_vel","/cmd_vel_nav_raw")]),
      Node(package="nav2_bt_navigator",executable="bt_navigator",name="bt_navigator",parameters=[nav,{"use_sim_time":False,"default_nav_to_pose_bt_xml":ntp,"default_nav_through_poses_bt_xml":ntps}]),
      Node(package="nav2_collision_monitor",executable="collision_monitor",name="collision_monitor",parameters=[collision,{"use_sim_time":False}]),
      Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_navigation",parameters=[{"use_sim_time":False,"autostart":True,"node_names":["map_server","planner_server","controller_server","behavior_server","bt_navigator"]}]),
      Node(package="phase4_lifecycle_manager_reconciled",executable="lifecycle_manager",name="lifecycle_manager_collision_monitor",parameters=[{"use_sim_time":False,"autostart":True,"node_names":["collision_monitor"],"service_call_timeout_sec":1.0,"transition_retry_interval_ms":100,"transition_retries":3}]),
      Node(package="parking_robot_bringup",executable="phase4_p4b_synthetic_obstacles",name="phase4_p4b_synthetic_obstacles",parameters=[{"mode":"CLEAR","frame_id":"base_footprint","publish_rate_hz":20.0,"publish_scan":True,"publish_pointcloud":False}]),
      Node(package="vehicle_cmd_safety",executable="collision_monitor_validity_monitor",name="collision_monitor_validity_monitor",parameters=[validity]),
      Node(package="vehicle_cmd_safety",executable="phase4_p4c_permission_fixture",name="phase4_p4e_permission_fixture",parameters=[{"publish_collision":False,"publish_localization":True,"publish_controller":True,"localization_valid":True,"controller_valid":True,"publish_rate_hz":20.0}]),
      Node(package="vehicle_cmd_safety",executable="guarded_vehicle_cmd_gate",name="guarded_vehicle_cmd_gate",parameters=[gate],remappings=[("/system/localization_valid","/phase4_qualification/p4e6b/gate_localization_valid")]),
      Node(package="wheelchair_cmd_adapter",executable="mock_wheelchair_cmd_adapter",name="mock_wheelchair_cmd_adapter",parameters=[adapter,{"qualification_health_control_enabled":True,"qualification_force_invalid":False}]),
      Node(package="parking_robot_bringup",executable="phase4_vehicle_cmd_fake_base",name="phase4_vehicle_cmd_fake_base",parameters=[fake]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_observation_relay",name="p4e6b_odom_relay",parameters=[{"kind":"odom","suppress":False}]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_observation_relay",name="p4e6b_safe_relay",parameters=[{"kind":"safe","suppress":False}]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_observation_relay",name="p4e6b_adapter_relay",parameters=[{"kind":"adapter","force_invalid":False}]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_observation_relay",name="p4e6b_gate_localization_relay",parameters=[{"kind":"gate_localization","force_invalid":False}]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_tf_observation_relay",name="p4e6b_tf_relay",parameters=[{"suppress_dynamic":False}]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_feedback_relay",name="p4e6b_feedback_relay",parameters=[{"suppress_feedback":False}]),
      Node(package="parking_robot_mission_manager",executable="mission_manager_node",name="mission_manager",parameters=[{"expected_topology_version":"v1","cancel_response_timeout_sec":2.0,"cancel_result_timeout_sec":5.0,"navigate_to_pose_action":"/navigate_to_pose","use_sim_time":False}],remappings=[("/navigate_to_pose/_action/feedback","/phase4_qualification/p4e6b/navigate_to_pose_feedback"),("/Odometry","/phase4_qualification/p4e6b/mm_odometry"),("/tf","/phase4_qualification/p4e6b/mm_tf"),("/cmd_vel_nav_safe","/phase4_qualification/p4e6b/mm_cmd_vel_nav_safe"),("/wheelchair_cmd_adapter/diagnostics","/phase4_qualification/p4e6b/mm_adapter_diagnostics")]),
      Node(package="parking_robot_bringup",executable="phase4_p4e6b_health_failure_runner",
           name="phase4_p4e6b_health_failure_runner",condition=IfCondition(enable_runner),
           arguments=["--case-id",case_id,"--output-dir",output_dir,"--execute-authorized-case"]),
    ]
    return LaunchDescription([
      DeclareLaunchArgument("enable_health_runner",default_value="false"),
      DeclareLaunchArgument("case_id",default_value=""),
      DeclareLaunchArgument("health_output_dir",default_value="/tmp/p4e6b_health_runtime"),
      *nodes])
