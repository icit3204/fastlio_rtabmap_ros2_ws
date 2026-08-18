"""P4-E.3A fake-only full chain with Mission Manager as sole Nav2 client."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("parking_robot_bringup"), "launch", "phase4_p4e1b_clear_full_chain.launch.py"])))
    manager = Node(package="parking_robot_mission_manager", executable="mission_manager_node",
                   name="mission_manager", output="screen",
                   parameters=[{"expected_topology_version":"v1",
                                "cancel_response_timeout_sec":2.0,
                                "cancel_result_timeout_sec":5.0,
                                "use_sim_time":False}])
    return LaunchDescription([base, manager])
