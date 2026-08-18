"""Full C-P01 qualification composition, runner disabled by default."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    source = PathJoinSubstitution([
        FindPackageShare("parking_robot_bringup"),
        "launch", "phase4_p4e6b_health_matrix.launch.py",
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(source),
            launch_arguments={"enable_health_runner": "false", "case_id": "C-P01"}.items(),
        ),
        Node(package="parking_robot_bringup", executable="phase4_p4e6c_progress_observer",
             name="phase4_p4e6c_progress_observer", parameters=[{"case_id": "C-P01"}],
             output="screen")
    ])
