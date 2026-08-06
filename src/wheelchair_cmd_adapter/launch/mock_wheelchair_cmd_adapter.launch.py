from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("wheelchair_cmd_adapter"),
        "config",
        "mock_wheelchair_cmd_adapter.yaml",
    ])
    return LaunchDescription([
        Node(
            package="wheelchair_cmd_adapter",
            executable="mock_wheelchair_cmd_adapter",
            name="mock_wheelchair_cmd_adapter",
            output="screen",
            parameters=[config],
        )
    ])
