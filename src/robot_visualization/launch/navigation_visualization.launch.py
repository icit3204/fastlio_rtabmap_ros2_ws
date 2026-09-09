"""Launch observation-only RViz navigation visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_visualization'), 'config', 'canonical_navigation.rviz']),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_fixture', default_value='false'),
        Node(
            package='robot_visualization',
            executable='navigation_visualization_helper',
            name='navigation_visualization_helper',
            output='screen',
        ),
        Node(
            package='robot_visualization',
            executable='visualization_fixture',
            name='visualization_fixture',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_fixture')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_canonical_navigation',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
