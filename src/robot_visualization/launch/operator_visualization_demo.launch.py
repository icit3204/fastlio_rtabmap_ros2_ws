"""Launch one non-production fixture with the standalone GUI and canonical RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_visualization'), 'config', 'canonical_navigation.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('initial_state', default_value='RECEIVED'),
        Node(
            package='robot_visualization', executable='integrated_demo_fixture',
            name='operator_visualization_integrated_demo_fixture', output='screen',
            parameters=[{'initial_state': LaunchConfiguration('initial_state')}],
        ),
        Node(
            package='robot_visualization', executable='navigation_visualization_helper',
            name='navigation_visualization_helper', output='screen',
        ),
        Node(
            package='operator_gui', executable='operator_gui', name='operator_gui', output='screen',
            condition=IfCondition(LaunchConfiguration('use_gui')),
            parameters=[{'gate_state_topic': '/vehicle_cmd_safety/state'}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_canonical_navigation', output='screen',
            arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
