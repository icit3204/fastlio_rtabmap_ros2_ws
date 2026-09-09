"""Launch the standalone Operator GUI only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gate_topic = LaunchConfiguration('gate_state_topic')
    return LaunchDescription([
        DeclareLaunchArgument('gate_state_topic', default_value='/vehicle_cmd_safety/state'),
        Node(
            package='operator_gui', executable='operator_gui', name='operator_gui', output='screen',
            parameters=[{'gate_state_topic': gate_topic}],
        ),
    ])
