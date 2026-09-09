"""Launch the Operator GUI with the offline status fixture."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='operator_gui', executable='operator_gui_mock_fixture', name='operator_gui_mock_fixture', output='screen'),
        Node(package='operator_gui', executable='operator_gui', name='operator_gui', output='screen'),
    ])
