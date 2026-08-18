from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="parking_robot_bringup",
            executable="phase4_p4e6c_recovery_feedback_relay",
            name="phase4_p4e6c_recovery_feedback_relay",
            output="screen",
        ),
    ])
