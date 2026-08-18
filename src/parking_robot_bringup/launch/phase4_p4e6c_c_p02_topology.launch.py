"""C-P02 qualification feedback remap topology fragment.

This launch starts only the qualification relay and a Mission Manager
qualification instance.  It is not a mission campaign and creates no action
server/client.  A full future campaign composes the same remap with its
already-qualified Nav2 launch.
"""
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
        Node(
            package="parking_robot_mission_manager",
            executable="mission_manager_node",
            name="phase4_p4e6c_mission_manager_qualification",
            parameters=[{"use_sim_time": False}],
            remappings=[(
                "/navigate_to_pose/_action/feedback",
                "/phase4_qualification/p4e6c/navigate_to_pose_feedback",
            )],
            output="screen",
        ),
    ])
