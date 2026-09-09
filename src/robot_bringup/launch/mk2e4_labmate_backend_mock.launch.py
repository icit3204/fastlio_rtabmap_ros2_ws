"""Mock-only MK-mini lower-backend overlay; this launch can never select SocketCAN."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("wheelchair_cmd_adapter"), "config", "gate_to_labmate_bridge.yaml"]
    )
    backend_config = PathJoinSubstitution(
        [FindPackageShare("wheelchair_controller"), "config", "wheelchair_controller_param.yaml"]
    )

    bridge = Node(
        package="wheelchair_cmd_adapter",
        executable="gate_to_labmate_bridge",
        name="gate_to_labmate_bridge",
        output="screen",
        parameters=[bridge_config],
    )
    backend = Node(
        package="wheelchair_controller",
        executable="wheelchair_controller_node",
        name="wheelchair_controller_node",
        output="screen",
        parameters=[
            backend_config,
            {
                "output_transport": "mock",
                "auto_start": True,
                "can_interface": "MK2E4_MOCK_FORBIDDEN",
            },
        ],
    )
    return LaunchDescription([bridge, backend])
